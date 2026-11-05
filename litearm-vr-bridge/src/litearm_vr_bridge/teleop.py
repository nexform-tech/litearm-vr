"""头显帧 → teleop：发到 zenoh 的 `vr_frame` 话题（DESIGN 6.1）。

连续相同按钮/追踪状态保留首帧和最新位姿，控制边沿保持顺序。
若控制边沿也撑满队列，发送 teleopInputFault 屏障，消费者必须保持并等待重新握持。
保留帧内容原样转发；合并/溢出计数。网络 I/O 始终在独立线程，避免阻塞看门狗。
"""

from __future__ import annotations

import json
import logging
import queue
import threading
import time

log = logging.getLogger("litearm_vr_bridge.teleop")

__all__ = ["TeleopPublisher"]

#: 排队上限；保留控制边沿，同状态位姿合并，边沿溢出发送失效屏障。
DEFAULT_QUEUE_SIZE = 64

#: 停线程时最多等它把手上那帧发完
_STOP_TIMEOUT_S = 2.0




class _LatestFrameQueue(queue.Queue):
    """Keep each control edge and its first pose, then only its newest pose.

    Packages deploy independently, so this small queue is mirrored in bridge/teleop.
    Overflow of distinct edges emits a fault barrier; the consumer must hold until
    release/regrip. Payloads that survive coalescing remain byte-for-byte unchanged.
    """

    def __init__(self, maxsize):
        if maxsize < 2:
            raise ValueError("queue_size must be >= 2 (fault barrier + latest frame)")
        super().__init__(maxsize)
        self._signature = None
        self._replaceable = False

    @staticmethod
    def _controls(raw):
        try:
            data = json.loads(raw)
            if not isinstance(data, dict) or "teleopInputFault" in data:
                return None
            states = []
            for hand in ("leftController", "rightController"):
                controller = data.get(hand)
                if controller is not None and not isinstance(controller, dict):
                    return None
                states.append(None if controller is None else {
                    key: value for key, value in controller.items()
                    if key not in ("position", "rotation", "quaternion")})
            if states == [None, None]:
                return None
            return json.dumps(states, sort_keys=True)
        except (ValueError, TypeError):
            return None

    def offer_latest(self, item, raw, fault_item):
        """Return discarded count. A first/edge pose is never coalesced away."""
        signature = self._controls(raw)
        with self.not_full:
            same = signature is not None and signature == self._signature
            if same and self._replaceable and self.queue:
                self.queue[-1] = item
                return 1
            discarded = 0
            if self._qsize() >= self.maxsize:
                discarded = self._qsize()
                self.queue.clear()
                self.queue.append(fault_item)
                # The newest frame becomes a new, protected reference.
                same = False
            self.queue.append(item)
            self._signature = signature
            self._replaceable = same
            self.not_empty.notify()
            return discarded


class TeleopPublisher:
    """把帧逐条发布到 zenoh。网络 I/O 在 `zenoh-pub` 线程上。"""

    def __init__(self, topic: str, *, queue_size: int = DEFAULT_QUEUE_SIZE) -> None:
        self._topic = topic
        self._queue: queue.Queue[tuple[bytes, float] | None] = _LatestFrameQueue(maxsize=queue_size)
        self._thread: threading.Thread | None = None
        self._publisher = None
        self._on_sent = None
        self._on_dropped = None

        # 这几个由发布线程写、事件循环读。CPython 里 int/float 的读写是原子的，
        # 读到一个"上一拍"的值没有影响（它们只进快照，不做判据）。
        self.frames_sent = 0
        self.frames_dropped = 0
        self.last_cost_ms: float | None = None
        self.last_error: str | None = None

    # ── 生命周期 ────────────────────────────────────────────────────────────
    def start(self, session, *, on_sent=None, on_dropped=None) -> None:
        """声明发布者并起线程。非阻塞。

        `on_sent(cost_ms)` / `on_dropped()` 由**发布线程**调用，调用方负责切回事件
        循环（`loop.call_soon_threadsafe`）。
        """
        if self._thread is not None:
            return
        self._on_sent = on_sent
        self._on_dropped = on_dropped
        self._publisher = session.declare_publisher(self._topic)
        self._thread = threading.Thread(
            target=self._run, name="zenoh-pub", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """停线程并 undeclare。调方线程（从事件循环来就 `asyncio.to_thread`）。"""
        thread, self._thread = self._thread, None
        if thread is None:
            return
        try:
            self._queue.put_nowait(None)  # 哨兵
        except queue.Full:
            pass
        thread.join(timeout=_STOP_TIMEOUT_S)
        if thread.is_alive():
            log.warning("zenoh-pub 线程 %.1fs 内没停下来，不再等它", _STOP_TIMEOUT_S)
        publisher, self._publisher = self._publisher, None
        if publisher is not None:
            try:
                publisher.undeclare()
            except Exception as exc:  # noqa: BLE001
                log.debug("undeclare 帧发布者时出错（忽略）：%s", exc)

    # ── 观测 ────────────────────────────────────────────────────────────────
    @property
    def subscribed(self) -> bool:
        """teleop 订上了没有。

        这是 `connected` 的等价物：pub/sub 没有"连接"事件，但 publisher 知道自己有
        没有匹配的订阅者。`teleop.start` 要等这个为真再下发 `start` —— 否则帧发出去
        没人接，而跟随服务已经收到了 start（就是 commit 6871570 修的那个抢跑）。
        """
        publisher = self._publisher
        if publisher is None:
            return False
        try:
            # ⚠ 这两级都是**属性**不是方法（`matching_status` 给出 MatchingStatus，
            #   它自己的 `.matching` 才是布尔）。zenoh-python 的 .pyi 把它们写成了
            #   `def matching_status(self) -> bool`，照着 .pyi 写会在运行时报
            #   "'MatchingStatus' object is not callable"。
            return bool(publisher.matching_status.matching)
        except Exception as exc:  # noqa: BLE001
            log.debug("查匹配状态失败：%s", exc)
            return False

    @property
    def pending(self) -> int:
        """还排在队列里没发出去的帧数 —— 真正的背压指标。"""
        return self._queue.qsize()

    # ── 收帧路径（事件循环） ────────────────────────────────────────────────
    def offer(self, raw: str) -> bool:
        """把一帧排进队列。返回 False 表示未连接；合并旧帧和溢出屏障均计数。

        **不做网络 I/O**，所以事件循环上直接调它是安全的。
        """
        if self._thread is None:
            self.frames_dropped += 1
            self._notify_dropped()
            return False
        if not self.subscribed:
            # **没人订的时候必须计成丢帧。** pub/sub 的 `put()` 在没有订阅者时是
            # "成功"的 —— 帧进 zenoh 就被丢掉了，调用方一无所知。照实计数，否则
            # `frames_dropped` 会在"teleop 根本没在听"的时候显示 0 帧丢失，而这正是
            # 最该被看见的一种故障（等价于改造前的 `connected == False`）。
            self.frames_dropped += 1
            self._notify_dropped()
            return False
        now = time.monotonic()
        discarded = self._queue.offer_latest(
            (raw.encode("utf-8"), now), raw,
            (b'{"teleopInputFault":"queue_overflow"}', now))
        self.frames_dropped += discarded
        for _ in range(discarded):
            self._notify_dropped()
        return True

    # ── 发布线程 ────────────────────────────────────────────────────────────
    def _run(self) -> None:
        fault_pending = False
        while True:
            item = self._queue.get()
            if item is None:  # 哨兵
                return
            payload, received_mono = item
            try:
                # A failed publish may have lost a release or the overflow barrier.
                # Re-establish the hold barrier before forwarding any later pose.
                if fault_pending:
                    self._publisher.put(b'{"teleopInputFault":"publish_failure"}')
                    fault_pending = False
                self._publisher.put(payload)
            except Exception as exc:  # noqa: BLE001
                fault_pending = True
                self.frames_dropped += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                log.warning("发布帧失败，丢弃该帧：%s", self.last_error)
                self._notify_dropped()
                continue
            cost_ms = (time.monotonic() - received_mono) * 1000.0
            self.frames_sent += 1
            self.last_cost_ms = cost_ms
            self.last_error = None
            if self._on_sent is not None:
                self._on_sent(cost_ms)

    def _notify_dropped(self) -> None:
        if self._on_dropped is not None:
            self._on_dropped()

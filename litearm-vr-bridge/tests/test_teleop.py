"""帧发布器：没订上、队满、以及"发布线程出错"这三种丢帧都要**记账**。

用假的 session/publisher —— 这里验的是记账逻辑本身，真 zenoh 的收发在 e2e 里验。
"""

from __future__ import annotations

import json
import threading
import time
import unittest

from litearm_vr_bridge.teleop import TeleopPublisher, _LatestFrameQueue


class _FakePublisher:
    def __init__(self, *, boom: bool = False) -> None:
        self.puts: list[bytes] = []
        self.undeclared = False
        self._boom = boom
        self.matching = True

    @property
    def matching_status(self):  # noqa: ANN201 - 与 zenoh 同形（属性套属性）
        outer = self

        class _Status:
            @property
            def matching(self) -> bool:
                return outer.matching

        return _Status()

    def put(self, payload: bytes) -> None:
        if self._boom:
            raise RuntimeError("网络断了")
        self.puts.append(payload)

    def undeclare(self) -> None:
        self.undeclared = True


class _FakeSession:
    def __init__(self, *, boom: bool = False) -> None:
        self.publisher = _FakePublisher(boom=boom)
        self.topic: str | None = None

    def declare_publisher(self, topic: str):  # noqa: ANN201
        self.topic = topic
        return self.publisher


class PublisherTest(unittest.TestCase):
    def _publisher(self, session: _FakeSession | None = None, **kw) -> TeleopPublisher:
        session = session or _FakeSession()
        pub = TeleopPublisher("litearm/v4/armA/vr_frame", **kw)
        pub.start(session)
        self.addCleanup(pub.stop)
        return pub

    def test_forwards_in_order_and_reports_cost(self) -> None:
        session = _FakeSession()
        sent: list[float] = []
        pub = TeleopPublisher("t")
        pub.start(session, on_sent=sent.append)
        self.addCleanup(pub.stop)

        for i in range(5):
            self.assertTrue(pub.offer(f"frame-{i}"))
        self._drain(pub, 5)

        self.assertEqual([p.decode() for p in session.publisher.puts],
                         [f"frame-{i}" for i in range(5)])
        self.assertEqual(pub.frames_sent, 5)
        self.assertEqual(pub.frames_dropped, 0)
        self.assertEqual(len(sent), 5)
        self.assertIsNotNone(pub.last_cost_ms)

    def test_latest_queue_preserves_press_anchor_and_release(self) -> None:
        frames = _LatestFrameQueue(8)
        for x, grip in [(0, True), (1, True), (2, True), (3, False),
                        (4, False), (5, True), (6, True), (7, True)]:
            raw = json.dumps({"rightController": {
                "gripActive": grip, "tracked": True,
                "position": {"x": x}}})
            frames.offer_latest(raw, raw, '{"teleopInputFault":"queue_overflow"}')
        actual = [json.loads(frames.get_nowait())["rightController"]
                  for _ in range(frames.qsize())]
        self.assertEqual([f["position"]["x"] for f in actual], [0, 2, 3, 4, 5, 7])
        self.assertEqual([f["gripActive"] for f in actual],
                         [True, True, False, False, True, True])

    def test_no_subscriber_counts_as_dropped(self) -> None:
        """**没人订的时候必须计成丢帧。**

        pub/sub 的 put() 在没有订阅者时是"成功"的，帧进 zenoh 就没了 —— 不记账的话
        `frames_dropped` 会在"teleop 根本没在听"时显示 0，而那正是最该被看见的故障。
        """
        session = _FakeSession()
        session.publisher.matching = False
        dropped: list[int] = []
        pub = TeleopPublisher("t")
        pub.start(session, on_dropped=lambda: dropped.append(1))
        self.addCleanup(pub.stop)

        self.assertFalse(pub.offer("frame"))
        self._drain(pub, 0)  # 什么都没入队，不该有 put

        self.assertEqual(session.publisher.puts, [])
        self.assertEqual(pub.frames_dropped, 1)
        self.assertEqual(pub.frames_sent, 0)
        self.assertEqual(len(dropped), 1)

    def test_full_queue_emits_fault_then_latest(self) -> None:
        """边沿无法保留时，失效屏障必须先于最新帧到达。"""

        class _Blocked(_FakePublisher):
            def __init__(self) -> None:
                super().__init__()
                self.entered = threading.Event()
                self.release = threading.Event()

            def put(self, payload: bytes) -> None:
                self.entered.set()
                self.release.wait(timeout=3.0)
                super().put(payload)

        session = _FakeSession()
        session.publisher = _Blocked()
        pub = TeleopPublisher("t", queue_size=2)
        pub.start(session)
        self.addCleanup(pub.stop)

        pub.offer("frame-0")  # 被线程取走，卡在 put 里
        self.assertTrue(session.publisher.entered.wait(timeout=3.0), "发布线程没进 put")

        pub.offer("frame-1")
        pub.offer("frame-2")
        self.assertTrue(pub.offer("frame-3"))
        self.assertEqual(pub.frames_dropped, 2)

        session.publisher.release.set()
        self._drain(pub, 2)
        self.assertEqual([p.decode() for p in session.publisher.puts],
                         ["frame-0", '{"teleopInputFault":"queue_overflow"}', "frame-3"])

    def test_publish_error_counts_as_dropped_and_keeps_going(self) -> None:
        """发布线程里出错不能把线程带走 —— 丢那帧、记账、继续。"""
        session = _FakeSession(boom=True)
        dropped: list[int] = []
        pub = TeleopPublisher("t")
        pub.start(session, on_dropped=lambda: dropped.append(1))
        self.addCleanup(pub.stop)

        self.assertTrue(pub.offer("frame"))
        self._drain(pub, 1)

        self.assertEqual(pub.frames_dropped, 1)
        self.assertEqual(pub.frames_sent, 0)
        self.assertEqual(len(dropped), 1)
        self.assertIsNotNone(pub.last_error)

    def test_publish_recovery_inserts_hold_before_next_pose(self) -> None:
        session = _FakeSession(boom=True)
        pub = self._publisher(session)
        pub.offer("lost-release")
        self._drain(pub, 1)
        session.publisher._boom = False
        pub.offer("held-again")
        self._drain(pub, 2)
        self.assertEqual(session.publisher.puts,
                         [b'{"teleopInputFault":"publish_failure"}', b"held-again"])

    def test_stop_before_start_is_a_noop(self) -> None:
        TeleopPublisher("t").stop()  # 不该抛

    # ── 工具 ────────────────────────────────────────────────────────────────
    def _drain(self, pub: TeleopPublisher, expect_sent: int, timeout_s: float = 3.0) -> None:
        """等发布线程把队列清干净。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if pub.frames_sent + pub.frames_dropped >= expect_sent and pub.pending == 0:
                time.sleep(0.05)  # 让最后一次 on_sent/on_dropped 落地
                return
            time.sleep(0.01)


if __name__ == "__main__":
    unittest.main()

"""给上位机（litearm-studio）的状态推送与动作入口，走 zenoh（DESIGN 4）。

两条纪律（与走 WebSocket 时一样）：
  1. 这条通道上**只发快照**。上位机把每条消息都当成整份快照覆盖，所以动作回执不另发
     消息，一律写进快照的 `events`。
  2. 上位机只订摘要，不订原始帧。

与改造前的三点不同：

* **状态是 pub，不是推给具体某条连接。** 没有"客户端"这个对象了，`clients` 变成
  "有没有订阅者"（0/1）。
* **动作走 query，不是同一条连接上的回包。** 这是唯一能区分「命令没送到」和
  「整条链路断了」的手段：桥挂住没死时 pub 下去没人处理，上位机只能等状态超时；
  query 超时就能给出结论。
* **没订阅者也照常发布。** `EventLog.drain()` 名字骗人 —— 它只按 TTL 剪枝、把剩下的
  全部返回，读**不**消费（见 events.py）。所以"没人看的时候事件被抽干"不成立。
  **不要**改成按订阅者数量 gate：没有可靠的计数，猜错的表现是**静默丢告警**。
"""

from __future__ import annotations

import asyncio
import json
import logging
import queue
import threading
import time
from typing import Any, Awaitable, Callable

log = logging.getLogger("litearm_vr_bridge.studio")

__all__ = ["StudioLink"]

SnapshotProvider = Callable[[], dict[str, Any]]
ActionHandler = Callable[[dict[str, Any]], Awaitable[dict[str, Any]]]

#: 发布线程等一次网络写完的上限；超了就当这一拍没发出去（下一拍马上又到）
_PUT_TIMEOUT_S = 2.0
_STOP_TIMEOUT_S = 2.0


class StudioLink:
    """`vr_status` 的发布者 + `vr_rpc` 的 queryable。"""

    def __init__(
        self,
        snapshot_provider: SnapshotProvider,
        action_handler: ActionHandler,
        *,
        status_topic: str,
        rpc_topic: str,
        status_hz: float = 10.0,
    ) -> None:
        self._snapshot_provider = snapshot_provider
        self._action_handler = action_handler
        self._status_topic = status_topic
        self._rpc_topic = rpc_topic
        self._interval_s = 1.0 / max(status_hz, 0.1)

        self._loop: asyncio.AbstractEventLoop | None = None
        self._task: asyncio.Task[None] | None = None
        self._publisher: Any = None
        self._queryable: Any = None
        #: maxsize=1：状态是整份覆盖式，积压的旧快照没有意义，直接丢
        self._queue: queue.Queue[bytes | None] = queue.Queue(maxsize=1)
        self._thread: threading.Thread | None = None
        self._closing = False

        self.published = 0
        self.publish_failures = 0
        self.commands = 0
        self.last_command_mono: float | None = None
        self.last_error: str | None = None

    # ── 生命周期 ────────────────────────────────────────────────────────────
    def start(self, session: Any, loop: asyncio.AbstractEventLoop) -> None:
        """声明发布者与 queryable，起发布线程，并在事件循环上起推送节拍。"""
        self._loop = loop
        self._publisher = session.declare_publisher(self._status_topic)
        self._queryable = session.declare_queryable(self._rpc_topic, self._on_query)
        self._thread = threading.Thread(target=self._run, name="zenoh-status", daemon=True)
        self._thread.start()
        self._task = loop.create_task(self._loop_tick(), name="status-publish")
        log.info("状态发布 %s（%.0f Hz）；动作 %s", self._status_topic, 1 / self._interval_s,
                 self._rpc_topic)

    def stop(self) -> None:
        """停线程、撤销声明。同步阻塞，从事件循环调用请用 `asyncio.to_thread`。"""
        self._closing = True
        task, self._task = self._task, None
        if task is not None and self._loop is not None:
            self._loop.call_soon_threadsafe(task.cancel)

        thread, self._thread = self._thread, None
        if thread is not None:
            try:
                self._queue.put_nowait(None)  # 哨兵
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._queue.put_nowait(None)
                except queue.Full:
                    pass
            thread.join(timeout=_STOP_TIMEOUT_S)
            if thread.is_alive():
                log.warning("zenoh-status 线程 %.1fs 内没停下来，不再等它", _STOP_TIMEOUT_S)

        for prim in (self._queryable, self._publisher):
            if prim is None:
                continue
            try:
                prim.undeclare()
            except Exception as exc:  # noqa: BLE001
                log.debug("撤销 zenoh 声明时出错（忽略）：%s", exc)
        self._queryable = self._publisher = None

    # ── 观测 ────────────────────────────────────────────────────────────────
    @property
    def clients(self) -> int:
        """有没有上位机在订（0/1）。快照里的 `bridge.clients.status` 用它。"""
        return 1 if self.subscribed else 0

    @property
    def subscribed(self) -> bool:
        publisher = self._publisher
        if publisher is None:
            return False
        try:
            return bool(publisher.matching_status.matching)  # 属性套属性，别加括号
        except Exception as exc:  # noqa: BLE001
            log.debug("查订阅状态失败：%s", exc)
            return False

    # ── 推送节拍（事件循环） ────────────────────────────────────────────────
    async def _loop_tick(self) -> None:
        """按 `status_hz` 生成快照并交给发布线程。

        **无条件发布**，不看有没有订阅者（理由见模块开头）。序列化失败只记日志，
        不让这一拍把整条节拍打断。
        """
        while not self._closing:
            started = time.monotonic()
            try:
                payload = json.dumps(self._snapshot_provider(), ensure_ascii=False)
            except Exception as exc:  # noqa: BLE001
                self.last_error = f"快照序列化失败：{exc}"
                log.error("%s", self.last_error)
            else:
                self._offer(payload.encode("utf-8"))
            await asyncio.sleep(max(self._interval_s - (time.monotonic() - started), 0.001))

    def _offer(self, payload: bytes) -> None:
        """入队最新快照；满了就把旧的挤掉（整份覆盖式，旧的无意义）。"""
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(payload)
            except queue.Full:
                pass

    def _run(self) -> None:
        while True:
            payload = self._queue.get()
            if payload is None:
                return
            try:
                self._publisher.put(payload)
            except Exception as exc:  # noqa: BLE001
                self.publish_failures += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                log.warning("发布快照失败（下一拍继续）：%s", self.last_error)
                continue
            self.published += 1

    # ── 动作（zenoh 回调线程） ──────────────────────────────────────────────
    def _on_query(self, query: Any) -> None:
        """queryable 的回调。

        **这里绝不能阻塞。** 回调跑在 zenoh 给的独立线程上（`Callback(indirect=True)`
        是默认），而冷启动的 `teleop.start` 要十几秒；在里面等会把这条通道整个堵住。

        做法：把工作排进事件循环就返回。**每条 query 一个协程** —— 于是慢的
        `teleop.start` 中间能被急停插进来（跟改造前"每条连接一个 task"的行为一致），
        所以这里**不加**"同一时刻只允许一条命令"的锁。
        """
        self.commands += 1
        self.last_command_mono = time.monotonic()
        try:
            data = json.loads(bytes(query.payload)) if query.payload is not None else {}
        except (ValueError, TypeError):
            data = {}
        if not isinstance(data, dict):
            data = {}

        loop = self._loop
        if loop is None or loop.is_closed():
            self._reply(query, {"ok": False, "error": {
                "type": "ShuttingDown", "message": "桥正在退出"}})
            return
        try:
            future = asyncio.run_coroutine_threadsafe(self._execute(data), loop)
        except RuntimeError as exc:  # 循环刚要关
            self._reply(query, {"ok": False, "error": {
                "type": "ShuttingDown", "message": str(exc)}})
            return
        # 回调在**完成它的那个线程**（也就是事件循环）上跑；`query.reply()` 跨线程调
        # 是成立的（实测：回调线程起 worker 回复，客户端收得到）。
        future.add_done_callback(lambda fut: self._reply(query, _future_result(fut)))

    async def _execute(self, data: dict[str, Any]) -> dict[str, Any]:
        try:
            result = await self._action_handler(data)
        except Exception as exc:  # noqa: BLE001
            log.error("处理动作 %r 时异常：%s", data.get("action"), exc)
            return {"ok": False, "error": {"type": type(exc).__name__, "message": str(exc)}}
        return result if isinstance(result, dict) else {"ok": True}

    def _reply(self, query: Any, result: dict[str, Any]) -> None:
        try:
            query.reply(query.key_expr, json.dumps(result, ensure_ascii=False).encode("utf-8"))
        except Exception as exc:  # noqa: BLE001
            log.warning("回动作回执失败：%s", exc)

    # ── 状态 ────────────────────────────────────────────────────────────────
    def state(self) -> dict[str, Any]:
        """给快照与 `/healthz` 用的一小段自述。"""
        return {
            "subscribed": self.subscribed,
            "published": self.published,
            "publish_failures": self.publish_failures,
            "commands": self.commands,
            "error": self.last_error,
        }


def _future_result(fut: Any) -> dict[str, Any]:
    try:
        value = fut.result()
    except asyncio.CancelledError:
        return {"ok": False, "error": {"type": "ShuttingDown", "message": "桥正在退出"}}
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": {"type": type(exc).__name__, "message": str(exc)}}
    return value if isinstance(value, dict) else {"ok": True}

"""测试公用件：临时端口、假 teleop、假跟随服务、头显帧构造。"""

from __future__ import annotations

import asyncio
import contextlib
import queue
import json
import socket
import time
from typing import Any


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def zenoh_endpoint(port: int) -> str:
    return f"tcp/127.0.0.1:{port}"


def open_test_session(*, connect: str | None = None, listen: str | None = None):
    """给用例开一条 zenoh 会话。

    **scouting 一律关掉**，两头都是 —— 不关的话同进程里跑的用例会通过 multicast
    互相发现（`unittest discover` 全在一个进程里），表现是随机的 flaky：
    某个用例收到另一个用例的帧。
    """
    import zenoh

    cfg = zenoh.Config()
    cfg.insert_json5("mode", '"client"' if connect else '"peer"')
    cfg.insert_json5("scouting/multicast/enabled", "false")
    cfg.insert_json5("scouting/gossip/enabled", "false")
    if listen:
        cfg.insert_json5("listen/endpoints", f'["{listen}"]')
    if connect:
        cfg.insert_json5("connect/endpoints", f'["{connect}"]')
    return zenoh.open(cfg)


class FakeTeleop:
    """假 teleop：在 zenoh 上订帧话题，把收到的帧攒起来给用例断言。

    **client 模式连桥的 listen 端点** —— 与现场 teleop 的接法一致（它也是订
    `vr_frame`）。收帧回调跑在 zenoh 自己的线程上，所以只做 `list.append`
    （CPython 下是原子的），别在这里碰事件循环。
    """

    def __init__(self, endpoint: str, topic: str) -> None:
        self._endpoint = endpoint
        self._topic = topic
        self._session = None
        self._subscriber = None
        self.frames: list[str] = []

    async def start(self) -> None:
        self._session = await asyncio.to_thread(open_test_session, connect=self._endpoint)
        self._subscriber = await asyncio.to_thread(
            self._session.declare_subscriber, self._topic, self._on_sample
        )

    def _on_sample(self, sample: Any) -> None:
        self.frames.append(bytes(sample.payload).decode("utf-8", "replace"))

    async def stop(self) -> None:
        session, self._session = self._session, None
        self._subscriber = None
        if session is not None:
            await asyncio.to_thread(session.close)


def frame(
    grip: bool = True,
    *,
    tracked: bool = True,
    trigger: float = 0.0,
    ts_ms: int | None = None,
    left: bool = False,
) -> str:
    """构造一帧头显 JSON（字段与 litearm-vr/docs/PROTOCOL.md 一致）。"""
    stamp = int(time.time() * 1000) if ts_ms is None else int(ts_ms)
    controller = {
        "tracked": tracked,
        "position": {"x": 0.1, "y": 0.2, "z": 0.3},
        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0},
        "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "gripActive": grip,
        "trigger": trigger,
    }
    idle = dict(controller, tracked=tracked, gripActive=False)
    return json.dumps(
        {
            "timestamp": stamp,
            "leftController": controller if left else idle,
            "rightController": idle if left else controller,
        },
        separators=(",", ":"),
    )


class FakeFollowServer:
    """按 vr/arm_follow_server_*.py 的协议返回一行一条 JSON。"""

    def __init__(self, *, q: list[float] | None = None) -> None:
        self.commands: list[str] = []
        self.q = list(q) if q else [0.0, -0.07, -0.09, -1.7, -1.46, -0.03, -0.79]
        self.state = "holding"
        self.fault: list[Any] = []
        self.respond = True
        self.server: asyncio.AbstractServer | None = None
        self.port = 0
        self.connections = 0

    async def start(self) -> int:
        self.server = await asyncio.start_server(self._handle, "127.0.0.1", 0)
        self.port = int(self.server.sockets[0].getsockname()[1])
        return self.port

    async def stop(self) -> None:
        if self.server is not None:
            self.server.close()
            await self.server.wait_closed()
            self.server = None

    def count(self, cmd: str) -> int:
        return self.commands.count(cmd)

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.connections += 1
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break
                try:
                    message = json.loads(line.decode("utf-8"))
                except (ValueError, TypeError):
                    continue
                cmd = str(message.get("cmd") or "")
                self.commands.append(cmd)
                if not self.respond:
                    continue
                writer.write((json.dumps(self._reply(cmd)) + "\n").encode("utf-8"))
                await writer.drain()
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            try:
                writer.close()
            except Exception:  # noqa: BLE001
                pass

    def _reply(self, cmd: str) -> dict[str, Any]:
        if cmd == "ping":
            if self.state == "following":
                self.q = [v + 0.01 for v in self.q]
            return {"ok": True, "state": self.state, "q": self.q, "fault": self.fault}
        if cmd == "start":
            self.state = "following"
            return {"ok": True, "cmd": cmd}
        if cmd == "hold":
            self.state = "holding"
            return {"ok": True, "cmd": cmd}
        if cmd == "latch":
            self.state = "holding"
            return {"ok": True, "cmd": cmd}
        if cmd == "stop":
            self.state = "stopped"
            return {"ok": True, "cmd": cmd}
        return {"ok": False, "error": f"unknown cmd {cmd!r}"}


class StatusWatcher:
    """假上位机：订 `vr_status` 收快照，用 `vr_rpc` 发动作。

    与现场上位机的接法同形（client 连桥的 listen 端点）。`recv` 给原始 JSON 文本、
    `call` 给回执 —— 刻意长得像原来那条 WebSocket，让用例的判读逻辑少改。
    """

    def __init__(self, endpoint: str, status_topic: str, rpc_topic: str,
                 *, call_timeout_s: float = 3.0) -> None:
        self._endpoint = endpoint
        self._status_topic = status_topic
        self._rpc_topic = rpc_topic
        self._call_timeout_s = call_timeout_s
        self._session = None
        self._subscriber = None
        self._inbox: queue.Queue[str] = queue.Queue(maxsize=256)

    async def start(self) -> None:
        self._session = await asyncio.to_thread(open_test_session, connect=self._endpoint)
        self._subscriber = await asyncio.to_thread(
            self._session.declare_subscriber, self._status_topic, self._on_sample
        )

    def _on_sample(self, sample: Any) -> None:
        text = bytes(sample.payload).decode("utf-8", "replace")
        try:
            self._inbox.put_nowait(text)
        except queue.Full:
            with contextlib.suppress(queue.Empty):
                self._inbox.get_nowait()
            with contextlib.suppress(queue.Full):
                self._inbox.put_nowait(text)

    async def recv(self, timeout_s: float = 3.0) -> str | None:
        try:
            return await asyncio.to_thread(self._inbox.get, True, max(timeout_s, 0.05))
        except queue.Empty:
            return None

    async def call(self, action: dict[str, Any]) -> dict[str, Any] | None:
        """发一条动作等回执。None = 桥没回（不可达/卡死）。"""
        return await asyncio.to_thread(self._call, action)

    def _call(self, action: dict[str, Any]) -> dict[str, Any] | None:
        import json as _json

        replies = self._session.get(
            self._rpc_topic,
            payload=_json.dumps(action, ensure_ascii=False).encode("utf-8"),
            timeout=self._call_timeout_s,
        )
        for reply in replies:
            ok = getattr(reply, "ok", None)
            if ok is None:
                continue
            try:
                parsed = _json.loads(bytes(ok.payload))
            except (ValueError, TypeError):
                continue
            if isinstance(parsed, dict):
                return parsed
        return None

    async def stop(self) -> None:
        session, self._session = self._session, None
        self._subscriber = None
        if session is not None:
            await asyncio.to_thread(session.close)

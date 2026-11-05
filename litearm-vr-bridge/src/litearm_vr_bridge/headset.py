"""头显入口：监听 8765，收手柄帧（DESIGN 2.1 / 5）。

**这个端口现在只服务头显。** 上位机的状态订阅迁到 zenoh 了（协议见 protocol.py），
但旧的 `/ws/status` 路径仍在 `_process_request` 里被明确回绝 —— 不能只是"不再特判"：
本模块的纪律是"新的头显连接顶替旧的"，所以一个缓存了旧页面的上位机连上
`/ws/status` 会**顶掉真正的头显**，还会开始往帧路径里灌它自己的空 socket。

另外提供一个只读的 HTTP `/healthz`，给 systemd / 巡检脚本用（非 WebSocket 请求）。
zenoh 挂了的时候，它是唯一的带外通道，所以别删。
"""

from __future__ import annotations

import json
import logging
import time
from typing import Any, Awaitable, Callable

from websockets.asyncio.server import ServerConnection, serve
from websockets.datastructures import Headers
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Response

from .config import LEGACY_STATUS_PATHS

log = logging.getLogger("litearm_vr_bridge.headset")

__all__ = ["HeadsetServer"]

FrameHandler = Callable[[str, float, float], Awaitable[None]]


class HeadsetServer:
    def __init__(
        self,
        host: str,
        port: int,
        *,
        on_frame: FrameHandler,
        on_client_change: Callable[[], None] | None = None,
        health_provider: Callable[[], dict[str, Any]] | None = None,
        legacy_status_paths: tuple[str, ...] = LEGACY_STATUS_PATHS,
        ping_interval_s: float = 10.0,
        ping_timeout_s: float = 10.0,
    ) -> None:
        self._host = host
        self._port = port
        self._on_frame = on_frame
        self._on_client_change = on_client_change
        self._health_provider = health_provider
        self._legacy_status_paths = set(legacy_status_paths)
        self._ping_interval_s = ping_interval_s
        self._ping_timeout_s = ping_timeout_s

        self._server: Any = None
        self._active: ServerConnection | None = None

        self.messages_in = 0
        self.bytes_in = 0
        self.bad_messages = 0
        self.connections = 0
        self.last_connect_at: float | None = None
        self.last_disconnect_at: float | None = None

    # ── 生命周期 ────────────────────────────────────────────────────────────
    async def start(self) -> None:
        self._server = await serve(
            self._handle,
            self._host,
            self._port,
            ping_interval=self._ping_interval_s,
            ping_timeout=self._ping_timeout_s,
            max_size=1 << 20,
            process_request=self._process_request,
        )
        log.info("监听 ws://%s:%d（只收头显帧；状态口已迁到 zenoh）", self._host, self._port)

    async def close(self) -> None:
        server, self._server = self._server, None
        conn, self._active = self._active, None
        if conn is not None:
            try:
                await conn.close()
            except Exception:  # noqa: BLE001
                pass
        if server is not None:
            server.close()
            try:
                await server.wait_closed()
            except Exception:  # noqa: BLE001
                pass

    @property
    def headset_connected(self) -> bool:
        return self._active is not None and _is_open(self._active)

    @property
    def clients(self) -> int:
        return 1 if self.headset_connected else 0

    # ── 分流 ────────────────────────────────────────────────────────────────
    async def _handle(self, conn: ServerConnection) -> None:
        await self._handle_headset(conn)

    async def _handle_headset(self, conn: ServerConnection) -> None:
        peer = _peer(conn)
        previous, self._active = self._active, conn
        if previous is not None and _is_open(previous):
            log.warning("新的头显连接 %s 顶替旧连接（同一时间只转发一路）", peer)
            try:
                await previous.close(code=1000, reason="replaced by a new connection")
            except Exception:  # noqa: BLE001
                pass

        self.connections += 1
        self.last_connect_at = time.time()
        log.info("头显已连接：%s（路径 %s）", peer, _path_of(conn))
        self._notify_change()
        try:
            async for message in conn:
                now_mono = time.monotonic()
                now_wall_ms = time.time() * 1000.0
                if isinstance(message, (bytes, bytearray)):
                    try:
                        raw = bytes(message).decode("utf-8")
                    except UnicodeDecodeError:
                        self.bad_messages += 1
                        continue
                else:
                    raw = message
                self.messages_in += 1
                self.bytes_in += len(raw)
                try:
                    await self._on_frame(raw, now_mono, now_wall_ms)
                except Exception as exc:  # noqa: BLE001
                    # 桥是链路的唯一入口：单帧处理出问题不能把整条连接带走
                    self.bad_messages += 1
                    log.error("处理头显帧出错（继续收帧）：%s", exc)
        except ConnectionClosed:
            pass
        except Exception as exc:  # noqa: BLE001
            log.warning("头显连接异常（%s）：%s", peer, exc)
        finally:
            if self._active is conn:
                self._active = None
            self.last_disconnect_at = time.time()
            log.info("头显断开：%s", peer)
            self._notify_change()

    # ── 健康检查 ────────────────────────────────────────────────────────────
    def _process_request(self, conn: ServerConnection, request: Any) -> Response | None:
        upgrade = (request.headers.get("Upgrade") or "").lower()
        if "websocket" in upgrade:
            return None
        path = (request.path or "/").split("?", 1)[0]
        if path in ("/healthz", "/health", "/"):
            payload = self._health_provider() if self._health_provider else {"ok": True}
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            return Response(
                200, "OK",
                Headers({"Content-Type": "application/json; charset=utf-8",
                         "Cache-Control": "no-store"}),
                body,
            )
        return Response(
            404, "Not Found",
            Headers({"Content-Type": "text/plain; charset=utf-8"}),
            b"litearm-vr-bridge: headset frames only; status is on zenoh\n",
        )

    def _notify_change(self) -> None:
        if self._on_client_change is not None:
            self._on_client_change()


def _path_of(conn: ServerConnection) -> str:
    request = getattr(conn, "request", None)
    path = getattr(request, "path", None) if request is not None else None
    if path is None:
        path = getattr(conn, "path", "/")
    return (path or "/").split("?", 1)[0]


def _is_open(conn: ServerConnection) -> bool:
    """websockets 各版本对"连接是否还开着"的表示不同，这里统一判一下。"""
    closed = getattr(conn, "closed", None)
    if isinstance(closed, bool):
        return not closed
    state = getattr(conn, "state", None)
    name = getattr(state, "name", None)
    if isinstance(name, str):
        return name == "OPEN"
    return True


def _peer(conn: ServerConnection) -> str:
    try:
        addr = conn.remote_address
    except Exception:  # noqa: BLE001
        return "?"
    if isinstance(addr, tuple) and len(addr) >= 2:
        return f"{addr[0]}:{addr[1]}"
    return str(addr)

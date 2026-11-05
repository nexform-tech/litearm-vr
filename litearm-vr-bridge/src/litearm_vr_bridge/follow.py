"""arm_follow_server（TCP 9001，每行一条 JSON）客户端。

DESIGN 第 5 节限制：本服务对 9001 的使用**限定在 start / hold / latch / stop / ping**，
绝不转发关节角流（那是 teleop 的事）。

`hold` 与 `latch` 的区别很重要：跟随服务的 `hold` 只是按实测角保持、不改模式；
`latch` 的实现是 `movej(当前角)` + 到位增刚，掉线兜底**用 hold，不用 latch**。

上游服务的特性（vr/arm_follow_server_*.py）：
  * 无状态广播，只能轮询 `ping` 拿 state/q/fault；
  * 客户端断开只打日志，跟随状态不变 —— 本项目的兜底正是补这个洞；
  * `movej` / `home` 会阻塞该连接线程，所以这里用**独立持久连接**，
    避免被 teleop 那边的阻塞命令拖住。
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Any

log = logging.getLogger("litearm_vr_bridge.follow")

__all__ = ["FollowClient"]


class FollowClient:
    def __init__(
        self,
        host: str,
        port: int,
        *,
        connect_timeout_s: float = 2.0,
        request_timeout_s: float = 1.0,
    ) -> None:
        self._host = host
        self._port = port
        self._connect_timeout_s = connect_timeout_s
        self._request_timeout_s = request_timeout_s

        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._lock = asyncio.Lock()
        self._last_log_mono = 0.0

        self.last_ok_mono: float | None = None
        self.last_error: str | None = None
        self.requests = 0
        self.failures = 0
        self.reconnects = 0

        # 最近一次 ping 得到的真机状态
        self.state: str | None = None
        self.q: list[float] | None = None
        self.fault: list[Any] | None = None

    # ── 观测 ────────────────────────────────────────────────────────────────
    @property
    def connected(self) -> bool:
        return self._writer is not None

    @property
    def endpoint(self) -> str:
        return f"{self._host}:{self._port}"

    def last_ok_age_ms(self, now_mono: float) -> float | None:
        if self.last_ok_mono is None:
            return None
        return max(0.0, (now_mono - self.last_ok_mono) * 1000.0)

    # ── 命令 ────────────────────────────────────────────────────────────────
    async def ping(self) -> dict[str, Any] | None:
        reply = await self.request({"cmd": "ping"})
        if isinstance(reply, dict):
            self._absorb_state(reply)
        return reply

    async def start(self, *, timeout_s: float = 3.0) -> dict[str, Any] | None:
        return await self.request({"cmd": "start"}, timeout_s=timeout_s)

    async def hold(self, *, timeout_s: float = 3.0) -> dict[str, Any] | None:
        return await self.request({"cmd": "hold"}, timeout_s=timeout_s)

    async def latch(self, *, timeout_s: float = 3.0) -> dict[str, Any] | None:
        return await self.request({"cmd": "latch"}, timeout_s=timeout_s)

    async def stop(self, *, timeout_s: float = 2.0) -> dict[str, Any] | None:
        return await self.request({"cmd": "stop"}, timeout_s=timeout_s)

    async def request(
        self,
        message: dict[str, Any],
        *,
        timeout_s: float | None = None,
    ) -> dict[str, Any] | None:
        """发一条命令并等一条回包；任何异常都返回 None（调用方按失败处理）。"""
        timeout = self._request_timeout_s if timeout_s is None else timeout_s
        self.requests += 1
        try:
            async with self._lock:
                await self._ensure_connection()
                assert self._writer is not None and self._reader is not None
                payload = (json.dumps(message, ensure_ascii=False) + "\n").encode("utf-8")
                self._writer.write(payload)
                await self._writer.drain()
                line = await asyncio.wait_for(self._reader.readline(), timeout=timeout)
        except (OSError, asyncio.TimeoutError, asyncio.IncompleteReadError) as exc:
            self.failures += 1
            self.last_error = f"{type(exc).__name__}: {exc}"
            self._log_failure(message.get("cmd"), self.last_error)
            await self.close()
            return None

        self.last_ok_mono = time.monotonic()
        self.last_error = None
        if not line:
            self.failures += 1
            self.last_error = "服务端关闭连接"
            await self.close()
            return None
        try:
            reply = json.loads(line.decode("utf-8", "replace"))
        except (ValueError, TypeError):
            self.failures += 1
            self.last_error = "回包不是合法 JSON"
            return None
        return reply if isinstance(reply, dict) else None

    async def close(self) -> None:
        writer, self._writer = self._writer, None
        self._reader = None
        if writer is None:
            return
        try:
            writer.close()
            await asyncio.wait_for(writer.wait_closed(), timeout=1.0)
        except Exception:  # noqa: BLE001
            pass

    # ── 内部 ────────────────────────────────────────────────────────────────
    async def _ensure_connection(self) -> None:
        if self._writer is not None and not self._writer.is_closing():
            return
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(self._host, self._port),
            timeout=self._connect_timeout_s,
        )
        self._reader = reader
        self._writer = writer
        self.reconnects += 1
        log.info("已连上跟随服务：%s", self.endpoint)

    def _log_failure(self, cmd: object, detail: str) -> None:
        """跟随服务不在时不要刷屏：同一类失败最多 5s 一条。"""
        now = time.monotonic()
        if now - self._last_log_mono >= 5.0:
            self._last_log_mono = now
            log.warning("跟随服务 %s 请求 %s 失败：%s", self.endpoint, cmd, detail)
        else:
            log.debug("跟随服务 %s 请求 %s 失败：%s", self.endpoint, cmd, detail)

    def _absorb_state(self, reply: dict[str, Any]) -> None:
        state = reply.get("state")
        if isinstance(state, str):
            self.state = state
        q = reply.get("q")
        if isinstance(q, (list, tuple)) and len(q) == 7:
            try:
                self.q = [float(x) for x in q]
            except (TypeError, ValueError):
                pass
        # 两版 follow_server 故障字段名不同：SDK 版 ping 回 `fault`（[(joint, err), ...]），
        # stm32 版回 `flags`（flag 名列表，健康为空）。统一都认，存成同一份 self.fault。
        fault = reply.get("fault")
        if not isinstance(fault, (list, tuple)):
            fault = reply.get("flags")
        if isinstance(fault, (list, tuple)):
            self.fault = list(fault)
        if reply.get("ok") is False:
            self.last_error = str(reply.get("error") or "follow_server 回包 ok=false")

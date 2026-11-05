"""受管进程（teleop / arm_follow_server）的启停与状态（DESIGN 2.1 / M3）。

设计取舍：
  * **不做自动重启**。这两个进程都会驱动机械臂，崩溃后自动拉起意味着机械可能
    在无人在场时重新上电/回零。这里只把状态如实报上去（running/stopped/crashed），
    由操作员决定；常驻与崩溃重启是 systemd 对本服务自己的事（见 deploy/）。
  * 停止时先 SIGTERM 整组（含子进程），超时才 SIGKILL。
"""

from __future__ import annotations

import asyncio
import logging
import os
import signal
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, IO

from .config import ProcessSpec

log = logging.getLogger("litearm_vr_bridge.processes")

__all__ = ["ProcessSupervisor", "ProcessRuntime"]

STATUS_RUNNING = "running"
STATUS_STOPPED = "stopped"
STATUS_CRASHED = "crashed"
STATUS_UNKNOWN = "unknown"


@dataclass
class ProcessRuntime:
    spec: ProcessSpec
    proc: asyncio.subprocess.Process | None = None
    log_path: Path | None = None
    log_handle: IO[bytes] | None = None
    started_at: float | None = None
    exited_at: float | None = None
    exit_code: int | None = None
    last_error: str | None = None
    starts: int = 0
    stops: int = 0
    stopping: bool = False
    stopped_by_us: bool = False
    extra: dict[str, Any] = field(default_factory=dict)

    def status(self) -> str:
        if self.proc is not None:
            return STATUS_RUNNING if self.proc.returncode is None else STATUS_STOPPED
        if self.stopped_by_us:
            return STATUS_STOPPED
        if self.exit_code not in (None, 0):
            return STATUS_CRASHED
        if self.started_at is None and self.exit_code is None:
            return STATUS_STOPPED
        return STATUS_STOPPED


class ProcessSupervisor:
    def __init__(
        self,
        specs: dict[str, ProcessSpec],
        *,
        log_dir: Path,
        on_exit: Callable[[str, int | None, bool], None] | None = None,
        stop_timeout_s: float = 5.0,
    ) -> None:
        self._specs = dict(specs)
        self._log_dir = Path(log_dir)
        self._on_exit = on_exit
        self._stop_timeout_s = stop_timeout_s
        self._runtime: dict[str, ProcessRuntime] = {
            name: ProcessRuntime(spec=spec) for name, spec in self._specs.items()
        }
        self._lock = asyncio.Lock()

    # ── 查询 ────────────────────────────────────────────────────────────────
    def names(self) -> list[str]:
        return list(self._specs)

    def configured(self, name: str) -> bool:
        return name in self._specs

    def running(self, name: str) -> bool:
        rt = self._runtime.get(name)
        return bool(rt and rt.proc is not None and rt.proc.returncode is None)

    def statuses(self) -> dict[str, str]:
        return {name: rt.status() for name, rt in self._runtime.items()}

    def details(self) -> dict[str, dict[str, Any]]:
        now = time.time()
        out: dict[str, dict[str, Any]] = {}
        for name, rt in self._runtime.items():
            proc = rt.proc
            # 已退出的进程要把运行时长冻结在退出那一刻，否则界面上的"已运行"
            # 会一直涨，看起来像还活着
            uptime_end = rt.exited_at if rt.exited_at is not None else now
            out[name] = {
                "status": rt.status(),
                "command": rt.spec.label(),
                "pid": proc.pid if proc is not None and proc.returncode is None else None,
                "uptime_s": round(uptime_end - rt.started_at, 1) if rt.started_at else None,
                "exit_code": rt.exit_code,
                "starts": rt.starts,
                "last_error": rt.last_error,
            }
        return out

    def unconfigured(self) -> list[str]:
        """期望有、但没配命令的进程名（status 里报 unknown）。"""
        known = ("teleop", "follow_server")
        return [name for name in known if name not in self._specs]

    def refresh(self) -> list[tuple[str, int | None, bool]]:
        """收割已退出的子进程；返回 [(name, exit_code, expected), ...]。"""
        exits: list[tuple[str, int | None, bool]] = []
        for name, rt in self._runtime.items():
            proc = rt.proc
            if proc is None or proc.returncode is None:
                continue
            expected = rt.stopping
            rt.exit_code = proc.returncode
            rt.exited_at = time.time()
            rt.proc = None
            rt.stopping = False
            rt.stopped_by_us = expected
            self._close_log(rt)
            if not expected:
                rt.last_error = f"退出码 {proc.returncode}"
            exits.append((name, proc.returncode, expected))
            if self._on_exit is not None:
                self._on_exit(name, proc.returncode, expected)
        return exits

    # ── 启停 ────────────────────────────────────────────────────────────────
    async def start(self, name: str) -> dict[str, Any]:
        spec = self._specs.get(name)
        if spec is None:
            return {
                "ok": False,
                "error": f"进程 {name} 未配置（需要 submodule 就位，或给 {name.upper()}_CMD）",
            }
        async with self._lock:
            rt = self._runtime[name]
            if rt.proc is not None and rt.proc.returncode is None:
                return {"ok": True, "already_running": True, "pid": rt.proc.pid}
            if not spec.argv:
                return {"ok": False, "error": f"进程 {name} 命令为空"}

            log_path = self._log_dir / f"{name}.log"
            try:
                self._log_dir.mkdir(parents=True, exist_ok=True)
                handle = open(log_path, "ab", buffering=0)
            except OSError as exc:
                rt.last_error = f"日志文件打不开: {exc}"
                return {"ok": False, "error": rt.last_error}

            env = dict(os.environ)
            env.update(spec.env)
            try:
                proc = await asyncio.create_subprocess_exec(
                    *spec.argv,
                    cwd=spec.cwd or None,
                    env=env,
                    stdin=asyncio.subprocess.DEVNULL,
                    stdout=handle,
                    stderr=asyncio.subprocess.STDOUT,
                    start_new_session=True,
                )
            except (OSError, ValueError) as exc:
                handle.close()
                rt.last_error = f"启动失败: {exc}"
                return {"ok": False, "error": rt.last_error}

            rt.proc = proc
            rt.log_handle = handle
            rt.log_path = log_path
            rt.started_at = time.time()
            rt.exited_at = None
            rt.exit_code = None
            rt.last_error = None
            rt.stopping = False
            rt.stopped_by_us = False
            rt.starts += 1
            log.info("已启动 %s (pid=%s)：%s", name, proc.pid, spec.label())
            return {"ok": True, "pid": proc.pid, "log": str(log_path)}

    async def stop(self, name: str, *, timeout_s: float | None = None) -> dict[str, Any]:
        spec = self._specs.get(name)
        if spec is None:
            return {"ok": False, "error": f"进程 {name} 未配置（无法停止）"}
        timeout = self._stop_timeout_s if timeout_s is None else timeout_s
        async with self._lock:
            rt = self._runtime[name]
            proc = rt.proc
            if proc is None or proc.returncode is not None:
                rt.proc = None
                return {"ok": True, "already_stopped": True}
            rt.stopping = True
            rt.stops += 1
            pid = proc.pid
            killed = await _terminate(proc, timeout)
            rt.exit_code = proc.returncode
            rt.exited_at = time.time()
            rt.proc = None
            rt.stopping = False
            rt.stopped_by_us = True
            self._close_log(rt)
            log.info("已停止 %s (pid=%s, 强制=%s)", name, pid, killed)
            return {"ok": True, "pid": pid, "forced": killed}

    async def stop_all(self) -> None:
        for name in list(self._specs):
            if self.running(name):
                try:
                    await self.stop(name)
                except Exception as exc:  # noqa: BLE001
                    log.warning("停止 %s 失败：%s", name, exc)

    # ── 内部 ────────────────────────────────────────────────────────────────
    @staticmethod
    def _close_log(rt: ProcessRuntime) -> None:
        handle, rt.log_handle = rt.log_handle, None
        if handle is not None:
            try:
                handle.close()
            except OSError:
                pass


async def _terminate(proc: asyncio.subprocess.Process, timeout_s: float) -> bool:
    """SIGTERM 整组 → 等待 → SIGKILL；返回是否用了强杀。"""
    _signal_group(proc, signal.SIGTERM)
    try:
        await asyncio.wait_for(proc.wait(), timeout=timeout_s)
        return False
    except asyncio.TimeoutError:
        log.warning("pid=%s 未在 %.1fs 内退出，SIGKILL", proc.pid, timeout_s)
        _signal_group(proc, signal.SIGKILL)
        try:
            await asyncio.wait_for(proc.wait(), timeout=2.0)
        except asyncio.TimeoutError:
            log.error("pid=%s 强杀后仍未回收", proc.pid)
        return True


def _signal_group(proc: asyncio.subprocess.Process, sig: signal.Signals) -> None:
    if proc.returncode is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.send_signal(sig)
        except (ProcessLookupError, OSError):
            pass

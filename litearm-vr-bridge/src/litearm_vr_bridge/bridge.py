"""服务装配：把收帧、转发、状态、兜底、进程启停拼成一个进程。

数据流（DESIGN 2.1）：

    头显 --ws--> HeadsetServer --原样--> TeleopPublisher --zenoh--> teleop
                     │
                     └── 统计 + TeleopGovernor ──┬──> FollowClient (start/hold/latch/stop/ping)
                                                 └──> StatusHub --ws--> litearm-studio

转发路径上只做两件小事：解析出 timestamp / gripActive（统计与兜底判据），
然后把**原始字符串**发出去。日志、事件、进程状态都在旁路。
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, Callable

from . import __version__
from .config import Config
from .events import EventLog
from .follow import FollowClient
from .frames import parse_frame
from .governor import Command, TeleopGovernor
from .headset import HeadsetServer
from .metrics import HeadsetMetrics, LatencyEstimator
from . import protocol
from .session import close_session, describe, open_session
from .studio import StudioLink
from .teleop import TeleopPublisher
from .supervisor import ProcessSupervisor

log = logging.getLogger("litearm_vr_bridge")

__all__ = ["Bridge"]

PROCESS_NAMES = ("teleop", "follow_server")
# "following" = SDK 版 ArmState；"MOVE_JS"/"MOVE_MIT"/"MIT_ALL" = stm32 版逐帧跟随的 mode_name
MANAGED_ARM_STATES = ("following", "MOVE_JS", "MOVE_MIT", "MIT_ALL")


class Bridge:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.started_mono = time.monotonic()
        self.started_wall_ms = int(time.time() * 1000)

        self.events = EventLog(ttl_ms=cfg.event_ttl_ms)
        self.metrics = HeadsetMetrics(latency=LatencyEstimator())
        self.governor = TeleopGovernor(
            warn_ms=cfg.frame_warn_ms,
            fallback_ms=cfg.frame_fallback_ms,
            fallback_enabled=cfg.fallback_enabled,
        )
        # zenoh 会话在 start() 里开（`--doctor` 和构造 Config 都不该碰网络）；
        # 帧发布器与状态口共用这一条会话。
        self.session: Any = None
        self.zenoh_error: str | None = None
        self.forwarder = TeleopPublisher(protocol.frame_topic(cfg.zenoh_arm_id))
        self.follow = FollowClient(cfg.follow_host, cfg.follow_port)
        self.supervisor = ProcessSupervisor(
            cfg.processes,
            log_dir=cfg.process_log_dir,
            on_exit=self._on_process_exit,
        )
        self.hub = StudioLink(
            self.snapshot,
            self.handle_action,
            status_topic=protocol.status_topic(cfg.zenoh_arm_id),
            rpc_topic=protocol.rpc_topic(cfg.zenoh_arm_id),
            status_hz=cfg.status_hz,
        )
        # 8765 从今只服务头显；上位机走 zenoh。`/healthz` 留在同一个口上 ——
        # zenoh 挂了的时候它是唯一的带外通道。
        self.headset = HeadsetServer(
            cfg.bridge_host,
            cfg.bridge_port,
            on_frame=self._on_headset_frame,
            health_provider=self.health,
        )

        # 默认透传：桥存在的意义是垫片，不改变原有链路行为；
        # teleop.stop / estop 会关上它（并 latch）。
        self._forward_enabled = True
        self._pump_event = asyncio.Event()
        self._watchdog_s = 0.05
        self._closing = False
        self._tasks: list[asyncio.Task[None]] = []
        self._fallback_hold_ok = False
        self._last_hold_try_mono = 0.0
        self._forward_blocked_total = 0

    # ── 生命周期 ────────────────────────────────────────────────────────────
    async def start(self) -> None:
        self._log_startup()
        # 头显口与看门狗**先**起来，zenoh 放后面：会话开不起来（zenoh 没装、端口被占）
        # 也不能推迟收帧和掉线兜底这两条安全路径。
        await self.headset.start()
        self._tasks = [
            asyncio.create_task(self._watchdog_loop(), name="watchdog"),
            asyncio.create_task(self._ping_loop(), name="ping"),
        ]
        await self._start_zenoh()
        if not self.cfg.fallback_enabled:
            log.warning("兜底已关闭（FALLBACK_ENABLED=0）：无帧只会告警，不会自动 hold")
        if not self.cfg.processes:
            log.warning("没有受管进程：要么没 init submodule（git submodule update --init）、"
                        "要么 *_CMD 被显式留空 —— process.* 动作将返回失败")
        if self.cfg.legacy_env:
            # 认是认了（见 config.LEGACY_ENV_PREFIX），但得让人知道该改
            log.warning(
                "配置里还在用旧变量名 %s：本次已按兼容处理，请改成 TELEOP_* "
                "（`--doctor` 也会提示；旧名字将来会删）",
                "、".join(self.cfg.legacy_env),
            )

    async def _start_zenoh(self) -> None:
        """开会话、起帧发布器。

        **失败不抛。** zenoh 起不来（没装、端口被占、router 连不上）时，桥仍要收帧、
        要看门狗、要掉线兜底 —— 那三条是安全路径，不能因为"上位机看不到"就一起停。
        这时 `snapshot()["zenoh"]["ready"]` 是 false，`/healthz` 也照实报。
        """
        loop = asyncio.get_running_loop()
        try:
            self.session = await asyncio.to_thread(open_session, self.cfg)
        except Exception as exc:  # noqa: BLE001
            self.zenoh_error = f"{type(exc).__name__}: {exc}"
            log.error("zenoh 会话开不起来：%s —— 帧不会发给 teleop，"
                      "上位机也看不到状态（其余功能不受影响）", self.zenoh_error)
            return
        self.forwarder.start(
            self.session,
            # 回调来自发布线程，切回事件循环 —— 计数器归 HeadsetMetrics 一个主人
            on_sent=lambda cost: loop.call_soon_threadsafe(self.metrics.on_forwarded, cost),
            on_dropped=lambda: loop.call_soon_threadsafe(self.metrics.on_dropped),
        )
        log.info("帧发布：%s（等 teleop 订阅）",
                 protocol.frame_topic(self.cfg.zenoh_arm_id))
        self.hub.start(self.session, loop)

    async def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        log.info("正在关闭 ...")

        # 关之前先把机械臂按住：正在跟随时发一次 latch（尽力而为，别拖太久）
        if self.governor.following:
            try:
                await asyncio.wait_for(self.follow.latch(), timeout=1.5)
                self._add_event("warn", "shutdown_latch",
                                "桥退出前下发 latch：机械臂就地持位")
            except Exception as exc:  # noqa: BLE001
                log.warning("退出前 latch 失败：%s", exc)

        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            try:
                await task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
        self._tasks = []

        await self.headset.close()
        await asyncio.to_thread(self.hub.stop)
        # 停线程 / 关会话都是同步阻塞的，扔到线程里做，别卡住事件循环的收尾
        await asyncio.to_thread(self.forwarder.stop)
        await asyncio.to_thread(close_session, self.session)
        self.session = None
        await self.follow.close()

        if self.cfg.stop_processes_on_exit:
            await self.supervisor.stop_all()
        log.info("已关闭")

    # ── 收帧路径 ────────────────────────────────────────────────────────────
    async def _on_headset_frame(self, raw: str, now_mono: float, now_wall_ms: float) -> None:
        info = parse_frame(raw)
        self.metrics.on_frame(now_mono, now_wall_ms, info)
        self.governor.on_frame(now_mono, info)

        if self._forward_enabled:
            # 只入队，不等网络（网络在 zenoh-pub 线程上）。成败由回调记账，
            # 见 `_on_frame_sent` / `_on_frame_dropped`。
            self.forwarder.offer(raw)
        else:
            self._forward_blocked_total += 1
            self.metrics.on_blocked()

        # 命令下发走旁路任务，不占转发路径
        self._pump_event.set()

    # ── 旁路任务 ────────────────────────────────────────────────────────────
    async def _watchdog_loop(self) -> None:
        while not self._closing:
            try:
                await asyncio.wait_for(self._pump_event.wait(), timeout=self._watchdog_s)
            except asyncio.TimeoutError:
                pass
            self._pump_event.clear()
            now_mono = time.monotonic()
            try:
                self.governor.tick(now_mono)
                await self._pump_governor(now_mono)
                await self._ensure_fallback_hold(now_mono)
            except Exception as exc:  # noqa: BLE001
                log.error("看门狗循环异常：%s", exc)
            self.supervisor.refresh()

    async def _ping_loop(self) -> None:
        while not self._closing:
            started = time.monotonic()
            try:
                await self.follow.ping()
            except Exception as exc:  # noqa: BLE001
                log.debug("ping 异常：%s", exc)
            elapsed = time.monotonic() - started
            await asyncio.sleep(max(self.cfg.ping_interval_s - elapsed, 0.05))

    async def _pump_governor(self, now_mono: float) -> None:
        self._flush_alarms()
        await self._flush_commands(now_mono)

    def _flush_alarms(self) -> None:
        """把状态机的告警落成事件：同步、无 await，保证与状态翻转同时生效。"""
        for alarm in self.governor.drain_alarms():
            self._add_event(alarm.level, alarm.code, alarm.message, alarm.data)

    async def _flush_commands(self, now_mono: float) -> list[dict[str, Any]]:
        """把排队的命令发出去，返回每一条的结果（动作回执要用）。"""
        results: list[dict[str, Any]] = []
        for command in self.governor.drain_commands():
            results.append(await self._run_command(command, now_mono))
        return results

    async def _run_command(self, command: Command, now_mono: float) -> dict[str, Any]:
        action = command.action
        if action == "hold":
            self._last_hold_try_mono = now_mono
        # 安全路径（掉线兜底）用更短的超时：跟随服务不通时也不能把看门狗按在地上，
        # 否则 tick / 命令泵会被一个 3s 的请求超时堵住。
        timeout_s = 1.0 if command.reason.startswith("no_frame") else None
        try:
            reply = await self._send_follow(action, timeout_s=timeout_s)
        except Exception as exc:  # noqa: BLE001
            reply = None
            log.warning("下发 %s 异常：%s", action, exc)

        ok = isinstance(reply, dict) and bool(reply.get("ok"))
        if action == "hold" and self.governor.fallback_active:
            self._fallback_hold_ok = ok
        data = {"action": action, "reason": command.reason, "ok": ok}
        if isinstance(reply, dict):
            data["reply"] = reply
        level = "info" if ok else "danger"
        code = f"{action}_done" if ok else f"{action}_failed"
        verb = {"hold": "就地保持（不 movej）", "latch": "就地持位",
                "stop": "急停", "start": "开始跟随"}.get(action, action)
        message = (
            f"{verb}指令已下发（{command.reason}）"
            if ok
            else f"{verb}指令下发失败（{command.reason}）：跟随服务 {self.cfg.follow_endpoint} 不可达或拒绝"
        )
        self._add_event(level, code, message, data)
        return data

    async def _ensure_fallback_hold(self, now_mono: float) -> None:
        """兜底 hold 没成功就重试（安全动作，直到跟随服务确认）。"""
        if not self.governor.fallback_active:
            self._fallback_hold_ok = False
            return
        if self._fallback_hold_ok:
            return
        if now_mono - self._last_hold_try_mono < 1.0:
            return
        self._last_hold_try_mono = now_mono
        reply = await self._send_follow("hold", timeout_s=1.0)
        self._fallback_hold_ok = isinstance(reply, dict) and bool(reply.get("ok"))
        if self._fallback_hold_ok:
            self._add_event("warn", "fallback_hold_retry",
                            "兜底 hold 重试成功：机械臂已就地保持", {"reply": reply})
        else:
            log.error("兜底 hold 重试失败：跟随服务 %s 无响应", self.cfg.follow_endpoint)

    async def _send_follow(
        self, action: str, *, timeout_s: float | None = None
    ) -> dict[str, Any] | None:
        if action == "hold":
            return await self.follow.hold(timeout_s=timeout_s or 3.0)
        if action == "latch":
            return await self.follow.latch(timeout_s=timeout_s or 3.0)
        if action == "stop":
            return await self.follow.stop(timeout_s=timeout_s or 2.0)
        if action == "start":
            return await self.follow.start(timeout_s=timeout_s or 3.0)
        raise ValueError(f"不支持的跟随指令 {action!r}")

    # ── 动作接口（DESIGN 4.2） ──────────────────────────────────────────────
    async def handle_action(self, message: dict[str, Any]) -> dict[str, Any]:
        """处理一条动作，返回回执。

        回执**同时**照旧写进快照的 `events` —— 上位机界面读的是那边，`ctl` 的码表也读
        那边；query 的回执只是让"这条命令到底送到没有、下游接没接"当场就有答案。
        两条都不能少：events 是持久的、谁都能看见的，回执是即时的、只给发起方。
        """
        action = str(message.get("action") or "")
        now_mono = time.monotonic()
        log.info("收到动作：%s %s", action, {k: v for k, v in message.items() if k != "action"})

        if self._closing:
            # 退出途中收到新动作：如实回绝，别让它排进队列里没人管
            return _receipt_error("ShuttingDown", "桥正在退出，不接受新动作")

        if action == "teleop.start":
            self._forward_enabled = True
            # 先把进程拉起来，再让 governor 产生 start 命令 —— 顺序不能反：
            # on_teleop_start 会把 start 排进命令队列，而看门狗（旁路，50ms 一轮）会在
            # 下面还在 await 的时候就把命令抽走发出去。真机上 follow_server 要等 STM32
            # 状态帧、几秒后才 listen，那条 start 会直接打在没监听的端口上丢掉
            # （实测过：start_failed，跟随服务日志里从头到尾没有 start）。
            started = await self._ensure_processes()
            self.governor.on_teleop_start(now_mono)
            # 状态与事件先一起落地，别让中间的快照出现"已开始但没事件"
            self._flush_alarms()
            self._add_event(
                "info", "teleop_started",
                "已开始 VR 遥操：转发打开，等待头显 grip",
                {"forward_enabled": True},
            )
            if started:
                await self._wait_for_teleop(2.5)
            return _receipt("teleop_started", await self._flush_commands(now_mono))

        if action == "teleop.stop":
            # 顺序要求（DESIGN 6.3）：先不发帧，再 latch，避免"先松手再持位"的空窗
            self._forward_enabled = False
            self.governor.on_teleop_stop(now_mono)
            self._flush_alarms()
            self._add_event(
                "info", "teleop_stopped",
                "已停止 VR 遥操：转发已关闭，并已请求就地持位",
            )
            return _receipt("teleop_stopped", await self._flush_commands(now_mono))

        if action == "estop":
            self._forward_enabled = False
            self.governor.on_estop(now_mono)
            self._flush_alarms()
            # 急停的回执必须带上**跟随服务接没接受** —— 只说"桥听见了"没用，
            # 要停的是机械臂。这就是当初选 query 而不是 pub 的原因。
            return _receipt("estop_sent", await self._flush_commands(now_mono))

        if action in ("process.start", "process.stop"):
            names = self._action_process_names(message)
            if not names:
                self._add_event("warn", "action_failed", "process.* 需要 name/processes 参数")
                return _receipt_error("BadRequest", "process.* 需要 name/processes 参数")
            results = []
            for name in names:
                if action == "process.start":
                    results.append(await self._start_process(name))
                else:
                    results.append(await self._stop_process(name))
            ok = all(r.get("ok") for r in results)
            code = "process_started" if action == "process.start" else "process_stopped"
            if not ok:
                return _receipt_error("ProcessFailed", f"{'启动' if action == 'process.start' else '停止'}失败",
                                      {"processes": results}, code=f"{code}_failed")
            return _receipt(code, [], {"processes": results})

        if action in ("event.ack", "event.clear"):
            event_id = message.get("id")
            removed = self.events.clear(
                int(event_id) if isinstance(event_id, (int, float, str)) and str(event_id).isdigit()
                else None
            )
            self._add_event("info", "events_cleared", f"已清空 {removed} 条事件")
            return _receipt("events_cleared", [], {"removed": removed})

        if action in ("ping", "status", "snapshot"):
            self._add_event("info", "pong", "桥在线", {"bridge_version": __version__})
            # 顺手把当前快照回过去：动作通道通不通，不用等下一拍状态就能确认
            return _receipt("pong", [], {"bridge_version": __version__,
                                        "snapshot": self.snapshot()})

        self._add_event("warn", "unknown_action", f"未知动作：{action or '(空)'}")
        return _receipt_error("UnknownAction", f"未知动作：{action or '(空)'}")

    @staticmethod
    def _action_process_names(message: dict[str, Any]) -> list[str]:
        raw: list[Any] = []
        for key in ("name", "process", "processes"):
            value = message.get(key)
            if isinstance(value, str):
                raw.append(value)
            elif isinstance(value, (list, tuple)):
                raw.extend(value)
        names: list[str] = []
        for item in raw:
            if isinstance(item, str) and item and item not in names:
                names.append(item)
        return names

    async def _ensure_processes(self) -> list[str]:
        """teleop.start 时把链路需要的进程拉起来（只在配了命令时生效）。"""
        started: list[str] = []
        # 先跟随服务，后 teleop：teleop 首次按 grip 会 movej 回 q_home 再跟随真机，
        # 而它的真机客户端一启动就去连 9001 —— 所以要等 9001 真的在听了再拉 teleop。
        for name in ("follow_server", "teleop"):
            if not self.supervisor.configured(name) or self.supervisor.running(name):
                continue
            reply = await self._start_process(name)
            if not reply.get("ok"):
                continue
            started.append(name)
            if name == "follow_server":
                await self._wait_for_port(
                    self.cfg.follow_host,
                    self.cfg.follow_port,
                    8.0,
                    alive=lambda: self.supervisor.running("follow_server"),
                )
        return started

    async def _wait_for_port(
        self,
        host: str,
        port: int,
        timeout_s: float,
        *,
        alive: Callable[[], bool] | None = None,
    ) -> bool:
        """等某个 TCP 端口能连上；连不上也只是告警，不拦后续启动。

        探针连上就断，跟随服务会为此打一行"客户端接入/断开"——它本来就不因客户端断开
        改变跟随状态（见 follow.py 模块注释），所以是无害的。

        `alive` 用来在进程已经退出时提前放弃，别白等满超时。
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if alive is not None and not alive():
                log.error("跟随服务进程已退出，不再等 %s:%d", host, port)
                return False
            try:
                _, writer = await asyncio.wait_for(
                    asyncio.open_connection(host, port), timeout=0.5
                )
            except (OSError, asyncio.TimeoutError):
                await asyncio.sleep(0.1)
                continue
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:  # noqa: BLE001
                pass
            return True
        log.warning("跟随服务 %s:%d 在 %.1fs 内没就绪，仍继续启动 teleop",
                    host, port, timeout_s)
        return False

    async def _start_process(self, name: str) -> dict[str, Any]:
        spec = self.cfg.processes.get(name)
        if spec is not None and "--real" in spec.argv:
            self._add_event(
                "warn", "teleop_real_warning",
                "启动 teleop 已接真机桥接（--real）：首次按 grip 会 movej 回 q_home 并跟随真机，"
                "请确认机械臂已上电、工作区无人、急停在手",
                {"process": name},
            )
        reply = await self.supervisor.start(name)
        ok = bool(reply.get("ok"))
        self._add_event(
            "info" if ok else "danger",
            "process_started" if ok else "process_start_failed",
            (f"进程 {name} 已启动" if ok else f"进程 {name} 启动失败：{reply.get('error')}"),
            {"process": name, **{k: v for k, v in reply.items() if k != "ok"}},
        )
        return reply

    async def _stop_process(self, name: str) -> dict[str, Any]:
        reply = await self.supervisor.stop(name)
        ok = bool(reply.get("ok"))
        self._add_event(
            "info" if ok else "danger",
            "process_stopped" if ok else "process_stop_failed",
            (f"进程 {name} 已停止" if ok else f"进程 {name} 停止失败：{reply.get('error')}"),
            {"process": name, **{k: v for k, v in reply.items() if k != "ok"}},
        )
        return reply

    async def _wait_for_teleop(self, timeout_s: float) -> bool:
        """等 teleop 订上帧话题。

        pub/sub 没有"连接"事件，但发布者知道自己有没有匹配的订阅者 —— 这就是
        `TeleopForwarder.connected` 的等价物。这一步不能省：帧发出去没人接，
        而跟随服务那边已经收到 `start` 了，正是 commit 6871570 修的那个抢跑。
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.forwarder.subscribed:
                return True
            await asyncio.sleep(0.1)
        return self.forwarder.subscribed

    def _on_process_exit(self, name: str, code: int | None, expected: bool) -> None:
        if expected:
            self._add_event("info", "process_exited", f"进程 {name} 已退出（code={code}）")
        else:
            self._add_event(
                "danger", "process_crashed",
                f"进程 {name} 意外退出（code={code}）；本服务不会自动重启它",
                {"process": name, "exit_code": code},
            )

    # ── 状态快照（DESIGN 4.1） ──────────────────────────────────────────────
    def snapshot(self) -> dict[str, Any]:
        now_mono = time.monotonic()
        now_wall_ms = int(time.time() * 1000)
        metrics = self.metrics
        age = metrics.age_ms(now_mono)
        online = age is not None and age <= self.cfg.frame_warn_ms
        latency = metrics.latency_ms()
        arm_state = self.follow.state
        owner = "vr" if (self.governor.following or arm_state in MANAGED_ARM_STATES) else "none"

        processes: dict[str, str] = {}
        for name in PROCESS_NAMES:
            if self.supervisor.configured(name):
                processes[name] = self.supervisor.statuses().get(name, "unknown")
            else:
                processes[name] = "unknown"

        return {
            "ts": now_wall_ms,
            "bridge": {
                "online": True,
                "version": __version__,
                "uptime_s": round(now_mono - self.started_mono, 1),
                "forward_cost_ms": _round(metrics.forward_cost.mean),
                "forward_cost_last_ms": _round(metrics.forward_cost.last),
                "forward_cost_max_ms": _round(metrics.forward_cost.max),
                "clients": {
                    "headset": self.headset.clients,
                    "status": self.hub.clients,
                },
                "teleop_connected": self.forwarder.subscribed,
            },
            "headset": {
                "online": online,
                "connected": self.headset.headset_connected,
                "fps": round(metrics.fps(), 1),
                "latency_ms": _round(latency),
                "latency_estimated": metrics.latency.estimated,
                "last_frame_age_ms": _round(age, ndigits=1),
                "following": self.governor.following,
                "grip": self.governor.grip_held,
                "tracked": bool(metrics.last_info and metrics.last_info.tracked),
                "frames": metrics.frames,
                "bad_frames": metrics.bad_frames,
                "frames_forwarded": metrics.frames_forwarded,
                "frames_dropped": metrics.frames_dropped,
                "frames_blocked": metrics.frames_blocked,
                "last_frame_ts": (
                    int(metrics.last_info.ts_ms)
                    if metrics.last_info and metrics.last_info.ts_ms
                    else None
                ),
            },
            "arm": {
                "source": self.cfg.arm_source,
                "q": self.follow.q,
                "state": arm_state,
                "fault": self.follow.fault,
                "connected": self.follow.connected,
                "last_ok_age_ms": _round(self.follow.last_ok_age_ms(now_mono), ndigits=1),
                "error": self.follow.last_error,
            },
            "processes": processes,
            "processes_detail": self.supervisor.details(),
            "owner": owner,
            "fallback": {
                "active": self.governor.fallback_active,
                "reason": self.governor.fallback_reason,
                "guard_enabled": self.cfg.fallback_enabled,
            },
            "link": {
                "state": self.governor.state.value,
                "enabled": self.governor.enabled,
                "forward_enabled": self._forward_enabled,
                "rearm_required": self.governor.rearm_required,
                "warn_ms": self.cfg.frame_warn_ms,
                "fallback_ms": self.cfg.frame_fallback_ms,
            },
            # zenoh 那一段的自述也进快照：上位机装的是"我这条订阅通不通"，
            # 而它看不到自己的对端 —— 桥这边替它说清楚。
            "zenoh": self._zenoh_state(),
            "events": self.events.drain(now_wall_ms=now_wall_ms),
        }

    def health(self) -> dict[str, Any]:
        now_mono = time.monotonic()
        age = self.metrics.age_ms(now_mono)
        return {
            "ok": True,
            "version": __version__,
            "ts": int(time.time() * 1000),
            "uptime_s": round(now_mono - self.started_mono, 1),
            "headset_online": bool(age is not None and age <= self.cfg.frame_warn_ms),
            "following": self.governor.following,
            "state": self.governor.state.value,
            "fallback": self.governor.fallback_active,
            "teleop_connected": self.forwarder.subscribed,
            "arm_source": self.cfg.arm_source,
            "processes": self.supervisor.statuses(),
            "zenoh": self._zenoh_state(),
        }

    def _zenoh_state(self) -> dict[str, Any]:
        """zenoh 那一段的自述，进快照也进 `/healthz`。

        `/healthz` 是 zenoh 挂了以后**唯一还活着**的带外通道（它走头显那个 HTTP 口），
        所以这段必须如实 —— 不然"桥活着但 zenoh 没起来"这件事只有在能看 zenoh 的
        时候才看得见，而那正是看不见的时候。
        """
        return {
            "ready": self.session is not None,
            "arm_id": self.cfg.zenoh_arm_id,
            "frame_topic": protocol.frame_topic(self.cfg.zenoh_arm_id),
            "status_topic": protocol.status_topic(self.cfg.zenoh_arm_id),
            "rpc_topic": protocol.rpc_topic(self.cfg.zenoh_arm_id),
            "teleop_subscribed": self.forwarder.subscribed,
            "queue_pending": self.forwarder.pending,
            "studio": self.hub.state(),
            "error": self.zenoh_error,
        }

    # ── 内部 ────────────────────────────────────────────────────────────────
    def _add_event(
        self,
        level: str,
        code: str,
        message: str,
        data: dict[str, Any] | None = None,
    ) -> None:
        event = self.events.add(
            level, code, message, data,
            now_wall_ms=int(time.time() * 1000),
            now_mono=time.monotonic(),
        )
        log_fn = {"danger": log.error, "warn": log.warning}.get(level, log.info)
        log_fn("[%s] %s", code, message)

    def _log_startup(self) -> None:
        cfg = self.cfg
        log.info("litearm-vr-bridge %s 启动", __version__)
        log.info("  头显入口      ws://%s:%d", cfg.bridge_host, cfg.bridge_port)
        log.info("  zenoh         %s", describe(cfg))
        log.info("  帧话题        %s（等 teleop 订阅；队满丢帧并计数）",
                 protocol.frame_topic(cfg.zenoh_arm_id))
        log.info("  状态/动作话题 %s / %s",
                 protocol.status_topic(cfg.zenoh_arm_id),
                 protocol.rpc_topic(cfg.zenoh_arm_id))
        log.info("  跟随服务      %s（start/hold/latch/stop/ping）", cfg.follow_endpoint)
        log.info("  阈值          告警 %.0f ms / 兜底 hold %.0f ms（%s）",
                 cfg.frame_warn_ms, cfg.frame_fallback_ms,
                 "开" if cfg.fallback_enabled else "关")
        log.info("  真机状态来源  %s", cfg.arm_source)
        for name, spec in cfg.processes.items():
            log.info("  受管进程      %s → %s", name, spec.label())
        log.warning("本服务不做逆解、不驱动机械臂；兜底只会下发 hold/stop，绝不 movej/home")


def _receipt(
    code: str, commands: list[dict[str, Any]], result: dict[str, Any] | None = None
) -> dict[str, Any]:
    """动作回执的形状，对齐 `litearm-server` 的 `RpcReply`（多一个 `code` 便于码表）。

    `commands` 里是这次动作实际下发给跟随服务的每一条及其结果 —— **`ok` 取自它们**，
    不是"桥收到了就算成功"。急停要停的是机械臂，回执得说到那一步。
    """
    payload: dict[str, Any] = {"commands": commands} if commands else {}
    if result:
        payload.update(result)
    if all(c.get("ok") for c in commands):
        return {"ok": True, "code": code, "result": payload}
    return {
        "ok": False,
        "code": f"{code}_failed",
        "result": payload,
        "error": {
            "type": "UpstreamRefused",
            "message": "跟随服务不可达或拒绝",
            "details": payload,
        },
    }


def _receipt_error(
    code: str, message: str, details: dict[str, Any] | None = None
) -> dict[str, Any]:
    """动作回执（没走到下游就失败了：参数不对、未知动作、正在退出）。"""
    return {
        "ok": False,
        "code": code,
        "error": {"type": code, "message": message, "details": details or {}},
    }


def _round(value: float | None, ndigits: int = 3) -> float | None:
    if value is None:
        return None
    return round(float(value), ndigits)

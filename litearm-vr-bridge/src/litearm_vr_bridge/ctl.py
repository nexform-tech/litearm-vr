"""给运行中的桥发动作：快捷启动的"扳机"（`--ctl up|down|status`）。

为什么需要它：桥的受管进程（teleop / follow_server）**只在收到 `teleop.start` 时才被
拉起**（见 `bridge.py` 的 `_ensure_processes`），而 `litearm-studio` 的 VR 页已按
AGENTS.md 删除 —— 现场没有任何东西能按这个按钮。这里用桥的 zenoh 接口补上。

三个动作：
  * `up`     `teleop.start`：打开帧转发，按配置拉起 follow_server + teleop
  * `down`   `teleop.stop`（先关转发再 latch 就地持位）→ `process.stop` 逐个收掉上游
  * `status` 收一帧快照，打印 docs/DEPLOY.md §4 那张判读表的字段

**它不需要 zenoh router。** `ctl` 跑在桥同一台机器上，用 client 模式直连桥的监听端点
（默认 `tcp/127.0.0.1:7447`）—— 上位机那套（router + 浏览器）挂了，现场这把扳机照样
能用。这是它存在的意义，别改成"绕一圈连 router"。

动作走 query，所以"桥不可达"不再靠猜：**收不到回执就是桥/router 没了，收到
`ok:false` 就是桥听见了但拒绝**。回执的同时，动作结果照旧写进快照的 `events`，
人看的那份明细还是从那儿来。
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import queue
from dataclasses import dataclass, field
from typing import Any, AsyncIterator

from .config import Config
from .protocol import rpc_topic, status_topic

__all__ = ["Result", "run_ctl", "status_lines", "bridge_endpoint"]

_WILDCARD_HOSTS = ("0.0.0.0", "::", "", "*")

# 兜底相关的时间上限；--ctl-timeout 可覆盖
_TIMEOUTS = {"up": 12.0, "down": 12.0, "status": 4.0}
# teleop.start 之后，跟随服务 start 指令的三种收场
_START_RESULTS = {"start_done", "start_failed", "teleop_start_needs_regrip"}

_LATCH_TIMEOUT_S = 5.0
_PROCESS_STOP_TIMEOUT_S = 5.0


@dataclass
class Result:
    code: int
    lines: list[str] = field(default_factory=list)


class BridgeUnreachable(RuntimeError):
    def __init__(self, url: str, cause: BaseException) -> None:
        super().__init__(f"{url}（{type(cause).__name__}: {cause}）")
        self.url = url
        self.cause = cause


def bridge_endpoint(cfg: Config) -> str:
    """桥的 zenoh 监听端点 —— `ctl` 直连它，不经过 router。

    桥监听在 0.0.0.0 时，本机连 127.0.0.1 而不是 0.0.0.0（后者不是个可连的地址）。
    """
    listen = cfg.zenoh_listen_endpoints[0] if cfg.zenoh_listen_endpoints else "tcp/0.0.0.0:7447"
    proto, _, addr = listen.partition("/")
    host, _, port = addr.rpartition(":")
    if host in _WILDCARD_HOSTS:
        host = "127.0.0.1"
    return f"{proto or 'tcp'}/{host}:{port}"


# ── 入口 ────────────────────────────────────────────────────────────────────


async def run_ctl(action: str, cfg: Config, *, timeout_s: float | None = None) -> Result:
    if action == "up":
        return await ctl_up(cfg, timeout_s or _TIMEOUTS["up"])
    if action == "down":
        return await ctl_down(cfg, timeout_s or _TIMEOUTS["down"])
    if action == "status":
        return await ctl_status(cfg, timeout_s or _TIMEOUTS["status"])
    return Result(2, [f"未知动作：{action}（可选 up / down / status）"])


async def ctl_up(cfg: Config, timeout_s: float) -> Result:
    """`teleop.start`：打开转发 + 按配置拉起上游进程。"""
    try:
        async with _session(cfg) as ws:
            events = _Events(await _snapshot(ws, 3.0))
            await ws.send(json.dumps({"action": "teleop.start"}, ensure_ascii=False))

            def done(snap: dict) -> bool:
                codes = events.codes()
                if codes & {"process_start_failed", "process_crashed"}:
                    return True
                if "teleop_started" not in codes:
                    return False
                # 必须等到 start 命令有结果：进程"running"只说明拉起来了，
                # 而 start 是随后才下发的（bridge.handle_action 里的顺序）。
                if not codes & _START_RESULTS:
                    return False
                if not cfg.processes:
                    return True
                statuses = snap.get("processes") or {}
                return all(statuses.get(name) == "running" for name in cfg.processes)

            snap = await _poll(ws, events, done, timeout_s)
            return _up_result(cfg, snap, events)
    except BridgeUnreachable as exc:
        return _unreachable(exc)


async def ctl_down(cfg: Config, timeout_s: float) -> Result:
    """`teleop.stop`（latch）→ `process.stop` 逐个收掉受管进程。"""
    budget = max(timeout_s, 1.0)
    try:
        async with _session(cfg) as ws:
            events = _Events(await _snapshot(ws, 3.0))

            await ws.send(json.dumps({"action": "teleop.stop"}, ensure_ascii=False))
            await _await_event(
                ws, events, {"latch_done", "latch_failed"}, min(budget, _LATCH_TIMEOUT_S)
            )

            for name in ("teleop", "follow_server"):
                if name not in cfg.processes:
                    continue
                await ws.send(
                    json.dumps({"action": "process.stop", "name": name}, ensure_ascii=False)
                )
                await _await_event(
                    ws, events,
                    {"process_stopped", "process_stop_failed"},
                    min(budget, _PROCESS_STOP_TIMEOUT_S),
                    process=name,
                )

            snap = await _snapshot(ws, 2.0)
            return _down_result(cfg, snap, events)
    except BridgeUnreachable as exc:
        return _unreachable(exc)


async def ctl_status(cfg: Config, timeout_s: float) -> Result:
    """收一帧快照，打印判读字段 + 当前未过期的事件。"""
    try:
        async with _session(cfg) as ws:
            snap = await _snapshot(ws, timeout_s)
    except BridgeUnreachable as exc:
        return _unreachable(exc)
    if snap is None:
        return Result(1, ["连上了桥，但没收到快照"])

    lines = status_lines(snap)
    events = _notable(snap)
    if events:
        lines.append("事件   :")
        lines += ["         " + _format_event(ev) for ev in events]
    return Result(0, lines)


# ── 输出 ────────────────────────────────────────────────────────────────────


def status_lines(snap: dict[str, Any]) -> list[str]:
    head = snap.get("headset") or {}
    bridge = snap.get("bridge") or {}
    arm = snap.get("arm") or {}
    link = snap.get("link") or {}
    fallback = snap.get("fallback") or {}
    processes = snap.get("processes") or {}

    forward = "开" if link.get("forward_enabled") else "关"
    rearm = "  rearm=需重新按 grip" if link.get("rearm_required") else ""
    procs = "  ".join(f"{name}={state}" for name, state in processes.items()) or "—"

    return [
        f"链路   : state={link.get('state')}  owner={snap.get('owner')}  转发={forward}{rearm}",
        f"头显   : {'在线' if head.get('online') else '离线'}  "
        f"fps={_num(head.get('fps'))}  延迟={_num(head.get('latency_ms'), 'ms')}  "
        f"帧龄={_num(head.get('last_frame_age_ms'), 'ms')}",
        f"机械臂 : state={arm.get('state')}  来源={arm.get('source')}  "
        f"故障={arm.get('fault') or '无'}  q={'有' if arm.get('q') else '—'}",
        f"转发耗时: {_num(bridge.get('forward_cost_ms'), 'ms', 2)}"
        f"（本服务段，正常 <1ms）  teleop 连接={bridge.get('teleop_connected')}",
        f"进程   : {procs}",
        f"兜底   : {'生效中（' + str(fallback.get('reason')) + '）' if fallback.get('active') else '未生效'}",
    ]


def _up_result(cfg: Config, snap: dict | None, events: _Events) -> Result:
    if snap is None:
        return Result(1, ["发了 teleop.start，但没等到桥的快照：桥可能刚退出，看 journalctl -u litearm-vr-bridge"])

    codes = events.codes()
    lines = [_format_event(ev) for ev in events.all() if ev.get("code") in _UP_CODES]
    lines += status_lines(snap)

    statuses = snap.get("processes") or {}
    missing = [name for name in cfg.processes if statuses.get(name) != "running"]
    failed = bool(codes & {"process_start_failed", "process_crashed"})

    if not cfg.processes:
        lines.append("[提示] 本机没配受管进程：桥只负责转发，teleop / follow_server 请自己启动")
    elif missing and not failed:
        lines.append(f"[警告] 这些进程还没起来：{', '.join(missing)}")
    if "teleop_start_needs_regrip" in codes:
        lines.append("[提示] 要重新按一次 grip 才会恢复跟随（teleop 还握着上一次的锚点）")
    if "start_failed" in codes:
        lines.append(f"[警告] 跟随服务 {cfg.follow_endpoint} 没接受 start：链路没真正接上")
        failed = True

    return Result(1 if (failed or missing) else 0, lines)


def _down_result(cfg: Config, snap: dict | None, events: _Events) -> Result:
    orders = ("teleop_stopped", "latch_done", "latch_failed",
              "process_stopped", "process_stop_failed", "process_crashed")
    lines = [_format_event(ev) for ev in events.all() if ev.get("code") in orders]

    statuses = (snap or {}).get("processes") or {}
    still = [name for name in cfg.processes if statuses.get(name) == "running"]
    if still:
        lines.append(f"[警告] 还没停下来：{', '.join(still)}")
    if snap is not None:
        lines.append(f"进程   : " + ("  ".join(f"{k}={v}" for k, v in statuses.items()) or "—"))

    failed = bool(events.codes() & {"latch_failed", "process_stop_failed"})
    return Result(1 if (failed or still) else 0, lines)


_UP_CODES = {
    "teleop_started", "teleop_start_needs_regrip", "teleop_real_warning",
    "start_done", "start_failed",
    "process_started", "process_start_failed", "process_stopped",
    "process_stop_failed", "process_crashed",
}


def _unreachable(exc: BridgeUnreachable) -> Result:
    return Result(2, [
        f"没连上桥：{exc}",
        "桥没在跑 → 先 `./scripts/up.sh`（或 systemctl start litearm-vr-bridge）；"
        "端口上有别的东西 → 用 `--doctor` 看清是谁占了",
    ])


def _num(value: Any, unit: str = "", digits: int = 1) -> str:
    if value is None:
        return "—"
    try:
        return f"{float(value):.{digits}f}{unit}"
    except (TypeError, ValueError):
        return str(value)


def _format_event(ev: dict[str, Any]) -> str:
    level = str(ev.get("level") or "info")
    return f"[{level:<6}] {ev.get('message')}"


def _notable(snap: dict[str, Any], limit: int = 6) -> list[dict[str, Any]]:
    """快照里当前还没过期的告警（warn/danger），最多几条。"""
    out = [ev for ev in snap.get("events") or [] if ev.get("level") in ("warn", "danger")]
    return out[-limit:]


# ── 会话与等待 ──────────────────────────────────────────────────────────────


class _Bridge:
    """`ctl` 与桥之间的那条 zenoh 通道：收快照 + 发动作。

    对外刻意长得像原来那条 WebSocket（`recv` 给 JSON 文本、`send` 收 JSON 文本），
    这样上面的判读逻辑（`_Events` / `_poll` / `status_lines`）一行都不用改。
    """

    def __init__(self, session: Any, cfg: Config, timeout_s: float) -> None:
        self._session = session
        self._timeout_s = timeout_s
        self._rpc_topic = rpc_topic(cfg.zenoh_arm_id)
        # zenoh 的回调在自己的线程上，用 Queue 交到 asyncio 这边
        self._inbox: queue.Queue[str] = queue.Queue(maxsize=64)
        self._subscriber = session.declare_subscriber(
            status_topic(cfg.zenoh_arm_id), self._on_sample
        )

    def _on_sample(self, sample: Any) -> None:
        text = bytes(sample.payload).decode("utf-8", "replace")
        try:
            self._inbox.put_nowait(text)
        except queue.Full:
            with contextlib.suppress(queue.Empty):
                self._inbox.get_nowait()  # 整份覆盖式，旧的没意义
            with contextlib.suppress(queue.Full):
                self._inbox.put_nowait(text)

    async def recv(self, timeout_s: float) -> str | None:
        try:
            return await asyncio.to_thread(self._inbox.get, True, max(timeout_s, 0.05))
        except queue.Empty:
            return None

    async def send(self, text: str) -> dict[str, Any] | None:
        """发一条动作，等回执。

        返回 None = **桥没回**（不可达 / 挂了 / 卡死）；`{"ok": false}` = 桥听见了
        但拒绝。`BridgeUnreachable` 只在**完全没回执**时抛 —— 这正是用 query 而不是
        pub 换来的那条区分。
        """
        try:
            data = json.loads(text)
        except (ValueError, TypeError):
            return None
        return await asyncio.to_thread(self._call, data) if isinstance(data, dict) else None

    def _call(self, data: dict[str, Any]) -> dict[str, Any] | None:
        import zenoh

        replies = self._session.get(
            self._rpc_topic,
            payload=json.dumps(data, ensure_ascii=False).encode("utf-8"),
            timeout=self._timeout_s,
            consolidation=zenoh.QueryConsolidation(zenoh.ConsolidationMode.NONE),
        )
        for reply in replies:
            ok = getattr(reply, "ok", None)
            if ok is None:
                continue
            try:
                parsed = json.loads(bytes(ok.payload))
            except (ValueError, TypeError):
                continue
            if isinstance(parsed, dict):
                return parsed
        return None

    def close(self) -> None:
        with contextlib.suppress(Exception):
            self._subscriber.undeclare()


@contextlib.asynccontextmanager
async def _session(cfg: Config, *, timeout_s: float = 6.0) -> AsyncIterator[_Bridge]:
    """连上桥的 zenoh 监听端点（client 模式，**不经过 router**）。"""
    import zenoh

    endpoint = bridge_endpoint(cfg)
    zcfg = zenoh.Config()
    zcfg.insert_json5("mode", '"client"')
    zcfg.insert_json5("connect/endpoints", f'["{endpoint}"]')
    # 固定拓扑，别让发现把"连不上"变成"连上了别的什么"
    zcfg.insert_json5("scouting/multicast/enabled", "false")
    zcfg.insert_json5("scouting/gossip/enabled", "false")

    try:
        session = await asyncio.to_thread(zenoh.open, zcfg)
    except Exception as exc:  # noqa: BLE001 — 命令行工具：连不上就把原因照抄给使用者
        raise BridgeUnreachable(endpoint, exc) from exc

    bridge = _Bridge(session, cfg, timeout_s)
    # client 模式下 zenoh.open 立刻返回，真正的"连上"没有事件可等 —— 所以能不能
    # 连上只能靠"收不收得到东西"来判断。这也是 `_session` 默认超时给到 6s 的原因。
    if await bridge.recv(timeout_s) is None:
        bridge.close()
        with contextlib.suppress(Exception):
            await asyncio.to_thread(session.close)
        raise BridgeUnreachable(endpoint, TimeoutError(f"{timeout_s:.0f}s 内没收到快照"))
    try:
        yield bridge
    finally:
        bridge.close()
        with contextlib.suppress(Exception):
            await asyncio.to_thread(session.close)


class _Events:
    """只认"本轮动作之后"出现的事件：构造时先记下基线 id。"""

    def __init__(self, baseline: dict | None) -> None:
        self._base = {ev.get("id") for ev in (baseline or {}).get("events") or []}
        self._seen: dict[Any, dict] = {}

    def absorb(self, snap: dict) -> None:
        for ev in snap.get("events") or []:
            eid = ev.get("id")
            if eid in self._base or eid in self._seen:
                continue
            self._seen[eid] = ev

    def codes(self) -> set[str]:
        return {str(ev.get("code")) for ev in self._seen.values()}

    def all(self) -> list[dict]:
        return [self._seen[key] for key in sorted(self._seen)]  # 事件 id 是递增整数


async def _snapshot(ws: Any, timeout_s: float) -> dict | None:
    try:
        raw = await ws.recv(max(timeout_s, 0.05))
    except (asyncio.TimeoutError, OSError):
        return None
    if raw is None:
        return None
    try:
        payload = json.loads(raw)
    except (ValueError, TypeError):
        return None
    return payload if isinstance(payload, dict) else None


async def _poll(
    ws: Any,
    events: _Events,
    done: Any,
    timeout_s: float,
) -> dict | None:
    """读快照直到 done(snap) 为真或超时；返回最后一份快照。"""
    loop = asyncio.get_running_loop()
    deadline = loop.time() + max(timeout_s, 0.05)
    latest: dict | None = None
    while loop.time() < deadline:
        snap = await _snapshot(ws, deadline - loop.time())
        if snap is None:
            break  # 到点了，或者桥把连接断了
        latest = snap
        events.absorb(snap)
        if done(snap):
            return latest
    return latest


async def _await_event(
    ws: Any,
    events: _Events,
    codes: set[str],
    timeout_s: float,
    *,
    process: str | None = None,
) -> bool:
    def matched() -> bool:
        for ev in events.all():
            if ev.get("code") not in codes:
                continue
            if process is not None and (ev.get("data") or {}).get("process") != process:
                continue
            return True
        return False

    if matched():
        return True
    await _poll(ws, events, lambda _snap: matched(), timeout_s)
    return matched()

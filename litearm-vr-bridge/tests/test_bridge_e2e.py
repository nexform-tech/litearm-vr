"""端到端：假 teleop + 假跟随服务 + 假头显 + 假上位机。

覆盖 DESIGN 的验收点：
  * M1：头显帧原样、逐帧、按序到 teleop；快照字段可用（帧率 / 延迟 / 转发耗时）
  * M2：teleop.stop 先停转发再 latch；无帧 500ms 自动 hold 且幂等；恢复不自动跟随
  * 4.1/4.2：快照契约字段齐备；动作回执进 events；/healthz 可访问
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import sys
import tempfile
import unittest
import urllib.request
from pathlib import Path
from typing import Any, AsyncIterator, Callable

import websockets
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from litearm_vr_bridge.bridge import Bridge
from litearm_vr_bridge.config import Config
from litearm_vr_bridge.protocol import frame_topic, rpc_topic, status_topic

sys.path.insert(0, str(Path(__file__).resolve().parent))
from helpers import (  # noqa: E402
    FakeFollowServer,
    StatusWatcher,
    FakeTeleop,
    frame,
    free_port,
    zenoh_endpoint,
)


class _Stack:
    def __init__(
        self,
        bridge: Bridge,
        follow: FakeFollowServer,
        teleop: FakeTeleop,
        status: StatusWatcher,
        tmp: tempfile.TemporaryDirectory[str],
    ) -> None:
        self.bridge = bridge
        self.follow = follow
        self.teleop = teleop
        self.status = status
        self.tmp = tmp

    @property
    def teleop_frames(self) -> list[str]:
        return self.teleop.frames

    @property
    def port(self) -> int:
        return self.bridge.cfg.bridge_port

    def headset_url(self) -> str:
        return f"ws://127.0.0.1:{self.port}/"


async def build_stack(**overrides: Any) -> _Stack:
    """起假对端 + 桥（未 start；调用方决定生命周期）。"""
    follow = FakeFollowServer()
    await follow.start()
    tmp = tempfile.TemporaryDirectory()

    # 桥在 listen 端点上等对端连进来（与现场同形）；假 teleop 用 client 连上去
    zenoh_endpoint_ = zenoh_endpoint(free_port())
    teleop = FakeTeleop(zenoh_endpoint_, frame_topic("armA"))
    status = StatusWatcher(zenoh_endpoint_, status_topic("armA"), rpc_topic("armA"))

    params: dict[str, Any] = dict(
        bridge_host="127.0.0.1",
        bridge_port=free_port(),
        zenoh_listen_endpoints=(zenoh_endpoint_,),
        follow_host="127.0.0.1",
        follow_port=follow.port,
        status_hz=50.0,
        ping_hz=20.0,
        frame_warn_ms=100.0,
        frame_fallback_ms=250.0,
        event_ttl_ms=10_000.0,
        processes={},
        process_log_dir=Path(tmp.name),
    )
    params.update(overrides)
    cfg = Config(**params)
    return _Stack(Bridge(cfg), follow, teleop, status, tmp)


async def teardown_stack(stack: _Stack) -> None:
    await stack.bridge.close()
    await stack.teleop.stop()
    await stack.status.stop()
    await stack.follow.stop()
    stack.tmp.cleanup()


@contextlib.asynccontextmanager
async def running_bridge(
    *, wait_teleop: bool = True, **overrides: Any
) -> AsyncIterator[_Stack]:
    stack = await build_stack(**overrides)
    bridge = stack.bridge
    await bridge.start()
    await stack.status.start()   # 上位机订阅：快照从这一刻起才会进队列
    if wait_teleop:
        # 假 teleop 是 client，连桥的 listen 端点 —— 所以要等桥的会话先开起来
        await stack.teleop.start()
        await wait_for(lambda: bridge.forwarder.subscribed, timeout=5.0)
    try:
        yield stack
    finally:
        await teardown_stack(stack)


async def wait_for(predicate: Callable[[], bool], timeout: float = 3.0) -> None:
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while loop.time() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.02)
    raise AssertionError("等待条件超时")


async def read_snapshot(ws: Any, predicate: Callable[[dict], bool], timeout: float = 3.0) -> dict:
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    latest: dict | None = None
    while loop.time() < deadline:
        remaining = max(deadline - loop.time(), 0.01)
        raw = await ws.recv(remaining)
        if raw is None:
            break
        latest = json.loads(raw)
        if predicate(latest):
            return latest
    raise AssertionError(f"快照条件超时，最后一份：{json.dumps(latest, ensure_ascii=False)[:400]}")


class SnapshotContractTest(unittest.IsolatedAsyncioTestCase):
    async def test_snapshot_matches_studio_contract(self) -> None:
        async with running_bridge() as stack:
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                first = json.loads(await status.recv(2.0))
                self.assertTrue(first["bridge"]["online"])
                self.assertFalse(first["headset"]["online"])
                self.assertIsNone(first["headset"]["last_frame_age_ms"])

                for _ in range(5):
                    await headset.send(frame(True))

                snap = await read_snapshot(status, lambda s: s["headset"]["frames"] >= 5)

                # DESIGN 4.1 的字段一个都不能少
                self.assertIsInstance(snap["ts"], int)
                bridge = snap["bridge"]
                self.assertIsInstance(bridge["forward_cost_ms"], (int, float))
                self.assertGreaterEqual(bridge["forward_cost_ms"], 0.0)
                self.assertLess(bridge["forward_cost_ms"], 50.0)

                # zenoh 那一段也要在快照里：上位机看不到自己的订阅对端，
                # 得靠桥告诉它"我这边是什么情况"。漏掉过一次，所以锁住。
                zenoh = snap["zenoh"]
                self.assertTrue(zenoh["ready"])
                self.assertEqual(zenoh["arm_id"], "armA")
                self.assertEqual(zenoh["frame_topic"], "litearm/v4/armA/vr_frame")
                self.assertIn("subscribed", zenoh["studio"])

                head = snap["headset"]
                self.assertTrue(head["online"])
                self.assertGreater(head["fps"], 0.0)
                self.assertGreaterEqual(head["latency_ms"], 0.0)
                self.assertLess(head["latency_ms"], 1000.0)
                self.assertIsInstance(head["last_frame_age_ms"], (int, float))
                self.assertTrue(head["following"])
                self.assertEqual(head["frames"], 5)

                arm = snap["arm"]
                self.assertEqual(arm["source"], "follow_server")
                self.assertEqual(len(arm["q"]), 7)
                self.assertIn(arm["state"], ("holding", "following"))
                self.assertEqual(arm["fault"], [])

                self.assertEqual(set(snap["processes"]), {"teleop", "follow_server"})
                self.assertEqual(snap["processes"]["teleop"], "unknown")
                self.assertEqual(snap["owner"], "vr")
                self.assertFalse(snap["fallback"]["active"])
                self.assertIsNone(snap["fallback"]["reason"])
                self.assertIsInstance(snap["events"], list)
                self.assertEqual(snap["link"]["state"], "following")

    async def test_healthz_served_over_http(self) -> None:
        async with running_bridge() as stack:
            def fetch() -> dict:
                with urllib.request.urlopen(
                    f"http://127.0.0.1:{stack.port}/healthz", timeout=2
                ) as resp:
                    return json.loads(resp.read().decode())

            payload = await asyncio.to_thread(fetch)
            self.assertTrue(payload["ok"])
            self.assertFalse(payload["headset_online"])
            self.assertIn("state", payload)


class ForwardingTest(unittest.IsolatedAsyncioTestCase):
    async def test_latest_pose_and_grip_edges_arrive_byte_identical_in_order(self) -> None:
        async with running_bridge() as stack:
            sent = [frame((i // 5) % 2 == 0, ts_ms=1_787_641_468_000 + i * 100) for i in range(20)]
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                for item in sent:
                    await headset.send(item)
                await wait_for(lambda: sent[-1] in stack.teleop_frames)
                indices = [sent.index(raw) for raw in stack.teleop_frames]
                self.assertEqual(indices, sorted(set(indices)))
                # First pose of every grip edge and latest pose must survive.
                for index in (0, 5, 10, 15, 19):
                    self.assertIn(index, indices)
                snap = await read_snapshot(status, lambda s:
                    s["headset"]["frames_forwarded"] + s["headset"]["frames_dropped"] >= len(sent))
                self.assertEqual(snap["headset"]["frames_forwarded"] +
                                 snap["headset"]["frames_dropped"], len(sent))
                self.assertEqual(snap["headset"]["bad_frames"], 0)

    async def test_bad_frame_still_forwarded(self) -> None:
        # DESIGN 6.1：不改帧内容 —— 坏帧也原样转发（只是统计里记一笔）
        async with running_bridge() as stack:
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                await headset.send("{not json")
                await wait_for(lambda: len(stack.teleop_frames) == 1)
                self.assertEqual(stack.teleop_frames[0], "{not json")
                snap = await read_snapshot(status, lambda s: s["headset"]["frames"] >= 1)
                self.assertEqual(snap["headset"]["bad_frames"], 1)
                self.assertFalse(snap["headset"]["following"])

    async def test_frames_dropped_while_teleop_down(self) -> None:
        async with running_bridge(wait_teleop=False) as stack:
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                for _ in range(3):
                    await headset.send(frame(True))
                snap = await read_snapshot(status, lambda s: s["headset"]["frames"] >= 3)
                self.assertFalse(snap["bridge"]["teleop_connected"])
                self.assertEqual(snap["headset"]["frames_dropped"], 3)
                self.assertTrue(snap["headset"]["online"])  # 头显照常在推


class TeleopActionTest(unittest.IsolatedAsyncioTestCase):
    async def test_teleop_stop_blocks_frames_then_latches(self) -> None:
        async with running_bridge() as stack:
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                for _ in range(3):
                    await headset.send(frame(True))
                # 等三帧（去抖 3 帧）真的被处理：这之后才谈得上"已跟随 → 停遥操"
                following = await read_snapshot(status, lambda s: s["headset"]["following"])
                self.assertEqual(following["link"]["state"], "following")
                await status.call({"action": "teleop.stop"})

                await wait_for(lambda: stack.follow.count("latch") >= 1)
                forwarded_at_stop = len(stack.teleop_frames)
                blocked_snap = await read_snapshot(
                    status, lambda s: s["link"]["forward_enabled"] is False
                )
                self.assertFalse(blocked_snap["link"]["forward_enabled"])
                self.assertFalse(blocked_snap["link"]["enabled"])
                self.assertEqual(blocked_snap["link"]["state"], "latched")

                # 停掉之后头显还在推，但一个字都不许再进 teleop（先不问、后持位的顺序）
                for _ in range(5):
                    await headset.send(frame(True))
                await asyncio.sleep(0.3)
                self.assertEqual(len(stack.teleop_frames), forwarded_at_stop)

                snap = await read_snapshot(status, lambda s: s["headset"]["frames"] >= 8)
                self.assertEqual(snap["headset"]["frames_blocked"], 5)
                self.assertIn("teleop_stopped", [e["code"] for e in snap["events"]])
                self.assertIn("latch_done", [e["code"] for e in snap["events"]])

    async def test_teleop_start_reenables_and_sends_start(self) -> None:
        async with running_bridge() as stack:
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                await headset.send(frame(True))          # 头显先接上
                await asyncio.sleep(0.1)
                await status.call({"action": "teleop.stop"})
                await wait_for(lambda: stack.follow.count("latch") >= 1)

                await status.call({"action": "teleop.start"})
                # 队列里可能还有动作之前推的陈旧快照，按"事件出现"来等最新一份
                snap = await read_snapshot(
                    status,
                    lambda s: "teleop_started" in [e["code"] for e in s["events"]],
                )
                self.assertTrue(snap["link"]["forward_enabled"])
                self.assertTrue(snap["link"]["enabled"])
                # 刚从 stop 出来：只打开转发，不替操作员 arming 跟随服务
                self.assertEqual(stack.follow.count("start"), 0)
                self.assertFalse(snap["headset"]["following"])
                self.assertIn(
                    "teleop_start_needs_regrip", [e["code"] for e in snap["events"]]
                )

                await headset.send(frame(True))
                await wait_for(lambda: len(stack.teleop_frames) >= 1)

    async def test_cold_teleop_start_arms_follow_server(self) -> None:
        async with running_bridge() as stack:
            status = stack.status
            await status.call({"action": "teleop.start"})
            await wait_for(lambda: stack.follow.count("start") >= 1)
            snap = await read_snapshot(
                status, lambda s: any(e["code"] == "start_done" for e in s["events"])
            )
            self.assertTrue(snap["link"]["forward_enabled"])

    async def test_estop_sends_stop_and_stops_forwarding(self) -> None:
        async with running_bridge() as stack:
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                for _ in range(3):
                    await headset.send(frame(True))
                await wait_for(lambda: len(stack.teleop_frames) >= 1)

                await status.call({"action": "estop"})
                await wait_for(lambda: stack.follow.count("stop") >= 1)
                # 状态翻转是同步的，但 stop 的回执要等 TCP 往返，所以等事件而不是等状态
                snap = await read_snapshot(
                    status, lambda s: "stop_done" in [e["code"] for e in s["events"]]
                )
                self.assertFalse(snap["link"]["forward_enabled"])
                codes = [e["code"] for e in snap["events"]]
                self.assertIn("estop", codes)
                self.assertIn("stop_done", codes)

    async def test_unknown_action_reported(self) -> None:
        async with running_bridge() as stack:
            status = stack.status
            await status.call({"action": "no.such.action"})
            snap = await read_snapshot(
                status, lambda s: any(e["code"] == "unknown_action" for e in s["events"])
            )
            self.assertTrue(
                any("no.such.action" in e["message"] for e in snap["events"])
            )

    async def test_process_action_without_config_fails_cleanly(self) -> None:
        async with running_bridge() as stack:
            status = stack.status
            await status.call({"action": "process.start", "name": "teleop"})
            snap = await read_snapshot(
                status, lambda s: any(e["code"] == "process_start_failed" for e in s["events"])
            )
            event = next(e for e in snap["events"] if e["code"] == "process_start_failed")
            self.assertIn("未配置", event["message"])

    async def test_event_ack_clears(self) -> None:
        async with running_bridge() as stack:
            status = stack.status
            await status.call({"action": "ping"})
            await read_snapshot(
                status, lambda s: any(e["code"] == "pong" for e in s["events"])
            )
            await status.call({"action": "event.clear"})
            snap = await read_snapshot(
                status,
                lambda s: any(e["code"] == "events_cleared" for e in s["events"]),
            )
            self.assertEqual([e["code"] for e in snap["events"]], ["events_cleared"])


class FallbackTest(unittest.IsolatedAsyncioTestCase):
    async def test_no_frame_holds_once_then_needs_regrip(self) -> None:
        async with running_bridge() as stack:
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                for _ in range(3):
                    await headset.send(frame(True))
                await wait_for(lambda: len(stack.teleop_frames) >= 1)
                following = await read_snapshot(status, lambda s: s["headset"]["following"])
                self.assertEqual(following["link"]["state"], "following")

                # 拔线：不再发帧
                snap = await read_snapshot(status, lambda s: s["fallback"]["active"], timeout=3.0)
                self.assertEqual(snap["link"]["state"], "fallback")
                self.assertFalse(snap["headset"]["following"])
                self.assertFalse(snap["headset"]["online"])
                self.assertEqual(stack.follow.count("hold"), 1)
                self.assertEqual(stack.follow.count("latch"), 0)
                codes = [e["code"] for e in snap["events"]]
                self.assertIn("frame_stall", codes)
                self.assertIn("fallback_hold", codes)

                # 幂等：再等一会儿也只有一次 hold
                await asyncio.sleep(0.5)
                self.assertEqual(stack.follow.count("hold"), 1)

                # 帧流恢复：不自动恢复跟随
                for _ in range(6):
                    await headset.send(frame(True))
                recovered = await read_snapshot(
                    status, lambda s: s["fallback"]["active"] is False and s["headset"]["online"]
                )
                self.assertFalse(recovered["headset"]["following"])
                self.assertEqual(recovered["link"]["state"], "latched")
                self.assertTrue(recovered["link"]["rearm_required"])

                # 松开再按 grip → 恢复跟随
                for _ in range(3):
                    await headset.send(frame(False))
                for _ in range(3):
                    await headset.send(frame(True))
                refollow = await read_snapshot(status, lambda s: s["headset"]["following"])
                self.assertEqual(refollow["link"]["state"], "following")

    async def test_warn_threshold_reports_stall_without_hold(self) -> None:
        async with running_bridge(frame_warn_ms=80.0, frame_fallback_ms=5000.0) as stack:
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                await headset.send(frame(True))
                snap = await read_snapshot(
                    status, lambda s: any(e["code"] == "frame_stall" for e in s["events"])
                )
                self.assertFalse(snap["fallback"]["active"])
                self.assertEqual(stack.follow.count("hold"), 0)

    async def test_fallback_disabled_only_warns(self) -> None:
        async with running_bridge(fallback_enabled=False) as stack:
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                await headset.send(frame(True))
                snap = await read_snapshot(
                    status,
                    lambda s: any(e["code"] == "fallback_disabled" for e in s["events"]),
                    timeout=3.0,
                )
                self.assertFalse(snap["fallback"]["active"])
                self.assertEqual(stack.follow.count("hold"), 0)

    async def test_hold_retried_when_follow_server_refuses(self) -> None:
        async with running_bridge() as stack:
            stack.follow.respond = False  # 跟随服务收下命令但不回包
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                await headset.send(frame(True))
                await read_snapshot(status, lambda s: s["fallback"]["active"], timeout=3.0)
                # 首次失败后每 1s 重试；这里只验证"确实重试了"
                await wait_for(lambda: stack.follow.count("hold") >= 2, timeout=8.0)


class ShutdownTest(unittest.IsolatedAsyncioTestCase):
    async def test_shutdown_latches_then_exits_with_live_clients(self) -> None:
        """systemd 重启的场景：客户端还连着、正在跟随时，close() 必须先 latch 再干净退出。"""
        stack = await build_stack()
        bridge = stack.bridge
        await bridge.start()
        await stack.status.start()
        await stack.teleop.start()
        await wait_for(lambda: bridge.forwarder.subscribed, timeout=5.0)
        try:
            status = stack.status
            async with connect(stack.headset_url()) as headset:
                for _ in range(3):
                    await headset.send(frame(True))
                await read_snapshot(status, lambda s: s["headset"]["following"])

                await asyncio.wait_for(bridge.close(), timeout=10.0)

                self.assertGreaterEqual(stack.follow.count("latch"), 1)
                self.assertFalse(bridge.headset.headset_connected)
                self.assertEqual(bridge.hub.clients, 0)
                # 关掉之后发布者已 undeclare，匹配状态回到 false
                self.assertFalse(bridge.forwarder.subscribed)
                # 客户端应收到服务端关闭（1001 going away），而不是被静默丢弃
                # 桥关了以后不再有新快照 —— 收不到就是收不到（pub/sub 没有关闭帧）
                self.assertIsNone(await status.recv(1.0))
        finally:
            await teardown_stack(stack)

    async def test_close_is_idempotent(self) -> None:
        stack = await build_stack()
        await stack.bridge.start()
        try:
            await stack.bridge.close()
            await asyncio.wait_for(stack.bridge.close(), timeout=5.0)
        finally:
            await teardown_stack(stack)


if __name__ == "__main__":
    unittest.main()

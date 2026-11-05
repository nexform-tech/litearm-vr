from __future__ import annotations

import json
import unittest

from litearm_vr_bridge.frames import parse_frame
from litearm_vr_bridge.governor import TeleopGovernor, TeleopState


def grip_frame(grip: bool):
    return parse_frame(
        json.dumps({"rightController": {"tracked": True, "gripActive": grip}})
    )


class GovernorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.gov = TeleopGovernor(warn_ms=200.0, fallback_ms=500.0, edge_debounce=3)
        self.now = 1000.0

    def feed(self, grip: bool, count: int = 3) -> None:
        for _ in range(count):
            self.now += 0.011
            self.gov.on_frame(self.now, grip_frame(grip))

    def test_starts_idle(self) -> None:
        self.assertEqual(self.gov.state, TeleopState.IDLE)
        self.assertFalse(self.gov.following)

    def test_rising_edge_starts_following_after_debounce(self) -> None:
        self.feed(True, count=2)
        self.assertFalse(self.gov.following, "去抖未满 3 帧不应开始跟随")
        self.feed(True, count=1)
        self.assertTrue(self.gov.following)
        self.assertEqual(self.gov.state, TeleopState.FOLLOWING)

    def test_bounce_does_not_start(self) -> None:
        for grip in (True, False, True, False, True, False):
            self.now += 0.011
            self.gov.on_frame(self.now, grip_frame(grip))
        self.assertFalse(self.gov.following)

    def test_falling_edge_latches(self) -> None:
        self.feed(True)
        self.feed(False)
        self.assertFalse(self.gov.following)
        self.assertEqual(self.gov.state, TeleopState.LATCHED)

    def test_stall_warns_then_falls_back_with_single_hold(self) -> None:
        self.feed(True)
        self.gov.drain_commands()
        self.gov.drain_alarms()

        self.now += 0.25
        self.gov.tick(self.now)
        alarms = self.gov.drain_alarms()
        self.assertEqual([a.code for a in alarms], ["frame_stall"])
        self.assertEqual(alarms[0].level, "warn")
        self.assertEqual(self.gov.drain_commands(), [])

        self.now += 0.4  # 累计 650ms > 500ms
        self.gov.tick(self.now)
        commands = self.gov.drain_commands()
        self.assertEqual([c.action for c in commands], ["hold"])
        self.assertEqual(commands[0].reason, "no_frame>500ms")
        self.assertTrue(self.gov.fallback_active)
        self.assertFalse(self.gov.following)
        self.assertEqual(self.gov.state, TeleopState.FALLBACK)

        # 幂等：同一次掉线不再重复下发
        for _ in range(5):
            self.now += 0.1
            self.gov.tick(self.now)
        self.assertEqual(self.gov.drain_commands(), [])

    def test_alarm_thresholds_are_ordered(self) -> None:
        with self.assertRaises(ValueError):
            TeleopGovernor(warn_ms=500.0, fallback_ms=200.0)

    def test_recovery_does_not_resume_following(self) -> None:
        self.feed(True)
        self.now += 0.7
        self.gov.tick(self.now)
        self.gov.drain_commands()
        self.gov.drain_alarms()

        # 帧流恢复，但手还握着 grip：不能自动恢复跟随
        for _ in range(10):
            self.now += 0.011
            self.gov.on_frame(self.now, grip_frame(True))
        self.assertFalse(self.gov.fallback_active)
        self.assertFalse(self.gov.following)
        self.assertEqual(self.gov.state, TeleopState.LATCHED)
        codes = [a.code for a in self.gov.drain_alarms()]
        self.assertIn("frames_recovered", codes)
        self.assertIn("fallback_cleared", codes)

        # 重新按 grip（松 → 按）才恢复
        self.feed(False)
        self.feed(True)
        self.assertTrue(self.gov.following)
        self.assertEqual(self.gov.state, TeleopState.FOLLOWING)

    def test_teleop_stop_latches_and_blocks_frames(self) -> None:
        self.feed(True)
        self.gov.drain_commands()
        self.gov.drain_alarms()
        self.gov.on_teleop_stop(self.now)
        commands = self.gov.drain_commands()
        self.assertEqual([c.action for c in commands], ["latch"])
        self.assertFalse(self.gov.enabled)
        self.assertFalse(self.gov.following)
        self.assertEqual(self.gov.state, TeleopState.LATCHED)

        # 停用期间按 grip 不应恢复跟随
        self.feed(False)
        self.feed(True)
        self.assertFalse(self.gov.following)

        # 停用期间不再产生帧流告警（是操作员主动停的）
        self.now += 5.0
        self.gov.tick(self.now)
        self.assertEqual(self.gov.drain_alarms(), [])
        self.assertEqual(self.gov.drain_commands(), [])

    def test_cold_teleop_start_requests_follow_server_start(self) -> None:
        """冷启动（还没进入过 rearm 态）：打开转发并请跟随服务 start。"""
        self.gov.on_teleop_start(self.now)
        commands = self.gov.drain_commands()
        self.assertEqual([c.action for c in commands], ["start"])
        self.assertEqual(commands[0].reason, "teleop.start")
        self.assertTrue(self.gov.enabled)

    def test_teleop_start_after_stop_does_not_arm_follow(self) -> None:
        """刚 stop 过：teleop 还握着旧锚点，不能替操作员放开跟随，须重新按 grip。"""
        self.feed(True)
        self.gov.drain_commands()
        self.gov.drain_alarms()
        self.gov.on_teleop_stop(self.now)
        self.gov.drain_commands()

        self.now += 0.05
        self.gov.on_teleop_start(self.now)
        self.assertEqual(self.gov.drain_commands(), [])
        self.assertTrue(self.gov.enabled)
        self.assertFalse(self.gov.following, "rearm 态下不该自动恢复跟随")
        self.assertEqual(self.gov.state, TeleopState.LATCHED)
        codes = [a.code for a in self.gov.drain_alarms()]
        self.assertIn("teleop_start_needs_regrip", codes)

        # 松开再按 grip（teleop 这时才会重新锚定并发 start）→ 恢复跟随
        self.feed(False)
        self.feed(True)
        self.assertTrue(self.gov.following)
        self.assertEqual(self.gov.state, TeleopState.FOLLOWING)

    def test_teleop_start_after_estop_does_not_arm_follow(self) -> None:
        self.feed(True)
        self.gov.drain_commands()
        self.gov.drain_alarms()
        self.gov.on_estop(self.now)
        self.gov.drain_commands()
        self.gov.drain_alarms()

        self.now += 0.05
        self.gov.on_teleop_start(self.now)
        self.assertEqual(self.gov.drain_commands(), [])
        self.assertFalse(self.gov.following)
        self.assertIn("teleop_start_needs_regrip", [a.code for a in self.gov.drain_alarms()])

    def test_teleop_start_without_grip_stays_idle(self) -> None:
        self.gov.on_teleop_start(self.now)
        self.gov.drain_commands()
        self.assertFalse(self.gov.following)
        self.assertEqual(self.gov.state, TeleopState.IDLE)

    def test_estop_sends_stop_and_requires_restart(self) -> None:
        self.feed(True)
        self.gov.drain_commands()
        self.gov.drain_alarms()
        self.gov.on_estop(self.now)
        commands = self.gov.drain_commands()
        self.assertEqual([c.action for c in commands], ["stop"])
        self.assertEqual([a.code for a in self.gov.drain_alarms()], ["estop"])
        self.assertFalse(self.gov.following)
        self.assertFalse(self.gov.enabled)
        self.assertEqual(self.gov.state, TeleopState.LATCHED)

    def test_fallback_can_be_disabled(self) -> None:
        gov = TeleopGovernor(warn_ms=100.0, fallback_ms=300.0, fallback_enabled=False)
        now = 0.0
        for _ in range(3):
            now += 0.011
            gov.on_frame(now, grip_frame(True))
        gov.drain_alarms()
        now += 1.0
        gov.tick(now)
        self.assertEqual(gov.drain_commands(), [])
        codes = [a.code for a in gov.drain_alarms()]
        self.assertIn("frame_stall", codes)
        self.assertIn("fallback_disabled", codes)

    def test_never_emits_movej_or_home(self) -> None:
        # Command.__post_init__ 会把非 hold/latch/stop/start 的动作直接拒掉
        from litearm_vr_bridge.governor import Command

        for illegal in ("movej", "home", "q", "ping"):
            with self.assertRaises(ValueError):
                Command(illegal, reason="test")


if __name__ == "__main__":
    unittest.main()

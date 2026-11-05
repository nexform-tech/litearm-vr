"""Control dispatch regression tests: no sockets, devices, or Bullet connection."""
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "ws_echo_server.py"
_SPEC = importlib.util.spec_from_file_location("control_bridge_under_test", _SCRIPT)
_SERVER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_SERVER)


def frame(grip=True, timestamp=None, a_button=False):
    data = {"rightController": {
        "tracked": True, "gripActive": grip, "trigger": 0.0, "aButton": a_button,
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}
    if timestamp is not None:
        data["timestamp"] = timestamp
    return data


class FakeArm:
    def __init__(self):
        self._tuning = SimpleNamespace(tracking={"input_timeout": 0.25,
                                                  "feedback_timeout": 0.25})
        self._active = False
        self._hold_reason = None
        self.last_input = None
        self.measured = None
        self.ticks = []

    def sync_measured(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise ValueError("invalid measured q")
        self.measured = q.copy()

    def suspend_tracking(self, now):
        pass

    def hold(self, reason):
        self._hold_reason = reason
        self._active = False

    def tick(self, now, measured_q, applied_q=None):
        self.ticks.append((now, copy.deepcopy(measured_q), copy.deepcopy(applied_q)))
        if now - self.last_input > self._tuning.tracking["input_timeout"]:
            self.hold("input timeout")
        elif measured_q is not None:
            self.sync_measured(measured_q)


class FakeSim:
    def __init__(self):
        self.arm = FakeArm()
        self._tuning = self.arm._tuning
        self.starts = []
        self.pauses = []
        self.updates = []
        self.homings = []

    def arms(self):
        return {"right": self.arm}

    def start(self, side, pos, quat, now):
        self.starts.append((side, pos.copy(), quat.copy(), now))
        self.arm._active = True
        self.arm.last_input = now

    def pause(self, side):
        self.pauses.append(side)
        self.arm._active = False

    def start_homing(self, side, now=None):
        self.homings.append((side, now))
        self.arm._active = False

    def update_pose(self, side, pos, quat, now):
        self.updates.append((side, now))
        self.arm.last_input = now

    def get_joint_states(self):
        return ["joint%d" % i for i in range(7)], [0.1] * 7

    def go_home(self, *args):
        raise AssertionError("input must never automatically go home")


class FakeReal:
    def __init__(self):
        self.state = {"q": [0.2] * 7}
        self.starts = []
        self.holds = []
        self.commands = []
        self.requests = []
        self.ready = True
        self.error = None

    def get_state(self, side, max_age):
        assert max_age == 0.25
        return copy.deepcopy(self.state)

    def request_state(self, side):
        self.requests.append(side)

    def is_started(self, side):
        return self.ready

    def command_error(self, side):
        return self.error

    def start(self, side):
        self.starts.append(side)

    def hold(self, side):
        self.holds.append(side)

    def send_q(self, side, q):
        self.commands.append((side, list(q)))

    def movej(self, *args):
        raise AssertionError("input must never automatically movej")


class ControlBridgeTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.clock = patch.object(_SERVER.time, "monotonic", side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.sim, self.real = FakeSim(), FakeReal()
        self.bridge = _SERVER.TeleopBridge(self.sim, real_client=self.real)

    def feed(self, grip=True, timestamp=None):
        self.bridge.handle_frame(frame(grip, timestamp))
        if self.bridge._starting["right"] is not None and self.real.ready:
            # Complete the asynchronous handshake before exercising active control.
            self.bridge.tick(self.now)
            self.sim.arm.ticks.clear()
            self.real.commands.clear()

    def tick(self, dt=0.01):
        self.now += dt
        self.bridge.tick(self.now)

    def test_start_ack_required_before_anchor_or_command(self):
        self.real.ready = False
        self.bridge.handle_frame(frame())
        self.tick()
        self.assertEqual(self.sim.starts, [])
        self.assertEqual(self.real.commands, [])
        self.now += .1
        self.bridge.handle_frame(frame())
        self.real.ready = True
        self.tick()
        self.assertEqual(len(self.sim.starts), 1)
        self.assertEqual(len(self.real.commands), 1)

    def test_release_cancels_pending_start_even_if_ack_arrives_late(self):
        self.real.ready = False
        self.bridge.handle_frame(frame())
        self.bridge.handle_frame(frame(False))
        self.real.ready = True
        self.tick()
        self.assertEqual(self.sim.starts, [])
        self.assertEqual(self.real.commands, [])

    def test_ticks_continue_without_new_input_and_use_measured_feedback(self):
        self.feed()
        np.testing.assert_allclose(self.sim.arm.measured, [0.2] * 7)
        self.assertEqual(self.real.commands, [], "input does not execute IK/send q")
        self.real.state = {"q": [0.3] * 7}
        for _ in range(5):
            self.tick()
        self.assertEqual(len(self.sim.arm.ticks), 5)
        self.assertEqual(len(self.real.commands), 5)
        np.testing.assert_allclose(self.sim.arm.measured, [0.3] * 7)

    def test_downstream_applied_command_is_forwarded_to_execution_monitor(self):
        self.real.state = {"q": [0.2] * 7, "cmd_q": [0.25] * 7}
        self.feed()
        self.tick()
        self.assertEqual(self.sim.arm.ticks[-1][2], [0.25] * 7)

    def test_release_sends_no_extra_joint_command_and_regrip_never_homes(self):
        self.feed()
        self.tick()
        self.feed(False)
        count = len(self.real.commands)
        for _ in range(3):
            self.tick()
        self.assertEqual(len(self.real.commands), count)
        self.assertEqual(self.sim.pauses, ["right"])
        self.assertIn("right", self.real.holds)
        self.feed(True)  # rapid second press must not trigger old double-click home
        self.tick()
        self.assertEqual(len(self.sim.starts), 2)
        self.assertEqual(len(self.real.commands), 1)

    def test_fault_latches_until_observed_release_and_regrip(self):
        self.feed()
        self.bridge.handle_frame({"teleopInputFault": "queue_overflow"})
        for _ in range(3):
            self.feed()
            self.tick()
        self.assertTrue(self.bridge._blocked["right"])
        self.assertEqual(len(self.sim.starts), 1)
        self.assertEqual(self.real.commands, [])
        self.feed(False)
        self.tick()
        self.assertEqual(self.real.commands, [])
        self.feed(True)
        self.tick()
        self.assertEqual(len(self.sim.starts), 2)
        self.assertEqual(len(self.real.commands), 1)

    def test_a_button_triggers_smooth_homing_and_disengages(self):
        self.feed()
        self.tick()
        self.assertTrue(self.bridge._engaged["right"])
        self.bridge.handle_frame(frame(grip=True, a_button=True))
        self.assertEqual(len(self.sim.homings), 1)
        self.assertEqual(self.sim.homings[0][0], "right")
        self.assertFalse(self.bridge._engaged["right"])

    def test_input_timeout_translates_arm_hold_into_latched_real_hold(self):
        self.feed()
        self.tick(0.3)
        self.assertTrue(self.bridge._blocked["right"])
        self.assertEqual(self.real.commands, [])
        self.feed(True)
        self.tick()
        self.assertEqual(len(self.sim.starts), 1)
        self.assertEqual(self.real.commands, [])

    def test_missing_feedback_prevents_start_and_requires_regrip(self):
        self.real.state = None
        self.feed()
        self.assertEqual(self.sim.starts, [])
        self.real.state = {"q": [0.2] * 7}
        self.feed()
        self.assertEqual(self.sim.starts, [])
        self.feed(False)
        self.feed(True)
        self.assertEqual(len(self.sim.starts), 1)

    def test_feedback_expiry_and_invalid_feedback_stop_commands(self):
        for state in (None, {"q": [float("nan")] * 7}, {"q": [0.0]}):
            with self.subTest(state=state):
                self.setUp()
                self.feed()
                self.real.state = state
                self.tick()
                self.assertTrue(self.bridge._blocked["right"])
                self.assertEqual(self.real.commands, [])

    def test_source_reorder_does_not_refresh_timeout_or_release(self):
        self.feed(timestamp=100000)
        original_time = self.sim.arm.last_input
        self.now += 0.1
        self.feed(False, timestamp=99999)
        self.feed(False, timestamp=100000)
        self.assertEqual(self.sim.pauses, [])
        self.assertEqual(self.sim.arm.last_input, original_time)
        self.tick(0.2)
        self.assertTrue(self.bridge._blocked["right"])

    def test_stale_and_jumping_source_timestamps_hold(self):
        for delta_now, stamp in ((0.4, 100001), (0.01, 102000)):
            with self.subTest(stamp=stamp):
                self.setUp()
                self.feed(timestamp=100000)
                self.now += delta_now
                self.feed(timestamp=stamp)
                self.tick()
                self.assertTrue(self.bridge._blocked["right"])
                self.assertEqual(self.real.commands, [])

    def test_invalid_frames_hold_without_commands(self):
        invalid = [None, [], {}, {"rightController": None},
                   {"rightController": {"tracked": False}}]
        for field, value in (("position", None), ("quaternion", {}),
                             ("trigger", float("nan"))):
            data = frame()
            data["rightController"][field] = value
            invalid.append(data)
        for value in (float("nan"), "bad"):
            invalid.append(frame(timestamp=value))
        zero = frame()
        zero["rightController"]["quaternion"]["w"] = 0.0
        invalid.append(zero)
        for data in invalid:
            with self.subTest(data=data):
                self.setUp()
                self.feed()
                self.bridge.handle_frame(data)
                self.tick(self.bridge._sim._tuning.tracking["input_timeout"] + 0.02)
                self.assertTrue(self.bridge._blocked["right"])
                self.assertEqual(self.real.commands, [])

    def test_arm_hold_is_not_logged_twice_by_bridge(self):
        from unittest.mock import Mock
        self.feed()
        self.sim.arm.hold("输入超时")
        self.sim.arm.hold = Mock(wraps=self.sim.arm.hold)
        self.bridge.hold("right", "输入超时")
        self.sim.arm.hold.assert_not_called()
        self.assertTrue(self.bridge._blocked["right"])
        self.assertIn("right", self.real.holds)

    def test_disconnect_holds_and_resets_source_clock(self):
        self.feed(timestamp=100000)
        self.bridge.disconnect()
        self.tick()
        self.assertEqual(self.real.commands, [])
        self.assertIsNone(self.bridge._last_timestamp)
        self.feed(False, timestamp=10)
        self.feed(True, timestamp=11)
        self.tick()
        self.assertEqual(len(self.sim.starts), 2)


if __name__ == "__main__":
    unittest.main()

import unittest
from measured_state import SequenceFreshness, response, sdk_response


class Arm:
    def __init__(self, state):
        self.state = state

    def get_state(self):
        return self.state


class MeasuredStateTests(unittest.TestCase):
    def test_sequence_must_advance_and_expires(self):
        fresh = SequenceFreshness()
        self.assertIsNone(fresh.age(1, 10.0))
        self.assertIsNone(fresh.age(1, 10.05))
        self.assertAlmostEqual(fresh.age(2, 10.10), 0.05)
        self.assertAlmostEqual(fresh.age(2, 10.30), 0.25)
        self.assertIsNone(fresh.age(3, 11.0))  # long gap cannot prove freshness
        self.assertAlmostEqual(fresh.age(4, 11.05), 0.05)

    def test_reply_validation(self):
        self.assertFalse(response(1, [0]*7, 0.3, True)["valid"])
        self.assertFalse(response(1, [float('nan')]*7, 0, True)["valid"])
        self.assertEqual(response(42, [0]*7, 0.01, True)["request_id"], 42)

    def test_sdk_requires_all_measured_joints_fresh(self):
        state = {"q": [0]*7, "fault": [], "feedback": {
            "joints": [{"fresh": True, "age_s": 0.01} for _ in range(7)]}}
        self.assertTrue(sdk_response(Arm(state), 1)["valid"])
        state["feedback"]["joints"][3]["fresh"] = False
        self.assertFalse(sdk_response(Arm(state), 1)["valid"])
        self.assertFalse(sdk_response(Arm({"q": [0]*7}), 1)["valid"])

# Load the actual server class without importing optional hardware SDKs.
import ast
from pathlib import Path
import threading
from unittest.mock import Mock


def server_class(filename):
    tree = ast.parse((Path(__file__).parent / filename).read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ArmFollowServer")
    namespace = {"sdk_response": sdk_response, "LiteArmError": RuntimeError}
    exec(compile(ast.Module(body=[node], type_ignores=[]), filename, "exec"), namespace)
    return namespace["ArmFollowServer"]


class HoldTests(unittest.TestCase):
    def test_stm32_state_reports_post_slew_command(self):
        cls = server_class("arm_follow_server_stm32.py")
        server = cls.__new__(cls)
        server._lock = threading.Lock()
        server._arm = Mock()
        server._arm.measured_state.return_value = {
            "ok": True, "cmd": "state", "request_id": 7,
            "valid": True, "q": [0.1]*7, "age_s": 0.01}
        server._q_cmd = [0.2]*7

        reply = server.handle({"cmd": "state", "request_id": 7})

        self.assertEqual(reply["q"], [0.1]*7)
        self.assertEqual(reply["cmd_q"], [0.2]*7)
        reply["cmd_q"][0] = 99
        self.assertEqual(server._q_cmd[0], 0.2)

    def test_stm32_hold_preserves_stream_and_drops_far_target(self):
        cls = server_class("arm_follow_server_stm32.py")
        for valid in (True, False):
            server = cls.__new__(cls)
            server._lock = threading.Lock()
            server._arm = Mock()
            server._arm.measured_state.return_value = {
                "valid": valid, "q": [0.1]*7 if valid else None}
            server._q_cmd = [0.2]*7
            server._last_target = [2]*7
            server._following = True
            server._mode = "jf"
            server._kp, server._kd = [50]*7, [2]*7
            reply = server.handle({"cmd": "hold"})
            expected = [0.1]*7 if valid else [0.2]*7
            self.assertTrue(reply["ok"])
            self.assertEqual(server._q_cmd, expected)
            self.assertIsNone(server._last_target)
            self.assertEqual(server._dq_cmd, [0]*7)
            server._arm.joint_follow_stream.assert_called_once_with(
                expected, [0]*7, [50]*7, [2]*7)
            server._arm.measured_state.return_value = {"valid": True, "q": [0.5]*7}
            server.handle({"cmd": "hold"})
            self.assertEqual(server._q_cmd, expected, "hold keepalive must not re-anchor")
            server._arm.movej.assert_not_called()
            server._arm.enable.assert_not_called()

    def test_sdk_hold_keeps_loop_running(self):
        cls = server_class("arm_follow_server_sdk.py")
        server = cls.__new__(cls)
        server._lock = threading.RLock()
        server._arm = Mock()
        server._arm.get_state.return_value = {"q": [0.1]*7, "feedback": {
            "joints": [{"fresh": True, "age_s": 0.01} for _ in range(7)]}}
        server._q_target = [2]*7
        server._following = True
        self.assertTrue(server.handle({"cmd": "hold"})["ok"])
        self.assertEqual(server._q_target, [0.1]*7)
        self.assertTrue(server._following)
        server._arm.request_stop.assert_not_called()
        server._holding = False
        server._arm.get_state.return_value = {"q": [2]*7}
        server.handle({"cmd": "hold"})
        self.assertIsNone(server._q_target)


if __name__ == "__main__":
    unittest.main()

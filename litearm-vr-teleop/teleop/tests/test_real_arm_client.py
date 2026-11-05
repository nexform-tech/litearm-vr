"""Hardware-free protocol tests using socket pairs."""
import importlib.util
import json
from pathlib import Path
import socket
import threading
import time
import unittest

path = Path(__file__).resolve().parents[1] / "scripts" / "real_arm_client.py"
spec = importlib.util.spec_from_file_location("real_arm_client", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.client = module.RealArmClient()
        self.sock, self.peer = socket.socketpair()
        self.client._socks["left"] = self.sock
        self.peer.settimeout(1)

    def tearDown(self):
        self.client.close()
        self.peer.close()

    def reply(self, **extra):
        self.client.request_state("left")
        request = json.loads(self.peer.recv(4096))
        result = dict(cmd="state", request_id=request["request_id"], valid=True,
                      q=[0.1] * 7, age_s=0.01)
        result.update(extra)
        wire = json.dumps(result).encode() + b"\n"
        # Verify fragmented JSON, command acknowledgements and multiple frames.
        self.peer.sendall(b'{"ok":true,"cmd":"q"}\n' + wire[:12])
        self.peer.sendall(wire[12:])
        deadline = time.monotonic() + 1
        while self.client._pending["left"] and time.monotonic() < deadline:
            time.sleep(0.005)

    def test_start_ack_and_hold_cancel(self):
        self.client.start("left")
        self.assertEqual(json.loads(self.peer.recv(4096))["cmd"], "start")
        self.assertFalse(self.client.is_started("left"))
        self.client._handle_response("left", self.sock, {"cmd": "start", "ok": True})
        self.assertTrue(self.client.is_started("left"))
        self.assertIsNone(self.client.get_state("left"))
        self.client.hold("left")
        self.assertEqual(json.loads(self.peer.recv(4096))["cmd"], "hold")
        self.client._handle_response("left", self.sock, {"cmd": "start", "ok": True})
        self.assertFalse(self.client.is_started("left"))

    def test_command_error_is_not_silently_drained(self):
        self.client._handle_response("left", self.sock,
                                     {"cmd": "q", "ok": False, "error": "disabled"})
        self.assertEqual(self.client.command_error("left"), "disabled")
        self.assertFalse(self.client.is_started("left"))

    def test_measured_feedback_and_expiry(self):
        self.reply(cmd_q=[0.15] * 7)
        state = self.client.get_state("left")
        self.assertEqual(state["q"], [0.1] * 7)
        self.assertEqual(state["cmd_q"], [0.15] * 7)
        state["q"][0] = 100
        state["cmd_q"][0] = 100
        self.assertEqual(self.client.get_state("left")["q"][0], 0.1)
        self.assertEqual(self.client.get_state("left")["cmd_q"][0], 0.15)
        self.assertIsNone(self.client.get_state("left", max_age=0.001))
        self.client._drop("left", self.sock)
        self.assertIsNone(self.client.get_state("left"))

    def test_invalid_and_nonfinite_rejected(self):
        self.reply(q=[float("nan")] * 7)
        self.assertIsNone(self.client.get_state("left"))

        self.client._last_request["left"] = 0
        self.reply(cmd_q=[float("nan")] * 7)
        self.assertIsNone(self.client.get_state("left"))

    def test_ack_or_unsolicited_state_is_not_measurement(self):
        self.client._handle_response("left", self.sock,
            dict(cmd="state", request_id=999, valid=True, q=[0] * 7, age_s=0))
        self.assertIsNone(self.client.get_state("left"))

    def test_invalid_measurement_clears_previous(self):
        self.reply()
        self.assertIsNotNone(self.client.get_state("left"))
        self.client._last_request["left"] = 0
        self.reply(valid=False)
        self.assertIsNone(self.client.get_state("left"))


if __name__ == "__main__":
    unittest.main()

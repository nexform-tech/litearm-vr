from __future__ import annotations

import json
import unittest

from litearm_vr_bridge.frames import parse_frame, timestamp_is_wall_clock


class ParseFrameTest(unittest.TestCase):
    def test_reads_timestamp_grip_and_trigger(self) -> None:
        info = parse_frame(
            json.dumps(
                {
                    "timestamp": 1787641468151,
                    "rightController": {
                        "tracked": True,
                        "gripActive": True,
                        "trigger": 0.75,
                    },
                    "leftController": {"tracked": False, "gripActive": False},
                }
            )
        )
        self.assertEqual(info.ts_ms, 1787641468151.0)
        self.assertTrue(info.grip)
        self.assertTrue(info.tracked)
        assert info.right is not None and info.left is not None
        self.assertAlmostEqual(info.right.trigger, 0.75)
        self.assertFalse(info.left.tracked)

    def test_grip_ignored_when_not_tracked(self) -> None:
        info = parse_frame(
            json.dumps({"timestamp": 1, "rightController": {"tracked": False, "gripActive": True}})
        )
        self.assertFalse(info.grip)

    def test_missing_tracked_defaults_to_true(self) -> None:
        # 有些客户端不发 tracked；上游按"在跟踪"处理，这里保持一致
        info = parse_frame(json.dumps({"rightController": {"gripActive": True}}))
        self.assertTrue(info.grip)

    def test_grip_accepts_numeric(self) -> None:
        self.assertTrue(parse_frame(json.dumps({"rightController": {"gripActive": 1}})).grip)
        self.assertFalse(parse_frame(json.dumps({"rightController": {"gripActive": 0}})).grip)

    def test_bad_json_is_tolerated(self) -> None:
        info = parse_frame("not json at all")
        self.assertIsNone(info.ts_ms)
        self.assertFalse(info.grip)
        self.assertIsNone(info.right)

    def test_non_object_json_is_tolerated(self) -> None:
        self.assertIsNone(parse_frame("[1,2,3]").ts_ms)

    def test_seconds_timestamp_normalized_to_ms(self) -> None:
        info = parse_frame(json.dumps({"timestamp": 1787641468.151}))
        self.assertAlmostEqual(info.ts_ms or 0.0, 1787641468151.0, places=0)
        self.assertTrue(timestamp_is_wall_clock(info.ts_ms))

    def test_absurd_timestamp_passes_through(self) -> None:
        # 头显自开机计时（1_234_567 ms）不是墙上时钟：保留但不当作可比时间
        info = parse_frame(json.dumps({"timestamp": 1_234_567}))
        self.assertEqual(info.ts_ms, 1_234_567.0)
        self.assertFalse(timestamp_is_wall_clock(info.ts_ms))

    def test_negative_or_zero_timestamp_rejected(self) -> None:
        self.assertIsNone(parse_frame(json.dumps({"timestamp": 0})).ts_ms)
        self.assertIsNone(parse_frame(json.dumps({"timestamp": -5})).ts_ms)
        self.assertIsNone(parse_frame(json.dumps({"timestamp": "x"})).ts_ms)


if __name__ == "__main__":
    unittest.main()

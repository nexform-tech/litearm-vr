"""假头显（`scripts/fake_headset.py`）造的帧，桥得认。

这个脚本不进包（它是给现场用的工具），但它的产出物必须能被桥解析 —— 帧格式写错了
它就白写了，而且错法很安静（桥只会把坏帧原样转发，不报错）。所以在这里用**桥自己的
解析器**过一遍，另外验一下默认幅度越得过上游的静止死区（越不过就"帧通了臂不动"，
现场很难查）。
"""

from __future__ import annotations

import importlib.util
import json
import unittest
from pathlib import Path

from litearm_vr_bridge.frames import parse_frame, timestamp_is_wall_clock

_SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "fake_headset.py"


def _load():
    spec = importlib.util.spec_from_file_location("fake_headset", _SCRIPT)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fake = _load()


class FakeHeadsetFrameTest(unittest.TestCase):
    def test_frame_parses_as_headset_data(self) -> None:
        info = parse_frame(fake.frame_json(fake.BASE_POSE, grip=True))
        self.assertIsNotNone(info.right)
        self.assertTrue(info.tracked)
        self.assertTrue(info.grip)          # gripActive=true → 遥操的"发令"信号
        self.assertTrue(timestamp_is_wall_clock(info.ts_ms))  # 本机能比的 Unix 毫秒

    def test_grip_released_reads_as_not_gripping(self) -> None:
        info = parse_frame(fake.frame_json(fake.BASE_POSE, grip=False))
        self.assertTrue(info.tracked)
        self.assertFalse(info.grip)

    def test_untracked_frames_are_ignored_by_consumer(self) -> None:
        info = parse_frame(fake.frame_json(fake.BASE_POSE, grip=True, tracked=False))
        self.assertFalse(info.tracked)      # 消费端应忽略这一帧
        self.assertFalse(info.grip)

    def test_frame_matches_protocol_fields(self) -> None:
        data = json.loads(fake.frame_json(fake.BASE_POSE, grip=False, trigger=0.7))
        self.assertEqual(sorted(data), ["leftController", "rightController", "timestamp"])
        right = data["rightController"]
        self.assertEqual(
            sorted(right),
            ["gripActive", "position", "quaternion", "rotation", "tracked", "trigger"],
        )
        self.assertEqual(sorted(right["position"]), ["x", "y", "z"])
        self.assertEqual(right["trigger"], 0.7)
        # 左手也发，但不按 grip（单臂侧装时只驱动右手）
        self.assertFalse(data["leftController"]["gripActive"])


class FakeHeadsetMotionTest(unittest.TestCase):
    """默认幅度必须越得过上游 teleop 的静止死区 —— 否则就是"帧通了但臂不动"。"""

    def test_default_circle_clears_the_deadband(self) -> None:
        delta = fake.per_frame_delta(radius=0.03, period=4.0, hz=90.0)
        self.assertGreater(delta, fake.DEADBAND_T_OUT)

    def test_hold_pattern_does_not_move(self) -> None:
        for t in (0.0, 1.0, 2.5):
            self.assertEqual(
                fake.pose_at(t, pattern="hold", radius=0.03, period=4.0), fake.BASE_POSE
            )

    def test_circle_actually_moves_and_stays_small(self) -> None:
        radius, period = 0.03, 4.0
        poses = [fake.pose_at(t, pattern="circle", radius=radius, period=period)
                 for t in (0.0, 0.5, 1.0, 1.5, 2.0)]
        self.assertGreater(len(set(poses)), 1)  # 在动
        for pos in poses:
            offset = max(abs(pos[i] - fake.BASE_POSE[i]) for i in range(3))
            self.assertLessEqual(offset, radius + 1e-9)  # 但幅度不超过半径


if __name__ == "__main__":
    unittest.main()

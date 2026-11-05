"""DIRECT replay: movement throughput, delayed feedback, and final settling.

No hardware, sockets, wall-clock sleeps, or mocked IK are used.
"""
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from litearm_teleop.teleop_sim import LiteArmTeleopSim, Tuning


DT = 1.0 / 60.0
QUAT = [0.0, 0.0, 0.0, 1.0]


class TrackingSpeedTests(unittest.TestCase):
    def replay(self, feedback_delay=0, seconds=1.0):
        # Move hand +X by 60 mm over one second, then hold the complete pose.
        # This maps to a reachable direction at the default upright home.
        tuning = Tuning(tracking={"input_timeout": 100.0,
                                  "stall_timeout": 100.0})
        sim = LiteArmTeleopSim(viewer="rviz", tuning=tuning,
                              ik_backend="pybullet")
        try:
            arm = next(iter(sim.arms().values()))
            arm.start([0.0, 0.0, 0.0], QUAT, 0.0)
            initial, _ = arm._target.raw_target()
            commands = [arm._command_q.copy()]
            positions = [initial.copy()]
            velocities = []
            for frame in range(1, round(seconds / DT) + 1):
                now = frame * DT
                arm.update_pose([0.06 * min(now, 1.0), 0.0, 0.0], QUAT, now)
                # delay=0 means the preceding command is measured exactly;
                # delay=3 adds 50 ms of feedback lag to the same replay.
                measured = commands[max(0, len(commands) - 1 - feedback_delay)]
                arm.tick(now, measured_q=measured)
                self.assertIsNone(arm._hold_reason)
                commands.append(arm._command_q.copy())
                velocities.append(arm._command_velocity.copy())
                p, q = arm._eef_pose()
                rr = Rotation.from_quat(q).as_matrix()
                positions.append(p + rr @ arm._tcp_offset)
            target_p, target_r = arm._target.raw_target()
            direction = (target_p - initial) / np.linalg.norm(target_p - initial)
            progress = (np.asarray(positions) - initial) @ direction
            return dict(commands=np.asarray(commands), progress=progress,
                        velocity=np.asarray(velocities),
                        position_error=float(np.linalg.norm(positions[-1] - target_p)),
                        rotation_error=float(Rotation.from_matrix(target_r @ rr.T).magnitude()))
        finally:
            sim.close()

    def test_continuous_reachable_motion_preserves_requested_speed(self):
        result = self.replay()
        # Exclude the first 250 ms of acceleration and input-filter startup.
        speed = (result["progress"][60] - result["progress"][15]) / 0.75
        self.assertGreater(speed, 0.045, "60 mm/s input was artificially slowed")
        self.assertLess(result["position_error"], 0.012)

    def test_fifty_ms_feedback_lag_does_not_slow_command_trajectory(self):
        ideal = self.replay()
        delayed = self.replay(feedback_delay=3)
        self.assertGreater(delayed["progress"][-1], ideal["progress"][-1] * 0.95)
        np.testing.assert_allclose(delayed["commands"], ideal["commands"], atol=1e-5)

    def test_complete_target_settles_after_motion_stops(self):
        result = self.replay(feedback_delay=3, seconds=5.0)
        self.assertLess(result["position_error"], 0.0012)
        self.assertLess(result["rotation_error"], 0.012)
        self.assertLess(np.max(np.abs(result["velocity"][-60:])), 0.005)
        # The command remains within configured velocity and acceleration limits.
        cfg = Tuning().tracking
        self.assertLessEqual(np.max(np.abs(result["velocity"])), cfg["max_joint_velocity"] + 1e-9)
        acceleration = np.diff(np.vstack([np.zeros(7), result["velocity"]]), axis=0) / DT
        self.assertLessEqual(np.max(np.abs(acceleration)), cfg["max_joint_acceleration"] + 1e-7)


if __name__ == "__main__":
    unittest.main()

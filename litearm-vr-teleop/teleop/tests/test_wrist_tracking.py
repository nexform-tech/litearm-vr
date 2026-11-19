"""Pure wrist rotation and recovery using the single upright arm in DIRECT."""

import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from litearm_teleop.teleop_sim import LiteArmTeleopSim, Tuning


FREQ = 60
HARDWARE_LOWER = np.array([
    -2.949607, -1.797689, -2.80998, -3.07178, -2.80998, -1.553343, -1.553343,
])
HARDWARE_UPPER = np.array([
    2.949607, 1.797689, 2.80998, 0.593412, 2.80998, 1.553343, 1.553343,
])


def wrist_replay(backend, tcp_offset=None):
    sim = LiteArmTeleopSim(viewer="rviz", ik_backend=backend, tcp_offset=tcp_offset)
    try:
        arm = sim.arms()["right"]
        arm.set_hardware_limits(HARDWARE_LOWER, HARDWARE_UPPER)
        hand_position = np.zeros(3)
        arm.start(hand_position, Rotation.identity().as_quat(), now=0.0)
        initial_tcp, initial_R = arm._target.raw_target()
        commands = [arm._get_q()]
        tcp_positions = [initial_tcp]
        orientations = [initial_R]

        # Rotate over two seconds, hold, rotate back, then independently translate.
        for frame in range(1, 9 * FREQ + 1):
            now = frame / FREQ
            angle = np.interp(now, [0.0, 2.0, 3.5, 5.5], [0.0, 120.0, 120.0, 0.0])
            position = hand_position + [np.interp(now, [7.0, 8.0], [0.0, 0.06]), 0.0, 0.0]
            hand_rotation = Rotation.from_euler("x", angle, degrees=True)
            arm.update_pose(position, hand_rotation.as_quat(), now)
            arm.tick(now)
            if arm._hold_reason is not None:
                raise AssertionError(arm._hold_reason)
            flange, quaternion = arm._eef_pose()
            orientation = Rotation.from_quat(quaternion).as_matrix()
            commands.append(arm._get_q())
            tcp_positions.append(flange + orientation @ arm._tcp_offset)
            orientations.append(orientation)

        target_tcp, _ = arm._target.raw_target()
        return np.array(commands), np.array(tcp_positions), np.array(orientations), target_tcp
    finally:
        sim.close()


class WristTrackingTests(unittest.TestCase):
    def test_high_rotation_keeps_tcp_and_recovers(self):
        for backend in ("pybullet", "exact"):
            with self.subTest(backend=backend):
                lowest_j7 = self.check_replay(backend, None, 0.001, 45.0)
                self.assertLess(lowest_j7, HARDWARE_LOWER[6] + np.deg2rad(0.1))

    def test_high_rotation_keeps_offset_tcp(self):
        self.check_replay("pybullet", [0.0, 0.0, 0.18], 0.0015, 30.0)

    def check_replay(self, backend, tcp_offset, position_tolerance, min_angle):
        commands, tcp, orientation, target_tcp = wrist_replay(backend, tcp_offset)
        rotation_end = int(3.5 * FREQ)
        translation_start = 7 * FREQ
        pure_rotation = slice(translation_start + 1)

        upstream = commands[pure_rotation, :4] - commands[0, :4]
        self.assertLessEqual(np.max(np.abs(upstream)), np.deg2rad(20.0) + 1e-8)
        self.assertTrue(np.all(commands >= HARDWARE_LOWER - 1e-8))
        self.assertTrue(np.all(commands <= HARDWARE_UPPER + 1e-8))
        position_error = np.linalg.norm(tcp[pure_rotation] - tcp[0], axis=1)
        self.assertLess(np.max(position_error), position_tolerance)
        angles = Rotation.from_matrix(orientation @ orientation[0].T).magnitude()
        self.assertGreater(angles[rotation_end], np.deg2rad(min_angle))
        self.assertLess(angles[translation_start], np.deg2rad(2.0))
        self.assertLess(np.linalg.norm(tcp[-1] - target_tcp), 0.003)

        cfg = Tuning().tracking
        velocities = np.diff(commands, axis=0, prepend=commands[:1]) * FREQ
        acceleration = np.diff(velocities, axis=0) * FREQ
        self.assertLessEqual(np.max(np.abs(velocities)), cfg["max_joint_velocity"] + 1e-8)
        self.assertLessEqual(np.max(np.abs(velocities[pure_rotation])),
                             cfg["wrist_max_joint_velocity"] + 1e-8)
        self.assertLessEqual(np.max(np.abs(acceleration)), cfg["max_joint_acceleration"] + 1e-7)
        return np.min(commands[:rotation_end + 1, 6])


if __name__ == "__main__":
    unittest.main()

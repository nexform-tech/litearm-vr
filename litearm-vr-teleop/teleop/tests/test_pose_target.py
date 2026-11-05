"""Regression tests for motion lost to per-frame deadbands and IK progress."""
import unittest
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation as R
from litearm_teleop.pose_target import AnchoredPoseTarget


class PoseTargetTests(unittest.TestCase):
    def make(self, alpha=1., deadband=True, frame="tool", mapping=None, signs=None):
        tuning = SimpleNamespace(smooth_alpha=alpha, deadband_t_in=.0002,
                                 deadband_t_out=.001, deadband_r_in=.002,
                                 deadband_r_out=.01)
        if not deadband:
            tuning.deadband_t_in = tuning.deadband_t_out = 0.
            tuning.deadband_r_in = tuning.deadband_r_out = 0.
        obj = AnchoredPoseTarget(tuning, np.eye(3) if mapping is None else mapping,
                                 np.eye(3), np.ones(3) if signs is None else signs, frame)
        obj.start(np.zeros(3), [0, 0, 0, 1], np.zeros(3), np.eye(3), 0.)
        return obj

    def test_slow_motion_accumulates(self):
        obj = self.make()
        for i in range(1, 301):
            obj.update([i*.0001, 0, 0], R.from_rotvec([0, 0, i*.001]).as_quat(), i/60)
            p, rot = obj.target(i/60)
        self.assertLess(np.linalg.norm(p - [.03, 0, 0]), .00101)
        self.assertLess((R.from_matrix(rot).inv()*R.from_rotvec([0, 0, .3])).magnitude(), .01001)

    def test_fast_out_slow_return_one_hundred_cycles(self):
        obj = self.make(alpha=.5)
        tick = 0
        for _ in range(100):
            for x in np.r_[.03, np.linspace(.03, 0, 61), np.zeros(30)]:
                tick += 1
                obj.update([x, 0, 0], R.from_rotvec([x*10, 0, 0]).as_quat(), tick/60)
                p, rot = obj.target(tick/60)
            self.assertLess(np.linalg.norm(p), .00101)
            self.assertLess(R.from_matrix(rot).magnitude(), .01001)

    def test_rotation_crosses_180_without_jump(self):
        obj = self.make(deadband=False, signs=[1, -1, -1])
        previous = np.eye(3)
        for i, deg in enumerate(range(1, 361), 1):
            obj.update([0, 0, 0], R.from_euler("y", deg, degrees=True).as_quat(), i/60)
            _, rot = obj.target(i/60)
            self.assertLess(R.from_matrix(previous.T @ rot).magnitude(), np.deg2rad(1.001))
            previous = rot
        np.testing.assert_allclose(rot, np.eye(3), atol=1e-12)

    def test_control_rate_invariance_and_no_new_samples(self):
        results = []
        for hz in (30, 60, 120):
            obj = self.make(alpha=.05, deadband=False)
            obj.update([.03, 0, 0], R.from_euler("z", .8).as_quat(), 0.)
            for i in range(1, hz+1):
                result = obj.target(i/hz)
            results.append(result)
        for p, rot in results[1:]:
            np.testing.assert_allclose(p, results[0][0], atol=1e-12)
            np.testing.assert_allclose(rot, results[0][1], atol=1e-12)

    def test_dropped_samples_preserve_final_target(self):
        final = []
        for stride in (1, 3, 17):
            obj = self.make(alpha=.4, deadband=False)
            for i in range(1, 301):
                if i % stride == 0 or i == 300:
                    obj.update([i*.0001, 0, 0], R.from_euler("x", i*.001).as_quat(), i/60)
                obj.target(i/60)
            for i in range(301, 421):
                result = obj.target(i/60)
            final.append(result)
        for p, rot in final:
            np.testing.assert_allclose(p, [.03, 0, 0], atol=1e-12)
            np.testing.assert_allclose(rot, R.from_euler("x", .3).as_matrix(), atol=1e-12)

    def test_stationary_noise_and_independent_rotation_deadband(self):
        obj = self.make()
        rng = np.random.default_rng(23)
        for i in range(1, 1001):
            obj.update(rng.uniform(-.0001, .0001, 3),
                       R.from_rotvec(rng.uniform(-.001, .001, 3)).as_quat(), i/60)
            p, rot = obj.target(i/60)
            np.testing.assert_array_equal(p, np.zeros(3))
            np.testing.assert_array_equal(rot, np.eye(3))
        obj.update([.03, 0, 0], R.from_rotvec([.001, 0, 0]).as_quat(), 17.)
        p, rot = obj.target(17.)
        self.assertGreater(p[0], .02)
        np.testing.assert_array_equal(rot, np.eye(3))

    def test_noncommuting_rotations_and_nonzero_anchor(self):
        for frame in ("tool", "base_link"):
            basis = R.from_euler("xyz", [.2, .7, -.8]).as_matrix()
            obj = self.make(deadband=False, frame=frame, mapping=basis, signs=[1, -1, -1])
            h0 = R.from_euler("xyz", [.8, -.3, .1]).as_matrix()
            tcp0 = R.from_euler("xyz", [-.4, .3, .6]).as_matrix()
            obj.start([.2, .1, -.1], R.from_matrix(h0).as_quat(), [.1, .2, .3], tcp0, 0.)
            h = R.from_euler("xyz", [1., .5, -.7]).as_matrix()
            obj.update([.3, .1, -.1], R.from_matrix(h).as_quat(), 1.)
            p, rot = obj.target(1.)
            c = np.diag([1, -1, -1]) @ (np.eye(3) if frame == "tool" else basis.T)
            expected = (tcp0 @ c @ h0.T @ h @ c.T if frame == "tool"
                        else c @ h @ h0.T @ c.T @ tcp0)
            np.testing.assert_allclose(rot, expected, atol=1e-12)
            np.testing.assert_allclose(p, [.1, .2, .3] + basis.T @ np.array([.1, 0, 0]))

    def test_invalid_input_rejected_without_corrupting_anchor(self):
        obj = self.make()
        for quat in ([0, 0, 0, 0], [0, 0, np.nan, 1], [0, 0, 1]):
            with self.assertRaises(ValueError):
                obj.update([0, 0, 0], quat, 1.)
        with self.assertRaises(ValueError):
            self.make(signs=[-1, 1, 1])
        with self.assertRaises(ValueError):
            self.make(frame="base_link", mapping=np.diag([2, 1, 1]))
        obj.update([.02, 0, 0], [0, 0, 0, 1], 1.)
        np.testing.assert_allclose(obj.target(1.)[0], [.02, 0, 0])


if __name__ == "__main__":
    unittest.main()

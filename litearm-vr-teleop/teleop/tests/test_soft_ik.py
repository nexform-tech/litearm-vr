"""Bounded partial task tracking at singularities and joint limits."""
import unittest

import numpy as np

from litearm_teleop.exact_ik import bounded_task_velocity


class SoftIKTests(unittest.TestCase):
    def test_rank_loss_preserves_reachable_direction(self):
        J = np.array([[1., 0.], [0., 0.]])
        desired = np.array([.4, .7])
        velocity = bounded_task_velocity(J, desired, [-1., -1.], [1., 1.])
        self.assertTrue(np.all(np.isfinite(velocity)))
        self.assertGreater(velocity[0], .39)
        self.assertAlmostEqual(velocity[1], 0., places=10)
        self.assertLess(np.linalg.norm(J @ velocity - desired), np.linalg.norm(desired))

    def test_excessive_task_speed_saturates_without_freezing_other_axes(self):
        velocity = bounded_task_velocity(np.eye(2), [10., .2], [-.3, -.4], [.3, .4])
        np.testing.assert_allclose(velocity, [.3, .2], atol=.001)
        self.assertTrue(np.all(velocity <= [.3, .4]))
        self.assertTrue(np.all(velocity >= [-.3, -.4]))

    def test_fixed_joint_bounds_leave_other_joints_available(self):
        J = np.array([[1., 1., 0.], [0., 0., 1.]])
        velocity = bounded_task_velocity(J, [.5, -.2], [0., -1., -.3], [0., 1., .3])
        self.assertEqual(velocity[0], 0.)
        np.testing.assert_allclose(J @ velocity, [.5, -.2], atol=.001)

    def test_all_fixed_bounds_return_finite_fixed_velocity(self):
        velocity = bounded_task_velocity(np.eye(2), [1., 2.], [0., 0.], [0., 0.])
        np.testing.assert_array_equal(velocity, [0., 0.])

    def test_near_singular_direction_is_damped_without_losing_good_axis(self):
        velocity = bounded_task_velocity(np.diag([1., 1e-8]), [.2, .2], [-1., -1.], [1., 1.])
        self.assertGreater(velocity[0], .19)
        self.assertLess(abs(velocity[1]), .001)
        self.assertTrue(np.all(np.isfinite(velocity)))

    def test_joint_limit_allows_retreat_but_blocks_outward_velocity(self):
        outward = bounded_task_velocity(np.eye(2), [1., .2], [-.5, -.5], [0., .5])
        self.assertLessEqual(outward[0], 0.)
        self.assertGreater(outward[1], .19)
        inward = bounded_task_velocity(np.eye(2), [-.2, .2], [-.5, -.5], [0., .5])
        self.assertLess(inward[0], -.19)


if __name__ == "__main__":
    unittest.main()

"""Real PyBullet DIRECT integration regressions for anchored TCP tracking.

Run with PYTHONPATH=src python -m unittest discover -s tests
-p test_anchored_tracking.py. No wall-clock sleeps, ROS, sockets or hardware.
"""
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation as R

from litearm_teleop.teleop_sim import LiteArmTeleopSim, Tuning


IDENTITY_QUAT = [0., 0., 0., 1.]
DT = 1. / 60.


class AnchoredTrackingTests(unittest.TestCase):
    def setUp(self):
        self.sims = []

    def tearDown(self):
        for sim in self.sims:
            sim.close()

    def make(self, dual=False, tracking=None, backend="pybullet", **tuning_args):
        # Long replay horizons deliberately disable unrelated timeout guards;
        # timeout behavior has a separate test with production defaults.
        limits = dict(input_timeout=100., stall_timeout=100.)
        limits.update(tracking or {})
        tune = Tuning(tracking=limits, **tuning_args)
        sim = LiteArmTeleopSim(viewer="rviz", dual=dual, tuning=tune, ik_backend=backend)
        self.sims.append(sim)
        return sim, next(iter(sim.arms().values()))

    @staticmethod
    def tcp(arm):
        p, q = arm._eef_pose()
        rot = R.from_quat(q).as_matrix()
        return p + rot @ arm._tcp_offset, rot

    def assert_tracking(self, arm, pos_tol=.0012, rot_tol=.012):
        self.assertIsNone(arm._hold_reason)
        actual_p, actual_R = self.tcp(arm)
        wanted_p, wanted_R = arm._target.raw_target()
        self.assertLess(np.linalg.norm(actual_p - wanted_p), pos_tol)
        self.assertLess(R.from_matrix(wanted_R @ actual_R.T).magnitude(), rot_tol)

    def test_exact_backend_tracks_persistent_pose(self):
        _, arm = self.make(backend="exact")
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        for i in range(1, 361):
            arm.update_pose([min(i, 100)*.0001, 0, 0], IDENTITY_QUAT, i*DT)
            arm.tick(i*DT)
        self.assert_tracking(arm)

    def test_pin_backend_tracks_persistent_pose(self):
        try:
            import pink
        except ImportError:
            self.skipTest("optional pin-pink not installed")
        _, arm = self.make(backend="pin", manipulability_cost=0.)
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        for i in range(1, 241):
            arm.update_pose([min(i, 60)*.0001, 0, 0], IDENTITY_QUAT, i*DT)
            arm.tick(i*DT)
        self.assert_tracking(arm)

    def test_stalled_solver_stays_active_and_recovers_without_new_grip(self):
        _, arm = self.make(tracking={"stall_timeout": .2})
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        solver = arm._step_pybullet
        arm._step_pybullet = lambda *args: None  # model an infeasible stationary solver
        for i in range(1, 31):
            arm.update_pose([.02, 0, 0], IDENTITY_QUAT, i*DT)
            arm.tick(i*DT)
        self.assertTrue(arm._active)
        self.assertIsNone(arm._hold_reason)
        wanted = arm._target.raw_target()[0]
        before = np.linalg.norm(self.tcp(arm)[0] - wanted)
        self.assertGreater(before, .015)
        # A feasible solve returns without a grip edge or anchor reset.
        arm._step_pybullet = solver
        for i in range(31, 271):
            arm.update_pose([.02, 0, 0], IDENTITY_QUAT, i*DT)
            arm.tick(i*DT)
        self.assertTrue(arm._active)
        self.assert_tracking(arm)

    def test_exact_failure_uses_bounded_motion_then_recovers(self):
        from litearm_teleop.exact_ik import TaskVelocitySolution, FAIL_BOX

        max_v, max_a = .12, .35
        _, arm = self.make(backend="exact", tracking={
            "max_joint_velocity": max_v, "max_joint_acceleration": max_a})
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        initial_q = previous_q = arm._get_q().copy()
        previous_v = np.zeros_like(previous_q)
        failed = TaskVelocitySolution(np.zeros_like(previous_q), failure=FAIL_BOX)
        with patch("litearm_teleop.teleop_sim.exact_task_velocity", return_value=failed):
            for i in range(1, 31):
                arm.update_pose([.01, 0, 0], IDENTITY_QUAT, i*DT)
                arm.tick(i*DT)
                current_q = arm._get_q()
                velocity = (current_q-previous_q)/DT
                self.assertLessEqual(np.max(np.abs(velocity)), max_v+1e-9)
                self.assertLessEqual(np.max(np.abs(velocity-previous_v))/DT, max_a+1e-7)
                previous_q, previous_v = current_q.copy(), velocity
                self.assertTrue(arm._active)
                self.assertIsNone(arm._hold_reason)
        self.assertGreater(np.linalg.norm(previous_q-initial_q), .001)
        for i in range(31, 361):
            arm.update_pose([.01, 0, 0], IDENTITY_QUAT, i*DT)
            arm.tick(i*DT)
        self.assert_tracking(arm)

    def test_pin_qp_failure_uses_bounded_partial_motion(self):
        _, arm = self.make()

        class FailedPin:
            def solve(self, seed, pos, rotation, dt, info):
                return seed.copy(), False

        arm._pin = FailedPin()
        before_q = arm._get_q().copy()
        before_p, before_R = self.tcp(arm)
        flange_p, _ = arm._eef_pose()
        arm._step_pin(flange_p + np.array([.001, 0., 0.]), before_R, DT)
        delta = arm._get_q() - before_q
        self.assertTrue(np.all(np.isfinite(delta)))
        self.assertGreater(np.linalg.norm(delta), 1e-6)
        self.assertLess(np.linalg.norm(self.tcp(arm)[0] - (before_p + [.001, 0, 0])), .001)
        self.assertLessEqual(np.max(np.abs(delta))/DT, arm._cfg.max_joint_vel + 1e-9)

    def test_operator_lead_does_not_hold_or_reanchor(self):
        # Exceed both old trip thresholds while keeping each input sample valid.
        _, arm = self.make(tracking={"max_position_error": .002,
                                     "max_rotation_error": .01})
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        anchor_p, anchor_R = arm._target.raw_target()
        measured = arm._get_q().copy()
        for i in range(1, 121):
            amount = min(i/10., 1.)
            hand_q = R.from_rotvec([0, 0, .06*amount]).as_quat()
            arm.update_pose([.02*amount, 0, 0], hand_q, i*DT)
            # Perfect execution of previous limited command; target can lead it.
            measured = arm._command_q.copy()
            arm.tick(i*DT, measured_q=measured)
            self.assertTrue(arm._active)
        for i in range(121, 421):
            arm.update_pose([0, 0, 0], IDENTITY_QUAT, i*DT)
            arm.tick(i*DT, measured_q=arm._command_q.copy())
        np.testing.assert_allclose(arm._target.raw_target()[0], anchor_p, atol=1e-12)
        np.testing.assert_allclose(arm._target.raw_target()[1], anchor_R, atol=1e-12)
        self.assert_tracking(arm)

    def test_real_execution_error_still_holds(self):
        _, arm = self.make(tracking={"max_position_error": .002,
                                     "max_rotation_error": .01})
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        measured = arm._command_q.copy()
        measured[0] += .1
        arm.tick(DT, measured_q=measured)
        self.assertFalse(arm._active)
        self.assertIn("实机执行偏差", arm._hold_reason)

    def test_downstream_wrist_slew_is_not_misclassified_as_execution_fault(self):
        sim, _ = self.make(dual=True)
        arm = sim.arms()["right"]
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        measured = arm._command_q.copy()
        downstream_velocity = np.zeros(7)
        hardware_lower = np.array([-2.949607, -1.797689, -2.80998, -3.07178,
                                   -2.80998, -1.553343, -1.553343])
        hardware_upper = np.array([2.949607, 1.797689, 2.80998, .593412,
                                   2.80998, 1.553343, 1.553343])

        # Reproduce the STM32 follow server's 2 rad/s, 3 rad/s^2 downstream
        # trapezoidal slew while the operator pitches the wrist quickly.
        for i in range(1, 181):
            angle = min(i*DT*3., 2.2)
            hand_q = R.from_rotvec([0., angle, 0.]).as_quat()
            arm.update_pose([0, 0, 0], hand_q, i*DT)
            target = np.clip(arm._command_q, hardware_lower, hardware_upper)
            for joint in range(7):
                diff = target[joint]-measured[joint]
                dv = 3.*DT
                stop = downstream_velocity[joint]**2/(2.*3.)
                desired = (0. if diff*downstream_velocity[joint] > 0.
                           and abs(diff) <= stop else
                           (np.copysign(2., diff) if diff else 0.))
                downstream_velocity[joint] = np.clip(
                    downstream_velocity[joint]
                    + np.clip(desired-downstream_velocity[joint], -dv, dv), -2., 2.)
                step = downstream_velocity[joint]*DT
                if diff*step > 0. and abs(step) >= abs(diff):
                    measured[joint] = target[joint]
                    downstream_velocity[joint] = 0.
                else:
                    measured[joint] += step
            arm.tick(i*DT, measured_q=measured, applied_q=measured)
            self.assertTrue(arm._active, arm._hold_reason)

        # Moving the hand back must remain recoverable without a release/regrip.
        for i in range(181, 481):
            amount = max(0., 1.-(i-180)/120.)
            hand_q = R.from_rotvec([0., 2.2*amount, 0.]).as_quat()
            arm.update_pose([0, 0, 0], hand_q, i*DT)
            arm.tick(i*DT, measured_q=measured, applied_q=measured)
            measured = arm._command_q.copy()
        self.assertTrue(arm._active)
        self.assertIsNone(arm._hold_reason)

    def test_confirmed_fast_hand_motion_keeps_original_anchor(self):
        _, arm = self.make()
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        initial_p, initial_R = arm._target.raw_target()
        hand_q = R.from_rotvec([0, 0, 1.]).as_quat()
        arm.update_pose([.2, 0, 0], hand_q, DT)
        self.assertTrue(arm._active)
        self.assertIsNone(arm._hold_reason)
        np.testing.assert_array_equal(arm._target.raw_target()[0], initial_p)
        np.testing.assert_array_equal(arm._target.raw_target()[1], initial_R)
        arm.update_pose([.21, 0, 0], hand_q, 2*DT)
        self.assertTrue(arm._active)
        expected = initial_p + arm._target.translation @ np.array([.21, 0, 0])
        np.testing.assert_allclose(arm._target.raw_target()[0], expected, atol=1e-12)
        self.assertGreater(R.from_matrix(arm._target.raw_target()[1] @ initial_R.T).magnitude(), .9)
        np.testing.assert_array_equal(arm._target.hand_p0, [0, 0, 0])
        np.testing.assert_array_equal(arm._target.tcp_p0, initial_p)
        # Confirming a large target must still respect the command speed limit.
        old_q = arm._command_q.copy()
        arm.tick(2*DT)
        speed = np.max(np.abs(arm._command_q-old_q))/(2*DT)
        self.assertLessEqual(speed, arm._tuning.tracking["max_joint_velocity"] + 1e-9)

    def test_one_frame_pose_spike_does_not_change_target_or_latch(self):
        _, arm = self.make()
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        initial_p, initial_R = arm._target.raw_target()
        arm.update_pose([.2, 0, 0], R.from_rotvec([0, 0, 1.]).as_quat(), DT)
        arm.tick(DT)
        np.testing.assert_array_equal(arm._target.raw_target()[0], initial_p)
        np.testing.assert_array_equal(arm._target.raw_target()[1], initial_R)
        arm.update_pose([0, 0, 0], IDENTITY_QUAT, 2*DT)
        arm.tick(2*DT)
        self.assertTrue(arm._active)
        self.assertIsNone(arm._hold_reason)
        np.testing.assert_array_equal(arm._target.raw_target()[0], initial_p)
        np.testing.assert_array_equal(arm._target.raw_target()[1], initial_R)

    def test_rotation_keeps_tcp_stationary(self):
        _, arm = self.make()
        before, _ = self.tcp(arm)
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        for i in range(1, 241):
            quat = R.from_rotvec([0, 0, .1*min(i/60, 1)]).as_quat()
            arm.update_pose([0, 0, 0], quat, i*DT)
            arm.tick(i*DT)
        after, _ = self.tcp(arm)
        self.assertLess(np.linalg.norm(after-before), .0012)
        self.assert_tracking(arm)

    def test_default_pybullet_slow_motion_reaches_absolute_target(self):
        _, arm = self.make()
        initial, _ = self.tcp(arm)
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        # 0.1 mm/frame was previously swallowed by incremental deadband logic.
        for i in range(1, 301):
            arm.update_pose([i*.0001, 0, 0], IDENTITY_QUAT, i*DT)
            arm.tick(i*DT)
        for i in range(301, 601):
            arm.tick(i*DT)
        final, _ = self.tcp(arm)
        self.assertGreater(final[0] - initial[0], .028)
        self.assert_tracking(arm)

    def test_fast_out_slow_return_position_and_rotation_close(self):
        _, arm = self.make()
        initial_p, initial_R = self.tcp(arm)
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        tick = 0
        # Noncommuting three-axis orientations exercise complete rotation targets.
        # Repeat in one grip so no re-anchoring can conceal accumulated errors.
        for _ in range(10):
            for amount in np.r_[1., np.ones(90), np.linspace(1., 0., 180), np.zeros(90)]:
                tick += 1
                pos = np.array([.012, .003, -.003])*amount
                quat = R.from_euler("xyz", np.array([.06, -.04, .05])*amount).as_quat()
                arm.update_pose(pos, quat, tick*DT)
                arm.tick(tick*DT)
            self.assert_tracking(arm)
            actual_p, actual_R = self.tcp(arm)
            self.assertLess(np.linalg.norm(actual_p-initial_p), .0012)
            self.assertLess(R.from_matrix(initial_R @ actual_R.T).magnitude(), .012)

    def test_limited_joint_motion_keeps_target_and_continues_without_new_input(self):
        _, arm = self.make(tracking={"max_joint_velocity": .05,
                                     "max_joint_acceleration": .2})
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        arm.update_pose([.025, 0, 0], IDENTITY_QUAT, DT)
        saved_p, saved_R = arm._target.raw_target()
        arm.tick(DT)
        first_p, _ = self.tcp(arm)
        first_error = np.linalg.norm(saved_p-first_p)
        self.assertGreater(first_error, .02)
        # Only the control clock advances: IK progress must not overwrite target.
        for i in range(2, 1201):
            arm.tick(i*DT)
        final_p, _ = self.tcp(arm)
        self.assertLess(np.linalg.norm(saved_p-final_p), first_error/10)
        np.testing.assert_array_equal(arm._target.raw_target()[0], saved_p)
        np.testing.assert_array_equal(arm._target.raw_target()[1], saved_R)
        self.assert_tracking(arm)

    def test_release_freezes_and_new_grip_reanchors_without_jump(self):
        _, arm = self.make()
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        arm.update_pose([.03, 0, 0], IDENTITY_QUAT, DT)
        for i in range(1, 6):
            arm.tick(i*DT)
        arm.pause()
        frozen_q = arm._get_q().copy()
        for i in range(6, 121):
            arm.update_pose([.06, 0, 0], IDENTITY_QUAT, i*DT)
            arm.tick(i*DT)
        np.testing.assert_array_equal(arm._get_q(), frozen_q)
        p, rot = self.tcp(arm)
        arm.start([.06, 0, 0], IDENTITY_QUAT, 2.1)
        np.testing.assert_allclose(arm._target.raw_target()[0], p, atol=1e-12)
        np.testing.assert_allclose(arm._target.raw_target()[1], rot, atol=1e-12)
        arm.tick(2.1+DT)
        np.testing.assert_allclose(arm._get_q(), frozen_q, atol=1e-6)

    def test_timeout_holds_and_fresh_pose_alone_does_not_resume(self):
        _, arm = self.make(tracking={"input_timeout": .25})
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        arm.update_pose([.02, 0, 0], IDENTITY_QUAT, DT)
        arm.tick(DT)
        arm.tick(.3)
        self.assertFalse(arm._active)
        self.assertIn("超时", arm._hold_reason)
        frozen_q = arm._get_q().copy()
        arm.update_pose([.02, 0, 0], IDENTITY_QUAT, .31)
        arm.tick(.31)
        np.testing.assert_array_equal(arm._get_q(), frozen_q)
        arm.start([.02, 0, 0], IDENTITY_QUAT, .32)
        self.assertTrue(arm._active)
        self.assertIsNone(arm._hold_reason)

    def test_joint_velocity_and_acceleration_bounds_during_reversal(self):
        max_v, max_a = .12, .35
        _, arm = self.make(tracking={"max_joint_velocity": max_v,
                                     "max_joint_acceleration": max_a})
        arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        previous_q = arm._get_q().copy()
        previous_velocity = np.zeros(7)
        for i in range(1, 901):
            x = .025 if i < 300 else (-.015 if i < 600 else 0.)
            arm.update_pose([x, 0, 0], IDENTITY_QUAT, i*DT)
            arm.tick(i*DT)
            q = arm._get_q()
            velocity = (q-previous_q)/DT
            acceleration = (velocity-previous_velocity)/DT
            self.assertLessEqual(np.max(np.abs(velocity)), max_v+1e-9)
            self.assertLessEqual(np.max(np.abs(acceleration)), max_a+1e-7)
            previous_q, previous_velocity = q.copy(), velocity
        self.assertIsNone(arm._hold_reason)

    def test_dual_arms_targets_and_joint_states_are_independent(self):
        sim, _ = self.make(dual=True)
        left, right = sim.arms()["left"], sim.arms()["right"]
        for arm in (left, right):
            arm.start([0, 0, 0], IDENTITY_QUAT, 0.)
        right_q = right._get_q().copy()
        right_p, right_R = right._target.raw_target()
        for i in range(1, 301):
            left.update_pose([min(i, 120)*.0001, 0, 0], IDENTITY_QUAT, i*DT)
            sim.tick(i*DT)
        np.testing.assert_allclose(right._get_q(), right_q, atol=1e-6)
        np.testing.assert_array_equal(right._target.raw_target()[0], right_p)
        np.testing.assert_array_equal(right._target.raw_target()[1], right_R)
        self.assert_tracking(left)
        self.assert_tracking(right)
        # Releasing left must not stop right's control state.
        left.pause()
        self.assertFalse(left._active)
        self.assertTrue(right._active)

    def test_smooth_homing_returns_to_home_smoothly(self):
        sim, arm = self.make()
        # Offset joints away from home
        q_offset = np.array([0.3, -0.2, 0.4, -0.3, 0.2, -0.4, 0.1])
        arm._set_q(arm._home + q_offset)
        self.assertFalse(np.allclose(arm._get_q(), arm._home, atol=1e-3))

        # Start homing
        started = arm.start_homing(now=0.0)
        self.assertTrue(started)
        self.assertTrue(arm.is_homing())

        # Step through homing duration
        duration = arm._homing_duration
        prev_q = arm._get_q().copy()
        for i in range(1, int(duration / DT) + 15):
            sim.tick(i * DT)
            q = arm._get_q()
            vel = (q - prev_q) / DT
            # Check velocities are bounded and finite
            self.assertLessEqual(np.max(np.abs(vel)), 3.0)
            prev_q = q.copy()

        # After duration finishes, arm is exactly home and homing is complete
        self.assertFalse(arm.is_homing())
        np.testing.assert_allclose(arm._get_q(), arm._home, atol=1e-4)


if __name__ == "__main__":
    unittest.main()

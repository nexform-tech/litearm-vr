"""Grip-anchored TCP targets, independent of input rate and IK progress.

Quaternions use scipy's xyzw convention. Times are monotonic seconds from one
clock. Call update only for new samples, target on every control tick. Deadbands
are spatial tolerances (metres/radians), never per-frame motion thresholds.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def _finite(value, shape, name):
    a = np.asarray(value, dtype=float)
    if a.shape != shape or not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must be finite with shape {shape}")
    return a.copy()


def _orthogonal(value, name, proper=False):
    a = _finite(value, (3, 3), name)
    if not np.allclose(a.T @ a, np.eye(3), atol=1e-7, rtol=0):
        raise ValueError(f"{name} must be orthogonal")
    if proper and not np.isclose(np.linalg.det(a), 1., atol=1e-7):
        raise ValueError(f"{name} must be a proper SO(3) rotation")
    return a


def _hand(pos, quat):
    p = _finite(pos, (3,), "hand_pos")
    q = _finite(quat, (4,), "hand_quat")
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("hand_quat must have nonzero finite norm")
    return p, Rotation.from_quat(q / norm).as_matrix()


def _time(now):
    now = float(now)
    if not np.isfinite(now):
        raise ValueError("time must be finite")
    return now


class AnchoredPoseTarget:
    def __init__(self, tuning, translation_remap, axis_remap, rotation_signs,
                 control_frame="tool"):
        if control_frame not in ("tool", "base_link"):
            raise ValueError("control_frame must be tool or base_link")
        self.control_frame = control_frame
        self.translation = _finite(translation_remap, (3, 3), "translation_remap").T
        axis = _orthogonal(axis_remap, "axis_remap")
        signs = _finite(rotation_signs, (3,), "rotation_signs")
        sign_rotation = _orthogonal(np.diag(signs), "rotation_signs", proper=True)
        # Conjugation, unlike component-wise Euler changes, preserves composition
        # and stays continuous when the relative hand rotation crosses 180 degrees.
        basis = (axis.T if control_frame == "tool" else
                 _orthogonal(self.translation, "base_link translation_remap"))
        self.rotation_map = sign_rotation @ basis
        self.alpha = float(tuning.smooth_alpha)
        if not np.isfinite(self.alpha) or not 0 < self.alpha <= 1:
            raise ValueError("smooth_alpha must be in (0, 1]")
        self.t_in, self.t_out, self.r_in, self.r_out = [
            float(getattr(tuning, key)) for key in
            ("deadband_t_in", "deadband_t_out", "deadband_r_in", "deadband_r_out")]
        if (not np.all(np.isfinite([self.t_in, self.t_out, self.r_in, self.r_out]))
                or not 0 <= self.t_in <= self.t_out
                or not 0 <= self.r_in <= self.r_out):
            raise ValueError("deadband thresholds must satisfy 0 <= in <= out")
        self.started = False

    def start(self, hand_pos, hand_quat, tcp_pos, tcp_R, now):
        hp, hr = _hand(hand_pos, hand_quat)
        tp = _finite(tcp_pos, (3,), "tcp_pos")
        tr = _orthogonal(tcp_R, "tcp_R", proper=True)
        now = _time(now)
        self.hand_p0, self.hand_R0 = hp, hr
        self.tcp_p0, self.tcp_R0 = tp, tr
        self._raw_p, self._filtered_p, self._output_p = tp.copy(), tp.copy(), tp.copy()
        self._raw_R, self._filtered_R, self._output_R = tr.copy(), tr.copy(), tr.copy()
        self._last_tick = self._last_sample = now
        self._moving_t = self._moving_r = False
        self.started = True

    def update(self, hand_pos, hand_quat, now):
        self._require_start()
        hp, hr = _hand(hand_pos, hand_quat)
        now = _time(now)
        if now < self._last_sample:
            raise ValueError("sample times must not go backwards")
        relative = (self.hand_R0.T @ hr if self.control_frame == "tool"
                    else hr @ self.hand_R0.T)
        mapped = self.rotation_map @ relative @ self.rotation_map.T
        self._raw_p = self.tcp_p0 + self.translation @ (hp - self.hand_p0)
        self._raw_R = (self.tcp_R0 @ mapped if self.control_frame == "tool"
                       else mapped @ self.tcp_R0)
        self._last_sample = now

    def raw_target(self):
        self._require_start()
        return self._raw_p.copy(), self._raw_R.copy()

    def target(self, now):
        self._require_start()
        now = _time(now)
        if now < self._last_tick:
            raise ValueError("control times must not go backwards")
        dt = now - self._last_tick
        self._last_tick = now
        # Preserve the old alpha's meaning at 60 Hz, independent of control rate.
        weight = (1. if self.alpha == 1 else -np.expm1(60 * dt * np.log1p(-self.alpha)))
        if dt == 0:
            weight = 0.
        self._filtered_p += weight * (self._raw_p - self._filtered_p)
        rv = Rotation.from_matrix(self._filtered_R.T @ self._raw_R).as_rotvec()
        self._filtered_R = self._filtered_R @ Rotation.from_rotvec(weight * rv).as_matrix()
        # Compare complete poses with the held output; never discard accumulated
        # subthreshold input. Translation and orientation have separate latches.
        et = np.linalg.norm(self._filtered_p - self._output_p)
        er = np.linalg.norm(Rotation.from_matrix(self._output_R.T @ self._filtered_R).as_rotvec())
        self._moving_t = et > (self.t_in if self._moving_t else self.t_out)
        self._moving_r = er > (self.r_in if self._moving_r else self.r_out)
        if self._moving_t:
            self._output_p = self._filtered_p.copy()
        if self._moving_r:
            self._output_R = self._filtered_R.copy()
        return self._output_p.copy(), self._output_R.copy()

    def _require_start(self):
        if not self.started:
            raise RuntimeError("start must establish a grip anchor first")

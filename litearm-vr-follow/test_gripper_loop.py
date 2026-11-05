#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`gripper_server.py` **控制环**的回归测试（纯 stdlib，不需要硬件/SDK/CAN）。

守的是「照 litegrip-cpp 的 ControlLoop 重写」之后必须成立的四条不变量
（分析见 `docs/gripper-control-loop-rewrite.md`）：

  1. **帧里要求的力不超过力矩预算**：`kp × (q_frame − 实测)` ≤ `--force × 0.1` Nm。
     这是「夹持力就是预算」的全部依据；破了它就会退回「顶死硬限位」那类抖动。
  2. **命令轨迹按 `--speed` 限速推进**，且**目标中途改变时立刻反向**（不需要等一个分片）。
  3. **没有目标时保持 limp**：不使能、不发帧（不是发零增益帧 —— 那是「别管我」的语义）。
  4. **掉线 / 过温 / 电机故障 → 发零增益帧**（不是停流：停流 ~100ms 电机即自锁成 0xD）。

用法: python3 test_gripper_loop.py
"""
import importlib.util
import math
import os
import sys
import threading
import time
import types
from dataclasses import dataclass

# ── 假 SDK ───────────────────────────────────────────────────────────────
# 只替身 gripper_server 用到的入口；`send_mit_frame` 把每一帧记下来供断言。

FRAMES = []            # [(t, q, kp, kd, measured)]，按发送顺序
_LOCK = threading.Lock()


def _clear():
    with _LOCK:
        FRAMES.clear()


def _frames():
    with _LOCK:
        return list(FRAMES)


@dataclass
class _State:
    #: ⚠ 从**张开端**起步：本机标定是「闭合 0.0 / 张开 -1.06」，而 close 是最常用的命令。
    #: 若默认停在 0.0（= 闭合目标），`close` 会无事可做 —— 用例会变成空断言（曾经如此）。
    position_rad: float = -1.06
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0
    temperature_mos: int = 30
    temperature_coil: int = 30
    error_code: int = 1              # 1 = 已使能


@dataclass
class _Cfg:
    """GripperConfig 替身：只需标定与换算常量。"""

    calibrated: bool = True
    mount: str = "normal"
    pos_closed_rad: float = 0.0
    pos_open_rad: float = -1.06
    rad_to_mm: float = 65.0


@dataclass
class _Enable:
    ok: bool = True
    tries: int = 1


class _FakeLiteGrip:
    """被控对象的开关（类属性，测试直接拧）。"""

    FOLLOWS = True          # True = 电机跟着指令走；False = 焊死（模拟撞上工件/限位）
    FEEDING = True          # False = 不再有新反馈帧（模拟掉线）
    FOLLOW_STEP = 0.05      # 每帧最多跟随多少 rad

    def __init__(self, **kw):
        self.config = _Cfg()
        self._st = _State()
        self.enable_calls = 0

    # ---- 连接 / 使能 ----
    def load_calibration(self, path=None):
        return None

    def connect(self):
        return True

    def disconnect(self):
        return None

    def disable(self):
        self._st.error_code = 0
        return True

    def enable(self, retries=None):
        self.enable_calls += 1
        self._st.error_code = 1
        return _Enable()

    def poll(self, timeout_s=0.0):
        if not self.FEEDING:
            return False
        time.sleep(min(timeout_s, 0.0005))
        return True

    def get_state(self, wait=True):
        return self._st

    # ---- 发帧（被断言的那一个）----
    def send_mit_frame(self, q, kp, kd, dq=0.0, tau=0.0):
        if self._st.error_code != 1:
            return False            # 与 SDK 同：未使能拒发
        measured = self._st.position_rad
        with _LOCK:
            FRAMES.append((time.monotonic(), q, kp, kd, measured))
        if self.FOLLOWS and kp > 0.0:
            err = q - measured
            step = max(-self.FOLLOW_STEP, min(self.FOLLOW_STEP, err))
            self._st.position_rad += step
            self._st.torque_nm = kp * (q - self._st.position_rad)
        return True


class _FakeLiteGripError(Exception):
    pass


def _install_fake_sdk():
    if "litegrip" in sys.modules:
        return
    pkg = types.ModuleType("litegrip")
    pkg.LiteGrip = _FakeLiteGrip
    pkg.LiteGripError = _FakeLiteGripError
    teleop = types.ModuleType("litegrip.teleop")
    teleop.rad_to_openness = lambda rad, cfg: 0.0
    pkg.teleop = teleop
    sys.modules["litegrip"] = pkg
    sys.modules["litegrip.teleop"] = teleop


def _load_server():
    _install_fake_sdk()
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gripper_server.py")
    spec = importlib.util.spec_from_file_location("gripper_server_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["gripper_server_under_test"] = mod
    spec.loader.exec_module(mod)
    return mod


_GS = _load_server()

FORCE_N = 20.0
BUDGET_NM = FORCE_N * _GS.N_TO_NM          # 2.0 Nm


def _make_control(**kw):
    """起一个环。默认不启停（启动动作有用例单独测）。"""
    kw.setdefault("can", "can0")
    kw.setdefault("speed", 50.0)
    kw.setdefault("force_n", FORCE_N)
    kw.setdefault("rate_hz", 300.0)
    kw.setdefault("startup_disable", False)
    kw.setdefault("startup_open", False)
    ctl = _GS.LiteGripperControl(**kw)
    time.sleep(0.05)
    return ctl


def _wait_for(pred, timeout=2.0, step=0.005):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(step)
    return False


def _worst_force():
    worst = 0.0
    for _, q, kp, _kd, measured in _frames():
        worst = max(worst, abs(kp * (q - measured)))
    return worst


def test_force_never_exceeds_budget():
    """不变量 1：**每一帧**里要求的力都不超过预算（焊死电机 = 撞上工件/限位）。

    ⚠ 先等到力**饱和**（轨迹领先实测超过 预算/kp 那一档），再断言全程不超预算 ——
    只断言「不超过」的话，「一发帧就不出力」也能通过，那正是撞死时最该防的相反错误。
    """
    _FakeLiteGrip.FOLLOWS = False           # 焊死：位置不动，误差会一直涨
    _clear()
    ctl = _make_control()
    try:
        ctl.close()
        assert _wait_for(lambda: len(_frames()) >= 30), "环没在发帧"
        assert _wait_for(lambda: _worst_force() >= 0.9 * BUDGET_NM, 3.0), \
            "撞死时帧里的力始终没到预算（最坏 %.3f / 预算 %.3f）" % (
                _worst_force(), BUDGET_NM)
        worst = _worst_force()
        assert worst <= BUDGET_NM + 1e-6, \
            "帧里要求的力矩 %.3f Nm 超过预算 %.3f Nm —— 夹持力就不再由 --force 定了" % (
                worst, BUDGET_NM)
        print("[1] 撞死时每帧的力 == 预算上限：%.3f Nm（预算 %.1f Nm）× %d 帧"
              % (worst, BUDGET_NM, len(_frames())))
    finally:
        ctl.shutdown()
        _FakeLiteGrip.FOLLOWS = True


def test_trajectory_rate_limited_and_reaches_target():
    """不变量 2a：命令轨迹按 --speed 限速推进，最终停在标定的一端。

    从张开端（-1.06）走到闭合端（0.0）= 68 mm；`--speed 50` ⇒ 约 1.36 s。
    """
    _clear()
    ctl = _make_control(speed=50.0)
    try:
        t0 = time.monotonic()
        ctl.close()
        # 轨迹在**下一拍**才由实测起步，所以这里等一拍再断言（别抢在环前面）。
        assert _wait_for(lambda: ctl.snapshot()["cmd_rad"] is not None, 1.0), "close 没投出目标"
        assert _wait_for(lambda: abs(ctl.snapshot()["cmd_rad"]) < 1e-3, 3.0), \
            "轨迹没走到闭合限位（停在 %r）" % ctl.snapshot()["cmd_rad"]
        took = time.monotonic() - t0
        traj = [f[1] for f in _frames()]
        assert len(traj) >= 100, "只发了 %d 帧，环没有持续跑" % len(traj)
        step_max = max(abs(b - a) for a, b in zip(traj, traj[1:]))
        # --speed 50 mm/s ÷ 65 mm/rad = 0.77 rad/s；测试周期 ~3.3ms ⇒ 单帧 ≲0.003 rad。
        # 留 10× 余量容忍调度抖动，但远小于「不限速时一帧走完 1.06 rad」。
        assert step_max <= 0.03, "单帧推进 %.4f rad 太大：限速没生效" % step_max
        assert took >= 0.8, "68mm 只用了 %.2fs —— 比 --speed 50mm/s 快，限速没生效" % took
        assert ctl.snapshot()["target_rad"] == 0.0, \
            "close 的目标应是标定闭合限位：%r" % ctl.snapshot()
        print("[2] 轨迹限速走到闭合限位：%.2fs、%d 帧、单帧最大 %.4f rad（不限速应是一帧 1.06）"
              % (took, len(traj), step_max))
    finally:
        ctl.shutdown()


def test_target_change_reverses_immediately():
    """不变量 2b：目标中途反向 ⇒ 命令轨迹**立刻**反向（没有「等这一片跑完」）。

    ⚠ 判据要看**轨迹**（`cmd_rad`）而不是「最后若干帧的 q」：假电机每帧能跟 0.05 rad，
    往返可能在几十毫秒内跑完，用帧尾部比较会变成「两边都在终点」的假象（曾如此）。
    """
    _clear()
    ctl = _make_control(speed=50.0)
    try:
        ctl.close()
        assert _wait_for(lambda: (ctl.snapshot()["cmd_rad"] or -9.0) > -0.96, 3.0), \
            "close 的轨迹没推进（cmd_rad=%r）" % ctl.snapshot()["cmd_rad"]
        ctl.open()                                   # 中途改目标
        t_rev = time.monotonic()
        assert _wait_for(lambda: (ctl.snapshot()["cmd_rad"] or 0.0) < -1.02, 1.0), \
            "目标改成 open 后轨迹没反向（cmd_rad=%r）" % ctl.snapshot()["cmd_rad"]
        took = time.monotonic() - t_rev
        # 反向后必须**朝张开侧**走（更负），且用时远小于一个「分片」级别的延迟。
        assert took < 0.2, "换向用了 %.3fs —— 比一拍长太多" % took
        print("[3] 目标中途反向：轨迹 %.3fs 内换向（老实现要等一个 ~0.75s 的分片）" % took)
    finally:
        ctl.shutdown()


def test_no_target_means_limp_and_silent():
    """不变量 3：还没收到目标时不使能、不发帧。"""
    _clear()
    ctl = _make_control()
    try:
        time.sleep(0.3)
        assert _frames() == [], "还没收到目标就发了帧：%r" % (_frames()[:3],)
        assert ctl.snapshot()["target_rad"] is None, "不该有目标"
        assert ctl.snapshot()["target_ratio"] is None, "没收到命令时 target_ratio 必须是 None"
        print("[4] 没有目标时保持 limp：0 帧、目标 None、target_ratio None")
    finally:
        ctl.shutdown()


def test_startup_open_posts_target_but_is_not_a_command():
    """启动归位投了目标，但**不算**收到命令（契约：target_ratio 是操作者意图）。"""
    _clear()
    ctl = _make_control(startup_open=True)
    try:
        assert _wait_for(lambda: ctl.snapshot()["target_rad"] is not None, 1.0)
        snap = ctl.snapshot()
        assert snap["target_rad"] == -1.06, snap
        assert snap["target_ratio"] is None, \
            "启动归位不该算「收到命令」：target_ratio=%r" % snap["target_ratio"]
        print("[5] 启动归位：投了张开目标 %+.2f rad，但 target_ratio 仍为 None"
              % snap["target_rad"])
    finally:
        ctl.shutdown()


def test_feedback_loss_streams_zero_gain_frames():
    """不变量 4a：掉线 → **继续发帧**但零增益（停流 ~100ms 电机即自锁成 0xD）。"""
    _clear()
    ctl = _make_control(feedback_timeout=0.05)
    try:
        ctl.close()
        assert _wait_for(lambda: len(_frames()) >= 5)
        _clear()
        _FakeLiteGrip.FEEDING = False        # 反馈断了
        time.sleep(0.3)
        tail = [f for f in _frames() if f[2] == 0.0 and f[3] == 0.0]
        assert tail, "掉线后没在发零增益帧（停流会把电机锁成 0xD）"
        assert ctl.snapshot()["fault_code"] == _GS.FAULT_NO_FEEDBACK, ctl.snapshot()
        assert ctl.snapshot()["fresh"] is False
        print("[6] 掉线：继续发帧但零增益（%d 帧），fault_code=%d"
              % (len(tail), _GS.FAULT_NO_FEEDBACK))
    finally:
        _FakeLiteGrip.FEEDING = True
        ctl.shutdown()


def test_overtemp_streams_zero_gain_frames():
    """不变量 4b：过温 → 零增益帧 + 对应故障码。"""
    _clear()
    ctl = _make_control(temp_limit=80)
    try:
        ctl.close()
        assert _wait_for(lambda: len(_frames()) >= 5)
        _clear()
        ctl._g._st.temperature_coil = 95          # 假 SDK 的反馈里报过温
        time.sleep(0.2)
        assert any(f[2] == 0.0 and f[3] == 0.0 for f in _frames()), "过温后没发零增益帧"
        assert ctl.snapshot()["fault_code"] == _GS.FAULT_OVERTEMP, ctl.snapshot()
        print("[7] 过温：零增益帧 + fault_code=%d" % _GS.FAULT_OVERTEMP)
    finally:
        ctl._g._st.temperature_coil = 30
        ctl.shutdown()


def test_enable_happens_before_motion():
    """未使能时先使能、那一拍不发运动帧（SDK 对未使能是拒发）。"""
    _clear()
    ctl = _make_control()
    try:
        ctl._g._st.error_code = 0            # 掉电重启后的状态：未使能
        ctl.close()
        assert _wait_for(lambda: ctl._g.enable_calls >= 1, 1.0), "没去使能"
        assert _wait_for(lambda: any(f[2] > 0.0 for f in _frames()), 1.0), "使能后没运动"
        print("[8] 未使能 → 先 enable(%d 次) 再运动" % ctl._g.enable_calls)
    finally:
        ctl.shutdown()


def test_uncalibrated_refuses_target():
    """未标定 → 拒绝投目标（占位限位不是真实行程），也不发帧。"""
    _clear()
    ctl = _make_control()
    try:
        ctl._g.config.calibrated = False
        ctl.close()
        time.sleep(0.2)
        assert ctl.snapshot()["target_rad"] is None, "未标定却投了目标"
        assert _frames() == [], "未标定却发了帧"
        print("[9] 未标定：拒绝投目标、0 帧（fail-closed）")
    finally:
        ctl._g.config.calibrated = True
        ctl.shutdown()


def test_status_snapshot_fields_still_present():
    """契约：老字段一个都不能少（status_msgs.parse_gripper_reply 认这些键）。"""
    ctl = _make_control()
    try:
        snap = ctl.snapshot()
        for key in ("q", "tau", "enabled", "error_code", "fresh", "position_mm",
                    "ratio", "target_ratio", "age_ms"):
            assert key in snap, "回包少了 %r" % key
        for key in ("t_mos", "t_coil", "fault_code", "target_rad", "cmd_rad"):
            assert key in snap, "新诊断字段少了 %r" % key
        assert math.isclose(snap["q"], -1.06, abs_tol=1e-9), \
            "缓存里的 q 应是假电机的实测位置，实际 %r" % snap["q"]
        print("[10] status 回包：老字段齐全 + 新诊断字段（fault/cmd_rad）就位")
    finally:
        ctl.shutdown()


def main():
    test_force_never_exceeds_budget()
    test_trajectory_rate_limited_and_reaches_target()
    test_target_change_reverses_immediately()
    test_no_target_means_limp_and_silent()
    test_startup_open_posts_target_but_is_not_a_command()
    test_feedback_loss_streams_zero_gain_frames()
    test_overtemp_streams_zero_gain_frames()
    test_enable_happens_before_motion()
    test_uncalibrated_refuses_target()
    test_status_snapshot_fields_still_present()
    print("GRIPPER LOOP TESTS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())

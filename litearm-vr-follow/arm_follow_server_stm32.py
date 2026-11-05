#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiteArm 七轴真机遥操跟随服务 (STM32 直连版)。

链路:
  VR(ws_echo_server.py) --TCP/JSON--> 本服务 --串口 0xA5--> STM32(litearm-stm32) --CAN--> 机械臂

串口这一层用 **litearm SDK**（`github/litearm/litearm-python`，包名 `litearm`，唯一依赖
pyserial）。SDK 自带读线程与状态解码，故本服务不再自带 `arm_console.py` 那套收发/解码。

对外协议与 arm_follow_server.py 完全一致 (VR 端无需改动):
  {"cmd":"home"}        回零位 (CMD_HOME, 阻塞到到位)
  {"cmd":"movej","q":[7]} 关节轨迹到位 (CMD_MOVE_J, 阻塞到到位)
  {"cmd":"q","q":[7]}   一帧 7 关节角目标, 逐帧跟随 (默认 JOINT_FOLLOW 0x08; 可选 MOVE_JS / MOVE_J / MOVE_MIT_ALL)
  {"cmd":"latch"}       置位: 就地持位 (movej 到当前角, 固件到位增刚持位)
  {"cmd":"start"}       开始跟随: 使能 + 读当前角 + following=True
  {"cmd":"stop"}        急停 (EMERGENCY_STOP)
  {"cmd":"ping"}        探活, 返回当前 q/状态

⚠️ 真机会真实运动! 运行前确认机械臂上电、工作区无人、急停在手边。

用法:
  python3 arm_follow_server_stm32.py --port /dev/ttyACM0 --host 0.0.0.0 --tcp-port 9001 \
      [--mode jf|js|j|mit] [--kp 7个] [--kd 7个] [--sp 0.5]
"""
import argparse
import json
import math
import signal
import socket
import struct
import sys
import threading
import time

from measured_state import SequenceFreshness, response as measured_response

from litearm import Arm as LitearmArm
from litearm import _protocol as _P
from litearm.errors import LiteArmError


# ---- STM32 状态帧解码: 已删除 ----
# 旧版这里自己按 flags(2B)+seq(2B)+每关节 21B 解状态帧。改用 litearm SDK 后，
# SDK 自带读线程把固件 100Hz 被动流解成 `RobotState`（mode/mode_name/flag_names/
# joints 都在），本文件不再需要自己的解码器。


def _to7(v):
    try:
        vals = [float(x) for x in v]
    except (TypeError, ValueError):
        return None
    return vals if len(vals) == 7 else None


class Stm32Arm:
    """STM32 串口封装 —— 底层是 litearm SDK (`github/litearm/litearm-python`)。

    SDK 自己起读线程消费固件 100Hz 的被动状态流（`get_state()` 返回最近一帧的缓存，
    不额外取帧），所以本类**不再自己读串口、不再自己解状态帧、不再自己做串口写互斥**
    —— 那三件事 SDK 都做了（含写锁与逐帧应答队列）。

    与旧 `arm_console` 版的对应关系:
      `LiteArm(port)` + `.ser` + `.recv_frame` + `.send`  →  SDK 的传输层与命令方法
      `decode_status()`                                  →  `RobotState`
      `CMD_*` / `RSP_*` / `MODE_NAMES` / `FLAG_NAMES`    →  `litearm._protocol`
    本类只把 `RobotState` 适配成本服务内部沿用的 dict 形状（flags/mode/mode_name）。
    """

    def __init__(self, port=None, transport_factory=None):
        self._feedback_freshness = SequenceFreshness()
        self._feedback_lock = threading.Lock()
        # port=None ⇒ 交给 SDK 用 find_cdc_port() 自动发现 (VID:PID 1d50:606f)。
        # transport_factory 是给离线测试注桩用的（litearm.testing.FakeTransport）。
        self._arm = LitearmArm(port, transport_factory=transport_factory).connect()
        deadline = time.time() + 3.0
        while time.time() < deadline and self._raw_state() is None:
            time.sleep(0.02)
        if self._raw_state() is None:
            raise RuntimeError("3s 内未收到 STM32 状态帧, 检查串口/固件是否运行")

    def _raw_state(self):
        """litearm 的 `RobotState`（读缓存帧，不发新请求）；无帧时 None。"""
        return self._arm.get_state().value

    # ---- 查询 ----
    def get_q(self):
        """当前 7 关节角；尚无状态帧时 None（调用方须判空）。"""
        st = self._raw_state()
        return list(st.q) if st is not None else None

    def get_state(self):
        """本服务内部沿用的 dict 形状；无帧时 None。"""
        st = self._raw_state()
        if st is None:
            return None
        return {"seq": st.seq, "mode": st.mode, "mode_name": st.mode_name,
                "flags": list(st.flag_names), "n": st.n,
                # `enabled` = flags bit9（固件 1.5.0 起上报），**唯一可靠**的"已使能"
                # 判据：`flag_names` 只看 bit0..5，这一位不在其中。
                # `joint_fault` = G7 断轴位图，非 0 时固件拒绝使能（ERR 0x10/0x06）。
                "enabled": st.enabled, "joint_fault": st.joint_fault,
                "joints": [(j.q, j.dq, j.tau, j.t_mos, j.t_coil, j.err)
                           for j in st.joints]}

    def measured_state(self, request_id):
        with self._feedback_lock:
            st = self._raw_state()
            if st is None:
                return measured_response(request_id, error="no measured state")
            age = self._feedback_freshness.age(st.seq)
            valid = age is not None and st.n == 7 and st.joint_fault == 0
            return measured_response(request_id, st.q, age, valid,
                                     None if valid else "feedback stream not yet verified")


    @property
    def n(self):
        st = self._raw_state()
        return st.n if st is not None else 0

    # ---- 命令 ----
    def enable(self, attempts=12):
        """清故障后使能, 返回**是否真的已使能**。

        判据: 状态帧 `enabled` 位 (flags bit9) 为真 且 `joint_fault == 0`。
        ⚠ **不能用 mode 判**: 固件 `ctrl_enable`→`do_enable_motors` 只置 `g_arm.enabled`,
        **不改 mode**（mode 由首个运动命令才切到 MOVE_J/MIT_ALL）—— 使能成功后 mode 仍是
        INIT(0), 拿 "mode not in (0,6)" 判「已使能」恒为假 ⇒ enable 明明成功却报失败,
        `_following` 永不置位, 真机收到 `q` 帧也被丢, 表现为"手柄按下臂使能了(变刚)但不动"。

        固件拒绝码 0x03=反馈未齐可重试、0x06=joint_fault/EMERGENCY 锁存须先 reset
        —— SDK 的 `enable(attempts)` 内部已按可重试码重试, 失败会抛错。
        CLEAR_FAULTS 已清 joint_fault; 仍失败则 RESET(清 joint_fault + safety 静态态)
        后重试一次。

        ⚠ SDK 明确记录 `enable()` **在状态帧反映之前就返回**(实测 2.0~3.1 ms 返回,
        而 `enabled` 首次为真要 17.6~48.5 ms) ⇒ 返回后必须**轮询确认**, 不能直接读。
        """
        def _try():
            try:
                self._arm.enable(attempts=attempts)
            except LiteArmError as e:
                print(f"[真机] enable 被拒: {e}", flush=True)
                return False
            end = time.time() + 3.0
            while time.time() < end:
                st = self.get_state()
                if st and st["enabled"] and st["joint_fault"] == 0:
                    return True
                time.sleep(0.05)
            return False

        self._arm.clear_faults()
        if _try():
            return True
        self._arm.reset()
        return _try()

    def home(self, timeout=30.0):
        """回零: 固件 CMD_HOME(各轴回 URDF 零位舒展姿), 阻塞到到位。

        SDK 的 `home()` 只收 `timeout`(关字), 固件侧速度写死 0.10(低安全速度),
        不接受速度参数。该命令要求先使能, 故未使能时先补一次使能。
        """
        st = self._raw_state()
        if st is None or not st.enabled:
            if not self.enable():
                print("[真机] home 前使能失败", flush=True)
                return False
        try:
            self._arm.home(timeout=timeout)
            return True
        except LiteArmError as e:
            print(f"[真机] home 失败: {e}", flush=True)
            return False

    def movej(self, q7, sp=0.5, timeout=30.0):
        """关节轨迹到位: MOVE_J(绝对角+速度倍率), 阻塞到到位。

        到位判定交给 SDK 的 `_arrive`: 连续数拍**位置不变**(且明确不读上报的 dq ——
        实测臂静止时 dq 仍在 0.007~0.256 rad/s 摆动, 读 dq 会永远判不到位)。

        500ms 保活重发同一目标**保留**(旧 arm_console 版就有): 固件把同目标重发当
        「续期」、不重启 S 曲线; SDK 文档说 MOVE_J 不受 0.1s 命令看门狗约束, 两者
        不冲突, 成本为零而收益是防长距离 MOVE_J 中途被看门狗掐成 fail-soft。
        """
        payload = _P.pack_f32s(q7) + struct.pack("<f", sp)
        stop = threading.Event()

        def _keepalive():
            while not stop.wait(0.5):
                try:
                    self._arm._write_cmd(_P.CMD_MOVE_J, payload)
                except Exception:
                    return

        threading.Thread(target=_keepalive, daemon=True).start()
        try:
            self._arm.movej(q7, speed=sp)
            return True
        except LiteArmError as e:
            print(f"[真机] movej 失败: {e}", flush=True)
            return False
        finally:
            stop.set()

    def send_mit_all(self, q7, kp7, kd7):
        """MIT 透传: 每关节 (q, dq=0, kp, kd, tau=0)。"""
        self._arm.send_mit_all(q7, [0.0] * len(q7), kp7, kd7, [0.0] * len(q7))

    def move_j_stream(self, q7, sp=0.5):
        """轨迹解算流: 每帧发 CMD_MOVE_J (固件 S 曲线 + 前馈, 目标连续则平滑)。

        ⚠ 这里碰了私有的 `_write_cmd` —— **SDK 没有公开的「非阻塞单发 MOVE_J」**:
        `movej()` 会阻塞到位, 其余的要么是笛卡尔、要么语义不同(`move_js` 是 0x03,
        帧里带 dq)。用工作线程跑 `movej()` 也能通, 但那会变成「到了才收下一个目标」,
        与这条模式「每帧重规划」的语义不同, 故宁可碰一次私有方法。
        另外三种模式(`jf`/`js`/`mit`)全部走公开 API。
        """
        self._arm._write_cmd(_P.CMD_MOVE_J, _P.pack_f32s(q7) + struct.pack("<f", sp))

    def move_js_stream(self, q7, dq7):
        """关节伺服流: 每帧发 MOVE_JS (q+dq, 不带 tau_ff)。
        固件 move_js 有内置 G+惯量+摩擦+积分前馈, 且自动钳位到关节限位。

        ⚠ 需调用方按 ≥10Hz 重发, 否则 0.1s 命令看门狗 fail-soft。"""
        self._arm.move_js(q7, dq7)

    def joint_follow_stream(self, q7, dq7, kp7, kd7):
        """从臂跟随流: 每帧发 CMD_JOINT_FOLLOW (固件 1.9.0+)。

        载荷 = q[N] + dq[N] + K[N] + B[N] 各 f32 = 16N 字节, **帧内无 tau** ——
        前馈由固件自算 τ_ff = clamp(G(q_meas) + wall, ±tau_max), 且每帧 watchdog_kick。
        固件复用 ARM_MODE_MOVE_MIT_ALL 并豁免「位置越界/超速」锁存。
        ⚠ 本命令需**周期重发**（fail-soft 看门狗 0.1 s）。"""
        self._arm.joint_follow(q7, dq7, kp7, kd7)

    def emergency(self):
        """急停（SDK 走 guarded=False: 零重力期也永远可达）。"""
        self._arm.emergency_stop()


class ArmFollowServer:
    """接收 VR JSON, 串行驱动 STM32 真机。slew/限位照搬 arm_follow_server.py。"""

    # 静止保持（粘滞设定点）：目标逐轴几乎不变则冻结到锚点，滤掉上位机残余的
    # 目标随机游走漂移；一旦变化超 BREAK_EPS 立即破冰。源头(仿真)已加静止死区时
    # 本层≈no-op，作为小阈值安全网。
    REST_EPS = 1e-4    # rad: 逐轴偏差低于此视为"没动"
    BREAK_EPS = 1e-3   # rad: 逐轴偏差达此立即破冰（继续跟随）
    HOLD_FRAMES = 5    # 连续 N 帧"没动"才进入冻结

    def __init__(self, port, mode="mit", kp=None, kd=None, sp=0.5,
                 q_min=None, q_max=None, max_speed=None, max_accel=None,
                 transport_factory=None):
        print(f"[真机] 连接 STM32 {port or '(自动发现 CDC)'} mode={mode}", flush=True)
        self._arm = Stm32Arm(port, transport_factory=transport_factory)
        if self._arm.n != 7:
            print(f"[真机] ⚠ 固件关节数={self._arm.n}, 期望 7 (7 关节整臂版)", flush=True)
        self._mode = mode
        self._port = port            # 串口（None=自动发现）；随 ping 回包上报，供状态端标名
        self._sp = sp
        self._kp = _to7(kp) if kp else [50.0] * 7
        self._kd = _to7(kd) if kd else [2.0] * 7
        print(f"[真机] kp={self._kp} kd={self._kd} sp={sp}", flush=True)

        # 关节限位: 默认用 litearm-stm32 固件限位 (与 smooth_wander.py 一致)
        self._q_min = _to7(q_min) if q_min else \
            [-2.949607, -1.797689, -2.80998, -3.07178, -2.80998, -1.553343, -1.553343]
        self._q_max = _to7(q_max) if q_max else \
            [2.949607, 1.797689, 2.80998, 0.593412, 2.80998, 1.553343, 1.553343]
        self._max_speed = _to7(max_speed) if max_speed else [2.0] * 7
        self._max_accel = _to7(max_accel) if max_accel else [3.0] * 7
        print(f"[真机] 限位 q_min={self._q_min}", flush=True)
        print(f"[真机] 限位 q_max={self._q_max}", flush=True)

        self._following = False
        self._holding = False
        self._lock = threading.Lock()
        self._q_cmd = None
        self._dq_cmd = [0.0] * 7
        self._last_t = None
        self._last_clamp_warn = 0.0
        self._last_stream_err = 0.0          # 流式帧被 SDK 拒的限频打印时刻
        self._shutdown = threading.Event()   # 优雅关闭标志 (信号触发)
        self._last_target = None             # 最近一帧仿真下发目标(原始, 未 slew)
        self._anchor = None                  # 静止保持锚点(粘滞设定点)
        self._hold_count = 0                 # 连续"目标几乎不变"帧计数

    # ---- slew/限位 (照搬 arm_follow_server.py) ----
    def _clamp_to_joint_limits(self, q):
        out, clamped = [], []
        for i in range(7):
            lo, hi = self._q_min[i], self._q_max[i]
            v = max(lo, min(hi, q[i]))
            if abs(v - q[i]) > 1e-6:
                clamped.append((i, q[i], v))
            out.append(v)
        return out, clamped

    def _reset_slew(self, q):
        self._q_cmd = list(q)
        self._dq_cmd = [0.0] * 7
        self._last_t = None
        self._anchor = list(q)
        self._hold_count = 0

    def _sticky_hold(self, raw):
        """静止保持：目标几乎不变则冻结到锚点(去随机游走漂移)，变化大则破冰跟随。"""
        if self._anchor is None:
            self._anchor = list(raw)
            self._hold_count = 0
            return list(raw)
        dev = max(abs(raw[i] - self._anchor[i]) for i in range(7))
        if dev >= self.BREAK_EPS:            # 实质变化 → 立即破冰并重锚
            self._anchor = list(raw)
            self._hold_count = 0
            return list(raw)
        if dev < self.REST_EPS:              # 几乎不动 → 累计到阈值后冻结
            self._hold_count += 1
            if self._hold_count >= self.HOLD_FRAMES:
                self._dq_cmd = [0.0] * 7     # 冻结时清速度前馈(jf/js 发 dq)
                return list(self._anchor)
        else:
            self._hold_count = 0             # 边缘带内：照常跟随（避免双死区）
        return list(raw)

    def _slew_target(self, raw_target, dt):
        safe_dt = max(dt, 1e-4)
        for i in range(7):
            v_limit = max(1e-4, self._max_speed[i])
            a_limit = max(1e-4, self._max_accel[i])
            dv_max = a_limit * safe_dt
            diff = raw_target[i] - self._q_cmd[i]
            v = self._dq_cmd[i]
            if abs(diff) < 1e-5 and abs(v) < dv_max:
                self._q_cmd[i] = raw_target[i]
                self._dq_cmd[i] = 0.0
                continue
            stopping_dist = (v * v) / (2.0 * a_limit)
            if (diff * v > 0.0) and abs(diff) <= stopping_dist:
                desired_v = 0.0
            else:
                desired_v = math.copysign(v_limit, diff)
            dv = max(-dv_max, min(dv_max, desired_v - v))
            v = max(-v_limit, min(v_limit, v + dv))
            step = v * safe_dt
            if diff * step > 0.0 and abs(step) >= abs(diff):
                self._q_cmd[i] = raw_target[i]
                self._dq_cmd[i] = 0.0
            else:
                self._q_cmd[i] += step
                self._dq_cmd[i] = v
        return list(self._q_cmd)

    # ---- 命令处理 ----
    def handle(self, msg: dict) -> dict:
        cmd = msg.get("cmd")

        if cmd == "start":
            self._holding = False
            # 已在「使能 + 跟随」中则幂等返回，不重跑 enable。
            # 原因：enable → do_enable_motors 会把 cmd.kp/kd 重播为参数表值
            # (mit_kp/kd, 如 J1=400/5)，而 joint_follow 帧的 kp/kd 是 50/2 —— 每按一次
            # start 都会把刚度顶到 400 好几拍，制造一次小刚度脉冲（与 latch 的模式阶跃
            # 同源）。方案 A 下按/松不再切模式，正常跟随期间 start 即幂等；仅在未使能/
            # 有故障时才真正执行使能，保留故障自愈。
            # ⚠ 判「已使能」看 `enabled` 位而非 mode —— 固件使能不切 mode（见 enable()）。
            st = self._arm.get_state()
            with self._lock:
                following = self._following
            if (following and st is not None
                    and st["enabled"] and st["joint_fault"] == 0):
                return {"ok": True, "cmd": cmd, "already": True}
            ok = self._arm.enable()              # 清故障+使能, 返回是否真的已使能
            q = self._arm.get_q()
            if not ok:
                st = self._arm.get_state()
                print(f"[真机] start 使能失败: state={st}", flush=True)
                return {"ok": False, "cmd": cmd,
                        "error": f"使能失败 (flags={st['flags'] if st else '?'}), "
                                 "先 home 回零清关节故障"}
            with self._lock:
                self._following = True
                if q:
                    self._reset_slew(q)
            resp = {"ok": True, "cmd": cmd}
            if q:
                resp["q"] = q
            return resp

        if cmd == "movej":
            q7 = _to7(msg.get("q")) if isinstance(msg.get("q"), (list, tuple)) else None
            if q7 is None:
                return {"ok": False, "error": "q 必须是 7 个数值"}
            with self._lock:
                self._following = False
            print(f"[真机] movej → {[round(x, 4) for x in q7]} (阻塞中)", flush=True)
            ok = self._arm.movej(q7, sp=self._sp)
            print(f"[真机] movej 到位 ok={ok}", flush=True)
            if ok:
                with self._lock:
                    self._following = True
                    self._reset_slew(q7)
            return {"ok": ok, "cmd": cmd}

        if cmd == "q":
            self._holding = False
            q7 = _to7(msg.get("q")) if isinstance(msg.get("q"), (list, tuple)) else None
            if q7 is None:
                return {"ok": False, "error": "q 必须是 7 个数值"}
            with self._lock:
                self._last_target = list(q7)   # 记录仿真下发的原始目标
                if self._following:
                    q7, clamped = self._clamp_to_joint_limits(q7)
                    if clamped:
                        now = time.time()
                        if now - self._last_clamp_warn > 2.0:
                            self._last_clamp_warn = now
                            detail = ", ".join(
                                f"J{i+1}: {orig:+.4f}→{new:+.4f} "
                                f"(限位[{self._q_min[i]:+.4f},{self._q_max[i]:+.4f}])"
                                for i, orig, new in clamped)
                            print("[真机] 目标超出关节限位, 已 clamp: %s" % detail,
                                  flush=True)
                    q7 = self._sticky_hold(q7)   # 静止保持：目标几乎不变则冻结去漂移
                    now = time.time()
                    dt = 0.05 if self._last_t is None else max(1e-4, min(0.05, now - self._last_t))
                    self._last_t = now
                    if self._q_cmd is None:
                        self._reset_slew(q7)
                    else:
                        q7 = self._slew_target(q7, dt)
                    try:
                        if self._mode == "j":
                            self._arm.move_j_stream(q7, sp=self._sp)
                        elif self._mode == "js":
                            self._arm.move_js_stream(q7, self._dq_cmd)
                        elif self._mode == "jf":
                            self._arm.joint_follow_stream(q7, self._dq_cmd,
                                                          self._kp, self._kd)
                        else:
                            self._arm.send_mit_all(q7, self._kp, self._kd)
                    except LiteArmError as e:
                        # 旧 arm_console 的 _send 是 fire-and-forget、不会抛; SDK 会 ——
                        # 而这条分支跑在连接线程里、本文件对 handle() 没有兜底, 抛出会
                        # 直接掐掉该连接。故在此转成响应, 并限频打印(这条路径 100Hz)。
                        now = time.time()
                        if now - self._last_stream_err > 2.0:
                            self._last_stream_err = now
                            print(f"[真机] {self._mode} 帧被拒: {e}", flush=True)
                        return {"ok": False, "cmd": cmd,
                                "error": f"{self._mode} 帧被拒: {e}"}
            return {"ok": True, "cmd": cmd}

        if cmd == "hold":
            measured = self._arm.measured_state(msg.get("request_id"))
            with self._lock:
                q = (self._q_cmd if getattr(self, "_holding", False) else
                     measured["q"] if measured["valid"] else self._q_cmd)
                self._holding = True
                self._last_target = None
                if q is None:
                    return {"ok": False, "cmd": cmd, "error": "no measured or commanded hold position"}
                self._reset_slew(q)
                # Preserve mode and gains; never use movej/latch for grip release.
                if self._following:
                    try:
                        if self._mode == "j":
                            self._arm.move_j_stream(q, sp=self._sp)
                        elif self._mode == "js":
                            self._arm.move_js_stream(q, self._dq_cmd)
                        elif self._mode == "jf":
                            self._arm.joint_follow_stream(q, self._dq_cmd, self._kp, self._kd)
                        else:
                            self._arm.send_mit_all(q, self._kp, self._kd)
                    except LiteArmError as exc:
                        return {"ok": False, "cmd": cmd, "error": str(exc)}
            return {"ok": True, "cmd": cmd, "measured": measured["valid"]}

        if cmd == "latch":
            with self._lock:
                self._following = False
            q = self._arm.get_q()
            if not q:
                # get_q() 现在无状态帧时返回 None（旧版返回 [0]*7）。若照旧往下走，
                # 会拿 None 去 movej —— 更糟的是若回退成零向量，就等于**命令真机回零**。
                return {"ok": False, "cmd": cmd, "error": "无状态帧, 拒绝持位"}
            # 就地持位: movej 到当前角 -> 固件 MOVE_J 到位增刚持位 (ht_on 关掉内置
            # 积分/摩擦前馈, τ 平滑过渡到 G(q_hold), kp 增 2×), 且固件自己喂狗。
            # 不能用 move_js 持位 (其内置积分前馈在静止态累积稳态误差 -> 极限环震荡)。
            self._arm.movej(q, sp=self._sp)
            return {"ok": True, "cmd": cmd}

        if cmd == "stop":
            with self._lock:
                self._following = False
            self._arm.emergency()
            return {"ok": True, "cmd": cmd}

        if cmd == "home":
            with self._lock:
                self._following = False
            print("[真机] home 回零 (阻塞中)", flush=True)
            ok = self._arm.home()
            print(f"[真机] home 到位 ok={ok}", flush=True)
            return {"ok": ok, "cmd": cmd}

        if cmd == "state":
            measured = self._arm.measured_state(msg.get("request_id"))
            with self._lock:
                # Post-clamp/post-sticky-hold/post-slew target that is actually
                # sent to the firmware.  Teleop must compare measured q with
                # this value, not with its faster upstream target, otherwise
                # intentional downstream wrist limiting looks like a fault.
                measured["cmd_q"] = (list(self._q_cmd)
                                     if self._q_cmd is not None else None)
            return measured

        if cmd == "ping":
            st = self._arm.get_state()
            q = self._arm.get_q()
            with self._lock:
                target = list(self._last_target) if self._last_target is not None else None
                anchor = list(self._anchor) if self._anchor is not None else None
                # cmd_q = 经限位钳制/静止保持/梯形 slew **之后**真正下发的目标。
                # 数据探针靠它把「metavr 送来的 target」与「server 决定发的 cmd_q」
                # 分开 —— 不暴露它就无法判断跳变是不是 slew/钳制造成的。
                cmd_q = list(self._q_cmd) if self._q_cmd is not None else None
                following = self._following
            return {"ok": True, "state": st["mode_name"] if st else None,
                    "flags": st["flags"] if st else None,
                    "enabled": st["enabled"] if st else None,
                    "joint_fault": st["joint_fault"] if st else None, "q": q,
                    # 每关节的 q/dq/tau/t_mos/t_coil/err 并行数组，供状态汇总取温度/故障；
                    # st 里早已算好，这里只是原样透出（老消费方忽略新键即可）。
                    "joints": st["joints"] if st else None,
                    # 串口即臂的身份（left/right 会猜错）；状态端据此给 device 标名。
                    "port": self._port,
                    "target": target, "anchor": anchor, "cmd_q": cmd_q,
                    "following": following, "mode": self._mode}

        return {"ok": False, "error": f"unknown cmd {cmd!r}"}

    # ---- TCP 服务 (JSON 每行一条, 同 arm_follow_server.py) ----
    def serve(self, host, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((host, port))
            srv.listen(5)
            srv.settimeout(0.5)   # 周期性唤醒, 检查 shutdown 信号
            print(f"[服务] 监听 {host}:{port}", flush=True)
            while not self._shutdown.is_set():
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                print(f"[服务] 客户端接入: {addr}", flush=True)
                threading.Thread(target=self._serve_conn, args=(conn, addr), daemon=True).start()
        # 退出监听循环后: 打断跟随并就地持位 (防塌臂), 再结束进程
        self._interrupt_and_hold()
        print("[服务] 已安全退出", flush=True)

    def _serve_conn(self, conn, addr):
        with conn:
            conn.settimeout(None)
            buf = b""
            while True:
                try:
                    data = conn.recv(4096)
                except OSError:
                    break
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line.decode("utf-8", "replace"))
                    except (ValueError, TypeError):
                        resp = {"ok": False, "error": "bad json"}
                    else:
                        resp = self.handle(msg)
                    try:
                        conn.sendall((json.dumps(resp) + "\n").encode())
                    except OSError:
                        return
        print(f"[服务] 客户端断开: {addr}", flush=True)

    # ---- 优雅关闭 (SIGINT/SIGTERM 触发) ----
    def request_shutdown(self):
        """请求优雅关闭 (由信号处理函数调用, 线程安全)。"""
        self._shutdown.set()

    def _interrupt_and_hold(self):
        """打断: 停止跟随并就地持位 (movej 到当前角, 固件增刚持位防塌臂)。"""
        with self._lock:
            self._following = False
        try:
            q = self._arm.get_q()
            if q:
                self._arm.movej(q, sp=self._sp)
                print("[真机] 已打断, 就地持位", flush=True)
        except Exception as e:
            print(f"[真机] 打断持位失败: {e}", flush=True)


def main():
    p = argparse.ArgumentParser(description="LiteArm 七轴真机遥操跟随服务 (STM32 直连)")
    p.add_argument("--port", default="/dev/ttyACM0", help="STM32 USB CDC 串口")
    p.add_argument("--mode", choices=["jf", "js", "j", "mit"], default="jf",
                   help="跟随策略: jf=JOINT_FOLLOW 0x08 固件内伺服环+前馈(推荐, 需固件>=1.9.0) / js=MOVE_JS 关节伺服 / j=MOVE_J 轨迹解算 / mit=MOVE_MIT_ALL 透传(无重力补偿, 慎用)")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--tcp-port", type=int, default=9001)
    p.add_argument("--kp", type=float, nargs=7, default=None, help="位置增益 K 7 个 (jf/joint_follow 与 mit 共用)")
    p.add_argument("--kd", type=float, nargs=7, default=None, help="阻尼增益 B 7 个 (jf/joint_follow 与 mit 共用)")
    p.add_argument("--sp", type=float, default=0.5, help="move_j 速度倍率 0~1")
    p.add_argument("--q-min", type=float, nargs=7, default=None)
    p.add_argument("--q-max", type=float, nargs=7, default=None)
    p.add_argument("--max-speed", type=float, nargs=7, default=None)
    p.add_argument("--max-accel", type=float, nargs=7, default=None)
    args = p.parse_args()

    server = ArmFollowServer(args.port, args.mode, args.kp, args.kd, args.sp,
                             args.q_min, args.q_max, args.max_speed, args.max_accel)

    # Ctrl+C / kill 时先打断机械臂就地持位, 再退出进程 (防跟随中突然断连塌臂)
    def _on_signal(signum, frame):
        print(f"[服务] 收到信号 {signum}, 打断并退出...", flush=True)
        server.request_shutdown()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    server.serve(args.host, args.tcp_port)


if __name__ == "__main__":
    main()

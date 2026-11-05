#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪常驻 CAN 服务（TCP + JSON），跑在夹爪 DM 电机所在的工控机上。

与 `arm_follow_server_*.py` 同类：都在「链路最靠近硬件的一环」——收指令、驱动电机。
臂收 7 关节角流，夹爪收离散的 close/open 边沿；本服务只管夹爪这一只 DM 电机。

驱动走 **litegrip-python SDK**，但**不用它的高层阻塞动作**（`open()` / `close()` /
`grasp()`）—— 那些是「一次性动作 + 堵转判据 + 分片保力」，现场踩过的三个坑（分片重跑
闭合斜坡、保活位置环顶死、`open()` 在第一个 0.25s 窗口被判堵转收工）都在那一层，见
`docs/gripper-jitter-analysis.md`。本服务照 **litegrip-cpp 的 `ControlLoop`**（同一套设计
也用在 ros2_control 与 lerobot 侧，见 `docs/gripper-control-loop-rewrite.md`）自己跑
**一条 ~200Hz 的控制环**，SDK 只用来发 `send_mit_frame`（专家级、公开入口）：

  上层只投**目标角度**（`close` → 标定闭合限位；`open` → 标定张开限位），环每拍自己做：

    读反馈 → 反馈看门狗/温度/电机故障 → 轨迹按 `--speed` 限速推进 → 力矩预算分给 kp/kd
    → 帧内位置钳位（领先实测最多 预算/kp）→ 发 MIT 帧（dq=0, tau=0）

  三条要点，缺一条就会退回老毛病：

  - **帧里的 q 是「命令轨迹」而不是「最近读到的位置」**。老实现拿最近一次读数当设定值，
    与同一时刻的实测值天然有偏差（实测 ~0.7 mm），`kp × Δ` 就是一牛·米级的持续力，
    把夹爪顶死在限位上形成 ~1.2s 的松弛振荡。命令轨迹是绝对目标，没有这个偏差。
  - **帧里要求的力被几何封住**：`q_frame = 实测 + clamp(轨迹 − 实测, ±预算/kp)`，
    所以撞上工件/硬限位时，帧里要求的力矩**不超过预算** —— 夹持力就是 `--force`，
    工件让位时误差变大、力自动保持，不需要第二套「保力」模式。
  - **目标是个变量，不是一次动作**：`set_target` 只覆盖它，下一拍轨迹就以 `--speed`
    朝新目标走（≤5ms 换向），没有「等这一片跑完」的延迟，也不会「一次没到位就收工」。

`--speed`（mm/s，默认 50）是**命令轨迹的推进速度上限**（不是 MIT 帧的 dq 字段，那恒为 0）；
`--force`（N，默认 20）是**力矩预算**（×0.1 = Nm），即夹持力上限。

TCP 线程只把目标写进一个槽位后立刻回执（与旧版语义一致，fire-and-forget），
所以远端 teleop 侧的 trigger 边沿逻辑不需要改。

**链路不通即丢弃**：投目标前若发现链路不可用（连不上 / 未标定），该指令直接丢弃，
**不留在槽里等链路恢复后补发**——否则链路上线（如 can0 up）的瞬间会把之前滞留的指令
一次性补执行，造成意外动作。

**绝不静默停流**：母线静默 ~100ms 电机即自锁成 0xD 通信丢失故障（之后既读不到反馈也收
不了命令）。故掉线/过温/电机故障时的出口是**发零增益帧**（不是不发）—— 与 C++ 的
`safe_stop()` 同一条理由。还没收到过目标时保持 limp、不发帧：那是「别管我」的语义。

**启动即 open 归位**：服务启动后自动投一次张开目标（`--no-startup-open` 可关）。头显遥操的
扳机是**切换**语义——第一下 = 闭合；夹爪先落在 open，第一下扳机才是「闭合」。
链路不通/未标定时这次目标会被丢弃、保持 limp，不会误动。

协议（TCP，每行一条 JSON 文本）：
  {"cmd":"close"}   闭合（斜坡到闭合限位 / 顶到工件止步）
  {"cmd":"open"}    张开（斜坡到标定工作行程）
  {"cmd":"ping"}    探活**并活读一帧**，返回 {"ok":true,"q":…,"tau":…}（rad / Nm）
  {"cmd":"status"}  **零 I/O** 回状态缓存（高频轮询用），额外带 enabled/error_code/
                    fresh/age_ms；状态由后台 worker 空闲时按 --status-hz 刷新

标定：SDK 用 JSON 标定文件描述行程。默认读 SDK 出厂标定；本机实测的标定需显式
给出（`--calib PATH`，或 `--mount normal|reverse` 选装向模板，或环境变量
`LITEGRIP_CALIB`）。**未标定时 SDK 拒绝移动**（占位限位不是真实行程），本服务会把
原因打进日志。

用法（在夹爪所在的工控机上）：
  python3 gripper_server.py --can can0 --host 0.0.0.0 --port 9003

CAN 接口需先 up（1Mbps classic），例如:
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set can0 up
"""
import argparse
import json
import os
import socket
import sys
import threading
import time

# 直接跑源码树时，把 SDK 的 src 目录加入 sys.path（装了包就不需要）。
_SDK_SRC = os.environ.get("LITEGRIP_SDK") or os.path.expanduser(
    "~/sl/litegrip-python/src")
if os.path.isdir(_SDK_SRC) and _SDK_SRC not in sys.path:
    sys.path.insert(0, _SDK_SRC)

from litegrip import LiteGrip, LiteGripError  # noqa: E402
#: 开合比例的**权威定义**在 SDK 里（别在这儿另写一个：同一台机器上两份换算必然漂）。
#: `rad_to_openness` 按**完整标定行程**归一（|open-closed|×rad_to_mm），故 `open()`
#: 走到工作行程时比例 ≈ 工作行程/完整行程（出厂标定下 80/86 ≈ 0.93），不是 1.0
#: —— 那是「占完整标定行程多少」的语义，不是「离全开还差多少」。
from litegrip.teleop import rad_to_openness  # noqa: E402

SPEED_MM_S = 100.0    #: 命令轨迹的**推进速度上限**（mm/s，开口量）
#: 闭合侧的**力矩预算**（N）。**夹持力就是它**：帧里要求的力矩被 `预算/kp` 的位置钳位
#: 封住，所以「撞上工件时压多大」只由这个数定，与目标还差多远无关。⚠ 不能留 0。
FORCE_N = 20.0
#: 预算换算成 Nm（SDK 的 `UnitConversion.N_TO_NM`：1 N ≈ 0.1 Nm）。
N_TO_NM = 0.1
RATE_HZ = 200.0      #: 控制环周期 Hz（照 litegrip-cpp 的 `control_rate_hz`）
#: 增益上限。实际取值还会被预算按**最坏情况**再压小（见 `_allocate_gains`）。
#: ⚠ 远小于 SDK 高层动作的 `kp=100`：那个刚度会把「设定值与实测的微小偏差」放大成
#: 牛米级的持续力 —— 现场那个 ~1.2s 振荡就是这么来的（docs/gripper-jitter-analysis.md §4.3.1）。
KP_CEIL = 20.0
KD_CEIL = 0.5
#: 预算分配用的「最坏位置误差」mm 与「最坏反馈速度」rad/s（照 C++ 的
#: `max_position_error_rad` / `max_feedback_velocity_rad_s`）。前者决定 kp 上限：`kp ≤ 预算/它`。
MAX_POS_ERROR_MM = 10.0
MAX_FB_VEL_RAD_S = 1.0
FEEDBACK_TIMEOUT_S = 0.5   #: 多久没有新反馈帧算掉线（照 C++）
TEMP_LIMIT_C = 80          #: MOS / 线圈温度上限 ℃（照 C++）
ENABLE_RETRY_S = 0.5       #: 使能失败的重试间隔 s
STATUS_HZ = 20.0     #: **还没收到目标**（limp）时刷状态缓存的频率 Hz，让 ping/status 有新鲜反馈


#: 环自己的故障码（0 = 正常）。编号沿用 litegrip-cpp 的 `FaultCode` 语义，便于两边对照。
FAULT_NONE = 0
FAULT_NO_FEEDBACK = 2      # 掉线：超过 `--feedback-timeout` 没有新帧
FAULT_OVERTEMP = 6         # MOS/线圈过温（C++ 侧同一条出口：安全停）
FAULT_MOTOR_BASE = 100     # + 电机 error_code（C++ 的 kMotorFaultBase）


class LiteGripperControl:
    """litegrip SDK 封装：**一条 ~200Hz 控制环**驱动夹爪（照 litegrip-cpp 的 ControlLoop）。

    上层只投目标角度（`close()` / `open()` 都非阻塞），环每拍自己做
    「轨迹限速 → 力矩预算分配 → 帧内位置钳位 → 发 MIT 帧 → 读反馈 → 看门狗/温度/故障」。
    见模块 docstring 与 `docs/gripper-control-loop-rewrite.md`。
    """

    def __init__(self, can="can0", can_id=0x08, mst_id=None, mount=None,
                 calib=None, speed=SPEED_MM_S, force_n=FORCE_N, rate_hz=RATE_HZ,
                 kp=KP_CEIL, kd=KD_CEIL, max_pos_error_mm=MAX_POS_ERROR_MM,
                 max_fb_vel=MAX_FB_VEL_RAD_S, feedback_timeout=FEEDBACK_TIMEOUT_S,
                 temp_limit=TEMP_LIMIT_C, status_hz=STATUS_HZ,
                 startup_disable=True, startup_open=True):
        self._can = can
        self._can_id = int(can_id)
        self._speed_mm_s = float(speed)
        #: 力矩预算（Nm）。夹持力由它定，见 FORCE_N 的注释。
        self._budget_nm = 0.0 if force_n is None else float(force_n) * N_TO_NM
        self._kp_ceil = float(kp)
        self._kd_ceil = float(kd)
        self._max_pos_error_mm = float(max_pos_error_mm)
        self._max_fb_vel_rad_s = float(max_fb_vel)
        self._rate_hz = max(10.0, float(rate_hz))
        self._feedback_timeout_s = float(feedback_timeout)
        self._temp_limit_c = float(temp_limit)
        self._startup_disable = bool(startup_disable)
        self._startup_open = bool(startup_open)
        # None = 关掉空闲刷状态（--status-hz 0）：此时不会只因轮询就连 CAN。
        self._status_interval = (1.0 / float(status_hz)) if status_hz > 0 else None

        # mount= 会立即装载同名模板；显式 --calib 再覆盖。
        self._g = LiteGrip(channel=can, can_id=self._can_id, mst_id=mst_id,
                           mount=mount, disable_on_disconnect=True)
        if calib is not None:
            self._g.load_calibration(path=calib)
        elif mount is None:
            self._g.load_calibration()  # 出厂标定（或 LITEGRIP_CALIB）

        # ---- 环的状态（下面这些只由 worker 线程写，读的地方加锁）----
        #: 上层投的目标角度；**None = 还没收到过目标**（保持 limp、不发帧）。
        self._target_rad = None
        #: 限速后的命令轨迹（rad）。None = 下一拍从**实测位置**起步（不跳变）。
        self._trajectory = None
        self._last_cycle_mono = None   # 上一拍时刻（dt 用实测 elapsed）
        self._last_fresh_mono = None   # 最近一次**新帧**的时刻（反馈看门狗）
        self._last_enable_mono = 0.0   # 上次试着使能的时刻（重试限速）
        self._last_log_mono = 0.0      # 发帧失败等日志的限流
        self._next_idle_mono = 0.0     # 下次允许（limp 时）刷状态缓存的时刻
        self._fault = FAULT_NONE       # 见 FAULT_*

        # ---- 对外状态缓存（供 ping/status，都是**实测**）----
        self._q = 0.0            # 最新反馈位置 rad（供 ping）
        self._position_mm = None  # 最新反馈张开量 mm（SDK 算好的，含方向处理）
        self._ratio = None        # 最新归一化开合比例 [0,1]（SDK 的 rad_to_openness）
        #: 最近一次**收到**的命令折算成的目标比例（close→0.0 / open→1.0）；从未 = None。
        #: ⚠ 它**粘住不清** —— 下游要的是「操作者的意图」，不是「此刻是否还在动」。
        #: 启动归位**不算**收到命令（它直接投目标），所以开机后 `target_ratio` 仍是
        #: None，别改成「开机就算 open」。
        self._last_cmd_ratio = None
        self._tau = 0.0          # 最新反馈扭矩 Nm
        self._t_mos = 0          # MOS 温度 ℃
        self._t_coil = 0         # 线圈温度 ℃
        self._error_code = 0     # 最新反馈错误码（1=已使能）
        self._fresh = False      # 最近一次读到的帧是否是当次新帧
        self._last_ok_mono = None  # 最近一次成功读状态的 monotonic 时刻
        self._ready = False      # 已 connect（CAN 已打开）
        self._warned_uncalibrated = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

    # ---- 后台线程：控制环 ----
    def _loop(self):
        """控制环主循环（`--rate-hz`，默认 200Hz）。

        **持续发帧**是这里的核心：DM 电机母线静默 ~100ms 即自锁成 0xD 通信丢失故障，
        之后就既读不到反馈帧、也执行不了命令。所以环里不存在「不发帧的空闲」——
        没有目标时是**不使能**（真 limp），有目标之后就一直发（含安全出口的零增益帧）。

        ⚠ 环里不许有阻塞动作：一切「等它到位 / 做不成就重试」的写法（分片 `grasp`、
        有界重试、0.25s 堵转窗口）都会退回老毛病，见 docs/gripper-jitter-analysis.md。
        """
        if self._startup_disable:
            self._do_startup_disable()
        if self._startup_open:
            # 启动归位：让头显扳机的第一下才是「闭合」。⚠ 它**不算**「收到命令」，
            # 所以 target_ratio 开机后仍是 None（契约要的是「操作者最近一次命令」）。
            self._post_target("open", from_command=False)
        period = 1.0 / self._rate_hz
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self._cycle(t0)
            except LiteGripError as e:
                print(f"[夹爪] 环：链路/硬件异常 {e}（1s 后重连）", flush=True)
                self._teardown()
                self._stop.wait(1.0)
            except Exception as e:  # noqa: BLE001 - 环不许死：死了就是静默停流
                print(f"[夹爪] 环：异常 {e!r}", flush=True)
                self._teardown()
                self._stop.wait(1.0)
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)))

    def _cycle(self, now):
        """控制环的一拍：读反馈 → 使能 → 安全出口 → 轨迹限速 → 预算分配 → 发帧。

        `dt` 用**实测** elapsed（不是名义周期），照 C++ 的 `hardware_cycle(elapsed)`。
        """
        elapsed = (0.0 if self._last_cycle_mono is None
                   else max(0.0, now - self._last_cycle_mono))
        self._last_cycle_mono = now

        with self._lock:
            target = self._target_rad

        # 0) 还没有目标：保持 limp —— **不使能、不发帧**（那是「别管我」的语义），
        #    只按 status_hz 轮询一次，让 ping/status 至少有值。
        if target is None:
            self._fault = FAULT_NONE
            if self._status_interval is not None and now >= self._next_idle_mono:
                self._next_idle_mono = now + self._status_interval
                st, fresh = self._read_state(timeout_s=0.05)
                self._cache(st, fresh)
            return

        if not self._ensure_connected():
            raise LiteGripError(f"CAN 未连接（{self._can}）")

        # 1) 读反馈 + 反馈看门狗（等不超过半个周期，别拖慢环）
        st, fresh = self._read_state(timeout_s=min(0.002, 0.5 / self._rate_hz))
        self._cache(st, fresh)
        if fresh:
            self._last_fresh_mono = now
        stale = (self._last_fresh_mono is None
                 or (now - self._last_fresh_mono) > self._feedback_timeout_s)

        # 2) 使能（有目标才使能）。失败按 ENABLE_RETRY_S 退避，不 200Hz 刷屏。
        #    未使能时 SDK 会拒发帧，所以这一拍到此为止。
        if st.error_code != 1 and not stale:
            if now - self._last_enable_mono >= ENABLE_RETRY_S:
                self._last_enable_mono = now
                try:
                    res = self._g.enable()
                    if res.ok:
                        print(f"[夹爪] 已使能（{self._can} can_id=0x{self._can_id:02X} "
                              f"mount={self._g.config.mount}）", flush=True)
                    else:
                        self._log_throttled(now, f"[夹爪] 使能未成功（tries={res.tries}）"
                                                 f"，{ENABLE_RETRY_S}s 后重试")
                except LiteGripError as e:
                    raise LiteGripError(f"使能失败：{e}") from e
            return

        # 3) 安全出口：掉线 / 过温 / 电机故障 → **零增益帧**（不是停流！停流 ~100ms 即
        #    自锁成 0xD，之后连命令都收不了。C++ 的 safe_stop() 同一条理由）。
        hot = (st.temperature_mos > self._temp_limit_c
               or st.temperature_coil > self._temp_limit_c)
        if stale or hot or st.error_code != 1:
            self._fault = (FAULT_NO_FEEDBACK if stale else
                           FAULT_OVERTEMP if hot else FAULT_MOTOR_BASE + int(st.error_code))
            self._log_throttled(now, f"[夹爪] 安全停：故障码 {self._fault}"
                                     f"（掉线={stale} 过温={hot} err={st.error_code}）"
                                     f"—— 继续发零增益帧")
            self._send_frame(st.position_rad, 0.0, 0.0, now)
            return
        self._fault = FAULT_NONE

        # 4) 轨迹限速推进（照 C++：限速施加在**命令轨迹**上，MIT 帧的 dq 恒为 0）
        if self._trajectory is None:
            self._trajectory = st.position_rad        # 从实测起步：不跳变
        step = self._max_vel_rad_s() * elapsed
        if self._trajectory < target:
            self._trajectory = min(self._trajectory + step, target)
        else:
            self._trajectory = max(self._trajectory - step, target)

        # 5) 预算分配 + 帧内位置钳位 → 发帧
        kp, kd = self._allocate_gains()
        measured = st.position_rad
        lead = (self._budget_nm / kp) if kp > 0.0 else 0.0
        err = self._trajectory - measured
        self._send_frame(measured + max(-lead, min(lead, err)), kp, kd, now)

    # ---- 环里的算式（都照 litegrip-cpp 的 ControlLoop）----
    def _max_vel_rad_s(self):
        """命令轨迹的推进速度上限 rad/s（`--speed` mm/s ÷ 本机 `rad_to_mm`）。"""
        r2m = self._rad_to_mm()
        return self._speed_mm_s / r2m if r2m > 0.0 else 0.0

    def _max_pos_error_rad(self):
        """预算分配用的「最坏位置误差」rad（0 = 不设限，则 kp 只用上限）。"""
        r2m = self._rad_to_mm()
        if self._max_pos_error_mm <= 0.0 or r2m <= 0.0:
            return 0.0
        return self._max_pos_error_mm / r2m

    def _rad_to_mm(self):
        try:
            return float(getattr(self._g.config, "rad_to_mm", 0.0) or 0.0)
        except Exception:  # noqa: BLE001
            return 0.0

    def _allocate_gains(self):
        """把力矩预算按**最坏情况界**分给 kd 与 kp（照 `control_loop.cpp::allocate_gains`）。

        预算先给阻尼：`kd = min(kd上限, 预算 / 最坏反馈速度)`；剩下的给位置：
        `kp = min(kp上限, 剩余 / 最坏位置误差)`。含义是「即使误差与速度都达到最坏值，
        帧里要求的力矩也不超过预算」—— 于是**夹持力就是 `--force`**，与目标还差多远无关。
        """
        budget = self._budget_nm
        if budget <= 0.0:
            return 0.0, 0.0
        v_b = max(self._max_fb_vel_rad_s, 1e-6)
        kd = min(self._kd_ceil, budget / v_b)
        rest = budget - kd * self._max_fb_vel_rad_s
        err_b = self._max_pos_error_rad()
        kp = min(self._kp_ceil, (rest / err_b) if err_b > 0.0 else self._kp_ceil)
        return max(kp, 0.0), max(kd, 0.0)

    def _send_frame(self, q, kp, kd, now):
        """发一帧 MIT 帧（`dq=0`、`tau=0`，力全部来自 kp/kd 与位置误差）。

        ⚠ 帧里的 `q` 由调用方保证「领先实测不超过 预算/kp」，所以**不会**出现
        「帧里要求一牛·米级的力去顶硬限位」这种事。
        """
        ok = self._g.send_mit_frame(float(q), float(kp), float(kd))
        if not ok:
            cause = getattr(self._g, "last_send_error", None)
            raise LiteGripError(f"发帧被拒：{cause!r}")

    def _log_throttled(self, now, msg, interval=1.0):
        """限流打印：环跑 200Hz，一条失败信息会刷满屏。"""
        if now - self._last_log_mono >= interval:
            self._last_log_mono = now
            print(msg, flush=True)

    # ---- 连接 / 使能 ----
    def _do_startup_disable(self):
        """启动时 connect + disable（只发 0xFD 失能，不使能、不动作）。

        让电机上电后立刻 limp，避免按出厂零位（≈闭合）保持。仅需连通即可，
        不要求标定（不移动）。链路不通就跳过——不阻塞、不打屏，空闲刷新会重连。
        """
        if not self._ensure_connected():
            return
        try:
            ok = self._g.disable()
            self._error_code = 0
            print(f"[夹爪] 启动已 disable（limp）ok={ok}", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[夹爪] 启动 disable 失败: {e!r}", flush=True)

    def _ensure_connected(self):
        """确保已 connect（**只**打开 CAN，不使能）。返回是否连通。

        与「使能」分开：读状态只需连通，不必使能电机 —— 还没收到目标时环保持 limp，
        不应该因为「进程起来了」就把电机使能起来。
        """
        if self._ready:
            return True
        try:
            self._g.connect()
        except LiteGripError as e:
            print(f"[夹爪] 连接失败（{self._can}）: {e}", flush=True)
            return False
        self._ready = True
        return True

    def _read_state(self, timeout_s=0.05):
        """读一帧状态，**并如实报告这一帧是不是新收到的**。

        返回 `(GripperState, fresh)`。

        ⚠ 新鲜度只能从 `poll()` 拿，不能从状态对象上拿：本版 litegrip SDK 的
        `GripperState` **没有** `fresh` 字段（只有 position/velocity/torque/
        temperature/error_code/timestamp），而 `timestamp` 是**读取时刻**的本地时间
        —— 它每次都变，拿它当「新帧」等于恒真。`update_state()` 报告「等到一帧没
        有」，但它是 CAN 层的私有方法。公开入口就是 `poll(timeout_s) -> bool`：
        「True if a status frame for this gripper was received」。

        所以是「先 `poll()` 等一帧，再 `get_state(wait=False)` 无等待地取缓存值」；
        反过来（`get_state(wait=True)` 再猜新鲜度）就拿不到这个信号了 —— 而
        `fresh` 正是下游区分「电机真在线」与「进程在线但电机没了」的唯一依据。
        """
        fresh = bool(self._g.poll(timeout_s=timeout_s))
        return self._g.get_state(wait=False), fresh

    def _cache(self, st, fresh=False):
        if st is None:
            return
        self._q = st.position_rad
        self._tau = st.torque_nm
        self._error_code = int(st.error_code)
        self._t_mos = int(getattr(st, "temperature_mos", 0) or 0)
        self._t_coil = int(getattr(st, "temperature_coil", 0) or 0)
        self._position_mm = getattr(st, "position_mm", None)
        # 比例在这里算：发布端是唯一知道「用的是哪份标定」的一方。
        try:
            self._ratio = rad_to_openness(st.position_rad, self._g.config)
        except Exception:  # noqa: BLE001 - 计量不该把主循环打断
            self._ratio = None
        self._fresh = bool(fresh)
        self._last_ok_mono = time.monotonic()

    def _teardown(self):
        try:
            self._g.disconnect()
        except Exception:  # noqa: BLE001
            pass
        self._ready = False

    # ---- 对外命令（非阻塞：只投目标，环下一拍就朝它走）----
    def close(self):
        self._post_target("close")

    def open(self):
        self._post_target("open")

    def _post_target(self, which, from_command=True):
        """投一个目标角度。

        `close` → 标定闭合限位、`open` → 标定张开限位（都用**标定值**）。未标定时
        **不投**：占位限位不是真实行程，拿它当目标就是让夹爪照假数据动。

        `from_command=False` 留给启动归位 —— 它确实投了目标，但**不算**「收到命令」：
        契约里 `target_ratio` 的语义是「操作者最近一次命令」，开机不该变成 open。
        """
        rad = self._end_rad(which)
        with self._lock:
            if rad is not None:
                self._target_rad = rad
            if from_command:
                self._last_cmd_ratio = 0.0 if which == "close" else 1.0

    def _end_rad(self, which):
        """标定的一端（rad）；未标定 → None（只警告一次）。"""
        cfg = self._g.config
        if not getattr(cfg, "calibrated", False):
            if not self._warned_uncalibrated:
                self._warned_uncalibrated = True
                print("[夹爪] 未标定：拒绝投目标（占位限位不是真实行程）。给出 "
                      "--calib PATH 或 --mount normal|reverse（也可设 LITEGRIP_CALIB）。",
                      flush=True)
            return None
        return float(cfg.pos_closed_rad if which == "close" else cfg.pos_open_rad)

    def last_cmd_ratio(self):
        """最近一次命令折算成的目标比例；**从未收到过** = None。

        ⚠ 只有客户端发来的 `close`/`open` 才算「收到命令」。启动归位走
        `_post_target(..., from_command=False)`，**不**经这条路径，所以开机后它仍是
        None —— 契约里那个字段的语义就是「操作者的意图」，不是「开机时夹爪在哪」。
        """
        with self._lock:
            return self._last_cmd_ratio

    def get_state(self):
        # 已连接时实时回读一帧（SDK 的每帧 CAN I/O 都由 io_lock 串行，安全）；
        # 读不到则退回后台线程刷新的缓存。
        if self._ready:
            try:
                st = self._g.get_state()
                self._cache(st)
            except Exception:  # noqa: BLE001
                pass
        return self._q, self._tau

    def snapshot(self):
        """缓存的完整状态（回包字段）—— **零 I/O**，供高频 `status` 轮询。

        `position_mm`/`ratio` 在还没读到过时是 **None**，不拿 0.0 冒充「全闭」：
        0.0 是合法值（真全闭），混在一起下游就分不出「没读到」与「闭合」。
        """
        age_ms = None
        if self._last_ok_mono is not None:
            age_ms = round((time.monotonic() - self._last_ok_mono) * 1000.0, 1)
        return {
            "q": round(self._q, 4), "tau": round(self._tau, 3),
            "enabled": self._error_code == 1, "error_code": self._error_code,
            "fresh": self._fresh,
            "position_mm": (None if self._position_mm is None
                            else round(self._position_mm, 3)),
            "ratio": None if self._ratio is None else round(self._ratio, 4),
            "target_ratio": self.last_cmd_ratio(),
            "age_ms": age_ms,
            # ↓ 控制环自己的诊断。老解析器（status_msgs.parse_gripper_reply）只取它认识
            #   的键，多出来的会被忽略，所以**只增不改**是安全的。
            "t_mos": self._t_mos, "t_coil": self._t_coil,
            "fault_code": self._fault,
            "target_rad": (None if self._target_rad is None
                           else round(self._target_rad, 4)),
            "cmd_rad": (None if self._trajectory is None
                        else round(self._trajectory, 4)),
        }

    def read_now(self):
        """**活读一帧**（等一帧新状态）→ 更新缓存 → 返回快照。

        与 `snapshot()` 的分工：`ping` 要活的，`status` 要快的。⚠ `ping` 必须走这条
        而不是「读 q/tau + 从缓存取 mm/ratio」—— 那样同一份回包里会混两种时刻的值。
        """
        st, fresh = self._read_state()
        self._cache(st, fresh)
        return self.snapshot()

    def shutdown(self):
        self._stop.set()
        self._worker.join(timeout=3.0)
        self._teardown()


class GripperServer:
    """TCP + JSON 服务，串行把命令转成 LiteGripperControl 的 open/close。"""

    def __init__(self, can, can_id, **kw):
        self._g = LiteGripperControl(can=can, can_id=can_id, **kw)
        self._lock = threading.Lock()
        self._can = can              # CAN 接口；随回包上报，供状态端标名

    def handle(self, msg):
        cmd = msg.get("cmd")
        if cmd == "close":
            with self._lock:
                self._g.close()
            return {"ok": True, "cmd": cmd}
        if cmd == "open":
            with self._lock:
                self._g.open()
            return {"ok": True, "cmd": cmd}
        if cmd == "ping":
            # 活读一帧（等一帧新状态）→ 更新缓存 → 回整份快照。
            return {"ok": True, "cmd": cmd, "can": self._can, **self._g.read_now()}
        if cmd == "status":
            # 零 I/O：直接回后台线程刷新的缓存（高频轮询用；ping 会活读一帧，较慢）。
            return {"ok": True, "cmd": cmd, "can": self._can, **self._g.snapshot()}
        return {"ok": False, "error": f"unknown cmd {cmd!r}"}

    def serve(self, host, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((host, port))
            srv.listen(5)
            print(f"[夹爪服务] 监听 {host}:{port}", flush=True)
            while True:
                conn, addr = srv.accept()
                print(f"[夹爪服务] 客户端接入: {addr}", flush=True)
                threading.Thread(target=self._serve_conn, args=(conn, addr),
                                 daemon=True).start()

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
                        try:
                            resp = self.handle(msg)
                        except Exception as e:  # noqa: BLE001
                            resp = {"ok": False, "error": repr(e)}
                    try:
                        conn.sendall((json.dumps(resp) + "\n").encode())
                    except OSError:
                        return
        print(f"[夹爪服务] 客户端断开: {addr}", flush=True)

    def shutdown(self):
        self._g.shutdown()


def main():
    p = argparse.ArgumentParser(description="夹爪常驻 CAN 服务（TCP + JSON，litearm litegrip SDK）")
    p.add_argument("--can", default="can0", help="CAN 接口（默认 can0）")
    p.add_argument("--can-id", type=lambda s: int(s, 0), default=0x08,
                   help="夹爪电机 CAN ID（默认 0x08）")
    p.add_argument("--mst-id", type=lambda s: int(s, 0), default=None,
                   help="状态帧 master ID（默认自动探测）")
    p.add_argument("--mount", choices=("normal", "reverse"), default=None,
                   help="装载向模板（正装/反装），不装则读出厂标定")
    p.add_argument("--calib", default=None,
                   help="显式标定 JSON 路径（本机实测的标定；不点名不读）")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=9003)
    p.add_argument("--speed", type=float, default=SPEED_MM_S, metavar="MM_S",
                   help=f"命令轨迹的推进速度上限 mm/s（默认 {SPEED_MM_S:g}）。⚠ 它限的是"
                        f"**命令轨迹**（每拍朝目标最多走这么多），不是 MIT 帧的 dq 字段"
                        f"（那恒为 0）")
    p.add_argument("--force", type=float, default=FORCE_N, metavar="N",
                   help=f"闭合侧的力矩预算 N（默认 {FORCE_N:g}）。**夹持力就是它**：帧里"
                        f"要求的力矩被「预算/kp」的位置钳位封住，所以撞上工件时压多大只由"
                        f"这个数定（{FORCE_N:g} N ≈ {FORCE_N * N_TO_NM:g} Nm）")
    p.add_argument("--rate-hz", type=float, default=RATE_HZ, metavar="HZ",
                   help=f"控制环周期 Hz（默认 {RATE_HZ:g}，照 litegrip-cpp 的 control_rate_hz）")
    p.add_argument("--kp", type=float, default=KP_CEIL, metavar="KP",
                   help=f"位置增益上限（默认 {KP_CEIL:g}）；实际取值还会被预算按最坏情况压小")
    p.add_argument("--kd", type=float, default=KD_CEIL, metavar="KD",
                   help=f"阻尼上限（默认 {KD_CEIL:g}）")
    p.add_argument("--max-pos-error-mm", type=float, default=MAX_POS_ERROR_MM, metavar="MM",
                   help=f"预算分配用的「最坏位置误差」mm（默认 {MAX_POS_ERROR_MM:g}）："
                        f"kp ≤ 预算/它；0 = 不设限（kp 只用 --kp）")
    p.add_argument("--max-fb-vel", type=float, default=MAX_FB_VEL_RAD_S, metavar="RAD_S",
                   help=f"预算分配用的「最坏反馈速度」rad/s（默认 {MAX_FB_VEL_RAD_S:g}）")
    p.add_argument("--feedback-timeout", type=float, default=FEEDBACK_TIMEOUT_S, metavar="S",
                   help=f"多久没有新反馈帧算掉线（默认 {FEEDBACK_TIMEOUT_S:g}s，"
                        f"照 C++ 的 feedback_timeout_s）")
    p.add_argument("--temp-limit", type=float, default=TEMP_LIMIT_C, metavar="C",
                   help=f"MOS/线圈温度上限 ℃（默认 {TEMP_LIMIT_C:g}），超限即安全停")
    p.add_argument("--status-hz", type=float, default=STATUS_HZ,
                   help="**还没收到目标**（limp）时刷新状态缓存的频率 Hz（默认 20；"
                        "status 命令走缓存零 CAN I/O，此值只影响那一档的主动读）")
    p.add_argument("--no-startup-disable", action="store_true",
                   help="启动时不失能夹爪（默认启动即 disable，避免上电保持在零位≈闭合）")
    p.add_argument("--no-startup-open", action="store_true",
                   help="启动时不自动 open（默认启动即投张开目标，"
                        "使头显扳机第一下为「闭合」）")
    args = p.parse_args()

    server = GripperServer(args.can, args.can_id,
                           mst_id=args.mst_id, mount=args.mount, calib=args.calib,
                           speed=args.speed, force_n=args.force, rate_hz=args.rate_hz,
                           kp=args.kp, kd=args.kd,
                           max_pos_error_mm=args.max_pos_error_mm,
                           max_fb_vel=args.max_fb_vel,
                           feedback_timeout=args.feedback_timeout,
                           temp_limit=args.temp_limit,
                           status_hz=args.status_hz,
                           startup_disable=not args.no_startup_disable,
                           startup_open=not args.no_startup_open)
    try:
        server.serve(args.host, args.port)
    except KeyboardInterrupt:
        print("\n[夹爪服务] 收到中断，退出", flush=True)
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()

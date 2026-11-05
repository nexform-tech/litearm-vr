#!/usr/bin/env python3
"""手柄遥操服务（单进程，单/双臂）。数据源二选一：

  --source zenoh（**出厂**）  订阅中间层 zenoh 的 `litearm/v4/{arm}/vr_frame`，
                              帧由 litearm-vr-bridge 从 8765 转发过来（桥的
                              话题约定见其 `src/litearm_vr_bridge/protocol.py`）。
  --source ws                 自建 WS 监听（`--host/--port`），头显/浏览器直连本机。

两条路收到的都是**同一种帧**，之后完全同构：解析 → litearm_teleop.teleop_sim
的 pybullet 遥操仿真（同进程内 import，无别的中间层）。

数据格式（每一帧是一条 JSON 文本）：
  {
    "leftController":  { position{x,y,z}, quaternion{x,y,z,w}, rotation{x,y,z},
                          gripActive, trigger, tracked },
    "rightController": { ...同上... },
    "timestamp": 1787641468151   # 毫秒
  }

遥操语义（gripActive 代替原 A/B 键，左右手各自独立）：
  - gripActive 上升沿 → 启动（记录当前真实位姿为参考）
  - gripActive 下降沿 → 暂停
  - position/quaternion 为固定握持锚点的绝对位姿输入
  - trigger 上升沿 → 灵巧手握拳，下降沿 → 张开（需 --hand）

用法:
  python scripts/ws_echo_server.py [--source zenoh|ws] [--host 0.0.0.0] [--port 8765]
      [--arm-id armA] [--zenoh-mode client|peer|router]
      [--zenoh-connect tcp/127.0.0.1:7447] [--zenoh-listen tcp/0.0.0.0:7447]
      [--dual] [--upright] [--side] [--viewer gui|rviz] [--ik pin|pybullet]
      [--tcp-offset X Y Z]
      [--real] [--hand --hand-remote HOST:PORT] [--gripper --gripper-remote HOST:PORT]
"""
import argparse
import asyncio
import json
import os
import socket
import sys
import threading
import time

import numpy as np
import websockets

from litearm_teleop.config import DEFAULT_PATH, get, load, set_path
from litearm_teleop.teleop_sim import (DEFAULT_TCP_OFFSET, LiteArmTeleopSim,
                                       sim_config_from_config, tuning_from_config)
from litearm_teleop.vr_source import (DEFAULT_ARM_ID, DEFAULT_SOURCE,
                                      DEFAULT_ZENOH_CONNECT, DEFAULT_ZENOH_LISTEN,
                                      DEFAULT_ZENOH_MODE, ZenohFrameSource,
                                      frame_topic)

# ROS2 发布（可选；--no-ros 或环境无 rclpy 时禁用）
try:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from ros_joint_state import JointStatePublisher
    _HAS_ROS = True
except Exception:
    _HAS_ROS = False


# 客户端 controller 字段名 → 遥操 side（左手→left 臂，右手→right 臂）
HAND_TO_SIDE = {"leftController": "left", "rightController": "right"}

# trigger 模拟量阈值：> 该值视为「按下」
TRIGGER_THRESHOLD = 0.5

# trigger 消抖：状态需稳定此秒数才确认边沿（过滤抖动）
DEBOUNCE_S = 0.05
# 最小闭合时间：闭合/握拳后至少保持此秒数才响应松开，
# 防止按动过快导致「刚闭合没到位就松开」
MIN_CLOSE_S = 0.5

# 旧配置兼容值：固定锚点模式下grip不触发回home
DOUBLE_CLICK_S = 0.35

# zenoh 取帧的轮询超时（秒）：`recv` 每这么久返回一次 None，主循环据此让出事件循环，
# 也让 Ctrl-C 时后台线程最多再等一个超时就结束（见 vr_source.ZenohFrameSource.recv）。
RECV_TIMEOUT_S = 0.5


def _parse_controller(c):
    """单个 controller 对象 → (pos[3], quat_xyzw[4], gripActive, trigger, a_button)。"""
    pos = np.array([c["position"]["x"], c["position"]["y"], c["position"]["z"]])
    quat = np.array([c["quaternion"]["x"], c["quaternion"]["y"],
                     c["quaternion"]["z"], c["quaternion"]["w"]])
    grip = bool(c.get("gripActive", False))
    trigger = float(c.get("trigger", 0.0))
    a_btn = bool(c.get("aButton") or c.get("a_button") or c.get("ax_button")
                 or c.get("buttonA") or c.get("a", False))
    return pos, quat, grip, trigger, a_btn


def _parse_addr(s):
    """'host:port' → (host, int(port))。"""
    host, _, port = str(s).partition(":")
    return (host, int(port))


class GripperRemoteClient:
    """远端夹爪 TCP 客户端：把 close/open 发给工控机上的 gripper_server_can.py。

    夹爪 DM 电机接在另一台工控机（如 192.168.66.95）的 CAN 总线上，本机没有 CAN
    直连，故通过 TCP + JSON 桥接。命令频率低（仅 trigger 边沿），fire-and-forget
    即可，无需读空响应。
    """

    def __init__(self, addr):
        self._addr = _parse_addr(addr) if isinstance(addr, str) else addr
        self._sock = None
        self._lock = threading.Lock()
        self._last_connect = 0.0

    def _connect(self):
        with self._lock:
            s = self._sock
        if s is not None:
            return s
        now = time.monotonic()
        if now - self._last_connect < 2.0:
            return None  # 连接失败后 2s 内不重试
        self._last_connect = now
        try:
            s = socket.create_connection(self._addr, timeout=1.0)
            s.settimeout(1.0)
        except OSError as e:
            print(f"[夹爪][远端] 连接失败 {self._addr}: {e}", flush=True)
            return None
        with self._lock:
            self._sock = s
        print(f"[夹爪][远端] 已连接 {self._addr}", flush=True)
        return s

    def _drop(self, s):
        with self._lock:
            if self._sock is s:
                self._sock = None
        try:
            s.close()
        except OSError:
            pass

    def _send(self, cmd):
        s = self._connect()
        if s is None:
            return
        try:
            s.sendall((json.dumps({"cmd": cmd}) + "\n").encode())
        except OSError as e:
            print(f"[夹爪][远端] 发送失败: {e}", flush=True)
            self._drop(s)

    def close(self):
        self._send("close")

    def open(self):
        self._send("open")

    def shutdown(self):
        with self._lock:
            s = self._sock
            self._sock = None
        if s is not None:
            try:
                s.close()
            except OSError:
                pass


class HandRemoteClient:
    """远端灵巧手 TCP 客户端：把 close/open(+side) 发给工控机上的 hand_server.py。

    L6 灵巧手接在另一台工控机（如 192.168.66.95）的 CAN 总线上，本机没有 CAN 直连，
    故通过 TCP + JSON 桥接。命令频率低（仅 trigger 边沿），fire-and-forget 即可，
    无需读空响应。side 为 "left" / "right"，分别驱动对应手的握拳/张开。
    """

    def __init__(self, addr):
        self._addr = _parse_addr(addr) if isinstance(addr, str) else addr
        self._sock = None
        self._lock = threading.Lock()
        self._last_connect = 0.0

    def _connect(self):
        with self._lock:
            s = self._sock
        if s is not None:
            return s
        now = time.monotonic()
        if now - self._last_connect < 2.0:
            return None  # 连接失败后 2s 内不重试
        self._last_connect = now
        try:
            s = socket.create_connection(self._addr, timeout=1.0)
            s.settimeout(1.0)
        except OSError as e:
            print(f"[灵巧手][远端] 连接失败 {self._addr}: {e}", flush=True)
            return None
        with self._lock:
            self._sock = s
        print(f"[灵巧手][远端] 已连接 {self._addr}", flush=True)
        return s

    def _drop(self, s):
        with self._lock:
            if self._sock is s:
                self._sock = None
        try:
            s.close()
        except OSError:
            pass

    def _send(self, cmd, side):
        s = self._connect()
        if s is None:
            return
        try:
            s.sendall((json.dumps({"cmd": cmd, "side": side}) + "\n").encode())
        except OSError as e:
            print(f"[灵巧手][远端] 发送失败: {e}", flush=True)
            self._drop(s)

    def grip(self, side):
        self._send("close", side)

    def release(self, side):
        self._send("open", side)

    def shutdown(self):
        with self._lock:
            s = self._sock
            self._sock = None
        if s is not None:
            try:
                s.close()
            except OSError:
                pass


class TeleopBridge:
    """WS 手柄帧 → teleop_sim 遥操语义（左右手各自独立）。

    维护每只手的 gripActive 上一状态，检测上升/下降沿，
    映射为对应臂的 start / pause，然后把位姿喂给对应臂的 feed_pose。

    手柄语义的四个时间/阈值参数可配（见 config/teleop.yaml 的 `controller` 段），
    默认值 = 本模块常量，一字不改。
    """

    def __init__(self, sim, ros_pub=None, real_client=None, hand=None, gripper=None,
                 trigger_threshold=TRIGGER_THRESHOLD, debounce_s=DEBOUNCE_S,
                 min_close_s=MIN_CLOSE_S, double_click_s=DOUBLE_CLICK_S):
        self._sim = sim
        self._ros_pub = ros_pub
        self._real = real_client
        self._hand = hand
        self._gripper = gripper
        self._trigger_threshold = float(trigger_threshold)
        self._debounce_s = float(debounce_s)
        self._min_close_s = float(min_close_s)
        # double_click_s retained for config compatibility; grip never commands home.
        self._grip_prev = {"left": False, "right": False}
        self._a_prev = {"left": False, "right": False}
        self._blocked = {"left": False, "right": False}
        self._engaged = {"left": False, "right": False}
        self._starting = {"left": None, "right": None}
        self._last_seen = {"left": None, "right": None}
        self._last_pose = {}
        self._tracking_missing = {"left": False, "right": False}
        self._homing_started = {"left": False, "right": False}
        self._last_timestamp = None
        self._timestamp_origin = None
        self._arrival_origin = None
        self._last_frame = None

        self._trigger_prev = {"left": False, "right": False}   # 稳定状态（消抖后）
        self._trigger_raw = {"left": False, "right": False}    # 原始状态
        self._trigger_since = {"left": 0.0, "right": 0.0}      # 原始状态变化时刻
        self._last_close = {"left": -1e9, "right": -1e9}       # 上次闭合时刻
        self._n = 0

    def hold(self, side, reason):
        arm = self._sim.arms()[side]
        was_engaged = self._engaged[side] or self._starting[side] is not None
        self._starting[side] = None
        newly_blocked = not self._blocked[side]
        if newly_blocked and (arm._active or arm._hold_reason != reason):
            arm.hold(reason)
        self._blocked[side] = True
        self._engaged[side] = False
        if self._real is not None and (was_engaged or newly_blocked):
            self._real.hold(side)

    def input_fault(self, reason):
        for side in self._sim.arms():
            self.hold(side, reason)

    def disconnect(self):
        self.input_fault("输入连接断开")
        self._last_timestamp = None
        self._timestamp_origin = None
        self._arrival_origin = None

    def handle_frame(self, data):
        now = time.monotonic()
        if not isinstance(data, dict):
            self.input_fault("无效输入帧")
            return
        if "teleopInputFault" in data:
            self.input_fault(str(data["teleopInputFault"]))
            return
        # timestamp 为源毫秒时钟；仅比较相对经过时间，不要求两机时钟同步。
        stamp = data.get("timestamp")
        if stamp is not None:
            try:
                stamp = float(stamp)/1000.0
                if not np.isfinite(stamp):
                    raise ValueError("timestamp")
            except (ValueError, TypeError):
                self.input_fault("无效时间戳")
                return
            if self._last_timestamp is not None and stamp <= self._last_timestamp:
                if self._last_frame is None or now-self._last_frame <= self._sim._tuning.tracking["input_timeout"]:
                    return  # 乱序/重复输入不能刷新存活时间。
                self.input_fault("源时间戳重启或长时间重复")
                self._timestamp_origin, self._arrival_origin = stamp, now
            if self._timestamp_origin is None:
                self._timestamp_origin, self._arrival_origin = stamp, now
            delay = (now-self._arrival_origin)-(stamp-self._timestamp_origin)
            if delay > self._sim._tuning.tracking["input_timeout"] or delay < -1.5:
                self.input_fault("输入过期或源时钟跳变")
                # 新时钟基线不解除保持；仍需观察到松开再握持。
                self._timestamp_origin, self._arrival_origin = stamp, now
                self._last_timestamp = stamp
                return
            self._last_timestamp = stamp
        self._last_frame = now
        for hand_key, side in HAND_TO_SIDE.items():
            if side not in self._sim.arms():
                continue
            arm = self._sim.arms()[side]
            c = data.get(hand_key)
            if c is None or (isinstance(c, dict) and not c.get("tracked", True)):
                self._tracking_missing[side] = True
                # Button release is meaningful even when the pose is unavailable.
                if isinstance(c, dict) and c.get("gripActive") is False:
                    if self._engaged[side] or self._starting[side] is not None:
                        self._sim.pause(side)
                        if self._real is not None:
                            self._real.hold(side)
                    self._engaged[side] = False
                    self._starting[side] = None
                    self._grip_prev[side] = False
                continue
            if not isinstance(c, dict):
                self.hold(side, "无效手柄数据")
                continue
            try:
                pos, quat, grip, trigger, a_btn = _parse_controller(c)
                if (not np.all(np.isfinite(pos)) or not np.all(np.isfinite(quat))
                        or np.linalg.norm(quat) < 1e-8 or not np.isfinite(trigger)):
                    raise ValueError("invalid pose")
            except (ValueError, TypeError, KeyError):
                self.hold(side, "无效手柄位姿")
                continue

            a_prev = self._a_prev[side]
            if a_btn and not a_prev:
                print("[遥操][%s] A 键按下 → 启动平滑回零" % side, flush=True)
                # 未握持时仿真臂只是出厂 home，未必等于真机实测位姿（尤其刚重启，
                # 仿真从 home 起步）。不先同步就等于拿「仿真已在 home」去算回零，
                # 整段直接 no-op —— 真机不动。按实测同步后再插值。
                if self._real is not None and not self._engaged[side]:
                    state = self._real.get_state(
                        side, arm._tuning.tracking["feedback_timeout"])
                    if state is not None:
                        try:
                            arm.sync_measured(state["q"])
                        except ValueError as error:
                            print("[保持][%s] A 回零前实机同步失败: %s" % (side, error),
                                  flush=True)
                if hasattr(self._sim, "start_homing"):
                    self._sim.start_homing(side, now)
                elif hasattr(arm, "start_homing"):
                    arm.start_homing(now)
                self._engaged[side] = False
                self._blocked[side] = False
                self._starting[side] = None
            self._a_prev[side] = a_btn
            if (self._tracking_missing[side] and self._last_seen[side] is not None
                    and now-self._last_seen[side] > arm._tuning.tracking["input_timeout"]
                    and (self._engaged[side] or self._starting[side] is not None)):
                self.hold(side, "手柄追踪持续丢失")
            self._tracking_missing[side] = False
            self._last_seen[side] = now
            self._last_pose[side] = (pos, quat)
            prev = self._grip_prev[side]
            if not grip:
                if prev or self._engaged[side] or self._starting[side] is not None:
                    self._sim.pause(side)
                    self._engaged[side] = False
                    self._starting[side] = None
                    if self._real is not None:
                        self._real.hold(side)
                self._blocked[side] = False
            elif not prev and not self._blocked[side]:
                # 每次握持以新鲜实测构型锚定，绝不自动 movej 到 home。
                state = None
                if self._real is not None:
                    state = self._real.get_state(side, arm._tuning.tracking["feedback_timeout"])
                    if state is None:
                        self.hold(side, "缺少新鲜实机关节反馈")
                if self._real is None:
                    try:
                        self._sim.start(side, pos, quat, now)
                        self._engaged[side] = True
                    except ValueError as error:
                        self.hold(side, str(error))
                elif state is not None:
                    self._real.start(side)
                    self._starting[side] = now
            self._grip_prev[side] = grip

            # trigger 边沿 → 灵巧手/夹爪，带消抖 + 最小闭合时间。
            # 末端执行器二选一（同时下发，只有一个 CAN ID 回应）。
            pressed = trigger > self._trigger_threshold
            now = time.monotonic()

            raw_prev = self._trigger_raw[side]
            if pressed != raw_prev:
                self._trigger_since[side] = now
                self._trigger_raw[side] = pressed

            stable = self._trigger_prev[side]
            if pressed != stable and (now - self._trigger_since[side]) >= self._debounce_s:
                if pressed:
                    # 上升沿 → 握拳/夹爪闭合
                    self._trigger_prev[side] = True
                    self._last_close[side] = now
                    if self._hand is not None:
                        self._hand.grip(side)
                    if self._gripper is not None:
                        # 现场排查用：一次扳机**只能打一行**。多行 = 触发边沿在反复翻
                        # （阈值是硬判据 `> 0.5` 且无滞回，手柄停在阈值附近就会来回跨），
                        # 见 litearm-vr-follow/docs/gripper-jitter-analysis.md §5.2。
                        print("[夹爪][遥操][%s] trigger %.3f 上升沿 → close" % (side, trigger),
                              flush=True)
                        self._gripper.close()
                else:
                    # 下降沿 → 张开，但受最小闭合时间限制（防止没到位就松开）
                    if now - self._last_close[side] >= self._min_close_s:
                        self._trigger_prev[side] = False
                        if self._hand is not None:
                            self._hand.release(side)
                        if self._gripper is not None:
                            print("[夹爪][遥操][%s] trigger %.3f 下降沿 → open"
                                  "（闭合保持 %.2fs）"
                                  % (side, trigger, now - self._last_close[side]),
                                  flush=True)
                            self._gripper.open()
                    # 仍在最小闭合窗口内：保持稳定 pressed，等窗口结束自动张开

            # 每帧喂位姿（暂停帧由 teleop_sim 内部喂零增量）
            self._sim.update_pose(side, pos, quat, now)
            if self._engaged[side] and not arm._active:
                self.hold(side, arm._hold_reason or "目标跟踪停止")
        self._n += 1

    def tick(self, now=None):
        """固定频率控制，与输入帧到达解耦。"""
        now = time.monotonic() if now is None else now
        measured = {}
        for side, arm in self._sim.arms().items():
            if (self._tracking_missing[side] and self._last_seen[side] is not None
                    and now-self._last_seen[side] > arm._tuning.tracking["input_timeout"]
                    and (self._engaged[side] or self._starting[side] is not None)):
                self.hold(side, "手柄追踪持续丢失")
            if self._real is not None:
                self._real.request_state(side)
                state = self._real.get_state(side, arm._tuning.tracking["feedback_timeout"])
                if self._starting[side] is not None:
                    if now-self._last_seen[side] > arm._tuning.tracking["input_timeout"]:
                        self.hold(side, "等待启动时手柄输入超时")
                    elif self._real.command_error(side):
                        self.hold(side, "实机启动失败: " + self._real.command_error(side))
                    elif now-self._starting[side] > 5.0:
                        self.hold(side, "等待实机启动确认超时")
                    elif (self._real.is_started(side) and state is not None
                          and not self._tracking_missing[side]):
                        try:
                            arm.sync_measured(state["q"])
                            pos, quat = self._last_pose[side]
                            self._sim.start(side, pos, quat, now)
                            self._engaged[side] = True
                            self._starting[side] = None
                        except ValueError as error:
                            self.hold(side, str(error))
                if self._engaged[side] and self._real.command_error(side):
                    self.hold(side, "实机命令失败: " + self._real.command_error(side))
                if self._engaged[side] and state is None:
                    self.hold(side, "实机关节反馈超时")
                elif state is not None:
                    measured[side] = state
            homing = getattr(arm, "is_homing", lambda: False)()
            if homing:
                if self._real is not None and not self._homing_started[side]:
                    # 回零（A/X 或 GUI 键）不受 grip 触发，客户端不会走 start 握手；
                    # 而真机服务端只在 following=True 时执行 q 帧，空闲/刚重启的服务会
                    # 把整段回零帧静默丢弃（仿真动、真机不动）。start 幂等（已在跟随
                    # 时服务端直接返回 already）；发送失败（尚未连上）时下一拍重试。
                    self._homing_started[side] = self._real.start(side)
            else:
                self._homing_started[side] = False
            if arm._active and self._tracking_missing[side]:
                arm.suspend_tracking(now)
            elif arm._active or homing:
                # 回零（A/X 键）期间 _active 为 False，但插值仍要按控制周期推进，
                # 否则 start_homing 只打日志、手臂永远不动。
                try:
                    feedback = measured.get(side)
                    arm.tick(now,
                             None if feedback is None else feedback["q"],
                             None if feedback is None else feedback.get("cmd_q"))
                except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
                    self.hold(side, "控制求解失败: %s" % error)
                if self._engaged[side] and not arm._active:
                    self.hold(side, arm._hold_reason or "目标跟踪停止")
        names, positions = self._sim.get_joint_states()
        if self._ros_pub is not None:
            self._ros_pub.publish(names, positions)
        if self._real is not None:
            self._send_real(positions)
        # marker 已在各 arm.tick() 内刷新（并已按 10/2 Hz 限流），此处不再重复绘制：
        # 重复一次等于加倍 GUI 同步 IPC，会把控制环拖慢、饿死真机跟随看门狗。
        if hasattr(self._sim, "poll_keyboard"):
            self._sim.poll_keyboard()

    def _send_real(self, positions):
        """把仿真关节角拆成左右臂各 7 维，逐帧下发给真机桥接客户端。"""
        arms = self._sim.arms()
        ordered = [side for side in ("left", "right") if side in arms]
        for index, side in enumerate(ordered):
            arm = arms[side]
            is_homing = getattr(arm, "is_homing", lambda: False)()
            if (self._engaged[side] and arm._active) or is_homing:
                self._real.send_q(side, positions[index*7:(index+1)*7])


async def _control_loop(bridge):
    period = 1.0 / bridge._sim._cfg.freq
    while True:
        started = time.monotonic()
        bridge.tick(started)
        await asyncio.sleep(max(0.001, period-(time.monotonic()-started)))


async def main(host, port, dual, side, use_ros, viewer, ik, real, real_left, real_right,
               hand, hand_remote, gripper, gripper_remote,
               tcp_offset=None, tuning=None, probe=None, sim_config=None,
               upright=False, controller_kwargs=None,
               source=DEFAULT_SOURCE, arm_id=DEFAULT_ARM_ID,
               zenoh_mode=DEFAULT_ZENOH_MODE,
               zenoh_connect=DEFAULT_ZENOH_CONNECT,
               zenoh_listen=DEFAULT_ZENOH_LISTEN):
    # 单进程：直接实例化 pybullet 遥操仿真（viewer 决定显示后端）
    sim = LiteArmTeleopSim(dual=dual, viewer=viewer, side=side, upright=upright,
                           ik_backend=ik,
                           tcp_offset=tcp_offset, tuning=tuning, probe=probe,
                           sim_config=sim_config)

    ros_pub = None
    if use_ros and _HAS_ROS:
        ros_pub = JointStatePublisher()
        print("ROS2 /joint_states 发布已启动")
    elif use_ros:
        print("[警告] rclpy 不可用，跳过 ROS2 发布（用 --no-ros 静默）")

    real_client = None
    if real:
        from real_arm_client import RealArmClient
        real_client = RealArmClient(left_addr=real_left, right_addr=real_right)
        print("真机桥接已启用（左臂 %s，右臂 %s）" % (real_left, real_right))

    hand_ctrl = None
    if hand:
        hand_ctrl = HandRemoteClient(hand_remote)
        print("灵巧手控制已启用（远端 TCP=%s，trigger 边沿 → 握拳/张开）" % hand_remote)

    gripper_ctrl = None
    if gripper:
        gripper_ctrl = GripperRemoteClient(gripper_remote)
        print("夹爪控制已启用（远端 TCP=%s，trigger 边沿 → 闭合/张开）"
              % gripper_remote)

    bridge = TeleopBridge(sim, ros_pub=ros_pub, real_client=real_client,
                          hand=hand_ctrl, gripper=gripper_ctrl,
                          **(controller_kwargs or {}))

    print("遥操仿真已就绪（显示=%s，%s）"
          % (viewer, "双臂正装" if (dual and upright)
             else ("双臂侧装" if dual
                   else ("单臂侧装（左臂）" if side else "单臂正装"))))

    control_task = asyncio.create_task(_control_loop(bridge))
    try:
        if source == "zenoh":
            await _serve_zenoh(bridge, arm_id, zenoh_mode, zenoh_connect,
                               zenoh_listen)
        else:
            await _serve_ws(bridge, host, port)
    finally:
        control_task.cancel()
        try:
            await control_task
        except asyncio.CancelledError:
            pass
        bridge.disconnect()
        if ros_pub is not None:
            ros_pub.shutdown()
        if real_client is not None:
            real_client.close()
        if hand_ctrl is not None:
            hand_ctrl.shutdown()
        if gripper_ctrl is not None:
            gripper_ctrl.shutdown()
        sim.close()


async def _serve_ws(bridge, host, port):
    async with websockets.serve(lambda ws: _handler(ws, bridge), host, port):
        print("数据源: 自建 WS 监听 ws://{}:{}（头显/浏览器直连本机）".format(host, port))
        await asyncio.Future()  # 持续运行


async def _serve_zenoh(bridge, arm_id, mode, connect, listen):
    """订阅中间层帧流，逐帧喂 TeleopBridge。

    取帧走 `asyncio.to_thread`：zenoh 回调只在队列里放货（见 vr_source），真正的
    解析与 IK 仍在**主线程**（pybullet 不能在别的线程调）。
    """
    source = ZenohFrameSource(frame_topic(arm_id), mode=mode,
                              connect=connect, listen=listen)
    source.start()
    print("数据源: 中间层 zenoh 订阅 %s" % source.describe())
    state = {"n": 0}
    try:
        while True:
            raw = await asyncio.to_thread(source.recv, RECV_TIMEOUT_S)
            if raw is not None:
                _handle_raw(bridge, raw, state)
    finally:
        source.stop()
        print("[zenoh] 收帧 %d，丢帧 %d%s"
              % (source.frames_received, source.frames_dropped,
                 "" if source.last_error is None else "，最近一次: %s" % source.last_error))


def _handle_raw(bridge, raw, state):
    """一条原始 JSON 文本 → 遥操语义。两个数据源共用（首帧打诊断）。"""
    try:
        data = json.loads(raw)
    except (ValueError, TypeError):
        bridge.input_fault("无效 JSON 输入")
        return
    state["n"] += 1
    if state["n"] == 1:
        # 诊断：打印首帧原始数据，确认字段名/结构
        print("[诊断] 首帧原始数据: %s" % raw[:600], flush=True)
    bridge.handle_frame(data)


async def _handler(websocket, bridge):
    print("客户端已连接")
    state = {"n": 0}
    try:
        async for message in websocket:
            _handle_raw(bridge, message, state)
    except websockets.exceptions.ConnectionClosed:
        print("客户端已断开")
    finally:
        bridge.disconnect()


def _build_parser():
    """命令行接口。

    所有 `default=` 都是 `argparse.SUPPRESS`：**没给过**的参数不出现在 namespace
    里，这样它才不会用「命令行的默认值」盖掉配置文件里显式写的值。真正的默认值
    只有一个来源 —— `config/teleop.yaml`（缺键时再退回出厂常量）。
    """
    ap = argparse.ArgumentParser(
        description="LiteArm VR 遥操服务（中间层 zenoh 订阅 / WS 监听 + pybullet 逆解"
                    " + 可选 RViz2）")
    ap.add_argument("--config", default=None, metavar="PATH",
                    help="配置文件（默认 config/teleop.yaml）；命令行显式给的项更优先")
    ap.add_argument("--source", choices=["zenoh", "ws"], default=argparse.SUPPRESS,
                    help="VR 数据源：zenoh=订阅中间层帧流（出厂，话题 "
                         "litearm/v4/{arm}/vr_frame，由 litearm-vr-bridge 转发）/ "
                         "ws=自建 WS 监听，头显直连 --host:--port")
    ap.add_argument("--arm-id", default=argparse.SUPPRESS, metavar="ARM",
                    help="zenoh 话题里的 arm 段（出厂 armA）；必须与桥的 "
                         "ZENOH_ARM_ID、litearm-server --arm-id 一致")
    ap.add_argument("--zenoh-mode", choices=["client", "peer", "router"],
                    default=argparse.SUPPRESS,
                    help="zenoh 会话模式（出厂 peer：连不上只后台重试，桥后起"
                         "也能接上；client 要求开会话时就已连上，桥没起会报错退出）")
    ap.add_argument("--zenoh-connect", default=argparse.SUPPRESS, metavar="EP[,EP…]",
                    help="zenoh 主动连的端点（出厂 tcp/127.0.0.1:7447 = 同机的桥）；"
                         "多个用逗号分隔")
    ap.add_argument("--zenoh-listen", default=argparse.SUPPRESS, metavar="EP[,EP…]",
                    help="zenoh 监听端点（出厂空 = 不监听；同机的桥已经占了 7447）")
    ap.add_argument("--host", default=argparse.SUPPRESS)
    ap.add_argument("--port", type=int, default=argparse.SUPPRESS)
    ap.add_argument("--dual", action=argparse.BooleanOptionalAction,
                    default=argparse.SUPPRESS,
                    help="加载双臂 URDF（默认单臂正装）；配 --upright 为双臂正装")
    ap.add_argument("--upright", action=argparse.BooleanOptionalAction,
                    default=argparse.SUPPRESS,
                    help="与 --dual 同用：双臂正装（两只独立竖直臂、无立柱）；"
                         "不传则双臂侧装立柱")
    ap.add_argument("--side", action=argparse.BooleanOptionalAction,
                    default=argparse.SUPPRESS,
                    help="单臂侧装 = 单左臂（左手柄驱动、home/映射取双臂侧装左臂，"
                         "真机走 :9001）")
    ap.add_argument("--viewer", choices=["gui", "rviz"], default=argparse.SUPPRESS,
                    help="显示后端：gui=pybullet 窗口（默认）/ rviz=RViz2")
    ap.add_argument("--ik", choices=["pin", "pybullet", "exact"],
                    default=argparse.SUPPRESS,
                    help="IK 后端：pybullet=calculateInverseKinematics（出厂）/ "
                         "pin=pinocchio+pink（metavr 策略）/ "
                         "exact=精确任务速度（无阻尼，增益 1 或 0）")
    ap.add_argument("--deadband-scale", type=float, default=argparse.SUPPRESS,
                    metavar="S",
                    help="静止死区阈值的乘数：1.0=出厂值，0.0=关掉死区"
                         "（出厂值在 60Hz 下等于平移 1.8cm/s / 旋转 0.18rad/s）")
    ap.add_argument("--smooth-alpha", type=float, default=argparse.SUPPRESS,
                    metavar="A",
                    help="位姿低通在60Hz下的新样本权重，取值 (0, 1]；1.0=不平滑")
    ap.add_argument("--resume-margin", type=float, default=argparse.SUPPRESS,
                    metavar="M",
                    help="--ik exact 的持位滞回：<1 时恢复需多留余量（1.0=朴素规则）")
    ap.add_argument("--task-residual-tolerance", type=float,
                    default=argparse.SUPPRESS, metavar="T",
                    help="--ik exact 判定「任务被产生」的残差比例上限，出厂 1e-3")
    ap.add_argument("--manip-weight", type=float, default=argparse.SUPPRESS,
                    metavar="W",
                    help="--ik pin 的操纵度任务权重（把冗余推向雅可比更良态的构型，"
                         "预防奇异）；0=关闭（出厂）")
    ap.add_argument("--control-frame", choices=["tool", "base_link"],
                    default=argparse.SUPPRESS,
                    help="旋转增量读在哪个系：tool=末端局部系（出厂，现状）/ "
                         "base_link=基座系（绕世界轴转，与平移同源）")
    ap.add_argument("--base-link-signs", type=float, nargs=3,
                    default=argparse.SUPPRESS, metavar=("X", "Y", "Z"),
                    help="--control-frame base_link 用的逐轴旋转符号（未重测时勿乱改）")
    ap.add_argument("--ros", action=argparse.BooleanOptionalAction,
                    dest="ros", default=argparse.SUPPRESS,
                    help="发布 ROS2 /joint_states（默认开；--no-ros 关掉）")
    ap.add_argument("--tcp-offset", type=float, nargs=3,
                    default=argparse.SUPPRESS, metavar=("X", "Y", "Z"),
                    help="末端偏移量(米，本体系)：操作者控制的工具点相对 tip link 的"
                         "偏移；传 0 0 0 = 控制法兰")
    ap.add_argument("--probe", default=argparse.SUPPRESS, metavar="JSONL",
                    help="数据探针：把每帧各阶段的量写成 JSONL（诊断跳变用，"
                         "见 scripts/probe.py）")
    ap.add_argument("--real", action=argparse.BooleanOptionalAction,
                    default=argparse.SUPPRESS,
                    help="启用真机桥接：把仿真关节角下发到 arm_follow_server")
    ap.add_argument("--real-left", default=argparse.SUPPRESS,
                    help="左臂 arm_follow_server 地址 host:port")
    ap.add_argument("--real-right", default=argparse.SUPPRESS,
                    help="右臂 arm_follow_server 地址 host:port")
    ap.add_argument("--hand", action=argparse.BooleanOptionalAction,
                    default=argparse.SUPPRESS,
                    help="启用灵巧手控制：trigger 边沿 → 远端 hand_server 握拳/张开")
    ap.add_argument("--hand-remote", default=argparse.SUPPRESS,
                    help="灵巧手服务 hand_server.py 地址 host:port（--hand 时必填）")
    ap.add_argument("--gripper", action=argparse.BooleanOptionalAction,
                    default=argparse.SUPPRESS,
                    help="启用夹爪控制：trigger 边沿 → 远端 gripper_server 闭合/张开")
    ap.add_argument("--gripper-remote", default=argparse.SUPPRESS,
                    help="夹爪服务 gripper_server.py 地址 host:port（--gripper 时必填）")
    ap.add_argument("--trigger-threshold", type=float, default=argparse.SUPPRESS,
                    help="扳机模拟量 > 该值视为「按下」")
    ap.add_argument("--debounce-s", type=float, default=argparse.SUPPRESS,
                    help="扳机消抖秒数")
    ap.add_argument("--min-close-s", type=float, default=argparse.SUPPRESS,
                    help="最小闭合时间（秒）")
    ap.add_argument("--double-click-s", type=float, default=argparse.SUPPRESS,
                    help="旧配置兼容项，固定锚点模式不再用双击grip回home")
    return ap


# CLI 选项 → 配置文件路径。命令行**显式给过**的项按这张表覆盖配置文件。
_OPTION_PATHS = {
    "source": "vr.source",
    "arm_id": "vr.arm_id",
    "zenoh_mode": "vr.zenoh_mode",
    "zenoh_connect": "vr.zenoh_connect",
    "zenoh_listen": "vr.zenoh_listen",
    "host": "server.host",
    "port": "server.port",
    "dual": "robot.dual",
    "upright": "robot.upright",
    "side": "robot.side",
    "viewer": "robot.viewer",
    "probe": "robot.probe",
    "ik": "tuning.ik",
    "tcp_offset": "tuning.tcp_offset",
    "deadband_scale": "tuning.deadband_scale",
    "smooth_alpha": "tuning.smooth_alpha",
    "resume_margin": "tuning.resume_margin",
    "task_residual_tolerance": "tuning.task_residual_tolerance",
    "manip_weight": "tuning.manip_weight",
    "control_frame": "tuning.control_frame",
    "base_link_signs": "tuning.base_link_signs",
    "ros": "server.publish_ros",
    "real": "real.enabled",
    "real_left": "real.left_addr",
    "real_right": "real.right_addr",
    "hand": "effector.hand",
    "hand_remote": "effector.hand_remote",
    "gripper": "effector.gripper",
    "gripper_remote": "effector.gripper_remote",
    "trigger_threshold": "controller.trigger_threshold",
    "debounce_s": "controller.debounce_s",
    "min_close_s": "controller.min_close_s",
    "double_click_s": "controller.double_click_s",
}


def _merge_args(cfg_data, args):
    """配置文件打底，命令行**显式给过**的项覆盖它。"""
    for dest, path in _OPTION_PATHS.items():
        if hasattr(args, dest):
            set_path(cfg_data, path, getattr(args, dest))
    return cfg_data


if __name__ == "__main__":
    parser = _build_parser()
    args = parser.parse_args()
    print("[配置] %s" % (args.config or DEFAULT_PATH))
    cfg = _merge_args(load(args.config), args)

    def C(path, default):
        return get(cfg, path, default)

    # 灵巧手/夹爪的实现已移出本仓（只留远端 TCP 客户端），所以开关与地址必须成对：
    # 只开了开关却没给地址，服务端会静默什么都不做，操作者会以为「按了没反应」是硬件问题。
    if C("effector.hand", False) and not C("effector.hand_remote", None):
        parser.error("effector.hand 需要 effector.hand_remote host:port"
                     "（灵巧手服务在电机所在的工控机上，见 litearm-vr-follow/hand_server.py）")
    if C("effector.gripper", False) and not C("effector.gripper_remote", None):
        parser.error("effector.gripper 需要 effector.gripper_remote host:port"
                     "（夹爪服务在电机所在的工控机上，见 litearm-vr-follow/gripper_server.py）")

    probe = None
    probe_path = C("robot.probe", None)
    if probe_path:
        from probe import JsonlWriter
        probe = JsonlWriter(probe_path)
        print("数据探针已启用 → %s（诊断用，正常遥操请勿开）" % probe_path)

    asyncio.run(main(
        C("server.host", "0.0.0.0"), int(C("server.port", 8765)),
        bool(C("robot.dual", False)), bool(C("robot.side", False)),
        use_ros=bool(C("server.publish_ros", True)),
        viewer=C("robot.viewer", "gui"), ik=C("tuning.ik", "pybullet"),
        upright=bool(C("robot.upright", False)),
        source=C("vr.source", DEFAULT_SOURCE),
        arm_id=C("vr.arm_id", DEFAULT_ARM_ID),
        zenoh_mode=C("vr.zenoh_mode", DEFAULT_ZENOH_MODE),
        zenoh_connect=C("vr.zenoh_connect", DEFAULT_ZENOH_CONNECT),
        zenoh_listen=C("vr.zenoh_listen", DEFAULT_ZENOH_LISTEN),
        real=bool(C("real.enabled", False)),
        real_left=C("real.left_addr", "127.0.0.1:9001"),
        real_right=C("real.right_addr", "127.0.0.1:9002"),
        hand=bool(C("effector.hand", False)),
        hand_remote=C("effector.hand_remote", None),
        gripper=bool(C("effector.gripper", False)),
        gripper_remote=C("effector.gripper_remote", None),
        tcp_offset=C("tuning.tcp_offset", list(DEFAULT_TCP_OFFSET)),
        probe=probe,
        tuning=tuning_from_config(cfg),
        sim_config=sim_config_from_config(cfg),
        controller_kwargs={
            "trigger_threshold": C("controller.trigger_threshold", TRIGGER_THRESHOLD),
            "debounce_s": C("controller.debounce_s", DEBOUNCE_S),
            "min_close_s": C("controller.min_close_s", MIN_CLOSE_S),
            "double_click_s": C("controller.double_click_s", DOUBLE_CLICK_S),
        }))

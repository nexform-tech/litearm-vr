#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiteArm 七轴真机遥操跟随服务 (pylitearm SDK 版)。

链路:
  VR(ws_echo_server.py) --TCP/JSON--> 本服务 --CAN socketcan(can0)--> DM 电机
                                                                    (经 STM32 gs_usb 桥)

对外协议与 arm_follow_server.py 完全一致 (VR 端无需改动):
  {"cmd":"home"}            回零 (Arm.home, 阻塞)
  {"cmd":"movej","q":[7]}   关节轨迹到位 (Arm.movej, 阻塞)
  {"cmd":"q","q":[7]}       一帧 7 关节角目标, 逐帧跟随 (Arm.joint_follow)
  {"cmd":"latch"}           就地持位 (停跟随, joint_follow 退出时自动 latch_hold)
  {"cmd":"start"}           开始跟随: connect + 起 joint_follow 后台线程
  {"cmd":"stop"}            急停 (Arm.request_stop → 控制环退出并高刚度持位)
  {"cmd":"ping"}            探活, 返回当前 state/q/fault

⚠️ 与 arm_follow_server_stm32.py (串口 0xA5) 是两条不同的控制路径:
  - 本服务走 can0 (socketcan): STM32 只当 CAN 桥, 控制律在主机 Python (JointFollowController)
  - 串口版走 /dev/ttyACM0: 控制律在 STM32 固件
  - 二者互斥, 不能同时占同一批电机; 跑本服务前先确保 can0 已 up
    (sudo ip link set can0 up type can bitrate 1000000)

⚠️ 真机会真实运动! 运行前确认机械臂上电、工作区无人、急停在手边。

用法:
  python3 arm_follow_server_sdk.py --iface can0 --host 0.0.0.0 --tcp-port 9001 \
      [--kp 7个] [--kd 7个] [--speed-limit 7个] [--accel-limit 7个] [--sp 0.5]
"""
import argparse
import json
import inspect
import os
import socket
import sys
import threading
import time

from measured_state import sdk_response

from pylitearm import Arm, ArmState, default_config_path


def _to7(v):
    try:
        vals = [float(x) for x in v]
    except (TypeError, ValueError):
        return None
    return vals if len(vals) == 7 else None


class ArmFollowServer:
    """接收 VR JSON, 串行驱动 pylitearm.Arm 真机。"""

    def __init__(self, config=None, iface="can0", dry_run=False,
                 kp=None, kd=None, speed_limit=None, accel_limit=None,
                 sp=0.5, engage_sec=0.3):
        self._config = config or default_config_path()
        print(f"[真机] pylitearm.Arm config={self._config} iface={iface} "
              f"dry_run={dry_run}", flush=True)
        self._arm = Arm(self._config, iface=iface, dry_run=dry_run)
        self._iface = iface          # CAN 接口；随 ping 回包上报，供状态端标名
        self._kp = _to7(kp) if kp else None
        self._kd = _to7(kd) if kd else None
        self._speed_limit = _to7(speed_limit) if speed_limit else None
        self._accel_limit = _to7(accel_limit) if accel_limit else None
        self._sp = sp
        self._engage_sec = engage_sec

        self._lock = threading.RLock()
        self._q_target = None      # 最近一帧 VR 目标 [7], None=持位
        self._following = False
        self._holding = False
        self._target_received_at = 0.0
        self._follow_thread = None
        self._feedback_only = False

    # ---- joint_follow 目标提供者 (每控制周期由 SDK 回调) ----
    def _target_provider(self):
        with self._lock:
            if not self._holding and time.monotonic()-self._target_received_at > 0.25:
                self._q_target = None  # lost sender cannot leave a distant target running
            if self._q_target is None:
                return None          # joint_follow 用当前 q_cmd 持位
            return (list(self._q_target), None)

    def _follow_loop(self):
        try:
            self._arm.joint_follow(
                self._target_provider,
                K=self._kp, B=self._kd,
                speed_limit=self._speed_limit, accel_limit=self._accel_limit,
                engage_sec=self._engage_sec)
        except Exception as e:
            print(f"[真机] 跟随退出: {e!r}", flush=True)
        finally:
            with self._lock:
                self._following = False

    def _join_follow(self, timeout=3.0):
        t = self._follow_thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout)

    def _ensure_connected(self):
        if self._feedback_only:
            self._arm.close()  # read-only close never parks/enables the arm
            self._feedback_only = False
        st = self._arm.state
        if st in (ArmState.DISCONNECTED, ArmState.FAULT):
            self._arm.connect()
        elif st == ArmState.DISABLED:
            self._arm.enable()

    # ---- 命令处理 ----
    def handle(self, msg: dict) -> dict:
        cmd = msg.get("cmd")

        if cmd == "start":
            if not callable(getattr(self._arm, "joint_follow", None)):
                return {"ok": False, "cmd": cmd, "error": "SDK does not provide joint_follow"}
            self._holding = False
            if self._following:
                return {"ok": True, "cmd": cmd}
            self._ensure_connected()
            self._arm.request_stop()      # 清掉可能的进行中运动
            self._join_follow()
            with self._lock:
                self._q_target = None
                self._following = True
            t = threading.Thread(target=self._follow_loop, daemon=True,
                                 name="joint_follow")
            self._follow_thread = t
            t.start()
            return {"ok": True, "cmd": cmd}

        if cmd == "q":
            q7 = _to7(msg.get("q")) if isinstance(msg.get("q"), (list, tuple)) else None
            if q7 is None:
                return {"ok": False, "error": "q 必须是 7 个数值"}
            with self._lock:
                self._holding = False
                self._target_received_at = time.monotonic()
                self._q_target = q7
            return {"ok": True, "cmd": cmd}

        if cmd == "movej":
            q7 = _to7(msg.get("q")) if isinstance(msg.get("q"), (list, tuple)) else None
            if q7 is None:
                return {"ok": False, "error": "q 必须是 7 个数值"}
            self._ensure_connected()
            self._arm.request_stop()
            self._join_follow()
            with self._lock:
                self._following = False
            print(f"[真机] movej → {[round(x, 4) for x in q7]} (阻塞中)", flush=True)
            ok = self._arm.movej(q7, speed=self._sp)
            print(f"[真机] movej 到位 ok={ok}", flush=True)
            return {"ok": ok, "cmd": cmd}

        if cmd == "home":
            self._ensure_connected()
            self._arm.request_stop()
            self._join_follow()
            with self._lock:
                self._following = False
            print("[真机] home 回零 (阻塞中)", flush=True)
            ok = self._arm.home(speed=0.3)
            print(f"[真机] home 到位 ok={ok}", flush=True)
            return {"ok": ok, "cmd": cmd}

        if cmd == "hold":
            measured = sdk_response(self._arm, msg.get("request_id"))
            with self._lock:
                # None means the SDK provider holds its actual current command,
                # not the previous remote target. Do not stop/restart the loop.
                if not getattr(self, "_holding", False):
                    self._q_target = list(measured["q"]) if measured["valid"] else None
                self._holding = True
            return {"ok": True, "cmd": cmd, "measured": measured["valid"]}

        if cmd == "latch":
            self._arm.request_stop()      # joint_follow 退出时自动 latch_hold 当前
            self._join_follow()
            with self._lock:
                self._following = False
            return {"ok": True, "cmd": cmd}

        if cmd == "stop":
            self._arm.request_stop()
            with self._lock:
                self._following = False
            return {"ok": True, "cmd": cmd}

        if cmd == "state":
            # Obtain feedback before grip/start without enabling or moving motors.
            if self._arm.state == ArmState.DISCONNECTED:
                if "read_only" not in inspect.signature(self._arm.connect).parameters:
                    return {"ok": True, "cmd": "state", "request_id": msg.get("request_id"),
                            "valid": False, "q": None, "age_s": None,
                            "error": "SDK needs connect(read_only=True) for pre-grip feedback"}
                self._arm.connect(read_only=True)
                self._feedback_only = True
            return sdk_response(self._arm, msg.get("request_id"))

        if cmd == "ping":
            try:
                st = self._arm.get_state()
                return {"ok": True, "state": st["state"], "q": st["q"],
                        "fault": st["fault"], "port": self._iface}
            except Exception as e:
                return {"ok": True, "state": self._arm.state.value, "port": self._iface,
                        "connected": False, "error": str(e)}

        return {"ok": False, "error": f"unknown cmd {cmd!r}"}

    # ---- TCP 服务 (JSON 每行一条, 同 arm_follow_server.py) ----
    def serve(self, host, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((host, port))
            srv.listen(5)
            print(f"[服务] 监听 {host}:{port}", flush=True)
            while True:
                conn, addr = srv.accept()
                print(f"[服务] 客户端接入: {addr}", flush=True)
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
                        except Exception as e:
                            resp = {"ok": False, "error": repr(e)}
                    try:
                        conn.sendall((json.dumps(resp) + "\n").encode())
                    except OSError:
                        return
        print(f"[服务] 客户端断开: {addr}", flush=True)


def main():
    p = argparse.ArgumentParser(description="LiteArm 七轴真机遥操跟随服务 (pylitearm SDK)")
    p.add_argument("--config", default=None, help="litearm.yaml 路径 (缺省用随包默认)")
    p.add_argument("--iface", default="can0", help="CAN 接口 (STM32 gs_usb 桥 = can0)")
    p.add_argument("--dry-run", action="store_true", help="无硬件仿真 (不连 CAN)")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--tcp-port", type=int, default=9001)
    p.add_argument("--kp", type=float, nargs=7, default=None, help="跟随刚度 K 7 个 (Nm/rad)")
    p.add_argument("--kd", type=float, nargs=7, default=None, help="跟随阻尼 B 7 个")
    p.add_argument("--speed-limit", type=float, nargs=7, default=None, help="速度限幅 7 个 (rad/s)")
    p.add_argument("--accel-limit", type=float, nargs=7, default=None, help="加速度限幅 7 个 (rad/s^2)")
    p.add_argument("--sp", type=float, default=0.5, help="movej 速度倍率 0~1")
    p.add_argument("--engage-sec", type=float, default=0.3, help="跟随接管软缓冲秒数")
    args = p.parse_args()

    server = ArmFollowServer(args.config, args.iface, args.dry_run,
                             args.kp, args.kd, args.speed_limit, args.accel_limit,
                             args.sp, args.engage_sec)
    server.serve(args.host, args.tcp_port)


if __name__ == "__main__":
    main()

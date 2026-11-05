#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真机桥接客户端：仿真关节角 → 真机跟随服务（TCP/JSON 协议）。

连两条 TCP 长连接（左右臂各一个 arm_follow_server 实例），把仿真 IK 解算的
关节角逐帧下发，gripActive 边沿映射为 start / latch。

协议（与 arm_follow_server_stm32.py / arm_follow_server_sdk.py 完全一致）：
  {"cmd":"start"}       开始跟随
  {"cmd":"latch"}       就地持位
  {"cmd":"q","q":[7]}   一帧 7 关节角目标（逐帧跟随）

真机服务对每条命令都会回一行 JSON 响应；本客户端在后台线程解析实测反馈并读空响应，
避免响应堆积在 TCP 缓冲里最终堵住真机服务的 sendall（那会导致真机卡死）。

用法（由 ws_echo_server.py --real 内部实例化，一般不单独运行）：
  from real_arm_client import RealArmClient
  client = RealArmClient()
  client.start("left"); client.send_q("left", q7); client.latch("left")
  client.close()
"""
import json
import math
import select
import socket
import threading
import time


def _parse_addr(s):
    """'host:port' 或 (host, port) → (host, int(port))。"""
    if isinstance(s, (tuple, list)):
        return (s[0], int(s[1]))
    host, _, port = str(s).partition(":")
    return (host, int(port))


class RealArmClient:
    """仿真 → 真机 TCP 桥接客户端（双臂 = 两条独立连接）。

    左/右臂各自连一台 arm_follow_server（可能在同一台工控机，也可能在
    两台工控机）。地址用 "host:port" 指定，分别对应工控机上的
    arm_follow_server 实例（串口设备名在该工控机侧用 --port 区分）。
    """

    def __init__(self, left_addr="127.0.0.1:9001",
                 right_addr="127.0.0.1:9002", send_hz=100.0):
        self._addrs = {"left": _parse_addr(left_addr), "right": _parse_addr(right_addr)}
        self._socks = {"left": None, "right": None}
        self._lock = threading.Lock()
        self._send_hz = max(1.0, send_hz)
        self._last_send = {"left": 0.0, "right": 0.0}
        self._last_q = {"left": None, "right": None}
        self._last_q_time = {"left": 0.0, "right": 0.0}
        self._last_connect = {"left": 0.0, "right": 0.0}
        self._states = {"left": None, "right": None}
        self._started = {"left": False, "right": False}
        self._holding = {"left": False, "right": False}
        self._start_pending = {"left": False, "right": False}
        self._command_error = {"left": None, "right": None}
        self._pending = {"left": {}, "right": {}}
        self._request_seq = 0
        self._last_request = {"left": 0.0, "right": 0.0}
        self._send_locks = {side: threading.Lock() for side in self._socks}
        self._stop = threading.Event()
        self._state_requests = {side: threading.Event() for side in self._socks}
        self._pollers = [threading.Thread(target=self._request_loop, args=(side,), daemon=True)
                         for side in self._socks]
        for worker in self._pollers:
            worker.start()
        self._drain = threading.Thread(target=self._drain_loop, daemon=True)
        self._drain.start()

    # ---- 连接管理 ----
    def _connect(self, side):
        with self._lock:
            s = self._socks[side]
        if s is not None:
            return s
        # 连接失败后 2s 内不重试，避免每帧 connect 阻塞仿真主循环
        now = time.time()
        if now - self._last_connect[side] < 2.0:
            return None
        self._last_connect[side] = now
        try:
            s = socket.create_connection(self._addrs[side], timeout=1.0)
            s.settimeout(0.02)  # bound command writes; connect runs only in background
        except OSError as e:
            print(f"[真机][{side}] 连接失败 {self._addrs[side]}: {e}")
            return None
        with self._lock:
            self._socks[side] = s
        print(f"[真机][{side}] 已连接 {self._addrs[side]}")
        return s

    def _drop(self, side, s):
        with self._lock:
            if self._socks[side] is s:
                self._socks[side] = None
                self._states[side] = None
                self._started[side] = False
                self._start_pending[side] = False
                self._last_q[side] = None
                self._pending[side].clear()
        try:
            s.close()
        except OSError:
            pass

    # ---- 发送 ----
    def _send(self, side, msg):
        with self._lock:
            s = self._socks[side]
        if s is None:
            self._state_requests[side].set()
            return False
        try:
            with self._send_locks[side]:
                s.sendall((json.dumps(msg) + "\n").encode())
            return True
        except OSError as e:
            print(f"[真机][{side}] 发送失败，断开: {e}")
            self._drop(side, s)
            return False

    def start(self, side):
        with self._lock:
            self._started[side] = False
            self._start_pending[side] = True
            self._holding[side] = False
            self._command_error[side] = None
            self._last_q[side] = None   # 上一次会话的目标不得作为保活补发出去
        return self._send(side, {"cmd": "start"})

    def is_started(self, side):
        with self._lock:
            return self._started[side] and self._command_error[side] is None

    def command_error(self, side):
        with self._lock:
            return self._command_error[side]

    def movej(self, side, q7):
        """movej 快速回到目标构型（阻塞在真机侧，本客户端 fire-and-forget）。"""
        self._send(side, {"cmd": "movej", "q": [round(float(x), 6) for x in q7]})

    def hold(self, side):
        """Discard pursuit of the old target without switching control mode."""
        with self._lock:
            self._started[side] = False
            self._start_pending[side] = False
            self._holding[side] = True
            self._last_q[side] = None
        self._send(side, {"cmd": "hold"})

    def latch(self, side):
        self._send(side, {"cmd": "latch"})

    def send_q(self, side, q7):
        now = time.time()
        if now - self._last_send[side] < 1.0 / self._send_hz:
            return  # 限流：关节角命令不超过 send_hz，避免串口拥塞
        self._last_send[side] = now
        q = [round(float(x), 6) for x in q7]
        with self._lock:
            # 有目标在流 = 不再持位。否则 20 Hz 的 hold 保活与 q 流交替下发：
            # 服务端每帧 q 会清 _holding，随后的 hold 看到 _holding=False 就拿
            # **实测角** 重置 slew 目标，把回零/任意非 start 触发的 q 流一路拉回
            # 实测位（A 键时真机不动的根因）。只有显式 hold()/start() 才改这个状态。
            self._holding[side] = False
            self._last_q[side] = q
            self._last_q_time[side] = now
        self._send(side, {"cmd": "q", "q": q})

    def request_state(self, side):
        """Schedule a measured-state query; never connects on the control thread."""
        self._state_requests[side].set()

    def _request_loop(self, side):
        event = self._state_requests[side]
        while not self._stop.is_set():
            # 超时也继续：除了按控制环请求查实测状态，还要在本线程里给固件
            # 100 ms 的 joint_follow 看门狗保活（见下方 last_q 补发）。
            # 控制环偶发卡顿（GUI 绘制等）时，这条路径不能跟着一起停。
            event.wait(0.05)
            event.clear()
            if self._stop.is_set():
                return
            delay = 0.05 - (time.monotonic() - self._last_request[side])
            if delay > 0 and self._stop.wait(delay):
                return
            # One independent worker per arm: an offline arm cannot block the
            # other arm's feedback, or the simulation/control loop.
            sock = self._connect(side)
            if sock is None or self._stop.is_set():
                if sock is not None:
                    self._drop(side, sock)
                continue
            now = time.monotonic()
            with self._lock:
                self._last_request[side] = now
                self._request_seq += 1
                request_id = self._request_seq
                pending = self._pending[side]
                for key, sent in list(pending.items()):
                    if now - sent > 1.0:
                        del pending[key]
                pending[request_id] = now
            self._send(side, {"cmd": "state", "request_id": request_id})
            with self._lock:
                holding = self._holding[side]
                started = self._started[side] and self._command_error[side] is None
                last_q = self._last_q[side]
            if holding:
                self._send(side, {"cmd": "hold"})  # idempotent hold stream keepalive
            elif (started and last_q is not None
                  and time.time() - self._last_q_time[side] > 0.04):
                # 固件 joint_follow 的命令看门狗 100 ms，跳闸即切 fail-soft
                # 持位（kp=mit_kp×0.6、τ=0），与跟随帧（kp=50、重力前馈）
                # 来回切换会让臂剧烈抖动。控制环若偶发卡顿 >40 ms，就由本
                # 线程用最后一帧目标补发保活 —— 目标不变，伺服原地保持，
                # 不产生任何额外运动。
                self._send(side, {"cmd": "q", "q": last_q})
                self._last_q_time[side] = time.time()

    def get_state(self, side, max_age=0.25):
        """Return fresh measured radians, or None. Never substitutes command q."""
        with self._lock:
            state = self._states[side]
            if self._socks[side] is None or state is None:
                return None
            age = state["age_s"] + time.monotonic() - state["received_at"]
            if age > max_age:
                return None
            return {**state, "q": list(state["q"]),
                    "cmd_q": (None if state.get("cmd_q") is None
                              else list(state["cmd_q"])),
                    "age_s": age}

    def _handle_response(self, side, sock, response):
        if not isinstance(response, dict):
            return
        if response.get("cmd") != "state":
            with self._lock:
                if self._socks[side] is not sock:
                    return
                if response.get("ok") is False:
                    self._command_error[side] = str(response.get("error", "command rejected"))
                    self._started[side] = False
                elif response.get("cmd") == "start" and self._start_pending[side]:
                    self._started[side] = True
                    self._start_pending[side] = False
                    self._states[side] = None  # require a post-start measurement
                    self._pending[side].clear()
            return
        now = time.monotonic()
        with self._lock:
            if self._socks[side] is not sock:
                return
            request_id = response.get("request_id")
            if not isinstance(request_id, int):
                return
            sent = self._pending[side].pop(request_id, None)
            if sent is None:
                return
            try:
                q = [float(x) for x in response["q"]]
                raw_cmd_q = response.get("cmd_q")
                cmd_q = (None if raw_cmd_q is None
                         else [float(x) for x in raw_cmd_q])
                age = float(response["age_s"])
                valid = (response.get("valid") is True and len(q) == 7
                         and all(math.isfinite(x) for x in q)
                         and (cmd_q is None or (len(cmd_q) == 7
                              and all(math.isfinite(x) for x in cmd_q)))
                         and math.isfinite(age) and age >= 0)
            except (KeyError, TypeError, ValueError):
                valid = False
            self._states[side] = ({"q": q, "cmd_q": cmd_q,
                                   "age_s": age + now - sent,
                                   "received_at": now, "request_id": request_id}
                                  if valid else None)

    # ---- Parse asynchronous JSON responses without blocking the other arm ----
    def _drain_loop(self):
        buffers = {}
        while not self._stop.is_set():
            with self._lock:
                sockets = {sock: side for side, sock in self._socks.items()
                           if sock is not None}
            buffers = {sock: buf for sock, buf in buffers.items() if sock in sockets}
            if not sockets:
                self._stop.wait(0.02)
                continue
            try:
                ready, _, _ = select.select(list(sockets), [], [], 0.05)
            except (OSError, ValueError):
                continue
            for sock in ready:
                side = sockets[sock]
                try:
                    data = sock.recv(4096)
                    if not data:
                        self._drop(side, sock)
                        continue
                    buf = buffers.get(sock, b"") + data
                    if len(buf) > 65536:
                        self._drop(side, sock)
                        continue
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            response = json.loads(line)
                        except (ValueError, UnicodeDecodeError):
                            continue
                        self._handle_response(side, sock, response)
                    buffers[sock] = buf
                except socket.timeout:
                    pass
                except OSError:
                    self._drop(side, sock)

    def close(self):
        self._stop.set()
        for event in self._state_requests.values():
            event.set()
        for side in ("left", "right"):
            with self._lock:
                s = self._socks[side]
            if s is not None:
                self._drop(side, s)

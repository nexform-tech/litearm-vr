#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""灵巧手(L6) 常驻 CAN 服务（TCP + JSON），跑在灵巧手所在的工控机上。

与 `arm_follow_server_*.py` / `gripper_server.py` 同类：都在「链路最靠近硬件的一环」
——收指令、驱动电机。臂收 7 关节角流，夹爪收 close/open 边沿，灵巧手收 close/open
加左/右手。

hand_grasp.left_close/right_close 等 SDK 调用是阻塞的（内部 time.sleep 约 1.5s，
握拳还有 0.5s 拇指延迟），必须放到后台线程串行执行，否则会卡死 TCP 收发线程。
左右手共用一个队列串行执行（同一 CAN 总线，本就无法并行）。
这样「按下握拳、松开张开」由远端 VR 端按 trigger 边沿发 close/open 实现。

协议（TCP，每行一条 JSON 文本）：
  {"cmd":"close","side":"left"}   左手握拳
  {"cmd":"open","side":"right"}   右手张开
  {"cmd":"ping"}                  探活，返回 {"ok":true}

依赖：hand_grasp（linkerbot-python-sdk-main）+ linkerbot SDK（src/），需在 PYTHONPATH
中（由 run_hand_server.sh 注入，可用 LINKERBOT_SDK 覆盖 SDK 目录）。

用法（在灵巧手所在的工控机上）：
  python3 hand_server.py --can can0 --host 0.0.0.0 --port 9004

CAN 接口需先 up（1Mbps classic），例如:
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set can0 up
"""
import argparse
import json
import queue
import socket
import threading


class HandCanControl:
    """灵巧手 L6：close/open + side 入队，后台 worker 串行调用 hand_grasp（阻塞）。"""

    def __init__(self, can="can0"):
        import hand_grasp  # 延迟 import：无 SDK 时报错清晰
        self._can = can
        self._actions = {
            "left": {"close": hand_grasp.left_close, "open": hand_grasp.left_open},
            "right": {"close": hand_grasp.right_close, "open": hand_grasp.right_open},
        }
        self._q = queue.Queue()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def _worker_loop(self):
        while True:
            side, action = self._q.get()
            name = "握拳" if action == "close" else "张开"
            try:
                self._actions[side][action](self._can)
                print(f"[灵巧手] {side} 手{name}完成", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"[灵巧手] {side} 手{name}失败: {e}", flush=True)

    def close(self, side):
        """握拳（入队，非阻塞）。"""
        self._q.put((side, "close"))

    def open(self, side):
        """张开（入队，非阻塞）。"""
        self._q.put((side, "open"))

    def shutdown(self):
        # worker 是 daemon 线程，随进程退出；队列里未执行的动作直接丢弃
        pass


class HandServer:
    """TCP + JSON 服务，把 close/open + side 转成 HandCanControl 的入队动作。"""

    def __init__(self, can):
        self._h = HandCanControl(can=can)

    def handle(self, msg):
        cmd = msg.get("cmd")
        if cmd in ("close", "open"):
            side = msg.get("side")
            if side not in ("left", "right"):
                return {"ok": False, "error": f"bad side {side!r}"}
            if cmd == "close":
                self._h.close(side)
            else:
                self._h.open(side)
            return {"ok": True, "cmd": cmd, "side": side}
        if cmd == "ping":
            return {"ok": True, "cmd": cmd}
        return {"ok": False, "error": f"unknown cmd {cmd!r}"}

    def serve(self, host, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((host, port))
            srv.listen(5)
            print(f"[灵巧手服务] 监听 {host}:{port}", flush=True)
            while True:
                conn, addr = srv.accept()
                print(f"[灵巧手服务] 客户端接入: {addr}", flush=True)
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
        print(f"[灵巧手服务] 客户端断开: {addr}", flush=True)

    def shutdown(self):
        self._h.shutdown()


def main():
    p = argparse.ArgumentParser(description="灵巧手(L6) 常驻 CAN 服务（TCP + JSON）")
    p.add_argument("--can", default="can0", help="CAN 接口（默认 can0）")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=9004)
    args = p.parse_args()

    server = HandServer(args.can)
    try:
        server.serve(args.host, args.port)
    except KeyboardInterrupt:
        print("\n[灵巧手服务] 收到中断，退出", flush=True)
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""follow 整机状态汇总 → Zenoh **按设备分 topic** 发布。

补齐 follow 的**接收半边**：各 follow 服务（`arm_follow_server_*.py` /
`gripper_server.py` / `hand_server.py`）只做「收命令 → 驱硬件」，本脚本经 TCP
**只读地**轮询它们的状态（`ping` / `status`，**永不**发 `start`/`q`/`movej`/`home`/
`close`/`open`，也**永不开**串口 / CAN），把**每个设备**的状态本体（见 `status_msgs.py`
的 `ArmStatus`/`GripperStatus`/`HandStatus`）pub 到各自的 Zenoh topic：

| kind | topic |
| --- | --- |
| arm | `{prefix}/{arm_id}/vr_arm` |
| gripper | `{prefix}/{arm_id}/vr_grip` |
| hand | `{prefix}/{arm_id}/vr_hand` |

`arm_id` 是**臂身份**（`armA`/`armB`），由**桥的臂清单**给 —— 话题契约与 bridge 的
`litearm/v4/{arm_id}/{channel}` 对齐。**follow 服务不认识臂身份**：它只知道自己开的
串口（回包 `port`）/ CAN 口（回包 `can`），所以身份不能从回包推，必须外部配进来。
`--arm armA=/dev/serial/by-id/…-if00@127.0.0.1:9001` 把「arm_id ← 串口 ← follow 端点」
绑在一起：发布时中段用 `arm_id`，`device` 字段仍是回包里的物理接口（与配置对不上会告警）。

（`--prefix` 默认 `litearm/v4`。早先合并成的单条 `FollowStatus` 已不再发布。）

```
臂 9001  夹爪 9003  灵巧手 9004   ← 各 follow 服务
  ▲
  │  本脚本 100Hz/20Hz 轮询 {"cmd":"ping"|"status"}
  │
status_aggregator.py ──put(JSON)──▶ litearm/v4/{armA,armB}/vr_* （Zenoh, peer）
```

线程模型（每条 `TcpPoller` = 一个设备 + 一条持久 TCP 连接，照 teleop 的
`probe.py:FollowSampler`）：
  - **`StatusStore`**：`threading.Lock` 保护 `{key: (msg, last_ok_mono)}`。
  - **`TcpPoller`**：按各自 hz 发 `{"cmd":"ping"}`（夹爪发 `{"cmd":"status"}`）→ 读一行
    JSON → 归一化 → 写 store。任何 `OSError`/`EOF`/`JSONDecodeError` ⇒ `store.fail()`
    （`connected=false`）+ 关 socket + 退避 1.0s 重连。**一设备故障绝不阻塞其它设备或发布。**
  - **`Publisher`**：**唯一碰 Zenoh 的线程**。按 `--publish-hz` 取快照 → 重算 `age_ms`、
    `age > --stale-ms` 置 `connected=false` → **每设备各发一条**其本体 JSON。`put()` 无
    订阅者是静默 no-op（不阻塞、不堆积）。

**关键约束**：臂的 `/dev/ttyACM0` 串口独占，只有臂服务进程能读臂状态；夹爪 `can0` 被
`gripper_server.py` 持有。所以状态**只能经各服务的 TCP 取**，本脚本不直连任何硬件。

Zenoh 约定照搬 `litegrip-python` 的 `_base_config()`：`mode=peer` + 多播 scouting / gossip
全关 + 显式 `listen`/`connect`。**session 必须 `close()`，否则进程不退出**。

用法：
  python3 status_aggregator.py \\
      --arm armA=/dev/serial/by-id/usb-…-if00@127.0.0.1:9001 \\
      --arm armB=/dev/serial/by-id/usb-…-if02@127.0.0.1:9002 \\
      --gripper armA=can0@127.0.0.1:9003 --gripper armB=can1@127.0.0.1:9004
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from status_msgs import (ArmStatus, GripperStatus, HandStatus, parse_arm_reply,
                         parse_gripper_reply, parse_hand_reply)

DEFAULT_ARM_HZ = 100.0
DEFAULT_GRIPPER_HZ = 20.0
DEFAULT_PUBLISH_HZ = 100.0
DEFAULT_STALE_MS = 1000.0
RECONNECT_S = 1.0


# ---------------------------------------------------------------- 设备描述
# topic 叶子：每种设备一个独立的 vr_* 后缀（不占用 bridge 的 vr_frame/status/rpc）。
_CHANNEL = {"arm": "vr_arm", "gripper": "vr_grip", "hand": "vr_hand"}
# 回包里承载「物理身份」的字段：臂=串口/CAN 接口，夹爪=CAN 接口。
_DEVICE_FIELD = {"arm": "port", "gripper": "can"}


@dataclass
class Device:
    """一个待轮询的 follow 服务端点 + 它的**臂身份**。

    `arm_id` 由**桥的臂清单**给（`armA`/`armB`）；follow 服务只知道自己开的串口，
    不知道左/右，所以身份不能从回包推，只能外部配进来。`serial` 是配置的物理接口
    （串口 / CAN 口），仅用于和回包里的身份对拍（对不上告警一次）。
    """
    arm_id: str       # 臂身份（topic 中段）："armA" | "armB"
    kind: str         # "arm" | "gripper" | "hand"
    host: str
    port: int
    hz: float
    serial: str = ""  # 配置的物理接口（串口 / CAN 口）；空 = 不校验
    _mismatch_warned: bool = field(default=False, init=False, repr=False)

    @property
    def key(self) -> str:
        """store / publisher 字典里的唯一键（两条臂 kind 相同、arm_id 不同）。"""
        return f"{self.arm_id}:{self.kind}"

    @property
    def interval(self) -> float:
        return 1.0 / max(1.0, self.hz)

    def topic(self, prefix: str) -> str:
        """本设备的状态 topic：`{prefix}/{arm_id}/{vr_*}`。"""
        return f"{prefix}/{self.arm_id}/{_CHANNEL[self.kind]}"

    @property
    def request(self) -> bytes:
        # 夹爪用专用 `status`（零 I/O 走缓存），臂/手用 `ping`。
        cmd = "status" if self.kind == "gripper" else "ping"
        return (json.dumps({"cmd": cmd}) + "\n").encode()

    @property
    def placeholder(self):
        dev = self.serial or self.arm_id
        if self.kind == "arm":
            return ArmStatus(device=dev, connected=False, ok=False)
        if self.kind == "gripper":
            return GripperStatus(device=dev, connected=False, ok=False)
        return HandStatus(device=dev, connected=False, ok=False)

    def parse(self, reply, prev):
        # device 取回包里的物理身份（臂=串口(port)/sdk=iface，夹爪=CAN 接口(can)）；
        # 回包里没有才退回配置的串口/CAN 口。与配置不符 ⇒ 告警一次（不阻断）。
        device = self.serial or self.arm_id
        if isinstance(reply, dict):
            field = _DEVICE_FIELD.get(self.kind)
            got = reply.get(field) if field else None
            if got:
                device = str(got)
                if (self.serial and device != self.serial
                        and not self._mismatch_warned):
                    self._mismatch_warned = True
                    print(f"[汇总] {self.key} 接口不符: 配置 {self.serial} "
                          f"≠ 回包 {device}", flush=True)
        if self.kind == "arm":
            return parse_arm_reply(device, reply, prev)
        if self.kind == "gripper":
            return parse_gripper_reply(device, reply, prev)
        return parse_hand_reply(device, reply, prev)


# ---------------------------------------------------------------- 状态仓库
class StatusStore:
    """线程安全的状态字典：`{key: (msg | None, last_ok_mono | None, error)}`。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._msg: Dict[str, object] = {}
        self._last_ok: Dict[str, Optional[float]] = {}
        self._error: Dict[str, Optional[str]] = {}

    def update(self, key, msg):
        with self._lock:
            self._msg[key] = msg
            self._last_ok[key] = time.monotonic()
            self._error[key] = getattr(msg, "error", None)

    def fail(self, key, error, placeholder):
        """链路失败：保留上次实测值，只把 `connected` 置假 + 记错误。"""
        with self._lock:
            old = self._msg.get(key)
            if old is None:
                old = placeholder
            self._msg[key] = dataclasses.replace(old, connected=False, error=error)
            self._error[key] = error

    def get(self, key):
        with self._lock:
            return self._msg.get(key)

    def snapshot(self) -> Dict[str, Tuple[object, Optional[float]]]:
        with self._lock:
            return {k: (self._msg[k], self._last_ok.get(k))
                    for k in self._msg}


# ---------------------------------------------------------------- TCP 轮询
class TcpPoller(threading.Thread):
    """单设备持久 TCP 轮询；断线退避重连，失败即 `store.fail()`。"""

    def __init__(self, dev: Device, store: StatusStore, log):
        super().__init__(name=f"poll:{dev.key}", daemon=True)
        self.dev = dev
        self.store = store
        self._log = log
        self._stop_evt = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._buf = b""
        self._connect_timeout = 2.0
        self._request_timeout = 1.0

    def stop(self):
        self._stop_evt.set()

    # ---- 内部 ----
    def _connect(self):
        self._close()
        s = socket.create_connection((self.dev.host, self.dev.port),
                                     timeout=self._connect_timeout)
        s.settimeout(self._request_timeout)
        self._sock = s
        self._buf = b""
        if self._log:
            print(f"[汇总] 已连接 {self.dev.key} "
                  f"{self.dev.host}:{self.dev.port}", flush=True)

    def _close(self):
        s, self._sock = self._sock, None
        if s is not None:
            try:
                s.close()
            except OSError:
                pass

    def _read_line(self) -> bytes:
        while b"\n" not in self._buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("eof")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line

    def run(self):
        while not self._stop_evt.is_set():
            try:
                self._connect()
            except OSError as e:
                self.store.fail(self.dev.key, f"connect: {e}", self.dev.placeholder)
                if self._log:
                    print(f"[汇总] {self.dev.key} 连接失败: {e}", flush=True)
                self._stop_evt.wait(RECONNECT_S)
                continue
            try:
                while not self._stop_evt.is_set():
                    t0 = time.monotonic()
                    self._sock.sendall(self.dev.request)
                    line = self._read_line()
                    reply = json.loads(line.decode("utf-8", "replace"))
                    msg = self.dev.parse(reply, self.store.get(self.dev.key))
                    if msg is not None:
                        self.store.update(self.dev.key, msg)
                    self._stop_evt.wait(max(0.0, self.dev.interval -
                                         (time.monotonic() - t0)))
            except (OSError, ValueError, ConnectionError,
                    json.JSONDecodeError, UnicodeDecodeError) as e:
                self.store.fail(self.dev.key, repr(e), self.dev.placeholder)
                if self._log:
                    print(f"[汇总] {self.dev.key} 链路中断: {e!r}",
                          flush=True)
                self._close()
                self._stop_evt.wait(RECONNECT_S)
        self._close()


# ---------------------------------------------------------------- Zenoh 发布
class Publisher(threading.Thread):
    """唯一碰 Zenoh 的线程：周期把 store 快照**按设备** pub 出去。"""

    def __init__(self, devices: List[Device], store: StatusStore, prefix: str,
                 hz: float, stale_ms: float, cfg, log):
        super().__init__(name="zenoh-publish", daemon=True)
        self.devices = devices
        self.store = store
        self.prefix = prefix
        self.hz = hz
        self.stale_ms = stale_ms
        self._cfg = cfg
        self._log = log
        self._stop_evt = threading.Event()

    def stop(self):
        self._stop_evt.set()

    def _build_device(self, dev, now, snap):
        """取该设备快照 → 重算 age_ms + 陈旧门，返回其状态本体。"""
        cur = snap.get(dev.key)
        msg, last = cur if cur is not None else (None, None)
        if msg is None:
            msg = dev.placeholder
            age_ms = self.stale_ms + 1.0      # 从未成功 → 视为陈旧
        elif last is None:
            age_ms = self.stale_ms + 1.0
        else:
            age_ms = (now - last) * 1000.0
        msg = dataclasses.replace(msg, age_ms=round(max(0.0, age_ms), 1))
        if age_ms > self.stale_ms:
            msg = dataclasses.replace(msg, connected=False)
        return msg

    def run(self):
        try:
            import zenoh
        except ImportError as e:
            print(f"[汇总] 无法 import zenoh（{e}）；仅轮询不发布", flush=True)
            return

        enc = getattr(getattr(zenoh, "Encoding", None),
                      "APPLICATION_JSON", None)
        session = None
        pubs: Dict[str, object] = {}
        try:
            session = zenoh.open(self._cfg)
            pubs = {dev.key: session.declare_publisher(dev.topic(self.prefix))
                    for dev in self.devices}
            if self._log:
                for dev in self.devices:
                    print(f"[汇总] Zenoh 发布 {dev.topic(self.prefix)} "
                          f"@ {self.hz:g}Hz", flush=True)
            while not self._stop_evt.is_set():
                t0 = time.monotonic()
                now = time.monotonic()
                snap = self.store.snapshot()
                for dev in self.devices:
                    msg = self._build_device(dev, now, snap)
                    payload = msg.to_json().encode("utf-8")
                    try:
                        if enc is not None:
                            pubs[dev.key].put(payload, encoding=enc)
                        else:
                            pubs[dev.key].put(payload)
                    except (OSError, RuntimeError) as e:
                        if self._log:
                            print(f"[汇总] {dev.key} put 失败: {e!r}",
                                  flush=True)
                self._stop_evt.wait(max(0.0, 1.0 / max(1.0, self.hz) -
                                    (time.monotonic() - t0)))
        except Exception as e:  # noqa: BLE001
            print(f"[汇总] Zenoh 发布线程异常: {e!r}", flush=True)
        finally:
            for pub in pubs.values():
                try:
                    pub.undeclare()
                except Exception:  # noqa: BLE001
                    pass
            if session is not None:
                try:
                    session.close()
                except Exception:  # noqa: BLE001
                    pass


# ---------------------------------------------------------------- Zenoh 配置
def build_config(args):
    """照搬 `litegrip` 的 `_base_config()`：peer + 关发现 + 显式端点。"""
    import zenoh

    if args.zenoh_config:
        return zenoh.Config.from_file(args.zenoh_config)

    c = zenoh.Config()
    c.insert_json5("mode", json.dumps(args.zenoh_mode))
    if not args.zenoh_scouting:
        c.insert_json5("scouting/multicast/enabled", "false")
        c.insert_json5("scouting/gossip/enabled", "false")
    listens = args.zenoh_listen if args.zenoh_listen else [
        f"tcp/0.0.0.0:{args.zenoh_listen_port}"]
    c.insert_json5("listen/endpoints", json.dumps(listens))
    c.insert_json5("connect/endpoints", json.dumps(args.zenoh_connect))
    return c


# ---------------------------------------------------------------- CLI
def _parse_device(spec):
    """`ARM_ID=[接口@]HOST:PORT` → `(arm_id, serial, host, port)`。

    `接口` = 串口（`/dev/serial/by-id/…`）/ CAN 口（`can0`），仅用于和回包对拍；
    省略（无 `@`）则不校验。
    """
    if "=" not in spec:
        raise argparse.ArgumentTypeError(
            f"设备需 ARM_ID=[接口@]HOST:PORT，收到 {spec!r}")
    arm_id, rest = spec.split("=", 1)
    serial = ""
    if "@" in rest:
        serial, hostport = rest.split("@", 1)
        serial = serial.strip()
    else:
        hostport = rest
    try:
        host, port = hostport.rsplit(":", 1)
        return arm_id.strip(), serial, host.strip(), int(port)
    except (ValueError, TypeError):
        raise argparse.ArgumentTypeError(f"端点应为 HOST:PORT，收到 {hostport!r}")


def build_devices(args) -> List[Device]:
    devices: List[Device] = []
    for spec in args.arm:
        arm_id, serial, host, port = _parse_device(spec)
        devices.append(Device(arm_id, "arm", host, port, args.arm_hz, serial))
    if args.gripper and "none" not in args.gripper:
        for spec in args.gripper:
            arm_id, serial, host, port = _parse_device(spec)
            devices.append(Device(arm_id, "gripper", host, port,
                                  args.gripper_hz, serial))
    for spec in args.hand:
        arm_id, serial, host, port = _parse_device(spec)
        devices.append(Device(arm_id, "hand", host, port, args.arm_hz, serial))
    return devices


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="follow 整机状态汇总 → Zenoh 按设备分 topic 发布")
    p.add_argument("--arm", action="append", default=None,
                   metavar="ARM_ID=[IFACE@]HOST:PORT",
                   help="臂端点（可重复）。默认 armA=127.0.0.1:9001；"
                        "ARM_ID 是臂身份（armA/armB），IFACE 是串口（对拍用，可省）")
    p.add_argument("--gripper", action="append", default=None,
                   metavar="ARM_ID=[IFACE@]HOST:PORT",
                   help="夹爪端点（可重复；'none' 关闭）。默认 armA=127.0.0.1:9003；"
                        "IFACE 是 CAN 口（对拍用，可省）")
    p.add_argument("--hand", action="append", default=[],
                   metavar="ARM_ID=[IFACE@]HOST:PORT",
                   help="灵巧手端点（可重复；v1 无状态读取，仅占位）")
    p.add_argument("--prefix", default="litearm/v4",
                   help="topic 前缀，各设备话题为 {prefix}/{ARM_ID}/vr_*"
                        "（默认 litearm/v4）")
    p.add_argument("--arm-hz", type=float, default=DEFAULT_ARM_HZ)
    p.add_argument("--gripper-hz", type=float, default=DEFAULT_GRIPPER_HZ)
    p.add_argument("--publish-hz", type=float, default=DEFAULT_PUBLISH_HZ)
    p.add_argument("--stale-ms", type=float, default=DEFAULT_STALE_MS,
                   help="超此龄发布态置 connected=false（默认 1000）")
    p.add_argument("--connect-timeout-s", type=float, default=2.0)
    p.add_argument("--request-timeout-s", type=float, default=1.0)
    p.add_argument("--zenoh-mode", default="peer", choices=("peer", "client"))
    p.add_argument("--zenoh-listen", action="append", default=None,
                   help="listen 端点（可重复；默认 tcp/0.0.0.0:17449）")
    p.add_argument("--zenoh-listen-port", type=int, default=17449)
    p.add_argument("--zenoh-connect", action="append", default=[],
                   help="connect 端点（可重复）")
    p.add_argument("--zenoh-scouting", action="store_true",
                   help="打开多播发现（本机临时联调）")
    p.add_argument("--zenoh-config", default=None,
                   help="zenoh 配置文件路径（整体覆盖上面的 zenoh 选项）")
    p.add_argument("--quiet", action="store_true")
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args(argv)

    if args.arm is None:
        args.arm = ["armA=127.0.0.1:9001"]
    if args.gripper is None:
        args.gripper = ["armA=127.0.0.1:9003"]
    return args


def main(argv=None):
    args = parse_args(argv)
    log = not args.quiet

    devices = build_devices(args)
    if not devices:
        print("[汇总] 无任何设备端点，退出", file=sys.stderr, flush=True)
        return 2

    store = StatusStore()
    stop = threading.Event()

    def _sig(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    pollers = []
    for dev in devices:
        pl = TcpPoller(dev, store, log)
        pl._connect_timeout = args.connect_timeout_s
        pl._request_timeout = args.request_timeout_s
        pollers.append(pl)

    pub = None
    try:
        for pl in pollers:
            pl.start()
        try:
            cfg = build_config(args)
        except Exception as e:  # noqa: BLE001
            print(f"[汇总] Zenoh 配置失败: {e!r}", flush=True)
            cfg = None
        if cfg is not None:
            pub = Publisher(devices, store, args.prefix, args.publish_hz,
                            args.stale_ms, cfg, log)
            pub.start()
        if log:
            print(f"[汇总] 设备: {[d.key for d in devices]}", flush=True)
        while not stop.wait(0.2):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[汇总] 退出中…", flush=True)
        for pl in pollers:
            pl.stop()
        if pub is not None:
            pub.stop()
        for pl in pollers:
            pl.join(timeout=2.0)
        if pub is not None:
            pub.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())

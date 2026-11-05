#!/usr/bin/env python3
"""VR 数据源（`src/litearm_teleop/vr_source.py`）的单测，**只依赖标准库**，秒级。

守四件事：
  1. 话题拼法与桥（`litearm-vr-bridge/src/litearm_vr_bridge/protocol.py`）**逐字一致**。
     两边漂了不会报错，只会表现为「桥在发、这儿一条也订不到」——最难查的一种。
  2. 端点的四种写法（None / 单串 / 逗号分隔 / flow 序列）都要解析对。
  3. 同状态只保留首帧和最新位姿，边沿按序；溢出发送失效屏障，坏载荷计数。
  4. 有 eclipse-zenoh 时再跑一遍**真链路**（真开会话、真 pub/sub）。没有就跳过：
     metavr-arm 环境出厂不装它也能跑 `--source ws`，不该因为这个测试挂掉。

用法: python tests/test_vr_source.py
"""
import json
import os
import signal
import socket
import subprocess
import sys
import time

from litearm_teleop.vr_source import (DEFAULT_ARM_ID, DEFAULT_SOURCE,
                                      DEFAULT_ZENOH_CONNECT, DEFAULT_ZENOH_MODE,
                                      PREFIX, ZenohFrameSource, frame_topic,
                                      parse_endpoints)

# 与桥的 protocol.frame_topic 逐字一致的期望值：改这里就必须同步改桥，反之亦然。
BRIDGE_TOPIC_ARM_A = "litearm/v4/armA/vr_frame"

# 头显帧的形状（与桥透传的、`ws_echo_server` 解析的同一份格式）。
FRAME = {
    "timestamp": 1792053507964,
    "leftController": {"tracked": True, "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                       "rotation": {"x": 0.0, "y": 0.0, "z": 0.0},
                       "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                       "gripActive": False, "trigger": 0.25},
    "rightController": {"tracked": True, "position": {"x": -0.1, "y": 0.2, "z": 0.3},
                        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                        "gripActive": True, "trigger": 0.25},
}


class _Sample:
    """冒充 zenoh 的 Sample：本模块只用到 `.payload`（且要能吃 `bytes()`）。"""

    def __init__(self, payload):
        self.payload = payload


def test_topic():
    assert PREFIX == "litearm/v4"
    assert DEFAULT_ARM_ID == "armA"
    assert frame_topic() == BRIDGE_TOPIC_ARM_A
    assert frame_topic(DEFAULT_ARM_ID) == BRIDGE_TOPIC_ARM_A
    assert frame_topic("armB") == "litearm/v4/armB/vr_frame"
    print("[1] 话题：frame_topic() == %s（与桥的 protocol.frame_topic 同形）"
          % BRIDGE_TOPIC_ARM_A)


def test_source_defaults():
    """出厂数据源是 zenoh，且会话默认值 = 桥 DEPLOY.md 里「消费者」那条配方。"""
    assert DEFAULT_SOURCE == "zenoh"
    assert DEFAULT_ZENOH_MODE == "peer"
    assert parse_endpoints(DEFAULT_ZENOH_CONNECT) == ("tcp/127.0.0.1:7447",)
    print("[2] 出厂：source=%s，mode=%s，connect=%s（peer 容忍桥后起）"
          % (DEFAULT_SOURCE, DEFAULT_ZENOH_MODE, ",".join(DEFAULT_ZENOH_CONNECT)))


def test_parse_endpoints():
    assert parse_endpoints(None) == ()
    assert parse_endpoints("") == ()
    assert parse_endpoints([]) == ()
    assert parse_endpoints("tcp/127.0.0.1:7447") == ("tcp/127.0.0.1:7447",)
    assert parse_endpoints("tcp/a:7447, tcp/b:7447") == ("tcp/a:7447", "tcp/b:7447")
    assert parse_endpoints("tcp/a:7447,,tcp/b:7447,") == ("tcp/a:7447", "tcp/b:7447")
    assert parse_endpoints(["tcp/a:7447"]) == ("tcp/a:7447",)
    assert parse_endpoints(DEFAULT_ZENOH_CONNECT) == ("tcp/127.0.0.1:7447",)
    print("[3] 端点：None / 空 / 单串 / 逗号分隔 / flow 序列都解析对")


def test_queue_discipline():
    source = ZenohFrameSource("t", queue_size=8)

    def push(grip, x, trigger=0.25):
        frame = json.loads(json.dumps(FRAME))
        frame["rightController"].update(gripActive=grip, trigger=trigger)
        frame["rightController"]["position"]["x"] = x
        source._on_sample(_Sample(json.dumps(frame).encode()))

    # 首个按下帧保留为锚点，连续位姿仅保留最新；release/repress 不合并。
    for x in range(100):
        push(True, x)
    push(False, 100)
    push(True, 101)
    push(True, 102)
    push(True, 103)
    frames = []
    while True:
        raw = source.recv(timeout=0.001)
        if raw is None:
            break
        frames.append(json.loads(raw)["rightController"])
    assert [f["position"]["x"] for f in frames] == [0, 99, 100, 101, 103]
    assert [f["gripActive"] for f in frames] == [True, True, False, True, True]
    assert source.frames_received == 104
    assert source.frames_dropped == 99

    # 控制边沿撑满时不能留下旧按下帧然后丢弃 release。
    source = ZenohFrameSource("t", queue_size=2)
    push(True, 0)
    push(False, 1)
    push(True, 2)
    assert json.loads(source.recv(timeout=0.01)) == {"teleopInputFault": "queue_overflow"}
    assert json.loads(source.recv(timeout=0.01))["rightController"]["position"]["x"] == 2
    assert source.frames_dropped == 2

    # trigger 数值变化也必须保留，不猜测上层可配置阈值。
    source = ZenohFrameSource("t", queue_size=8)
    push(True, 0, 0.1)
    push(True, 1, 0.9)
    push(True, 2, 0.1)
    assert [json.loads(source.recv(timeout=0.01))["rightController"]["trigger"]
            for _ in range(3)] == [0.1, 0.9, 0.1]
    for payload in (b"\xff\xfe", b"", b"   "):
        before = source.frames_dropped
        assert source._on_sample(_Sample(payload)) is False
        assert source.frames_dropped == before + 1
    print("[4] 队列：最新位姿、首帧锚点、控制边沿与溢出失效屏障通过")


def _free_port():
    """问内核要一个空闲端口（免得测试与同机的桥/别的进程抢固定端口）。"""
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def test_real_zenoh_roundtrip_bridge_starts_late():
    """真链路，且**按出厂顺序**：teleop 先起、桥后起，桥一上线就收到帧。

    出厂配置是 peer + 显式连端点（不是 client）：client 模式要求开会话时就已连上，
    桥没起就抛错退出 —— 而「先起 teleop、后起桥」是完全正常的顺序。
    """
    try:
        import zenoh
    except ImportError:
        print("[5] 跳过真链路：本解释器没有 eclipse-zenoh"
              "（装它：%s -m pip install eclipse-zenoh）" % sys.executable)
        return

    endpoint = "tcp/127.0.0.1:%d" % _free_port()
    # ① teleop 侧：端点此刻**没人监听**（桥还没起），start() 必须不抛
    source = ZenohFrameSource(frame_topic(DEFAULT_ARM_ID), connect=endpoint)
    source.start()
    try:
        assert frame_topic() in source.describe() and "mode=peer" in source.describe()
        assert source.recv(timeout=0.05) is None

        # ② 桥侧：peer + listen 在那个端点上（= 桥的形态），发帧
        config = zenoh.Config()
        config.insert_json5("mode", json.dumps("peer"))
        config.insert_json5("listen/endpoints", json.dumps([endpoint]))
        session = zenoh.open(config)
        publisher = session.declare_publisher(frame_topic())
        try:
            raw = json.dumps(FRAME)
            # 让 teleop 的重连追上（peer 后台重试），拿不到就重发
            deadline = time.time() + 15.0
            got = None
            while got is None and time.time() < deadline:
                publisher.put(raw.encode("utf-8"))
                got = source.recv(timeout=0.3)
            assert got == raw, "收到的帧与发出的不一致: %r" % got
            assert json.loads(got)["rightController"]["gripActive"] is True
            assert source.frames_received >= 1 and source.frames_dropped == 0
        finally:
            publisher.undeclare()
            session.close()
    finally:
        source.stop()
    assert source.recv(timeout=0.01) is None, "stop() 后不该再有帧"
    print("[5] 真 zenoh：teleop 先起（端点无人听）→ 桥后起 → 收到整帧 JSON"
          "（收 %d 丢 %d）" % (source.frames_received, source.frames_dropped))


def test_ctrl_c_not_swallowed_by_zenoh():
    """开完 zenoh 会话后，SIGINT（Ctrl-C）必须还能打断进程。

    **这是实测踩到的坑**：zenoh 的 Rust 运行时会装自己的 SIGINT 处理，装完就把信号
    吞了 —— 服务端前台按 Ctrl-C 毫无反应（只能 SIGTERM / Ctrl-\\），操作员会以为卡死。
    `ZenohFrameSource.start()` 在 `zenoh.open()` 之后再装一次 Python 的处理器盖回来。

    信号这种进程级行为只能在**子进程**里验：父进程发 SIGINT，看子进程有没有把
    KeyboardInterrupt 走到。
    """
    try:
        import zenoh  # noqa: F401
    except ImportError:
        print("[6] 跳过 SIGINT 用例：本解释器没有 eclipse-zenoh")
        return
    child = subprocess.Popen([sys.executable, os.path.abspath(__file__),
                              "--sigint-child"],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True)
    try:
        # 等它打出 READY（= 会话已开、订阅已声明，此时正是被 zenoh 装过信号的时刻）
        deadline = time.time() + 15.0
        while time.time() < deadline:
            if child.stdout.readline().strip() == "READY":
                break
        else:
            raise AssertionError("子进程没在 15s 内就绪")
        child.send_signal(signal.SIGINT)
        out, _ = child.communicate(timeout=15.0)
    finally:
        if child.poll() is None:
            child.kill()
    assert "SIGINT-OK" in out, "SIGINT 被子进程吞了：%r" % out
    print("[6] SIGINT：开 zenoh 会话后 Ctrl-C 仍能打断（zenoh 会吞信号，已在 start() 里盖回来）")


def _sigint_child():
    """被 `test_ctrl_c_not_swallowed_by_zenoh` 当子进程跑：开会话 → 等 SIGINT。

    用的是**出厂默认**（peer + 默认端点），桥不在也没关系（peer 容忍连不上）。
    """
    source = ZenohFrameSource(frame_topic())
    source.start()
    print("READY", flush=True)
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("SIGINT-OK", flush=True)
    finally:
        source.stop()
    return 0


def main():
    test_topic()
    test_source_defaults()
    test_parse_endpoints()
    test_queue_discipline()
    test_real_zenoh_roundtrip_bridge_starts_late()
    test_ctrl_c_not_swallowed_by_zenoh()
    print("VR SOURCE TESTS PASSED")
    return 0


if __name__ == "__main__":
    if "--sigint-child" in sys.argv:
        sys.exit(_sigint_child())
    sys.exit(main())

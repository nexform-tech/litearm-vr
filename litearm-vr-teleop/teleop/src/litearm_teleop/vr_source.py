"""VR 手柄帧的数据源：中间层 zenoh（出厂）。

链路（见 `litearm-vr-bridge/docs/DESIGN.md` §3/§5）：

    头显 ──ws 8765──▶ litearm-vr-bridge ──zenoh litearm/v4/{arm}/vr_frame──▶ teleop

桥和本层均保留控制状态边沿及其首帧，连续相同状态只保留最新位姿。
按钮边沿自身溢出时发出 teleopInputFault，消费者必须保持并等待松开后重新握持。
zenoh 回调只解码、入队；主线程负责 IK。合并和溢出均计入 frames_dropped。

本模块**只用标准库**且**延迟 import zenoh**：`--source ws` 时不需要装
`eclipse-zenoh`，`launch/run_teleop_webxr.py`（只用标准库）也要能 import 本模块
拿默认值。
"""
import json
import queue
import signal
import sys
import time

#: 协议根与 arm 段默认值 —— 与 litearm-vr-bridge 的 `protocol.py` 必须一致，
#: 也对齐 `litearm-server --arm-id`（两边填不同值就订阅不到任何帧）。
PREFIX = "litearm/v4"
DEFAULT_ARM_ID = "armA"

#: 数据源：zenoh = 订阅中间层帧流（出厂）；ws = 自建 WS 监听（头显直连本机，回退）。
DEFAULT_SOURCE = "zenoh"

#: zenoh 会话默认值：**peer + 显式连桥的监听端点**。
#:
#: 为什么是 peer 而不是 client：`client` 模式要求**开会话时**就连上至少一个端点，
#: 桥没起就 `zenoh.open` 直接抛 `ZError: Unable to connect to any of [...]` —— 服务端
#: 崩在启动阶段，而「先起 teleop、后起桥」是完全正常的顺序。peer 模式容忍连不上
#: （后台重试），桥起来后自动接上（实测）。
#:
#: 为什么不 listen：桥自己 listen 在 7447，同机再听同一个端口会撞（桥的
#: `deploy/zenoh-router.service` 注释写明了）。
DEFAULT_ZENOH_MODE = "peer"
DEFAULT_ZENOH_CONNECT = ("tcp/127.0.0.1:7447",)
DEFAULT_ZENOH_LISTEN = ()

#: 帧队列深度（见模块开头）。
DEFAULT_QUEUE_SIZE = 64


def frame_topic(arm_id=DEFAULT_ARM_ID):
    """头显原始帧话题：桥 pub，本仓订阅。与桥的 `protocol.frame_topic` 同一拼法。"""
    return "%s/%s/vr_frame" % (PREFIX, arm_id)


def parse_endpoints(value):
    """配置/命令行里的端点 → tuple。

    四种写法都收：None（= 空）、单个 `"tcp/a:7447"`、逗号分隔的
    `"tcp/a:7447,tcp/b:7447"`（与桥的 `ZENOH_*_ENDPOINTS` 环境变量同一种写法）、
    以及配置文件里的 flow 序列 `[tcp/a:7447]`。空串与空元素都丢掉。
    """
    if value is None:
        return ()
    items = [value] if isinstance(value, str) else list(value)
    out = []
    for item in items:
        out += [part.strip() for part in str(item).split(",") if part.strip()]
    return tuple(out)




class _LatestFrameQueue(queue.Queue):
    """Keep each control edge and its first pose, then only its newest pose.

    Packages deploy independently, so this small queue is mirrored in bridge/teleop.
    Overflow of distinct edges emits a fault barrier; the consumer must hold until
    release/regrip. Payloads that survive coalescing remain byte-for-byte unchanged.
    """

    def __init__(self, maxsize):
        if maxsize < 2:
            raise ValueError("queue_size must be >= 2 (fault barrier + latest frame)")
        super().__init__(maxsize)
        self._signature = None
        self._replaceable = False

    @staticmethod
    def _controls(raw):
        try:
            data = json.loads(raw)
            if not isinstance(data, dict) or "teleopInputFault" in data:
                return None
            states = []
            for hand in ("leftController", "rightController"):
                controller = data.get(hand)
                if controller is not None and not isinstance(controller, dict):
                    return None
                states.append(None if controller is None else {
                    key: value for key, value in controller.items()
                    if key not in ("position", "rotation", "quaternion")})
            if states == [None, None]:
                return None
            return json.dumps(states, sort_keys=True)
        except (ValueError, TypeError):
            return None

    def offer_latest(self, item, raw, fault_item):
        """Return discarded count. A first/edge pose is never coalesced away."""
        signature = self._controls(raw)
        with self.not_full:
            same = signature is not None and signature == self._signature
            if same and self._replaceable and self.queue:
                self.queue[-1] = item
                return 1
            discarded = 0
            if self._qsize() >= self.maxsize:
                discarded = self._qsize()
                self.queue.clear()
                self.queue.append(fault_item)
                # The newest frame becomes a new, protected reference.
                same = False
            self.queue.append(item)
            self._signature = signature
            self._replaceable = same
            self.not_empty.notify()
            return discarded


class ZenohFrameSource:
    """订阅 `vr_frame`，把 payload 文本排进队列。

    丢帧照实计数：同状态合并旧位姿，边沿溢出发送失效屏障；解不出 UTF-8 文本、
    或载荷为空的样本（delete 之类）也计入 `frames_dropped`。
    """

    def __init__(self, topic, mode=DEFAULT_ZENOH_MODE, connect=(),
                 listen=(), queue_size=DEFAULT_QUEUE_SIZE):
        self._topic = topic
        self._mode = mode
        self._connect = parse_endpoints(connect)
        self._listen = parse_endpoints(listen)
        self._queue = _LatestFrameQueue(maxsize=queue_size)
        self._session = None
        self._subscriber = None
        self.frames_received = 0
        self.frames_dropped = 0
        self.last_error = None
        self.last_frame_mono = None

    # ── 生命周期 ────────────────────────────────────────────────────────────
    def start(self):
        """开会话并声明订阅者。

        两种失败都指名道姓：zenoh 没装（ImportError，告诉装哪个包）、会话打不开
        （RuntimeError，多半是 client 模式连不上端点）。
        """
        try:
            import zenoh
        except ImportError as error:
            raise ImportError(
                "数据源 zenoh 需要 eclipse-zenoh，但当前解释器里没有：%s\n"
                "       装它：%s -m pip install eclipse-zenoh"
                "（或用 --source ws 走自建 WS 监听）" % (sys.executable, sys.executable)
            ) from error

        config = zenoh.Config()
        config.insert_json5("mode", json.dumps(self._mode))
        if self._connect:
            config.insert_json5("connect/endpoints", json.dumps(list(self._connect)))
        if self._listen:
            config.insert_json5("listen/endpoints", json.dumps(list(self._listen)))
        if self._connect:
            # 给了显式端点就不做组播/流言发现：本机只该跟写明的那个桥说话，
            # 免得在同网段里跟无关的 zenoh 节点勾搭上。
            config.insert_json5("scouting/multicast/enabled", "false")
            config.insert_json5("scouting/gossip/enabled", "false")

        try:
            self._session = zenoh.open(config)
        except Exception as error:
            # client 模式连不上端点时会在这里抛（peer 不会）。指名说清楚怎么办，
            # 别让操作者对着 `ZError: Unable to connect to any of [...]` 猜。
            raise RuntimeError(
                "zenoh 会话打不开（mode=%s connect=%s）：%s\n"
                "       client 模式要求开会话时就已连上端点 —— 桥没起就会这样。"
                "改 `vr.zenoh_mode: peer`（出厂值）即可容忍「先起 teleop 后起桥」。"
                % (self._mode, ",".join(self._connect) or "<空>", error)
            ) from error
        self._subscriber = self._session.declare_subscriber(self._topic,
                                                            self._on_sample)
        self._restore_sigint()
        return self._subscriber

    @staticmethod
    def _restore_sigint():
        """把 Ctrl-C 从 zenoh 手里拿回来。

        zenoh 的 Rust 运行时会装自己的 SIGINT 处理（它要做优雅退出），装完就把信号
        吞了 —— Ctrl-C 不再变成 Python 的 KeyboardInterrupt，于是 `make teleop`
        前台按 Ctrl-C 停不下来（只能 SIGTERM 或 Ctrl-\\），操作员会以为服务卡死。
        它直接调 sigaction、不经过 Python 的 signal 模块，所以在**它之后再装一次**
        Python 的处理器就能盖回来（实测有效，见 tests/test_vr_source.py 的信号用例）。

        ⚠ 必须在主线程调（`signal.signal` 的约束）；本模块的 `start()` 就是主线程调的。
        """
        try:
            signal.signal(signal.SIGINT, signal.default_int_handler)
        except ValueError:
            # 非主线程调用（本项目没有这种用法）：那就别动信号，别让启动崩掉
            pass

    def stop(self):
        """undeclare + 关会话。幂等；没起来过就是空操作。"""
        subscriber, self._subscriber = self._subscriber, None
        session, self._session = self._session, None
        if subscriber is not None:
            subscriber.undeclare()
        if session is not None:
            session.close()

    # ── 观测 ────────────────────────────────────────────────────────────────
    def describe(self):
        """一行日志：话题 + 会话怎么连的。"""
        parts = ["mode=%s" % self._mode]
        if self._connect:
            parts.append("connect=%s" % ",".join(self._connect))
        if self._listen:
            parts.append("listen=%s" % ",".join(self._listen))
        return "%s（%s）" % (self._topic, " ".join(parts))

    # ── 收帧（zenoh 线程） ──────────────────────────────────────────────────
    def _on_sample(self, sample):
        """zenoh 回调：只解码 + 入队，**不做 I/O、不做 IK**（见模块开头）。"""
        try:
            raw = bytes(sample.payload).decode("utf-8")
        except UnicodeDecodeError as error:
            return self._reject("payload 不是 UTF-8 文本: %s" % error)
        if not raw.strip():
            return self._reject("空载荷样本（不是帧）")
        discarded = self._queue.offer_latest(
            raw, raw, '{"teleopInputFault":"queue_overflow"}')
        if discarded:
            self.frames_dropped += discarded
            self.last_error = "合并旧位姿或队列溢出（%d 帧）" % discarded
        self.frames_received += 1
        self.last_frame_mono = time.monotonic()

    def _reject(self, reason):
        """记一次丢帧。`last_error` 只留最近一条，够现场判「为什么没跟上」。"""
        self.frames_dropped += 1
        self.last_error = reason
        return False

    # ── 取帧（主线程） ──────────────────────────────────────────────────────
    def recv(self, timeout=None):
        """取一帧原始 JSON 文本；超时返回 None。

        服务端用带超时的轮询（`asyncio.to_thread`）：被 Ctrl-C 打断时线程最多再
        等一个 timeout 就结束，不会卡住 asyncio 解释器关闭时的线程 join。
        """
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

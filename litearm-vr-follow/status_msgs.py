#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""follow 整机状态的消息类型（ROS 风格 Python 类 + JSON）。

**非** ROS2 线格式：字段名/类型按 ROS `.msg` 习惯写，序列化走
`json.dumps(asdict(self))`（人类可读、自研消费方易解析）。**不可与真 ROS2 互操作**
（真互操作要 CDR/rosidl，另说）。本模块**纯 stdlib**，不 import zenoh。

**发布**按设备分 topic：`status_aggregator.py` 为每个设备各发一条独立消息
（臂 `ArmStatus`、夹爪 `GripperStatus`、手 `HandStatus`——各自 `to_json`）。
`FollowStatus`（`header` + `seq` + 若干 `ArmStatus` + `GripperStatus` + `hand`）是
早先的**合并**封装，现已不再发布，仅保留供解析/测试。

归一化：上游 follow 服务的 `ping` 回包有 stm32 / sdk 两版（字段不同），
`parse_arm_reply` 把两版都收敛成本模块的 `ArmStatus`——规则照搬 bridge 的
`FollowClient._absorb_state`：`fault` 优先、回退 `flags`；`q` 仅当 `len==7` 才收。
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import List, Optional

QUOTE_JSON = {"separators": (",", ":")}


# ---------------------------------------------------------------- 基础类型
@dataclass
class Time:
    """ROS `builtin_interfaces/Time`。"""
    sec: int = 0
    nanosec: int = 0


@dataclass
class Header:
    """ROS `std_msgs/Header`（frame_id 用默认）。"""
    stamp: Time = field(default_factory=Time)
    frame_id: str = ""

    @classmethod
    def from_dict(cls, d):
        if d is None:
            return cls()
        t = d.get("stamp") or {}
        return cls(stamp=Time(int(t.get("sec", 0)), int(t.get("nanosec", 0))),
                   frame_id=str(d.get("frame_id", "")))


@dataclass
class JointArray:
    """7 关节的并行数组（ROS 惯用：一条消息装齐所有关节的同类量）。"""
    q: List[float] = field(default_factory=list)
    dq: List[float] = field(default_factory=list)
    tau: List[float] = field(default_factory=list)
    t_mos: List[float] = field(default_factory=list)
    t_coil: List[float] = field(default_factory=list)
    err: List[int] = field(default_factory=list)

    @classmethod
    def from_dict(cls, d):
        if d is None:
            return None
        g = lambda k: list(d.get(k) or [])  # noqa: E731
        return cls(q=g("q"), dq=g("dq"), tau=g("tau"), t_mos=g("t_mos"),
                   t_coil=g("t_coil"), err=g("err"))


@dataclass
class FaultEntry:
    """一条关节故障：`joint` 号 + 该关节的 err 码。"""
    joint: int = 0
    err: int = 0

    @classmethod
    def from_dict(cls, d):
        return cls(int(d.get("joint", 0)), int(d.get("err", 0)))


# ---------------------------------------------------------------- 设备状态
@dataclass
class ArmStatus:
    """一条臂的实测状态（stm32 / sdk 两版归一化后的并集）。

    `target`/`anchor`/`cmd_q`/`joints` 只有 stm32 版有；sdk 版缺省为 null。
    `fault`(sdk 风格，FaultEntry 列表) 与 `flags`(stm32 风格，置位名列表) 语义一致
    ——健康时都空。
    """
    device: str = ""
    connected: bool = False
    ok: bool = False
    source: str = ""                  # "stm32" | "sdk" | ""
    state: Optional[str] = None
    mode: Optional[object] = None
    enabled: Optional[bool] = None
    following: Optional[bool] = None
    flags: List[str] = field(default_factory=list)
    fault: List[FaultEntry] = field(default_factory=list)
    joint_fault: Optional[int] = None
    q: Optional[List[float]] = None
    target: Optional[List[float]] = None
    anchor: Optional[List[float]] = None
    cmd_q: Optional[List[float]] = None
    joints: Optional[JointArray] = None
    age_ms: float = 0.0
    error: Optional[str] = None

    # ---- 序列化（分设备 topic 直接发本体，不套 FollowStatus 信封）----
    def to_dict(self):
        return asdict(self)

    def to_json(self):
        return json.dumps(self.to_dict(), **QUOTE_JSON)

    @classmethod
    def from_dict(cls, d):
        return cls(
            device=str(d.get("device", "")),
            connected=bool(d.get("connected", False)),
            ok=bool(d.get("ok", False)),
            source=str(d.get("source", "")),
            state=d.get("state"),
            mode=d.get("mode"),
            enabled=d.get("enabled"),
            following=d.get("following"),
            flags=list(d.get("flags") or []),
            fault=[FaultEntry.from_dict(x) for x in (d.get("fault") or [])],
            joint_fault=d.get("joint_fault"),
            q=d.get("q"),
            target=d.get("target"),
            anchor=d.get("anchor"),
            cmd_q=d.get("cmd_q"),
            joints=JointArray.from_dict(d.get("joints")),
            age_ms=float(d.get("age_ms", 0.0)),
            error=d.get("error"),
        )


@dataclass
class GripperStatus:
    """夹爪 DM 电机的状态：**实测**（开合 + 弧度/扭矩/使能/错误码）+ **命令意图**。

    ⚠ `ratio` **由发布端算**（`litegrip.teleop.rad_to_openness`）。理由与 litegrip 的
    线上设计同源（`teleop.py` 的 "Why the wire carries `openness` and not radians"）：
    每个夹爪有各自的零点、方向与行程标定，**原始角度在对面没有意义** —— 下游各自拿
    常量去换算，就一定会有一方用的是别人的标定。这台机器上真发生过：服务端按 A 标定
    运动、采集端按 B 标定换算，录出来的开合比例被压缩 14%，而且值仍在 [0,1] 内看不出来。

    `ratio` 与 `target_ratio` 的分工：前者是**实测**（现在到哪了），后者是**命令**
    （操作者想让它到哪）。
    """
    device: str = ""
    connected: bool = False
    ok: bool = False
    q: float = 0.0
    tau: float = 0.0
    enabled: Optional[bool] = None
    error_code: Optional[int] = None
    #: 张开量（mm）。单位是**物理量**，跨机比较仍有意义（与 `ratio` 不同，后者依赖标定）。
    position_mm: Optional[float] = None
    #: 归一化开合比例 [0,1]（0 = 全闭）。按**完整标定行程**归一，见上文。
    ratio: Optional[float] = None
    #: **命令意图**：最近一次 `close`/`open` 折算成的目标比例（close→0.0 / open→1.0）；
    #: 从未收到过命令 = None。⚠ 它**粘住不清**（是「操作者的意图」而不是「此刻是否还在
    #: 动」），且链路故障/旧版服务缺该字段时**保留上一次的值**。
    target_ratio: Optional[float] = None
    age_ms: float = 0.0
    error: Optional[str] = None

    # ---- 序列化（分设备 topic 直接发本体）----
    def to_dict(self):
        return asdict(self)

    def to_json(self):
        return json.dumps(self.to_dict(), **QUOTE_JSON)

    @classmethod
    def from_dict(cls, d):
        if d is None:
            return None
        def _opt_f(key):
            v = d.get(key)
            return None if v is None else float(v)

        return cls(
            device=str(d.get("device", "")),
            connected=bool(d.get("connected", False)),
            ok=bool(d.get("ok", False)),
            q=float(d.get("q", 0.0)),
            tau=float(d.get("tau", 0.0)),
            enabled=d.get("enabled"),
            error_code=d.get("error_code"),
            position_mm=_opt_f("position_mm"),
            ratio=_opt_f("ratio"),
            target_ratio=_opt_f("target_ratio"),
            age_ms=float(d.get("age_ms", 0.0)),
            error=d.get("error"),
        )


@dataclass
class HandStatus:
    """灵巧手（v1 占位：现无状态读取，恒为空）。"""
    device: str = ""
    connected: bool = False
    ok: bool = False
    angles: List[float] = field(default_factory=list)
    error: Optional[str] = None

    # ---- 序列化（分设备 topic 直接发本体）----
    def to_dict(self):
        return asdict(self)

    def to_json(self):
        return json.dumps(self.to_dict(), **QUOTE_JSON)

    @classmethod
    def from_dict(cls, d):
        return cls(
            device=str(d.get("device", "")),
            connected=bool(d.get("connected", False)),
            ok=bool(d.get("ok", False)),
            angles=list(d.get("angles") or []),
            error=d.get("error"),
        )


# ---------------------------------------------------------------- 顶层消息
@dataclass
class FollowStatus:
    """合并消息：一次整机状态快照（臂 + 夹爪 + 手部占位）。

    **已不再用于发布**：`status_aggregator.py` 现改为**每设备一条独立 topic**、
    直接发各设备本体（`ArmStatus` / `GripperStatus` / `HandStatus` 的 `to_json`）。
    本类保留仅供解析/测试（`from_json` 等）与历史兼容。
    """
    header: Header = field(default_factory=Header)
    seq: int = 0
    arms: List[ArmStatus] = field(default_factory=list)
    gripper: Optional[GripperStatus] = None
    hand: List[HandStatus] = field(default_factory=list)

    # ---- 序列化 ----
    def to_dict(self):
        return asdict(self)

    def to_json(self):
        return json.dumps(self.to_dict(), **QUOTE_JSON)

    @classmethod
    def from_dict(cls, d):
        return cls(
            header=Header.from_dict(d.get("header")),
            seq=int(d.get("seq", 0)),
            arms=[ArmStatus.from_dict(x) for x in (d.get("arms") or [])],
            gripper=GripperStatus.from_dict(d.get("gripper")),
            hand=[HandStatus.from_dict(x) for x in (d.get("hand") or [])],
        )

    @classmethod
    def from_json(cls, s):
        return cls.from_dict(json.loads(s))


# ---------------------------------------------------------------- 归一化解析
def _vec7(v):
    """只收长度 7 的浮点向量；否则 None。"""
    if isinstance(v, (list, tuple)) and len(v) == 7:
        return [float(x) for x in v]
    return None


def _parse_fault(v):
    """sdk 风格 `fault`: `[(joint, err), …]` 或 `[[joint, err], …]`。"""
    out = []
    if not isinstance(v, (list, tuple)):
        return out
    for item in v:
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            out.append(FaultEntry(int(item[0]), int(item[1])))
        elif isinstance(item, dict):
            out.append(FaultEntry.from_dict(item))
    return out


def _parse_joints(v):
    """stm32 风格 `joints`: `[(q, dq, tau, t_mos, t_coil, err), …]` → 并行数组。"""
    if not isinstance(v, (list, tuple)) or not v:
        return None
    q, dq, tau, t_mos, t_coil, err = [], [], [], [], [], []
    for row in v:
        if not isinstance(row, (list, tuple)) or len(row) < 6:
            return None
        q.append(float(row[0]))
        dq.append(float(row[1]))
        tau.append(float(row[2]))
        t_mos.append(float(row[3]))
        t_coil.append(float(row[4]))
        err.append(int(row[5]))
    return JointArray(q=q, dq=dq, tau=tau, t_mos=t_mos, t_coil=t_coil, err=err)


def parse_arm_reply(device, reply, prev=None):
    """把臂 follow 服务的 `ping` 回包归一化成 `ArmStatus`。

    容忍 stm32 / sdk 两版：
      - stm32: `{ok, state(mode_name), flags, enabled, joint_fault, q, target, anchor,
                 cmd_q, following, mode}`（**无** `fault`）
      - sdk  : `{ok, state, q, fault}`，或故障时 `{ok, state, connected:false, error}`
    连接失败 / 回包 `ok==false` 时**保留 `prev` 的实测值**，只把 `connected` 置假、
    带上 `error`（`age_ms` 由调用方按时间重算）。
    """
    if not isinstance(reply, dict):
        return None

    ok = bool(reply.get("ok", True))
    connected = bool(reply.get("connected", True)) and ok
    source = _arm_source(reply)
    error = reply.get("error")

    if not connected and prev is not None:
        # 链路在但设备报故障：保留上次实测，只更新连接/错误
        return ArmStatus(
            device=device, connected=False, ok=ok, source=source or prev.source,
            state=reply.get("state", prev.state),
            mode=reply.get("mode", prev.mode),
            enabled=prev.enabled, following=prev.following,
            flags=prev.flags, fault=prev.fault, joint_fault=prev.joint_fault,
            q=prev.q, target=prev.target, anchor=prev.anchor, cmd_q=prev.cmd_q,
            joints=prev.joints, age_ms=prev.age_ms, error=error or prev.error)

    flags = reply.get("flags")
    return ArmStatus(
        device=device, connected=connected, ok=ok, source=source,
        state=reply.get("state"),
        mode=reply.get("mode"),
        enabled=reply.get("enabled"),
        following=reply.get("following"),
        flags=[str(x) for x in flags] if isinstance(flags, list) else [],
        fault=_parse_fault(reply.get("fault")),
        joint_fault=reply.get("joint_fault"),
        q=_vec7(reply.get("q")),
        target=_vec7(reply.get("target")),
        anchor=_vec7(reply.get("anchor")),
        cmd_q=_vec7(reply.get("cmd_q")),
        joints=_parse_joints(reply.get("joints")),
        age_ms=0.0, error=error)


def _arm_source(reply):
    """辨识回包来自哪一版服务。"""
    if reply.get("connected") is False or "fault" in reply:
        return "sdk"
    if any(k in reply for k in ("mode", "flags", "target", "cmd_q")):
        return "stm32"
    return ""


def parse_gripper_reply(device, reply, prev=None):
    """把夹爪服务的 `status` 回包归一化成 `GripperStatus`。

    期望 `{ok, q, tau, enabled, error_code, fresh, age_ms, position_mm, ratio,
    target_ratio}`；失败时保留 `prev` 的实测值与命令意图、`connected=false` + `error`。

    `fresh` 是**硬件**真话（服务端最近一次 CAN 读是否成功）：夹爪掉线时服务端仍
    回得动 TCP、`ok=true`，回包里的 `q`/`tau` 只是缓存冻结值，唯 `fresh=false` 能
    区分「真在线」与「进程在线但硬件没了」。故 `fresh` 与 `ok` 一起 gate `connected`。
    （`ping` 回包不带 `fresh`，默认 True，不受影响。）
    """
    if not isinstance(reply, dict):
        return None

    ok = bool(reply.get("ok", True))
    connected = (bool(reply.get("connected", True)) and ok
                 and bool(reply.get("fresh", True)))
    error = reply.get("error")

    def _opt(key, fallback):
        # ⚠ 缺字段 ≠ 「没有值」：`target_ratio` 是粘住的意图，链路一抖把它擦成 None
        #   会让下游丢掉已经收到的命令；`ratio`/`position_mm` 是实测，也照 prev 保留。
        v = reply.get(key)
        if v is not None:
            return float(v)
        return getattr(prev, key, None) if prev is not None else fallback

    if not connected and prev is not None:
        return GripperStatus(
            device=device, connected=False, ok=ok, q=prev.q, tau=prev.tau,
            enabled=prev.enabled, error_code=prev.error_code,
            position_mm=_opt("position_mm", None), ratio=_opt("ratio", None),
            target_ratio=_opt("target_ratio", None),
            age_ms=prev.age_ms, error=error or prev.error)

    return GripperStatus(
        device=device, connected=connected, ok=ok,
        q=float(reply.get("q", 0.0)), tau=float(reply.get("tau", 0.0)),
        enabled=reply.get("enabled"), error_code=reply.get("error_code"),
        position_mm=_opt("position_mm", None), ratio=_opt("ratio", None),
        target_ratio=_opt("target_ratio", None),
        age_ms=float(reply.get("age_ms", 0.0)), error=error)


def parse_hand_reply(device, reply, prev=None):
    """把灵巧手服务的 `ping` 回包归一化成 `HandStatus`（v1 无角度，恒空）。"""
    if not isinstance(reply, dict):
        return None
    ok = bool(reply.get("ok", True))
    angles = reply.get("angles")
    return HandStatus(
        device=device, connected=ok, ok=ok,
        angles=[float(x) for x in angles] if isinstance(angles, list) else [],
        error=reply.get("error"))

"""VR 链路在 zenoh 上的话题（key expression）约定。

命名跟着 `litearm-server` 那条既有规则走，别自创：

    litearm/v4/{arm_id}/{channel}

`arm_id` 是**单个** key-expression 段（默认 `armA`），channel 是扁平的单个后缀。
既有四个通道是 `rpc` / `state` / `command` / `estop`，**新增能力只加新后缀，绝不占用
那四个** —— 它们各有各的载荷格式和语义，混用会让订阅方分不清谁是谁。

本服务涉及三个：

| 话题 | 类型 | 方向 | 载荷 |
| --- | --- | --- | --- |
| `vr_frame`  | pub/sub   | 桥 → teleop    | 头显原始帧，原样透传，~90Hz |
| `vr_status` | pub/sub   | 桥 → 上位机    | 桥的状态快照，JSON，10Hz |
| `vr_rpc`    | queryable | 上位机 → 桥    | 动作请求/回执，JSON |

为什么不复用既有的两个：

* **不用 `state`**：那是 `litearm-server` 的 `RobotState`（protobuf，~50Hz）。桥的快照
  是**链路**状态不是机械臂状态；同一个 key 上两种载荷，订阅方无从分辨。
* **不用 `rpc`**：那是 litearm-server 的运动/调参 RPC（protobuf，带方法路由表）。
  `teleop.start` 不该出现在那张表里。
* **叫 `vr_rpc` 不叫 `vr_command`**：这套约定里 `rpc` 就是请求/应答通道，而 `command`
  是高频 pub/sub 目标流 —— 叫 `vr_command` 会让人以为可以 `put()` 一个急停。

跟随服务将来报真机状态时**也走它自己的新后缀**（比如 `arm_state`），别挤 `state` ——
理由同上。
"""

from __future__ import annotations

__all__ = ["PREFIX", "DEFAULT_ARM_ID", "frame_topic", "status_topic", "rpc_topic"]

#: 协议根。`v4` 是**产品代际**，与 `litearm-server` 的 `litearm/v4/...` 对齐。
PREFIX = "litearm/v4"

#: 和 `litearm-server --arm-id` 的默认值一致；两边必须填同一个值才连得上。
DEFAULT_ARM_ID = "armA"


def frame_topic(arm_id: str = DEFAULT_ARM_ID) -> str:
    """头显原始帧：桥 pub，teleop 订阅。

    帧是控制输入，逐帧透传、不合并、不节流（DESIGN §6.1 的转发纪律）。
    """
    return f"{PREFIX}/{arm_id}/vr_frame"


def status_topic(arm_id: str = DEFAULT_ARM_ID) -> str:
    """桥的状态快照：桥 pub，上位机订阅。整包覆盖式，字段见 DESIGN §4.1。"""
    return f"{PREFIX}/{arm_id}/vr_status"


def rpc_topic(arm_id: str = DEFAULT_ARM_ID) -> str:
    """动作接口：桥声明 queryable，上位机 query。

    用 query 而不是 pub 的理由：它是唯一能区分「命令没送到」和「整条链路断了」的手段
    —— 桥挂住没死时 pub 下去没人处理，界面只能等状态超时；query 超时就能给出结论。
    """
    return f"{PREFIX}/{arm_id}/vr_rpc"

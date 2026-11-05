"""头显帧解析：只取统计所需的少量字段，**不改帧内容**。

帧格式见 litearm-vr/docs/PROTOCOL.md：

    {
      "timestamp": 1712345678901,          # 头显 Unix 毫秒
      "leftController":  {"tracked": bool, "position": {...}, "quaternion": {...},
                          "rotation": {...}, "gripActive": bool, "trigger": float},
      "rightController": { ... }
    }

遥操只用右手（见 vr/teleop），所以 grip 语义取自 rightController。
"""

from __future__ import annotations

import json
from dataclasses import dataclass

__all__ = ["ControllerState", "FrameInfo", "parse_frame"]

# 秒级时间戳（1e9 ~ 1e12）视为秒，乘 1000 归一到毫秒
_SECONDS_LO = 1e9
_SECONDS_HI = 1e12
_MILLIS_LO = 1e12
_MILLIS_HI = 1e14


@dataclass(frozen=True)
class ControllerState:
    tracked: bool = False
    grip: bool = False
    trigger: float = 0.0


@dataclass(frozen=True)
class FrameInfo:
    """一帧里我们关心的内容；解析失败时为全 None。"""

    ts_ms: float | None = None
    right: ControllerState | None = None
    left: ControllerState | None = None

    @property
    def grip(self) -> bool:
        """右手是否握着 grip（握把键按下且被跟踪）——遥操的"发令"信号。"""
        return bool(self.right and self.right.tracked and self.right.grip)

    @property
    def tracked(self) -> bool:
        return bool(self.right and self.right.tracked)


def parse_frame(raw: str) -> FrameInfo:
    """解析一帧 JSON；任何异常都吞掉（坏帧仍会被原样转发）。"""
    try:
        data = json.loads(raw)
    except (ValueError, TypeError):
        return FrameInfo()
    if not isinstance(data, dict):
        return FrameInfo()
    return FrameInfo(
        ts_ms=normalize_timestamp(data.get("timestamp")),
        right=_controller(data.get("rightController")),
        left=_controller(data.get("leftController")),
    )


def normalize_timestamp(value: object) -> float | None:
    """把帧内时间戳归一到毫秒。

    上游发的是 Unix 毫秒；容忍两种偏差：
      * 秒级时间戳（1e9~1e12）→ ×1000；
      * 其它一律按毫秒返回，由延迟估计去判断是否可比。
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    ts = float(value)
    if ts <= 0:
        return None
    if _SECONDS_LO <= ts < _SECONDS_HI:
        ts *= 1000.0
    return ts


def timestamp_is_wall_clock(ts_ms: float | None) -> bool:
    """时间戳是否像"本机也能比的 Unix 毫秒"。"""
    return ts_ms is not None and _MILLIS_LO <= ts_ms < _MILLIS_HI


def _controller(node: object) -> ControllerState | None:
    if not isinstance(node, dict):
        return None
    return ControllerState(
        tracked=_truthy(node.get("tracked"), default=True),
        grip=_truthy(node.get("gripActive"), default=False),
        trigger=_number(node.get("trigger"), default=0.0),
    )


def _truthy(value: object, *, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return float(value) > 0.5
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return default


def _number(value: object, *, default: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return default
    return float(value)

"""一次性事件（告警 / 动作回执）：短 TTL 保留，供上位机在快照里读到。

DESIGN 4.1 写的是"消费后清空"。上游 litearm-studio 只做 `snapshotValue = JSON.parse(...)`
覆盖式取用，没有消费确认通道；真正"推一次就清"会让告警在偶发丢帧时被吞掉。
因此这里按 TTL（默认 20s）+ 上限保留，`event.ack` 动作可显式清空。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any

__all__ = ["Event", "EventLog"]

_LEVELS = ("debug", "info", "warn", "danger")


@dataclass
class Event:
    id: int
    ts: int
    level: str
    code: str
    message: str
    data: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "id": self.id,
            "ts": self.ts,
            "level": self.level,
            "code": self.code,
            "message": self.message,
        }
        if self.data:
            payload["data"] = self.data
        return payload


class EventLog:
    """有界事件缓冲；同一 code 的连续重复事件会被合并计数，避免刷屏。"""

    def __init__(
        self,
        ttl_ms: float = 20_000.0,
        max_items: int = 50,
        dedupe_window_s: float = 1.0,
    ) -> None:
        self._ttl_ms = max(ttl_ms, 0.0)
        self._max = max(max_items, 1)
        self._dedupe_s = max(dedupe_window_s, 0.0)
        self._items: deque[Event] = deque(maxlen=self._max)
        self._next_id = 1
        self._counts: dict[str, int] = {}
        self._last_code: str | None = None
        self._last_at_mono: float = 0.0

    def add(
        self,
        level: str,
        code: str,
        message: str,
        data: dict[str, Any] | None = None,
        *,
        now_wall_ms: int,
        now_mono: float,
    ) -> Event:
        level = level if level in _LEVELS else "info"
        if (
            code == self._last_code
            and (now_mono - self._last_at_mono) <= self._dedupe_s
            and self._items
            and self._items[-1].code == code
        ):
            # 同一秒内的同码事件合并：就地更新计数，不新增条目
            self._counts[code] = self._counts.get(code, 1) + 1
            merged = self._items[-1]
            merged.message = f"{message}（×{self._counts[code]}）"
            merged.ts = now_wall_ms
            self._last_at_mono = now_mono
            return merged

        event = Event(
            id=self._next_id,
            ts=now_wall_ms,
            level=level,
            code=code,
            message=message,
            data=dict(data or {}),
        )
        self._next_id += 1
        self._items.append(event)
        self._counts[code] = 1
        self._last_code = code
        self._last_at_mono = now_mono
        return event

    def drain(self, *, now_wall_ms: int) -> list[dict[str, Any]]:
        if self._ttl_ms <= 0:
            self._items.clear()
            self._reset_dedupe()
            return []
        cutoff = now_wall_ms - self._ttl_ms
        while self._items and self._items[0].ts < cutoff:
            self._items.popleft()
        if not self._items:
            self._reset_dedupe()
        return [event.to_dict() for event in self._items]

    def clear(self, event_id: int | None = None) -> int:
        if event_id is None:
            count = len(self._items)
            self._items.clear()
            self._reset_dedupe()
            return count
        kept = [event for event in self._items if event.id != event_id]
        removed = len(self._items) - len(kept)
        if removed:
            self._items.clear()
            self._items.extend(kept)
            self._reset_dedupe()
        return removed

    @property
    def size(self) -> int:
        return len(self._items)

    def _reset_dedupe(self) -> None:
        self._counts.clear()
        self._last_code = None
        self._last_at_mono = 0.0

"""帧率 / 延迟 / 转发耗时统计（纯逻辑，便于单测）。

延迟拆两段（DESIGN 6.2）：
  * 无线段 latency_ms = 收到帧时刻 − 帧内 timestamp，正常 5–30ms
  * 本服务段 forward_cost_ms = 转发出去时刻 − 收帧时刻，正常 < 1ms
"""

from __future__ import annotations

from collections import deque

from .frames import FrameInfo, timestamp_is_wall_clock

__all__ = ["RollingStats", "LatencyEstimator", "HeadsetMetrics"]


class RollingStats:
    """最近 N 个样本的 last / mean / p99 / max。"""

    def __init__(self, size: int = 300) -> None:
        self._buf: deque[float] = deque(maxlen=max(size, 1))

    def add(self, value: float) -> None:
        self._buf.append(float(value))

    def clear(self) -> None:
        self._buf.clear()

    @property
    def count(self) -> int:
        return len(self._buf)

    @property
    def last(self) -> float | None:
        return self._buf[-1] if self._buf else None

    @property
    def mean(self) -> float | None:
        if not self._buf:
            return None
        return sum(self._buf) / len(self._buf)

    @property
    def max(self) -> float | None:
        return max(self._buf) if self._buf else None

    def percentile(self, q: float) -> float | None:
        if not self._buf:
            return None
        ordered = sorted(self._buf)
        idx = min(int(round((len(ordered) - 1) * q)), len(ordered) - 1)
        return ordered[idx]


class LatencyEstimator:
    """无线段延迟估计。

    头显发的是 `Time.get_unix_time_from_system()*1000`（Unix 毫秒），正常情况下
    直接相减即可。但头显时钟可能没同步（或跑在别的纪元上），这时用滚动窗口里的
    最小差值当基线，报"相对基线的额外延迟"，并标记 estimated=True。
    """

    def __init__(
        self,
        window_s: float = 10.0,
        plausible_max_ms: float = 2000.0,
        plausible_min_ms: float = -1000.0,
        size: int = 2000,
    ) -> None:
        self._window_s = max(window_s, 0.1)
        self._max = plausible_max_ms
        self._min = plausible_min_ms
        self._samples: deque[tuple[float, float]] = deque(maxlen=size)
        self.estimated = False
        self.clock_skew_ms: float | None = None

    def update(self, recv_mono: float, recv_wall_ms: float, ts_ms: float | None) -> float | None:
        if ts_ms is None or recv_wall_ms <= 0:
            return None
        raw = recv_wall_ms - ts_ms
        self._samples.append((recv_mono, raw))
        self._prune(recv_mono)

        if timestamp_is_wall_clock(ts_ms) and self._min <= raw <= self._max:
            self.estimated = False
            self.clock_skew_ms = None
            return max(0.0, raw)

        # 时钟不可比：以窗口内最小 raw 为基线，报相对延迟
        baseline = min(sample for _, sample in self._samples)
        self.estimated = True
        self.clock_skew_ms = baseline
        return max(0.0, raw - baseline)

    def _prune(self, now_mono: float) -> None:
        cutoff = now_mono - self._window_s
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()


class HeadsetMetrics:
    """头显侧统计：帧率、帧龄、延迟、转发耗时。"""

    def __init__(self, fps_window_s: float = 1.0, latency: LatencyEstimator | None = None) -> None:
        self._fps_window_s = max(fps_window_s, 0.1)
        self._recv_mono: deque[float] = deque(maxlen=4096)
        self.latency = latency if latency is not None else LatencyEstimator()
        self.forward_cost = RollingStats(size=300)

        self.frames = 0
        self.bad_frames = 0
        self.frames_forwarded = 0
        self.frames_dropped = 0
        self.frames_blocked = 0

        self.last_recv_mono: float | None = None
        self.last_recv_wall_ms: float | None = None
        self.last_info: FrameInfo | None = None
        self.last_latency_ms: float | None = None

    # ── 输入 ────────────────────────────────────────────────────────────────
    def on_frame(
        self,
        now_mono: float,
        now_wall_ms: float,
        info: FrameInfo,
    ) -> None:
        self.frames += 1
        if info.ts_ms is None:
            self.bad_frames += 1
        self._recv_mono.append(now_mono)
        self.last_recv_mono = now_mono
        self.last_recv_wall_ms = now_wall_ms
        self.last_info = info
        self.last_latency_ms = self.latency.update(now_mono, now_wall_ms, info.ts_ms)
        self._prune(now_mono)

    def on_forwarded(self, cost_ms: float | None) -> None:
        self.frames_forwarded += 1
        if cost_ms is not None:
            self.forward_cost.add(cost_ms)

    def on_dropped(self) -> None:
        self.frames_dropped += 1

    def on_blocked(self) -> None:
        self.frames_blocked += 1

    # ── 输出 ────────────────────────────────────────────────────────────────
    def fps(self) -> float:
        if len(self._recv_mono) < 2:
            return 0.0
        span = self._recv_mono[-1] - self._recv_mono[0]
        if span <= 1e-6:
            return 0.0
        return (len(self._recv_mono) - 1) / span

    def age_ms(self, now_mono: float) -> float | None:
        if self.last_recv_mono is None:
            return None
        return max(0.0, (now_mono - self.last_recv_mono) * 1000.0)

    def latency_ms(self) -> float | None:
        return self.last_latency_ms

    def _prune(self, now_mono: float) -> None:
        cutoff = now_mono - self._fps_window_s
        while self._recv_mono and self._recv_mono[0] < cutoff:
            self._recv_mono.popleft()

"""遥操状态机与掉线兜底（DESIGN 6.3 / 6.4 / 6.5）。

纯逻辑、无 IO：吃"帧 / 超时 / 操作员动作"，吐出"要下发的命令 + 要报的告警"，
由 bridge 负责真正发 TCP、写事件。这样兜底逻辑可以脱离硬件单测。

状态机：

    IDLE ──帧到达 + grip──▶ FOLLOWING ──grip 松开──▶ LATCHED
      ▲                        │                      │
      │                        └──无帧 > 500ms──▶ FALLBACK
      └──────────── 全链路停止（teleop.stop / estop）────────────┘

硬约束（DESIGN 6.3）：
  * 兜底只允许 `hold`（就地保持）与 `stop`（急停），**绝不擅自 movej/home**；
    `latch` 在跟随服务侧是 `movej(当前角)`，掉线兜底不能用它；
  * 兜底幂等：一次掉线只下一次 hold；
  * 帧流恢复**不自动恢复跟随**：要操作员重新按 grip（teleop 会重新锚定）；
  * 主动断开先不发帧再 latch（顺序由 bridge 保证：先关转发，再执行这里的 latch）。

关于 `teleop.start`（界面上的"开始遥操"）：
  * 冷启动/首次：打开转发 + 请跟随服务 `start`。此时跟随服务 q_target 为空，只是原地
    持位，等操作员按 grip 才真正动 —— 安全。
  * 刚从 stop/estop/兜底 出来（rearm 态）：**不**替操作员 arming 跟随服务。teleop 那边
    还留着上一次的锚点，若此刻放开跟随，teleop 的 q 帧会被接受并按旧锚点伺服 ——
    手柄要是移动过就是一次跳跃。这里要求操作员重新按 grip，让 teleop 重新锚定后再随
    （与 6.3"不自动恢复跟随"同一条原则）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from .frames import FrameInfo

__all__ = ["TeleopState", "Alarm", "Command", "TeleopGovernor"]


class TeleopState(str, Enum):
    IDLE = "idle"
    FOLLOWING = "following"
    LATCHED = "latched"
    FALLBACK = "fallback"


@dataclass
class Alarm:
    level: str
    code: str
    message: str
    data: dict[str, object] = field(default_factory=dict)


@dataclass
class Command:
    """要下发给 arm_follow_server 的命令（只会有 hold / latch / stop / start）。"""

    action: str
    reason: str

    def __post_init__(self) -> None:
        if self.action not in ("hold", "latch", "stop", "start"):
            raise ValueError(
                f"兜底/遥操只允许 hold|latch|stop|start，收到 {self.action!r}"
            )


class TeleopGovernor:
    """帧流 → 遥操状态。"""

    def __init__(
        self,
        *,
        warn_ms: float = 200.0,
        fallback_ms: float = 500.0,
        fallback_enabled: bool = True,
        edge_debounce: int = 3,
    ) -> None:
        if fallback_ms < warn_ms:
            raise ValueError("FRAME_FALLBACK_MS 不能小于 FRAME_WARN_MS")
        self._warn_ms = warn_ms
        self._fallback_ms = fallback_ms
        self._fallback_enabled = fallback_enabled
        self._debounce = max(int(edge_debounce), 1)

        self._enabled = True  # 转发/遥操是否被操作员打开（teleop.stop / estop 会关）
        self._state = TeleopState.IDLE
        self._following = False
        self._ever_followed = False
        self._rearm_required = False

        self._grip_raw = False
        self._grip_stable = False
        self._grip_count = 0

        self._last_frame_mono: float | None = None
        self._stalled = False
        self._stall_peak_ms = 0.0
        self._fallback_active = False
        self._fallback_since_ms: float | None = None
        self._fallback_noop_warned = False
        self._fallback_reason: str | None = None

        self._commands: list[Command] = []
        self._alarms: list[Alarm] = []

    # ── 输入：帧 ────────────────────────────────────────────────────────────
    def on_frame(self, now_mono: float, info: FrameInfo | None) -> None:
        self._last_frame_mono = now_mono

        if self._stalled:
            self._stalled = False
            self._alarms.append(
                Alarm(
                    "info",
                    "frames_recovered",
                    f"帧流已恢复（最长 {self._stall_peak_ms:.0f} ms 无帧）",
                )
            )
            self._stall_peak_ms = 0.0

        if self._fallback_active:
            # 恢复 ≠ 恢复跟随：teleop 需要重新按 grip 才会重新锚定
            self._fallback_active = False
            self._fallback_since_ms = None
            self._fallback_reason = None
            self._rearm_required = True
            self._following = False
            self._state = (
                TeleopState.LATCHED if self._ever_followed else TeleopState.IDLE
            )
            self._alarms.append(
                Alarm(
                    "warn",
                    "fallback_cleared",
                    "帧流恢复；机械臂保持就地持位，需重新按 grip 才能继续跟随",
                )
            )

        if info is None:
            return

        grip = bool(info.grip)
        if grip != self._grip_raw:
            self._grip_raw = grip
            self._grip_count = 1
        else:
            self._grip_count += 1

        if self._grip_count >= self._debounce and grip != self._grip_stable:
            self._grip_stable = grip
            if grip:
                self._on_grip_rising()
            else:
                self._on_grip_falling()

    # ── 输入：超时检查 ──────────────────────────────────────────────────────
    def tick(self, now_mono: float) -> None:
        age = self.age_ms(now_mono)
        if age is None or not self._enabled:
            return

        if age > self._stall_peak_ms:
            self._stall_peak_ms = age

        if age > self._warn_ms and not self._stalled:
            self._stalled = True
            self._alarms.append(
                Alarm(
                    "warn",
                    "frame_stall",
                    f"{age:.0f} ms 没有新帧（阈值 {self._warn_ms:.0f} ms）",
                    {"age_ms": round(age, 1), "warn_ms": self._warn_ms},
                )
            )

        if age <= self._fallback_ms:
            return

        if not self._fallback_enabled:
            if not self._fallback_noop_warned:
                self._fallback_noop_warned = True
                self._alarms.append(
                    Alarm(
                        "danger",
                        "fallback_disabled",
                        f"{age:.0f} ms 没有新帧，但兜底已关闭（FALLBACK_ENABLED=0）",
                    )
                )
            return

        if self._fallback_active:
            return  # 幂等：同一次掉线只下一次 hold

        self._fallback_active = True
        self._fallback_since_ms = age
        self._fallback_reason = f"no_frame>{self._fallback_ms:.0f}ms"
        self._following = False
        self._state = TeleopState.FALLBACK
        self._commands.append(Command("hold", reason=self._fallback_reason))
        self._alarms.append(
            Alarm(
                "danger",
                "fallback_hold",
                f"超过 {self._fallback_ms:.0f} ms 无新帧：已下发 hold 就地保持（不 movej）",
                {"age_ms": round(age, 1), "fallback_ms": self._fallback_ms},
            )
        )

    # ── 输入：操作员动作 ────────────────────────────────────────────────────
    def on_teleop_start(self, now_mono: float) -> None:
        """操作员启用遥操：允许转发；是否同时 arming 跟随服务取决于是否处于 rearm 态。"""
        self._enabled = True
        self._fallback_active = False
        self._fallback_since_ms = None
        self._fallback_reason = None
        self._fallback_noop_warned = False
        self._stalled = False

        if self._rearm_required:
            # 刚从 stop/estop/兜底 出来：转发打开，但不替操作员放开跟随（见类注释）
            self._following = False
            self._state = (
                TeleopState.LATCHED if self._ever_followed else TeleopState.IDLE
            )
            self._alarms.append(
                Alarm(
                    "warn",
                    "teleop_start_needs_regrip",
                    "已打开转发；teleop 仍持有旧锚点，需重新按 grip 才会恢复跟随",
                )
            )
            return

        self._commands.append(Command("start", reason="teleop.start"))
        self._rearm_required = False
        if self._grip_stable and self._fresh(now_mono):
            self._following = True
            self._ever_followed = True
            self._state = TeleopState.FOLLOWING
        else:
            self._following = False
            self._state = TeleopState.LATCHED if self._ever_followed else TeleopState.IDLE

    def on_teleop_stop(self, now_mono: float) -> None:
        """操作员停止遥操：先关转发（bridge 负责顺序），再 latch 就地持位。"""
        self._enabled = False
        self._following = False
        self._rearm_required = True
        self._stalled = False
        self._state = TeleopState.LATCHED if self._ever_followed else TeleopState.IDLE
        self._commands.append(Command("latch", reason="teleop.stop"))

    def on_estop(self, now_mono: float) -> None:
        """急停：转成跟随服务的 stop；在重新 teleop.start 之前不再自动跟帧。"""
        self._enabled = False
        self._following = False
        self._rearm_required = True
        self._stalled = False
        self._fallback_active = False
        self._fallback_reason = None
        self._state = TeleopState.LATCHED if self._ever_followed else TeleopState.IDLE
        self._commands.append(Command("stop", reason="estop"))
        self._alarms.append(Alarm("danger", "estop", "急停：已向跟随服务下发 stop"))

    # ── 输出 ────────────────────────────────────────────────────────────────
    def drain_commands(self) -> list[Command]:
        out = self._commands
        self._commands = []
        return out

    def drain_alarms(self) -> list[Alarm]:
        out = self._alarms
        self._alarms = []
        return out

    # ── 观测 ────────────────────────────────────────────────────────────────
    def age_ms(self, now_mono: float) -> float | None:
        if self._last_frame_mono is None:
            return None
        return max(0.0, (now_mono - self._last_frame_mono) * 1000.0)

    def _fresh(self, now_mono: float) -> bool:
        age = self.age_ms(now_mono)
        return age is not None and age <= self._warn_ms

    @property
    def state(self) -> TeleopState:
        return self._state

    @property
    def following(self) -> bool:
        return self._following

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def grip_held(self) -> bool:
        return self._grip_stable

    @property
    def fallback_active(self) -> bool:
        return self._fallback_active

    @property
    def fallback_reason(self) -> str | None:
        return self._fallback_reason

    @property
    def rearm_required(self) -> bool:
        return self._rearm_required

    def _on_grip_rising(self) -> None:
        if not self._enabled:
            # 操作员已停遥操（或急停）；此时按 grip 不应当恢复跟随
            return
        self._rearm_required = False
        self._following = True
        self._ever_followed = True
        self._state = TeleopState.FOLLOWING
        self._alarms.append(Alarm("info", "follow_started", "grip 按下：开始跟随"))

    def _on_grip_falling(self) -> None:
        if not self._following:
            return
        self._following = False
        self._state = TeleopState.LATCHED
        self._alarms.append(Alarm("info", "follow_paused", "grip 松开：暂停跟随"))

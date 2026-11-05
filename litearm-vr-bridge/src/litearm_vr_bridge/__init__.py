"""litearm-vr-bridge：头显与 teleop 之间的常驻垫片。

职责（见 docs/DESIGN.md）：
  1. 收帧与分流：监听 8765 收头显原始帧，逐帧原样转发给 teleop。
  2. 状态汇总：把头显侧 / 真机侧 / 进程侧信息合成一条状态流给上位机。
  3. 掉线兜底：无帧超时后主动 `hold` 就地保持（不 movej），并告警。
  4. 进程启停：按配置拉起 / 停止 teleop 与 arm_follow_server。

不做：逆解、驱动机械臂、界面、向前端推原始帧。
"""

__version__ = "0.1.0"

__all__ = ["__version__"]

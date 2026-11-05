"""LiteArm V4 七轴机械臂的 pybullet 遥操仿真（单进程，无 zenoh，单/双臂）。

架构（无 ZMQ / 无 Open-Teach / 无 zenoh / 纯手柄输入）：

  Quest 3 WebXR 页面（gripSpace 手柄位姿）→ ws :8765
    → scripts/ws_echo_server.py（解析 left/rightController，gripActive 边沿=启动/暂停）
    → 本模块：固定握持锚点/位姿滤波/受约束跟踪
    → IK（--ik pin 默认：pinocchio+pink 微分 IK；--ik pybullet：
      calculateInverseKinematics）→ 驱动关节
    → pybullet GUI 窗口渲染

双臂（侧装立柱）：左右臂关节链完全相同（复用 litearm_macro），差异仅在
mount_joint 的 rpy——左臂 X 轴朝上、右臂 X 轴朝下（镜像背靠背）。因此
遥操映射的轴符号左右相反（见 SIDE_CONFIG），实现对称协作。

遥操语义：grip 按下建立固定手柄/TCP 锚点；绝对位姿目标经过空间滞回和
时间低通，由独立控制周期持续追踪。限幅不覆盖目标，松开或失效清除追赶。
所有 IK 后端使用同一输入映射。

用法:
  python -m litearm_teleop.teleop_sim [--dual] [--no-viewer] [--tcp-offset X Y Z]
"""
import argparse
import os
import time

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R

from .pose_target import AnchoredPoseTarget

from .exact_ik import (NEAR_SINGULAR_SIGMA, box_binding, describe_failure,
                       exact_task_velocity, bounded_task_velocity)
# pink 后端的 metavr 策略常量（**单一真源在 pin_ik.py**，这里只当 SimConfig 的工厂默认，
# 避免同一组数字在两处各写一份而漂移）。pin_ik 模块级只 import numpy，不会带出 pinocchio。
from .pin_ik import (PINK_DT, PINK_ERR_TOL, PINK_LM_DAMPING, PINK_MANIP_COST,
                     PINK_MANIP_RATE, PINK_MAX_ITERS, PINK_POSTURE_COST)

# 单臂 URDF（models/litearm_description/，与双臂 models/dual_litearm_column_description/ 对称）
# teleop_sim.py 位于 <项目根>/src/litearm_teleop/，往上 2 层到项目根
_PROJECT_DIR = os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))
SINGLE_URDF_DIR = os.path.join(_PROJECT_DIR, "models", "litearm_description")
URDF_PATH = os.path.join(SINGLE_URDF_DIR, "urdf", "litearm_description.urdf")
# 单臂侧装（水平安装）URDF：在 world → base_link 间插入 mount 固定关节
# rpy="-1.5708 0 0"（roll=-90°），使 base 的 Y 轴竖直向下（重力 gy=+9.8）。
SIDE_URDF_PATH = os.path.join(SINGLE_URDF_DIR, "urdf", "litearm_side.urdf")
SINGLE_MESH_PKG = "package://litearm_description/meshes/"
# pybullet 渲染用降采样副本（meshes_lowpoly），原图 meshes/ 保留给 RViz
SINGLE_MESH_DIR = os.path.join(SINGLE_URDF_DIR, "meshes_lowpoly")

# 双臂侧装立柱 URDF（含立柱，单个 robot；package:// 路径运行时替换）
DUAL_URDF_DIR = os.path.join(_PROJECT_DIR, "models",
                             "dual_litearm_column_description")
DUAL_URDF_PATH = os.path.join(DUAL_URDF_DIR, "urdf", "dual_litearm_column.urdf")
DUAL_MESH_PKG = "package://dual_litearm_column_description/meshes/"
# pybullet 渲染用降采样副本（meshes_lowpoly），原图 meshes/ 保留给 RViz
DUAL_MESH_DIR = os.path.join(DUAL_URDF_DIR, "meshes_lowpoly")

# 双臂正装（两只独立竖直臂，无立柱）URDF：与侧装同包（复用同一套 mesh 与 macro），
# 只是两臂以 rpy=0 正装挂在一个轻量共同根下、沿 ±y 分开。单个 robot，14 关节。
UPRIGHT_URDF_PATH = os.path.join(DUAL_URDF_DIR, "urdf", "dual_litearm_upright.urdf")

# ---- 仿真参数 ----
# 左臂初始构型（用户指定：二轴 0.6、四轴 -1.7，其余 0）：
#   base 侧装（绕 X 轴 −90°）后，J1 偏 −90°，使末端世界位姿保持不变
#   （仍为 [0.487, 0.209, 0]，前方 49cm、侧 21cm、高 0），sigma_min≈0.13，限位内
Q_HOME = np.array([-1.5708, 0.6, 0.0, -1.7, 0.0, 0.0, 0.0])
# 单臂正装初始构型（用户指定）：二轴 -0.5、四轴 -2.1、五轴 -1.57，其余 0
SINGLE_Q_HOME = np.array([0.0, -0.5, 0.0, -2.1, -1.57, 0.0, 0.0])
# 双臂镜像符号：右臂 home = 左臂 home × MIRROR_SIGNS（新几何实测，镜像误差 0mm）
# 双臂背靠背侧装（右臂 = 左臂绕立柱 Z 轴转 180°），第 2/4/6 关节符号不变，
# 第 1/3/5/7 关节取反。
MIRROR_SIGNS = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
# 单次IK候选步长兜底（pin/pybullet）；绝对目标不被裁剪结果覆盖。
# 实际执行另受tracking中的关节速度/加速度限制。
MAX_STEP_RAD = 0.20
# 关节速度上限（rad/s）。**只被 --ik exact 用**（速度盒的一半，另一半是「行程/dt」）。
# 出厂 10.0：60Hz 下 10·dt ≈ 0.167 rad/帧，是一个真会起作用的界（手速快时 exact
# 会因此就地持位并印 [持位][box]）。要**关掉**这一界就调到 99.0
# （99·dt ≈ 1.65 rad/帧，已远超单帧能走的行程 ⇒ 等价关掉）。
MAX_JOINT_VEL = 10.0
FREQ = 60.0           # 遥操主循环上限频率（Hz）
# ⚠ 已停用（留作回退）：关节目标低通只服务 --ik pybullet，而该路径已整体回退到
# metavr 的处置方式（metavr 没有这一层）。常量与 config 的 sim.q_lowpass_tau
# 保留以便回退，当前**不参与任何计算**。
Q_LOWPASS_TAU = 0.02  # 关节目标低通时间常数（s），≈8Hz 截止

# ---- 空间滞回（历史DELTA前缀保留，单位已改为m/rad而非每帧增量）----
# 比较完整滤波目标与保持输出；固定握持锚点永不被死区/IK/限幅覆盖。
DELTA_DEADBAND_T_IN = 1.5e-4   # m       进入静止（冻结）
DELTA_DEADBAND_T_OUT = 3.0e-4  # m       退出静止（破冰）
DELTA_DEADBAND_R_IN = 1.5e-3   # rad    进入静止
DELTA_DEADBAND_R_OUT = 3.0e-3  # rad    退出静止
REST_CONFIRM_FRAMES = 3        # 旧配置兼容项，位姿滞回不再依赖帧数

# IK 收敛判定（pybullet 无 ok 标志，用 FK 残差检查）
IK_POS_TOL = 3e-3     # m
IK_ROT_TOL = 0.05     # rad（rotvec 范数）

# ---- 遥操语义常数（实测调定，单臂正装基准 = 左臂） ----
# 手柄→机器人旋转映射：恒等（不做轴对应）。
# ⚠ 它**不等于**「末端绕世界轴转」。恒等 + `tgt_R = cur_R @ exp(rel_rotvec)`（右乘）
#    的结果是末端绕**自身局部轴**转（等价 teleop-vr 的 control_frame: tool）。
#    要绕世界轴转需要左乘 —— 见 `Tuning.control_frame`。
# ⚠ 另：`diag(1,-1,-1)` 共轭 **完全等价** 于逐轴符号 `[1,-1,-1]`（数值验证 0.0 差），
#    所以「翻转一根轴」与「绕某轴转 180°」在 det=+1 的那几个符号上是同一件事。
#    若将来把 `axis_remap` 设成非恒等，`rotation_signs` 必须回到 [1,1,1]，否则修正两次。
CTRL_AXIS_REMAP = np.array(
    [[1.0, 0.0, 0.0],
     [0.0, 1.0, 0.0],
     [0.0, 0.0, 1.0]]
)
# 平移映射（2026-08-27 实测标定，三轴一次定位）：
# ⚠ **WebXR 参考系是右手系，前方是 −z。** W3C WebXR Device API §6.2 逐字：
#     "The native origin for every XRReferenceSpace describes a coordinate system
#      where +X is considered "Right", +Y is considered "Up", and -Z is considered
#      "Forward"."
#   本文件早先写成「+x 右、+y 上、**+z 前**」—— 那是**左手系**三元组，注释是错的
#   （2026-10-14 用探针从 9777 帧真实数据反解，矩阵数值与实测**逐元素一致**，
#    所以错的是注释里的推理前提，不是这段代码。**重标定时别照旧注释推。**）
# 语义 rel_t = translation_remap.T @ 手柄delta，即「列=机器人轴、行=手柄轴」。
# 实测（按右手系读）：手柄右移(+x)→机器人 +x；手柄上移(+y)→机器人 +z；
#   手柄后移(+z)→机器人 −y，等价地 **手柄前推(−z)→机器人 +y**。
CTRL_TRANSLATION_REMAP = np.array(
    [[1.0, 0.0, 0.0],
     [0.0, 0.0, 1.0],
     [0.0, -1.0, 0.0]]
)
# 逐轴旋转符号（作用在 rel_rotvec 上）。**作用在哪个系取决于 control_frame**：
#   control_frame="tool"（出厂）→ 分量 0/1/2 = 末端**局部** X/Y/Z 轴
#   control_frame="base_link"  → 分量 0/1/2 = 机器人**基座** X/Y/Z 轴
# 两个系读的是同样三个数但表达在不同系里，**一边对的镜像在另一边就是错的** ⇒
# 换模式必须换一组符号（见 BASE_LINK_ROTATION_SIGNS）。
# 出厂值：滚转(X)保持；俯仰(Y)与偏航(Z)方向取反（实测调定，与 teleop-vr 的
# `tool_rotation_signs = [1,-1,-1]` 逐值相同 —— 这也独立印证了本文件是 tool 语义）。
ROTATION_AXIS_SIGNS = np.array([1.0, -1.0, -1.0])
# control_frame="base_link" 用的符号组。**初值借自 teleop-vr 的实测
# `base_link_rotation_signs = [-1,-1,1]`，本台架尚未重测** —— 而且那边是在它自己的
# translation_remap 对应下测的，本文件的对应不同，故它只是起点、不是结论。
BASE_LINK_ROTATION_SIGNS = np.array([-1.0, -1.0, 1.0])
# 旋转增量读在哪个系：
#   "tool"（出厂，保持现状，便于回退）= 手柄自身系增量 → 末端局部系右乘
#       语义「手柄绕哪轴 → 末端绕末端自己哪轴」，工具逐轴复制手的轴。
#       ⚠ 只在工具系与手柄系处于**标定那一个特定关系**时才与 base_link 等价；
#         任何其它腕部 roll 都会把响应整体旋转该差值。
#   "base_link" = 增量搬到机器人基座系再施加 → 工具绕**世界轴**转，与臂姿态、
#       与手柄怎么拿都无关。**也让旋转这一半与平移那一半同在基座系**（平移本来就是
#       世界方向），消掉"两半不同源"的问题。
CONTROL_FRAME = "tool"
# 动作增量的平滑 = **一阶低通（EMA）**，三个后端共用同一个。
# 为什么不用箱式移动平均：箱式 10 帧群延迟 ≈4.5 帧（50~75ms @60Hz），是「跟手不够」
# 的主因，且快挥/急停的单帧位移进不了均值就被稀释掉。EMA 群延迟 ≈(1-α)/α 帧
# （α=0.5 → 1 帧）；常速输入下两者稳态相同（都收敛到同一个值），差别只在瞬态。
# α = **新样本的权重**：1.0 = 不平滑（最跟手也最抖），越小越稳。
# ⚠ α 必须 > 0：α=0 时状态永不更新（增量恒为 0），表现为「臂完全不动」而不是「很稳」。
SMOOTH_ALPHA = 0.85

# ---- 左右臂轴映射配置 ----
# 双臂侧装（水平伸出）与单臂正装（竖直）末端朝向差 90°，故平移映射需独立标定，
# 不复用单臂的 CTRL_TRANSLATION_REMAP（那是单臂正装基准）。
#
# 双臂侧装几何（实测 home）：两臂末端均朝世界 +x 水平伸出、+z 朝上。
# 平移语义 rel_t = translation_remap.T @ 手柄delta，即「列=机器人轴、行=手柄轴」。
# ⚠ 同样按**右手系**读（前方 = −z，见上面 CTRL_TRANSLATION_REMAP 的 W3C 原文）。
#   手柄右移(+x) → 世界 ∓y；手柄上移(+y) → 世界 +z；手柄后移(+z) → 世界 −x
#   （等价地 **手柄前推 −z → 世界 +x**，与"两臂朝 +x 水平伸出"的几何一致）。
# 镜像协作：左右臂仅在「左右」分量镜像（右臂 y 符号与左臂相反），前后/上下相同。
DUAL_TRANSLATION_REMAP_LEFT = np.array(
    [[0.0, -1.0, 0.0],
     [0.0, 0.0, 1.0],
     [-1.0, 0.0, 0.0]]
)
DUAL_TRANSLATION_REMAP_RIGHT = np.array(
    [[0.0, -1.0, 0.0],
     [0.0, 0.0, 1.0],
     [-1.0, 0.0, 0.0]]
)

SIDE_CONFIG = {
    "left": {
        "axis_remap": CTRL_AXIS_REMAP.copy(),
        "translation_remap": DUAL_TRANSLATION_REMAP_LEFT.copy(),
        "rotation_signs": ROTATION_AXIS_SIGNS.copy(),
    },
    "right": {
        "axis_remap": CTRL_AXIS_REMAP.copy(),
        "translation_remap": DUAL_TRANSLATION_REMAP_RIGHT.copy(),
        # 实测：滚转(X)/摆腕(Z) 分量取反；俯仰(Y)保持
        "rotation_signs": np.array([-1.0, 1.0, -1.0]),
    },
}

# 双臂正装（两只独立竖直臂，无立柱）轴映射：home 与映射都**复用单臂正装基准**（左臂），
# 右臂 home 走 MIRROR_SIGNS 镜像。两臂几何与单臂正装同朝向（base 的 Z 竖直向上），
# 故左右臂映射与单臂正装同值（不能复用 SIDE_CONFIG——那是水平侧装基准，朝向差 90°）。
# ⚠ 右臂镜像映射尚未上真机标定：双臂侧装实测右臂需 rotation_signs=[-1,1,-1]
#   （见 SIDE_CONFIG["right"]）。正装若真机手感不对，照 SIDE_CONFIG 的方式补一组符号。
UPRIGHT_CONFIG = {
    "left": {
        "axis_remap": CTRL_AXIS_REMAP.copy(),
        "translation_remap": CTRL_TRANSLATION_REMAP.copy(),
        "rotation_signs": ROTATION_AXIS_SIGNS.copy(),
    },
    "right": {
        "axis_remap": CTRL_AXIS_REMAP.copy(),
        "translation_remap": CTRL_TRANSLATION_REMAP.copy(),
        "rotation_signs": ROTATION_AXIS_SIGNS.copy(),
    },
}




def ema_smooth(values, state, alpha):
    """增量的一阶低通（EMA）平滑。`state` = 单元素 list，`None` 表示未初始化。

    对速度式 delta 映射的口味：相比箱式移动平均，群延迟小一个量级
    （≈(1-α)/α 帧），且对单帧尖峰（快挥/急停）的总位移不稀释 —— 几何级数收敛到原值，
    而箱式要等旧样本滑出窗口才补得回来。首次调用直接返回原值（无滞后）。

    `alpha` 是新样本权重（见 `SMOOTH_ALPHA`）：1.0 = 不平滑。
    """
    x = np.asarray(values, float)
    if state[0] is None:
        state[0] = x.copy()
    else:
        state[0] = alpha * x + (1.0 - alpha) * state[0]
    return state[0]


# 出厂末端偏移量（米，本体系）：默认直接控制七轴末端法兰 [0, 0, 0]。
# 沿 tip link 的 +z（工具轴朝外）。若装夹爪/灵巧手可配为如 [0, 0, 0.18]。
DEFAULT_TCP_OFFSET = (0.0, 0.0, 0.0)


def as_tcp_offset(value):
    """归一化末端偏移量（TCP）：None → 出厂默认 `DEFAULT_TCP_OFFSET`。

    语义 = 操作者实际控制的那个工具点，相对 tip link（单臂 ee_link / 双臂
    ee_frame_link）**在本体系下**的偏移。传 `[0, 0, 0]` = 直接控制法兰。
    """
    if value is None:
        value = DEFAULT_TCP_OFFSET
    offset = np.asarray(value, float)
    if offset.shape != (3,) or not np.all(np.isfinite(offset)):
        raise ValueError("tcp_offset 必须是 3 个有限数(米): %r" % (value,))
    return offset


def _positive(value, name):
    """正有限数校验（配置值从 YAML 来，这里是边界，所以校验放在这一层）。"""
    number = float(value)
    if not np.isfinite(number) or number <= 0.0:
        raise ValueError("%s 必须是正的有限数: %r" % (name, value))
    return number


class Tuning:
    """遥操输入端与求解器的可调量。默认值 = 模块常量，一字不改。

    `deadband_scale` 是四个死区阈值共用的乘数：1.0 = 出厂值；0.0 = 关掉死区
    （任何非零增量都能破冰、且不再冻结）。**关掉死区会把手柄追踪噪声原样放行**
    —— 它存在就是为了挡这个（见顶部常量注释），真机会抖。要降它先降噪声。

    四个死区原始阈值可以逐个覆写（`deadband_*_in/out`，配置文件里就是 deadband
    那一段）；**覆写值也要再乘 `deadband_scale`**，所以 scale=0 仍然能一键关掉整
    个死区，而不用把四个阈值都写 0。
    """

    __slots__ = ("deadband_t_in", "deadband_t_out", "deadband_r_in",
                 "deadband_r_out", "rest_confirm_frames", "smooth_alpha",
                 "resume_margin", "task_residual_tolerance", "manipulability_cost",
                 "control_frame", "base_link_rotation_signs", "tracking")

    def __init__(self, deadband_scale=1.0,
                 smooth_alpha=SMOOTH_ALPHA,
                 resume_margin=1.0, task_residual_tolerance=1e-3,
                 manipulability_cost=PINK_MANIP_COST, control_frame=CONTROL_FRAME,
                 base_link_rotation_signs=None,
                 deadband_t_in=None, deadband_t_out=None,
                 deadband_r_in=None, deadband_r_out=None,
                 rest_confirm_frames=REST_CONFIRM_FRAMES, tracking=None):
        scale = float(deadband_scale)
        if not np.isfinite(scale) or scale < 0.0:
            raise ValueError("deadband_scale 必须是非负有限数: %r" % (deadband_scale,))
        raw = {
            "deadband_t_in": (DELTA_DEADBAND_T_IN, deadband_t_in),
            "deadband_t_out": (DELTA_DEADBAND_T_OUT, deadband_t_out),
            "deadband_r_in": (DELTA_DEADBAND_R_IN, deadband_r_in),
            "deadband_r_out": (DELTA_DEADBAND_R_OUT, deadband_r_out),
        }
        for name, (factory, override) in raw.items():
            base = factory if override is None else _positive(override, name)
            setattr(self, name, base * scale)
        if int(rest_confirm_frames) < 1:
            raise ValueError("rest_confirm_frames 至少为 1: %r" % (rest_confirm_frames,))
        self.rest_confirm_frames = int(rest_confirm_frames)
        # ⚠ 上界 1.0 是「不平滑」，下界开区间是因为 α=0 会让状态永不更新 ——
        # 那不是「很稳」，是「臂完全不动」，必须在这里拦住而不是让操作者去猜。
        alpha = float(smooth_alpha)
        if not np.isfinite(alpha) or not 0.0 < alpha <= 1.0:
            raise ValueError("smooth_alpha 必须在 (0, 1] 内: %r"
                             "（1.0 = 不平滑；0 会让增量恒为 0，臂不动）"
                             % (smooth_alpha,))
        self.smooth_alpha = alpha
        if not np.isfinite(resume_margin) or resume_margin <= 0.0:
            raise ValueError("resume_margin 必须为正: %r" % (resume_margin,))
        self.resume_margin = float(resume_margin)
        if not np.isfinite(task_residual_tolerance) or task_residual_tolerance <= 0.0:
            raise ValueError("task_residual_tolerance 必须为正: %r"
                             % (task_residual_tolerance,))
        self.task_residual_tolerance = float(task_residual_tolerance)
        # 仅 pin 后端用：操纵度任务权重。0 = 关闭（出厂）。调大会与 FrameTask
        # 抢优先级（牺牲工具精度）换取"不进奇异区"，按台架量着调。
        cost = float(manipulability_cost)
        if not np.isfinite(cost) or cost < 0.0:
            raise ValueError("manipulability_cost 必须是非负有限数: %r"
                             % (manipulability_cost,))
        self.manipulability_cost = cost
        # 旋转增量读在哪个系。默认 "tool" = 现状，便于回退（改了才需要重标符号）。
        if control_frame not in ("tool", "base_link"):
            raise ValueError("control_frame 只能是 'tool' 或 'base_link': %r"
                             % (control_frame,))
        self.control_frame = control_frame
        signs = (BASE_LINK_ROTATION_SIGNS if base_link_rotation_signs is None
                 else base_link_rotation_signs)
        signs = np.asarray(signs, float)
        if signs.shape != (3,) or not np.all(np.isfinite(signs)):
            raise ValueError("base_link_rotation_signs 必须是 3 个有限数: %r"
                             % (base_link_rotation_signs,))
        self.base_link_rotation_signs = signs
        # Keep these in lockstep with teleop/config/teleop.yaml; test_config.py
        # enforces the match.
        defaults = dict(input_timeout=1.0, feedback_timeout=0.5,
                        max_position_error=0.3, max_rotation_error=1.5,
                        stall_timeout=5.0, position_tolerance=0.001,
                        rotation_tolerance=0.01, max_joint_velocity=2.0,
                        max_joint_acceleration=12.0, max_linear_velocity=1.5,
                        max_angular_velocity=8.0, hand_jump_position=0.15,
                        hand_jump_rotation=0.8)
        for key, value in (tracking or {}).items():
            if key not in defaults:
                raise ValueError("unknown tracking setting: %s" % key)
            defaults[key] = value
        self.tracking = {key: _positive(value, "tracking." + key)
                         for key, value in defaults.items()}


class SimConfig:
    """仿真/几何/限幅层的可调量。默认值 = 模块常量，一字不改。

    与 `Tuning` 的分工：`Tuning` 管**遥操输入端**（死区、平滑、坐标系、滞回），
    `SimConfig` 管**仿真与运动学**（频率、限幅、home、轴映射、收敛容差、两个 IK
    后端的求解器参数）。配置文件 `config/teleop.yaml` 的两段就按这个划分。

    轴映射逐臂取：单臂用 `axis_remap` / `translation_remap` / `rotation_signs`；
    双臂用 `dual_*` 那四条，**不复用单臂的**（两者末端朝向差 90°，映射不同）。
    这些都只在标定时才动 —— 它们是「对着真机量出来的方向对应关系」，改错不会
    报错，只会让手和臂的对应错掉。
    """

    __slots__ = ("freq", "max_joint_vel", "max_step_rad", "q_lowpass_tau",
                 "ik_pos_tol", "ik_rot_tol", "near_singular_sigma",
                 "q_home", "single_q_home", "mirror_signs",
                 "axis_remap", "translation_remap", "rotation_signs",
                 "base_link_rotation_signs",
                 "dual_translation_remap_left", "dual_translation_remap_right",
                 "dual_rotation_signs_left", "dual_rotation_signs_right",
                 "upright_translation_remap_left", "upright_translation_remap_right",
                 "upright_rotation_signs_left", "upright_rotation_signs_right",
                 "pin_solver", "pin_position_cost", "pin_orientation_cost",
                 "pin_lm_damping", "pin_posture_cost", "pin_velocity_limit",
                 "pin_manip_rate", "pin_dt", "pin_max_iters", "pin_err_tol",
                 "rank_tolerance")

    def __init__(self, freq=FREQ, max_joint_vel=MAX_JOINT_VEL,
                 max_step_rad=MAX_STEP_RAD, q_lowpass_tau=Q_LOWPASS_TAU,
                 ik_pos_tol=IK_POS_TOL, ik_rot_tol=IK_ROT_TOL,
                 near_singular_sigma=NEAR_SINGULAR_SIGMA,
                 q_home=None, single_q_home=None, mirror_signs=None,
                 axis_remap=None, translation_remap=None, rotation_signs=None,
                 base_link_rotation_signs=None,
                 dual_translation_remap_left=None, dual_translation_remap_right=None,
                 dual_rotation_signs_left=None, dual_rotation_signs_right=None,
                 upright_translation_remap_left=None,
                 upright_translation_remap_right=None,
                 upright_rotation_signs_left=None, upright_rotation_signs_right=None,
                 pin_solver="quadprog", pin_position_cost=1.0,
                 pin_orientation_cost=1.0, pin_lm_damping=PINK_LM_DAMPING,
                 pin_posture_cost=PINK_POSTURE_COST, pin_velocity_limit=1.0,
                 pin_manip_rate=PINK_MANIP_RATE, pin_dt=PINK_DT,
                 pin_max_iters=PINK_MAX_ITERS, pin_err_tol=PINK_ERR_TOL,
                 rank_tolerance=1e-9):
        self.freq = _positive(freq, "freq")
        # ⚠ max_joint_vel 允许 0 会退化成「完全不能动」，所以只校验有限非负；
        #   要「关掉速度这一界」请给一个够大的数（如 99.0），见常量区注释。
        # ⚠ 只被 --ik exact 用（速度盒的左半；右半是「行程/dt」）。metavr 没有这个量，
        #   所以它在配置文件里属于 `exact_ik:` 而不是全局的 `sim:`。
        self.max_joint_vel = _nonneg(max_joint_vel, "max_joint_vel")
        self.max_step_rad = _positive(max_step_rad, "max_step_rad")
        # 0 = 关掉低通滤波，所以允许非负
        self.q_lowpass_tau = _nonneg(q_lowpass_tau, "q_lowpass_tau")
        self.ik_pos_tol = _positive(ik_pos_tol, "ik_pos_tol")
        self.ik_rot_tol = _positive(ik_rot_tol, "ik_rot_tol")
        self.near_singular_sigma = _positive(near_singular_sigma, "near_singular_sigma")

        self.q_home = _vector7(Q_HOME if q_home is None else q_home, "q_home")
        self.single_q_home = _vector7(
            SINGLE_Q_HOME if single_q_home is None else single_q_home, "single_q_home")
        self.mirror_signs = _vector7(
            MIRROR_SIGNS if mirror_signs is None else mirror_signs, "mirror_signs")

        self.axis_remap = _matrix3(
            CTRL_AXIS_REMAP if axis_remap is None else axis_remap, "axis_remap")
        self.translation_remap = _matrix3(
            CTRL_TRANSLATION_REMAP if translation_remap is None else translation_remap,
            "translation_remap")
        self.rotation_signs = _vector3(
            ROTATION_AXIS_SIGNS if rotation_signs is None else rotation_signs,
            "rotation_signs")
        self.base_link_rotation_signs = _vector3(
            BASE_LINK_ROTATION_SIGNS if base_link_rotation_signs is None
            else base_link_rotation_signs, "base_link_rotation_signs")
        self.dual_translation_remap_left = _matrix3(
            DUAL_TRANSLATION_REMAP_LEFT if dual_translation_remap_left is None
            else dual_translation_remap_left, "dual_translation_remap_left")
        self.dual_translation_remap_right = _matrix3(
            DUAL_TRANSLATION_REMAP_RIGHT if dual_translation_remap_right is None
            else dual_translation_remap_right, "dual_translation_remap_right")
        self.dual_rotation_signs_left = _vector3(
            SIDE_CONFIG["left"]["rotation_signs"] if dual_rotation_signs_left is None
            else dual_rotation_signs_left, "dual_rotation_signs_left")
        self.dual_rotation_signs_right = _vector3(
            SIDE_CONFIG["right"]["rotation_signs"] if dual_rotation_signs_right is None
            else dual_rotation_signs_right, "dual_rotation_signs_right")
        # 双臂正装：出厂＝单臂正装基准（末端同为竖直朝向），见 UPRIGHT_CONFIG 的注释。
        # 单独成字段是因为它**尚未上真机标定** —— 手感不对时按臂补一组符号即可，
        # 不必去改单臂正装那份（改错了会连带影响单臂）。
        self.upright_translation_remap_left = _matrix3(
            UPRIGHT_CONFIG["left"]["translation_remap"]
            if upright_translation_remap_left is None
            else upright_translation_remap_left, "upright_translation_remap_left")
        self.upright_translation_remap_right = _matrix3(
            UPRIGHT_CONFIG["right"]["translation_remap"]
            if upright_translation_remap_right is None
            else upright_translation_remap_right, "upright_translation_remap_right")
        self.upright_rotation_signs_left = _vector3(
            UPRIGHT_CONFIG["left"]["rotation_signs"]
            if upright_rotation_signs_left is None
            else upright_rotation_signs_left, "upright_rotation_signs_left")
        self.upright_rotation_signs_right = _vector3(
            UPRIGHT_CONFIG["right"]["rotation_signs"]
            if upright_rotation_signs_right is None
            else upright_rotation_signs_right, "upright_rotation_signs_right")

        self.pin_solver = str(pin_solver)
        self.pin_position_cost = _nonneg(pin_position_cost, "pin_position_cost")
        self.pin_orientation_cost = _nonneg(pin_orientation_cost, "pin_orientation_cost")
        self.pin_lm_damping = _nonneg(pin_lm_damping, "pin_lm_damping")
        self.pin_posture_cost = _nonneg(pin_posture_cost, "pin_posture_cost")
        self.pin_velocity_limit = _positive(pin_velocity_limit, "pin_velocity_limit")
        self.pin_manip_rate = _positive(pin_manip_rate, "pin_manip_rate")
        self.pin_dt = _positive(pin_dt, "pin_dt")
        self.pin_max_iters = int(pin_max_iters)
        if self.pin_max_iters < 1:
            raise ValueError("pin_max_iters 至少为 1: %r" % (pin_max_iters,))
        self.pin_err_tol = _positive(pin_err_tol, "pin_err_tol")
        self.rank_tolerance = _positive(rank_tolerance, "rank_tolerance")

    def arm_mapping(self, name, dual=False, upright=False, side=False):
        """某只臂的轴映射 dict（`Arm` 的 `config` 参数）。

        三套（不是两套）标定过的方向对应关系，按**末端朝向**分：
          双臂侧装（dual，非 upright）水平伸出 ⇒ 用 `dual_*` 那组
          双臂正装（dual + upright） 竖直 ⇒ 与单臂正装同朝向，用 `dual_upright_*`
                                              （出厂＝单臂正装基准）
          单臂                                              side=正装 / 不 side=侧装，
                                              其中侧装等价于「双臂侧装里的那只左臂」
        **不能混用**：侧装与正装末端朝向差 90°，一个朝向标定出来的矩阵放到另一个上是错的。
        """
        if dual and not upright:
            if name == "left":
                return {"axis_remap": self.axis_remap.copy(),
                        "translation_remap": self.dual_translation_remap_left.copy(),
                        "rotation_signs": self.dual_rotation_signs_left.copy()}
            return {"axis_remap": self.axis_remap.copy(),
                    "translation_remap": self.dual_translation_remap_right.copy(),
                    "rotation_signs": self.dual_rotation_signs_right.copy()}
        if dual and upright:
            if name == "left":
                return {"axis_remap": self.axis_remap.copy(),
                        "translation_remap": self.upright_translation_remap_left.copy(),
                        "rotation_signs": self.upright_rotation_signs_left.copy()}
            return {"axis_remap": self.axis_remap.copy(),
                    "translation_remap": self.upright_translation_remap_right.copy(),
                    "rotation_signs": self.upright_rotation_signs_right.copy()}
        if side:
            # 单臂侧装 = 双臂侧装里的那只左臂（键盘/手柄槽位也是 left）
            return {"axis_remap": self.axis_remap.copy(),
                    "translation_remap": self.dual_translation_remap_left.copy(),
                    "rotation_signs": self.dual_rotation_signs_left.copy()}
        return {"axis_remap": self.axis_remap.copy(),
                "translation_remap": self.translation_remap.copy(),
                "rotation_signs": self.rotation_signs.copy()}


def _nonneg(value, name):
    number = float(value)
    if not np.isfinite(number) or number < 0.0:
        raise ValueError("%s 必须是非负有限数: %r" % (name, value))
    return number


def _vector3(value, name):
    vector = np.asarray(value, float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError("%s 必须是 3 个有限数: %r" % (name, value))
    return vector


def _vector7(value, name):
    vector = np.asarray(value, float)
    if vector.shape != (7,) or not np.all(np.isfinite(vector)):
        raise ValueError("%s 必须是 7 个有限数: %r" % (name, value))
    return vector


def _matrix3(value, name):
    matrix = np.asarray(value, float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("%s 必须是 3×3 矩阵: %r" % (name, value))
    return matrix


def tuning_from_config(data):
    """`config/teleop.yaml` 的 `tuning` 段 → `Tuning`。

    缺键就退回出厂常量（`get` 的 default 就是它），所以配置文件里少写一个键不会
    改变行为，只会少一个可调点。服务端与 `teleop_sim.main()` 共用这一份映射，
    避免两处各写一遍默认值而漂移。
    """
    from .config import get
    return Tuning(
        deadband_scale=get(data, "tuning.deadband_scale", 1.0),
        smooth_alpha=get(data, "tuning.smooth_alpha", SMOOTH_ALPHA),
        resume_margin=get(data, "tuning.resume_margin", 1.0),
        task_residual_tolerance=get(data, "tuning.task_residual_tolerance", 1e-3),
        manipulability_cost=get(data, "tuning.manip_weight", 0.0),
        control_frame=get(data, "tuning.control_frame", CONTROL_FRAME),
        base_link_rotation_signs=get(data, "tuning.base_link_signs", None),
        deadband_t_in=get(data, "tuning.deadband.t_in", None),
        deadband_t_out=get(data, "tuning.deadband.t_out", None),
        deadband_r_in=get(data, "tuning.deadband.r_in", None),
        deadband_r_out=get(data, "tuning.deadband.r_out", None),
        tracking=get(data, "tracking", {}),
        rest_confirm_frames=get(data, "tuning.rest_confirm_frames",
                                REST_CONFIRM_FRAMES),
    )


def sim_config_from_config(data):
    """`config/teleop.yaml` 的 `sim` / `mapping` / `pin_ik` / `exact_ik` 段 → `SimConfig`。"""
    from .config import get
    return SimConfig(
        freq=get(data, "sim.freq", FREQ),
        max_joint_vel=get(data, "exact_ik.max_joint_vel", MAX_JOINT_VEL),
        max_step_rad=get(data, "sim.max_step_rad", MAX_STEP_RAD),
        q_lowpass_tau=get(data, "sim.q_lowpass_tau", Q_LOWPASS_TAU),
        ik_pos_tol=get(data, "sim.ik_pos_tol", IK_POS_TOL),
        ik_rot_tol=get(data, "sim.ik_rot_tol", IK_ROT_TOL),
        near_singular_sigma=get(data, "sim.near_singular_sigma", NEAR_SINGULAR_SIGMA),
        q_home=get(data, "sim.q_home", None),
        single_q_home=get(data, "sim.single_q_home", None),
        mirror_signs=get(data, "sim.mirror_signs", None),
        axis_remap=get(data, "mapping.axis_remap", None),
        translation_remap=get(data, "mapping.translation_remap", None),
        rotation_signs=get(data, "mapping.rotation_signs", None),
        base_link_rotation_signs=get(data, "mapping.base_link_rotation_signs", None),
        dual_translation_remap_left=get(
            data, "mapping.dual_translation_remap_left", None),
        dual_translation_remap_right=get(
            data, "mapping.dual_translation_remap_right", None),
        dual_rotation_signs_left=get(data, "mapping.dual_rotation_signs_left", None),
        dual_rotation_signs_right=get(data, "mapping.dual_rotation_signs_right", None),
        upright_translation_remap_left=get(
            data, "mapping.upright_translation_remap_left", None),
        upright_translation_remap_right=get(
            data, "mapping.upright_translation_remap_right", None),
        upright_rotation_signs_left=get(
            data, "mapping.upright_rotation_signs_left", None),
        upright_rotation_signs_right=get(
            data, "mapping.upright_rotation_signs_right", None),
        pin_solver=get(data, "pin_ik.solver", "quadprog"),
        pin_position_cost=get(data, "pin_ik.position_cost", 1.0),
        pin_orientation_cost=get(data, "pin_ik.orientation_cost", 1.0),
        pin_lm_damping=get(data, "pin_ik.lm_damping", PINK_LM_DAMPING),
        pin_posture_cost=get(data, "pin_ik.posture_cost", PINK_POSTURE_COST),
        pin_velocity_limit=get(data, "pin_ik.velocity_limit", 1.0),
        pin_manip_rate=get(data, "pin_ik.manip_rate", PINK_MANIP_RATE),
        pin_dt=get(data, "pin_ik.dt", PINK_DT),
        pin_max_iters=get(data, "pin_ik.max_iters", PINK_MAX_ITERS),
        pin_err_tol=get(data, "pin_ik.err_tol", PINK_ERR_TOL),
        rank_tolerance=get(data, "exact_ik.rank_tolerance", 1e-9),
    )


def _load_urdf_with_mesh(cid, urdf_path, pkg_prefix, mesh_dir):
    """加载 URDF：把 package://<pkg_prefix> 路径替换为本地绝对路径后 loadURDF。

    pybullet 的 loadURDF 对字符串参数不解析绝对路径 mesh，故写入临时文件加载。
    """
    import tempfile
    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_str = f.read().replace(pkg_prefix, mesh_dir + "/")
    tmp_path = os.path.join(tempfile.gettempdir(),
                            os.path.basename(urdf_path) + ".tmp.urdf")
    with open(tmp_path, "w", encoding="utf-8") as f:
        f.write(urdf_str)
    return p.loadURDF(tmp_path, useFixedBase=True, physicsClientId=cid)


class ArmVisualMarker:
    """PyBullet 3D visual markers for current TCP and Target TCP.

    Visualizes:
      - End-Effector (TCP): Solid Green sphere + RGB coordinate triad (X=Red, Y=Green, Z=Blue) + Label
      - Target TCP: Semi-transparent Orange sphere + RGB coordinate triad + Label
      - Tracking error: Connection line between current TCP and Target TCP
    """

    # GUI 调试绘制是同步 IPC，且每次都会带着整臂场景重渲染（实测双臂 URDF 下
    # 一次重画 ~50 ms，其中文字网格重建更贵）。若随控制周期（60 Hz）重画，会把
    # 控制环拖到 ~5 Hz、饿死固件 100 ms 跟随看门狗。故几何限到 5 Hz、文字限到
    # 1 Hz，且两条臂的文字刷新错开相位（见 __init__）：单拍 GUI 开销 ~50 ms，
    # 稳态控制周期仍 ~20 ms；控制/下发路径完全不受影响。
    GEOM_INTERVAL = 0.2
    TEXT_INTERVAL = 1.0

    def __init__(self, cid, name):
        self._cid = cid
        self.name = name
        self._is_gui = False
        try:
            info = p.getConnectionInfo(physicsClientId=cid)
            self._is_gui = (info.get("connectionMethod") == p.GUI)
        except Exception:
            self._is_gui = False

        self._mb_ee = None
        self._mb_tgt = None
        self._lines_ee = [-1, -1, -1]
        self._lines_tgt = [-1, -1, -1]
        self._line_err = -1
        self._text_ee = -1
        self._text_tgt = -1
        self._last_geom = -1e9
        # 左右臂错开半个文字周期：两条臂同时重建文字网格会叠成 ~90 ms 的单拍卡顿，
        # 逼近固件 100 ms 跟随看门狗。右臂延迟首画，此后各按自己的时间滚动。
        self._last_text = (-1e9 if name.endswith("left")
                           else time.monotonic() - self.TEXT_INTERVAL * 0.5)

    def _ensure_visual_bodies(self):
        if not self._is_gui or self._mb_ee is not None:
            return
        try:
            vs_ee = p.createVisualShape(
                p.GEOM_SPHERE, radius=0.012,
                rgbaColor=[0.0, 0.9, 0.2, 0.9],
                physicsClientId=self._cid)
            self._mb_ee = p.createMultiBody(
                baseVisualShapeIndex=vs_ee,
                basePosition=[0, 0, -10],
                physicsClientId=self._cid)

            vs_tgt = p.createVisualShape(
                p.GEOM_SPHERE, radius=0.015,
                rgbaColor=[1.0, 0.55, 0.0, 0.8],
                physicsClientId=self._cid)
            self._mb_tgt = p.createMultiBody(
                baseVisualShapeIndex=vs_tgt,
                basePosition=[0, 0, -10],
                physicsClientId=self._cid)
        except Exception:
            pass

    def update(self, cur_tcp, cur_R, tgt_tcp=None, tgt_R=None, active=False, force=False):
        if not self._is_gui:
            return
        now = time.monotonic()
        draw_geom = force or now - self._last_geom >= self.GEOM_INTERVAL
        draw_text = force or now - self._last_text >= self.TEXT_INTERVAL
        if not (draw_geom or draw_text):
            return
        if draw_geom:
            self._last_geom = now
        if draw_text:
            self._last_text = now
        self._ensure_visual_bodies()
        try:
            cur_p = np.asarray(cur_tcp, dtype=float)
            cur_rot = np.asarray(cur_R, dtype=float)
            cur_quat = R.from_matrix(cur_rot).as_quat()

            # 1. End-Effector (TCP) sphere
            if draw_geom and self._mb_ee is not None:
                p.resetBasePositionAndOrientation(
                    self._mb_ee, cur_p.tolist(), cur_quat.tolist(),
                    physicsClientId=self._cid)

            # 2. End-Effector RGB axes (X=Red, Y=Green, Z=Blue)
            axis_len = 0.07
            colors_ee = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.3, 1.0]]
            if draw_geom:
                for i in range(3):
                    end_pt = cur_p + cur_rot[:, i] * axis_len
                    self._lines_ee[i] = p.addUserDebugLine(
                        cur_p.tolist(), end_pt.tolist(), colors_ee[i], lineWidth=3.5,
                        replaceItemUniqueId=self._lines_ee[i], physicsClientId=self._cid)

            # 3. End-Effector text label
            if draw_text:
                self._text_ee = p.addUserDebugText(
                    "[%s] TCP" % self.name, (cur_p + np.array([0, 0, 0.025])).tolist(),
                    textColorRGB=[0.0, 0.9, 0.2], textSize=0.85,
                    replaceItemUniqueId=self._text_ee, physicsClientId=self._cid)

            # 4. Target Marker (if target given)
            if tgt_tcp is not None and tgt_R is not None:
                tgt_p = np.asarray(tgt_tcp, dtype=float)
                tgt_rot = np.asarray(tgt_R, dtype=float)
                tgt_quat = R.from_matrix(tgt_rot).as_quat()

                if draw_geom and self._mb_tgt is not None:
                    p.resetBasePositionAndOrientation(
                        self._mb_tgt, tgt_p.tolist(), tgt_quat.tolist(),
                        physicsClientId=self._cid)

                colors_tgt = [[1.0, 0.4, 0.1], [0.4, 1.0, 0.1], [0.1, 0.7, 1.0]]
                if draw_geom:
                    for i in range(3):
                        end_tgt = tgt_p + tgt_rot[:, i] * axis_len
                        self._lines_tgt[i] = p.addUserDebugLine(
                            tgt_p.tolist(), end_tgt.tolist(), colors_tgt[i], lineWidth=2.5,
                            replaceItemUniqueId=self._lines_tgt[i], physicsClientId=self._cid)

                if draw_text:
                    lbl = "[%s] Target" % self.name if active else "[%s] Target (Hold)" % self.name
                    self._text_tgt = p.addUserDebugText(
                        lbl, (tgt_p + np.array([0, 0, 0.025])).tolist(),
                        textColorRGB=[1.0, 0.6, 0.0], textSize=0.85,
                        replaceItemUniqueId=self._text_tgt, physicsClientId=self._cid)

                # Tracking error line
                if draw_geom:
                    dist = float(np.linalg.norm(tgt_p - cur_p))
                    if dist > 0.002:
                        self._line_err = p.addUserDebugLine(
                            cur_p.tolist(), tgt_p.tolist(), [1.0, 0.85, 0.0], lineWidth=2.0,
                            replaceItemUniqueId=self._line_err, physicsClientId=self._cid)
                    else:
                        self._line_err = p.addUserDebugLine(
                            cur_p.tolist(), cur_p.tolist(), [1.0, 0.85, 0.0], lineWidth=0.1,
                            replaceItemUniqueId=self._line_err, physicsClientId=self._cid)
            elif draw_geom and self._mb_tgt is not None:
                p.resetBasePositionAndOrientation(
                    self._mb_tgt, [0, 0, -10], [0, 0, 0, 1], physicsClientId=self._cid)
        except Exception:
            pass

    def close(self):
        if not self._is_gui:
            return
        try:
            if self._mb_ee is not None:
                p.removeBody(self._mb_ee, physicsClientId=self._cid)
                self._mb_ee = None
            if self._mb_tgt is not None:
                p.removeBody(self._mb_tgt, physicsClientId=self._cid)
                self._mb_tgt = None
            for lid in self._lines_ee + self._lines_tgt + [self._line_err, self._text_ee, self._text_tgt]:
                if lid != -1:
                    p.removeUserDebugItem(lid, physicsClientId=self._cid)
            self._lines_ee = [-1, -1, -1]
            self._lines_tgt = [-1, -1, -1]
            self._line_err = -1
            self._text_ee = -1
            self._text_tgt = -1
        except Exception:
            pass


class Arm:
    """单臂：pybullet 模型 + 遥操状态 + IK。

    从原 LiteArmTeleopSim 抽出，持有一只臂的全部状态与轴映射常量。
    """

    def __init__(self, cid, robot_id, name, config, prefix="", ik_index=None,
                 all_rev_ids=None, joint_stem="Joint", ee_link_name="ee_frame_link",
                 home=Q_HOME, ik_backend="pybullet", pin_urdf=None, pin_ee_frame=None,
                 all_joint_names=None, tcp_offset=None, tuning=None, probe=None,
                 sim_config=None):
        self._cid = cid
        self._robot = robot_id
        self.name = name
        self._prefix = prefix  # 关节/link 名前缀（双臂 left_/right_；单臂为空）
        self._home = np.asarray(home, float)  # 本臂初始构型（双臂左右不同）

        # 轴映射（左右臂各自独立，见 SIDE_CONFIG）
        self._axis_remap = config["axis_remap"]
        self._translation_remap = config["translation_remap"]
        self._rotation_signs = config["rotation_signs"]
        self._tcp_offset = as_tcp_offset(tcp_offset)
        self._tuning = tuning if tuning is not None else Tuning()
        # 仿真/限幅/容忍层的可调量。None = 出厂常量（逐位等于改动前）。
        self._cfg = sim_config if sim_config is not None else SimConfig()
        # 数据探针（诊断用）：每帧被调用一次，入参是本帧各阶段的 dict。
        # None = 关闭（默认），对行为零影响，只有一次 `is not None` 判断的开销。
        self._probe = probe
        self._last_ik_info = {}
        # 旋转增量读在哪个系 + 该系自己的那组符号。
        # control_frame="tool"（出厂）取 config 里的逐臂符号 ⇒ **与改动前逐位相同**，
        # 所以这是可回退的开关而不是行为变更。base_link 需要另一组符号（换模式必须
        # 换符号：两个系读同样三个数、在一边对的镜像在另一边就是错的）。
        self._control_frame = self._tuning.control_frame
        self._rot_signs = (np.asarray(self._tuning.base_link_rotation_signs, float)
                           if self._control_frame == "base_link"
                           else config["rotation_signs"])

        n_j = p.getNumJoints(self._robot, physicsClientId=self._cid)
        # 7 个可动旋转关节（<prefix><joint_stem>1..7）；精确匹配关节名，避免串臂
        joint_names = {prefix + joint_stem + str(k): k - 1 for k in range(1, 8)}
        joint_id_by_name = {}
        for i in range(n_j):
            jname = p.getJointInfo(self._robot, i, physicsClientId=self._cid)[1].decode()
            if jname in joint_names:
                joint_id_by_name[jname] = i
        self._joint_ids = [joint_id_by_name[prefix + joint_stem + str(k)]
                           for k in range(1, 8)]
        # 限位取自 URDF（几何唯一真源）
        self._lower = np.array([p.getJointInfo(self._robot, i, physicsClientId=self._cid)[8]
                                for i in self._joint_ids])
        self._upper = np.array([p.getJointInfo(self._robot, i, physicsClientId=self._cid)[9]
                                for i in self._joint_ids])
        # 末端 link：ee_joint（fixed）的子 link <prefix><ee_link_name>
        self._ee_link = next(
            i for i in range(n_j)
            if p.getJointInfo(self._robot, i, physicsClientId=self._cid)[12].decode()
            == prefix + ee_link_name
        )

        # 双臂共用 robot_id：IK 输入/输出是 14 维（左 0..6 + 右 7..13），
        # 只含可动关节（fixed 关节被 pybullet 跳过）。
        # ik_index = 本臂在 14 维向量里的位置；单臂模式 = [0..6]。
        if ik_index is None:
            self._ik_index = list(range(len(self._joint_ids)))
        else:
            self._ik_index = list(ik_index)
        # 整个 robot 的可动关节 pybullet 索引（双臂=14，单臂=7）
        if all_rev_ids is None:
            self._all_rev_ids = self._joint_ids
        else:
            self._all_rev_ids = all_rev_ids

        # ---- 遥操状态 ----
        self._active = False
        self._a_held = False
        self._b_held = False
        self._homing = False
        self._homing_start_q = None
        self._homing_start_t = 0.0
        self._homing_duration = 0.0
        self._ctrl_prev_H = None
        self._smooth = [None]      # 增量 EMA 平滑状态（三后端共用）
        self._last_singular_warn = 0.0
        self._last_step_t = None   # 上次 _step_teleop 时刻（pin/exact 需要）
        self._resting = True       # 静止死区状态（True=判为静止，增量置 0）
        self._rest_confirm = 0     # 连续低于进入阈值 IN 的帧计数

        # ---- IK 后端 ----
        # "pybullet" = calculateInverseKinematics（出厂，= metavr 的默认）；
        # "pin" = pinocchio + pink，metavr 的策略（逐拍积分到收敛，见 pin_ik.py）；
        # "exact" = 优先精确任务速度，不可行时使用受限阻尼跟随。
        self._ik_backend = ik_backend
        # 所有后端共享固定锚点映射；求解器不再决定手柄控制语义。
        self._target = AnchoredPoseTarget(self._tuning, self._translation_remap,
                                         self._axis_remap, self._rot_signs,
                                         self._control_frame)
        self._pin = None
        self._exact_held = False   # exact 后端的 hold 滞回状态
        if ik_backend == "pin":
            from .pin_ik import PinIKSolver
            self._pin = PinIKSolver(
                pin_urdf, pin_ee_frame, all_joint_names, self._ik_index,
                solver=self._cfg.pin_solver,
                position_cost=self._cfg.pin_position_cost,
                orientation_cost=self._cfg.pin_orientation_cost,
                lm_damping=self._cfg.pin_lm_damping,
                posture_cost=self._cfg.pin_posture_cost,
                velocity_limit=self._cfg.pin_velocity_limit,
                manipulability_cost=self._tuning.manipulability_cost,
                manip_rate=self._cfg.pin_manip_rate,
                dt=self._cfg.pin_dt,
                max_iters=self._cfg.pin_max_iters,
                err_tol=self._cfg.pin_err_tol)

        self._marker = ArmVisualMarker(self._cid, self.name)
        self.reset()

    # ---------- 状态 ----------
    def _get_q(self):
        return np.array([p.getJointState(self._robot, j, physicsClientId=self._cid)[0]
                         for j in self._joint_ids])

    def _get_q_all(self):
        """整个 robot 可动关节（双臂=14，单臂=7）的当前角，pybullet 序。"""
        return np.array([p.getJointState(self._robot, j, physicsClientId=self._cid)[0]
                         for j in self._all_rev_ids])

    def _set_q(self, q):
        for j, qv in zip(self._joint_ids, q):
            p.resetJointState(self._robot, j, float(qv), physicsClientId=self._cid)

    def _eef_pose(self):
        """末端世界位姿 (pos[3], quat_xyzw[4])。"""
        ls = p.getLinkState(self._robot, self._ee_link,
                            computeForwardKinematics=True,
                            physicsClientId=self._cid)
        return np.asarray(ls[4], float), np.asarray(ls[5], float)

    def _fk(self, q):
        self._set_q(q)
        return self._eef_pose()

    def _get_robot_state_vector(self):
        pos, quat = self._eef_pose()
        return np.concatenate([[0.0, 0.0], pos, quat])

    def reset(self):
        self._set_q(self._home)
        self._active = False
        self._a_held = False
        self._b_held = False
        self._homing = False
        self._homing_start_q = None
        self._ctrl_prev_H = None
        self._smooth = [None]
        self._last_step_t = None
        self._resting = True
        self._rest_confirm = 0
        self._exact_held = False
        self._hold_reason = None
        self._last_input_t = None
        self._last_hand = None
        self._command_q = self._get_q().copy()
        self._command_velocity = np.zeros(7)
        self._progress_t = None
        self._best_error = float("inf")
        self.update_marker()
        return self._get_robot_state_vector()

    def update_marker(self):
        if getattr(self, "_marker", None) is None:
            return
        # 未参与控制的臂（如未连接的另一侧）不持续绘制：GUI 绘制是同步 IPC，
        # 每条臂每次 ~20 ms，白画会让控制环掉速（进而饿死真机跟随看门狗）。
        # 静置臂只画一次作为位置参考；停止跟随那一次强制重画到实际位姿。
        live = bool(self._active or getattr(self, "_homing", False))
        was_live = getattr(self, "_marker_live", False)
        if not live and not was_live:
            if getattr(self, "_marker_idle_drawn", False):
                return
            self._marker_idle_drawn = True
        self._marker_live = live
        cur_pos, cur_quat = self._eef_pose()
        cur_R = R.from_quat(cur_quat).as_matrix()
        cur_tcp = cur_pos + cur_R @ self._tcp_offset
        if getattr(self, "_homing", False):
            tgt_tcp, tgt_R = cur_tcp, cur_R
            self._marker.update(cur_tcp, cur_R, tgt_tcp, tgt_R, active=True)
            return
        if getattr(self, "_target", None) is not None and self._target.started and self._active:
            tgt_tcp, tgt_R = self._target.raw_target()
        else:
            tgt_tcp, tgt_R = cur_tcp, cur_R
        # 停止跟随那一次强制重画，把标记从旧目标位置收回到当前 TCP。
        self._marker.update(cur_tcp, cur_R, tgt_tcp, tgt_R, active=self._active,
                            force=not live)

    def close(self):
        if getattr(self, "_marker", None) is not None:
            self._marker.close()

    # ---------- 运动学 ----------
    def ik(self, pos_d, R_d, q_seed=None, info=None):
        q_cur = self._get_q()
        seed = q_cur if q_seed is None else np.asarray(q_seed, float)

        # 构造全可动关节（双臂=14）维参数：本臂自由，其余关节 lo=up=当前值 锁死。
        # pybullet 的 calculateInverseKinematics 只对可动关节返回解（跳过 fixed），
        # 故参数/输出维度 = len(self._all_rev_ids)。
        n_all = len(self._all_rev_ids)
        cur_all = np.array([p.getJointState(self._robot, j, physicsClientId=self._cid)[0]
                            for j in self._all_rev_ids])
        lo = [0.0] * n_all
        up = [0.0] * n_all
        ranges = [0.0] * n_all
        rest = [0.0] * n_all
        for k, j in enumerate(self._all_rev_ids):
            if k in self._ik_index:
                # ⚠ 别叫 `info` —— 它会遮蔽本方法的 `info` 形参（诊断回填用）。
                jinfo = p.getJointInfo(self._robot, j, physicsClientId=self._cid)
                lo[k] = jinfo[8]
                up[k] = jinfo[9]
                ranges[k] = up[k] - lo[k]
                rest[k] = cur_all[k]
            else:
                # 锁死其它关节（另一臂）：上下限都 = 当前值
                lo[k] = cur_all[k]
                up[k] = cur_all[k]
                ranges[k] = 0.0
                rest[k] = cur_all[k]

        quat_d = R.from_matrix(R_d).as_quat()
        q_all = np.asarray(p.calculateInverseKinematics(
            self._robot, self._ee_link,
            targetPosition=list(np.asarray(pos_d, float)),
            targetOrientation=list(quat_d),
            lowerLimits=lo,
            upperLimits=up,
            jointRanges=ranges,
            restPoses=rest,
            jointDamping=[0.01] * n_all,
            maxNumIterations=200,
            residualThreshold=1e-4,
            physicsClientId=self._cid), float)
        # 取本臂的 7 维解
        q_tgt = q_all[self._ik_index]

        pos, quat = self._fk(q_tgt)
        self._set_q(q_cur)
        err_pos = np.linalg.norm(pos - np.asarray(pos_d, float))
        err_rot = np.linalg.norm(R.from_matrix(
            R_d.T @ R.from_quat(quat).as_matrix()
        ).as_rotvec())
        if info is not None:
            info["err_pos"] = float(err_pos)
            info["err_rot"] = float(err_rot)
            info["pos_tol"] = self._cfg.ik_pos_tol
            info["rot_tol"] = self._cfg.ik_rot_tol
        return q_tgt, (err_pos < self._cfg.ik_pos_tol
                       and err_rot < self._cfg.ik_rot_tol)

    def sigma_min(self, q):
        """雅可比最小奇异值（本臂 7 关节）。

        双臂共用 robot_id 时 calculateJacobian 需要全可动关节（14）维输入，
        这里取整臂雅可比后只保留本臂的 7 列（ik_index），再求最小奇异值。
        """
        n_all = len(self._all_rev_ids)
        cur_all = np.array([p.getJointState(self._robot, j, physicsClientId=self._cid)[0]
                            for j in self._all_rev_ids])
        # 把 q 写回本臂关节，其它臂关节保持当前值
        for k, idx in enumerate(self._ik_index):
            cur_all[idx] = float(np.asarray(q, float)[k])
        Jlin, Jang = p.calculateJacobian(
            self._robot, self._ee_link, [0, 0, 0], list(cur_all),
            [0.0] * n_all, [0.0] * n_all, physicsClientId=self._cid)
        J = np.vstack([np.asarray(Jlin, float), np.asarray(Jang, float)])
        J = J[:, self._ik_index]  # 只取本臂列
        return float(np.linalg.svd(J, compute_uv=False)[-1])

    # ---------- 遥操语义 ----------
    @staticmethod
    def _pose_to_homo(pos, quat):
        H = np.eye(4)
        H[:3, 3] = pos
        H[:3, :3] = R.from_quat(quat).as_matrix()
        return H

    def start(self, pos, quat, now=None):
        """固定手柄/当前 TCP 锚点；重新握持是唯一重锚入口。"""
        self._homing = False
        now = time.monotonic() if now is None else now
        ee_p, ee_q = self._eef_pose()
        ee_R = R.from_quat(ee_q).as_matrix()
        self._target.start(pos, quat, ee_p + ee_R @ self._tcp_offset, ee_R, now)
        self._active = True
        self._hold_reason = None
        self._last_input_t = now
        self._last_step_t = now
        self._last_hand = (np.asarray(pos, float).copy(), R.from_quat(quat))
        self._pending_hand = None
        self._tracking_constrained = False
        self._command_q = self._get_q().copy()
        self._command_velocity = np.zeros(7)
        self._progress_t = now
        self._best_error = float("inf")
        self._progress_target = self._target.raw_target()
        print("[遥操][%s] grip 按下：固定锚点跟踪" % self.name)

    def pause(self):
        self._active = False
        self._command_velocity = np.zeros(7)
        self._command_q = self._get_q().copy()

    def hold(self, reason):
        self.pause()
        self._hold_reason = reason
        print("[保持][%s] %s；松开 grip 后重新握持" % (self.name, reason))

    def start_homing(self, now=None, max_joint_vel=None):
        """平滑回零（S-curve Cosine Profile）。"""
        now = time.monotonic() if now is None else now
        q_cur = self._get_q()
        delta = self._home - q_cur
        max_diff = float(np.max(np.abs(delta)))
        if max_diff < 1e-3:
            self._set_q(self._home)
            self._command_q = self._home.copy()
            self._command_velocity = np.zeros(7)
            self._homing = False
            return False

        self._active = False
        self._exact_held = False
        self._hold_reason = None

        if max_joint_vel is None:
            max_joint_vel = min(1.5, self._tuning.tracking.get("max_joint_velocity", 2.0) * 0.5)
        duration = max(1.2, float(np.pi * max_diff / (2.0 * max_joint_vel)))

        self._homing = True
        self._homing_start_q = q_cur.copy()
        self._homing_start_t = now
        self._homing_duration = duration
        print(f"[遥操][{self.name}] 启动平滑回零：最大关节偏差 {np.degrees(max_diff):.1f}°，预计耗时 {duration:.2f}s", flush=True)
        return True

    def is_homing(self):
        return self._homing

    def _step_homing(self, now):
        now = time.monotonic() if now is None else now
        elapsed = now - self._homing_start_t
        if elapsed >= self._homing_duration:
            self._set_q(self._home)
            self._command_q = self._home.copy()
            self._command_velocity = np.zeros(7)
            self._homing = False
            print(f"[遥操][{self.name}] 平滑回零完成", flush=True)
            return

        tau = np.clip(elapsed / self._homing_duration, 0.0, 1.0)
        s = 0.5 * (1.0 - np.cos(np.pi * tau))
        s_dot = 0.5 * np.pi * np.sin(np.pi * tau) / self._homing_duration

        delta = self._home - self._homing_start_q
        q_target = self._homing_start_q + s * delta
        v_target = s_dot * delta

        self._set_q(q_target)
        self._command_q = q_target.copy()
        self._command_velocity = v_target

    def sync_measured(self, q):
        q = np.asarray(q, float)
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise ValueError("measured joints must be seven finite radians")
        if np.any(q < self._lower - 0.02) or np.any(q > self._upper + 0.02):
            raise ValueError("measured joints outside model limits")
        self._set_q(np.clip(q, self._lower, self._upper))

    def update_pose(self, pos, quat, now=None):
        """只接收输入；不在网络回调中执行 IK。"""
        if not self._active:
            return
        now = time.monotonic() if now is None else now
        try:
            pp = np.asarray(pos, float)
            qq = np.asarray(quat, float)
            if pp.shape != (3,) or qq.shape != (4,) or not np.all(np.isfinite(pp)) or not np.all(np.isfinite(qq)):
                raise ValueError("invalid pose")
            rr = R.from_quat(qq)
            cfg = self._tuning.tracking
            old_p, old_r = self._last_hand
            dt = max(0.0, now - self._last_input_t)
            dt_extra = max(0.0, dt - (1.0 / 60.0))
            max_p = cfg["hand_jump_position"] + cfg["max_linear_velocity"] * dt_extra
            max_r = cfg["hand_jump_rotation"] + cfg["max_angular_velocity"] * dt_extra
            if (np.linalg.norm(pp-old_p) > max_p or
                    (old_r.inv()*rr).magnitude() > max_r):
                # Confirm a discontinuous sample before accepting it. A fast but
                # persistent movement remains a full anchored target; rate limits
                # in tick govern the robot, not a latched input-speed fault.
                pending = self._pending_hand
                confirmed = (pending is not None
                             and now - pending[2] <= cfg["input_timeout"]
                             and np.linalg.norm(pp-pending[0]) <= max_p
                             and (pending[1].inv()*rr).magnitude() <= max_r)
                if not confirmed:
                    self._pending_hand = (pp.copy(), rr, now)
                    return
            self._pending_hand = None
            self._target.update(pp, qq, now)
            self._last_hand = (pp.copy(), rr)
            self._last_input_t = now
        except (ValueError, TypeError):
            self.hold("无效手柄位姿")

    def feed_pose(self, pos, quat):
        """程序化兼容入口：输入后执行一步；服务端使用 update_pose + tick。"""
        self.update_pose(pos, quat)
        self.tick()

    def _step_teleop(self, pos, quat):
        self.feed_pose(pos, quat)

    def suspend_tracking(self, now):
        """Short input occlusion: keep command/anchor, discard motion velocity."""
        self._command_velocity = np.zeros(7)
        self._last_step_t = now
        self._set_q(self._command_q)

    def tick(self, now=None, measured_q=None, applied_q=None):
        if not self._active:
            if self._homing:
                self._step_homing(now)
            self.update_marker()
            return
        now = time.monotonic() if now is None else now
        cfg = self._tuning.tracking
        if now - self._last_input_t > cfg["input_timeout"]:
            self.hold("手柄输入超时")
            return
        elapsed = now - self._last_step_t
        if elapsed <= 0:
            return
        dt = min(elapsed, 0.05)
        self._last_step_t = now
        command_tcp = command_R = None
        if measured_q is not None:
            # Compare hardware feedback with the command actually applied by
            # the downstream follow server.  That server intentionally adds a
            # second joint-limit/slew layer, so comparing against this process'
            # faster upstream command turns normal wrist lag into a false
            # execution fault.  Older servers do not report applied_q; retain
            # the conservative legacy comparison for them.
            feedback_command = self._command_q
            if applied_q is not None:
                feedback_command = np.asarray(applied_q, float)
                if (feedback_command.shape != (7,)
                        or not np.all(np.isfinite(feedback_command))):
                    raise ValueError("applied joints must be seven finite radians")
                if (np.any(feedback_command < self._lower - 0.02)
                        or np.any(feedback_command > self._upper + 0.02)):
                    raise ValueError("applied joints outside model limits")
                feedback_command = np.clip(feedback_command, self._lower, self._upper)
            command_pos, command_quat = self._fk(feedback_command)
            command_R = R.from_quat(command_quat).as_matrix()
            command_tcp = command_pos + command_R @ self._tcp_offset
            self.sync_measured(measured_q)
        cur_pos, cur_quat = self._eef_pose()
        cur_R = R.from_quat(cur_quat).as_matrix()
        cur_tcp = cur_pos + cur_R @ self._tcp_offset
        raw_p, raw_R = self._target.raw_target()
        ep = float(np.linalg.norm(raw_p-cur_tcp))
        er = float(R.from_matrix(raw_R @ cur_R.T).magnitude())
        execution_ep = execution_er = 0.0
        if command_tcp is not None:
            execution_ep = float(np.linalg.norm(command_tcp-cur_tcp))
            execution_er = float(R.from_matrix(command_R @ cur_R.T).magnitude())
            if (execution_ep > cfg["max_position_error"] or
                    execution_er > cfg["max_rotation_error"]):
                self.hold("实机执行偏差过大（位置 %.1f mm，角度 %.1f°）"
                          % (execution_ep*1000, np.degrees(execution_er)))
                return
        # Operator lead is normal: retain the fixed anchor and chase the latest
        # absolute target under the existing Cartesian/joint rate constraints.
        error = max(ep/cfg["position_tolerance"], er/cfg["rotation_tolerance"])
        previous_p, previous_R = self._progress_target
        if (np.linalg.norm(raw_p-previous_p) > cfg["position_tolerance"] or
                R.from_matrix(raw_R @ previous_R.T).magnitude() > cfg["rotation_tolerance"]):
            self._progress_target = (raw_p.copy(), raw_R.copy())
            self._progress_t = now
            self._best_error = float("inf")
        if error <= 1 or error < self._best_error - 0.1:
            self._best_error = error
            self._progress_t = now
        # Unreachable targets are a recoverable constraint, not a grip fault.
        # Keep solving the latest absolute target so moving back resumes motion.
        self._tracking_constrained = now-self._progress_t > cfg["stall_timeout"]
        # 目标输入变化后允许新的收敛过程；保护计时由实际误差下降重置。
        if error <= 1:
            self._best_error = float("inf")
        # Feedback monitors actual convergence/execution; it must not rewind the
        # continuous command trajectory to a delayed measurement every cycle.
        measured_tcp = cur_tcp.copy()
        self._set_q(self._command_q)
        cur_pos, cur_quat = self._eef_pose()
        cur_R = R.from_quat(cur_quat).as_matrix()
        cur_tcp = cur_pos + cur_R @ self._tcp_offset
        tgt_tcp, tgt_R = self._target.target(now)
        self._resting = not (self._target._moving_t or self._target._moving_r)
        dp = tgt_tcp-cur_tcp
        dr = R.from_matrix(tgt_R @ cur_R.T).as_rotvec()
        # 中间可达参考限速，不覆盖保存的手柄绝对目标。
        step_fraction = min(1.0,
                            cfg["max_linear_velocity"]*dt/max(np.linalg.norm(dp), 1e-12),
                            cfg["max_angular_velocity"]*dt/max(np.linalg.norm(dr), 1e-12))
        dp *= step_fraction
        dr *= step_fraction
        step_R = R.from_rotvec(dr).as_matrix() @ cur_R
        step_p = cur_tcp+dp-step_R @ self._tcp_offset
        q_before = self._get_q_all()
        self._last_ik_info = {}
        if self._ik_backend == "exact":
            self._step_exact(dp, dr, dt)
        elif self._ik_backend == "pin":
            self._step_pin(step_p, step_R, dt)
        else:
            self._step_pybullet(step_p, step_R, dt)
        candidate = self._get_q()
        # 所有后端共享实际控制周期的速度/加速度约束。
        delta = candidate-self._command_q
        accel = cfg["max_joint_acceleration"]
        # The candidate is one rate-limited step, not a stop waypoint. Estimate
        # distance to the complete target for braking; near the endpoint the
        # fraction is 1 and this is the original finite-target stopping rule.
        remaining = np.abs(delta) / max(step_fraction, 1e-12)
        braking = np.sqrt((accel*dt)**2 + 2*accel*remaining)-accel*dt
        speed = np.minimum(np.abs(delta)/dt, braking)
        velocity = np.sign(delta)*np.minimum(speed, cfg["max_joint_velocity"])
        velocity = np.clip(velocity, self._command_velocity-accel*dt,
                           self._command_velocity+accel*dt)
        q_new = np.clip(self._command_q+velocity*dt, self._lower, self._upper)
        self._command_velocity = (q_new-self._command_q)/dt
        self._command_q = q_new
        self._set_q(q_new)
        self._last_ik_info.update(position_error=ep, rotation_error=er,
                                  execution_position_error=execution_ep,
                                  execution_rotation_error=execution_er,
                                  target_tcp=tgt_tcp.tolist(), target_R=tgt_R.tolist(),
                                  raw_target_tcp=raw_p.tolist(), measured_tcp=measured_tcp.tolist(),
                                  step_fraction=step_fraction,
                                  constrained=self._tracking_constrained,
                                  measured_q=None if measured_q is None else list(measured_q))
        if self._probe is not None:
            self._emit_probe(dt, self._last_hand[0], self._last_hand[1].as_quat(),
                             self._last_hand[0], dp, dr, dp, dr,
                             np.linalg.norm(dp), np.linalg.norm(dr), False,
                             q_before, cur_pos, cur_quat,
                             tgt_tcp-tgt_R @ self._tcp_offset, tgt_R)
        self.update_marker()

    # ---------- exact 后端 ----------
    def _jacobian_at_tcp(self, q_all):
        """被控点（TCP）处的 6×7 雅可比（只取本臂列），本体系。

        `localPosition` 传 tcp_offset：pybullet 的 calculateJacobian 接受连杆系
        下的偏移点，所以「被控点是工具尖而不是法兰」在这一处就做掉了 —— 与
        pin/pybullet 路径把偏移做在目标位姿上语义等价。
        """
        n_all = len(self._all_rev_ids)
        Jlin, Jang = p.calculateJacobian(
            self._robot, self._ee_link, list(self._tcp_offset), list(q_all),
            [0.0] * n_all, [0.0] * n_all, physicsClientId=self._cid)
        J = np.vstack([np.asarray(Jlin, float), np.asarray(Jang, float)])
        return J[:, self._ik_index]

    def _velocity_box(self, q_cur, dt):
        """逐关节速度盒 = 关节速度上限 ∩ 关节行程在 dt 内的可及量（两者取交）。"""
        limit = self._cfg.max_joint_vel
        lo = np.maximum(-limit, (self._lower - q_cur) / dt)
        hi = np.minimum(limit, (self._upper - q_cur) / dt)
        return lo, hi

    def _step_exact(self, rel_pos, rel_rot_world, dt):
        """Prefer exact task velocity; use bounded damped motion when infeasible."""
        t = self._tuning
        q_all_cur = self._get_q_all()
        q_cur = q_all_cur[self._ik_index]
        twist = np.concatenate([rel_pos, rel_rot_world]) / dt
        lo, hi = self._velocity_box(q_cur, dt)
        # No stop/resume hysteresis: constraints reduce the requested motion
        # continuously and the outer governor bounds acceleration.
        margin = 1.0
        box_lo, box_hi = lo, hi
        # J 只算一次，喂给求解器也算给 σ_min —— 告警里的 σ 必须与规则看到的同一个 J。
        # （早先告警取的是 sigma_min()：ee_link、零偏移，与带 tcp_offset 的 J 不是同一个。）
        J = self._jacobian_at_tcp(q_all_cur)
        sol = exact_task_velocity(J, twist, box_lo, box_hi,
                                  rank_tolerance=self._cfg.rank_tolerance,
                                  residual_tolerance=t.task_residual_tolerance)
        speed_limit = margin * self._cfg.max_joint_vel
        sigma = float(np.linalg.svd(J, compute_uv=False)[-1])
        # 探针：held_before 是**进入本帧时**的滞回状态，用来区分「一直不动」与
        # 「刚从持位恢复」（后者会在恢复那一帧突然把积压的手部动作放出来）。
        self._last_ik_info = {
            "backend": "exact", "achieved": bool(sol.achieved),
            "failure": sol.failure, "residual": float(sol.residual),
            "worst_joint": int(sol.worst_joint),
            "worst_excess": float(sol.worst_excess),
            # ↓ 判「速度上限还是行程」的两个量：哪一界被顶，那一界的盒值是多少。
            # box_lo_min/box_hi_max 是**全体关节**的极值（保留旧键，probe 在用）；
            # box_lo_worst/box_hi_worst 才定位到违约的那个关节。
            "worst_side": int(sol.worst_side),
            "binding": box_binding(sol, box_lo, box_hi, speed_limit),
            "sigma_min": sigma,
            "near_singular": bool(sigma < self._cfg.near_singular_sigma),
            "speed_limit": float(speed_limit),
            "box_lo_worst": (float(box_lo[sol.worst_joint])
                             if sol.worst_joint >= 0 else None),
            "box_hi_worst": (float(box_hi[sol.worst_joint])
                             if sol.worst_joint >= 0 else None),
            "held_before": bool(self._exact_held),
            "margin": float(margin),
            "box_lo_min": float(np.min(box_lo)),
            "box_hi_max": float(np.max(box_hi)),
        }
        velocity = sol.velocity
        if not sol.achieved:
            velocity = bounded_task_velocity(J, twist, box_lo, box_hi)
        self._exact_held = False
        self._last_ik_info["limited"] = not sol.achieved
        self._last_ik_info["delivered_residual"] = float(
            np.linalg.norm(J @ velocity - twist) / max(np.linalg.norm(twist), 1e-9))
        q_new = q_all_cur.copy()
        q_new[self._ik_index] = np.clip(q_cur + velocity * dt,
                                        self._lower, self._upper)
        self._set_q(q_new[self._ik_index])

    def _warn_exact(self, sol, binding, sigma):
        """exact 后端持位告警（限频 2s）。

        前缀是「持位」而不是「奇异」：这条规则每一次失败都是就地持位，而失败原因
        多数与奇异无关（实测 σ_min=0.16 的良态位姿照样报 box）。把前缀写死成「奇异」
        会把人只往奇异点引 —— 措辞本身就是一次归因，也得被数据管着。
        """
        now = time.time()
        if now - self._last_singular_warn <= 2.0:
            return
        self._last_singular_warn = now
        print("[持位]%s（sigma_min=%.4f，限频 2s）"
              % (describe_failure(sol, self.name, binding, sigma,
                                  near_singular_sigma=self._cfg.near_singular_sigma),
                 sigma))

    # ---------- 数据探针 ----------
    def _emit_probe(self, dt, hand_p, hand_q, prev_pos, rel_pre_t, rel_pre_r,
                    rel_t, rel_r, mag_t, mag_r, zeroed, q_before,
                    cur_pos, cur_quat, tgt_pos, tgt_R):
        """把本帧**各阶段**的量打包交给探针（诊断用；关掉时零开销）。

        为什么要全给：单看最终关节角无法归属一次跳变的原因 ——
        手柄增量小但解出的 dq 大 = IK/奇异；dq 大但 ``ik`` 里 ``achieved=False`` = 策略持位；
        ``zeroed`` 翻转 = 死区解冻；``dq`` 大而末端几乎没动 = 零空间换分支（不是真在动）。
        所以这里把「手 → 映射 → 平滑 → 死区 → 目标 → IK → 关节 → 末端」整条都留下。

        ⚠ 探针抛异常必须被吞掉：这是真机遥操的热路径，一个诊断 bug 不能把机械臂停住。
        """
        q_after = self._get_q_all()
        ee_after, quat_after = self._eef_pose()
        J = self._jacobian_at_tcp(q_after)
        try:
            self._probe(self, {
                "t": time.time(),
                "dt": dt,
                "hand_p": list(hand_p),
                "hand_q": list(hand_q),
                "hand_dp": list(np.asarray(hand_p, float) - prev_pos),
                "rel_pre_t": list(rel_pre_t),
                "rel_pre_r": list(rel_pre_r),
                "rel_t": list(rel_t),
                "rel_r": list(rel_r),
                "mag_t": mag_t,
                "mag_r": mag_r,
                "resting": bool(self._resting),
                "rest_confirm": self._rest_confirm,
                "zeroed": zeroed,
                "cur_ee_p": list(cur_pos),
                "cur_ee_q": list(cur_quat),
                "tgt_p": None if tgt_pos is None else list(tgt_pos),
                "q_before": list(q_before),
                "q_after": list(q_after),
                "ee_after_p": list(ee_after),
                "ee_after_q": list(quat_after),
                # TCP 处的最小奇异值 = 后端（exact）真正看到的那个量
                "sigma_min_tcp": float(np.linalg.svd(J, compute_uv=False)[-1]),
                "ik": dict(self._last_ik_info),
                "control_frame": self._control_frame,
            })
        except Exception:
            pass

    def _warn_ik(self, q_cur):
        """IK 未收敛告警（限频 2s），附当前构型奇异值。"""
        now = time.time()
        if now - self._last_singular_warn > 2.0:
            self._last_singular_warn = now
            print("[奇异预警][%s] IK 未收敛（sigma_min=%.4f）。"
                  "保留绝对目标继续受限追踪；持续不可达时自动保持。"
                  % (self.name, self.sigma_min(q_cur)))

    def _step_pin(self, tgt_pos, tgt_R, dt):
        """pinocchio + pink 微分 IK：每帧一次 QP 速度解算 + dt 积分。

        速度上限/配置限位由 pink 的 VelocityLimit/ConfigurationLimit 硬约束
        保证（对齐真机 max-speed），零空间由 PostureTask 正则到当前构型，
        候选解仍经过共享速度/加速度限制。QP失败时改用受限阻尼雅可比跟随。
        """
        q_all_cur = self._get_q_all()
        info = {"backend": "pin"}
        q_all_new, ok = self._pin.solve(q_all_cur, tgt_pos, tgt_R, dt, info)
        info["ok"] = bool(ok)
        if not ok:
            # A failed QP must not freeze all otherwise feasible task directions.
            cur_pos, cur_quat = self._eef_pose()
            cur_R = R.from_quat(cur_quat).as_matrix()
            dp = (tgt_pos + tgt_R @ self._tcp_offset
                  - cur_pos - cur_R @ self._tcp_offset)
            dr = R.from_matrix(tgt_R @ cur_R.T).as_rotvec()
            lo, hi = self._velocity_box(q_all_cur[self._ik_index], dt)
            velocity = bounded_task_velocity(self._jacobian_at_tcp(q_all_cur),
                                             np.r_[dp, dr] / dt, lo, hi)
            q_all_new = q_all_cur.copy()
            q_all_new[self._ik_index] += velocity * dt
            info["limited"] = True
        # 外层步长钳制：**metavr 对 pink 与 pybullet 一视同仁**（其 _step_teleop 只有
        # 一处 `clip(q_tgt - q_cur, ±MAX_STEP_RAD)`，在 IK 分派之后）。因为求解是
        # 「本帧内积分到收敛」，一次大目标（快挥/掉帧）可能让关节一口气走很远，这层
        # 就是挡那个的「异常跳变兜底」；正常小增量下它不生效。
        q_cur = q_all_cur[self._ik_index]
        q_new = q_all_new[self._ik_index]
        limit = self._cfg.max_step_rad
        info["dq_max"] = float(limit)
        info["clipped"] = bool(np.any(np.abs(q_new - q_cur) > limit))
        self._last_ik_info = info
        self._set_q(q_cur + np.clip(q_new - q_cur, -limit, limit))

    def _step_pybullet(self, tgt_pos, tgt_R, dt):
        """PyBullet candidate IK; shared tick applies physical-time limits.

        A best-effort candidate never replaces the persistent TCP target.
        """
        q_cur = self._get_q()
        ik_info = {}
        q_tgt, ok = self.ik(tgt_pos, tgt_R, q_seed=q_cur, info=ik_info)
        if not ok:
            self._warn_ik(q_cur)
            # metavr 语义：best-effort —— 只告警，仍然把（限幅后的）解下发。
            # 不置 q_tgt = q_cur：那会让参数化里被削掉的位移永远补不回来。

        # 步长限幅：与 metavr 同式 —— **只有** MAX_STEP_RAD 这一层。
        # （metavr 的注释：速度式 delta 映射下被削掉的位移不会补回，故 0.20 而非 0.08。）
        dq_max = self._cfg.max_step_rad
        dq = np.clip(q_tgt - q_cur, -dq_max, dq_max)
        # 探针：clipped 说明这一帧是**步长钳制**在决定关节走了多少（不是 IK）。
        ik_info.update({"backend": "pybullet", "ok": bool(ok),
                        "dq_max": float(dq_max),
                        "clipped": bool(np.any(np.abs(q_tgt - q_cur) > dq_max))})
        self._last_ik_info = ik_info
        self._set_q(q_cur + dq)


class LiteArmTeleopSim:
    """pybullet 遥操仿真环境：单臂（默认）或双臂（侧装立柱 / 正装无立柱）。

    默认单臂（right 正装）行为与原实现一致；传入 dual=True 加载双臂立柱 URDF，
    left/right 各一只臂，左右手手柄分别遥操。dual=True, upright=True 时改为加载
    双臂正装 URDF（两只独立竖直臂、无立柱）。
    """

    def __init__(self, dual=False, viewer="gui", side=False, ik_backend="pybullet",
                 tcp_offset=None, tuning=None, probe=None, sim_config=None,
                 upright=False):
        """viewer 选择显示后端："gui" = pybullet 自带 3D 窗口（默认，不依赖 ROS2）；
        "rviz" = pybullet DIRECT 无窗口，显示交给 RViz2（需 ROS2 发 /joint_states）。

        side=True 时单臂加载 litearm_side.urdf（水平侧装，mount roll=-90°），
        仅影响 world→base 朝向，不改变单臂关节名/限位。

        upright=True（需 dual=True）时加载双臂正装 URDF——两只独立竖直臂、无立柱，
        home/映射复用单臂正装基准（左臂 SINGLE_Q_HOME，右臂镜像）。

        ik_backend: "pybullet"（出厂，= metavr 的默认）用 calculateInverseKinematics；
        "pin" 用 pinocchio + pink（metavr 策略，见 pin_ik.py）；"exact" 精确任务速度。

        tcp_offset: 末端偏移量（米，本体系），操作者控制的工具点相对 tip link 的
        偏移；None = 出厂默认 `DEFAULT_TCP_OFFSET`（本台架夹爪的抓取点），
        `[0, 0, 0]` = 控制法兰。双臂两臂共用同一本体系偏移（两条链几何相同，
        故镜像后世界系偏移也镜像）。

        sim_config: `SimConfig`。None = 出厂常量（频率/限幅/home/轴映射/容差/求解器
        参数都是原值，逐位与改动前相同）。
        """
        self._viewer = viewer
        self._tcp_offset = as_tcp_offset(tcp_offset)
        self._tuning = tuning if tuning is not None else Tuning()
        self._cfg = sim_config if sim_config is not None else SimConfig()
        self._probe = probe
        if viewer == "rviz":
            self._cid = p.connect(p.DIRECT)
        else:
            # pybullet GUI 窗口
            self._cid = p.connect(p.GUI)
            p.configureDebugVisualizer(
                p.COV_ENABLE_GUI, 0, physicsClientId=self._cid)
            p.resetDebugVisualizerCamera(
                1.02, -90, 12, [0, 0.3, 0.35], physicsClientId=self._cid)
            print("pybullet GUI launched (PC window)")

        # ---- 机器人 ----
        self._arms = {}
        if dual:
            # 侧装（立柱）与正装（无立柱）走同一条「单 body、14 关节」路径，只换 URDF、
            # 映射配置与 home。两臂关节名/前缀/ik_index/pin frame 完全一致，无需分支。
            urdf_path = UPRIGHT_URDF_PATH if upright else DUAL_URDF_PATH
            home_left = (self._cfg.single_q_home if upright
                         else self._cfg.q_home)
            robot_id = _load_urdf_with_mesh(self._cid, urdf_path,
                                            DUAL_MESH_PKG, DUAL_MESH_DIR)
            # 可动关节 pybullet 索引（含固定关节，按 URDF 顺序）：
            # left_Joint1..7 → IK 向量 0..6；right_Joint1..7 → IK 向量 7..13
            n_j = p.getNumJoints(robot_id, physicsClientId=self._cid)
            all_rev_ids = [
                i for i in range(n_j)
                if p.getJointInfo(robot_id, i, physicsClientId=self._cid)[2]
                == p.JOINT_REVOLUTE
            ]
            # pin 模型（14 维）关节名，顺序与 all_rev_ids 一致（左臂后右臂）。
            all_joint_names = (["left_Joint%d" % k for k in range(1, 8)]
                               + ["right_Joint%d" % k for k in range(1, 8)])
            self._arms["left"] = Arm(self._cid, robot_id, "left",
                                     self._cfg.arm_mapping("left", dual=True,
                                                           upright=upright),
                                     prefix="left_",
                                     ik_index=list(range(0, 7)),
                                     all_rev_ids=all_rev_ids,
                                     joint_stem="Joint",
                                     ee_link_name="ee_frame_link",
                                     home=home_left, ik_backend=ik_backend,
                                     pin_urdf=urdf_path,
                                     pin_ee_frame="left_ee_frame_link",
                                     all_joint_names=all_joint_names,
                                     tcp_offset=self._tcp_offset,
                                     tuning=self._tuning, probe=self._probe,
                                     sim_config=self._cfg)
            self._arms["right"] = Arm(self._cid, robot_id, "right",
                                      self._cfg.arm_mapping("right", dual=True,
                                                            upright=upright),
                                      prefix="right_",
                                      ik_index=list(range(7, 14)),
                                      all_rev_ids=all_rev_ids,
                                      joint_stem="Joint",
                                      ee_link_name="ee_frame_link",
                                      home=home_left * self._cfg.mirror_signs,
                                      ik_backend=ik_backend,
                                      pin_urdf=urdf_path,
                                      pin_ee_frame="right_ee_frame_link",
                                      all_joint_names=all_joint_names,
                                      tcp_offset=self._tcp_offset,
                                      tuning=self._tuning, probe=self._probe,
                                      sim_config=self._cfg)
        else:
            urdf_path = SIDE_URDF_PATH if side else URDF_PATH
            robot_id = _load_urdf_with_mesh(self._cid, urdf_path,
                                            SINGLE_MESH_PKG,
                                            SINGLE_MESH_DIR)
            # 单臂侧装 = 只接一只**左**臂（水平侧装），语义与该臂在双臂侧装里的那只
            # 完全一致：占 "left" 槽（左手柄驱动、真机走 :9001）、home 取双臂侧装左臂的
            # Q_HOME（不外推 mirror_signs——那是右臂的）、映射取双臂侧装左臂那组。
            # 单臂正装无前缀、占 "right" 槽（保持原 on_controller_payload 行为），
            # home=single_q_home、映射复用单臂正装基准 translation_remap
            # （不能复用双臂侧装那组——那是水平侧装基准，与竖直正装朝向差 90°）。
            # 单臂 URDF 命名 joint1~7 / ee_link。
            arm_side = "left" if side else "right"
            home = self._cfg.q_home if side else self._cfg.single_q_home
            self._arms[arm_side] = Arm(
                self._cid, robot_id, arm_side,
                self._cfg.arm_mapping(arm_side, dual=False, side=side),
                prefix="",
                joint_stem="joint", ee_link_name="ee_link",
                home=home, ik_backend=ik_backend,
                pin_urdf=urdf_path, pin_ee_frame="ee_link",
                tcp_offset=self._tcp_offset,
                tuning=self._tuning, probe=self._probe,
                sim_config=self._cfg,
                all_joint_names=["joint%d" % k for k in range(1, 8)])

        self.name = ("DualLiteArmUpright_Sim" if upright
                     else ("DualLiteArm_Sim" if dual else "LiteArm_Sim"))
        if np.any(self._tcp_offset):
            print("末端偏移量 tcp_offset = %s m（本体系：控制该工具点而非法兰）"
                  % np.array2string(self._tcp_offset, precision=4))

    # ---------- 对外接口（WS 服务调用） ----------
    def arms(self):
        return self._arms

    def start(self, side, pos, quat, now=None):
        self._arms[side].start(pos, quat, now)

    def update_pose(self, side, pos, quat, now=None):
        self._arms[side].update_pose(pos, quat, now)

    def poll_keyboard(self):
        """GUI 键盘轮询（A 键回零）。与 marker 绘制解耦：控制环每次 tick 都调用它，
        但不再因此重画 marker（marker 已按 10/2 Hz 限流，见 ArmVisualMarker）。"""
        if not getattr(self, "_gui_ok", False):
            return
        try:
            keys = p.getKeyboardEvents(physicsClientId=self._cid)
            for k, v in keys.items():
                if k in (ord('a'), ord('A')) and (v & p.KEY_WAS_TRIGGERED):
                    for side in self._arms:
                        self.start_homing(side)
        except Exception:
            pass

    def update_markers(self):
        for arm in self._arms.values():
            arm.update_marker()
        self.poll_keyboard()

    def tick(self, now=None, measured=None):
        for side, arm in self._arms.items():
            arm.tick(now, None if measured is None else measured.get(side))
        self.update_markers()

    def pause(self, side):
        self._arms[side].pause()

    def feed_pose(self, side, pos, quat):
        self._arms[side].feed_pose(pos, quat)

    def home_q(self, side):
        """本臂 home 构型（7 关节角，rad），供首次跟随前 movej 回零用。"""
        return self._arms[side]._home.tolist()

    def go_home(self, side):
        """本臂平滑回到 home 构型并停止跟随。"""
        if side in self._arms:
            self._arms[side].start_homing()
        print("[遥操][%s] 启动平滑回零" % side, flush=True)

    def start_homing(self, side=None, now=None):
        """启动指定臂或所有臂的平滑回零。"""
        if side is not None:
            if side in self._arms:
                return self._arms[side].start_homing(now)
            return False
        started = False
        for arm in self._arms.values():
            started = arm.start_homing(now) or started
        return started

    def is_homing(self, side=None):
        if side is not None:
            return self._arms[side].is_homing() if side in self._arms else False
        return any(arm.is_homing() for arm in self._arms.values())

    def get_joint_states(self):
        """返回 (joint_names[list], positions[list])，供 ROS2 /joint_states 发布。

        双臂 = left_Joint1..7 + right_Joint1..7（14 个，与 joint_names.yaml 一致）；
        单臂 = joint1..7（7 个，小写，与单臂 URDF 一致）。
        """
        names = []
        positions = []
        if "left" in self._arms and "right" in self._arms:
            for side in ("left", "right"):
                for k in range(1, 8):
                    names.append(f"{side}_Joint{k}")
                    positions.append(0.0)
                arm = self._arms[side]
                positions[-7:] = arm._get_q().tolist()
        else:
            # 单臂：只有一只，键可能是 "left"（单臂侧装）或 "right"（单臂正装）。
            arm = next(iter(self._arms.values()))
            for k in range(1, 8):
                names.append(f"joint{k}")
            positions = arm._get_q().tolist()
        return names, positions

    # ---------- 单臂兼容接口（原 on_controller_payload） ----------
    @staticmethod
    def parse_payload(payload):
        """解析手柄载荷 "px,py,pz|qx,qy,qz,qw|trigger|a|b" → (pos, quat, trigger, a, b)。"""
        floats = [float(v) for group in payload.split("|") for v in group.split(",")]
        if len(floats) != 10:
            raise ValueError("bad payload: %r" % payload[:80])
        pos = np.asarray(floats[0:3])
        quat = np.asarray(floats[3:7])
        return pos, quat, floats[7], floats[8], floats[9]

    def on_controller_payload(self, payload, side=None):
        """单臂兼容入口：处理一帧手柄数据（单臂只有一只，默认取它）。"""
        try:
            pos, quat, trigger, a, b = self.parse_payload(payload)
        except ValueError:
            return
        arm = self._arms[side] if side in self._arms else next(iter(self._arms.values()))
        if a > 0.5 and not arm._a_held:
            if not arm._active:
                arm.start(pos, quat)
        if b > 0.5 and not arm._b_held:
            if arm._active:
                arm.pause()
        arm._a_held = a > 0.5
        arm._b_held = b > 0.5
        arm.feed_pose(pos, quat)

    # ---------- 便捷访问（测试用） ----------
    def __getattr__(self, name):
        """把底层方法/属性透传到默认（right）臂，保持旧单臂测试 API 不变。"""
        arms = self.__dict__.get("_arms")
        if arms:
            arm = arms.get("right")
            if arm is not None and hasattr(arm, name):
                return getattr(arm, name)
        raise AttributeError(name)

    # ---------- 主循环 ----------
    def stream(self):
        print("LiteArmTeleopSim stream started (程序化模式，无 zenoh)")
        period = 1.0 / self._cfg.freq
        while True:
            for arm in self._arms.values():
                arm._get_robot_state_vector()
            now = time.time()
            dt = now - self._last_iter if hasattr(self, "_last_iter") else 0
            if dt < period:
                time.sleep(period - dt)
            self._last_iter = time.time()

    def close(self):
        for arm in self._arms.values():
            arm.close()
        p.disconnect(self._cid)


def main():
    # 各项 default 都用 SUPPRESS：**没给过**的命令行参数不该出现在 args 里，
    # 否则「命令行的默认值」会盖掉配置文件里显式写的值。优先级靠下面 hasattr 实现。
    ap = argparse.ArgumentParser(description="LiteArm pybullet 遥操仿真（无 zenoh）")
    ap.add_argument("--config", default=None, metavar="PATH",
                    help="配置文件（默认 config/teleop.yaml）；命令行显式给的项更优先")
    ap.add_argument("--dual", action="store_true", default=argparse.SUPPRESS,
                    help="加载双臂 URDF（默认单臂正装）；配 --upright 为双臂正装")
    ap.add_argument("--upright", action="store_true", default=argparse.SUPPRESS,
                    help="--dual 时加载双臂正装 URDF（两只独立竖直臂、无立柱）；"
                         "不传则双臂侧装立柱")
    ap.add_argument("--side", action="store_true", default=argparse.SUPPRESS,
                    help="单臂侧装 = 单左臂（左手柄驱动、home/映射取双臂侧装左臂，"
                         "真机走 :9001）")
    ap.add_argument("--viewer", choices=["gui", "rviz"], default=argparse.SUPPRESS,
                    help="显示后端：gui=pybullet 窗口（默认）/ rviz=RViz2")
    ap.add_argument("--ik", choices=["pin", "pybullet", "exact"],
                    default=argparse.SUPPRESS,
                    help="IK 后端：pybullet=calculateInverseKinematics（出厂）/ "
                         "pin=pinocchio+pink（metavr 策略）/ "
                         "exact=精确任务速度（无阻尼，增益 1 或 0）")
    ap.add_argument("--tcp-offset", type=float, nargs=3, default=argparse.SUPPRESS,
                    metavar=("X", "Y", "Z"),
                    help="末端偏移量(米，本体系)：操作者控制的工具点相对 tip link 的偏移，"
                         "默认 %s = 本台架夹爪的抓取点；传 0 0 0 = 控制法兰"
                         % " ".join(str(v) for v in DEFAULT_TCP_OFFSET))
    ap.add_argument("--deadband-scale", type=float, default=argparse.SUPPRESS,
                    metavar="S",
                    help="静止死区阈值的乘数：1.0=出厂值，0.0=关掉死区（会放行追踪噪声）")
    ap.add_argument("--smooth-alpha", type=float, default=argparse.SUPPRESS,
                    metavar="A",
                    help="位姿低通在60Hz下的新样本权重，取值 (0, 1]；1.0=不平滑")
    ap.add_argument("--resume-margin", type=float, default=argparse.SUPPRESS,
                    metavar="M",
                    help="--ik exact 的持位滞回：<1 时恢复需多留余量（1.0=朴素规则）")
    ap.add_argument("--task-residual-tolerance", type=float,
                    default=argparse.SUPPRESS, metavar="T",
                    help="--ik exact 判定「任务被产生」的残差比例上限，出厂 1e-3")
    ap.add_argument("--manip-weight", type=float, default=argparse.SUPPRESS,
                    metavar="W",
                    help="--ik pin 的操纵度任务权重（把冗余推向雅可比更良态的构型, "
                         "预防奇异）；0=关闭（出厂）。调大会牺牲工具精度")
    ap.add_argument("--control-frame", choices=["tool", "base_link"],
                    default=argparse.SUPPRESS,
                    help="旋转增量读在哪个系：tool=末端局部系（出厂, 现状）/ "
                         "base_link=基座系(绕世界轴转, 与平移同源)")
    ap.add_argument("--base-link-signs", type=float, nargs=3,
                    default=argparse.SUPPRESS, metavar=("X", "Y", "Z"),
                    help="--control-frame base_link 用的逐轴旋转符号（未重测时勿乱改）")
    args = ap.parse_args()

    from .config import DEFAULT_PATH, get, load, set_path
    cfg_data = load(args.config)
    # 命令行 > 配置文件。只有**显式给过**的项才覆盖（见 SUPPRESS 注释）。
    explicit = {
        "tuning.deadband_scale": "deadband_scale",
        "tuning.smooth_alpha": "smooth_alpha",
        "tuning.resume_margin": "resume_margin",
        "tuning.task_residual_tolerance": "task_residual_tolerance",
        "tuning.manip_weight": "manip_weight",
        "tuning.control_frame": "control_frame",
        "tuning.base_link_signs": "base_link_signs",
    }
    for path, key in explicit.items():
        if hasattr(args, key):
            set_path(cfg_data, path, getattr(args, key))
    print("[配置] %s" % (args.config or DEFAULT_PATH))

    sim = LiteArmTeleopSim(
        dual=getattr(args, "dual", bool(get(cfg_data, "robot.dual", False))),
        side=getattr(args, "side", bool(get(cfg_data, "robot.side", False))),
        upright=getattr(args, "upright",
                        bool(get(cfg_data, "robot.upright", False))),
        viewer=getattr(args, "viewer", get(cfg_data, "robot.viewer", "gui")),
        ik_backend=getattr(args, "ik", get(cfg_data, "tuning.ik", "pybullet")),
        tcp_offset=getattr(args, "tcp_offset",
                           get(cfg_data, "tuning.tcp_offset", None)),
        tuning=tuning_from_config(cfg_data),
        sim_config=sim_config_from_config(cfg_data))
    try:
        sim.stream()
    except KeyboardInterrupt:
        print("\n[已停止] 遥操仿真退出")
    finally:
        sim.close()


if __name__ == "__main__":
    main()

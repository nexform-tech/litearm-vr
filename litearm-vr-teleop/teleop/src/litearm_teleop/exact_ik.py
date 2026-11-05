"""「精确任务速度」微分 IK 后端（--ik exact，遥操作 IK 后端之一）。

设计要点（为「增益不随位姿变化」）：
  - 任务要么被**精确**产生、要么**不产生**：在关节速度盒里若存在使
    `J·dq = twist` 成立的解，就精确交付该 twist（增益 1）；否则就地持位
    （增益 0）。中间刻意没有「按比例缩小」这一档 —— 缩放过的命令会让同一个
    人手动作在不同位姿产生不同的臂部运动，操作者读到的就是「臂不跟手」，
    而任何重新整定都救不回一个随位姿变化的增益。
  - **阻尼不参与。** 唯一的截断是浮点地板 `rank_tolerance`（把数值上为零的
    奇异值当成零，而不是把它求逆成无界速度）；幅值由关节速度盒界定。
    阻尼（含 pink 的 lm_damping 与 posture_cost）会按位姿连续衰减任务，
    正是本后端要消除的东西。
  - 冗余自由度只按**盒的要求**花：解集在零空间方向是一族
    `dq(α) = particular + α·direction`，取 ‖α‖ 最小的那个（α 尽量取 0）。
    没人强迫时让冗余轴留在原处 —— 否则操作者读到的是「臂自己在游走」。
  - 残差是**回代雅可比实测**的，不信任代数结果：截断的方向、秩亏的雅可比、
    被盒逼迫的冗余轴，都必须在这里显形，否则不宣称交付成功。

上述精确规则仅描述 exact_task_velocity；遥操在精确解不可行时使用
bounded_task_velocity 降级跟随，避免整步持位。

本模块与 pinocchio/pink 无关：雅可比以矩阵传入，故可对着合成雅可比测试
（含任何真实机械臂都到不了的构型）。移植自 litearm-teleop-vr 的
`include/litearm_vr_teleop/task_velocity.hpp`。
"""
import numpy as np

# 失败原因。它们不相似、且操作者只能对其中的一个采取措施，故分开报而不是
# 折成一个数 —— 折成一个数就会出现「形状不匹配 / 雅可比无方向 / 盒不可行」
# 都报成「100% 不可达」这种什么也没说的输出。
FAIL_NONE = "none"            # 交付成功
FAIL_SHAPE = "shape"          # 数组互相不匹配 —— 调用方 bug，与位姿无关
FAIL_NO_JACOBIAN = "no_jacobian"  # 雅可比没有任何可用方向，没有关节速度能移动工具
FAIL_BOX = "box"              # 产生该 twist 需要的关节速度超出盒，且冗余轴也救不回
FAIL_RESIDUAL = "residual"    # 解出来了，但残差大到不能称作「交付」

# 「这个位姿是否接近奇异」的**措辞**门槛：只决定告警句子怎么写，**不参与任何控制
# 决策**（真正的界限是盒，见模块头）。它存在是因为描述 FAIL_BOX 时若一律建议
# 「往奇异点外移」，而同一行印出的 σ_min 当场就能驳倒它 —— 一个错的补救方向比
# 没有补救方向更贵。数值与 probe.py 的 sigma_th 取齐。
NEAR_SINGULAR_SIGMA = 0.05


class TaskVelocitySolution:
    """规则要什么、能给什么。"""

    __slots__ = ("velocity", "residual", "achieved", "failure",
                 "worst_joint", "worst_excess", "worst_side")

    def __init__(self, velocity, residual=0.0, achieved=False, failure=FAIL_NONE,
                 worst_joint=-1, worst_excess=0.0, worst_side=0):
        self.velocity = velocity
        self.residual = residual
        self.achieved = achieved
        self.failure = failure
        # 仅 kBox 有意义：最违约的关节（0 基）与违约量 [rad/s]；不适用时 -1。
        self.worst_joint = worst_joint
        self.worst_excess = worst_excess
        # 仅 kBox 有意义：超出的是哪一界（+1 = 上界、-1 = 下界、0 = 未知）。
        # 判定「是速度上限还是行程在起作用」必须知道它，见 box_binding。
        self.worst_side = worst_side


def null_space_interval(velocity, direction, lower, upper):
    """使 `velocity + α·direction` 逐关节落在 `[lower, upper]` 内的 α 区间。

    每个关节给出自己的区间，答案是它们的**交集** —— 所以结果是「冗余轴的
    哪个位置是共同合法的」，而不是逐关节折中。某关节的 direction 分量为 0
    表示该关节由任务定死：它要么满足自己的盒，要么整个区间为空。

    Returns:
        (low, high, empty)
    """
    low, high = -np.inf, np.inf
    n = velocity.size
    if n == 0 or direction.size != n or lower.size != n or upper.size != n:
        return (low, high, True)

    for i in range(n):
        d = direction[i]
        if d == 0.0:
            if velocity[i] < lower[i] or velocity[i] > upper[i]:
                return (low, high, True)
            continue
        # 除以负分量会交换上下界，所以这里显式排序而不是假定。
        a = (lower[i] - velocity[i]) / d
        b = (upper[i] - velocity[i]) / d
        low = max(low, min(a, b))
        high = min(high, max(a, b))
    return (low, high, low > high)


def exact_task_velocity(jacobian, twist, lower, upper,
                        rank_tolerance=1e-9, residual_tolerance=1e-3):
    """产生 `twist` 的关节速度，冗余方向只在盒要求时才花。

    关节多于任务行数时，`J·dq = twist` 的解构成一个仿射集，而产生不了任何
    工具运动的那个关节方向参数化了它。这里取**最小范数**解（冗余轴留在姿态
    已把它放在的地方），除非盒把它排除在外 —— 那时冗余轴恰好承担盒所要求的
    那份差额。

    Args:
        jacobian: (rows, cols)，本机 6×7。
        twist: (rows,)，本体系。
        lower / upper: (cols,)，逐关节速度盒 [rad/s]。
        rank_tolerance: 相对最大奇异值的截断比。这是**数值**而非策略：把数值
            上的零奇异值当成零，避免伪逆求出无界关节速度。
        residual_tolerance: 交付的**策略**线 —— 「任务被产生了」与「任务没被
            产生、于是臂保持」之间的分界，按 twist 自身幅值的比例计。

    Returns:
        TaskVelocitySolution。`achieved` 为 False 时 `velocity` 全零。
    """
    jacobian = np.asarray(jacobian, float)
    twist = np.asarray(twist, float).ravel()
    lower = np.asarray(lower, float).ravel()
    upper = np.asarray(upper, float).ravel()
    rows, cols = jacobian.shape

    if cols <= 0 or rows <= 0 or twist.size != rows \
            or lower.size != cols or upper.size != cols:
        return TaskVelocitySolution(np.zeros(max(cols, 0)),
                                    residual=1.0, failure=FAIL_SHAPE)

    # 完整分解而非薄分解：宽矩阵的薄 SVD 不含零空间对应的那一列 V。
    svd_u, singular, svd_vh = np.linalg.svd(jacobian, full_matrices=True)
    if singular.size == 0 or singular[0] <= 0.0:
        return TaskVelocitySolution(np.zeros(cols),
                                    residual=1.0, failure=FAIL_NO_JACOBIAN)

    cut = rank_tolerance * singular[0]
    keep = singular > cut
    # 把每个奇异方向承担的那份 twist 放回承载它的关节方向。不存在的方向
    # 贡献 0，而不是贡献无穷大。
    particular = (svd_vh[:singular.size][keep].T
                  @ ((svd_u[:, :singular.size][:, keep].T @ twist) / singular[keep]))

    # 产生不了任何工具运动的关节方向。关节多于任务行数时才存在；秩亏只会让
    # 零空间更大，所以 V 的最后一列仍然在它里面。
    direction = svd_vh[cols - 1].copy() if cols > rows else np.zeros(cols)

    low, high, empty = null_space_interval(particular, direction, lower, upper)
    if empty:
        # 冗余轴的任何位置都不合法，故无解。**哪个关节的盒最救不回来、差多少**
        # 才是可行动的，而最小的那个缺口是最诚实的数 —— 它是限位最少要让出的量。
        # 缺口对 α 是分段线性的，故最小值必在断点上，而每个断点都是某关节
        # 离开或重新进入自己的盒。
        breakpoints = [0.0]
        for i in range(cols):
            if direction[i] == 0.0:
                continue
            breakpoints.append((lower[i] - particular[i]) / direction[i])
            breakpoints.append((upper[i] - particular[i]) / direction[i])
        best = np.inf
        worst_joint = -1
        worst_side = 0
        for alpha in breakpoints:
            value = particular + alpha * direction
            miss = np.maximum(0.0, np.maximum(lower - value, value - upper))
            j = int(np.argmax(miss))
            if miss[j] < best:
                best = float(miss[j])
                worst_joint = j
                # 超出的是哪一界。必须记下来：盒是「速度上限 ∩ 行程/dt」的交集，
                # 两半能给出同一个「超出」，但补救方向相反（减速 vs 退回行程内），
                # 而只看 magnitude 分不出是哪一半，见 box_binding。
                worst_side = 1 if value[j] > upper[j] else -1
        return TaskVelocitySolution(np.zeros(cols), residual=1.0, failure=FAIL_BOX,
                                    worst_joint=worst_joint,
                                    worst_excess=0.0 if not np.isfinite(best) else best,
                                    worst_side=worst_side)

    alpha = float(min(high, max(low, 0.0)))
    velocity = particular + alpha * direction

    # 回代雅可比实测，而不是相信代数：截断的方向、秩亏的雅可比、被盒逼迫的
    # 冗余轴，都必须在这里显形，否则不宣称交付成功。
    reference = max(float(np.linalg.norm(twist)), 1e-9)
    residual = float(np.linalg.norm(jacobian @ velocity - twist))
    ratio = residual / reference
    if residual > residual_tolerance * reference:
        return TaskVelocitySolution(np.zeros(cols), residual=ratio,
                                    failure=FAIL_RESIDUAL)
    return TaskVelocitySolution(velocity, residual=ratio, achieved=True)


def box_binding(solution, box_lower, box_upper, speed_limit):
    """某个 FAIL_BOX 的「超出」是**哪一半限位**给的约束。

    盒是「关节速度上限 ∩ 关节行程/dt」的交集（见 `teleop_sim._velocity_box`）。两半
    能给出同一个「超出」，但补救方向相反：速度上限 ⇒ 手速太快、该减速或降增益；
    行程 ⇒ 该关节已到行程末端、该把工具移回工作空间内。而求解器这一层看不出是谁
    给的（它只拿到交集后的盒），所以由组装盒的人把盒与上限一起递进来做判定。

    **判定必须知道超出的是哪一界**（`solution.worst_side`）：若一边被行程收窄、
    另一边被速度顶掉，只比较 magnitude 就会指错方向。

    Args:
        solution: `exact_task_velocity` 的返回。`failure` 不是 FAIL_BOX 时返回 None。
        box_lower / box_upper: 交给求解器的那个盒（含任何滞回余量）。
        speed_limit: 速度上限本身，**量纲与盒一致**（即已含同样的滞回余量）。

    Returns:
        "speed" | "range" | None
    """
    j = int(solution.worst_joint)
    if solution.failure != FAIL_BOX or j < 0 or solution.worst_side == 0:
        return None
    if solution.worst_side > 0:
        bound, ceiling = box_upper[j], speed_limit
    else:
        bound, ceiling = box_lower[j], -speed_limit
    # 顶到上限 ⇒ 这一界就是速度上限给的；明显小于上限 ⇒ 行程收窄了它。
    return "speed" if abs(bound - ceiling) <= 1e-9 else "range"


def describe_failure(solution, side="", binding=None, sigma_min=None,
                     near_singular_sigma=None):
    """给操作者的一句话，用规则自己的话讲，且每种失败给各自的补救方向。

    `binding` 与 `sigma_min` 都只影响 FAIL_BOX 的措辞，且都由**调用方**提供 ——
    判定它们需要盒的来源与雅可比的奇异值，而这两样在纯函数这一层没有：

      binding   "speed" = 速度上限在起作用 · "range" = 行程在起作用 · None = 不知道。
                **不知道就不给补救方向**：先前的版本对每个 box 失败都印「把手往
                奇异点外移」，而同一行印出的 σ_min 常常当场驳倒它（本台架实测
                σ_min=0.16 的良态位姿也在报这句）。一个错的补救方向比没有更贵。
      sigma_min 与 `near_singular_sigma` 比，决定要不要提奇异点。None = 不提。
      near_singular_sigma 是那个「接近奇异」的门槛，None = 用本模块出厂
                `NEAR_SINGULAR_SIGMA`。它可配是为了让配置能改**措辞**；它不进
                任何控制决策，所以改它不会改变臂怎么动。
    """
    threshold = (NEAR_SINGULAR_SIGMA if near_singular_sigma is None
                 else near_singular_sigma)
    tag = "[%s]" % solution.failure if solution.failure != FAIL_NONE else "[ok]"
    if side:
        tag = "[%s][%s]" % (side, solution.failure)
    if solution.failure == FAIL_BOX:
        text = ("%s 关节 %d 需要的关节速度比该位姿允许的多 %.2f rad/s，"
                "且冗余方向上没有任何位置能让它合法。"
                % (tag, solution.worst_joint + 1, solution.worst_excess))
        if binding == "speed":
            text += ("起作用的是关节速度上限：手速超出了臂在这个位姿能产生的速度。"
                     "放慢手速，或调低手柄到臂的映射增益。")
        elif binding == "range":
            text += "起作用的是关节行程：该关节已到行程末端。把手往工作空间内移。"
        if sigma_min is not None:
            text += ("该位姿接近奇异（σ_min 低），所以需要的关节速度才这么大，"
                     "把手往奇异点外移。" if sigma_min < threshold
                     else "该位姿不接近奇异，这一次不是奇异点的问题。")
        return text
    if solution.failure == FAIL_NO_JACOBIAN:
        return ("%s 此位姿的雅可比没有任何可用方向，没有关节速度能移动工具。"
                "把手往奇异点外移。" % tag)
    if solution.failure == FAIL_RESIDUAL:
        return ("%s 命令的 twist 有 %.1f%% 在此位姿产生不了。把手往奇异点外"
                "或工作空间内移。" % (tag, 100.0 * solution.residual))
    if solution.failure == FAIL_SHAPE:
        return "%s 数组互相不匹配 —— 这是本节点的 bug，与位姿无关。" % tag
    return "%s twist 已被精确交付。" % tag


def bounded_task_velocity(jacobian, twist, lower, upper, damping=0.02):
    """Best feasible task motion with damping, including rank-deficient poses.

    Solve ||J v - twist||² + damping² ||v||² inside the velocity/range
    box. Fixed joints are eliminated so zero-width bounds remain valid.
    There is no null-space posture motion or target reanchoring.
    """
    from scipy.optimize import lsq_linear

    J = np.asarray(jacobian, float)
    target = np.asarray(twist, float)
    lo, hi = np.asarray(lower, float), np.asarray(upper, float)
    if (J.ndim != 2 or target.shape != (J.shape[0],)
            or lo.shape != (J.shape[1],) or hi.shape != lo.shape
            or not all(np.all(np.isfinite(x)) for x in (J, target, lo, hi))
            or np.any(lo > hi) or not np.isfinite(damping) or damping <= 0):
        raise ValueError("invalid bounded task velocity inputs")
    free = hi > lo
    velocity = lo.copy()
    if np.any(free):
        count = int(np.sum(free))
        A = np.vstack((J[:, free], damping * np.eye(count)))
        b = np.concatenate((target - J[:, ~free] @ velocity[~free], np.zeros(count)))
        result = lsq_linear(A, b, bounds=(lo[free], hi[free]),
                            method="bvls", tol=1e-10, max_iter=100)
        velocity[free] = np.clip(result.x, lo[free], hi[free])
    return velocity

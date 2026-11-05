"""pinocchio + pink 逆运动学求解器（遥操作 IK 后端之一）。

**策略 = metavr 的 `PinkIk`**（参数与求解结构都对齐，见 `PINK_*` 常量），唯一
保留的差别是**双臂**：本模块用一个 14 维模型、左右臂各持一个实例，本臂自由关节
之外的「被动关节」每次迭代后速度置零 → 锁死另一臂（等价 pybullet 后端 lo=up 的
锁死语义）。metavr 的 pink 只支持单臂（`prefix=="" and all_rev_ids is None`）。

任务栈（同一个 QP 里按代价分层）：
  FrameTask(ee)            主任务：位置 + 姿态跟踪
  ManipulabilityTask(ee)   零空间：最大化可操作度 —— **主动推离奇异**（常开）
  PostureTask              零空间：拉向种子 —— 解连续 + 轻阻尼
约束（QP 硬约束）：ConfigurationLimit（不越限位）+ VelocityLimit（速度有界）。
近奇异的第二道保险是 FrameTask 的 lm_damping（DLS 阻尼）：奇异方向上的关节速度
被压住 —— 两层合起来保证「趋近奇异时减速，而不是爆速度」。

求解结构：**从种子出发逐拍积分到收敛**（最多 `max_iters` 拍、每拍步长固定
`dt`，任务误差范数低于 `err_tol` 即停），不是「每帧只解一步速度」。所以它是
「把目标位姿在本帧内解出来」，而不是「按本帧 dt 推进一小步」。

世界系目标位姿可直接传入：本模块加载的 URDF 与 pybullet 加载的是同一份，
模型世界系与 pybullet 世界系一致（侧装 mount / 立柱固定关节已烘焙进模型）。
"""
import numpy as np

# ---- 求解策略常量（= metavr teleop_sim.py 的 PINK_*，一字不差）----
# ⚠ 抄的是 metavr 的**数值**，不是它选定的 QP 后端：metavr 写的是 "daqp"，而本机
# （metavr-arm）qpsolvers 只装了 quadprog（`available_solvers == ['quadprog']`，
# 传 daqp 会 SolverNotFound）⇒ 那个后端在本环境里根本跑不起来。故后端取
# "quadprog"（可配置），其余数值照抄。
PINK_DT = 0.05              # 积分步长 (s)：× URDF 速度限幅(1 rad/s) = 0.05 rad/迭代
PINK_MAX_ITERS = 200        # 每帧迭代上限（小步目标 ~10 次收敛；大跳变最坏 ~100ms）
PINK_ERR_TOL = 1e-4         # 任务误差范数收敛阈值
PINK_LM_DAMPING = 1e-3      # FrameTask 的 Levenberg-Marquardt 阻尼（近奇异限速）
PINK_POSTURE_COST = 1e-3    # 姿态任务代价（钉住种子 → 解连续 + 轻阻尼）
PINK_MANIP_COST = 1e-2      # 可操作度任务代价（零空间主动推离奇异）
PINK_MANIP_RATE = 0.1       # 可操作度变化率上限


class PinIKSolver:
    """封装 pinocchio 模型 + pink 微分 IK，供单臂/双臂的 Arm 调用。"""

    def __init__(self, urdf_path, ee_frame, joint_names, active_idx,
                 solver="quadprog", position_cost=1.0, orientation_cost=1.0,
                 lm_damping=PINK_LM_DAMPING, posture_cost=PINK_POSTURE_COST,
                 velocity_limit=1.0, manipulability_cost=PINK_MANIP_COST,
                 manip_rate=PINK_MANIP_RATE, dt=PINK_DT,
                 max_iters=PINK_MAX_ITERS, err_tol=PINK_ERR_TOL):
        """构建求解器。

        Args:
            urdf_path: 与 pybullet 加载的同名 URDF 路径。
            ee_frame: 末端 frame 名（单臂 "ee_link"；双臂
                "left_ee_frame_link" / "right_ee_frame_link"）。
            joint_names: 全 q 的 pybullet 顺序关节名（单臂 7 个；双臂 14 个
                left_Joint1..7 + right_Joint1..7），用于与 pinocchio 关节序
                建立置换并校验。
            active_idx: 本臂自由关节在全 q 中的下标（单臂 [0..6]；双臂左
                [0..6]、右 [7..13]）；其余为被动关节，求解后速度置零。
            solver: qpsolvers 后端名。**出厂 "quadprog"** —— metavr 写的是 "daqp"，
                但本环境没装它（见 PINK_* 上方注释），照抄会直接跑不起来。
            position_cost / orientation_cost / lm_damping: FrameTask 参数。
            posture_cost: PostureTask 权重（小权重 → 弱正则，仅约束零空间）。
            velocity_limit: 关节速度上限（rad/s）。**出厂 1.0 = URDF 自己的值**
                （本仓 URDF 全关节 `velocity="1"`），与 metavr 的
                `VelocityLimit(model)`（不给显式向量）等价。
            manipulability_cost / manip_rate: 可操作度任务。**常开**（metavr 无开关），
                把冗余自由度推向雅可比更良态的构型 —— 是**预防**奇异而不是事后削速度。
                cost 同时缩放 Hessian 与线性项 ⇒ 调大会与 FrameTask 抢优先级、牺牲
                工具精度，按台架量着调。
            dt / max_iters / err_tol: 逐拍积分到收敛的三个参数（见模块头）。
        """
        import pinocchio as pin
        import pink
        from pink.limits import ConfigurationLimit, VelocityLimit

        self._pin = pin
        self._pink = pink
        self._solver = solver
        self._position_cost = position_cost
        self._orientation_cost = orientation_cost
        self._lm_damping = lm_damping
        self._posture_cost = posture_cost
        self._v_limit = float(velocity_limit)
        self._manip_cost = float(manipulability_cost)
        self._manip_rate = float(manip_rate)
        self._dt = float(dt)
        self._max_iters = int(max_iters)
        self._err_tol = float(err_tol)

        self.model = pin.buildModelFromUrdf(urdf_path)

        # ee frame：单臂 URDF 的 fixed 关节与其子 link 同名（都是 "ee_link"），
        # pinocchio 会同时建 JOINT 与 BODY 两个同名 frame，按名查找有歧义
        # （pink 内部用 model.getFrameId(name)）。这里显式取 BODY frame，并加
        # 一个唯一名字的 OP_FRAME 别名交给 pink，避免改动 URDF 资产。
        try:
            ee_id = self.model.getFrameId(ee_frame, pin.FrameType.BODY)
        except Exception:
            ee_id = self.model.getFrameId(ee_frame)
        if ee_id >= self.model.nframes:
            raise ValueError("URDF 中找不到 ee frame: %s" % ee_frame)
        self._ee_frame = "_pink_ee_" + ee_frame
        _f = self.model.frames[ee_id]
        self.model.addFrame(pin.Frame(
            self._ee_frame, _f.parentJoint, _f.parentFrame, _f.placement,
            pin.FrameType.OP_FRAME))
        self._frame_id = self.model.getFrameId(self._ee_frame)

        self.data = self.model.createData()
        self.nv = self.model.nv

        # pinocchio 关节序（跳过 universe）→ pybullet 序 的置换。
        # q_pin = q_py[perm]；写回 q_py[perm] = q_pin。
        try:
            perm = [joint_names.index(self.model.names[k + 1])
                    for k in range(self.model.nq)]
        except ValueError as e:
            raise ValueError(
                "joint_names 与 URDF 关节不一致: %r" % (joint_names,)) from e
        if sorted(perm) != list(range(len(perm))):
            raise ValueError("joint_names 与 URDF 关节集合不一致: %r" % (joint_names,))
        self._perm = np.asarray(perm, int)

        active = np.asarray(active_idx, int)
        self._passive = np.setdiff1d(np.arange(self.nv), active)

        # 限位：配置限位取自 URDF。速度限位形式上给显式向量（出厂值 = URDF 的 1.0），
        # 与 metavr 的 `VelocityLimit(model)` 数值一致；给向量只是为了留一个可调口。
        self._limits = [
            ConfigurationLimit(self.model),
            VelocityLimit(self.model, np.full(self.nv, float(velocity_limit))),
        ]

        # 可操作度任务：**常开**（metavr 无开关），把冗余自由度推向"雅可比更良态"
        # 的构型，是**预防**奇异而不是事后再削速度的唯一手段。
        # ⚠ 必须传 `self._ee_frame`（本类刚建的唯一 OP_FRAME 别名），不能传原始
        # `ee_frame`：单臂 URDF 里 fixed 关节与其子 link 同名，按名查找有歧义，
        # ManipulabilityTask 内部 `getFrameId(name)` 会直接抛
        # "Several frames match the filter"。
        # ⚠ 该任务要求「根到 frame 的路径全是转动关节」，构造时就会检查 ⇒ 失败时
        # 降级为"不启用"并留痕，绝不因为它把整个遥操打断。
        # ⚠ 保留 teleop 的这一层兜底（metavr 没有）：构造失败时降级为「不启用」并留痕，
        # 绝不因为它把整个遥操打断。构造成功时行为与 metavr 一致。
        self._manip = None
        self._manip_err = None
        if self._manip_cost > 0.0:
            try:
                from pink.tasks import ManipulabilityTask
                self._manip = ManipulabilityTask(
                    self._ee_frame, self.model, cost=self._manip_cost,
                    reference_frame=pin.ReferenceFrame.WORLD,
                    manipulability_rate=self._manip_rate)
            except Exception as e:
                self._manip_err = "%s: %s" % (type(e).__name__, e)
                print("[pin_ik] 操纵度任务构造失败, 已忽略: %s" % self._manip_err,
                      flush=True)

    # ---------- 内部 ----------
    def _frame_pose(self, q_pin):
        """pinocchio FK 得到 ee frame 世界位姿 (pos[3], R[3,3])。"""
        pin = self._pin
        pin.forwardKinematics(self.model, self.data, q_pin)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self._frame_id]
        return np.asarray(oMf.translation, float), np.asarray(oMf.rotation, float)

    # ---------- 对外 ----------
    def fk(self, q_py_full):
        """正运动学：pybullet 序全 q → ee 世界位姿 (pos, R)。"""
        pos, Rot = self._frame_pose(np.asarray(q_py_full, float)[self._perm])
        return pos, Rot

    def solve(self, q_py_full, pos_d, R_d, dt, info=None):
        """从种子出发逐拍积分到目标位姿（metavr 式），失败时交回当前最优构型。

        **不是**「每帧只解一步速度」：本方法在自己的循环里把 QP 反复积分到任务误差
        低于 `err_tol`（最多 `max_iters` 拍、每拍固定 `self._dt`），所以它在本帧内
        就把目标位姿解出来了。`dt`（本帧实测）**只进诊断**，不参与积分 —— 积分步长
        恒为 `self._dt`（= metavr 的 PINK_DT），这样「能解多远」不受掉帧影响。

        Args:
            q_py_full: 当前全 q（pybullet 序）。
            pos_d: 目标位置（世界系）。
            R_d: 目标姿态（世界系，3x3）。
            dt: 本帧实测 dt [s]（仅回填 `info["frame_dt"]`）。
            info: 可选的 dict，回填诊断（探针用）。键：
                `iters` = 实际迭代拍数；`converged` = 是否在 err_tol 内收敛；
                `err_final` = 收敛时的任务误差范数；
                `v_absmax` = 最后一拍解出的关节速度最大值 [rad/s]；
                `at_limit` = 最后一拍是否顶到 velocity_limit（盒在起作用）；
                `pink_error` = QP 失败时为 True。

        Returns:
            (q_py_full_new, ok):
              q_py_full_new: 积分后的全 q（pybullet 序）；QP 失败时 = 输入 q；
              ok: 本步 QP 是否全程没抛 PinkError（False ⇒ 调用方按 metavr 的
                  best-effort 语义照常下发并告警，而不是就地持位）。
        """
        pin = self._pin
        pink = self._pink
        q_py_full = np.asarray(q_py_full, float)
        pos_d = np.asarray(pos_d, float)
        R_d = np.asarray(R_d, float)
        q_pin = q_py_full[self._perm]

        iters = 0
        err_final = float("inf")
        vmax = 0.0
        try:
            configuration = pink.Configuration(self.model, self.data, q_pin)
            frame_task = pink.FrameTask(
                self._ee_frame,
                position_cost=self._position_cost,
                orientation_cost=self._orientation_cost,
                lm_damping=self._lm_damping,
            )
            frame_task.transform_target_to_world = pin.SE3(R_d, pos_d)
            posture = pink.PostureTask(cost=self._posture_cost)
            posture.set_target(q_pin)
            tasks = [frame_task, posture]
            if self._manip is not None:
                tasks.append(self._manip)
            step = self._dt
            for _ in range(self._max_iters):
                iters += 1
                velocity = pink.solve_ik(
                    configuration, tasks, step,
                    solver=self._solver, limits=self._limits, safety_break=False)
                if self._passive.size:
                    # 锁死另一臂：被动关节速度置零后再积分（等价 pybullet 的 lo=up）。
                    velocity[self._passive] = 0.0
                vmax = float(np.max(np.abs(velocity))) if velocity.size else 0.0
                configuration.integrate_inplace(velocity, step)
                err_final = float(np.linalg.norm(
                    frame_task.compute_error(configuration)))
                if err_final < self._err_tol:
                    break
            q_pin_new = np.array(configuration.q, float)
        except pink.PinkError:
            # QP 无解（如贴限位）：交回种子（= 本帧不动），并如实报 ok=False。
            if info is not None:
                info["pink_error"] = True
            return q_py_full.copy(), False

        if info is not None:
            info["iters"] = iters
            info["converged"] = bool(err_final < self._err_tol)
            info["err_final"] = err_final
            info["frame_dt"] = float(dt)
            # 顶到速度上限 = 这次是**限位**在决定答案，不是任务 —— 探针靠它把
            # 「被限位削掉」与「QP 真的解出了这个速度」分开。
            info["v_absmax"] = vmax
            info["v_limit"] = self._v_limit
            info["at_limit"] = vmax >= self._v_limit * 0.999
            info["manip_cost"] = self._manip_cost
            info["manip_active"] = self._manip is not None
            info["manip_err"] = self._manip_err

        q_py_new = q_py_full.copy()
        q_py_new[self._perm] = q_pin_new
        return q_py_new, True



#!/usr/bin/env python3
"""exact 后端「持位告警」措辞的**纯函数**单测（只依赖 numpy，不起 pybullet）。

覆盖 `--ik exact` 报 FAIL_BOX 时那句「为什么不动、该往哪改」：
  - 超出的那**一半**限位是**关节速度上限**还是**关节行程**；
  - 该位姿到底近不近奇异。

为什么措辞要单独测：告警句是操作者唯一能据以行动的东西。先前的版本对每个 box
失败都印「把手往奇异点外或工作空间内移」，而同一行印出的 σ_min 常常当场驳倒它
（本台架实测 σ_min≈0.16 的良态位姿也在报这句）—— 一句**错的补救方向比没有更贵**。

用法: python tests/test_exact_ik.py
"""
import sys

import numpy as np

from litearm_teleop.exact_ik import (FAIL_BOX, FAIL_NONE, FAIL_RESIDUAL,
                                     NEAR_SINGULAR_SIGMA, TaskVelocitySolution,
                                     box_binding, describe_failure,
                                     exact_task_velocity, null_space_interval)

# 单行任务 J = [[1, 0]]（1 行 2 列）：关节 0 被任务定死，关节 1 是零空间方向
# ⇒ 关节 0 的值就是 twist，与冗余轴无关，便于把「违约」钉在一个已知关节上。
J_ONE = np.array([[1.0, 0.0]])


def _box(twist0, lower, upper):
    return exact_task_velocity(J_ONE, np.array([twist0]),
                               np.asarray(lower, float), np.asarray(upper, float))


def main():
    box = np.array([-1.0, -1.0]), np.array([1.0, 1.0])

    # 1) 超上界：worst_side=+1，且上界就是速度上限 ⇒ "speed"
    sol_up = _box(5.0, *box)
    assert sol_up.failure == FAIL_BOX, sol_up.failure
    assert sol_up.worst_joint == 0 and sol_up.worst_side == +1, \
        "超上界应记 worst_side=+1"
    assert abs(sol_up.worst_excess - 4.0) < 1e-12, sol_up.worst_excess
    assert box_binding(sol_up, *box, 1.0) == "speed"
    print("[1] 超上界: worst_joint=0 worst_side=+1 超出=%.2f -> speed"
          % sol_up.worst_excess)

    # 2) 超下界：worst_side=-1，下界就是速度上限 ⇒ "speed"
    sol_dn = _box(-5.0, *box)
    assert sol_dn.worst_joint == 0 and sol_dn.worst_side == -1, \
        "超下界应记 worst_side=-1"
    assert box_binding(sol_dn, *box, 1.0) == "speed"
    print("[2] 超下界: worst_side=-1 -> speed")

    # 3) 被顶的那一界明显小于速度上限 ⇒ 是行程收窄了它 ⇒ "range"
    lower3, upper3 = np.array([-2.0, -2.0]), np.array([0.5, 2.0])
    sol_rng = _box(5.0, lower3, upper3)
    assert sol_rng.worst_joint == 0 and sol_rng.worst_side == +1
    assert box_binding(sol_rng, lower3, upper3, 2.0) == "range"
    print("[3] 上界 0.5 < 速度上限 2.0 -> range")

    # 4) 判别性对照（worst_side 存在的理由）：关节 0 的**下界**被行程收窄到 -0.5，
    #    但它超的是**上界**、且上界 = 速度上限 ⇒ 正确答案是 speed。
    #    只比幅值、或只问「有没有关节被收窄」的实现会答 range。
    lower4, upper4 = np.array([-0.5, -2.0]), np.array([2.0, 2.0])
    sol_mix = _box(5.0, lower4, upper4)
    assert sol_mix.worst_joint == 0 and sol_mix.worst_side == +1
    assert box_binding(sol_mix, lower4, upper4, 2.0) == "speed", \
        "超的是上界(=速度上限)，下界被收窄不影响结论"
    # 对照组：同一个盒，只把「哪一界被顶」改成下界，答案必须翻过来 ——
    # 这一个来回才证明 worst_side 在起作用，而不是碰巧蒙对。
    wrong_side = TaskVelocitySolution(np.zeros(2), failure=FAIL_BOX, worst_joint=0,
                                      worst_excess=sol_mix.worst_excess, worst_side=-1)
    assert box_binding(wrong_side, lower4, upper4, 2.0) == "range", \
        "对照组必须给出相反的答案，否则本测无判别力"
    print("[4] 混合场景: 上界=上限/下界被收窄 -> speed；改成下界被顶 -> range（对照）")

    # 5) 非 FAIL_BOX / 无效关节 ⇒ 不下结论（None）
    assert box_binding(TaskVelocitySolution(np.zeros(2), failure=FAIL_RESIDUAL,
                                            worst_joint=0, worst_side=1),
                       *box, 1.0) is None
    assert box_binding(TaskVelocitySolution(np.zeros(2), failure=FAIL_BOX,
                                            worst_joint=-1, worst_side=1),
                       *box, 1.0) is None
    assert box_binding(sol_up, *box, 1.0) == "speed"
    print("[5] 非 box / worst_joint 无效 -> None")

    # 6) 告警句本身
    sol = TaskVelocitySolution(np.zeros(2), failure=FAIL_BOX, worst_joint=4,
                               worst_excess=9.21, worst_side=1)
    t_speed = describe_failure(sol, "right", "speed", 0.1625)
    assert "关节 5" in t_speed and "9.21" in t_speed, t_speed
    assert "速度上限" in t_speed, t_speed
    assert "不是奇异点" in t_speed, "σ_min 正常时必须明说不是奇异点: " + t_speed
    assert "奇异点外" not in t_speed, "旧文案那句错的补救方向必须消失: " + t_speed

    t_range = describe_failure(sol, "right", "range", 0.1625)
    assert "行程" in t_range and "工作空间内" in t_range, t_range
    assert "奇异点外" not in t_range, t_range

    t_near = describe_failure(sol, "right", "speed", 0.01)
    assert "接近奇异" in t_near, t_near
    assert NEAR_SINGULAR_SIGMA == 0.05

    # 判不出是哪一半时**不给补救方向**（不给 > 给错）
    t_unknown = describe_failure(sol, "right", None, None)
    assert "奇异点外" not in t_unknown and "工作空间内" not in t_unknown, t_unknown
    print("[6] 措辞: speed/range/未知/近奇异 四路；'奇异点外' 只在近奇异时出现")

    # 7) 回归：改动没有污染既有路径 —— 盒内可解时 worst_* 保持哨兵值
    ok = exact_task_velocity(J_ONE, np.array([0.5]),
                             np.array([-1.0, -1.0]), np.array([1.0, 1.0]))
    assert ok.achieved and ok.worst_joint == -1 and ok.worst_side == 0, \
        "解成功时 worst_joint/-side 必须是哨兵值"
    assert ok.failure == FAIL_NONE
    assert abs(ok.residual) < 1e-12, ok.residual
    # 零空间区间这条老路径照旧
    low, high, empty = null_space_interval(np.array([0.5, 0.0]), np.array([0.0, 1.0]),
                                           np.array([-1.0, -1.0]), np.array([1.0, 1.0]))
    assert (not empty) and low == -1.0 and high == 1.0
    print("[7] 回归: 盒内可解路径与 null_space_interval 未被改动影响")

    print("EXACT_IK TESTS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())

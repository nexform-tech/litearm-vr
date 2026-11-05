#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`status_msgs.py` 的契约单测（**纯 stdlib**，不需要硬件/zenoh）。

守四件事：
  1. `FollowStatus` 的 JSON 往返（旧的整体信封，仍被引用）；分设备 body 的
     `to_json` / `from_dict` 往返。
  2. `GripperStatus.ratio` / `position_mm` 进出正常 —— 采集侧现在**直接用**这两个
     值，不再自己拿弧度换算（各自拿常量换算就一定会有一方用的是别人的标定）。
  3. `target_ratio` 的**粘住**语义：链路一抖把它擦成 None，下游就把已收到的命令丢了。
  4. 「从未命令过」(None) 与「命令过 close」(0.0) 必须**可分**。
  5. `ArmStatus` 归一化 stm32 / sdk 两版。

用法: python3 test_status_msgs.py
"""
import json
import sys

import status_msgs as M


def test_roundtrip():
    gs = M.GripperStatus(device="gripper", connected=True, ok=True, q=-1.0595, tau=-0.085,
                         enabled=True, error_code=1, position_mm=79.975, ratio=0.9299,
                         target_ratio=0.0, age_ms=3.0)
    st = M.FollowStatus(seq=7, arms=[M.ArmStatus(device="left", q=[0.1] * 7,
                                                target=[0.2] * 7)], gripper=gs)
    back = M.FollowStatus.from_json(st.to_json())
    assert back.seq == 7
    assert back.arms[0].q == [0.1] * 7 and back.arms[0].target == [0.2] * 7
    assert back.gripper.target_ratio == 0.0

    # 分设备 body 是**直接** to_json（不套 FollowStatus 信封）
    body = M.GripperStatus.from_dict(json.loads(gs.to_json()))
    assert (body.position_mm, body.ratio, body.target_ratio) == (79.975, 0.9299, 0.0)
    assert body.q == -1.0595 and body.tau == -0.085
    print("[1] 往返：FollowStatus 信封 + 分设备 body 都对得上")


def test_gripper_command_intent_sticks():
    parsed = M.parse_gripper_reply("g", {"ok": True, "q": -1.05, "tau": -0.08,
                                         "enabled": True, "error_code": 1,
                                         "fresh": True, "age_ms": 2.0,
                                         "position_mm": 79.9, "ratio": 0.93,
                                         "target_ratio": 1.0})
    assert parsed.target_ratio == 1.0 and parsed.ratio == 0.93

    # 故障：实测冻结、但**粘住的意图必须留着**
    failed = M.parse_gripper_reply(
        "g", {"ok": True, "fresh": False, "q": 0.0, "error": "CAN 丢帧"}, parsed)
    assert failed.connected is False
    assert failed.q == parsed.q
    assert failed.target_ratio == 1.0, "故障回包不该把命令意图擦掉"
    assert failed.ratio == 0.93, "故障回包不该把实测比例擦掉"

    # 旧版服务（回包里没这些字段）：同样不该擦掉已经收到的值
    partial = M.parse_gripper_reply("g", {"ok": True, "q": -0.02, "fresh": True}, parsed)
    assert partial.target_ratio == 1.0, "缺字段 ≠ 没有命令"
    assert partial.ratio == 0.93

    # 从未命令过 = None，与 close 的 0.0 可分；也没读到过时 ratio 是 None 而不是 0.0
    never = M.parse_gripper_reply("g", {"ok": True, "q": 0.0, "fresh": True})
    assert never.target_ratio is None and never.ratio is None
    assert never.target_ratio != 0.0 and never.ratio != 0.0, \
        "None 与 0.0（全闭/命令过 close）必须可分 —— 混了就等于编了个假值"
    print("[2] target_ratio/ratio：解析 ✅ 故障保留 ✅ 缺字段保留 ✅ None≠0.0 ✅")


def test_arm_normalisation():
    # stm32 版：joints 是 (q,dq,tau,t_mos,t_coil,err) 的行 → 拆成并行数组
    stm32 = M.parse_arm_reply("left", {
        "ok": True, "state": "JOINT_FOLLOW", "flags": [], "enabled": True,
        "joint_fault": 0, "q": [0.1] * 7, "target": [0.2] * 7,
        "joints": [[0.1, 0.01, 0.5, 40.0, 41.0, 1] for _ in range(7)]})
    assert stm32.source == "stm32"
    assert stm32.joints.dq == [0.01] * 7 and stm32.joints.tau == [0.5] * 7

    # sdk 版：没有 joints/target，缺省 None（下游必须容忍）
    sdk = M.parse_arm_reply("left", {"ok": True, "state": "OK", "q": [0.3] * 7})
    assert sdk.joints is None and sdk.target is None and sdk.q == [0.3] * 7

    # 长度不是 7 的 q 一律不收（宁可 None，也不要半截向量）
    assert M.parse_arm_reply("left", {"ok": True, "q": [0.1, 0.2]}).q is None
    print("[3] ArmStatus 归一化：stm32 拆并行数组 ✅ sdk 缺省 None ✅ 非 7 长度不收 ✅")


def test_the_publish_topics_are_per_device():
    """分设备 topic 的拼法 —— 采集侧要按臂订**两条**，漂了要在这里炸。

    ⚠ `bc8d778` 起中段是 **arm_id**（`armA`/`armB`，由桥的臂清单给），不再是 kind：
    `{prefix}/{arm_id}/{channel}`。跟着改的是本用例，不是契约本身。
    """
    from status_aggregator import _CHANNEL, Device
    assert _CHANNEL["arm"] == "vr_arm"
    assert _CHANNEL["gripper"] == "vr_grip"
    assert _CHANNEL["hand"] == "vr_hand"
    assert _CHANNEL["arm"] != _CHANNEL["gripper"], \
        "臂与夹爪必须落在**不同**主题上，否则两端会静默解错"
    dev = Device(arm_id="armA", kind="gripper", host="127.0.0.1", port=9003, hz=20.0)
    assert dev.topic("litearm/v4") == "litearm/v4/armA/vr_grip", dev.topic("litearm/v4")
    print("[4] 分设备 topic：{prefix}/{arm_id}/{channel} → litearm/v4/armA/vr_grip")


def main():
    test_roundtrip()
    test_gripper_command_intent_sticks()
    test_arm_normalisation()
    test_the_publish_topics_are_per_device()
    print("STATUS_MSGS TESTS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())

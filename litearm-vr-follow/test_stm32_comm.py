#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主机 <-> STM32(litearm-stm32 固件) 串口通信自测。

验证两件事:
  1) 32 -> 主机: 固件上电即 100Hz 主动上报状态帧(0x40), 无需任何命令
  2) 固件版本: litearm SDK 在 connect() 时已取回并校验(前缀 Litearm + 最低版本)

底层是 **litearm SDK** (`github/litearm/litearm-python`, 包名 litearm), 不再依赖
已消失的 litearm-stm32/tools/arm_console.py。收发/解码/应答队列都由 SDK 负责。

用法:
  python3 test_stm32_comm.py                 # 默认 /dev/ttyACM0 (不存在则自动发现)
  python3 test_stm32_comm.py /dev/ttyACM0    # 显式指定

注意: 串口独占, 运行前确保没有别的进程(arm_follow_server 等)占着这个口。
通过后, 基于同协议建 server(与 VR 通信 + 向 32 通信)即可。
"""
import os
import sys
import time

from litearm import MIN_FW, Arm


def fmt_status(st):
    if st is None:
        return "(空)"
    parts = [f"seq={st.seq} mode={st.mode_name}({st.mode}) flags={st.flag_names or '-'}"]
    for i, j in enumerate(st.joints):
        parts.append(f"J{i + 1} q={j.q:7.3f} dq={j.dq:6.2f} τ={j.tau:6.2f} "
                     f"T={j.t_mos:5.1f}/{j.t_coil:5.1f}C e={j.err:02X}")
    return " | ".join(parts)


def test_passive(arm, duration=3.0):
    """测试 1: 被动读 100Hz 状态流 (32 -> 主机)。

    SDK 的读线程一直在收，`get_state()` 只取最近一帧的缓存；`seq` 是固件自己的
    帧计数，故"看到多少个不同的 seq"就是这三秒里真正收到了多少帧。
    """
    print(f"\n[1/2] 被动读状态帧 {duration:.0f}s (32 主动上报)...")
    end = time.time() + duration
    seen = set()
    last = None
    msg = None
    while time.time() < end:
        msg = arm.get_state()
        if msg.value is not None:
            seen.add(msg.value.seq)
            last = msg.value
            if len(seen) <= 3 or len(seen) % 20 == 0:
                print(f"  #{len(seen):3d} {fmt_status(last)}")
        time.sleep(0.02)
    n = len(seen)
    hz = getattr(msg, "hz", 0.0) if msg is not None else 0.0
    if n:
        print(f"  ✅ 收到 {n} 帧去重状态 (约 {n / duration:.0f} Hz; SDK 统计 {hz:.1f} Hz)")
    else:
        print("  ❌ 没收到任何状态帧 —— 检查接线/USB 枚举/是否被别的进程占用")
    return n, last


def test_firmware(arm):
    """测试 2: 固件版本。SDK 在 connect() 时已取回并校验，不再手发 0x41。"""
    print("\n[2/2] 固件版本 (SDK connect() 时已取回并校验)...")
    fw = getattr(arm, "firmware", None)
    ver = getattr(arm, "fw_version", None)
    if not fw:
        print("  ❌ 未取到固件版本字符串")
        return False
    ok = bool(ver) and tuple(ver) >= tuple(MIN_FW)
    print(f"  {'✅' if ok else '❌'} 固件: {fw}   解析版本={ver}  最低要求={MIN_FW}")
    return ok


def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    elif os.path.exists("/dev/ttyACM0"):
        port = "/dev/ttyACM0"
    else:
        port = None      # 交给 SDK 用 find_cdc_port() 自动发现

    print(f"打开 {port or '(自动发现 CDC)'} ...")
    try:
        arm = Arm(port).connect()
    except Exception as e:
        print(f"❌ 打开/握手失败: {e}")
        print("   检查: 串口是否被占用 / USB 枚举 / 固件是否在跑")
        sys.exit(1)
    print(f"  已连接, 固件={arm.firmware}")

    try:
        n, _ = test_passive(arm, duration=3.0)
        ok = test_firmware(arm)
    finally:
        arm.close()

    print("\n=== 结果 ===")
    print(f"  32->主机 状态流: {'PASS' if n else 'FAIL'}")
    print(f"  固件版本握手:    {'PASS' if ok else 'FAIL'}")
    if n and ok:
        print("  串口链路双向正常, 可以基于此建 server(VR 通信 + 向 32 通信)")
        print("  单臂遥操记得让 server 监听 9002 (metavr 只往 right 发)")
    else:
        print("  链路异常, 先排查串口/接线/固件")
        sys.exit(1)


if __name__ == "__main__":
    main()

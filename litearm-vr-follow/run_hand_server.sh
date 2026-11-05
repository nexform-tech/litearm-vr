#!/usr/bin/env bash
# 启动灵巧手(L6) 常驻 CAN 服务 (hand_server.py)，跑在灵巧手所在的工控机上。
#
# 与 arm_follow_server / gripper_server 同类：收 close/open(+side) 指令，经 socketcan
# 驱动 L6 灵巧手。灵巧手需先 CAN up（1Mbps classic，脚本不代劳，见下）。
#
# hand_grasp 来自 linkerbot SDK，需在 PYTHONPATH（默认 $HOME/sl/linkerbot-python-sdk-main，
# 可用 LINKERBOT_SDK 覆盖 SDK 目录）。
#
# 用法 (参数原样透传给 hand_server.py)：
#   ./run_hand_server.sh                                    # 按 follow.yaml 的 hand 段启动
#   ./run_hand_server.sh --can can0 --port 9004             # 临时覆盖
# 配置段在 follow.yaml 的 hand（FOLLOW_CONFIG 可换一份）；命令行 > follow.yaml > 出厂常量。
set -euo pipefail

# 切到脚本所在目录, 保证能找到 hand_server.py
cd "$(dirname "$0")"

# linkerbot SDK 路径：默认 ~/sl/linkerbot-python-sdk-main，可用 LINKERBOT_SDK 覆盖
LINKERBOT_SDK="${LINKERBOT_SDK:-$HOME/sl/linkerbot-python-sdk-main}"
if [ -d "$LINKERBOT_SDK" ]; then
    export PYTHONPATH="$LINKERBOT_SDK:$LINKERBOT_SDK/src${PYTHONPATH:+:$PYTHONPATH}"
fi

exec python3 follow_config.py --exec hand -- "$@"

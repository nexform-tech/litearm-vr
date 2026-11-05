#!/usr/bin/env bash
# 启动 pylitearm SDK 版遥操 server (arm_follow_server_sdk.py)。
#
# 修复本机 ROS humble 与 numpy2.x 的 pinocchio 冲突:
#   ROS humble 的 pinocchio 是按 numpy1.x 编译的, 而 pylitearm 需要 cmeel 的
#   pin 4.1.0 (numpy2.x 兼容)。本脚本把 cmeel 的 Python 路径和动态库路径顶到
#   sys.path / LD_LIBRARY_PATH 最前, 越过 ROS 旧版。
#
# 配置：读同目录 follow.yaml 的 arm_sdk 段（FOLLOW_CONFIG 可换一份），命令行给的参数覆盖
# 它 —— **命令行 > follow.yaml > 出厂常量**。日常改 follow.yaml 即可：
#   ./run_sdk_server.sh                                  # 按 follow.yaml 启动
# 临时覆盖仍可直接传（参数原样透传给 arm_follow_server_sdk.py）:
#   ./run_sdk_server.sh --dry-run --tcp-port 9001        # 无硬件验协议
#   ./run_sdk_server.sh --iface can0 --tcp-port 9001     # 真机(需 can0 up + 机械臂接好)
set -euo pipefail

# 切到脚本所在目录, 保证能找到 arm_follow_server_sdk.py
cd "$(dirname "$0")"

# cmeel 自带 pin 4.1.0 / libpinocchio / libeigenpy 顶到最前
CMEEL_SITE="$HOME/.local/lib/python3.10/site-packages/cmeel.prefix"
if [ ! -d "$CMEEL_SITE" ]; then
    echo "[run_sdk_server] 未找到 cmeel 目录: $CMEEL_SITE" >&2
    echo "                请确认 pin==4.1.0 (cmeel) 已安装: pip install pin==4.1.0" >&2
    exit 1
fi

export PYTHONPATH="$CMEEL_SITE/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$CMEEL_SITE/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

exec python3 follow_config.py --exec arm_sdk -- "$@"

#!/usr/bin/env bash
# 启动 STM32 直连版遥操 server (arm_follow_server_stm32.py)。
#
# 串口层用 **litearm SDK** (`github/litearm/litearm-python`, 包名 litearm, 只依赖
# pyserial) —— 不再依赖已消失的 litearm-stm32/tools/arm_console.py。
# 本脚本把 SDK 的 src 顶到 PYTHONPATH 最前; 若已 pip 装过 litearm, 这段会自动跳过。
#
# 配置：读同目录 follow.yaml 的 arm_stm32 段（FOLLOW_CONFIG 可换一份），命令行给的参数
# 覆盖它 —— **命令行 > follow.yaml > 出厂常量**。日常改 follow.yaml 即可：
#   ./run_stm32_server.sh                                # 按 follow.yaml 启动
# 临时覆盖仍可直接传（参数原样透传给 arm_follow_server_stm32.py）:
#   ./run_stm32_server.sh --mode j --tcp-port 9002
#   ./run_stm32_server.sh --mode mit --kp 50 50 50 50 50 50 50 --kd 2 2 2 2 2 2 2
#
# ⚠️ 单臂遥操时 metavr 只往 "right" 发 ⇒ 端口要用 9002 (或给 metavr --real-right)。
# ⚠️ 串口独占: 跑本 server 时别同时 cat /dev/ttyACM0 或开别的程序占串口。
set -euo pipefail

# 切到脚本所在目录, 保证能找到 arm_follow_server_stm32.py
cd "$(dirname "$0")"

# litearm SDK 源码目录。显式指定可用 LITEARM_PY_SRC=/path/to/litearm-python/src
LITEARM_PY_SRC="${LITEARM_PY_SRC:-$HOME/github/litearm/litearm-python/src}"
if [ -d "$LITEARM_PY_SRC/litearm" ]; then
    export PYTHONPATH="$LITEARM_PY_SRC${PYTHONPATH:+:$PYTHONPATH}"
elif ! python3 -c "import litearm" 2>/dev/null; then
    echo "[run_stm32_server] 找不到 litearm SDK: $LITEARM_PY_SRC/litearm" >&2
    echo "        装它: python3 -m pip install -e $HOME/github/litearm/litearm-python" >&2
    echo "        或指路: export LITEARM_PY_SRC=<litearm-python>/src" >&2
    exit 1
fi

exec python3 follow_config.py --exec arm_stm32 -- "$@"

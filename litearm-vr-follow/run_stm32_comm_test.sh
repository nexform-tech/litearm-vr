#!/usr/bin/env bash
# 主机 <-> STM32 串口通信自测 (test_stm32_comm.py)。
#
# 串口层用 **litearm SDK** (`github/litearm/litearm-python`) —— 不再依赖已消失的
# litearm-stm32/tools/arm_console.py。本脚本把 SDK 的 src 顶到 PYTHONPATH 最前;
# 若已 pip 装过 litearm, 这段会自动跳过。
#
# 用法 (参数原样透传给 test_stm32_comm.py):
#   ./run_stm32_comm_test.sh                # 用 follow.yaml 的 stm32_comm 段 (缺省自动发现串口)
#   ./run_stm32_comm_test.sh /dev/ttyACM0   # 命令行显式指定串口(覆盖 yaml)
#
# ⚠️ 串口独占: 跑前确保没有 arm_follow_server 占着这个口。
set -euo pipefail

# 切到脚本所在目录, 保证能找到 test_stm32_comm.py
cd "$(dirname "$0")"

# litearm SDK 源码目录。显式指定可用 LITEARM_PY_SRC=/path/to/litearm-python/src
LITEARM_PY_SRC="${LITEARM_PY_SRC:-$HOME/github/litearm/litearm-python/src}"
if [ -d "$LITEARM_PY_SRC/litearm" ]; then
    export PYTHONPATH="$LITEARM_PY_SRC${PYTHONPATH:+:$PYTHONPATH}"
elif ! python3 -c "import litearm" 2>/dev/null; then
    echo "[run_stm32_comm_test] 找不到 litearm SDK: $LITEARM_PY_SRC/litearm" >&2
    echo "        装它: python3 -m pip install -e $HOME/github/litearm/litearm-python" >&2
    echo "        或指路: export LITEARM_PY_SRC=<litearm-python>/src" >&2
    exit 1
fi

exec python3 follow_config.py --exec stm32_comm -- "$@"

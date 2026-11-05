#!/usr/bin/env bash
# 启动夹爪常驻 CAN 服务 (gripper_server.py)，跑在夹爪 DM 电机所在的工控机上。
#
# 与 arm_follow_server 同类：收 close/open 指令，经 socketcan 驱动夹爪 DM 电机。
# 夹爪 DM 电机需先 CAN up（1Mbps classic，脚本不代劳，见下）。
#
# 驱动走 litegrip-python SDK，需在 PYTHONPATH（默认 $HOME/sl/litegrip-python/src，
# 可用 LITEGRIP_SDK 覆盖 SDK 目录）；已安装 `litegrip` 包时无需设置。
#
# 配置：读同目录 follow.yaml 的 gripper 段（FOLLOW_CONFIG 可换一份），命令行给的参数覆盖
# 它 —— **命令行 > follow.yaml > 出厂常量**。日常改 follow.yaml 即可：
#   ./run_gripper_server.sh                                 # 按 follow.yaml 启动
# 临时覆盖仍可直接传（参数原样透传给 gripper_server.py）：
#   ./run_gripper_server.sh --can can0 --speed 20           # 调 open/close 速度 (默认 50 mm/s)
set -euo pipefail

# 切到脚本所在目录, 保证能找到 gripper_server.py
cd "$(dirname "$0")"

# litegrip SDK：`LITEGRIP_SDK` 显式指定优先，否则按候选路径探测，最后看有没有装成包。
#
# ⚠ 这里原来写死 `$HOME/sl/litegrip-python/src` —— 那个目录在本机**不存在**，
#   于是 `from litegrip import LiteGrip` 直接 ModuleNotFoundError、进程**秒退**，
#   而在 status_aggregator 那侧只看到 `:9003 Connection refused`：一个路径问题被
#   伪装成「服务没起」。所以探不到就**指名报错**，把候选路径与修法打出来。
LITEGRIP_CANDIDATES=(
    "$HOME/github/litegrip/litegrip-python/src"
    "$HOME/sl/litegrip-python/src"
    "$HOME/litegrip-python/src"
)
if [ -z "${LITEGRIP_SDK:-}" ]; then
    for _cand in "${LITEGRIP_CANDIDATES[@]}"; do
        if [ -f "$_cand/litegrip/__init__.py" ]; then
            LITEGRIP_SDK="$_cand"
            break
        fi
    done
fi

if [ -n "${LITEGRIP_SDK:-}" ] && [ -f "$LITEGRIP_SDK/litegrip/__init__.py" ]; then
    export PYTHONPATH="$LITEGRIP_SDK${PYTHONPATH:+:$PYTHONPATH}"
    echo "[夹爪服务] litegrip SDK: $LITEGRIP_SDK"
elif ! python3 -c "import litegrip" >/dev/null 2>&1; then
    {
        echo "找不到 litegrip SDK，而且它也没装成 python3 的包。找过："
        echo "  \$LITEGRIP_SDK"
        for _c in "${LITEGRIP_CANDIDATES[@]}"; do echo "  $_c"; done
        echo "  修法：LITEGRIP_SDK=/path/to/litegrip-python/src $0 $*"
        echo "        或 python3 -m pip install -e /path/to/litegrip-python"
    } >&2
    exit 1
fi

exec python3 follow_config.py --exec gripper -- "$@"

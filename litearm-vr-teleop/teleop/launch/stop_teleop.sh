#!/usr/bin/env bash
# 一键停止遥操服务端（含所有残留子进程）
set -euo pipefail

STOPPED=0
if pkill -f "[w]s_echo_server" 2>/dev/null; then
    STOPPED=1
fi
if pkill -f "[l]itearm_teleop.teleop_sim" 2>/dev/null; then
    STOPPED=1
fi

if [ "$STOPPED" = "1" ]; then
    sleep 2
    echo "[OK] 遥操服务端已停止"
else
    echo "[提示] 没有运行中的遥操服务端"
fi

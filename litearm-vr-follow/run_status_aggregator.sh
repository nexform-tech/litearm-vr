#!/usr/bin/env bash
# 启动 follow 整机状态汇总服务 (status_aggregator.py)：经 TCP 只读轮询各 follow
# 服务的状态（臂 ping / 夹爪 status），**按臂身份分 topic** 用 Zenoh pub 出去
# （armA→litearm/v4/armA/vr_arm，armA 的夹爪→litearm/v4/armA/vr_grip）。
#
# 只读：绝不发 start/q/movej/home/close/open，也绝不开串口/CAN。可与遥操发送端并行跑。
#
# zenoh 是 pip 装的（`pip install eclipse-zenoh`），无需 SDK PYTHONPATH；缺 zenoh 时
# 本脚本给出清晰报错。默认监听 tcp/0.0.0.0:17449（17447/17448 已被臂/夹爪遥操占用）。
#
# 配置：读同目录 follow.yaml 的 aggregator 段（FOLLOW_CONFIG 可换一份），命令行给的参数
# 覆盖它 —— **命令行 > follow.yaml > 出厂常量**。日常改 follow.yaml 即可：
#   ./run_status_aggregator.sh                                   # 按 follow.yaml 启动
# 临时覆盖仍可直接传（参数原样透传给 status_aggregator.py）：
#   ./run_status_aggregator.sh --arm armB=/dev/serial/…-if02@127.0.0.1:9002
#   ./run_status_aggregator.sh --gripper none                    # 关夹爪
#   ./run_status_aggregator.sh --prefix litearm/v4               # 改 topic 前缀
set -euo pipefail

# 切到脚本所在目录, 保证能找到 status_aggregator.py
cd "$(dirname "$0")"

# zenoh 缺失时给清晰提示（不静默失败）
if ! python3 -c "import zenoh" 2>/dev/null; then
    echo "错误: 未找到 zenoh 模块。请先 pip install eclipse-zenoh" >&2
    exit 1
fi

exec python3 follow_config.py --exec aggregator -- "$@"

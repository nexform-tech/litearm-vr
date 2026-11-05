#!/usr/bin/env bash
# metavr 环境一键重建脚本
#
# 用法: bash environments/setup.sh
# 前置: 已装 conda；本机网络注意事项见 docs/troubleshooting.md
#
# 通道策略：conda 26 的 `env create` 不支持 -c/--override-channels 且会
# 强制合并 ~/.condarc（本机指向的通道已 404 不可用），因此这里改用
# `conda create`（支持 -c/--override-channels）+ 显式镜像。
# 注：USTC anaconda 镜像已停用（现 302 跳转，conda 取 repodata 会 HTTP 000），
# 默认改用清华镜像；换源：CONDA_MAIN=… CONDA_R=… bash environments/setup.sh
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_DIR="$PROJECT_DIR/environments"

# conda 根：直接问 conda 自己，不猜——猜错会让环境建在一处、启动脚本去另一处找。
# 可用 CONDA_ROOT 覆盖。
CONDA_ROOT="${CONDA_ROOT:-$(conda info --base 2>/dev/null || true)}"
if [ -z "$CONDA_ROOT" ]; then
    echo "[错误] 没问到 conda 根目录（conda info --base 失败）。" >&2
    echo "       请先装 conda 并确保它在 PATH 里，或设 CONDA_ROOT 指到 conda 根目录。" >&2
    exit 1
fi
# conda 通道：默认清华镜像（USTC anaconda 镜像已停用，见 docs/troubleshooting.md）。
# 可用环境变量覆盖，例如走 Anaconda 官方源：
#   CONDA_MAIN=https://repo.anaconda.com/pkgs/main \
#   CONDA_R=https://repo.anaconda.com/pkgs/r bash environments/setup.sh
CONDA_MAIN="${CONDA_MAIN:-https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main}"
CONDA_R="${CONDA_R:-https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r}"

echo "=============================================="
echo " metavr 环境重建（metavr-arm）"
echo "=============================================="

# ---------- 1. 重建 conda 环境（已存在则先删除） ----------
echo "[1/3] 重建 metavr-arm 环境（python 3.10 + pip）..."
conda env remove -n metavr-arm -y 2>/dev/null || true
conda create -n metavr-arm -y python=3.10 pip \
    --override-channels -c "$CONDA_MAIN" -c "$CONDA_R"

P_ARM="$CONDA_ROOT/envs/metavr-arm/bin/python"

# ---------- 2. pip 依赖 + editable 安装 ----------
echo "[2/3] pip 依赖 + editable 安装 litearm-teleop..."
"$P_ARM" -m pip install -q -r "$ENV_DIR/metavr-arm-requirements.txt"
"$P_ARM" -m pip install -q -e "$PROJECT_DIR/src/litearm_teleop"

# ---------- 3. 验证 ----------
echo "[3/3] 验证..."
"$P_ARM" -c "import pybullet, websockets, litearm_teleop; print('  metavr-arm OK')"
# 出厂数据源是中间层 zenoh（--source zenoh），且 vr_source.py 里是延迟 import ——
# 装漏了要到真跑遥操才炸（表现为「一条帧都收不到」）。
"$P_ARM" -c "import zenoh; print('  zenoh 数据源依赖 OK')"
# pin 微分 IK 是默认后端，且 pin_ik.py 里是延迟 import——装漏了要到真跑遥操才炸
"$P_ARM" -c "import pinocchio, pink, qpsolvers; print('  pin 微分 IK 依赖 OK')"
echo ""
echo "=============================================="
echo " 环境重建完成。解释器: $P_ARM"
echo " 冒烟测试: make smoke"
echo "=============================================="

#!/usr/bin/env bash
# 把服务端冻成**自带解释器**的可执行文件（PyInstaller onedir），在 Docker 里构建。
#
#   teleop/packaging/frozen/build.sh                 # 构建
#   teleop/packaging/frozen/build.sh --rebuild-image # 改过 Dockerfile.builder 之后
#
# 产物：<仓库根>/dist/teleop-frozen/ —— **整棵目录一起发**，里面：
#   teleop-server      可执行文件（不带 .py，目标机上不需要 Python/conda）
#   共享库与扩展模块    全部平铺在它旁边（调用方只给 LD_LIBRARY_PATH=<该目录>）
# 用法：启动脚本 `launch/run_teleop_webxr.py --frozen <该文件的绝对路径>`，
#       或写进 config/teleop.yaml 的 env.frozen。
#
# 为什么在容器里：冻出来的东西链接**构建机**的 glibc，得固定 Ubuntu 22.04 +
# Python 3.10（与 metavr-arm 环境同一代）；目标机要能跑，就得是同一代发行版。
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"   # teleop/
REPO_DIR="$(cd -- "$PROJECT_DIR/.." && pwd)"                              # 仓库根
IMAGE="litearm-teleop-frozen:latest"
REBUILD_IMAGE=0

[ "${1:-}" = "--rebuild-image" ] && REBUILD_IMAGE=1

command -v docker >/dev/null || { echo "错误：没装 docker（构建需要它，跑不需要）" >&2; exit 1; }

if [ "$REBUILD_IMAGE" = 1 ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "[镜像] 构建 $IMAGE（ubuntu 22.04 + python3.10 + 运行期依赖）"
    # 构建上下文只取本目录：Dockerfile 不需要仓库里任何文件，
    # 拿仓库根当上下文会把 dist/ 几百 MB 一起发给 docker daemon。
    docker build -t "$IMAGE" -f "$PROJECT_DIR/packaging/frozen/Dockerfile.builder" \
        "$PROJECT_DIR/packaging/frozen"
else
    echo "[镜像] 复用 $IMAGE（要重建加 --rebuild-image）"
fi

echo "[构建] 在容器里跑 build_in_container.sh"
# --entrypoint 显式给容器内的绝对路径：镜像里烤着的那份可能是上一版目录名
# （packaging/nuitka/…），那时点了它就直接 "no such file or directory"。
# --user：产物落回宿主用户手里，别在仓库里留一堆 root 属主的文件。
# HOME=/tmp：pip --user 与 PyInstaller 的缓存都落在那里；容器里 uid 1000 的 HOME 不可写。
docker run --rm \
    --entrypoint /workspace/teleop/packaging/frozen/build_in_container.sh \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -v "$REPO_DIR:/workspace" \
    "$IMAGE"

echo
echo "[产物] $REPO_DIR/dist/teleop-frozen/"
du -sh "$REPO_DIR/dist/teleop-frozen" 2>/dev/null || true

# 冻结服务端的**构建环境**（不是产物镜像）。
#
# 为什么固定在 Ubuntu 22.04 + Python 3.10：冻出来的可执行文件链接的是**构建机**的
# glibc，在新发行版上打的包在 22.04/24.04 上起不来；而 metavr-arm 环境上游定的也是
# Python 3.10（见 environments/setup.sh）。
#
# 依赖照抄 environments/metavr-arm-requirements.txt —— 那份是唯一真源，这里手工同步；
# 改了那边要跟着改这里（构建容器只负责"能打包"，运行期依赖是被冻进产物的那一份）。
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential patchelf ccache \
        ca-certificates python3 python3-dev python3-pip \
        libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# 运行期依赖：装它们是为了让 PyInstaller 能看见并收进产物。
# （Nuitka 那套 `nuitka ordered-set zstandard` 已随方案切换删掉；现存镜像里还留着，
#   不影响构建，下次 `--rebuild-image` 之后就干净了。）
RUN python3 -m pip install --no-cache-dir --upgrade pip wheel \
 && python3 -m pip install --no-cache-dir \
        numpy scipy pybullet==3.2.7 websockets "eclipse-zenoh>=1.10" \
        "pin>=2.6.3" "pin-pink>=4.4.0" "qpsolvers>=4.3.1" quadprog loop-rate-limiters

# 注：PyInstaller 本身**不**装在这里，而是 build_in_container.sh 里 `pip install --user`
# 现装（几秒钟）—— 换打包器时不必重建这个十几分钟的镜像。要固化再挪进来。
WORKDIR /workspace
# 这个路径要跟仓库里 packaging/frozen/ 保持一致；build.sh 也会显式传 --entrypoint
# （镜像里烤着的路径可能还是上一版的，显式传就不吃这个亏）。
ENTRYPOINT ["/workspace/teleop/packaging/frozen/build_in_container.sh"]

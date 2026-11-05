# 环境准备与配置

> 本文面向**首次部署/新用户**：从零准备一台能跑通 LiteArm VR 遥操作系统的电脑。
> 已配置好的机器请直接看 [使用手册](user-guide.md)。

## 1. 系统要求

| 项 | 要求 | 说明 |
|---|---|---|
| 操作系统 | Ubuntu 22.04 LTS | ROS2 humble 官方支持 |
| ROS2 | humble（可选） | 仅 RViz2 可视化需要；无 ROS2 也能跑 pybullet 逆解 |
| Python 环境 | conda（`metavr-arm`，Python 3.10） | 遥操仿真核心 |
| 头显 | Meta Quest 3 | WebXR 浏览器手柄，与电脑同一 Wi-Fi |

> **最低配置**：只要 Python 环境 `metavr-arm` 即可运行遥操逆解（无 3D 显示）。
> **完整配置**：Python 环境 + ROS2 humble（RViz2 显示机械臂模型）。

## 2. 环境组成（两层依赖）

### 2.1 Python 环境 `metavr-arm`（必需）

遥操仿真的唯一运行时依赖，由 `environments/setup.sh` 一键创建：

| 依赖 | 版本 | 用途 |
|---|---|---|
| numpy | 2.2.x | 数值计算 |
| scipy | 1.15.x | 旋转向量/齐次变换 |
| pybullet | `==3.2.7` | 机械臂 IK + 仿真（**必须锁 3.2.7**，3.2.6 与 numpy 2 ABI 不兼容） |
| websockets | 16.x | WebSocket 遥操服务（`--source ws` 回退路径） |
| eclipse-zenoh | `>=1.10` | **出厂数据源**：订阅中间层 `litearm-vr-bridge` 转发的 `vr_frame` 话题 |
| pin | `>=2.6.3` | pinocchio，微分 IK 的模型/雅可比（`--ik pin` 后端；出厂后端是 `pybullet`） |
| pin-pink | `>=4.4.0` | pink，微分 IK 的任务/QP 框架（PyPI 名是 `pin-pink`，`pink` 是无关的格式化工具） |
| qpsolvers + quadprog | `>=4.3.1` | pink 的 QP 求解器后端 |
| loop-rate-limiters | — | pink 的控制周期节流 |

依赖清单唯一真源：`environments/metavr-arm-requirements.txt`。

> `setup.sh` 建完环境会把解释器路径打出来。三个入口（`setup.sh` /
> `launch/run_teleop_webxr.py` / `Makefile`）按同一套逻辑解析这个路径，
> 所以装完直接 `make smoke` 即可；环境在非常规位置时传 `--metavr-py <该环境的 python>`
> （或 `export METAVR_PY=<该环境的 python>`）。

### 2.2 ROS2 humble（可选，仅 RViz2 可视化）

- `robot_state_publisher`：读 URDF → 发布 TF
- `rviz2`：显示 RobotModel + TF

无 ROS2 时，`launch/run_teleop_webxr.py` 会降级为纯 pybullet 逆解（无 3D 窗口）。

## 3. 一键安装

```bash
# 1. Python 环境（需已装 conda；环境建在 conda 自己的根目录下）
bash environments/setup.sh

# 2. ROS2 humble（可选，Ubuntu 22.04）
# 参考官方安装：https://docs.ros.org/en/humble/Installation.html
# 安装 robot_state_publisher + rviz2：
sudo apt install ros-humble-robot-state-publisher ros-humble-rviz2
```

`environments/setup.sh` 会做三件事：重建 conda 环境 → 装 pip 依赖 + editable 安装 `litearm-teleop` → 验证 import。

## 4. 验证安装

```bash
# Python 环境冒烟测试（不弹窗口）
make smoke
# 期望输出末尾：SMOKE TEST PASSED

# 完整链路（RViz2 显示 + 遥操服务）
make teleop     # 或 ./launch/run_teleop_webxr.py（单臂）/ …… --dual（双臂）
```

## 5. 网络注意事项（本机镜像渠道）

| 资源 | 可用渠道 |
|---|---|
| conda 包 | 清华镜像（USTC anaconda 镜像已停用）：`--override-channels -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r` |
| pip 包 | 清华 PyPI（正常） |
| GitHub | `ghfast.top` 镜像 zip（注意默认分支） |
| HuggingFace / Meta | 不可达 |

> 镜像渠道已固化在 `environments/setup.sh`（默认清华），可用 `CONDA_MAIN` / `CONDA_R` 环境变量覆盖；
> 其他网络正常的机器可改回 `defaults`。

## 6. 快速开始（配置完成后）

```bash
make teleop          # 一键启动：清残留 → robot_state_publisher + rviz2 → 遥操服务（默认 zenoh 订阅）
                     # 帧要先有源：另开一个终端跑 litearm-vr-bridge（它收头显的 ws 8765）
make teleop ARGS="--source ws"   # 不接桥：本仓自己监听 8765，头显直连
make stop            # 停止
```

详见 [使用手册](user-guide.md) 与 [排障手册](troubleshooting.md)。

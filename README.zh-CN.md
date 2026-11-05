[English](README.md) | 简体中文

面向 **LiteArm V4 七轴机械臂**的实时 VR 遥操，可选配夹爪与 L6 灵巧手。戴上 **Quest 3**
头显，手柄怎么动，机械臂就怎么跟。

# LiteArm VR 遥操系统（litearm-vr）

本仓把完整链路放在一处：WebSocket 桥、逆运动学遥操服务、硬件跟随服务。克隆一次，
纯仿真和真机都从同一棵工作树跑起来。

---

## 需要什么

| 项目 | 要求 |
| --- | --- |
| 电脑 | Ubuntu 20.04 / 22.04，x86_64，带图形桌面，Python 3 |
| 头显 | Quest 3，运行 LiteArm VR 客户端（闭源，单独分发），与电脑在同一局域网 |
| 遥操环境 | [conda](https://docs.conda.io/)：遥操服务跑在独立的 `metavr-arm` 环境里 |
| 机械臂（可选） | LiteArm V4 七轴臂；可选夹爪、L6 灵巧手 |
| 接口（仅真机） | STM32 路线用 USB 串口（`/dev/ttyACM*`）；或 SocketCAN 适配器（`can0`）。经典 CAN，1 Mbps |
| 权限（仅真机） | `dialout` 组成员；CAN 配置与桥的一次性安装需要 `sudo` |
| 显示（可选） | ROS 2 Humble，仅 RViz2 可视化后端需要 |

硬件是可选的。只有电脑和头显也能把整条链路跑在仿真里；没有头显时还可以用仓库自带的
假数据源。

---

## 工作方式

```
Quest 3 头显（手柄位姿，约 72 Hz）
        │  WebSocket  ws://<PC-IP>:8765
        ▼
litearm-vr-bridge      收帧并转发，汇总链路状态                                    （可选）
        │  zenoh  litearm/v4/{arm_id}/vr_frame
        ▼
litearm-vr-teleop      手柄位姿 → 遥操映射 → 逆运动学 → 7 个关节角
        │  TCP/JSON  :9001 / :9002（臂）   :9003（夹爪）   :9004（灵巧手）
        ▼
litearm-vr-follow      关节角 / 开合指令 → 电机
        ├─ arm_follow_server_stm32.py / _sdk.py   臂（串口 / SocketCAN，二选一）
        ├─ gripper_server.py                      夹爪
        └─ hand_server.py                         灵巧手
        ▼
LiteArm V4 七轴臂 · 夹爪 · L6 灵巧手
```

**桥是可选组件。** 不带桥时，头显直接连遥操服务的 8765 端口。带桥时，桥占用 8765、
转发到遥操的 8766，并额外负责链路状态汇总、掉帧兜底和受管进程的启停。

---

## 目录结构

```
litearm-vr/
├── README.md / README.zh-CN.md            # 总览（English / 中文）
├── QUICKSTART.md / QUICKSTART.zh-CN.md    # 分步上手（English / 中文）
├── litearm-vr-bridge/                     # WebSocket → zenoh 桥、链路状态、进程控制
│   ├── scripts/{install.sh,up.sh}         # 一次性安装 / 日常启停
│   ├── src/litearm_vr_bridge/             # 桥源码
│   └── submodules/                        # 软链 → ../litearm-vr-teleop、../litearm-vr-follow
├── litearm-vr-teleop/                     # 头显遥操（手柄位姿 → 关节角）
│   ├── teleop/launch/run_teleop_webxr.py  # 一键启动脚本
│   ├── teleop/src/litearm_teleop/         # 逆解、遥操映射、仿真
│   └── teleop/config/teleop.yaml          # 全部可选项，含标定值
├── litearm-vr-follow/                     # 硬件服务（关节角 / 开合 → 电机）
│   ├── arm_follow_server_stm32.py / _sdk.py
│   ├── gripper_server.py / hand_server.py
│   ├── status_aggregator.py / status_listen.py
│   └── follow.yaml                        # 各服务默认值（端口、接口、增益）
├── third_party/litearm-can-dkms/          # vendored CAN 驱动，GPL-2.0-only（仅发行包）
├── packaging/                             # bundle 与冻结 teleop 构建脚本
├── scripts/                               # 本地与 CI 检查
└── docs/                                  # 文档索引
```

三个组件各自保留了自己的 README 与 `docs/`，内部细节以它们为准。

---

## 组件一览

| 组件 | 职责 | 主要入口 |
| --- | --- | --- |
| **litearm-vr-bridge** | 把头显帧经 zenoh 转给遥操；汇总链路状态；掉帧兜底、受管进程启停 | `litearm-vr-bridge/scripts/up.sh` |
| **litearm-vr-teleop** | 手柄位姿 → 遥操映射 → 逆运动学 → 7 关节角；3D 预览（pybullet / RViz2） | `litearm-vr-teleop/teleop/launch/run_teleop_webxr.py` |
| **litearm-vr-follow** | 关节角 / 开合指令 → 电机；只读状态汇总并发布到 zenoh | `litearm-vr-follow/run_{stm32,sdk,gripper,hand}_server.sh` |

---

## 通道与端口契约

| 边 | 通道 | 说明 |
| --- | --- | --- |
| 头显 → 桥 | WebSocket `ws://<PC-IP>:8765` | 头显客户端只说 WebSocket |
| 桥 → 遥操 | zenoh `litearm/v4/{arm_id}/vr_frame` | `arm_id` 默认 `armA`（`ZENOH_ARM_ID`） |
| 桥 ↔ 控制端 | zenoh `.../vr_status`、`.../vr_rpc` | 状态订阅、动作 queryable |
| 遥操 → 臂 | TCP `:9001`（左 / 单臂）与 `:9002`（右） | 一行一条 JSON |
| 遥操 → 夹爪 | TCP `:9003` | `{"cmd":"close"}` / `{"cmd":"open"}` |
| 遥操 → 灵巧手 | TCP `:9004` | `{"cmd":"close","side":"left"}` |
| 跟随 → 状态 | zenoh `litearm/v4/{armA,armB}/vr_{arm,grip,hand}` | 只读，按臂分 topic |

`arm_id` 才是「左 / 右」的归宿：跟随服务只知道自己打开的串口或 CAN 口，永远不知道自己是
哪条臂。身份由外部传入，跟随侧用 `--arm ARM_ID=[IFACE@]HOST:PORT`。

两端端口必须对齐：跟随服务的 `--tcp-port`（或 `follow.yaml`）与遥操启动脚本的
`--real-left` / `--real-right`（或 `teleop.yaml`）。

---

## 快速上手

完整步骤与排障见 [QUICKSTART.zh-CN.md](QUICKSTART.zh-CN.md)（English:
[QUICKSTART.md](QUICKSTART.md)）。最短路径：

```bash
git clone https://github.com/nexform-tech/litearm-vr.git
cd litearm-vr

# 1) 建遥操环境（首次，需 conda）
bash litearm-vr-teleop/teleop/environments/setup.sh

# 2) 纯仿真跑通（不接真机；头显连 ws://<PC-IP>:8765）
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py

# 3) 接真机：先起硬件跟随服务，再让遥操 --real 下发
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
litearm-vr-follow/run_stm32_server.sh --mode j --tcp-port 9001
litearm-vr-follow/run_gripper_server.sh --can can0 --port 9003
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --real \
    --gripper --gripper-remote 127.0.0.1:9003
```

---

## 安全须知

> **真机会真实运动。** 给机械臂上电前先读这一节。

- 开始真机操作前，确认机械臂正确上电、工作区无人、急停在手边。
- **串口与 CAN 独占。** 跑跟随服务时，不要让别的程序占用对应串口或 CAN 口。
- 臂的两版跟随服务（`stm32` 走串口、`sdk` 走 SocketCAN）驱动同一批电机，**互斥**，不能同跑。
- 掉帧时桥会让机械臂就地保持实测位姿（不发 MOVE_J、不产生突跳）；操作者主动停止仍走
  锁存停止。
- 桥管理的遥操带 `--real` 时，桥**不会**自动拉起上游链路；需显式执行
  `./scripts/up.sh --link`（非交互环境加 `--yes`）。
- 先在仿真里跑通，再让机械臂空载、低速测试，最后才做完整遥操。

---

## 文档与支持

- 分步上手与排障表：[QUICKSTART.zh-CN.md](QUICKSTART.zh-CN.md)
- 首次真机联调：`litearm-vr-bridge/docs/REAL-ARM-BRINGUP.md`
- 桥的设计与部署：`litearm-vr-bridge/docs/DESIGN.md`、`docs/DEPLOY.md`
- 遥操选项与逆解后端：`litearm-vr-teleop/README.md`
- 跟随服务协议与调参：`litearm-vr-follow/README.md`
- 跟踪模型与迁移说明：`litearm-vr-teleop/teleop/docs/anchored-tracking.md`

遇到问题请到对应组件所属仓库的 issue 区反馈，并附上启动命令、完整终端输出，以及
`follow.yaml` / `teleop.yaml`（发布前先抹掉凭据）。

---

## 关于本仓

本仓把原先分立的三个组件仓（`litearm-vr-bridge`、`litearm-vr-teleop`、
`litearm-vr-follow`）合并为一个自包含单体仓，便于一处克隆、一处跑通完整链路。三个子目录
就是各组件的工作树。

桥默认从 `<bridge>/submodules/litearm-vr-{teleop,follow}` 找上游；本仓里这两个路径是指向
同级组件的**软链**，所以开箱即用。若把组件挪到别处，用 `TELEOP_REPO` / `FOLLOW_REPO`
环境变量指到各组件根即可。

# litearm-vr-teleop — Quest 3 VR 遥操（手柄位姿 → 关节角）

当前控制采用固定握持锚点与受约束位姿跟踪；部署前请看[漂移修复与迁移说明](teleop/docs/anchored-tracking.md)。

基于 Quest 3 WebXR 手柄的机械臂遥操作系统。手柄位姿经**中间层 zenoh**（由
[`litearm-vr-bridge`](https://gitee.com/yudao_hz_1/litearm-vr-bridge) 从 8765 转发到
`litearm/v4/{arm}/vr_frame`）进来，做遥操语义（固定锚定位姿跟踪）+ 逆运动学解算，
输出 LiteArm V4 七轴机械臂的关节角——仿真里画出来，或者经 TCP 下发给真机跟随服务。
也可以 `--source ws` 退回「本仓库自己监听 8765、头显直连」。

本仓库**只管到关节角为止**。关节角怎么变成电机指令，是
[`litearm-vr-follow`](https://gitee.com/yudao_hz_1/litearm-vr-follow) 的事（见其 README 的 9001 协议契约）。

---

## 数据流

```
Quest 3 手柄(~72Hz)
      │ ws 8765
      ▼
litearm-vr-bridge ──zenoh litearm/v4/{arm}/vr_frame──▶ 本仓库（ws_echo_server.py）
      │  （默认数据源）                                        │
      │                          ├─ 遥操语义：速度式 delta 映射
      │                          ├─ IK：pybullet（默认）/ pinocchio + pink / exact
      │                          │
      │                          ├──▶ pybullet GUI 窗口 或 RViz2（看）
      │                          ├──▶ real_arm_client ──TCP 9001/9002──▶ arm_follow_server（动）
      │                          ├──▶ HandRemoteClient ──TCP 9004──▶ hand_server（灵巧手）
      │                          └──▶ GripperRemoteClient ──TCP 9003──▶ gripper_server（夹爪）
      │
      └──zenoh .../vr_status──▶ litearm-studio
```

- **默认 `--source zenoh`**：订阅桥转发的帧流（桥的协议见其
  `src/litearm_vr_bridge/protocol.py`）。话题 `litearm/v4/{arm_id}/vr_frame`，
  会话默认 **peer + 连 `tcp/127.0.0.1:7447`**（桥的监听端点）：peer 连不上只后台重试，
  所以**先起 teleop、后起桥也照样跑**（桥上来约 1s 内接上）。arm 段与端点都可配：
  `--arm-id` / `--zenoh-mode` / `--zenoh-connect` / `--zenoh-listen`。
- **`--source ws`**：本仓库自己监听 8765，Quest 客户端直连，不经过桥。
  适合「桥没起也要调遥操」或没有 zenoh 的机器。

---

## 功能特性

- **四种构型**：单臂正装（默认，`litearm_description.urdf`）、单臂水平侧装（`litearm_side.urdf`）、
  双臂侧装立柱（`dual_litearm_column_description`，左右臂镜像协作）、
  双臂正装（`dual_litearm_upright.urdf`，两只独立竖直臂、无立柱，home/映射复用单臂正装基准）
- **三种 IK 后端**（出厂 `--ik pybullet`，= metavr 的默认）：
  - `pybullet`（出厂）— 传统 `calculateInverseKinematics`，轻依赖，行为对齐 metavr
    （best-effort：未收敛也照常下发；无静止死区；外层只钳步长）
  - `pin` — pinocchio + pink，**策略对齐 metavr 的 `PinkIk`**：任务栈分层（FrameTask +
    可操作度任务 + PostureTask）、逐拍积分到收敛、QP 硬约束限位与速度。本仓比 metavr
    多支持双臂
  - `exact` — 精确任务速度（`exact_ik.py`）。**无阻尼**：twist 要么按手的速度精确
    产生、要么就地持位，**增益恒为 1 或 0、不随位姿变化**；幅值由关节速度盒界定，
    冗余自由度只在盒强迫时才花；失败分「盒不可行 / 残差超限 / 雅可比无方向」并指名关节
- **真机跟随**（`--real`）：解算出的关节角逐帧经 TCP/JSON 下发 `arm_follow_server`，
  gripActive 边沿映射为 `start` / `latch`。协议与 follow 仓库两版服务端完全一致
- **灵巧手 / 夹爪**（`--hand` / `--gripper`）：trigger 边沿经 TCP 远端驱动，
  按下=握拳/闭合，松开=张开。硬件驱动都在 `litearm-vr-follow`，本仓只发指令
- **两种显示后端**：pybullet 自带 3D 窗口（默认，零依赖）/ RViz2（需 ROS2，可叠加 TF）
- **速度式 delta 遥操**：gripActive 按下=启动/锚定，松开=暂停；手柄逐帧位姿差驱动，
  手柄停则机械臂停

---

## 快速开始

### 1. 安装环境（首次）

```bash
bash environments/setup.sh   # 一键重建 conda 环境 metavr-arm
make smoke                   # 冒烟测试（应输出 SMOKE TEST PASSED）
```

> ROS2 humble 仅 RViz 显示需要（可选）。详见 [环境准备与配置](docs/environment-setup.md)。

### 2. 启动遥操服务

```bash
./launch/run_teleop_webxr.py    # 一键启动（默认：单臂正装 + pybullet 逆解 + pybullet 窗口，不连真机）
make stop                        # 停止
```

默认数据源是中间层 zenoh，所以**先把桥起起来**（桥收头显的 `ws://<电脑IP>:8765`，
再转发到 `litearm/v4/armA/vr_frame`）：

```bash
# 不接桥、头显直连本机时，退回自建 WS 监听（8765）
./launch/run_teleop_webxr.py --source ws
```

### 3. 按需要打开真机 / 末端执行器

启动脚本的参数与 `--help` 一一对应，全部直通：

```bash
./launch/run_teleop_webxr.py --real                        # 连真机（关节角下发 arm_follow_server）
./launch/run_teleop_webxr.py --real --hand --hand-remote 192.168.66.95:9004   # 真机 + 灵巧手
./launch/run_teleop_webxr.py --real --gripper --gripper-remote 192.168.66.95:9003  # 真机 + 夹爪
./launch/run_teleop_webxr.py --dual                        # 双臂侧装立柱
./launch/run_teleop_webxr.py --dual --upright               # 双臂正装（两只独立竖直臂）
./launch/run_teleop_webxr.py --ik exact --deadband-scale 0 # 精确任务速度 + 关死区
```

`make teleop` 也一样，用 `ARGS` 直通：`make teleop ARGS="--dual --viewer rviz"`。

---

## 配置

**全部可选项都写在 [ `teleop/config/teleop.yaml` ](teleop/config/teleop.yaml)**（含单位、
取值范围与「为什么是这个值」），启动脚本与服务端都读它，用 `--config` 换一份：

```bash
cp teleop/config/teleop.yaml teleop/config/my_arm.yaml
./launch/run_teleop_webxr.py --config teleop/config/my_arm.yaml
./launch/run_teleop_webxr.py --port 9000     # 命令行只用来临时覆盖单项
```

优先级：**命令行 > 配置文件 > 代码里的出厂常量**。布尔项都支持 `--x / --no-x`，
所以配置文件里写 `true` 也能从命令行关掉。配置段落：`server` / `vr`（数据源：zenoh 话题与会话）、
`env` / `robot`（含单双臂、侧装/正装）、`tuning`（手感）、`sim`（频率与限幅）、`mapping`
（**标定值**：三套轴映射）、`pin_ik` / `exact_ik`、`real` / `effector` / `controller`。

配置里标着「标定值」的段落是对着真机量出来的，没有重新标定别改。加新选项要动的地方
与两张「选项 → 配置路径」表的说明见 [开发指南](teleop/docs/development.md#配置文件configteleopyaml)。

---

## 启动方式详解

所有参数见 `./launch/run_teleop_webxr.py --help`（默认值见上面的配置文件）。

### 构型

| 构型 | 命令 | 说明 |
|---|---|---|
| **单臂正装**（默认） | `./launch/run_teleop_webxr.py` | 仅右手柄遥操单臂 |
| 单臂侧装 | `--side` | 只接一只**左**臂：占 left 槽（左手柄驱动、真机走 `:9001`），home/映射取双臂侧装左臂。与 `--dual` 同给时按双臂 |
| 双臂侧装立柱 | `--dual` | 左右手分别遥操左右臂（镜像协作） |
| 双臂正装（无立柱） | `--dual --upright` | 两只独立竖直臂，home/映射复用单臂正装基准，右臂镜像 |

### IK 后端

| 后端 | 参数 | 依赖 | 增益（手→臂） |
|---|---|---|---|
| **pybullet**（默认） | `--ik pybullet` | 仅 pybullet | ~0.44~0.98（见下方「已知缺陷」） |
| pinocchio + pink | `--ik pin` | `pin` / `pin-pink` / `qpsolvers` / `quadprog` | 0.9998（逐拍积分到收敛） |
| `exact` 精确任务速度 | `--ik exact` | 无（纯 NumPy） | **1.000**，或 0（持位） |

> **已知缺陷（pybullet 后端）**：它用绝对迭代 IK，接受容差 3 mm / 0.05 rad，而目标每帧
> 从**已到位的位姿**重建 —— 于是每帧的 IK 残差被永久丢掉。实测手→臂交付比只有
> **0.435 / 0.462 / 0.480**（5 / 10 / 20 cm/s），即手移 10 cm 臂只走约 4.4 cm。
> 它只是轻依赖回退；要跟手请用 `pin` 或 `exact`。

### 速度与步长限制

两个配置键（**不是**命令行参数），都在 `teleop/config/teleop.yaml`：

| 配置键 | 出厂 | 作用 |
|---|---|---|
| `sim.max_step_rad` | `0.2` rad/帧（60 Hz ⇒ 12 rad/s） | 每帧关节步长上限，**防 IK 跳变**，与速度无关。`--ik pybullet` 与 `--ik pin` 共用（metavr 也在 IK 分派之后只钳一次）；出厂 0.2 是 metavr 的值，速度式 delta 下被削掉的位移不会补回，实测 0.08 在 5cm 快挥时只到位 40~86% |
| `exact_ik.max_joint_vel` | `10.0` rad/s | **只被 `--ik exact` 用**（速度盒 = 它与「行程/dt」的交集）。⚠ metavr 没有这个量，所以它在 `exact_ik:` 段而不是全局 `sim:` |

**为什么限速度**：一是和真机同速。真机被 `arm_follow_server` 的梯形 slew 限在
`--max-speed 2.0` / `--max-accel 3.0`，仿真不限速就会「大幅挥手瞬时到位」，看着对、真机跟不上。
二是 `--ik exact` 的判据本身就取自它：手速要的关节速度超出速度盒 ⇒ **就地持位**并印
`[持位][box]`，大声停下，而不是静默欠交付。

**为什么限步长**：与速度无关的兜底。7 轴冗余换解、或经过近奇异位姿时，一帧的解可能跳
1 rad 以上，`sim.max_step_rad` 把这种跳变削平。

**想关掉速度这一界**：把 `exact_ik.max_joint_vel` 从出厂 `10.0` 调到 **`99.0`**
（60 Hz 下 `99·dt ≈ 1.65 rad/帧`，已远大于任何可达的关节步长 ⇒ 等价于关掉）。
⚠ 三条前提：

- **`--ik pin` 不受影响**：它的上限是 pink 的 `VelocityLimit`（配置里
  `pin_ik.velocity_limit`，出厂 `1.0` = URDF 值），要一起放开得改那里。
- **不会让真机变快**：真机链上另有两层 —— `arm_follow_server` 的 `--max-speed` /
  `--max-accel`，以及固件/电机自身的限幅。只改这里的唯一效果是**仿真不再因速度而持位**：
  目标更激进，真机照旧削回。
- **别顺手放开 `sim.max_step_rad`**：它挡的是跳变，不是速度。

### 显示后端

| 后端 | 参数 | 依赖 | 说明 |
|---|---|---|---|
| **pybullet GUI**（默认） | `--viewer gui` | 仅 Python 环境 | 自带 3D 窗口，鼠标拖拽旋转/滚轮缩放 |
| RViz2 | `--viewer rviz` | + ROS2 humble | 显示 RobotModel + TF，可叠加更多可视化 |

### 真机 / 灵巧手 / 夹爪

| 参数 | 默认 | 说明 |
|---|---|---|
| `--real` | 关 | 启用真机桥接（`real_arm_client.py`，TCP/JSON 下发关节角） |
| `--real-left` / `--real-right` | `127.0.0.1:9001` / `:9002` | 左右臂 `arm_follow_server` 地址 |
| `--hand` | 关 | 启用灵巧手（trigger 边沿 → 握拳/张开） |
| `--hand-remote` | 空 | 灵巧手服务 `hand_server.py` 地址，`--hand` 时必填（灵巧手在另一台工控机的总线上） |
| `--gripper` | 关 | 启用夹爪（trigger 边沿 → 闭合/张开） |
| `--gripper-remote` | 空 | 夹爪服务 `gripper_server.py` 地址，`--gripper` 时必填（夹爪在另一台工控机的总线上） |

> 灵巧手与夹爪都是同一根 trigger 边沿的消费者，**通常只启用实际装在臂上的那个**。
> 两者的硬件驱动都在 `litearm-vr-follow` 仓（`hand_server.py` / `gripper_server.py`），
> 本机经 TCP 发指令，**不碰 CAN、不装 SDK**。

### 命令行参数总表（`launch/run_teleop_webxr.py`）

启动脚本**只用标准库**，可由系统 `python3` 直接跑；它自己负责找到 metavr-arm 环境。
完整表见 `./launch/run_teleop_webxr.py --help`。

| 参数 | 默认 | 说明 |
|---|---|---|
| `--metavr-py PATH` | 自动解析 | metavr-arm 环境的 python；不传则按 `CONDA_PREFIX` → `conda info --base` → 常见 conda/venv 根 的顺序找 |
| `--source` | `zenoh` | 数据源：`zenoh`=订阅中间层帧流（桥转发）/ `ws`=自建 WS 监听，头显直连 |
| `--arm-id` | `armA` | zenoh 话题里的 arm 段（`litearm/v4/{arm_id}/vr_frame`）。必须与桥的 `ZENOH_ARM_ID` 一致，**填错不报错、只是收不到帧** |
| `--zenoh-mode` | `peer` | zenoh 会话模式：`peer`=连不上只后台重试，**桥后起也能接上** / `client`=开会话时就要求连上（桥没起会报错退出）/ `router` |
| `--zenoh-connect` | `tcp/127.0.0.1:7447` | zenoh 主动连的端点，逗号分隔（默认 = 同机桥的监听端点）；桥在别的机器就写那台 IP |
| `--zenoh-listen` | 空 | zenoh 监听端点（默认不监听：同机的桥已占 7447） |
| `--host` | `0.0.0.0` | WS 监听地址（`--source ws` 时用） |
| `--port` | `8765` | WS 监听端口（`--source ws` 时用） |
| `--dual` | 关 | 双臂（默认单臂正装）；配 `--upright` 为双臂正装，否则双臂侧装立柱 |
| `--upright` | 关 | 与 `--dual` 同用：双臂正装（两只独立竖直臂、无立柱）；仅 `--dual` 时生效 |
| `--side` | 关 | 单臂侧装 = 单左臂（左手柄驱动、真机走 `:9001`；与 `--dual` 同给时按双臂） |
| `--viewer` | `gui` | `gui`=pybullet 窗口，`rviz`=RViz2（需 ROS2） |
| `--ik` | `pybullet` | `pybullet`（出厂，= metavr 的 `IK_BACKEND`）/ `pin` / `exact` |
| `--tcp-offset X Y Z` | `0 0 0.18` | 末端偏移量（米，本体系，沿工具轴 +z）：控制的工具点相对 tip link 的偏移。**出厂 0.18 = 本台架夹爪的法兰→抓取点距离**；`0 0 0` = 直接控法兰 |
| `--deadband-scale` | `1.0` | 静止死区阈值乘数；`0`=关掉（**会放行追踪噪声**）。出厂值在 60Hz 下等于 1.8cm/s / 0.18rad/s |
| `--smooth-alpha` | `0.5` | 增量 EMA 平滑的新样本权重，取值 `(0, 1]`；`1.0`=不平滑（最跟手也最抖）。⚠ `0` 会让臂完全不动 |
| `--resume-margin` | `1.0` | `--ik exact` 的持位滞回（`1.0`=朴素规则） |
| `--task-residual-tolerance` | `1e-3` | `--ik exact` 判定「任务被产生」的残差比例上限 |
| `--manip-weight` | `1e-2` | `--ik pin` 的操纵度任务权重（把冗余推向雅可比更良态的构型，**预防**奇异）。出厂 `1e-2`（metavr 该任务常开）；写 0 可关掉。调大会与 FrameTask 抢优先级 |
| `--control-frame` | `tool` | 旋转增量读在哪个系：`tool`=末端局部系（出厂/现状）/ `base_link`=基座系（绕**世界轴**转，与平移同源） |
| `--base-link-signs X Y Z` | `-1 -1 1` | `--control-frame base_link` 用的逐轴旋转符号（初值借自 teleop-vr 实测，**本台架未重测**） |
| `--real` | 关 | 启用真机桥接 |
| `--real-left` / `--real-right` | `127.0.0.1:9001` / `:9002` | 左右臂 follow 服务地址 |
| `--hand` | 关 | 启用灵巧手（trigger 边沿 → 握拳/张开） |
| `--hand-remote HOST:PORT` | 空 | 灵巧手服务 `hand_server.py` 地址（`--hand` 时必填，如 `192.168.66.95:9004`） |
| `--gripper` | 关 | 启用夹爪（trigger 边沿 → 闭合/张开） |
| `--gripper-remote HOST:PORT` | 空 | 夹爪服务 `gripper_server.py` 地址（`--gripper` 时必填，如 `192.168.66.95:9003`） |

### 直接调用（不走一键脚本）

```bash
# 遥操服务（数据源默认 zenoh；真机/灵巧手/夹爪开关见 --help）
python scripts/ws_echo_server.py --source zenoh --arm-id armA \
    --zenoh-mode peer --zenoh-connect tcp/127.0.0.1:7447 \
    --viewer gui --ik pin [--dual] [--upright] [--side] [--no-ros] \
    [--real --real-left 127.0.0.1:9001 --real-right 127.0.0.1:9002] \
    [--hand --hand-remote host:port] \
    [--gripper --gripper-remote host:port]

# 头显直连本机（不经桥、不用 zenoh）
python scripts/ws_echo_server.py --source ws --host 0.0.0.0 --port 8765 \
    --viewer gui --ik pin [--no-ros]

# 纯仿真（不经任何数据源，含 --viewer / --ik / --dual / --upright / --side）
python -m litearm_teleop.teleop_sim --dual --upright --viewer gui --ik pin
```

---

## 遥操操作

| 手柄输入 | 行为 |
|---|---|
| **gripActive 按下** | 启动/锚定（记录当前手柄位姿为参考） |
| 按住期间移动手柄 | 机械臂跟随（速度式 delta 映射）；`REAL=1` 时同时下发给真机 |
| **gripActive 松开** | 暂停 |
| **trigger 按下** | 灵巧手握拳 / 夹爪闭合（需 `HAND=1` / `GRIPPER=1`） |
| **trigger 松开** | 灵巧手张开 / 夹爪张开 |

trigger 是模拟量：超过阈值才算「按下」，且带消抖，过滤抖动。

陷入奇异时：暂停 → 手柄回中立位 → 重新按下 gripActive 重新锚定。
（`IK=pin` 后端有 LM 阻尼 + 零空间正则，抗奇异能力比 `pybullet` 好得多。）

---

## 目录结构

```
litearm-vr-teleop/
└── teleop/                            # 遥操工程：模型描述 + 源码 + 启停脚本
    ├── docs/                          # 架构 / 环境 / 使用 / 排障 / 开发
    ├── src/litearm_teleop/            # 自有源码：数据源 + 遥操语义 + IK
    │   ├── teleop_sim.py              # 单/双臂遥操语义 + 轴映射常量 + 三个 IK 后端的分派
    │   ├── vr_source.py               # VR 数据源：订阅中间层 zenoh 的 vr_frame 话题
    │   ├── pin_ik.py                  # pinocchio + pink 微分 IK 后端
    │   └── exact_ik.py                # 精确任务速度后端（无阻尼，增益 1 或 0）
    ├── launch/
    │   ├── run_teleop_webxr.py        # 一键启动（仅标准库；参数取代原环境变量）
    │   └── stop_teleop.sh             # 停止
    ├── scripts/
    │   ├── ws_echo_server.py          # 遥操服务（zenoh 订阅或 WS 监听 → 同进程驱动 teleop_sim）
    │   ├── real_arm_client.py         # 真机桥接：关节角 → arm_follow_server（TCP/JSON）
    │   ├── ros_joint_state.py         # ROS2 /joint_states 发布（RViz 用）
    │   └── decimate_meshes.py         # STL 降采样（生成 meshes_lowpoly）
    ├── models/                        # 模型描述（几何真源，各为独立 ROS2 包）
    │   ├── litearm_description/       # 单臂（上游 litearm-description 的 SE 包）
    │   │   ├── urdf/litearm_description.urdf   # 正装（joint1~7 / ee_link）
    │   │   ├── urdf/litearm_side.urdf # 水平侧装（正装的派生物）
    │   │   ├── meshes/                # 高精度 STL（RViz 用）
    │   │   └── meshes_lowpoly/        # 降采样 STL（pybullet 渲染用）
    │   └── dual_litearm_column_description/    # 双臂 URDF + STL：侧装立柱 / 正装（无立柱），
    │                                          #   两变体复用同一 macro 与 mesh
    ├── environments/                  # conda 环境定义 + 一键重建
    ├── tests/                         # 冒烟测试
    └── Makefile  CHANGELOG.md
```

---

## 环境准备

首次部署请先看 [环境准备与配置](docs/environment-setup.md) —— 系统要求、依赖清单、一键安装、网络镜像渠道。

```bash
bash environments/setup.sh   # 一键重建 conda 环境 metavr-arm
make smoke                   # 冒烟测试
```

三个入口（`setup.sh` / `run_teleop_webxr.py` / `Makefile`）找的是**同一个环境**，
不用手工对齐路径：

- `setup.sh` 直接问 conda 根目录在哪（`conda info --base`），**不猜**——环境就建在那里
- 启动脚本和 `make` 按 `--metavr-py`（或 `METAVR_PY`） → 已激活的同名环境 → `conda info --base`
  → 常见 conda/venv 根 的顺序解析

所以 `bash environments/setup.sh` 之后，`make smoke` / `make teleop` 直接就能用，
不需要再告诉它们环境在哪。conda 装在非常规位置时给个提示即可：

```bash
export METAVR_PY=~/myconda/envs/metavr-arm/bin/python   # 或
make PY_ARM=~/myconda/envs/metavr-arm/bin/python smoke
```

### 运行时依赖（Python 包之外）

| 能力 | 额外需要 |
|---|---|
| 中间层 zenoh 数据源（**出厂** `--source zenoh`） | 同机跑着的 `litearm-vr-bridge`（它 listen `tcp/0.0.0.0:7447`）+ 环境里的 `eclipse-zenoh`（`environments/metavr-arm-requirements.txt` 已含） |
| RViz2 显示 | ROS2 humble |
| 真机跟随 | `litearm-vr-follow` 的 `arm_follow_server` 在 `REAL_LEFT`/`REAL_RIGHT` 上监听 |
| 灵巧手 | `litearm-vr-follow` 的 `hand_server.py` 在 `--hand-remote` 上监听（本机不需要 CAN/SDK） |
| 夹爪 | `litearm-vr-follow` 的 `gripper_server.py` 在 `--gripper-remote` 上监听（本机不需要 CAN） |

---

## 文档索引

- [环境准备与配置](docs/environment-setup.md) — 系统要求 / 依赖 / 安装 / 网络镜像
- [系统架构](docs/architecture.md) — 数据流 / 遥操语义 / 坐标系映射
- [使用手册](docs/user-guide.md) — 日常操作与常见问题
- [排障手册](docs/troubleshooting.md) — 全部实战踩坑记录
- [开发指南](docs/development.md) — 目录 / 配置旋钮 / 模型修改

# 开发指南

## 目录结构

```
teleop/
├── docs/                              # 架构 / 环境 / 使用 / 排障 / 开发
├── config/
│   └── teleop.yaml                    # 全部可选项的唯一真源（启动脚本与服务端都读它）
├── src/litearm_teleop/                # 自有源码（唯一 PC 端程序）
│   ├── teleop_sim.py                  # pybullet 遥操仿真：遥操语义 → IK → 驱动
│   ├── config.py                      # 配置文件读取（stdlib-only YAML 子集解析器）
│   └── setup.py                       # 包定义（editable 安装，不打包 URDF）
├── models/                            # 模型描述（几何真源，每个都是独立的 ROS2 包）
│   ├── litearm_description/           # 单臂（上游 litearm-description 的 SE 包）
│   │   ├── urdf/litearm_description.urdf   # 正装（关节名 joint1~7 / ee_link）
│   │   ├── urdf/litearm_side.urdf     # 水平侧装（= 正装 + world/mount 固定关节）
│   │   ├── meshes/                    # 高精度 STL（RViz 用）
│   │   └── meshes_lowpoly/            # 降采样 STL（pybullet 渲染用）
│   └── dual_litearm_column_description/    # 双臂侧装立柱（含立柱）
│       ├── urdf/                      # xacro 源 + 生成好的 .urdf
│       └── meshes/ + meshes_lowpoly/  # 同上分工
├── launch/                            # 一键启停脚本（run_teleop_webxr.py / stop_teleop.sh）
├── scripts/                           # 遥操服务（zenoh 订阅 / WS 监听）、ROS2 发布、动作辅助脚本
├── environments/                      # conda 环境定义 + 一键重建
├── tests/                             # 测试
├── Makefile                           # 常用命令
└── CHANGELOG.md
```

## 环境

单环境 `metavr-arm`（python 3.10）：pybullet 3.2.7 + websockets + pinocchio/pink（微分 IK）。
依赖清单唯一真源：`environments/metavr-arm-requirements.txt`。
重建：`bash environments/setup.sh`（editable 安装 `src/litearm_teleop`）。

环境解释器由 `--metavr-py`（或 `METAVR_PY`）统一指定。`setup.sh` /
`run_teleop_webxr.py` / `Makefile` 三个入口按同一套逻辑解析（显式指定 → 已激活的同名
环境 → `conda info --base` → 常见 conda/venv 根），换机器不用逐个改脚本。

**移动目录后必须重装 editable 包**（在 `metavr-arm` 环境里执行）：
```bash
python -m pip install -e src/litearm_teleop
```

忘了也不要紧：`run_teleop_webxr.py` 启动前会 import 一次 `litearm_teleop` 并核对它是否
就是**本检出**这一份（`check_package`），不是就指名报错并退出。加这道检查是因为两种
失败都很难查：没装时服务端抛一片 `ModuleNotFoundError` Traceback（看着像服务端崩了），
而装到了**别的检出**时更糟 —— editable 安装是 `MetaPathFinder`，优先级高于 `PYTHONPATH`
和 `sys.path`，`PYTHONPATH=src` 压不住它，于是**一声不响地跑旧代码**。

## 常用命令

```bash
make teleop       # 一键启动 LiteArm 遥操服务端（单进程）
make stop         # 停止
make smoke        # pybullet 环境冒烟测试（DIRECT 模式，无需显示器）
make test-exact   # 纯函数单测（告警措辞 / 盒判定，秒级）
make test-config  # 配置单测（配置文件 == 代码常量 / 解析器 / 优先级，秒级）
make test         # 全部
```

## 配置文件（`config/teleop.yaml`）

**所有可调项的默认值都写在这一个文件里**，含单位、取值范围与「为什么是这个值」。
读取器是 `src/litearm_teleop/config.py`（**只用标准库**，自带 YAML 子集解析器）——
启动脚本要在 metavr-arm 建起来之前用系统 python3 跑，不能依赖第三方包，所以没引 pyyaml。

优先级：**命令行 > 配置文件 > 代码里的出厂常量**。实现方式是「命令行没给过的项才
从配置取」，所以「没给」与「显式给了出厂值」是分得开的（见各处 argparse 的 `default`
不加默认值 / `argparse.SUPPRESS`）。

段落划分（与代码分层对齐）：

| 段 | 落到哪 |
|---|---|
| `server` / `env` / `robot` | 启动脚本（host/port、解释器、单双臂与侧装/正装、显示后端） |
| `tuning` | `teleop_sim.Tuning`（死区、平滑、坐标系、滞回、精确 IK 容差） |
| `sim` | `teleop_sim.SimConfig`（频率、限幅、低通、home、收敛容差） |
| `mapping` | `teleop_sim.SimConfig` 的轴映射（**标定值**，侧装/正装/单臂各一套） |
| `pin_ik` / `exact_ik` | 两个 IK 后端的求解器参数 |
| `real` / `effector` / `controller` | 真机桥接、灵巧手/夹爪远端地址、手柄语义 |

### 加一个新选项要动的地方

1. 代码里给它一个**出厂常量**（`teleop_sim.py` 模块顶部或对应类的 `__init__` 默认值）；
2. `config/teleop.yaml` 里写上同名键，值 = 该常量，并写清单位与作用；
3. 若它要能被命令行覆盖：在 `launch/run_teleop_webxr.py` 的 `OPTION_PATHS`（和/或
   `scripts/ws_echo_server.py` 的 `_OPTION_PATHS`）里加一行「选项 → 配置路径」；
4. 跑 `tests/test_config.py` —— 它会逐位比对「配置文件的值 == 代码常量」，并检查两张
   路径表指向的键真实存在（路径写错只会让选项**静默失效**，不报错，所以必须测）。

## 关键配置旋钮

**改这些一律改 `config/teleop.yaml`，不要改 `teleop_sim.py` 的模块常量** —— 常量只是
「配置文件里连键都没有时」的兜底，改了不会生效，而且会和配置文件漂移（有测试守着）。

| 配置文件里的键 | 作用 |
|---|---|
| `robot.dual` / `robot.upright` | 双臂侧装立柱 / 双臂正装（无立柱）；都不给 = 单臂正装 |
| `robot.side` | 单臂侧装 = 单左臂（home/映射取双臂侧装左臂，真机走 :9001） |
| `mapping.axis_remap` | 手柄→机器人旋转映射（pinv(H_R)·R·H_R 约定） |
| `mapping.translation_remap` | 手柄→机器人平移映射（单臂正装基准） |
| `mapping.dual_translation_remap_left` / `_right` | 双臂侧装平移映射（镜像协作，左右臂各自独立） |
| `mapping.upright_translation_remap_left` / `_right` | 双臂正装平移映射（出厂＝单臂正装基准；右臂**尚未上真机标定**） |
| `mapping.rotation_signs` | 逐轴旋转符号（俯仰 Y 与摆腕 Z 翻转，滚转 X 保持） |
| `mapping.base_link_rotation_signs` | `control_frame=base_link` 专用的那组符号（**换坐标系必须换符号**） |
| `tuning.smooth_alpha` | 增量 EMA 平滑的新样本权重，取值 (0,1]（1.0=不平滑，出厂 0.5） |
| `sim.max_step_rad` | 每帧最大关节步长（rad，出厂 0.2 = metavr 的值，防 IK 跳变；与速度无关。`--ik pybullet` 与 `--ik pin` 共用） |
| `exact_ik.max_joint_vel` | **只管 `--ik exact`** 的关节速度上限（rad/s，出厂 10.0；调到 99.0 ≈ 关掉这一界）。metavr 无此量 ⇒ 不放在全局 `sim:` 段 |
| `pin_ik.velocity_limit` | `--ik pin` 自己的速度上限（rad/s，出厂 1.0 = URDF 值，= metavr 的 `VelocityLimit(model)`） |
| `sim.q_home` / `sim.single_q_home` | 双臂侧装 / 单臂正装 的初始构型（双臂正装复用后者） |
| `sim.mirror_signs` | 双臂右臂 home = 左臂 × 镜像符号 |
| `sim.ik_pos_tol` / `sim.ik_rot_tol` | IK 收敛判定残差（m / rad） |
| `tuning.deadband.*` | 静止死区的四个原始阈值（乘 `tuning.deadband_scale`） |
| `vr.source` | VR 数据源：`zenoh`（出厂，订阅中间层帧流）/ `ws`（自建 WS 监听，头显直连） |
| `vr.arm_id` | zenoh 话题里的 arm 段（`litearm/v4/{arm_id}/vr_frame`，出厂 `armA`）。**与桥填得不一样不会报错，只是收不到帧** |
| `vr.zenoh_mode` / `vr.zenoh_connect` / `vr.zenoh_listen` | zenoh 会话：模式（出厂 `peer`）/ 主动连的端点（出厂 = 同机桥的 `tcp/127.0.0.1:7447`）/ 监听端点（出厂空 —— 同机的桥已占 7447） |

命令行只用来**临时**覆盖；可用的开关见 `./launch/run_teleop_webxr.py --help`。
布尔项都支持 `--x / --no-x`，所以配置文件里写 `true` 也能从命令行关掉。

## 修改仿真模型

模型 URDF 位于 `models/litearm_description/urdf/litearm_description.urdf`（单臂，
上游 [litearm-description](https://github.com/nexform-tech/litearm-description) 的 SE 包）
与 `models/dual_litearm_column_description/urdf/`（双臂，xacro 源 + 生成 .urdf）。
pybullet 直接 `loadURDF`（`teleop_sim.py` 内 `_load_urdf_with_mesh` 做 package:// 路径替换），
无编译步骤——改 URDF 后重启服务端即生效。

单臂有两个 URDF，`litearm_side.urdf` 是 `litearm_description.urdf` 的派生物：正文逐字节
相同，只在 `<robot>` 之后插入 `world` link 与 `mount` 固定关节（`rpy="-1.5708 0 0"`）。
**改完正装 URDF 要同步重生成侧装版本**，否则两者会漂移。

双臂改限位/几何要改 `urdf/litearm_macro.xacro`（唯一真源），再用 xacro 重生成：

```bash
cd models/dual_litearm_column_description/urdf
xacro dual_litearm_column.xacro -o dual_litearm_column.urdf
```

**不要只改 `dual_litearm_column.urdf`**——它是生成物，下次重生成会把改动冲掉。
单双臂的关节限位必须取自同一张表：单臂取自上游 SE URDF，双臂取自 `litearm_macro.xacro`，
改一侧就要改另一侧。

## 数据源方案（中间层 zenoh 出厂；WebSocket 为回退，无需 APK/Unity/侧载）

**出厂**：头显端为外部 Quest 客户端（ws 明文连桥的 `:8765`），数据链：
手柄 JSON → ws(8765) → 桥 → zenoh `litearm/v4/{arm}/vr_frame` →
`src/litearm_teleop/vr_source.py`（订阅 + 队列）→ `scripts/ws_echo_server.py`
（解析 + 遥操语义）→ `teleop_sim.py`（同进程 import 直接驱动）+ 可选 ROS2
`/joint_states` 发布（RViz2 显示）。

**回退 `--source ws`**：本仓自己监听 8765，头显直连，不经过桥、不需要 zenoh；
两条路之后完全同构（同一个 `TeleopBridge.handle_frame`）。

```bash
./launch/run_teleop_webxr.py              # 一键：robot_state_publisher + rviz2 + 遥操服务（zenoh 订阅）
./launch/run_teleop_webxr.py --source ws  # 头显直连本机（8765）
```

- WebXR 坐标系为右手系；首次联调若方向不对，调 `teleop_sim.py` 的
  `SIDE_CONFIG`（左右臂各自独立的轴映射常量）
- 帧流不再经过「本仓自建 WS」这一段，但**帧格式、遥操语义、IK 全都没变**
- 加新数据源时的位置：`src/litearm_teleop/vr_source.py`（会话/话题/队列纪律）+ 服务端
  `_serve_*` 各一个分支；两个数据源必须共用同一个 `_handle_raw`
- 话题拼法必须与桥的 `protocol.frame_topic` 一致（`tests/test_vr_source.py` 钉住了
  `litearm/v4/armA/vr_frame` 这个字面量）

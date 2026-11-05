# 系统架构

## 总体数据流（中间层 zenoh 手柄方案，出厂）

```
Quest 3 客户端（手柄位姿，~72Hz）
   │ 手柄 JSON：leftController / rightController
   │ 每帧 { position{x,y,z}, quaternion{x,y,z,w}, rotation(欧拉), gripActive, trigger, tracked }
   ▼ WebSocket (ws, 8765)
litearm-vr-bridge（同机；收帧、状态汇总、掉线兜底、进程启停）
   │ **原样透传**每帧（不合并/不节流/不重排），zenoh pub
   ▼ zenoh 话题 litearm/v4/{arm_id}/vr_frame（arm_id 默认 armA）
遥操服务 scripts/ws_echo_server.py（单进程）
   │ 订阅上述话题（src/litearm_teleop/vr_source.py）→ 解析 → 遥操语义
   │ （gripActive 边沿=锚定/暂停/轴映射/平滑）
   │ 直接驱动 src/litearm_teleop/teleop_sim.py（同进程 import，无中间层）
   ▼
pybullet calculateInverseKinematics → 纯位置控制直驱关节 → pybullet GUI 窗口
```

- 本仓**只订阅、不上报**：话题、arm 段与会话参数都可配（`config/teleop.yaml` 的 `vr:` 段）。
  ⚠ **不做解析以外的加工** —— 桥那边定死了「只透传」，本仓再合并/节流就等于偷偷改协议。
- **`--source ws`（回退）**：本仓自己监听 8765，Quest 客户端直连（下面「WS 回退」一节）。
  两条路收到的都是**同一种帧**，之后完全同构。

## 为什么这个数据源是「中间层」而不是本仓自建 WS

桥（`litearm-vr-bridge`）要集中做三件本仓做不了的事：状态汇总（头显/真机/进程合成一条
状态流给上位机）、掉线兜底（>500ms 无帧就地持位）、进程启停。这些都要求**收帧位置唯一**，
所以头显只连桥，本仓从 zenoh 拿帧。桥的完整设计见其 `docs/DESIGN.md`。

## zenoh 会话怎么连（`vr_source.ZenohFrameSource`）

| 配置 | 出厂 | 为什么 |
|---|---|---|
| `vr.source` | `zenoh` | 出厂走中间层；`ws` 是本仓自建监听（无桥时用） |
| `vr.arm_id` | `armA` | 话题里的 arm 段，必须与桥的 `ZENOH_ARM_ID`、`litearm-server --arm-id` 一致。**不一致不报错，只是永远收不到帧** |
| `vr.zenoh_mode` | `peer` | 可连也可被连。**为什么不是 client**：client 要求开会话时就已连上端点，桥没起会直接 `ZError` 抛错退出 —— 而「先起 teleop、后起桥」是正常顺序。peer 连不上只是后台重试（实测桥后起约 **1s** 内接上） |
| `vr.zenoh_connect` | `tcp/127.0.0.1:7447` | 桥的监听端点（桥在别的机器就写那台 IP）。逗号分隔可给多个。**配了它就不做组播/流言发现**（只跟写明的端点说话） |
| `vr.zenoh_listen` | 空 | 默认不监听：桥已经 listen 在 7447，同机再听会撞端口。要给别的机器留入口才配 |

话题与会话这套与桥 `docs/DEPLOY.md` 里「消费者」那条配方同源（只是模式从 client 换成 peer，
理由见上表）。

**回调线程与队列。** zenoh 的订阅回调跑在它自己的线程上，而 pybullet 必须在主线程调，
所以回调只做「解码 + 入队」（`queue.Queue`，深度 64 同桥），服务端用一个协程按自己的
节奏把帧取出来喂 `TeleopBridge`。队列满则丢**当前**帧并计数（顺序比新鲜度重要），
空载荷/非 UTF-8 样本同样计数 —— 与桥的 `TeleopPublisher` 同一纪律：宁可顶回来让我们
看见，也不静默丢。

**取帧用带超时的轮询**（`recv(timeout=0.5)` 走 `asyncio.to_thread`）：被 Ctrl-C 打断时
后台线程最多再等一个超时就结束，不会卡住 asyncio 关闭时的线程 join。

**Ctrl-C 要自己抢回来。** zenoh 的 Rust 运行时会装自己的 SIGINT 处理（做优雅退出），
装完就把信号吞了 —— KeyboardInterrupt 再也到不了 Python，前台按 Ctrl-C 毫无反应
（只能 SIGTERM / Ctrl-\\）。所以 `start()` 在 `zenoh.open()` **之后**又装了一次 Python
的处理器（它直接调 sigaction、不走 Python 的 signal 模块，之后再装才盖得住）。
回归用例：`tests/test_vr_source.py` 的信号用例（真发 SIGINT，验子进程能不能被打断）。

## WS 回退（`--source ws`）

```
Quest 3 客户端 ──ws 8765──▶ scripts/ws_echo_server.py（同进程驱动 teleop_sim）
```

- 没有桥、或本机没有 zenoh，也要能调遥操：这条路与改造前逐字相同，只是变成了**可选**。
- 数据源不同、之后完全同构：同一个 `TeleopBridge.handle_frame`。
- 启动：`./launch/run_teleop_webxr.py --source ws`（或直接
  `python scripts/ws_echo_server.py --source ws --host 0.0.0.0 --port 8765`）。
- 只有这条路占 8765，所以启动脚本**只在 `--source ws` 时**查端口占用。

## 配置层（`config/teleop.yaml`）

```
config/teleop.yaml ──► launch/run_teleop_webxr.py ──┬─► 自己用（host/port/单双臂/显示后端/数据源）
   （全部可选项）        优先级: 命令行>配置>常量     └─► --config 转发 ──► scripts/ws_echo_server.py
                                                              │ 读同一份文件取 vr/sim/mapping/pin_ik/
                                                              │ exact_ik/controller/夹爪电机参数
                                                              ▼
                                                    teleop_sim.Tuning / teleop_sim.SimConfig
```

- 读取器 `src/litearm_teleop/config.py` **只用标准库**（自带 YAML 子集解析器）。这不是
  洁癖：启动脚本要在 metavr-arm 环境建好之前、用系统 python3 跑，那时既没有 pybullet
  也没有第三方 YAML 库。
- 两个入口都读**同一个文件**（启动脚本把 `--config` 原样转发给服务端），所以不会出现
  「启动脚本按 A 配置、服务端按 B 配置」。
- 分两层落到仿真：`Tuning` 管**遥操输入端**（死区、平滑、坐标系、滞回、精确 IK 容差），
  `SimConfig` 管**仿真与运动学**（频率、限幅、home、轴映射、收敛容差、两个 IK 后端的
  求解器参数）。两者的默认值都还是模块常量，逐位不变 —— 配置层是加法，不是行为变更。
- 灵巧手/夹爪的**开关**由启动脚本说了算并显式转发 `--hand/--no-hand`：准备 CAN 接口与
  PYTHONPATH 是它的活，服务端不该自己去读配置里那个 `true`。

## 手柄数据格式

**两个数据源收到的是同一种帧**（桥就是原样透传头显发来的那条 JSON），每帧一条 JSON 文本：

```json
{
  "leftController": {
    "position": {"x": .., "y": .., "z": ..},
    "quaternion": {"x": .., "y": .., "z": .., "w": ..},
    "rotation": {"x": .., "y": .., "z": ..},
    "gripActive": false, "trigger": 0.0, "tracked": true
  },
  "rightController": { "..." },
  "timestamp": 1787641468151
}
```

- 遥操只取 `rightController`（右手手柄）；`tracked=false` 时跳过该帧
- `gripActive` 边沿驱动锚定/暂停；`trigger` 为扳机（当前不驱动夹爪）
- `rotation`（欧拉角，度）当前未使用——遥操只用 `position` + `quaternion`

## 频率

| 环节 | 频率 |
|---|---|
| 手柄位姿推流 | ~72 Hz（客户端） |
| 桥收帧 → zenoh → 本仓（或 `--source ws` 直收） | 客户端驱动，逐帧处理（无独立主循环限频） |
| 动作平滑 | 增量一阶低通（EMA，α=`tuning.smooth_alpha`，出厂 0.5 ⇒ 群延迟 ≈1 帧） |
| zenoh 帧队列 | 深度 64（`vr_source.DEFAULT_QUEUE_SIZE`，90Hz 下 ≈0.7s 缓冲）；满了丢当前帧并计数 |

## 遥操语义（teleop_sim.py）

- **gripActive 上升沿**：启动——记录当前手柄位姿为 delta 参考（不产生本帧增量）
- **gripActive 下降沿**：暂停——保持现位姿；暂停帧喂零增量（恢复遥操时从平滑小增量起步）
- **扳机**：数据链路保留，夹爪未建模（环境忽略）
- **速度式 delta**：取手柄相邻两帧位姿差 `prev_ctrl_H⁻¹ @ cur_ctrl_H`，旋转/平移分别
  经轴映射（`axis_remap`/`translation_remap`）映射成本帧末端增量；手柄停 → delta≈0 → 机器人停
- **平滑**：增量一阶低通（EMA，α 见 `config/teleop.yaml` 的 `tuning.smooth_alpha`）。
  曾用箱式 10 帧移动平均（群延迟 ≈4.5 帧，是「跟手不够」的主因），已全换成 EMA
- **IK**：`calculateInverseKinematics` 每帧以当前关节角为种子（restPoses + jointDamping
  拉 null space 保持 7-DOF 冗余解连续），FK 残差判定收敛（3mm / 0.05rad）
- **驱动**：纯位置控制——`q = q_cur + clip(q_tgt − q_cur, ±sim.max_step_rad)`（出厂 0.2 rad/帧），
  无重力无动力学，pybullet 仅作运动学显示；IK 失败时用最优解（best-effort），
  近奇异时终端限频打印预警（sigma_min = 6×7 世界系雅可比最小奇异值）

## 双臂侧装（dual）

- **模型加载**：直接加载**单个双臂+立柱 URDF**（`dual_litearm_column.urdf`，含立柱
  `column_base_link`），一个 robot_id。`package://` mesh 路径运行时替换为本地绝对路径。
- **关节索引**：双臂 URDF 有 18 个关节（14 可动 + 4 fixed）。pybullet 的
  `calculateInverseKinematics` 只对**可动关节**返回解（跳过 fixed），输入/输出都是
  14 维：`left_Joint1..7` → 0..6，`right_Joint1..7` → 7..13（见 `Arm._ik_index`）。
- **单臂 IK**：对某臂做 IK 时，构造 14 维参数——本臂关节用真实限位+当前 q 作种子，
  **另一臂关节用 `lo=up=当前值` 锁死**（`jointRanges=0`），使求解器只在本臂关节上收敛。
  否则 restPoses 只作用到前 7 关节（左臂），右臂 IK 无法收敛。
- **挂载位姿**：左臂 `pos=(0,+0.055,0)` rpy=(−90°,0,0)（单臂侧装姿态，base 绕 X 轴 −90°）；
  右臂 `pos=(0,-0.055,0)` rpy=(−90°,0,180°)（= 左臂绕立柱竖轴 Z 转 180°，背靠背镜像）。
  home 构型相应把 J1 偏 ∓90°（左 −π/2、右 +π/2），使两臂末端初始世界位姿与改前一致。
- **对称协作**：左右臂轴映射符号相反（见 `SIDE_CONFIG`）——同一手柄动作在两臂产生
  镜像末端运动（双手同时前推 → 两臂同时向外）

## 双臂正装（upright）

`--dual --upright`：两只**独立竖直臂、无立柱**（不是柱子上两臂侧伸）。

- **模型**：`models/dual_litearm_column_description/urdf/dual_litearm_upright.urdf`（xacro 源
  `dual_litearm_upright.xacro`）。与侧装**同包**（复用同一 `litearm_macro.xacro` 与
  `meshes/`），root 是一个无 visual 的轻量共同 link `world`，两臂以 `rpy=0 0 0`
  **正装**挂在其下、沿 ±y 分开 0.25m。
- **镜像**：右臂 mount `rpy="0 0 3.1416"`（绕竖直轴转 180°，与侧装的 Rz(180) 同源），
  home = `SINGLE_Q_HOME * MIRROR_SIGNS`，与左臂构成精确镜像。
- **home/映射**：复用**单臂正装基准**——两臂均用 `CTRL_AXIS_REMAP` +
  `CTRL_TRANSLATION_REMAP` + `ROTATION_AXIS_SIGNS`（见 `UPRIGHT_CONFIG`），
  不复用 `SIDE_CONFIG`（那是水平侧装基准，末端朝向差 90°）。
  ⚠ 右臂镜像映射尚未上真机标定；若手感不对，照 `SIDE_CONFIG["right"]` 的方式补符号。
- **代码路径**：与 `dual` 完全同构（单 body、14 关节、`left_`/`right_` 前缀、
  `ik_index` 0..6 / 7..13）——`LiteArmTeleopSim.__init__` 里按 `upright` 只换
  URDF 路径、映射配置与 home 三项。

## 坐标系与映射（关键！）

- **WebXR 空间**：右手系，X 右 Y 上 Z 后（-Z 前）
- **机械臂基座系**：右手系，Z 上；初始位姿下臂在 xz 平面内，末端 Z 轴≈-x
- **VR↔基座对应**（实测调定，模块常数见 `teleop_sim.py` 的 `SIDE_CONFIG`）：
  - `axis_remap`：旋转映射（算子约定 pinv(H_R)·R·H_R）
  - `translation_remap`：平移映射（实测反解定标：手柄右移→画面右 +x、
    前推→画面深处 +y、上移→上 +z）
  - `rotation_signs`：逐轴旋转符号（rel_rotvec 分量 0/1/2 = 末端局部 X/Y/Z）：
    俯仰(Y)与摆腕(Z)翻转，滚转(X)保持（手柄俯仰经轴映射后落在末端 Y 分量）
  - **左右差异**：右臂 `translation_remap` 的「左右」分量与 `rotation_signs` 的
    「俯仰/摆腕」分量符号相反（对称协作）；联调若方向不对，只调 `SIDE_CONFIG`。

## pybullet 实现要点

- **模型**：`models/litearm_description/`（单臂正装 / 单臂侧装）与
  `models/dual_litearm_column_description/`（双臂侧装立柱 / 双臂正装无立柱）
  URDF 直接 `loadURDF(useFixedBase=True)`，7 关节限位取自 URDF（几何唯一真源）；
  mesh 分两套——`meshes/`（高精度，RViz 用）、`meshes_lowpoly/`（降采样，pybullet 渲染用）
- **FK 帧陷阱**：`getLinkState` 的 `[0][1]` 是质心帧（COM），`[4][5]` 才是 URDF link
  原点帧（与旧 pylitearm 的 ee_frame_link 帧同构，差 5cm+ 不可混用）
- **IK**：DLS 求解器，`lowerLimits/upperLimits` 钳制、`restPoses=当前 q` +
  `jointDamping=0.01` 拉 null space；home 处 sigma_min=0.142 与旧 pinocchio 口径一致
- **显示**：RViz2（RobotModel + TF，`/joint_states`）；无 ROS2 时降级 pybullet DIRECT（无窗口）

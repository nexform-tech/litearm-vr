## 固定锚点遥操与完整目标追踪

- 统一三个IK后端的固定握持映射；位置/姿态滤波与空间滞回不再丢弃慢速累计动作。
- 独立控制周期保留目标，统一关节速度/加速度约束；不可达、失效与松开结束追赶。
- 真机增加新鲜实测状态与start握手；首次握持和双击不再自动回home。
- 最新位姿队列保留状态边沿，溢出/发布失败发送保持屏障。
- 迁移与新配置见 [anchored-tracking.md](docs/anchored-tracking.md)。

# Changelog

## [Unreleased]

### VR 数据源改走中间层 zenoh（`--source zenoh`，出厂）+ 保留 WS 回退
- **新增 `src/litearm_teleop/vr_source.py`**：订阅桥（`litearm-vr-bridge`）转发的话题
  `litearm/v4/{arm_id}/vr_frame`（arm 段默认 `armA`，与其 `protocol.frame_topic` 同拼法）。
  桥的纪律是「原样透传」，所以本层只做「收下 payload」，解析与遥操语义都在上层。
  - **会话默认值 = 桥 `docs/DEPLOY.md` 里「消费者」那条配方的位置，但模式取 `peer`
    而不是 `client`**：`peer` + `connect=tcp/127.0.0.1:7447` + 不做组播/流言发现。
    `client` 要求**开会话时**就连上端点，桥没起会直接 `ZError: Unable to connect to
    any of [...]` 崩在启动阶段，而「先起 teleop、后起桥」是正常顺序；`peer` 连不上只是
    后台重试，桥上来约 **1s** 内接上（实测）。配了 `zenoh_connect` 就关掉组播/流言发现
    （只跟写明的端点说话）。**也不 listen** —— 桥自己 listen 在 7447，同机再听同一个
    端口会撞（桥的 `deploy/zenoh-router.service` 注释写明过）。这些全部可配。
  - **Ctrl-C 要从 zenoh 手里抢回来**：zenoh 的 Rust 运行时会装自己的 SIGINT 处理，
    装完把信号吞了 —— Ctrl-C 不再变成 KeyboardInterrupt，`make teleop` 前台按 Ctrl-C
    停不下来（只能 SIGTERM / Ctrl-\），操作员会以为服务卡死。`start()` 在
    `zenoh.open()` **之后**再装一次 Python 的处理器盖回来（实测有效，并有回归用例）。
  - **回调不做事、队列才做事**：zenoh 回调跑在它自己的线程上，而 pybullet 必须在主线程调，
    所以回调只做「解码 + 入队」（`queue.Queue`，深度 64 同桥）；服务端用协程按自己的节奏
    取帧喂 `TeleopBridge`。队列满丢**当前**帧并计数，空载荷/非 UTF-8 样本同样计数 ——
    与桥的 `TeleopPublisher` 同一纪律（宁可顶回来让我们看见，也不静默丢）。
  - 取帧走 `recv(timeout=0.5)` + `asyncio.to_thread` 轮询：Ctrl-C 时后台线程最多再等一个
    超时就结束，不会卡住 asyncio 关闭时的线程 join（实测退出 <1 个超时）。
  - `zenoh` 是**延迟 import**：`--source ws` 不需要装 `eclipse-zenoh`，
    `launch/run_teleop_webxr.py`（只用标准库）也 import 本模块取默认值。
- **`scripts/ws_echo_server.py` 支持 `--source zenoh|ws`（出厂 zenoh）**，两个数据源共用
  同一个 `_handle_raw`（首帧诊断、JSON 解析、`TeleopBridge.handle_frame` 一字不改）。
  `--source ws` 就是改造前那条路（本仓自己监听 `--host/--port`，头显直连），
  没有桥/没有 zenoh 的机器仍可调遥操。
- **`config/teleop.yaml` 新增 `vr:` 段**：`source` / `arm_id` / `zenoh_mode` /
  `zenoh_connect` / `zenoh_listen`；启动脚本也只**在 `--source ws` 时**查 8765 端口占用
  （zenoh 模式根本不占它），并打印实际用的数据源、话题与会话参数。
- **依赖**：`environments/metavr-arm-requirements.txt` 增 `eclipse-zenoh>=1.10`
  （与桥/zenoh-router 的 1.10.x 对齐），`setup.sh` 的验证步骤同步加一条 import 检查。
- **测试**：新增 `tests/test_vr_source.py`（话题字面量 / 端点四种写法 / 队列与丢帧纪律 /
  **按出厂顺序的真链路**：teleop 先起、桥后起仍能收到帧 / **真发 SIGINT** 验 Ctrl-C 没被
  zenoh 吞掉；没装 `eclipse-zenoh` 就跳过真链路并打印原因）+
  `make test-vr`（已并入 `make test`）；`tests/test_config.py` 的 `[10]` 增加数据源转发
  断言与一个 `--source ws` 用例，新增 `[12]` 守着 `vr.*` 的出厂值 == 代码常量。
- 端到端验证（出厂配置）：teleop 先起（此时无人监听 7447，不崩）→ 假桥后起 listen 7447
  并发 20 帧 → 收 15（前 ~1s 是连接重试窗口）、丢 0，首帧诊断打印；Ctrl-C 干净退出并打
  `[zenoh] 收帧 N，丢帧 0`。

### `--ik pin` 对齐 metavr 的 `PinkIk` 策略 + 默认后端改回 `pybullet`
- **求解结构换成 metavr 的「逐拍积分到收敛」**（`pin_ik.PinIKSolver.solve`）：从种子
  出发最多 `max_iters` 拍、每拍固定 `dt`(=`PINK_DT`)，任务误差范数低于 `err_tol` 即停。
  此前是「每帧只解一步速度、按本帧 dt 积分」。**`sim.freq` 与掉帧都不再影响它能走多远**。
- **参数全部取 metavr 的数值**：`lm_damping` `1.0`→`1e-3`、`posture_cost` `1e-2`→`1e-3`、
  `ManipulabilityTask` 由「默认可选」改成**常开**（cost `1e-2`、`rate 0.1`、`WORLD`）、
  `VelocityLimit` 由显式 `2.0` 改成 URDF 值（`1.0`，等价 metavr 的 `VelocityLimit(model)`）。
  这些数字的单一真源是 `pin_ik.py` 的 `PINK_*` 常量，`SimConfig` 的工厂默认直接引用它，
  配置文件的 `pin_ik.*` 键与之一一对应（有测试逐位比对）。
- **⚠ 唯一没照抄的是 QP 后端**：metavr 写 `daqp`，但本环境（metavr-arm）qpsolvers 只装了
  `quadprog`（`available_solvers == ['quadprog']`，传 `daqp` 直接 `SolverNotFound`）⇒
  出厂保持 `quadprog`。装了 daqp 再把 `pin_ik.solver` 改回去即可。
- **保留双臂**：metavr 的 pink 只支持单臂（`prefix=="" and all_rev_ids is None`），本仓
  仍是「14 维模型 + 被动关节速度每拍置零」的锁死语义。这是与 metavr 的**唯一**结构性差异。
- **外层步长钳制加回到 pin**：metavr 对 pink 与 pybullet 一视同仁（其 `_step_teleop` 只有
  一处 `clip(q_tgt - q_cur, ±MAX_STEP_RAD)`，在 IK 分派之后）。既然求解是「本帧内积分到
  收敛」，一次大目标（快挥/掉帧）可能让关节一口气走很远，这层就是挡那个的兜底。
- **默认 IK 后端 `pin` → `pybullet`**（代码默认 + 配置文件 `tuning.ik`），与 metavr 的
  `IK_BACKEND = "pybullet"` 一致。
  ⚠ **代价要清楚**：出厂后端因此**没有静止死区**（metavr 无此机制、增量原样进 IK），
  手柄静置时追踪噪声会被放行 —— 这正是当初加死区要挡的东西。需要噪声抑制就把
  `tuning.ik` 换成 `pin` 或 `exact`（死区是这两个后端的性质）。
- **`max_joint_vel` 从全局 `sim:` 段移到 `exact_ik:` 段**：metavr 没有这个量，而它只被
  `--ik exact` 的速度盒用 ⇒ 不该占全局位置。`sim.max_step_rad` 留在全局（metavr 也有，
  且两个后端共用），注释同步写明。
- **`exact_ik.max_joint_vel` 出厂 `99.0` → `10.0`**（`MAX_JOINT_VEL` 常量同步）：
  10.0 在 60Hz 下是 0.167 rad/帧，是一个**真会起作用**的界（手速快时 exact 因此就地
  持位，预警句会写明「起作用的是关节速度上限」），而不是一个形同虚设的数。
  想**关掉**这一界再调回 `99.0`（≈1.65 rad/帧，已远超单帧行程），注释里两条都写了。
- **测试**：`[5]`（静止死区）与 `[6]`（TCP 换算）改为**显式指定后端** —— 它们测的是
  死区/TCP 换算本身，而新出厂后端 `pybullet` 按 metavr 语义没有死区、且 best-effort IK
  解不出纯旋转（对照组法兰会飘 74mm）。不指定后端就会测成「出厂后端有没有死区」。
- 实测改善（冒烟）：纯旋转法兰位移 `36.8 → 40.6 mm`（解析解 40.7）、TCP 漂移
  `3.9 → 0.4 mm`、pin 增益 `0.9967 → 0.9998`、`base_link` 世界轴夹角 `8.5° → 0.8°`。

### 增量平滑全换成 `ema_smooth`：箱式移动平均彻底删除
- **三个 IK 后端共用同一个 EMA 平滑**。此前 pin / exact 用箱式 10 帧移动平均、
  pybullet 用 EMA（上面那条回退带来的分裂）。但平滑作用在后端分派**之前**的共享
  前端里，而「跟手」是**手柄→臂这条链**的体感，不该随后端变 —— 于是统一到 EMA。
- **删掉的东西**：`moving_average()`、`Arm._ma_queue`、`MOVING_AVERAGE_LIMIT`、
  `Tuning.moving_average_limit`、`--moving-average`、配置键 `tuning.moving_average`。
  残留一个平滑机制比留着两套更难维护，所以是删不是并存。
- **新增 `Tuning.smooth_alpha` / 配置键 `tuning.smooth_alpha` / CLI `--smooth-alpha`**，
  取值 **(0, 1]**，出厂 `0.5`（`SMOOTH_ALPHA` 常量）。
  ⚠ 上界 1.0 是「不平滑」，下界是**开区间**：α=0 会让状态永不更新、增量恒为 0，
  表现为「臂完全不动」而不是「很稳」，所以在 `Tuning` 里就拦住。
- **删掉环境变量 `LITEARM_SMOOTH_ALPHA`**：它是在没有配置层时的临时旋钮，而配置
  文件才是本项目的单一真源（同一个量两处可调，迟早对不上）。
- **暂停帧同样走 EMA 喂零**（不再分后端）。恢复遥操时滤波状态连续，不会把暂停期间
  积压的增量一次性放出来。
- 实测：冒烟 `[3]` 手柄 +x 5cm 的末端位移 47.9 → **48.8 mm**（少一个 EMA 的群延迟，
  更贴近满 5cm）。

### `--ik pybullet` 回退到 metavr：前端与结果处置都按后端分开
起因：以 metavr 为基准核对 pybullet 流程，发现 teleop 在其上叠了 6 处改动。
本次把 pybullet 拉回 metavr 行为，**只保留末端偏移量（TCP）**一项。

- **必须「拉开 if 分支」而不是改共享路径**：死区 / 平移映射跑在后端分派
  **之前**的共享前端里（`_step_teleop`），直接改会连带改掉 pin / exact 的手感。
  故新增 `Arm._legacy_pybullet = (ik_backend == "pybullet")`，在同一函数内分流。
- **平移映射回退**（唯一一处运动学语义差异）：pybullet 分支恢复 metavr 的
  `rel_t = T⁻¹·R_prevᵀ·Δp` —— 先把手柄世界位移转进**手柄自身系**再重映射，
  控制轴随手柄朝向转；其余后端仍是 `T⁻¹·Δp`（世界方向跟随）。
  手柄直立时两者相等，故既有平移标定不变。
- **增量平滑回退**：pybullet 改用 metavr 的 `ema_smooth(α=0.5)`，不再用箱式移动平均
  10 帧（metavr 实测：箱式群延迟 ≈4.5 帧是「跟手不够」的主因）。暂停时 `feed_pose`
  喂零保持滤波状态连续。**后续已把这条统一到三个后端**，见上面那条 —— 平滑不再是
  「按后端分开」的那几项之一（`_legacy_pybullet` 仍管平移映射与静止死区）。
- **静止死区对 pybullet 关闭**：metavr 无此机制，增量原样进 IK。
- **结果处置回退三处**（`_step_pybullet`）：
  ① IK 未收敛改回 **best-effort**（只告警，限幅后照常下发，不再就地持位）；
  ② **删掉关节目标低通**（`_q_lp` 状态与 `q_lowpass_tau` 的使用）；
  ③ 限幅**只剩步长这一层**（`dq_max = max_step_rad`），去掉 `max_joint_vel·dt`。
- **`MAX_STEP_RAD` `0.08` → `0.20`**：与 metavr 一致。metavr 的实测依据是
  速度式 delta 映射下被削掉的位移**不会补回**，0.08 在 5cm 快挥时只到位 40~86%。
  该常量只被 pybullet 用（pin 的上限在 pink `VelocityLimit`；exact 由速度盒界定）。
  本次同时修掉了「config `max_step_rad` 已是 0.2 而代码常量还是 0.08」的漂移。
- **`q_lowpass_tau` 停用但保留**（常量 + config 键 + 注释），便于回退。
- **冒烟测试新增 `[9]`**，三条判据各配判别性对照，防止共享前端被「顺手统一」：
  (a) 平移映射（pybullet `[0.015,0,-0.026]` vs exact `[0.03,0,0]`）；
  (b) 静止 30 帧 `zeroed`（pybullet `0/30` vs exact `30/30`）；
  (c) `max_joint_vel=1.0` 下 `dq_max == MAX_STEP_RAD`（0.20，不被速度层压低）。
- `--control-frame base_link` 对 pybullet 无效（metavr 无此开关）；同时给出会
  打印一条警告，不再静默忽略。

### 灵巧手实现移出：本地 CAN → 远端 `hand_server`（归 follow 仓）
- **删 `scripts/hand_control.py`**：L6 灵巧手驱动的实现挪到
  `litearm-vr-follow/hand_server.py`（与夹爪 `gripper_server`、臂 `arm_follow_server`
  同类，都属硬件层）。本仓不再含手部驱动，只留客户端。
- **`ws_echo_server.py`**：`--hand` 只走远端 TCP（`HandRemoteClient`，命令带 `side`
  区分左右手），去掉本地 CAN 分支与 `--hand-can`；`--hand` 必须配
  `--hand-remote HOST:PORT`（否则报错退出）。
- **`run_teleop_webxr.py`**：同步去掉 `--hand-can` / `--linkerbot-sdk` 与灵巧手的
  SDK 探测、`ensure_can_up`、`prepend_pythonpath`（本机不再碰灵巧手 CAN，也不需 SDK）；
  随之删除此后无引用的 `prepend_pythonpath` / `interface_exists` / `interface_is_up` /
  `ensure_can_up` 四个辅助函数。
- **README**：数据流/参数表/目录树/依赖表改成本机无灵巧手 CAN/SDK、经 `hand_server` TCP 桥接。

### 夹爪实现移出：本地 CAN → 远端 `gripper_server`（归 follow 仓）
- **删 `scripts/gripper_control.py`**：夹爪 DM 电机驱动的实现挪到
  `litearm-vr-follow/gripper_server.py`（与臂的 `arm_follow_server` 同类，都属硬件层）。
  本仓不再含电机驱动，只留客户端。
- **`ws_echo_server.py`**：`--gripper` 只走远端 TCP（`GripperRemoteClient`），
  去掉本地 CAN 分支与 `--gripper-can` / `--gripper-can-id`；`--gripper` 必须配
  `--gripper-remote HOST:PORT`（否则报错退出）。
- **`run_teleop_webxr.py`**：同步去掉 `--gripper-can` / `--gripper-can-id` / `--pyarmdyn`
  与夹爪的 `ensure_can_up`（本机不再碰夹爪 CAN）。
- **README**：数据流/参数表/目录树/依赖表改成本机无夹爪 CAN、经 `gripper_server` TCP 桥接。

### 新增双臂正装（两只独立竖直臂、无立柱）
- **新 URDF** `models/dual_litearm_column_description/urdf/dual_litearm_upright.xacro`
  （+ 生成物 `.urdf`）：与侧装**同包**（复用同一 `litearm_macro.xacro` 与 `meshes/`），
  root 是一个无 visual 的轻量共同 link `world`，两臂以 `rpy="0 0 0"` 正装挂在
  其下、沿 ±y 分开 0.25m；右臂 mount `rpy="0 0 3.1416"`（绕竖直轴转 180°）与左臂
  构成精确镜像。
- **`teleop_sim.py`**：新增 `UPRIGHT_URDF_PATH`、`UPRIGHT_CONFIG` 与
  `LiteArmTeleopSim(..., upright=False)`。dual 分支按 `upright` 只换 URDF 路径、
  映射配置、home 三项——`home_left=SINGLE_Q_HOME`（右臂 `*MIRROR_SIGNS`），
  映射复用单臂正装基准（`CTRL_AXIS_REMAP`+`CTRL_TRANSLATION_REMAP`+
  `ROTATION_AXIS_SIGNS`）。两臂关节名/前缀/`ik_index`/pin frame 与侧装完全一致。
  ⚠ 右臂镜像映射尚未上真机标定。
- **入口**：`launch/run_teleop_webxr.py` 与 `scripts/ws_echo_server.py` 加 `--upright`
  （与 `--dual` 同用选正装）；就绪打印/模式提示新增「双臂正装」一档。
- 文档（README / architecture / development / 本包 README）补正装一档。
### 配置文件：`config/teleop.yaml` 收全所有可选项（含 teleop_sim 那一层）
- **新增 `config/teleop.yaml`**：频率/限幅/低通、home 构型与镜像符号、全部轴映射
  （单臂 + 双臂侧装 + 双臂正装三套 `axis_remap`/`translation_remap`/`rotation_signs`）、
  单双臂与侧装/正装的选择、IK 收敛容差、死区四个原始阈值与 `rest_confirm_frames`、
  两个 IK 后端的求解器参数（`pin_ik.*`、`exact_ik.rank_tolerance`）、近奇异措辞门槛，
  加上原有那些启动项（服务/显示/手感/真机/灵巧手与夹爪远端地址/手柄语义）
  —— 每项都写了单位与作用。
- **新增 `src/litearm_teleop/config.py`**：读取器，**只用标准库**（自带 YAML 子集解析器：
  映射、缩进嵌套、流式序列、`#` 注释、引号、null）。没引 pyyaml 是有意的——启动脚本
  要在 metavr-arm 建好之前用系统 python3 跑。
- **优先级：命令行 > 配置文件 > 代码里的出厂常量**。实现方式是「命令行没给过的项才从
  配置取」，所以「没给」与「显式给了出厂值」区分得开（见各处 argparse 的 `default`）。
  布尔项都支持 `--x / --no-x`，配置里写 `true` 也能从命令行关掉。
- **`teleop_sim.py` 新增 `SimConfig`**，并把 `Arm`/`LiteArmTeleopSim` 里原本直接读模块
  常量的地方（`FREQ`/`MAX_JOINT_VEL`/`MAX_STEP_RAD`/`Q_LOWPASS_TAU`/`IK_*_TOL`/
  `NEAR_SINGULAR_SIGMA`/`Q_HOME`/`SINGLE_Q_HOME`/`MIRROR_SIGNS`/`SIDE_CONFIG`/
  `UPRIGHT_CONFIG`）改读实例配置。**默认值仍是那些常量，逐位不变**——这是加法不是行为
  变更，`SimConfig()` 与 `Tuning()` 的出厂行为与改动前完全相同。`Tuning` 同时新增
  `rest_confirm_frames` 与四个死区阈值的逐个覆写（覆写值再乘 `deadband_scale`，所以
  scale=0 仍能一键关死区）。
- **双臂正装的映射单独成一字段**（`mapping.upright_*`）而不是直接复用单臂那份：
  它**尚未上真机标定**，手感不对时按臂补符号即可，不会连带改到单臂正装。
- **`--ik pin` 的求解器参数贯通**：`solver`/`position_cost`/`orientation_cost`/
  `lm_damping`/`posture_cost`/`velocity_limit` 从配置传进 `PinIKSolver`（此前只有
  `manipulability_cost` 被传入，其余吃的是类内默认值）。`exact_ik.describe_failure`
  的「接近奇异」门槛改为可传入，告警措辞可配。
- **服务端新增 `--config` 与一批可配项**：`ws_echo_server.py` 读同一份配置文件取
  仿真侧选项，并新增手柄语义四项（`trigger_threshold`/`debounce_s`/`min_close_s`/
  `double_click_s`）的入口；新增 `server.publish_ros`（等价原来的 `--no-ros`）。
  灵巧手/夹爪的开关与远端地址也进配置，且**开关与地址必须成对**——只开开关不给地址会
  静默什么都不做，操作者容易把「按了没反应」当成硬件问题，所以两边都在启动时就报错。
- **显式转发开关**：启动脚本一定传 `--hand/--no-hand`、`--gripper/--no-gripper`、
  `--real/--no-real`，服务端不自己去读配置里那个 `true`，避免「启动脚本判定为不启用、
  服务端却启用了」这种两套判断。
- **新增 `tests/test_config.py`（`make test-config`）**：逐位比对「配置文件里的值 == 代码
  常量」（配置与常量两份默认值最容易静默漂移）、解析器行为与错误、优先级、两张
  「选项 → 配置路径」表指向的键真实存在（路径写错只会让选项静默失效）、以及启动脚本拼出的
  argv 能被服务端解析器接住。
- **启动前核对 `litearm_teleop` 归属**（`run_teleop_webxr.py` 的 `check_package`）：
  import 一次并核对它是不是**本检出**这一份，不是就指名报错并给出重装命令。加这道检查
  是因为项目目录搬走之后 editable 安装里写死的旧绝对路径仍在（本机已撞过）：没装时
  服务端抛一片 `ModuleNotFoundError` Traceback，看着像服务端崩了；而装到了**别的检出**
  时更糟——editable 是 `MetaPathFinder`，优先级高于 `PYTHONPATH`/`sys.path`，
  `PYTHONPATH=src` 压不住它，于是会**一声不响地跑旧代码**。
- **文档**：`development.md` 增「配置文件」一节（含加新选项要动的四处）、`user-guide.md`
  增使用与排障条目、`architecture.md` 增配置层数据流。

### 目录重整：模型描述收进 `models/`、启停脚本收进 `launch/`、单臂换成上游 litearm-description
- **模型描述集中到 `models/`**：原根目录的 `litearm/` 与 `dual_litearm_column_description/`
  现为 `models/litearm_description/` 与 `models/dual_litearm_column_description/`。
  路径常量同步改在 `src/litearm_teleop/teleop_sim.py:41-55` 与
  `launch/run_teleop_webxr.py`（原 `scripts/run_teleop_webxr.py`）的 rviz 分支。
- **启停脚本移到 `launch/`**：`run_teleop_webxr.py` / `stop_teleop.sh` 从 `scripts/` 迁出，
  `PROJECT_DIR` 推导改为「上一级」；`scripts/` 只留 WS 服务、ROS2 发布与动作辅助脚本。
  `Makefile` 与全部文档的命令行同步更新。
- **单臂模型换上游**：`models/litearm_description/` 取自
  [litearm-description](https://github.com/nexform-tech/litearm-description) 的 `SE/litearm_description`
  （包名 `litearm` → `litearm_description`，URDF `litearm.urdf` → `litearm_description.urdf`，
  mesh URI 相应改名）。9 个 STL 与 `display.rviz` 与原文件**逐字节相同**，几何无变化。
- **关节限位统一到上游 SE 表**：上游与本地原限位表 7 个关节全部不同，本次取上游为准，
  并**同步改双臂** `models/dual_litearm_column_description/urdf/litearm_macro.xacro` 的七处
  `<limit>`，再 `xacro` 重生成 `dual_litearm_column.urdf` ⇒ 单双臂限位表一致。
  变化最大的是 J4 上限 `0.593412` → `0.035` rad（34° → 2°），单臂与双臂的可达工作空间都会收窄。
- **`litearm_side.urdf` 改为派生物**：上游没有侧装版本，由
  `litearm_description.urdf` 插入 `world` link + `mount` 固定关节生成（正文逐字节相同）。
  改正装 URDF 后必须同步重生成，规则写在 `docs/development.md`。

### 速度与步长限制：写进 README + `MAX_JOINT_VEL` 抬到 10.0
- **`MAX_JOINT_VEL`（`teleop_sim.py:69`）`2.0` → `10.0` rad/s**：仿真的关节速度上限不再
  贴着真机 `--max-speed 2.0`，`--ik exact` 因速度而就地持位的门槛相应放宽。
- **README 新增「速度与步长限制」一节**，写清两个常量的位置、作用与该限的理由：
  限速度是为了和真机同速（真机被 `arm_follow_server` 的梯形 slew 限在 `--max-speed 2.0` /
  `--max-accel 3.0`），而且它就是 `--ik exact`「精确或持位」判据的来源；限步长
  （`MAX_STEP_RAD` `0.08` rad/帧）与速度无关，挡的是 IK 在冗余/近奇异处一帧跳变。
- **取消速度限制**：把 `MAX_JOINT_VEL` 改到 `100.0`（60 Hz 下 `100·dt ≈ 1.67 rad/帧`，
  等价关掉）。文档同时写明三条前提：`--ik pin` 的上限在 pink `VelocityLimit`
  （`pin_ik.py`，默认 `2.0`）**不**受这个常量管；真机另有两层限幅（follow server +
  固件/电机），所以只改这里不会让真机变快；`MAX_STEP_RAD` 不该一起放开。
- **控制逻辑未改**：`_velocity_box` 与 pybullet 的步长钳制保持原样，本次只动常量取值与文档。

### 手柄坐标系：修正错误注释 + 旋转参考系开关
- **修正注释前提**（行为不变）：`teleop_sim.py` 原写「WebXR 手柄系 +x 右、+y 上、**+z 前**」
  ——那是**左手系**三元组。W3C WebXR Device API §6.2 逐字是
  `+X Right, +Y Up, -Z Forward`（**右手系，前是 −z**）。用探针从 9777 帧真实数据反解，
  映射矩阵与实测**逐元素一致**（差 5.6e-16）⇒ 错的是注释里的推理前提，不是代码；
  注释已按规范改写并标注「重标定时别照旧注释推」。同时修掉 `CTRL_AXIS_REMAP` 那句
  「绕世界轴转」的同类错误（恒等 + 右乘其实是绕**末端局部**轴）。
  另记一条数值结论：`diag(1,-1,-1)` 共轭 **≡** 逐轴符号 `[1,-1,-1]`（差 0.0），
  所以「翻一根轴」与「绕某轴转 180°」在 det=+1 的那几个符号上是同一件事 ——
  改 `axis_remap` 必须同时把符号归零，否则修正两次。
- **新增 `--control-frame {tool,base_link}`**（**默认 `tool` = 现状，便于回退**）：
  `tool` = 手柄自身系增量 → 末端局部系右乘（现在的行为，逐位不变）；
  `base_link` = 增量搬到基座系后左乘 ⇒ 工具绕**世界轴**转，且让旋转这一半与平移那一半
  同在基座系，消掉「两半不同源」。配套 `--base-link-signs`（换模式必须换一组符号：
  两个系读同样三个数，在一边对的镜像在另一边就是错的）。
  判据已进冒烟测试 `[8]`：同一手部旋转在两个末端朝向下，世界转动轴夹角
  `tool 58.4°` vs `base_link 8.5°`。
- **新增 `--manip-weight`**：`--ik pin` 接上 pink 的 `ManipulabilityTask`（最大化
  Yoshikawa 操纵度），把冗余自由度推向雅可比更良态的构型 —— **预防**奇异，而不是事后再削速度。
  用真实手柄轨迹 A/B：`0.05` 把 σ_min<0.02 的帧从 24.2% 降到 **3.7%**，`0.2` 降到 **0.0%**，
  且 TCP 路程**变长**（1.569→1.716）—— 因为基线本来就在奇异区丢掉里程；精度代价另用探针
  新加的 `|ee_after − ee_target|` 度量（基线中位 0.03 mm / p90 1.39 mm）。
  ⚠ 单臂 URDF 的 `ee_link` 有 JOINT/BODY 两个同名 frame，必须传 `pin_ik` 里已有的唯一别名，
  否则 `ManipulabilityTask` 抛 `Several frames match the filter`。
- **修掉一个真 bug**：`Arm.ik()` 里 pybullet 的 `getJointInfo` 结果局部变量名也叫 `info`，
  会遮蔽诊断回填用的 `info` 形参 ⇒ 一传诊断就 `TypeError`。由冒烟测试发现（pybullet 后端
  路径，之前一直跑 pin 故未触发）。已改名 `jinfo`。
- 探针每帧新增记录 `control_frame`；`docs/troubleshooting.md` 补「定位关节跳变」小节。

### 启动脚本改为 Python，参数取代环境变量
- **`scripts/run_teleop_webxr.sh` → `scripts/run_teleop_webxr.py`**：原来的 17 个环境
  变量改为命令行参数（`--dual` / `--ik` / `--viewer` / `--real` / `--hand` / …），
  一张 `--help` 就是总表。**脚本只用标准库**，可由系统 `python3` 直接跑
- 行为逐条保留：解释器解析、source ROS2、RViz2 的 URDF mesh 路径重写 + 参数 YAML、
  CAN 接口按需 up、清残留、端口检查、前台跑服务、退出清理
- **修好三处 shell 版做不到的**：① 命令缺失（无 conda/ss/ip）不再崩；② 收到 SIGTERM
  也走清理（原来临时 URDF/YAML 会残留、子进程成孤儿）；③ 服务与 ROS 子进程纳入跟踪，
  退出时按 PID 精确终止后再清扫
- 解释器探测补上 **venv 布局**（`~/venvs/metavr-arm/bin/python`）—— 原脚本只探
  conda 根，venv 装的机器上命不中
- `make teleop` 改为参数直通：`make teleop ARGS="--dual --viewer rviz"`
- **新增** `--tcp-offset`（末端偏移量，本体系）与 `--deadband-scale`（静止死区阈值乘数）
  两个开关；`--tcp-offset` **出厂默认 `0 0 0.18`**（本台架夹爪的法兰→抓取点距离，
  权威定义是 `teleop_sim.DEFAULT_TCP_OFFSET`；启动脚本里那份副本由测试守护不漂移），
  传 `0 0 0` 即直接控法兰。并在 README 记录：出厂死区在 60Hz 下等于平移 1.8cm/s /
  旋转 0.18rad/s，低于它机械臂完全不动

### 新增 exact IK 后端（精确任务速度，无阻尼）
- **`src/litearm_teleop/exact_ik.py`**：`exactTaskVelocity` 的 NumPy 移植（移植自
  `litearm-teleop-vr` 的 `task_velocity.hpp`）。截断伪逆 + 零空间区间求交 + 残差回代实测，
  **无阻尼** ⇒ 增益恒为 1 或 0、不随位姿变化；失败分「盒不可行 / 残差超限 / 雅可比无方向
  / 形状不匹配」并指名违约关节与超出量
- `--ik exact`：与 `pin` / `pybullet` 并存，两条老路径不动。实测手→臂稳态增益
  **exact 1.0000 vs pin 0.9967**（5cm/s）
- TCP 雅可比取在被控点（`calculateJacobian` 的 `localPosition` 即在 URDF link 原点系，
  与 `tcp_offset` 定义一致）

### 商业化整理：清理冗余 + 环境文档 + 双臂标定
- **删除冗余**：`logs/`（运行日志 + 自签名证书）、`litearm/export.log`、
  `litearm/urdf/litearm.csv`（SW2URDF 中间产物）、`dual_litearm_column_description/`
  的 `mujoco_viewer.py` + `robot.xml`（MuJoCo 已删后的残留）、ROS1 旧 launch
  （`display.launch`/`gazebo.launch`）、空 `src/litearm_teleop/assets/`
- **新增** `docs/environment-setup.md`：面向新用户的完整环境文档（系统要求/
  依赖清单/一键安装/网络镜像渠道）；README 目录结构与数据流更新
- **修正** docs 过时路径（`assets/litearm_urdf/` → 根目录 `litearm/`、
  `dual_litearm_column_description/`）；troubleshooting 补 RViz 三坑
- **双臂标定**：平移/旋转轴映射独立于单臂（`DUAL_TRANSLATION_REMAP_LEFT/RIGHT`），
  右臂 `rotation_signs=[-1,1,-1]`，左右臂镜像协作
- **修复** run 脚本三处 RViz 显示 bug：CRLF 传参 → `--params-file`；mesh 裸路径 →
  `file://` 前缀；单臂误用双臂 rviz config

### 双臂左右手遥操作（侧装立柱对称协作）
- **新增**双臂模式：`teleop_sim.py` 从单臂重构为 `Arm` 类 + `SIDE_CONFIG`，
  支持 left/right 两臂各自独立的轴映射常量与遥操状态
- **模型加载**：直接加载**单个双臂+立柱 URDF**（含 `column_base_link`），一个
  robot_id。单臂 IK 时构造全 14 维参数——目标臂自由、另一臂 `lo=up=当前值` 锁死
  （否则 restPoses 只作用到前 7 关节，右臂 IK 无法收敛）
- **对称协作**：左右臂轴映射符号相反（`SIDE_CONFIG`），同一手柄动作在两臂
  产生镜像末端运动；验证左右臂 IK 各 0.3mm、对称位移 27.5mm
- **`ws_echo_server.py`** 扩展到左右手：同时解析 `leftController`/`rightController`，
  各自 gripActive 边沿独立（`--dual` 加载双臂）
- **遥操语义**：速度式 delta 映射（手柄逐帧差驱动），gripActive 按下=启动、
  松开=暂停，启动用真实位姿作参考（无跳变）
- 单臂模式向后兼容，`tests/test_teleop_sim.py` 冒烟测试仍通过

### 删除 zenoh：ws 明文单进程遥操
- **删除**：zenoh 全部逻辑与依赖（`eclipse-zenoh`）、
  `scripts/webxr_zenoh_server.py`、`scripts/zenoh_controller_subscriber.py`、
  `scripts/test_webxr_teleop.py`、`scripts/gen_webxr_cert.sh`、`scripts/webxr_page/`
  （旧 HTTPS 页面 + 自签证书方案，数据源已换成外部 ws 明文客户端）
- **改造** `scripts/ws_echo_server.py`：从回显工具变为单进程遥操服务——ws 明文
  接收手柄 JSON（`leftController`/`rightController`，含 position/quaternion/
  gripActive/trigger），同进程直接驱动 `teleop_sim`
- **遥操语义变更**：gripActive 上升沿=开始/重新锚定（原 A 键），下降沿=暂停
  （原 B 键）；右手手柄位姿为遥操输入
- **改造** `src/litearm_teleop/teleop_sim.py`：删除 `open_zenoh_session`/
  `_ZenohLatest`/`_publish_endeff`/zenoh 订阅与发布，暴露纯回调接口
  `on_controller_payload`（外部直接调用）
- **依赖**：`eclipse-zenoh`+`aiohttp` → `websockets`
- 冒烟测试 `tests/test_teleop_sim.py` 不变仍通过；新增 ws 端到端验证
  （gripActive 边沿锚定/暂停 + 位姿驱动 IK）

## [Unreleased]

### 架构极简化：pybullet + zenoh 单进程遥操仿真
- **删除**：third_party/Open-Teach 全部逻辑、pylitearm（Pinocchio IK 依赖链）、
  MuJoCo（含 build_mjcf.py/model 生成物）、全部 ZMQ 通信与死代码、
  LIBERO/robosuite 遗留环境（metavr.yml）、相机流（原已无消费者）
- **新增** `src/litearm_teleop/teleop_sim.py` — 唯一 PC 端程序（单进程）：
  zenoh 订阅 `metavr/controller/right` 手柄位姿 → 遥操语义（A 键锚定/B 键暂停/
  轴映射/10 帧平滑，数学与原 LiberoSimOperator+LiteArmSimEnv 逐行等价，
  删除 rel×50/×5 编码往返）→ pybullet `calculateInverseKinematics`
  （当前构型为种子 + null-space 阻尼，60Hz 连续）→ 纯位置控制直驱关节
  （0.08 rad/帧限幅）→ pybullet GUI 窗口 + zenoh 发布 `metavr/endeff`
- **URDF 资产入包**：`src/litearm_teleop/assets/litearm_urdf/`
  （URDF + 9 STL，package:// 路径修复为相对路径，几何唯一真源）
- **全链路 zenoh**：WebXR 页面 → wss 8443 → zenoh → 仿真；无 ZMQ
- **单 conda 环境** metavr-arm（pybullet 3.2.7 + eclipse-zenoh + aiohttp），
  `environments/setup.sh` 单环境一键重建
- 纯手柄输入：删除手部轴显示与掌心重建数学
- 冒烟测试 `tests/test_teleop_sim.py`（home 位姿 / IK 0.6mm / 遥操语义
  43.4mm / B 键暂停静止 / IK 连续性 0.022 rad）；E2E 测试改 zenoh 断言并提速

## [0.1.0] - 2026-08-19

首个可用版本：Quest 3 → LiteArm（MuJoCo 仿真）VR 遥操作。

### 新增
- LiteArm V4 七轴机械臂 MuJoCo 仿真（URDF 原生编译 + pylitearm pinocchio IK）
- Quest 3 手部追踪遥操（复用 Open-Teach 管线，头显 App 无需改动）
- 纯位置控制模式（无动力学，60Hz IK + qpos 直接控制）
- 电脑端 MuJoCo 交互窗口 + 头显相机流双显示
- 基座/末端/手部三组坐标轴可视化（含原始手部轴调试显示）
- 一键启停脚本 + 手部平移映射标定脚本
- 键盘/手势双模式控制（回车=开始 / 食指捏合=开始）
- 奇异位形预警与 best-effort IK 降级

### 修复（相对上游 Open-Teach）
- 探测器三通道顺序阻塞 → poller 多路复用
- 手部轴显示 det=-1 静默失败 → 轴式右手化构造
- 手部系左手标架符号问题 → 可配置逐轴映射 + flip_rotation 开关

### 已知限制
- 无夹爪（LiteGrip 未集成）
- 头显 App 长时间运行后手部追踪可能冻结，需重启 App
- 旋转映射的左手系符号需按体感微调（flip_rotation）

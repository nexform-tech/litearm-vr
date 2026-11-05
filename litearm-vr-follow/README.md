# litearm-vr-follow

LiteArm 的**真机硬件驱动服务**：臂收 7 个关节角让它跟上，夹爪收 `close`/`open` 驱动开合，
灵巧手收 `close`/`open`+左手/右手驱动握拳/张开。

VR 遥操链路里最靠近硬件的一环：

```
litearm-vr-teleop  ──TCP :9001/9002──▶  臂跟随服务   ──▶  STM32  ──CAN──▶  臂电机
   （解算关节角）                          （跟随执行）
litearm-vr-teleop  ──TCP :9003───────▶  夹爪服务      ──▶  socketcan ──CAN──▶  夹爪 DM 电机
   （trigger 边沿）                        （close/open）
litearm-vr-teleop  ──TCP :9004───────▶  灵巧手服务    ──▶  linkerbot SDK ─CAN─▶  L6 灵巧手
   （trigger 边沿）                        （close/open+side）

status_aggregator.py ──TCP :9001/9003─▶ 各服务(ping/status，只读) ──Zenoh──▶ litearm/v4/arm/vr_arm
   （汇总状态，100Hz/20Hz）                 （收状态、不驱硬件）              litearm/v4/grip/vr_grip
```

它**不做逆解、不碰头显数据**——上游给什么（关节角 / 开关），就让硬件做什么。

除了「收命令 → 驱硬件」，本仓还带一个**只读的状态汇总**：`status_aggregator.py` 经 TCP
轮询各服务的实测状态，汇成一条 `FollowStatus` 用 Zenoh pub 出去（见下）。

## 两个版本，二选一

| | `arm_follow_server_stm32.py` | `arm_follow_server_sdk.py` |
| --- | --- | --- |
| 到机械臂 | 串口 `/dev/ttyACM0` | socketcan `can0` |
| 控制律在哪 | STM32 固件里 | 主机上（`pylitearm`） |
| 额外依赖 | `arm_console.py`（见下） | `pylitearm` |
| 启动 | `./run_stm32_server.sh` | `./run_sdk_server.sh` |
| 无硬件自测 | `./run_stm32_comm_test.sh` | `--dry-run` |

⚠️ **两个版本互斥**：它们驱动同一批电机，不能同时跑。

⚠️ **真机会真实运动**。运行前确认机械臂上电、工作区无人、急停在手边。

## 配置：`follow.yaml`

所有 `run_*.sh` 的参数都来自同目录的 **`follow.yaml`**——改那个文件、脚本就按新值启动，
**不用再传形参**：

```bash
./run_sdk_server.sh            # 读 follow.yaml 的 arm_sdk 段
./run_gripper_server.sh        # 读 gripper 段
```

优先级 **命令行 > follow.yaml > 脚本里的出厂常量**（把某个键删掉 = 回退出厂值，不会报错）。
命令行仍可临时覆盖：

```bash
./run_sdk_server.sh --tcp-port 9002                       # 覆盖 arm_sdk.tcp_port
FOLLOW_CONFIG=/path/to/other.yaml ./run_sdk_server.sh     # 换一份配置
```

段落 ↔ 脚本：

| 段 | 脚本 | | 段 | 脚本 |
| --- | --- | --- | --- | --- |
| `arm_stm32` | `run_stm32_server.sh` | | `hand` | `run_hand_server.sh` |
| `arm_sdk` | `run_sdk_server.sh` | | `aggregator` | `run_status_aggregator.sh` |
| `gripper` | `run_gripper_server.sh` | | `stm32_comm` | `run_stm32_comm_test.sh` |

各键含义与取值都写在 `follow.yaml` 的注释里。解析器 `follow_config.py` 只用标准库（不依赖
PyYAML）；`python3 follow_config.py --print <段>` 可先看某段会翻成什么命令行。

## 夹爪服务（`gripper_server.py`）

夹爪 DM 电机（CAN ID 0x08，DM4310）由 `gripper_server.py` 单独驱动，与臂的 follow server 并列
（同属「收指令、驱电机」的硬件层）：

```bash
./run_gripper_server.sh --can can0 --port 9003         # 真机夹爪
```

协议：TCP，一行一条 JSON，`{"cmd":"close"}` / `{"cmd":"open"}` / `{"cmd":"ping"}`（回 `{"ok":true,"q":…,"tau":…}`）/ `{"cmd":"status"}`（**零 I/O** 回状态缓存，额外带 `enabled`/`error_code`/`fresh`/`age_ms` + 环诊断 `fault_code`/`target_rad`/`cmd_rad`/`t_mos`/`t_coil`，供 20Hz 轮询）。

`close` / `open` 都只做一件事：**投一个目标角度**（`close` → 标定闭合限位，`open` → 标定张开限位），立刻回执。
真正的驱动是服务里**一条 ~200Hz 的控制环**（照 `litegrip-cpp` 的 `ControlLoop`，见
[`docs/gripper-control-loop-rewrite.md`](docs/gripper-control-loop-rewrite.md)）：每拍
「读反馈 → 看门狗/温度/故障 → 轨迹按 `--speed` 限速推进 → 力矩预算分给 kp/kd → 帧内位置钳位 → 发 MIT 帧」。

三条要点，改这一块前必须先懂（否则会退回老毛病）：

- **帧里的 `q` 是「命令轨迹」，不是「最近读到的位置」。** 拿最近一次读数当设定值，与同一时刻的实测
  天然有偏差（实测 ~0.7 mm），`kp × Δ` 就是一牛·米级的持续力，会把夹爪顶死在限位上形成 ~1.2s 的松弛振荡。
- **帧里要求的力被几何封住**：`q_frame = 实测 + clamp(轨迹 − 实测, ±预算/kp)`，所以撞上工件/硬限位时
  帧里要求的力矩**不超过预算** —— **夹持力就是 `--force`**，工件让位时误差变大、力自动保持，不需要
  第二套「保力」模式，也不会「一次没到位就收工」。
- **目标是个变量，不是一次动作**：改目标只覆盖它，下一拍轨迹就朝新目标走（≤5ms 换向），没有
  「等这一片跑完」的延迟。**没有任何分片、重试、堵转判据。**

参数（全部可从命令行给，默认值是保守的起步档）：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `--speed` | 50 mm/s | 命令轨迹的推进速度上限（**不是** MIT 帧的 dq 字段，那恒为 0） |
| `--force` | 20 N | 闭合侧的**力矩预算**（×0.1 = 2 Nm）= 夹持力上限 |
| `--rate-hz` | 200 | 控制环周期 |
| `--kp` / `--kd` | 20 / 0.5 | 增益上限；实际取值还会被预算按最坏情况压小 |
| `--max-pos-error-mm` | 10 | 预算分配用的「最坏位置误差」：`kp ≤ 预算/它`。**调小 = 更硬**（如 5 → kp≈19.5） |
| `--max-fb-vel` | 1 rad/s | 预算分配用的「最坏反馈速度」（决定 kd 上限） |
| `--feedback-timeout` | 0.5 s | 多久没有新反馈帧算掉线 → 改发零增益帧 |
| `--temp-limit` | 80 ℃ | MOS/线圈过温上限 → 安全停（零增益帧） |
| `--status-hz` | 20 | **还没收到目标**（limp）时刷状态缓存的频率 |

> 抖动换过四次实现，根因各不相同，**改前先读 [`docs/gripper-jitter-analysis.md`](docs/gripper-jitter-analysis.md)**：
> ① 自写「纯力矩保力 + open 打断」→ 空爪时纯力矩把夹爪一路挤进闭合硬限位、越挤越死，随后
> `open` 第一段脱开位移太短、被 SDK 的堵转判据误判成「堵转」，表现为来回寸动；
> ② 改用 SDK `close()` / `open()` → 位置环直推会压坏工件；
> ③ `close` 改用 SDK `grasp()` + 分片 → 每片重跑整段闭合斜坡，分片边界一松一紧；
> ④ 空闲保活用配置增益（kp=100）跑位置环 → 设定位取不到同一时刻的实测位置，顶死限位形成 ~1.2s 振荡。
> 别为了让某一条现象消失就把上一条改回去。

**启动即 disable**：DM 电机上电默认保持在出厂零位（0 ≈ 闭合），故服务启动后**先发一帧 0xFD 失能**，
让夹爪处于 limp，避免「一上电夹爪就闭合」。失能只需连通、不要求标定；**还没收到目标时**环保持
limp（不使能、不发帧），收到第一个目标才按需 re-enable。要保留旧行为加 `--no-startup-disable`。

**启动即 open 归位**：服务启动后自动投一次**张开**目标。头显遥操的扳机是**切换**语义——第一下 =
闭合；夹爪先落在 open，第一下扳机才是「闭合」。⚠ 这次归位**不算**「收到命令」，所以
`status` 里的 `target_ratio` 开机后仍是 `None`（契约要的是「操作者最近一次命令」）。
链路不通 / 未标定时它会被丢弃、保持 limp，不会误动。要关掉加 `--no-startup-open`。

**绝不静默停流**：母线静默 ~100ms 电机即自锁成 0xD 通信丢失故障（之后既读不到反馈也收不了命令）。
所以掉线 / 过温 / 电机故障的出口都是**发零增益帧**（不是不发）—— 与 C++ 的 `safe_stop()` 同一条理由。

依赖 `litegrip` SDK（脚本把 `~/sl/litegrip-python/src` 加进 `sys.path`，可用 `LITEGRIP_SDK`
覆盖；装了包则无需设置）。SDK 需 JSON 标定文件描述行程，默认出厂标定；本机实测的标定要显式
给出（`--calib PATH` / `--mount normal|reverse` / 环境变量 `LITEGRIP_CALIB`），**未标定会拒绝移动**。

⚠️ 夹爪 CAN 需先 up（1Mbps classic）：`sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up`

## 灵巧手服务（`hand_server.py`）

L6 灵巧手由 `hand_server.py` 单独驱动，与臂的 follow server、夹爪服务并列（同属「收指令、驱电机」的硬件层）：

```bash
./run_hand_server.sh --can can0 --port 9004            # 真机灵巧手
```

协议：TCP，一行一条 JSON，`{"cmd":"close","side":"left"}` / `{"cmd":"open","side":"right"}` / `{"cmd":"ping"}`。
`side` 为 `"left"` / `"right"`，即左右手各自握拳/张开。底层调 `hand_grasp` 的 `left_close/right_close/…`
（**阻塞**，约 1.5s），故入队后由后台线程串行执行（左右手共用一条 CAN 总线，本就无法并行）。
依赖 linkerbot SDK（`hand_grasp` + `src/`），由 `run_hand_server.sh` 压进 `PYTHONPATH`（默认
`$HOME/sl/linkerbot-python-sdk-main`，可用 `LINKERBOT_SDK` 覆盖）。

⚠️ 灵巧手 CAN 需先 up（1Mbps classic）：`sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up`

## 状态汇总服务（`status_aggregator.py`）

补齐**接收半边**：把臂 + 夹爪的实测状态（q/dq/tau/温度/故障、夹爪位置/扭矩/使能）
**按臂身份分 topic** pub 到 Zenoh：

| 设备 | topic |
| --- | --- |
| 臂 | `litearm/v4/{armA,armB}/vr_arm` |
| 夹爪 | `litearm/v4/{armA,armB}/vr_grip` |
| 灵巧手（v1 占位） | `litearm/v4/{armA,armB}/vr_hand` |

中段 `armA`/`armB` 是**臂身份**，与 bridge 的话题契约 `litearm/v4/{arm_id}/{channel}` 对齐。
**follow 服务不认识臂身份**——它只知道自己开的串口（回包 `port`）/ CAN 口（回包 `can`），
永远不知道自己是左还是右。所以身份不能从回包推，必须**外部配进来**：

```bash
# armA=左臂: 串口 @ follow 端点；IFACE 只用于和回包对拍（对不上告警一次）
./run_status_aggregator.sh \
    --arm armA=/dev/serial/by-id/usb-…-if00@127.0.0.1:9001 \
    --arm armB=/dev/serial/by-id/usb-…-if02@127.0.0.1:9002 \
    --gripper armA=can0@127.0.0.1:9003 --gripper armB=can1@127.0.0.1:9004
./run_status_aggregator.sh --gripper none                       # 关夹爪
./run_status_aggregator.sh --prefix litearm/v4                  # 改 topic 前缀
python3 status_listen.py                                        # 通配订阅看看（调试用）
```

（前缀 `litearm/v4` 可用 `--prefix` 改；每条消息就是该设备的状态本体，不再套信封。
早先合并成的**单条** `FollowStatus`（`litearm/v4/follow/status`）**已不再发布**。）

**只读**：只发 `ping` / `status`，**永不**发 `start`/`q`/`movej`/`home`/`close`/`open`，也**永不开**
串口/CAN；可与遥操发送端并行跑，互不干扰（独立 TCP 连接）。

- **消息类型**：`status_msgs.py` 的 ROS 风格 `@dataclass`（`ArmStatus` / `GripperStatus` /
  `HandStatus`；`FollowStatus` 合并封装仅留解析/测试），序列化走 `json.dumps(asdict(...))`
  ——**非** ROS2 线格式，不可与真 ROS2 互操作。
  臂 `ArmStatus` 归一化 stm32/sdk 两版 `ping`（`fault` 优先、回退 `flags`；`q` 仅当 `len==7` 才收）。
- **臂身份 `arm_id`（topic 中段）**：`armA`/`armB`，由**桥的臂清单**给，配在 `--arm`/`--gripper`
  的 `ARM_ID=` 里。follow 服务本身给不出这个 —— 它只知道串口。
- **设备名 `device`**：payload 里的物理身份，取**回包里的串口/CAN 口**（臂 stm32 的 `port`，
  如 `/dev/ttyACM0`；sdk 的 `iface`；夹爪的 `can`，如 `can0`），回包里没有才退回配置的接口。
  与配置的 `IFACE` 对不上会**告警一次**（不阻断）。
- **频率**：默认发布 100Hz / 臂 ping 100Hz / 夹爪 status 20Hz（`--publish-hz`/`--arm-hz`/`--gripper-hz`）。
  夹爪走 `status` 缓存（零 CAN I/O），故 20Hz 无压力；臂 ping 读的是 SDK 缓存帧，很便宜。
- **超时**：某设备超 `--stale-ms`（默认 1000）未更新 ⇒ 发布态置 `connected=false`，但**发布不停**；
  链路断则 1s 内退避重连。
- **Zenoh**：照搬 `litegrip` 的 `_base_config()`——`mode=peer` + 关多播 scouting/gossip + 显式
  `listen`/`connect`。默认 `listen tcp/0.0.0.0:17449`（17447/17448 已被臂/夹爪遥操占用）。
  订阅端必须知道这个端口（`--zenoh-listen/--zenoh-connect` 可改）。

依赖 `zenoh`（pip `eclipse-zenoh`；脚本缺它会报错退出）。

## 对外协议

TCP，一行一条 JSON（`\n` 分隔）。上游（`litearm-vr-teleop` 的 `real_arm_client.py`、`litearm-vr-bridge`）按这份契约说话。

### 命令

| 命令 | 作用 | 回包 |
| --- | --- | --- |
| `{"cmd":"start"}` | 开始跟随：使能 + 读当前角 + `following=True` | `{"ok":true,"cmd":"start","q":[7]}`；已在跟随则多一个 `"already":true`；使能失败 `{"ok":false,"error":…}` |
| `{"cmd":"q","q":[7]}` | 一帧 7 关节角目标，逐帧跟随 | `{"ok":true,"cmd":"q"}` |
| `{"cmd":"latch"}` | 就地持位 | `{"ok":true,"cmd":"latch"}` |
| `{"cmd":"stop"}` | 急停 | `{"ok":true,"cmd":"stop"}` |
| `{"cmd":"movej","q":[7]}` | 关节轨迹到位（**阻塞**到 `\|q-target\|<tol`） | `{"ok":bool,"cmd":"movej"}` |
| `{"cmd":"home"}` | 回零位（**阻塞**到 `\|q\|≈0`） | — |
| `{"cmd":"ping"}` | 探活 | 见下 |

`q` 与 `movej` 的区别：**`q` 是逐帧流的跟随目标**（高频、不阻塞），**`movej` 是一次性轨迹**（阻塞、等到位）。

> `movej` / `home` 会**阻塞这条连接**。客户端要么给它单独的连接，要么接受这条连接在此期间没有响应。

### `ping` 的回包——两版字段不同

| 字段 | stm32 版 | sdk 版 |
| --- | --- | --- |
| `state` | `mode_name`：`INIT` / `MOVE_J` / `MOVE_P` / `MOVE_JS` / `MOVE_MIT` / `MIT_ALL` / `EMERGENCY` | `ArmState.value`：`disconnected` / `moving` / `following` / … |
| `flags` | ✅ 置位的 flag 名列表（健康时为空） | ✗ 无 |
| `fault` | ✗ 无 | ✅ `[(joint, err), …]`（健康时为空） |
| `q` | ✅ | ✅ |
| `target` / `anchor` / `cmd_q` | ✅ | ✗ 无 |
| `joints` | ✅ `[(q, dq, tau, t_mos, t_coil, err), …]`（每关节温度/故障） | ✗ 无 |

消费方要**两个都认**：`fault` 取不到就回退 `flags`。两者语义一致——健康时都是空列表，非空即有故障。

## 依赖：`arm_console.py`

stm32 版有 `from arm_console import (LiteArm, CMD_…)`，这个文件**不在本仓库**，来自
[`litearm-stm32`](https://gitee.com/yudao_hz_1/litearm-stm32) 仓库的 `tools/` 目录
（STM32 0xA5 协议封装 + `LiteArm` 串口类）。

代码找 import 的顺序是**先看自己所在目录，再走 `PYTHONPATH`**，所以两种放法都行：

```bash
# 法一：PYTHONPATH（run_stm32_server.sh 用的就是这个）
export PYTHONPATH=/path/to/litearm-stm32/tools:$PYTHONPATH

# 法二：把 arm_console.py 直接放到本目录旁边
```

> `run_stm32_server.sh` / `run_stm32_comm_test.sh` 按这个顺序找 `tools` 目录：
> `$LITEARM_TOOLS` → 同级目录的 `litearm-stm32/tools` → `~/sl/litearm-stm32/tools`。
> 三者都对不上时脚本报错退出——此时把 `LITEARM_TOOLS` 指过去即可。

sdk 版依赖 `pylitearm`（走 `can0`）。`run_sdk_server.sh` 里有一段对 cmeel `pin 4.1.0` 的路径处理，
注释写了原因：绕开 ROS humble 自带的、按 numpy 1.x 编译的旧版 pinocchio。

## 运行

```bash
# 日常：改 follow.yaml 即可，直接跑
./run_sdk_server.sh
./run_gripper_server.sh
./run_hand_server.sh
./run_status_aggregator.sh

# 临时覆盖（命令行 > follow.yaml）
./run_stm32_server.sh --mode js --tcp-port 9001               # 免调增益
./run_sdk_server.sh  --dry-run                                # 无硬件验协议
```

`--mode` 四选一，对应固件的四种跟随方式：`jf`（joint follow，**默认**）/ `js` / `j` / `mit`。

⚠️ **串口独占**：跑服务时别同时 `cat /dev/ttyACM0`，或用 `arm_console` 占着串口。

## 安全相关的坑（踩过，别改回去）

- **`latch` 必须走 `movej` 到当前角**，让固件在 `MOVE_J` 到位后增刚持位。
  **不能用 `move_js` 持位**——它内置的积分前馈在静止态会累积稳态误差，导致极限环震荡。
- `stop` 走固件的 `EMERGENCY_STOP`。
- 目标角超出关节限位时会被 clamp 到限位内并打日志（限流 2s 一条），不会硬撞限位。

## 来源

内容拆自 `vr` 聚合仓库（`e3c9145` 于 2026-09-16 创建，`dbebe58` 于 2026-10-14 更新）；
夹爪服务原为工控机上的 `gripper_server_can.py`，2026-10-15 收编入本仓为 `gripper_server.py`
（同日改用 `litegrip-python` SDK 驱动，弃用手写的 `damiao_socketcan` 闭环）；
灵巧手原为 teleop 仓的 `scripts/hand_control.py`（本地 CAN 驱动），同日按同规则迁入本仓为 `hand_server.py`。
`status_aggregator.py` / `status_msgs.py` / `status_listen.py` 为 2026-10-15 新增（状态汇总 + Zenoh 发布），
并给 `arm_follow_server_stm32.py`（ping 加 `joints`）与 `gripper_server.py`（加 `status` 缓存读取）做了**仅新增字段**的小改。
此后本仓库是这些文件的唯一归宿，`vr` 仓库已搁置。

### 遥操实测反馈与不切模式保持

`{"cmd":"state","request_id":123}` 是只读查询，不使能、不回零。回复含
`cmd=state`、原样 `request_id`、`valid`、`q`（7 个实测关节角，弧度）和
`age_s`。无效时 `q/age_s` 为 null，绝不返回 `target/cmd_q` 充当实测值。
STM32 在连续查询观察到固件状态序号推进后才有效；250 ms 未推进或查询中断后
重新验证。SDK 路径要求 `get_state()` 提供全部 7 轴的
`feedback.joints[].fresh/age_s`，缺失该元数据的旧 SDK 明确返回无效。

`{"cmd":"hold"}` 丢弃远处的旧遥操目标，优先保持新鲜实测角度；反馈不可用时
保持当前实际下发的指令。保持现有跟随模式与增益，不调用 movej、不重使能。
STM32 清除速度前馈；SDK 保持原有 joint_follow 循环（无反馈时 provider 返回
None，依 SDK 原有约定保持当前 q_cmd）。此命令用于 grip 松开或遥操输入失效。

遥操客户端通过 `request_state(side)`（内部限制 20 Hz）异步查询，
`get_state(side, max_age=0.25)` 返回有效实测状态或 None。客户端同时计入服务端
反馈年龄与请求往返时间，并在断线时立即清除缓存。部署须同时更新服务端与客户端；
不支持新查询或不能证明反馈新鲜度的后端不会被当成闭环测量来源。

`hold`重复发送保持第一次选定的位置，不会每次跟随测量噪声重新锚定；客户端在保持
期间通过后台查询周期重发hold保活。SDK跟随目标超过250ms未更新也清除旧目标。
SDK只读预连接需要`connect(read_only=True)`；start时转换为正常连接。

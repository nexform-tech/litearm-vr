# litearm-vr-bridge 设计方案

> 本文汇总本轮讨论的全部结论，作为实现依据。日期：2026-09-17。

## 1. 背景

### 1.1 现在有两条独立链路

**链路 A：VR 遥操（单向，只控不看）**

```
头显 litearm-vr (Godot)
   │  ws 8765，手柄位姿 ~90Hz，只发最新帧
   ▼
teleop（litearm-vr-teleop 仓库，submodule）
   │  遥操语义（grip 锚定/暂停）→ 坐标映射 → pybullet/pin 逆解 → 7 关节角
   ├─▶ pybullet 3D 窗口
   │  TCP 9001
   ▼
arm_follow_server（litearm-vr-follow 仓库，submodule；sdk / stm32 二选一）
   ▼
机械臂本体
```

**链路 B：上位机日常操作（双向）**

```
litearm-studio ──ws 7449──▶ litearm-server（控制器守护）──▶ 机械臂本体
                              （~50Hz 全量状态广播）
```

两条链路都能驱动机械臂，**互斥**，不能同时占用同一批电机。

### 1.2 三个缺口

1. **状态黑箱**：teleop 只收不发，头显是否在线、位姿帧率、端到端延迟、是否正在跟随，
   外部一概不知道。上位机拿不到 VR 链路状态，也拿不到真机在 VR 跟随下的姿态。
2. **掉线无兜底**：头显掉线（或 TCP 断开）时，跟随服务的 `_q_target` 冻结在最后一帧，
   `joint_follow` 线程继续以跟随刚度伺服该目标 —— 机械臂看起来"停住了"，但**没有告警、
   没有进入持位态**。SDK 路尤其如此（只有 STM32 固件自带命令看门狗）。
3. **归属不明**：VR 在控还是上位机在控，界面没有任何判断依据。

另有一项待办：上位机与 teleop 使用的 **URDF 不是同一份**（关节轴向、限位、几何均不同），
3D 预览会与 teleop 的逆解模型不一致。

### 1.3 为什么必须有一个独立服务

- 浏览器/前端无法监听 8765，也无法起停本机进程；
- 头显是常推数据的设备，服务必须独立于界面窗口长期存活；
- 掉线兜底需要的"最后一帧时间"只有在收帧位置才拿得到；
- 归属判定需要同时看到头显侧与机械臂侧。

## 2. 定位与边界

### 2.1 做什么

1. **收帧与分流**：监听 8765 收头显原始帧，逐帧转发给 teleop。
2. **状态汇总**：把头显侧、真机侧、进程侧的信息合成一条状态流给上位机界面。
3. **掉线兜底**：无帧超时后主动让机械臂就地持位，并告警。
4. **进程启停**：一键拉起/停止 teleop、arm_follow_server 等进程。

### 2.2 不做什么

| 不做 | 原因 |
| --- | --- |
| 逆解 | 由 teleop 负责，本服务只转发原始帧 |
| 驱动机械臂 | 由 `arm_follow_server`（VR 场景）或 `litearm-server`（平时）负责 |
| 界面 | 由 `litearm-studio` 负责，本服务无窗口、无 UI |
| 向前端推原始 90Hz 帧 | 前端只需要状态与关节角结果；推原始流会拖垮界面并引入反压 |
| 解析上游的运行环境 | conda 环境、`arm_console`、ROS2、灵巧手 SDK 由上游启动脚本自己解析；本服务复制一份就等于制造"手工能起、服务起不来" |
| 修改上游仓库 | `litearm-vr-teleop` / `litearm-vr-follow` 以 submodule pin 在具体 commit 上，只通过它们的启动脚本与端口对接（上游在 submodule 之外时用 `TELEOP_REPO` / `FOLLOW_REPO` 指路） |

## 3. 部署与拓扑

本服务必须与 teleop **同机部署**（帧转发走 loopback），以 systemd 常驻：
开机自启、崩溃自动重启。它是链路的唯一入口，头显只会连这一个地址。

```
                     同一台机器（跑 teleop 的那台）
        ┌───────────────────────────────────────────────┐
头显 ──▶│ litearm-vr-bridge :8765                       │
  ws    │      │ 原始帧转发                              │
        │      ▼                                        │
        │   teleop :8766 ──TCP 9001──▶ arm_follow_server│──▶ 真机
        │      ▲                            ▲           │
        │      └─ ping / start / latch ─────┘           │
        └───────┬───────────────────────────────────────┘
                │ ws（状态摘要 + 动作）
                ▼
        litearm-studio（使用者电脑）
```

**关键点：teleop 不需要改代码**，它自带 `--port` 参数，从 8765 改到 8766 即可。

## 4. 对外接口（给 litearm-studio）

### 4.1 状态推送

话题 `litearm/v4/{arm}/vr_status`（`{arm}` = `ZENOH_ARM_ID`，默认 `armA`），
**pub/sub，整包推送，建议 10Hz**（3D 预览要更平滑时可对 `arm.q` 单独提频到 20–30Hz，
但那要新开一个后缀，不要往这条上叠）。载荷：

```json
{
  "ts": 1787641468151,
  "bridge": { "online": true, "forward_cost_ms": 0.3 },
  "headset": {
    "online": true,
    "fps": 90.2,
    "latency_ms": 18,
    "last_frame_age_ms": 4,
    "following": true,
    "frames": 128450
  },
  "arm": {
    "source": "follow_server",
    "q": [0, -0.07, -0.09, -1.7, -1.46, -0.03, -0.79],
    "state": "following",
    "fault": []
  },
  "processes": { "teleop": "running", "follow_server": "running" },
  "owner": "vr",
  "fallback": { "active": false, "reason": null },
  "events": []
}
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `bridge.forward_cost_ms` | 本服务"收帧→转发"自身耗时，正常应 < 1ms；变大即说明本服务卡了 |
| `headset.online` | 最近一帧是否在阈值内（见 6.3） |
| `headset.latency_ms` | 端到端延迟 = 收到帧时刻 − 帧内 `timestamp`（无线段） |
| `headset.following` | 是否处于跟随（由 grip 语义推断） |
| `arm.source` | 真机状态来源：`follow_server` / `controller`（见第 8 节待实测） |
| `arm.q` | 真机 7 关节角（rad），供 3D 预览 |
| `owner` | 谁在控臂：`vr` / `console` / `none` |
| `fallback` | 是否处于兜底持位态及原因 |
| `events` | 一次性事件（告警、动作回执），消费后清空 |

### 4.2 动作接口

话题 `litearm/v4/{arm}/vr_rpc`，**queryable（请求-应答）**。请求 `{"action": ..., ...}`：

| 动作 | 说明 |
| --- | --- |
| `teleop.start` | 开始 VR 遥操（拉起数据链路 / 通知跟随服务 `start`） |
| `teleop.stop` | 停止遥操（`latch` 就地持位） |
| `estop` | 急停（转发 `stop`） |
| `process.start` / `process.stop` | 启停 teleop、arm_follow_server 等进程 |
| `ping` / `status` / `snapshot` | 判活；回执里附当前快照 |

**为什么用 query 而不是 pub。** 它是唯一能区分「命令没送到」和「整条链路断了」的手段：
桥挂住没死时 pub 下去没人处理，上位机只能等状态超时；query 超时就能给出结论。

回执形状对齐 `litearm-server` 的 `RpcReply`（多一个 `code` 便于码表）：

```json
{"ok": true,  "code": "teleop_started", "result": {"commands": [...]}}
{"ok": false, "code": "estop_sent_failed",
 "error": {"type": "UpstreamRefused", "message": "跟随服务不可达或拒绝",
           "details": {"commands": [{"action": "stop", "reason": "estop", "ok": false}]}}}
```

`ok` 取自**下发给跟随服务的每条命令的结果**，不是"桥收到了就算成功" —— 急停要停的是
机械臂，回执得说到那一步。没走到下游就失败的（未知动作、参数不对、正在退出）回
`code` 为错误类型的回执。

动作结果**同时**照旧写进 `events`：那是持久的、谁都能看见的；回执是即时的、只给发起方。

### 4.3 与 studio 的边界

- 平时（上位机自己控臂）：studio 直连 7449，本服务不参与。
- VR 场景：studio 订阅本服务；`owner` 字段用于做"二选一"互斥提示。
- **studio 不直连 zenoh**：浏览器只能走 WebSocket，它经一台
  `zenoh-bridge-remote-api` 进网络（见 `docs/DEPLOY.md`）。
- **前端只订摘要，不订原始帧。**

## 5. 对内接口

| 对端 | 协议 | 用途 |
| --- | --- | --- |
| 头显（litearm-vr） | WebSocket 服务端，监听 8765 | 收手柄帧；帧格式见 `litearm-vr/docs/PROTOCOL.md` |
| teleop | **zenoh pub** `litearm/v4/{arm}/vr_frame` | 逐帧转发原始帧（teleop 订阅） |
| arm_follow_server | TCP 9001，每行一条 JSON | 发 `start` / `hold` / `latch` / `stop`；`ping` 读 `q`/`state`/`fault` |
| litearm-server | ws 7449 | 暂不接入（见第 8 节） |

注意：9001 同时是控制口（谁连上谁能发 `q` 控臂，且无鉴权）。本服务对它的使用
**限定在 `start`/`hold`/`latch`/`stop`/`ping`**，绝不转发关节角流。

## 6. 关键行为设计

### 6.1 转发纪律（硬要求）

1. **只透传**：不合并、不节流、不重排、不改帧内容。
2. **转发路径上不碰磁盘、不等网络**：日志与统计放到旁路。
3. **前端只订摘要**：绝不把原始帧推给界面。
4. 目标：`forward_cost_ms` 稳定 < 1ms。

> 依据：`vr` 的文档明确指出"降采样到 60Hz 只会让间隔在 11/22ms 之间抖，对遥操有害无益"。
> 遥操手感的主要杀手是抖动，不是平均延迟。

### 6.2 延迟指标拆两段

- **无线段** = 本服务收帧时刻 − 帧内 `timestamp`（头显时间戳），正常 5–30ms。
- **本服务段** = 转发出去时刻 − 收帧时刻，正常 < 1ms。

两段分开暴露，便于定位"到底哪儿慢"。

### 6.3 掉线兜底（本项目最重要的安全职责）

阈值取自上游协议自身的建议"WS 断开或超过 ~500ms 没新帧，消费端应进入安全态"：

| 无帧时长 | 动作 |
| --- | --- |
| > 200ms | 界面上报黄色告警"帧流异常" |
| > 500ms | 发 `hold` 就地保持，进入兜底态（红色告警） |
| 帧流恢复 | **不自动恢复跟随**，需操作员重新按 grip（teleop 会重新锚定） |

约束：

- 兜底只允许发 `hold`（就地保持）与 `stop`（急停），**绝不擅自 `movej`/`home`**。
  跟随服务的 `latch` 实现是 `movej(当前角)`，**掉线兜底不能用它**（会产生一次
  MOVE_J 与到位增刚）；`latch` 只保留给操作员主动 stop。
- 兜底动作必须幂等：重复触发不重复下发。
- 主动断开手柄链路时，先不发帧、再 latch，避免"先松手再持位"的空窗。

### 6.4 状态机

```
IDLE ──帧到达──▶ FOLLOWING ──grip 松开──▶ LATCHED
  ▲                 │                       │
  │                 └──无帧 > 500ms──▶ FALLBACK
  └──────────────全链路停止───────────────┘
```

### 6.5 归属判定（owner）

- `vr`：跟随服务处于跟随态，且最近有帧。
- `console`：上位机侧自己上报（studio 自己知道在不在控）。
- 本服务只负责"VR 侧在不在控"，最终展示由 studio 合成。

## 7. 配置项

| 配置 | 默认 | 说明 |
| --- | --- | --- |
| `BRIDGE_HOST` / `BRIDGE_PORT` | `0.0.0.0` / `8765` | 监听地址（头显连这里） |
| `TELEOP_ENDPOINT` | `127.0.0.1:8766` | 传给上游 launcher 的 `--port`（桥不再连它） |
| `FOLLOW_SERVER_ENDPOINT` | `127.0.0.1:9001` | 动作与状态 |
| `ZENOH_ARM_ID` | `armA` | 话题里的 arm 段；要与 teleop、上位机、`litearm-server --arm-id` 一致 |
| `ZENOH_MODE` | `peer` | 会话模式（peer 既连出去也让人连进来） |
| `ZENOH_LISTEN_ENDPOINTS` | `tcp/0.0.0.0:7447` | 桥听在这；**同机的 litearm-server 默认也用 7447** |
| `ZENOH_CONNECT_ENDPOINTS` | 空 | 主动连谁；留空 = 等对端连进来 |
| `ZENOH_REPLY_TIMEOUT_S` | `20` | 动作应答上限；冷启动 `teleop.start` 约 14s，别往小调 |
| `FRAME_WARN_MS` | `200` | 帧流异常告警阈值 |
| `FRAME_FALLBACK_MS` | `500` | 兜底持位阈值 |
| `STATUS_HZ` | `10` | 状态摘要推送频率 |
| `LOG_LEVEL` | `INFO` | 日志级别 |
| 上游位置 | `submodules/` 下的两个 submodule | 从仓库布局推出；上游在 submodule 之外才用 `TELEOP_REPO` / `FOLLOW_REPO` |
| 受管进程 | 两个上游都托管 | 起的是上游自己的启动脚本；`TELEOP_CMD` / `FOLLOW_SERVER_CMD` 可整体替换，显式留空 = 不托管 |
| 机器特性 | 见 `deploy/bridge.env.example` | 串口 / CAN 口、正装 / 侧装、真机 / 仿真、灵巧手 / 夹爪 |

> 上游的运行环境（conda、`arm_console`、ROS2、SDK 路径）不在本服务的配置里：那是上游
> 启动脚本自己解析的，要覆盖就 export 它们认的变量（`TELEOP_SERVER_PY` / `LITEARM_TOOLS` 等）。

## 8. 待决策 / 待实测

1. **真机状态来源**：VR 在跑时，`litearm-server`（7449）是否可用？
   `pylitearm` 在连接前会做 0.3s 总线静默检查，检测到活跃控制报文会抛 `BusBusyError`
   拒绝连接。需实测：先起 litearm-server，再起 arm_follow_server，看是否被拒；
   若两者可共存，则 7449 可作真机数据源（数据更全），否则只能用 9001 的 `ping`。
   **实测前，实现的默认值取 `follow_server`（9001 `ping`）。**
2. **部署机器**：与 teleop 同机这一条已定；具体是控制器还是操作员工控机待确认。
3. **URDF 统一**：上位机 `public/description/litearm.urdf` 与 teleop 的
   `assets/litearm_urdf/urdf/litearm_urdf.urdf` 不同（关节轴向符号、限位、origin 均有差异），
   建议以 teleop 那份为准（逆解在那边算），同步到上位机。
4. **鉴权**：局域网内、无登录鉴权；若部署环境不可信，需加 token。

## 9. 里程碑

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| M1 转发 + 状态 | 监听 8765、逐帧发到 zenoh `vr_frame`、状态 `vr_status`、`forward_cost_ms` | 头显推到数据的全过程在界面上可见，帧率/延迟与头显侧一致 |
| M2 动作 + 兜底 | `teleop.start/stop`、`estop`、无帧超时自动 `hold` + 告警 | 拔掉头显电源，500ms 内机械臂进入保持并报警 |
| M3 进程启停 | 一键拉起/停止 teleop 与跟随服务；systemd 常驻、崩溃重启 | 冷启动一台机器，一条命令进入可遥操状态 |
| M4（可选）头显回显 | 配合 `litearm-vr` 接上已有但未使用的 `message_received` 信号 | 头显面板上显示跟随状态与告警 |

## 10. 附录：上游组件速查

### teleop（`litearm-vr-teleop`，submodule `submodules/litearm-vr-teleop`）

- 启动：`teleop/launch/run_teleop_webxr.py`（参数见 `--help`，默认值在
  `teleop/config/teleop.yaml`；本服务按那张表拼命令行）。找解释器、source ROS2、
  起 RViz、清残留进程、查端口都在这个 launcher 里，本服务不复制。
- 监听 ws（`--port`，默认 8765），收 `{timestamp, leftController, rightController}`。
- 遥操语义：右手柄 `gripActive` 上升沿=锚定/恢复，下降沿=暂停；扳机控制灵巧手/夹爪。
- 逆解：默认 `--ik pin`（pinocchio + pink 微分 IK，抗奇异）；`--ik pybullet` 是旧实现回退。
- 输出：本地 3D 窗口（或 RViz2）+ 可选 TCP 9001 转发。
- **不回传任何状态。**

### arm_follow_server（`litearm-vr-follow`，submodule `submodules/litearm-vr-follow`，二选一）

| | sdk 版 | stm32 版 |
| --- | --- | --- |
| 启动 | `run_sdk_server.sh` | `run_stm32_server.sh` |
| 链路 | CAN `can0` | 串口 `/dev/ttyACM0` |
| 控制律 | 主机（pylitearm JointFollowController） | STM32 固件 |

协议（TCP 9001，每行一条 JSON）：`start` / `q` / `movej` / `home` / `latch` / `stop` / `ping`。

- `ping` 只返回 `state` / `q`，加一个健康字段：stm32 版是 `flags`，sdk 版是 `fault`
  （语义一致，健康时都为空 —— 本服务的 `follow.py` 两个都认）。
- 没有状态广播，必须轮询。
- 客户端断开只打印日志，跟随状态不变（本项目的兜底正是补这个洞）。

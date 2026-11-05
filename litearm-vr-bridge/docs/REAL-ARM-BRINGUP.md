# 真机调试（Bring-up）

> 这篇是**给站在机器前面的人**看的：第一次把 VR 链路接到真机械臂上，或者换了一台机器
> 重新拉起。配置项含义见 [DESIGN](DESIGN.md) 第 7 节，日常启停见 [README](../README.md)，
> 部署与排障见 [DEPLOY](DEPLOY.md)。
>
> **先读完第 0、1 节再动手。** 第 2 节是四档推进，每一档都有"过了是什么样"，
> 不要跳档。

## 0. 先定机器：三条硬约束

| 约束 | 为什么 | 不满足会怎样 |
| --- | --- | --- |
| **STM32 的 USB 线插在谁身上** | 真机跟随服务要开 `/dev/ttyACM0`（STM32 是 USB CDC） | 起不来 —— 日志里是上游那句"串口不存在" |
| **metavr-arm 的 conda 环境在哪台** | teleop 要 pybullet / pinocchio / pink | 起不来 —— "找不到 metavr-arm 环境的 python" |
| **头显能不能连到那台** | 桥监听在那台的 LAN IP 上，头显按 IP 连 | 帧进不来 —— 假头显能通、真头显不通 |

⚠️ **STM32 不在"控制器"上。** 谁插着 USB 线，设备就在谁那儿（见 `litearm-vr` 那份
STM32 实操笔记）。所以"真机在哪台机器上调试"取决于机械臂的 USB 线插在哪。

于是只有三种形态：

- **A. 一台真 Linux（工控机 / 开发机）** —— 三条都能满足，**首选**。要做的只是把桥
  放过去：`git clone --recursive` + `git submodule update --init` + `./scripts/install.sh --user`。
- **B. WSL / 容器里** —— USB 要直通（WSL 用 `usbipd attach --wsl`），conda 环境得现建
  （上游 `environments/setup.sh`）；**WSL2 默认 NAT，头显连不进来**（桥的 `lan_addr`
  那行会打印一个 198.18.x.x 的地址，那是 WSL 自己的）。要么改 WSL 的 mirrored 网络 /
  做端口转发，要么这一档改用假头显（第 2 节档 1）。
- **C. Windows 直接跑** —— pybullet / pinocchio 在 Windows 上装起来很麻烦，不建议。

## 1. 上电前要定两件事

### 1.1 跟随时用哪种控制律（`FOLLOW_SERVER_CTRL_MODE`）

上游 `arm_follow_server_stm32.py --mode` 的四挡语义（抄自它的 `--help`）：

| 值 | 含义 |
| --- | --- |
| `jf` | JOINT_FOLLOW 0x08，**固件内伺服环 + 前馈** —— 上游标"推荐"，**需固件 ≥ 1.9.0** |
| `js` | MOVE_JS 关节伺服 |
| `j` | MOVE_J 轨迹解算 |
| `mit` | MOVE_MIT_ALL 透传（**无重力补偿，慎用**） |

桥默认传 **`js`**（上一个会话"对齐真机参数"时定的）。板上固件如果是 1.9.0 及以上，
应该试上游推荐的 `jf`：

```bash
FOLLOW_SERVER_CTRL_MODE=jf ./scripts/up.sh --link
```

**先读板上固件版本**（别猜）：

```bash
./submodules/litearm-vr-follow/run_stm32_comm_test.sh /dev/ttyACM0
# 第 2 步就是发 CMD_GET_FIRMWARE(0x41)，会打印 "Litearm1.9.0-*" 这样的版本串
```

### 1.2 两个环境变量指哪

上游脚本自己解析环境，桥只负责把环境传下去。这台机器上要 export 的：

```bash
export LITEARM_TOOLS=<litearm-stm32 仓库的 tools 目录>   # arm_console.py 在哪
export TELEOP_SERVER_PY=<envs/metavr-arm/bin/python>            # 找不到 conda 环境时才要
```

（`--doctor` 的 `follow_tools` / `teleop_python` 两项就是查这个，它们给的提示里带着
它会按什么顺序去找。）

### 1.3 安全前置（照 `deploy/bridge.env.example` 那条 ⚠ 来）

臂已上电、**工作区清空**、**急停在手**、重力补偿参数写好。
**首次按 grip 会 `movej` 回 `q_home`** —— 那是臂自己动，不是你控制的。

## 2. 四档推进

### 档 0 · 只读预检（不碰硬件）

```bash
./scripts/up.sh --doctor
```

**过了是什么样**：15 项里没有解释不清的 FAIL。

预期会看到的（能解释就行，不用消掉）：

| 项 | 什么时候会是 FAIL/WARN | 怎么处理 |
| --- | --- | --- |
| `hardware` | 串口没插 / 没直通 | 插上 USB；WSL 里 `usbipd attach --wsl --busid <busid>` |
| `follow_tools` | 找不到 `arm_console.py` | `export LITEARM_TOOLS=...`（见 1.2） |
| `teleop_python` | 没有 conda 环境 | `export TELEOP_SERVER_PY=...`，或先跑上游 `environments/setup.sh` |
| `display` | 系统服务里没有 DISPLAY | 用 `install.sh --user`，或 `TELEOP_VIEWER=rviz` |
| `submodules` | 没 init / 被就地改过 | `git submodule update --init`；改上游请去它自己的仓库 |
| `env:teleop` / `flags:follow_server` | 上游换了变量名/参数名 | 上游被改过，核对 submodule 里那份 |

### 档 1 · 纯仿真：只验帧路，不碰臂（**不插 USB 也能做**）

```bash
TELEOP_ARM_HOST= ./scripts/up.sh          # 显式留空 = 不传 `--real`，up.sh 会自动拉上游
```

没有头显就用假头显推帧：

```bash
python3 scripts/fake_headset.py --status
```

**过了是什么样**：

- 假头显那边每秒一行状态：`fps≈90`、`帧龄` 个位数毫秒、`转发` 在涨、`耗时 <1ms`、`teleop=已连`；
- teleop 那一侧，pybullet 窗口里的模型跟着手柄动（按 grip 后才动）；
- `链路=following`、`臂=—`（还没接真机）。

没头显又想装一个真头显：`litearm-vr` 里 `./build.sh quest install`（走下 adb），
地址在头显的 3D 面板里手填 —— 填桥打印的那个 `ws://<LAN IP>:8765`。

### 档 2 · 只连硬件、不下发：臂使能但不跟随

先脱开桥，单测串口：

```bash
./submodules/litearm-vr-follow/run_stm32_comm_test.sh /dev/ttyACM0
```

再让桥把跟随服务拉起来（**不接真机**，teleop 留在仿真档）：

```bash
TELEOP_ARM_HOST= ./scripts/up.sh --link    # 拉 follow_server；teleop 仍是仿真，不下发
```

⚠️ 这一档 `teleop.start` 会给跟随服务发 `start` —— **电机就通电持位了**。
工作区检查（第 1.3 节）在这一档同样要做，别以为"还没接真机"就可以随便。

**过了是什么样**：`--ctl status` 的 `机械臂` 行有 `state` 和 `q`（桥 4Hz `ping` 到了），
`arm.connected=True`。

### 档 3 · 真机跟随

```bash
./scripts/up.sh --link        # 会先拦你确认"工作区无人、急停在手"（非交互要 --yes）
```

**第一次做**：手不要离开急停，先在**小幅度**下走一遍 —— 假头显的默认半径只有 3cm。

**过了是什么样**：按 grip → 臂先 `movej` 回 `q_home`，然后跟着手柄小幅移动；
松 grip → 暂停；再按 grip → 从当前位置重新锚定（不会跳）。

### 档 4 · 兜底验收（M2 的验收项，别跳）

让帧停掉 —— 假头显一条命令就能复现"头显卡住"：

```bash
python3 scripts/fake_headset.py --drop-after 5 --status
```

**过了是什么样**（时间点看桥的日志）：

| 时刻 | 应该发生 |
| --- | --- |
| 200ms（`FRAME_WARN_MS`） | `[frame_stall]` 告警 |
| 500ms（`FRAME_FALLBACK_MS`） | `[fallback_hold]` —— 臂**就地保持**（不 movej） |
| 帧恢复后 | `[fallback_cleared]`，臂保持就地保持，**要重新按 grip** 才能继续 |

大号验证：直接拔头显电源 / 关 WiFi（= 断线，不是卡住），效果应当一样。

> 假头显在这台机器上实测过：`--drop-after` 之后 202ms 出 `frame_stall`、500ms 出
> `fallback_hold`，与配置一致（当时的 `hold` 失败是因为那台机器没接 follow_server）。

## 3. 卡住时的对照表

| 症状 | 先看 |
| --- | --- |
| 头显/假头显连不上 | `--doctor` 的 `lan_addr`（IP 对不对）、`port_bridge`（被占没有） |
| 有 fps 但臂不动 | `link.forward_enabled`（转发开关）→ `bridge.teleop_connected` → `arm.state`（认没认 `start`）→ 假头显打印的每帧增量有没有越过去上游死区 |
| 帧通、teleop 也连上，但仿真里模型不动 | 假头显的 grip 按了没（`--warmup` 后才按）；`headset.grip` 是不是 true |
| 上游起不来 | `PROCESS_LOG_DIR/<name>.log` —— **上游脚本自己的报错原文在里面** |
| 真机不动但 `arm.state=following` | `--mode` 选错（见 1.1）；或 `--kp/--kd`（`jf`/`mit` 共用）没调 |
| 一直不出兜底告警 | `fallback.guard_enabled`、`FALLBACK_ENABLED` |

## 4. 出事怎么办

- **别直接 `pkill` 跟随服务**：它一停电机就没人持位了。停要走桥：
  `./scripts/up.sh --stop`（先 latch 再收进程）。
- 急停：状态机的 `estop`（只发 `stop`，永远不会 `movej`）。
- 退回仿真：`TELEOP_ARM_HOST=` 留空，`./scripts/up.sh --stop` 之后重来。
- 桥**不自动重启**上游进程（它们会驱动机械臂，崩了只报警）—— 见 README 的实现取舍 2。

## 5. 现在还没有的东西（会限制这次调试）

- **上位机没有 VR 页**（`litearm-studio` 里没有 VR 路由），M4 头显回显也没做 ——
  所以这次调试**没有图形界面**，只能靠 `--ctl status`、假头显的 `--status` 行、
  pybullet 窗口和日志。真机调试里"操作员视角"的那部分要等 VR 页。
- 假头显只验**平移**，姿态一律单位四元数（腕部姿态没覆盖）。
- 头显连不上时，整条链的"人在环里"这一段（握把语义、扳机控手/夹爪）验不了。

# litearm-vr-bridge

LiteArm 上位机（`litearm-studio`）在 **VR 遥操场景**下使用的常驻服务。

一句话：**站在头显和 teleop 中间的那个垫片** —— 收头显的手柄帧、发到 zenoh 上给 teleop
做逆解，同时把链路状态汇总给上位机、并接受它的动作，还负责掉线兜底与进程启停。

```
                    ┌──────────── litearm-vr-bridge ────────────┐
头显 ──ws 8765────▶ │ ① 原始帧 ──▶ zenoh litearm/v4/{arm}/vr_frame│ ──▶ teleop（订阅）
                    │                        │                  │ ──TCP 9001──▶ follow ──▶ 真机
                    │ ② 状态     ──▶ .../vr_status（上位机订阅）  │ ◀── ping / start / latch
                    │ ③ 动作     ◀── .../vr_rpc（上位机 query）   │
                    │ ④ 掉线兜底 + 进程启停                      │
                    └───────────────────────────────────────────┘
```

**对外的两条边都在 zenoh 上**（帧→teleop、状态/动作↔上位机）。8765 只服务头显
（Godot 只会 WebSocket），桥↔跟随服务仍是 TCP 9001。

上位机不在这张图上直连：它经一台 `zenoh-bridge-remote-api` 进 zenoh 网络（浏览器
只能走 WebSocket，见 `docs/DEPLOY.md`）。**`ctl` 不需要那台 router** —— 它跑在桥同机，
直连桥的 zenoh 监听端点，见下面「快捷启动」。

## 状态

**M1 / M2 已实现并有测试覆盖；M3 代码完成、待现场联调。**
完整设计见 [docs/DESIGN.md](docs/DESIGN.md)，部署见 [docs/DEPLOY.md](docs/DEPLOY.md)，
**第一次接真机看 [docs/REAL-ARM-BRINGUP.md](docs/REAL-ARM-BRINGUP.md)**（先定机器、控制律与逐步推进）。

| 里程碑 | 状态 |
| --- | --- |
| M1 转发 + 状态（8765 收帧 → zenoh `vr_frame` 逐帧转发、`vr_status` 摘要、`forward_cost_ms`） | 完成 |
| M2 动作 + 兜底（`teleop.start/stop`、`estop`、200/500ms 告警与自动 hold） | 完成 |
| M3 进程启停 + systemd（`process.start/stop`、`deploy/`、`scripts/`） | 代码完成，待现场联调 |
| M4 头显回显 | 未开始（依赖 `litearm-vr` 接上 `message_received`） |

## 快捷启动

```bash
sudo ./scripts/install.sh     # 一次性：建 venv、装 systemd、生成 /etc/litearm-vr-bridge.env
                              # （图形会话 / 无 sudo 的场景用 ./scripts/install.sh --user）

./scripts/up.sh               # 之后日常就这一条
./scripts/up.sh --link        # 接真机链路（见下面的"真机分档"）
./scripts/up.sh --status      # 看一眼链路现状
./scripts/up.sh --stop        # 停（它会先 latch 就地持位，再收掉子进程）
```

`up.sh` 依次做：加载 env → 预检（`--doctor`）→ 起桥 → 按规则拉链路 → 打印头显该填的地址。

**双臂真机**时桥管不了两个 follow 端点（它只持有 `FOLLOW_SERVER_ENDPOINT` 一个），
`scripts/start.sh` 把"起两个 follow server + `up.sh --link`"合成一条：

```bash
./scripts/start.sh          # 起 left/right follow → 桥 + teleop；--stop 反着停；--status 看现状
```

默认 `left=/dev/ttyACM0:9001`、`right=/dev/ttyACM1:9002`，用 `FOLLOW_{LEFT,RIGHT}_{SERIAL,PORT}`
覆盖；follow 仓默认找同级 `../litearm-vr-follow`（或 `FOLLOW_DIR=`）。

**真机分档。** teleop 接真机（桥给 launcher 传 `--real`）时，`up.sh` **不会**自动拉起上游；
那一步要人明确敲 `--link`（非交互环境还要 `--yes`，且会先要你确认工作区无人、急停在手）。
仿真（`TELEOP_ARM_HOST` 显式留空 = 不传 `--real`）才一键到底。

这么设计是因为：桥的默认姿态是"垫片不改变原有链路行为"，而它**不做自动重启**的理由
（见下面的取舍 2）同样适用于自动拉起 —— 一条命令把"监控垫片"和"真机控制栈上电"
耦合成一个动作，不合适。想让桥完全不碰上游进程，就把 `TELEOP_CMD=` / `FOLLOW_SERVER_CMD=`
显式留空（`config.py` 的语义是"这个进程我不要"），那 `teleop.start` 就只会打开转发。

### 启动前体检

```bash
python3 -m litearm_vr_bridge --doctor     # 有 fail 时退出码 1；up.sh 会自动跑它
```

把现场那些"起来了但不知道为什么不动"的问题提前问清楚：端口被占、submodule 没 init、
上游被就地改过、串口/CAN 不在、conda 里缺 pybullet、`arm_console.py` 找不到、
没有 DISPLAY 所以 pybullet 窗口起不来……

还有**两种对拍**：命令行还是本服务拼的（follow，参数透传到上游脚本），就拿将要传的参数
和脚本里声明的 `add_argument` 逐一比对；参数变成环境变量的（teleop，由上游启动脚本读），
就比对上游脚本读不读我们设的那几个 `$VAR`。上游被 submodule pin 住之后，这条防的不再是
"现场那份上游比本服务假设的老"，而是"上游换了接口"和"有人把 submodule 就地改了"。

### 扳机

桥的受管进程**只在收到 `teleop.start` 时才拉起**，而上位机的 VR 页已按 AGENTS.md 删除 ——
现场没有任何东西能按这个按钮。`--ctl` 补的就是这个（`up.sh` 用的也是它）：

```bash
python3 -m litearm_vr_bridge --ctl up       # teleop.start：开转发 + 拉起 follow_server/teleop
python3 -m litearm_vr_bridge --ctl status   # 打印一帧快照的判读字段
python3 -m litearm_vr_bridge --ctl down     # teleop.stop（latch）→ process.stop 逐个收掉
```

## 上游从哪来

本服务要起停的两个上游，是仓库里的两个 **submodule**，pin 在具体 commit 上：

```
submodules/litearm-vr-teleop   teleop（原 metavr）：手柄位姿 → 7 关节角（IK + 3D 预览）
submodules/litearm-vr-follow   7 关节角 → 电机（真机跟随服务）
```

```bash
git clone --recursive git@gitee.com:yudao_hz_1/litearm-vr-bridge.git
git submodule update --init          # 已经 clone 过、但没带 --recursive 的话补这一条
```

路径从仓库布局推出来（`config.py` 的 `repo_root()`），默认不用配；上游确实放在 submodule
之外时才用 `TELEOP_REPO` / `FOLLOW_REPO` 指到各仓根。
上游**怎么起**也交给上游自己的启动脚本（teleop 的 `launch/run_teleop_webxr.py`、follow 的
`run_stm32_server.sh`）—— conda 环境、`arm_console`、ROS2、灵巧手 SDK 的路径由它们自己解析，
本服务不复制一份。要覆盖这些，就 export 它们认的变量（`TELEOP_SERVER_PY` / `LITEARM_TOOLS` 等），
子进程原样继承。

升级上游 = 显式地挪 pin：

```bash
git -C submodules/litearm-vr-teleop pull
git add submodules/litearm-vr-teleop && git commit -m "chore(submodule): teleop 跟到 <sha>"
```

`--doctor` 会把两个 submodule 的 commit 和"有没有被就地改过"一并报出来；就地改的改动
下次 `submodule update` 就没了，要改请去它自己的仓库。

## 手动跑（开发 / 排障）

```bash
python3 -m pip install --user "websockets>=13"   # 13 起才有 websockets.asyncio.*
cd litearm-vr-bridge
git submodule update --init                      # 上游两个仓库

PYTHONPATH=src python3 -m litearm_vr_bridge --print-config   # 看生效配置
PYTHONPATH=src python3 -m litearm_vr_bridge --doctor         # 先体检
PYTHONPATH=src python3 -m litearm_vr_bridge                  # 前台起服务

PYTHONPATH=src python3 -m unittest discover -s tests -t .    # 跑测试（126 项）
```

最小配置（全部环境变量见 DESIGN 第 7 节）：

```bash
BRIDGE_PORT=8765                     # 只有头显连这里
ZENOH_ARM_ID=armA                    # 话题段；要与 teleop、上位机一致
ZENOH_LISTEN_ENDPOINTS=tcp/0.0.0.0:7447   # teleop 与上位机 router 连进来
TELEOP_ENDPOINT=127.0.0.1:8766       # 上游 teleop 改成 --port 8766，不用改代码
FOLLOW_SERVER_ENDPOINT=127.0.0.1:9001
```

`/healthz` 是给巡检用的只读 HTTP 接口（挂在头显那个口上）。**zenoh 挂了的时候它是
唯一的带外通道**，所以 `up.sh` 的判活走它、而不走 `--ctl status`（后者要连 zenoh）。
上位机的订阅口是 `litearm/v4/{arm}/vr_status`。

## 与 DESIGN 的几处实现取舍

1. **`events` 不是"推完即清"，而是按 TTL（默认 20s）+ 上限保留。**
   上游 `litearm-studio` 对状态流是覆盖式取用（`snapshotValue = JSON.parse(...)`），
   没有消费确认通道；"推一次就清"会让偶发告警被吞掉。可用动作 `event.clear` 显式清空。
2. **受管子进程不做自动重启。** teleop / 跟随服务都会驱动机械臂，崩溃后自动拉起等于
   可能无人值守地重新上电或回零（teleop 启动时上游会自动 `home`）。这里只如实上报
   `running/stopped/crashed`；常驻与崩溃重启交给 systemd 管本服务自己。
3. **3D 预览提频用 `STATUS_HZ`，不开第二条消息通道。**
   上位机对状态流是"每条消息都当整份快照覆盖"，另发一种 `arm.q` 消息会被当成残缺快照；
   快照本身只有几百字节，把 `STATUS_HZ` 提到 20~30 即可满足预览平滑度。
4. **`teleop.start` 只在"冷启动"时替操作员放开跟随服务。**
   刚从 `teleop.stop` / `estop` / 兜底出来时，teleop 那边仍握着上一次的锚点（它没看见
   grip 松开），此时若把跟随服务放开，teleop 的 q 帧会立刻被接受并按旧锚点伺服 —— 手柄
   要是移动过就是一次跳跃。所以这种情况只打开转发，并回一条
   `teleop_start_needs_regrip` 事件，等操作员重新按 grip 让 teleop 重新锚定后再随。
   冷启动（还没进入过上面这些状态）则照常发 `start`：此时跟随服务 q_target 为空、只是
   原地持位，操作员按 grip 才真正动。

另外，快照比 DESIGN 4.1 多带了一些排障字段（`bridge.teleop_connected`、
`headset.frames_forwarded/frames_dropped`、`link.state`、`processes_detail` 等），
上位机按可选字段处理即可（`litearm-studio` 的类型定义本来就把它们当可选）。

## 它不是什么

- 不做逆解（在 teleop 里）
- 不驱动机械臂（在 `arm_follow_server` / `litearm-server` 里）
- 不是界面（在 `litearm-studio` 里）
- 不向前端推送 90Hz 原始手柄帧（前端只要状态摘要）

## 相关项目

| 项目 | 关系 |
| --- | --- |
| `litearm-studio` | 上位机前端，本服务的唯一界面消费者 |
| `litearm-vr-teleop` | 上游（submodule）：逆解中继，本服务把帧转发给它（不改其代码） |
| `litearm-vr-follow` | 下游（submodule）：真机跟随服务，本服务发 `start/hold/latch/stop`、轮询 `ping` |
| `litearm-vr`（Godot） | 头显端，本服务的上游数据源 |
| `litearm-server` | 机械臂控制器守护进程（ws 7449），平时由上位机直连 |

> 状态快照里的进程名仍是 `teleop` / `follow_server`（跟 `TELEOP_*` 配置项、上位机的字段
> 对齐），`teleop` 指的就是 `litearm-vr-teleop` 这个组件。

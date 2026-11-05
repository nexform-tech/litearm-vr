# 部署与运行

> 配置项定义见 [DESIGN.md](DESIGN.md) 第 7 节；本文只讲"怎么把它跑起来、怎么确认它是对的"。

## 1. 依赖与安装

本服务只需要 Python 3.10+ 和 `websockets>=13`（`websockets.asyncio.*` 与
`websockets.http11` 是 13 起才有的 API，装 12 会 import 失败）。

**上游两个仓库是 submodule，clone 时要带上**（没有它们，桥起来了也没东西可托管）：

```bash
git clone --recursive git@gitee.com:yudao_hz_1/litearm-vr-bridge.git
git submodule update --init          # 已经 clone 过、但没带 --recursive 的话补这一条
```

```bash
# 方式 A：一条命令装好（拉 submodule + 建 venv + 写 env + 装 systemd + 开机自启）
sudo ./scripts/install.sh
./scripts/install.sh --user          # 图形会话 / 没有 sudo 时改装用户级服务

# 方式 B：直接跑源码（开发、排障）
cd /opt/litearm/litearm-vr-bridge
python3 -m pip install --user "websockets>=13"
PYTHONPATH=src python3 -m litearm_vr_bridge --print-config   # 先看生效配置
PYTHONPATH=src python3 -m litearm_vr_bridge --doctor         # 再体检
PYTHONPATH=src python3 -m litearm_vr_bridge                  # 正式启动

# 方式 C：装成包（得到 litearm-vr-bridge 命令）
python3 -m pip install --user .
litearm-vr-bridge
```

跑测试（stdlib unittest，不需要 pytest）：

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -t .    # 126 项
```

## 2. 上游怎么起：交给它们自己的启动脚本

两个上游都是 submodule，桥**不自己拼它们的命令行**，而是调它们自己的启动脚本：

```
submodules/litearm-vr-teleop/teleop/launch/run_teleop_webxr.py  ← 桥把 --host/--port/--viewer/--real… 拼给它
submodules/litearm-vr-follow/run_stm32_server.sh                ← 桥把 --host/--tcp-port/--mode/--port 透传
```

这样 conda 环境（`TELEOP_SERVER_PY` → 已激活的 metavr-arm → `conda info --base` → 常见根）、
`arm_console.py`（`LITEARM_TOOLS`）、ROS2、灵巧手/夹爪 SDK 的解析只有上游那一份 —— 现场
"手工起得来、服务起不来"这类故障因此少掉一大类。要覆盖这些路径，就在 env 文件里
export 上游认的变量，受管子进程原样继承。

手工跑（排障时）与桥起的是同一条路径：

```bash
# 以前：头显直连 teleop 的 8765
submodules/litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --port 8765

# 现在：头显连桥的 8765，桥逐帧转发给 teleop 的 8766
submodules/litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --port 8766
```

**上游不需要改代码**（`--port` 是它自己的开关，launcher 也认）。头显 3D 面板里填的地址
仍然是 `ws://<本机IP>:8765`，不用动。

> 第一次接到真机械臂上，按 [REAL-ARM-BRINGUP.md](REAL-ARM-BRINGUP.md) 走：
> 先定机器（谁的 USB 口、conda 环境在哪、头显够不够得着），再一档一档往上推。

### 托管到 systemd 下的几个注意

上游脚本是给"有人坐在机器前面"写的，托管起来有三处要心里有数（都不致命，但现场会撞上）：

- **`run_teleop_webxr.sh` 启动时会 `pkill -f ws_echo_server`**：它会先清掉同名进程。
  桥这边通常是收掉自己上次拉起的那个；但你要是同时手工开了一个在排障，会被一起清掉。
- **灵巧手 / 夹爪的 CAN 是上游脚本 `sudo ip link set ...` 配的**：systemd 服务里不一定有
  `sudo`（也不该有）。启用 `TELEOP_HAND` / `TELEOP_GRIPPER` 前，先把接口配好：
  `sudo ip link set can0 up type can bitrate 1000000`（已经 UP 的它不会重复配）。
- **ROS2 发布由上游决定**：源到 `/opt/ros/humble/setup.bash` 且 `rclpy` 可用时，teleop 会
  发 `/joint_states`（没有就自己警告跳过）。桥这边**没有**关掉它的旋钮了 —— 需要的话得在
  上游脚本上加一个，这属于上游的事。

## 3. systemd 常驻

一条命令（会建 venv、把单元里的 `/opt` 路径换成实际路径、生成 env 文件）：

```bash
sudo ./scripts/install.sh                    # 系统服务
./scripts/install.sh --user                  # 用户级服务（systemctl --user）
```

手工装的话：

```bash
sudo cp deploy/litearm-vr-bridge.service /etc/systemd/system/
sudo cp deploy/bridge.env.example /etc/litearm-vr-bridge.env
sudoedit /etc/litearm-vr-bridge.env          # 至少确认 FOLLOW_SERVER_MODE 与串口/CAN 口
                                             # （上游不在 submodule 里才需要 TELEOP_REPO/FOLLOW_REPO）
sudo systemctl daemon-reload
sudo systemctl enable --now litearm-vr-bridge
systemctl status litearm-vr-bridge
journalctl -u litearm-vr-bridge -f
```

单元文件里 `KillMode=mixed` 是刻意的：停止服务时先让桥自己走完
"若在跟随 → 先 latch 就地持位 → 再收掉 teleop / 跟随服务"这段流程。

**系统服务 vs 用户服务。** teleop 的 pybullet 窗口（`TELEOP_VIEWER=gui`）要在图形会话里
才打得开，而系统服务默认没有 `DISPLAY`。`install.sh` 会把当前会话的 `DISPLAY`/`XAUTHORITY`
写进 drop-in；手工装就自己加，或者干脆用 `--user`（用户级服务天然继承图形会话，
再补一句 `loginctl enable-linger $USER` 就能开机自启）。

**env 文件的格式**：`KEY=VALUE`，一行一个，注释要**单独占一行**（不支持行尾注释），
值里有空格可加引号。这不是随便定的 —— systemd 的 `EnvironmentFile=` 就是这么读的，
`up.sh` 用的是同一套语义，两边必须一致。别写 `$(...)`：那不是 shell。

⚠ 早期版本的 `bridge.env.example` 带行尾注释，配上 `EnvironmentFile=` 会把注释也当成值，
`TELEOP_SIDE` 这类选项会被**静默忽略**（解析不出就退回默认值）。已修，但现场要是从旧文件
拷过来的，记得核一遍。

## 4. 冒烟检查（现场几步）

```bash
# ⓪ 启动前体检：端口 / 上游脚本 / 参数对拍 / 串口或 CAN / 依赖 / 图形会话
./scripts/up.sh --doctor

# ① 桥活着吗（HTTP，不是 WebSocket）
curl -s http://127.0.0.1:8765/healthz | python3 -m json.tool

# ①' 或者让桥自己讲：链路状态、头显、机械臂、进程、兜底、最近的告警
./scripts/up.sh --status

# ② 上位机视角的状态流（zenoh；client 直连桥，不需要 router）
python3 - <<'PY'
import json, zenoh
cfg = zenoh.Config()
cfg.insert_json5("mode", '"client"')
cfg.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
cfg.insert_json5("scouting/multicast/enabled", "false")
cfg.insert_json5("scouting/gossip/enabled", "false")
seen = []
session = zenoh.open(cfg)
session.declare_subscriber("litearm/v4/armA/vr_status",
    lambda s: seen.append(json.loads(bytes(s.payload))))
import time; time.sleep(0.5)
for snap in seen[-3:]:
    print(snap["owner"], snap["link"]["state"], snap["headset"]["fps"],
          snap["headset"]["latency_ms"], snap["bridge"]["forward_cost_ms"])
session.close()
PY

# ③ 头显连上后，这里应看到 fps≈90、last_frame_age_ms 个位数
```

判读要点：

| 现象 | 含义 |
| --- | --- |
| `bridge.forward_cost_ms` > 1ms | 本服务自己卡了（CPU/日志/被抢） |
| `headset.latency_ms` 变大而 `forward_cost_ms` 正常 | 慢在头显或无线段 |
| `headset.online=false` 且 `last_frame_age_ms` 持续涨 | 头显掉线/断连 |
| `fallback.active=true` | 已超过 500ms 没帧，机械臂被 hold 就地保持 |
| `link.forward_enabled=false` | 有人按了"停止遥操"（头显还在推，但一个字都不转发） |

## 5. 冷启动到可遥操（M3）

```bash
./scripts/up.sh            # 起桥 → 预检 → 打印头显该填的地址
./scripts/up.sh --link     # 接真机：发 teleop.start，按配置拉起上游
```

两个上游默认从 submodule 里取，所以不用配路径；放在别处时给 `TELEOP_REPO` /
`FOLLOW_REPO`（或两个 `*_CMD`）。

`--link` 之后桥会依次（`bridge.handle_action` 的 `teleop.start` 分支）：

1. 拉起 `follow_server`（9001），**并等它的端口真的能连上**再往下走
   （真机上它要等 STM32 状态帧，要几秒；这一步顺序错了 `start` 会打在没监听的端口上丢掉）；
2. 拉起 `teleop`（8766）—— 接真机（`--real`）时会先回一条黄色警告事件；
   teleop 启动**不会自动 home**，首次按 grip 才 movej 回 q_home 再跟随；
3. 打开帧转发，并向跟随服务下发 `start`；
4. 操作员戴上头显、按右手 grip 开始跟随。

> `up.sh` 默认只在**仿真**时自动做这一步；生成的 teleop 命令带 `--real` 时会停下来，
> 让你确认工作区无人、急停在手之后再敲 `--link`。想让桥完全不托管上游进程，把
> `TELEOP_CMD=` / `FOLLOW_SERVER_CMD=` 显式留空即可。

> 不想让 teleop 碰真机：把 `TELEOP_ARM_HOST=` 显式留空，它就只跑 pybullet 仿真。

**再次进入遥操**（`--ctl down`、`teleop.stop` 或急停之后）：再发 `teleop.start`
（`./scripts/up.sh --link` 或 `--ctl up`）只会打开帧转发，状态流里会回一条
`teleop_start_needs_regrip`；teleop 还握着上一次的锚点，必须由操作员**重新按一次
grip**（松 → 按）才会重新锚定并继续跟随。头显掉线兜底后的恢复同理。这样设计是为了避免
"按旧锚点突然追上手柄"的跳跃。

## 6. 排障

先跑 `./scripts/up.sh --doctor`：端口、submodule、两种对拍、串口/CAN、依赖、DISPLAY 都能一次问清。

- **头显连不上**：确认 `BRIDGE_PORT` 没被占用、防火墙放行、头显面板地址是 `<本机IP>:8765`。
- **上游起不来**：看 `processes_detail` 里的 `exit_code` 和 `PROCESS_LOG_DIR/<name>.log` ——
  上游启动脚本自己的报错（"未找到 arm_console.py"、"找不到 metavr-arm 环境的 python"）
  就落在那个日志里，照着修即可。
- **`--doctor` 报 `env:teleop` / `flags:follow_server`**：我们传给上游的参数（或设的环境变量）
  它不认了 —— 多半是 submodule 被就地改过，或上游换了接口；预检会点名是哪几个。
- **`--link` 报"跟随服务没接受 start"**：follow_server 起来了但没在听 9001（崩了、
  串口没接、或还在等 STM32 状态帧超过 8 秒）。它的日志在 `PROCESS_LOG_DIR/follow_server.log`。
- **状态页有 fps 但机械臂不动**：看 `link.forward_enabled`（是不是被停了）、
  `bridge.teleop_connected`（转发目标在不在）、`arm.state`（跟随服务认没认 `start`）。
- **一直不出兜底告警**：检查 `FALLBACK_ENABLED`；快照里 `fallback.guard_enabled` 会如实反映。
- **`process.*` 返回"未配置"**：两条路都没有 —— 既没 init submodule（`git submodule
  update --init`）也没给 `TELEOP_REPO` / `FOLLOW_REPO`；或者这个进程的 `*_CMD` 被显式留空
  （见 env 示例第 4 节注释）。
- **跟随服务一直 ping 失败**：`arm.error` 里有原因（连接被拒 / 超时 / 回包非法）。
- **日志太吵**：`LOG_LEVEL=WARNING`；跟随服务不可用时的失败日志本身已限流到 5s 一条。

## 7. 已知边界

- 桥**不做逆解、不驱动机械臂**；兜底只会发 `hold` / `stop`，永远不会 `movej` / `home`。
- 同一时刻只转发**一路**头显连接（后到的顶替先到的，日志里会写明）。
- 受管进程**不自动重启**：它们会驱动机械臂，崩了只报警，由人决定是否重启
  （常驻与崩溃重启是 systemd 对本服务自己的事）。
- 局域网内无鉴权（DESIGN 8.4）。9001 本来就是"谁连上谁能控臂"，别暴露到不可信网络。

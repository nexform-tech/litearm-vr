# LiteArm VR 上位机操作说明

[English](README.md) | [简体中文](README.zh-CN.md)

适用：本机 VR 遥操环境，当前机型为 **正装单臂 + 夹爪**（`arms = "single"`、`side = false`、`grip = "can0@127.0.0.1:9003"`）。

> 本文所有用户路径都写成环境变量或相对路径，不绑定用户名：
> `$HOME`、`${XDG_CONFIG_HOME:-$HOME/.config}`、`${XDG_STATE_HOME:-$HOME/.local/state}`。
> 仓库内部一律用相对路径。

## 下载与安装

本版本的 Debian 安装包发布在 [Releases](https://github.com/nexform-tech/litearm-vr/releases) 页面：

```bash
sudo apt install ./litearm-vr-bundle_0.1.0_amd64.deb
```

该包安装桥、上游 teleop/follow、冻结版 teleop 可执行文件、各 SDK、`litearm-can` DKMS 驱动（需要 `dkms`、`build-essential` 与匹配的内核头文件）、udev 规则和 systemd 单元；`postinst` 会配置 DKMS 模块，并为安装用户把桥装成用户级 systemd 服务。

安装后体检：

```bash
/opt/litearm/litearm-vr-bridge/scripts/up.sh --doctor
```

---

## 0. 先设置环境变量（每个新终端执行一次）

```bash
export STUDIO_DIR="${HOME}/vr/litearm-vr-bridge/studio"      # 上位机源码/构建目录（随桥仓库）
export BRIDGE_DIR="${HOME}/vr/litearm-vr-bridge"              # 桥的源码检出（含预设脚本）
export BRIDGE_INSTALL="${LITEARM_BRIDGE_INSTALL:-/opt/litearm/litearm-vr-bridge}"  # 已安装的桥
export BRIDGE_CONFIG_DIR="${XDG_CONFIG_HOME:-${HOME}/.config}/litearm-vr-bridge"
export BRIDGE_STATE_DIR="${XDG_STATE_HOME:-${HOME}/.local/state}/litearm-vr-bridge"
export VR_STUDIO="${HOME}/.local/bin/litearm-vr-only"          # 启动器
```

可以放进 `~/.bashrc`，以后开终端不用再设。

---

## 1. 启动上位机

### 图形界面

在应用菜单里搜索 **“LiteArm VR”** 并打开。

### 终端

```bash
"${VR_STUDIO}"
```

启动器内部会先确保桥服务在运行，然后用本软件的前端打开窗口。

> 源码 daemon 需要 `pywebview`；缺失时启动器会明确提示：
> `python3 -m pip install --user pywebview`。

### 仿真试跑（不碰真机）

```bash
"${VR_STUDIO}" --fake
```

> ⚠ **不要用原来的 “LiteArm Studio” 图标做 VR 遥操。**
> 那个是完整版，daemon 会自动连接并占用 `/dev/ttyACM0`；VR 遥操时这个串口必须留给桥的 follow 服务。
> 本启动器用 `--port /dev/ttyDOES_NOT_EXIST` 正是为了让 daemon 不抢串口。
> VR 页的状态、关节角和电机负载都来自桥的 zenoh 通道，不需要 daemon 自己连臂。

---

## 2. 连接头显并开始遥操

头显 WebSocket 地址（自动取本机 IP）：

```bash
echo "ws://$(hostname -I | awk '{print $1}'):8765"
```

例如输出 `ws://192.168.31.6:8765`，把它填进头显面板。

操作顺序：

1. 打开上位机，确认右列 **「臂映射」**：
   - 臂：`armA`
   - 夹爪：`can0`
   - 串口：留空 = 自动识别；也可以手动选择 `/dev/serial/by-id/...`
2. 头显上线后，点 **「▶ 开始遥操」**。
3. 桥会自动拉起：
   - follow 服务（占用机械臂串口）
   - 夹爪服务（`can0:9003`）
4. 头显手柄：
   - **grip 按下 = 开始跟随，松开 = 暂停**
   - **trigger 按下 = 夹爪闭合（握拳），松开 = 夹爪张开**

> 首次按 grip 会先 `movej` 回零位，再开始跟随。这是机械臂自己的动作，不是手柄实时控制的。
> 确认机械臂已上电、工作区无人、急停在手后再按。

如果之前 teleop 意外退出（页面显示“兜底持位”）：**再点一次「开始遥操」** 即可重新拉起。

---

## 3. 开夹爪 / 关夹爪

夹爪不需要单独启动，**「开始遥操」时会随 follow 一起拉起**，桥会确认它真的使能。

| 操作 | 手柄 |
| --- | --- |
| 夹紧 | **press trigger**（按下扳机） |
| 张开 | **release trigger**（松开扳机） |

注意：

- 本页面**没有手动开合夹爪的按钮**（手动控制页已按需求删除），夹爪只由头显扳机控制。
- 页面左列「电机负载」里的 **夹爪** 一行显示当前读数；右侧「臂映射」显示它用的 CAN 口。
- 夹爪不动时检查：
  ```bash
  ip -br link | grep can0
  tail -40 "${BRIDGE_STATE_DIR}/logs/gripper_server_armA.log"
  ```
  `can0` 不在、或日志里写未使能，就先查 CAN 盒/线。

---

## 4. 修改配置文件

配置文件：

```bash
"${BRIDGE_CONFIG_DIR}/config.toml"
```

优先级从高到低：

1. `overrides.json`（上位机「链路配置」页面改的项）
2. `config.toml`
3. 代码里的出厂默认

### 4.1 用预设（改机型首选）

预设脚本在**源码检出**里（已安装的 0.1.0 包里没有这个脚本）：

```bash
cd "${BRIDGE_DIR}"
./scripts/preset.sh list
./scripts/preset.sh single-upright-grip     # 正装单臂 + 夹爪
```

脚本会：备份旧配置 → 校验 → 写入 `${BRIDGE_CONFIG_DIR}/config.toml` → 重启桥。
备份文件名形如 `config.toml.bak-YYYYmmdd-HHMMSS`。

### 4.2 手工编辑

```bash
nano "${BRIDGE_CONFIG_DIR}/config.toml"
systemctl --user restart litearm-vr-bridge
```

当前机型的最小配置：

```toml
[machine]
arms = "single"
side = false          # 正装
real = true

[arms.A]
grip = "can0@127.0.0.1:9003"
```

建议把串口固定下来（避免自动识别歧义）：

```bash
ls /dev/serial/by-id/
```

把看到的那条 STM32 路径写进去（TOML **不会**展开环境变量，必须写实际路径）：

```toml
[arms.A]
device = "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_XXXX-if01"
grip = "can0@127.0.0.1:9003"
```

改完重启：

```bash
systemctl --user restart litearm-vr-bridge
```

### 4.3 在上位机页面改

VR 页右上角 **「链路配置」** 可直接改：

- `machine.arms` / `machine.side` / `machine.real`
- `arms.A.device` / `arms.A.grip`
- `follow.mode` / `follow.ctrl_mode`
- `bridge.log_level` 等

页面改的项写入：

```bash
"${BRIDGE_CONFIG_DIR}/overrides.json"
```

它**优先于** `config.toml`。要让 `config.toml` 重新生效：

- 在对话框里点「恢复默认」，或
- ```bash
  rm -f "${BRIDGE_CONFIG_DIR}/overrides.json"
  systemctl --user restart litearm-vr-bridge
  ```

`machine.arms` 这类项需要重启桥，对话框会标出 `apply=restart`。

### 4.4 确认改对了

```bash
# 实际生效的配置（含 overrides）
PYTHONPATH="${BRIDGE_INSTALL}/src:${BRIDGE_INSTALL}/vendor" \
  python3 -m litearm_vr_bridge --print-config

# 体检：配置来源 / 串口 / CAN / 进程 / 参数
cd "${BRIDGE_INSTALL}" && ./scripts/up.sh --doctor
```

`--doctor` 里 `7447` / `8765` 显示“被占用”是因为桥自己正在监听，属正常。

---

## 5. 配置文件完整选项

### 5.1 文件位置与优先级

按顺序查找，先找到的算：

| 顺序 | 位置 |
| --- | --- |
| 1 | `--config PATH`（命令行显式指定） |
| 2 | `${XDG_CONFIG_HOME:-$HOME/.config}/litearm-vr-bridge/config.toml` |
| 3 | `/etc/litearm-vr-bridge.toml` |

三处都没有 = 全部使用出厂默认，桥仍可启动（合法状态）。

上位机「链路配置」页面改的项写入：

```bash
"${XDG_CONFIG_HOME:-${HOME}/.config}/litearm-vr-bridge/overrides.json"
```

优先级：**`overrides.json` > `config.toml` > 出厂默认**。

> ⚠ TOML 文件**不会展开** `$HOME` 之类的环境变量。本文里的 `${...}` 只用于 shell 命令；
> 写进 `config.toml` 的路径必须是实际路径（例如 `/dev/serial/by-id/...`）。

### 5.2 生效方式

| 生效方式 | 含义 |
| --- | --- |
| `live` | 立即生效 |
| `respec` | 重建受影响的受管进程命令行；链路忙时会拒绝，等空闲后再生效 |
| `restart` | 保存后需要重启桥：`systemctl --user restart litearm-vr-bridge` |
| 页面锁定 | 页面不能改，只能写进 `config.toml`；安全 / 网络 / 可执行程序类，避免远程改坏或失联 |

### 5.3 `[machine]` —— 这台机器是什么

| 键 | 类型 / 取值 | 默认 | 生效 | 页面可改 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `real` | bool | `true` | respec | 是 | `true` = 接真机；`false` = 只跑 pybullet 仿真，不碰机械臂 |
| `side` | bool | `false` | respec | 是 | 单臂安装方向：`false` = 正装，`true` = 侧装；仅单臂有效 |
| `viewer` | enum：`gui` / `rviz` | `gui` | respec | 是 | 上游 teleop 的查看器；`rviz` 需要 ROS2 |
| `hand` | bool | `false` | respec | 是 | 灵巧手开关 |
| `hand_remote` | text：`host:9004` | 空 | respec | 是 | 灵巧手服务地址（`hand = true` 时） |
| `gripper` | bool | `false` | respec | 是 | 上游级夹爪开关（通常与 `gripper_remote` 搭配） |
| `gripper_remote` | text：`host:9003` | 空 | respec | 是 | 夹爪服务地址（`gripper = true` 时） |
| `upright` | bool | `false` | respec | 是 | 双臂正装；仅 `arms = "dual"` 有效 |
| `arms` | enum：`single` / `dual` | `single` | restart | 是 | 机械臂构型；重启桥后生效 |
| `server_py` | text | 空 = 自动找 | 重启桥 | 否（锁定） | 上游 teleop 服务端解释器（metavr-arm 环境的 python） |

> 当前单臂夹爪使用 `[arms.A].grip = "can0@127.0.0.1:9003"`。
> `machine.gripper` / `gripper_remote` 是上游级开关，通常**不要**和 `[arms.A].grip` 重复配。

### 5.4 `[follow]` —— 跟随服务怎么跑

| 键 | 类型 / 取值 | 默认 | 生效 | 页面可改 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `mode` | enum：`stm32` / `sdk` | `stm32` | respec | 是 | `stm32` = 串口；`sdk` = CAN |
| `ctrl_mode` | text：`jf`／`js`／`j`／`mit` | `js` | respec | 是 | 对应上游 `--mode`；`jf` 需板上固件 ≥ 1.9.0 |
| `serial` | text | 空 | respec | 是 | 单臂硬件口的简写；**不能**与 `[arms.*].device` 同时写 |
| `dry_run` | bool | `false` | respec | 是 | 正常解算，但不下发到机械臂 |

### 5.5 `[arms.A]` / `[arms.B]` —— 每条臂的口

单臂只写 `[arms.A]`；双臂时 A = 左（左手柄）、B = 右（右手柄）。

| 键 | 类型 / 取值 | 默认 | 生效 | 页面可改 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `device` | text | 空 = 自动识别 | respec | 是 | 臂的硬件口；建议用 `/dev/serial/by-id/...`，`ttyACM*` 会漂 |
| `grip` | text：`can0@host:9003` | 空 = 无夹爪 | respec | 是 | 这条臂夹爪的 CAN 口@服务地址；本机地址时桥会一起管夹爪服务 |
| `host` | text | 空 = 本机 | respec | 是 | follow 服务所在主机（跨机部署时） |
| `port` | int | A=`9001`，B=`9002` | respec | 是 | follow 服务端口 |
| `id` | text | A=`armA`，B=`armB` | 重启桥 | 否（锁定） | 臂身份 / zenoh 话题中段；改了上位机会失联 |

当前机型示例：

```toml
[arms.A]
# device = "/dev/serial/by-id/usb-...-if01"   # 留空则自动识别；建议按实际 by-id 固定
grip = "can0@127.0.0.1:9003"
```

### 5.6 `[safety]` —— 真机确认闸门

| 键 | 类型 / 取值 | 默认 | 生效 | 页面可改 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `require_real_ack` | bool | `false` | 重启桥 | 否（锁定） | `true` 时 `teleop.start` 必须带 `real_ack: true`，否则拒绝；页面不能自行关闭 |

### 5.7 `[bridge]` —— 桥自身

| 键 | 类型 / 取值 | 默认 | 生效 | 页面可改 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `host` | text | `0.0.0.0` | 重启桥 | 否（锁定） | WebSocket 监听地址 |
| `port` | int | `8765` | 重启桥 | 否（锁定） | 头显连接的 WebSocket 端口 |
| `arm_id` | text | `armA` | 重启桥 | 否（锁定） | zenoh 话题中段；要与 teleop、上位机一致 |
| `zenoh_listen` | text | `tcp/127.0.0.1:7447` | 重启桥 | 否（锁定） | zenoh 监听端点；跨机上位机时改成 `tcp/0.0.0.0:7447` |
| `zenoh_connect` | text | 空 | 重启桥 | 否（锁定） | 主动连接的 zenoh 端点；同机部署留空 |
| `log_level` | enum：`DEBUG`／`INFO`／`WARNING`／`ERROR` | `INFO` | live | 是 | 日志级别，立即生效 |
| `log_format` | enum：`text` / `json` | `text` | restart | 是 | 日志格式 |
| `log_file` | text | 空 | restart | 是 | 额外写一份日志文件 |
| `manifest_file` | text | 空 = 与 `arms.json` 同目录 | restart | 是 | 臂映射记录文件 |

### 5.8 `[upstream]` —— 托管哪些上游进程

| 键 | 类型 / 取值 | 默认 | 生效 | 页面可改 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `manage` | bool 或列表 | `true` | restart | 是 | `true` / `false`，或点名列表如 `["teleop", "follow", "gripper", "status"]` |
| `teleop_repo` | text | 未设置 = 用仓库内 submodule | 重启桥 | 否（锁定） | 上游 teleop 路径 |
| `follow_repo` | text | 未设置 = 用仓库内 submodule | 重启桥 | 否（锁定） | 上游 follow 路径 |
| `teleop_frozen` | text | 未设置 = 包内有冻结产物就用 | 重启桥 | 否（锁定） | 冻结版 teleop 可执行文件；写了空字符串 = 强制不用 |

### 5.9 `[upstream.env]` —— 给受管子进程的环境变量

这一段是**唯一**会真的变成环境变量的地方，键名任意、值必须是字符串：

```toml
[upstream.env]
LITEARM_PY_SRC = "/opt/litearm/follow"     # follow 的 run_stm32_server.sh 读
LITEGRIP_SDK = "/opt/litearm/litegrip/src" # 夹爪服务读
CMEEL_SITE = "/opt/cmeel"                  # sdk 模式读
TELEOP_PYTHONPATH = "/opt/extra"           # 额外 import 路径
# GRIPPER_SERVER_CMD = "..."               # 自定义夹爪服务命令
```

> 这一段**页面锁定**，只能写进 `config.toml`，上位机「链路配置」不提供修改。
> 桥自己的键（`machine.*` / `follow.*` / `arms.*` / `bridge.*` / `upstream.*`）**不要**写进这里，
> 写错会当场报错并点名。

### 5.10 跨键规则

- `side` 只对 `arms = "single"` 有意义。
- `[follow] serial` 与 `[arms.*].device` 是同一件事的两种写法，**只能给一个**。
- 单臂只认 `[arms.A]`；双臂才写 `[arms.B]`。
- `machine.arms`、`upstream.manage`、`bridge.log_format` / `log_file` / `manifest_file`
  需要**重启桥**。
- 写错的段 / 键 / 取值会**当场报错并点名**，不会静默退回默认值。

---

## 6. 看状态 / 日志 / 重启

```bash
# 服务状态
systemctl --user status litearm-vr-bridge

# 链路摘要：头显帧率、臂设备、follow、兜底状态
cd "${BRIDGE_INSTALL}" && ./scripts/up.sh --status

# 实时日志
journalctl --user -u litearm-vr-bridge -f

# 各进程日志
ls "${BRIDGE_STATE_DIR}/logs/"
tail -f "${BRIDGE_STATE_DIR}/logs/teleop.log"
tail -f "${BRIDGE_STATE_DIR}/logs/follow_server_armA.log"
tail -f "${BRIDGE_STATE_DIR}/logs/gripper_server_armA.log"

# 重启桥
systemctl --user restart litearm-vr-bridge
```

---

## 7. 常见问题

| 现象 | 处理 |
| --- | --- |
| 「开始遥操」是灰的 | 头显没上线，或桥不在线；先看页面「VR 链路」与 `up.sh --status` |
| 头显连不上 `ws://<IP>:8765` | 确认头显与本机同网段；`ss -ltnp \| grep 8765`；检查本机 IP 是否变了 |
| 页面显示「跟随服务报错: Connection refused」 | 还没点「开始遥操」时属正常；开始后 follow 才会监听 9001 |
| teleop 意外退出 / 页面显示“兜底持位” | 再点「开始遥操」；反复退出看 `teleop.log`（常见：手动关掉了 teleop 的 pybullet 窗口） |
| 夹爪不动 | `ip -br link \| grep can0`；看 `gripper_server_armA.log`；确认配置里有 `grip = "can0@..."` |
| `ls /dev/serial/by-id/` 为空 | 臂没有插好/被拔掉，或 USB 线/供电问题；重新插拔后再看 |
| 串口被占用、follow 起不来 | `cd "${BRIDGE_INSTALL}" && ./scripts/up.sh --doctor`，它会指出占用者 |
| 改了 `config.toml` 却不生效 | 看 `overrides.json` 是否压过了它（见 4.3） |

---

## 8. 安全提醒

- 首次按 grip 会 **`movej` 回零位**，确认工作区无人、急停在手。
- 页面上的 **急停** 会向 follow 下发 stop。
- 真机模式（`real = true`）下，「开始遥操」会真的驱动机械臂；调试界面先用 `"${VR_STUDIO}" --fake`。
- 松开 grip 只是暂停跟随，不是断电；需要保持时让机械臂自己持位。

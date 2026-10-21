# LiteArm VR 上位机操作说明

[English](README.md) | [简体中文](README.zh-CN.md)

适用：本机 VR 遥操环境，当前机型为 **正装单臂 + 夹爪**（`arms = "single"`、`side = false`、`grip = "can0@127.0.0.1:9003"`）。

> **装完就能用，不需要设置任何环境变量。** 安装包把路径都定死了，本文一律写默认值；
> 只有把配置 / 状态整体挪到别处（共享账号、只读家目录）时才需要动 `XDG_CONFIG_HOME` / `XDG_STATE_HOME`。

| 用途 | 默认位置 |
| --- | --- |
| 桥（程序 + 脚本 + SDK） | `/opt/litearm/litearm-vr-bridge` |
| 桥的配置 | `~/.config/litearm-vr-bridge/config.toml`（没有这份文件 = 全用出厂默认） |
| 上位机改的覆盖层 | `~/.config/litearm-vr-bridge/overrides.json` |
| 臂映射记录（串口 / 夹爪 CAN） | `~/.config/litearm-vr-bridge/arms.json` |
| 受管进程日志 | `~/.local/state/litearm-vr-bridge/logs/` |
| 上位机 | 应用菜单 **LiteArm VR**，或命令 `litearm-studio` |

> 桥**不读**环境变量（唯一例外是 `config.toml` 里的 `[upstream.env]`，见 5.9）；
> 旧版那份 `/etc/litearm-vr-bridge.env` 已经不再读，留着的可以删掉。
> 上游启动脚本认的 `LITEARM_PY_SRC` / `LITEGRIP_SDK` 由安装脚本写进 systemd 单元，不用你 export。

---

## 下载与安装

安装：**桥的 deb**，外加**VR专用上位机** `litearm-studio` 的 deb。发布页见 [Releases](https://github.com/nexform-tech/litearm-vr/releases)。

```bash
sudo dpkg -i ./litearm-vr-bridge_0.1.2_amd64.deb 
sudo dpkg -i ./litearm-can-dkms_0.1.2_all.deb
```

- 需要联网装 `dkms` 与匹配的内核头文件（`linux-headers-$(uname -r)`）。**不需要 git，不需要拉代码。**
- Secure Boot 开着时系统会拒绝未签名的 CAN 驱动，需要关掉或给模块签名。
- `postinst` 会配置 DKMS 模块，并把桥装成**装它的那个用户**的用户级 systemd 服务、随即启动。
  认不出该装给谁（没人登录、也没在终端里 sudo）时它不猜，只打印那条命令：
  用你自己的账号跑一次 `/opt/litearm/litearm-vr-bridge/scripts/install.sh`。
- 升级：同样一条命令，桥会自动重启；**已有的 `config.toml` 不会被覆盖**。

上位机（界面）是单独的包，按 [litearm-studio 的发布页](https://github.com/nexform-tech/litearm-studio/releases) 装：

```bash
sudo dpkg -i ./litearm-studio_<版本>_amd64.deb
```

装完 `litearm-studio` 就在 PATH 上，应用菜单里出现 **LiteArm VR**。窗口用的是系统自带的 GTK / WebKitGTK（apt 依赖会装上），不需要另装浏览器或 `pywebview`。

安装后确认服务在跑：

```bash
systemctl --user status litearm-vr-bridge
journalctl --user -u litearm-vr-bridge -n 50 --no-pager   # 起不来的原因都在这里
```

> 安装包的 `scripts/` 里另有几个辅助脚本（`up.sh` 体检 / 看状态、`preset.sh` 换机型预设、
> `ports.sh` 列板子与串口）。它们只是把下面的 `systemctl` / `journalctl` 包了一层；
> **这几个脚本不在也不影响本文的任何操作**，照下面的命令做即可。

---

## 1. 启动上位机

### 图形界面

在应用菜单里搜索 **“LiteArm VR”** 并打开。

### 终端

```bash
litearm-studio
```

程序会打印监听地址并打开窗口。已经有一个实例在跑时，再启动不会多开进程，而是把那个窗口抬到前面。
**关掉窗口就是退出界面**；桥是独立的 systemd 服务，不跟着退（要停桥见 §6）。

### 仿真试跑（不碰真机）

```bash
litearm-studio --fake
```

> ⚠ VR 遥操时，机械臂那条串口必须留给桥的 follow 服务。
> 如果界面自带的守护进程去抢它（表现：页面连不上臂、follow 起不来），就让它别连臂：
>
> ```bash
> litearm-studio --port /dev/ttyDOES_NOT_EXIST
> ```
>
> VR 页的状态、关节角和电机负载都来自桥的 zenoh 通道，不需要界面自己连臂。

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
  tail -40 ~/.local/state/litearm-vr-bridge/logs/gripper_server_armA.log
  ```
  `can0` 不在、或日志里写未使能，就先查 CAN 盒/线（`dkms status` 里 `litearm-can` 应为 `installed`）。

---

## 4. 修改配置文件

配置文件：

```bash
~/.config/litearm-vr-bridge/config.toml
```

优先级从高到低：

1. `overrides.json`（上位机「链路配置」页面改的项）
2. `config.toml`
3. 代码里的出厂默认

### 4.1 用预设（改机型首选）

预设脚本在安装包的 `scripts/` 里：

```bash
/opt/litearm/litearm-vr-bridge/scripts/preset.sh list
/opt/litearm/litearm-vr-bridge/scripts/preset.sh single-upright-grip   # 正装单臂 + 夹爪
```

脚本会：备份旧配置 → 校验 → 写入 `~/.config/litearm-vr-bridge/config.toml` → 重启桥，备份文件名形如 `config.toml.bak-YYYYmmdd-HHMMSS`。
没有这个脚本（精简安装 / 源码直装）就直接按 4.2 手改 `config.toml`。

### 4.2 手工编辑

```bash
nano ~/.config/litearm-vr-bridge/config.toml
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
ip -br link | grep can                     # 夹爪所在的 CAN 口
```

（安装包里另有 `scripts/ports.sh`，把这些列成一张表、`-w` 还能监视插拔；没有它上面两条就够。）
把看到的那条 STM32 路径写进去（TOML **不会**展开 `$HOME` 之类的变量，必须写实际路径）：

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
~/.config/litearm-vr-bridge/overrides.json
```

它**优先于** `config.toml`。要让 `config.toml` 重新生效：

- 在对话框里点「恢复默认」，或
- ```bash
  rm -f ~/.config/litearm-vr-bridge/overrides.json
  systemctl --user restart litearm-vr-bridge
  ```

`machine.arms` 这类项需要重启桥，对话框会标出 `apply=restart`。

### 4.4 确认改对了

```bash
systemctl --user restart litearm-vr-bridge
systemctl --user status litearm-vr-bridge
journalctl --user -u litearm-vr-bridge -n 30 --no-pager
```

启动日志里逐条写着：头显入口与 zenoh 端点、每条臂的串口与 follow 端点、受管进程命令、
以及**有哪几项被 `overrides.json` 压住了**（会点出 `config.toml` 里相应的值）。
“改了没生效”先看这几行。

### 4.5 进阶：换 `follow.yaml` / `teleop.yaml`（夹爪力度、跟手手感、标定）

桥自己的参数在 `config.toml`；**夹爪 / follow 的参数在 `follow.yaml`，teleop 的参数在 `teleop.yaml`**。
包内那两份在 `/opt` 下、升级会被覆盖，所以改法是「复制到家目录 + 告诉桥读那份」：

```bash
mkdir -p ~/litearm-config
cp -n /opt/litearm/litearm-vr-bridge/submodules/litearm-vr-follow/follow.yaml \
      ~/litearm-config/follow.yaml
cp -n /opt/litearm/litearm-vr-bridge/submodules/litearm-vr-teleop/teleop/config/teleop.yaml \
      ~/litearm-config/teleop.yaml
```

然后在 `~/.config/litearm-vr-bridge/config.toml` **末尾**加上这两行（`[upstream.env]` 是段名，要带方括号）：

```toml
[upstream.env]
FOLLOW_CONFIG = "/home/<你的用户名>/litearm-config/follow.yaml"
TELEOP_CONFIG = "/home/<你的用户名>/litearm-config/teleop.yaml"
```

```bash
systemctl --user restart litearm-vr-bridge
```

- `FOLLOW_CONFIG` 由 follow 的启动脚本自己读（follow / 夹爪 / 状态汇总都读它）；`TELEOP_CONFIG` 由桥翻成上游 launcher 的 `--config` 传下去。值**必须是绝对路径**：这里不展开 `~` 和 `$HOME`，写错当场报错。
- 这一段只收**上游启动脚本认的名字**（除这两条外，如 `LITEARM_PY_SRC`）；桥自己的键（`machine.*` 等）写进这里会报错，并指出该写在哪一段。
- 升级只覆盖 `/opt` 下的两份 yaml，`~/litearm-config/` 里的不受影响 —— 不需要再动包内的 yaml，也不需要软链接。
- 改完要重启桥；teleop 进程还要等页面上点「开始遥操」才会拉起。

常改的几项（出厂值取自当前包内这两份 yaml）：

| 想改什么 | 文件 | 键 | 出厂值 |
| --- | --- | --- | --- |
| 夹爪夹持力上限（N） | `follow.yaml` | `gripper.force` | `20.0`（夹持力 ≈ 这个值；×0.1 = Nm 力矩预算） |
| 夹爪开合速度上限（mm/s） | `follow.yaml` | `gripper.speed` | `100.0` |
| 夹爪开合方向反了 | `follow.yaml` | `gripper.mount = "reverse"` | 未设（`normal`） |
| 换了夹爪电机、CAN ID 变了 | `follow.yaml` | `gripper.can_id` | `"0x08"` |
| 上电夹爪自己闭合 | `follow.yaml` | `gripper.no_startup_open` | `false` |
| 夹爪过温保护 | `follow.yaml` | `gripper.temp_limit` | `80`（℃） |
| 手停下时机械臂抖 | `teleop.yaml` | `tuning.deadband_scale` | `0.1`（`1.0` = 用原始阈值；`0.0` = 关掉死区，噪声会原样放行，别关） |
| 跟手太飘 / 太钝 | `teleop.yaml` | `tuning.smooth_alpha` | `0.2`（`1.0` 最跟手也最抖，`0.5` 更跟手；**不能写 0**） |
| 末端抓取点高度变了 | `teleop.yaml` | `tuning.tcp_offset` | `[0.0, 0.0, 0.18]`（米，标定值） |
| 快挥时关节跳变 | `teleop.yaml` | `sim.max_step_rad` | `0.2`（rad/帧） |
| 换 IK 后端 | `teleop.yaml` | `tuning.ik` | `pybullet`（另有 `pin` / `exact`） |
| 手柄方向不对 | `teleop.yaml` | `mapping.*` | 标定值，别凭手感改 |

> 两份 yaml 的全部键（包括 `tuning.notch` / `twist_accel`、`pin_ik`、`aggregator` 等）
> 见随交付的《LiteArm VR Bridge 参数对照表》（`litearm-vr-bridge-config.md`）。
> 表里标「标定值」的段落是对着真机量出来的，换机器 / 换夹爪要重新标定。

---

## 5. 配置文件完整选项

### 5.1 文件位置与优先级

按顺序查找，先找到的算：

| 顺序 | 位置 |
| --- | --- |
| 1 | `--config PATH`（命令行显式指定） |
| 2 | `~/.config/litearm-vr-bridge/config.toml` |
| 3 | `/etc/litearm-vr-bridge.toml` |

三处都没有 = 全部使用出厂默认，桥仍可启动（合法状态）。

上位机「链路配置」页面改的项写入：

```bash
~/.config/litearm-vr-bridge/overrides.json
```

优先级：**`overrides.json` > `config.toml` > 出厂默认**。

> ⚠ TOML 文件**不会展开** `$HOME` 之类的环境变量。
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
| `server_py` | text | 空 = 自动找 | 重启桥 | 否（锁定） | 上游 teleop 服务端解释器；包内已带冻结版 teleop，正常留空 |

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
`[arms.A]` 的默认槽位是 **left（侧装）**；单臂**正装**要同时写明 `side = false`，臂才占 right 槽
（`preset.sh` 的预设就是这么写的）。槽位配反了不报错，表现是"哪只手柄在控哪条臂"反了。

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
| `teleop_repo` | text | 未设置 = 用包内那份 | 重启桥 | 否（锁定） | 上游 teleop 路径；包内已带（submodule 冻结） |
| `follow_repo` | text | 未设置 = 用包内那份 | 重启桥 | 否（锁定） | 上游 follow 路径；包内已带（submodule 冻结） |
| `teleop_frozen` | text | 未设置 = 包内有冻结产物就用 | 重启桥 | 否（锁定） | 冻结版 teleop 可执行文件；写了空字符串 = 强制不用 |

### 5.9 `[upstream.env]` —— 给受管子进程的环境变量（正常不用写）

装包时这些变量已经写进 systemd 单元，日常用不到这一段：

```bash
grep '^Environment=' ~/.config/systemd/user/litearm-vr-bridge.service
# Environment=PYTHONPATH=/opt/litearm/litearm-vr-bridge/src:/opt/litearm/litearm-vr-bridge/vendor:/opt/litearm/litearm-vr-bridge/vendor-upstream
# Environment=LITEGRIP_SDK=/opt/litearm/litearm-vr-bridge/sdk/litegrip-python/src
# Environment=LITEARM_PY_SRC=/opt/litearm/litearm-vr-bridge/sdk/litearm-python/src
```

只有下面这几种情况才需要自己写（键名任意、值必须是字符串）。换 `follow.yaml` / `teleop.yaml`
的完整做法见 4.5：

```toml
[upstream.env]
# 换上游那两份 yaml（**只写绝对路径**；包内那两份不用动，改包内的会被升级冲掉）
# FOLLOW_CONFIG = "/home/me/litearm-config/follow.yaml"
# TELEOP_CONFIG = "/home/me/litearm-config/teleop.yaml"
# 自己换了一份源码检出 / SDK，或要指定 teleop 的解释器
# LITEARM_PY_SRC = "/opt/litearm/follow"
# LITEGRIP_SDK = "/opt/litearm/litegrip/src"
# TELEOP_SERVER_PY = "/home/me/miniconda3/envs/metavr-arm/bin/python"
# 自定义受管命令
# GRIPPER_SERVER_CMD = "..."
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
# 服务状态 / 重启 / 停止
systemctl --user status litearm-vr-bridge
systemctl --user restart litearm-vr-bridge
systemctl --user stop litearm-vr-bridge

# 实时日志（最近 50 行 + 跟随）
journalctl --user -u litearm-vr-bridge -n 50 -f

# 各进程日志
ls ~/.local/state/litearm-vr-bridge/logs/
tail -f ~/.local/state/litearm-vr-bridge/logs/teleop.log
tail -f ~/.local/state/litearm-vr-bridge/logs/follow_server_armA.log
tail -f ~/.local/state/litearm-vr-bridge/logs/gripper_server_armA.log
```

链路摘要（头显帧率、臂设备、follow、兜底状态）在 VR 页面上看；页面打不开时，
就用 `journalctl` 看桥自己的输出。

> 装成**系统级**服务（无人值守的机器）时，上面这些命令去掉 `--user`，日志改看 `journalctl -u litearm-vr-bridge`。

---

## 7. 常见问题

| 现象 | 处理 |
| --- | --- |
| 「开始遥操」是灰的 | 头显没上线，或桥不在线；先看页面「VR 链路」与 `systemctl --user status litearm-vr-bridge` |
| 头显连不上 `ws://<IP>:8765` | 确认头显与本机同网段；`ss -ltnp \| grep 8765`；检查本机 IP 是否变了 |
| 页面显示「跟随服务报错: Connection refused」 | 还没点「开始遥操」时属正常；开始后 follow 才会监听 9001 |
| teleop 意外退出 / 页面显示“兜底持位” | 再点「开始遥操」；反复退出看 `teleop.log`（常见：手动关掉了 teleop 的 pybullet 窗口） |
| 夹爪不动 | `ip -br link \| grep can0`；看 `gripper_server_armA.log`；确认配置里有 `grip = "can0@..."`；`dkms status` 里 `litearm-can` 应为 `installed` |
| `ls /dev/serial/by-id/` 为空 | 臂没有插好/被拔掉，或 USB 线/供电问题；重新插拔后再看 |
| 串口被占用、follow 起不来 | `sudo fuser -v /dev/ttyACM*` 看谁占着；也可能是上位机守护进程抢了串口（见 §1）；桥这边的说法在 `journalctl --user -u litearm-vr-bridge -n 100` |
| 改了 `config.toml` 却不生效 | 看 `overrides.json` 是否压过了它（见 4.3） |
| 服务没起来 / 装完没有任何单元 | `systemctl --user status litearm-vr-bridge`；没有这个单元说明 `postinst` 没装成：用自己的账号跑一次安装包里的 `scripts/install.sh`（装包时它打印的就是这条命令），或重装 deb |

---

## 8. 安全提醒

- 首次按 grip 会 **`movej` 回零位**，确认工作区无人、急停在手。
- 页面上的 **急停** 会向 follow 下发 stop。
- 真机模式（`real = true`）下，「开始遥操」会真的驱动机械臂；调试界面先用 `litearm-studio --fake`。
- 松开 grip 只是暂停跟随，不是断电；需要保持时让机械臂自己持位。

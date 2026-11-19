[English](QUICKSTART.md) | 简体中文

# 快速上手

从零把整套 VR 遥操链路跑起来：先仿真，再接真机。

> 命令一律**从仓库根目录**（`litearm-vr/`）出发。`<PC-IP>` 换成本机局域网地址，设备名
> （`/dev/ttyACM0`、`can0`）换成你现场实际使用的那个。

---

## 0. 前置条件

**电脑**

- Ubuntu 20.04 / 22.04，x86_64，带图形桌面（pybullet 预览窗口需要）。
- Python 3 与 [conda](https://docs.conda.io/)（或 miniconda）。遥操服务跑在独立的
  `metavr-arm` 环境里（Python 3.10）。
- **可选**：ROS 2 Humble，仅 RViz2 可视化后端需要。

**头显**

- 受支持的 OpenXR VR 头显，运行 LiteArm VR 客户端（闭源，单独分发），与电脑在同一 Wi-Fi / 局域网。

**真机（可选）**

- LiteArm V4 七轴臂；可选夹爪与 L6 灵巧手。
- 一种臂接口：USB 串口（`/dev/ttyACM*`，STM32 路线）**或** SocketCAN 适配器（`can0`，
  SDK 路线）。两者不能同时使用。
- CAN 总线配成经典 CAN，1 Mbps。
- 串口权限：把用户加入 `dialout` 组，然后重新登录：

  ```bash
  sudo usermod -aG dialout "$USER"
  ```

- CAN 接口配置与桥的一次性安装需要 `sudo`。

纯仿真不需要任何硬件。

---

## 1. 获取代码

```bash
git clone https://github.com/nexform-tech/litearm-vr.git
cd litearm-vr
```

三个组件都在子目录里，无需 `--recursive`——它们是工作树，不是 git 子模块。

---

## 2. 建遥操环境（首次）

```bash
bash litearm-vr-teleop/teleop/environments/setup.sh   # 创建/重建 conda 环境 metavr-arm
cd litearm-vr-teleop/teleop && make smoke && cd -      # 冒烟测试：应输出 SMOKE TEST PASSED
```

conda 不在 `PATH` 时，先激活它，或给 `setup.sh` 传 `CONDA_ROOT=/path/to/conda`。公司网络
如需换软件源，用 `CONDA_MAIN` 与 `CONDA_R` 覆盖，详见脚本头部注释。

---

## 3. 纯仿真跑通（无硬件）

```bash
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py
```

- VR 头显客户端连 `ws://<PC-IP>:8765`。
- 默认：单臂正装、pybullet 逆解、pybullet 预览窗口，**不接真机**。
- 操作：按住 `gripActive` 启动 / 锚定，松开暂停；`trigger` 边沿切换夹爪开合。
- 停止：`Ctrl-C`，或在 `litearm-vr-teleop/teleop` 下 `make stop`。

手边没有头显？起桥之后可以先用假数据源喂链路：

```bash
python3 litearm-vr-bridge/scripts/fake_headset.py --help
```

---

## 4. 起桥（可选）

桥把头显帧经 zenoh 转给遥操，并额外提供链路状态汇总、掉帧兜底和受管进程启停。
纯仿真不需要它。

```bash
cd litearm-vr-bridge
sudo ./scripts/install.sh     # 一次性：建 venv、装 systemd unit、生成 /etc/litearm-vr-bridge.env
./scripts/up.sh               # 之后日常就这一条（会先跑 --doctor 预检）
cd ..
```

图形会话里没有 `sudo` 时，用 `./scripts/install.sh --user`。

第一次接真机、逐项联调请看 `litearm-vr-bridge/docs/REAL-ARM-BRINGUP.md`。

---

## 5. 接真机

> ⚠️ **真机会真实运动。** 确认机械臂已上电、工作区无人、急停在手边。

### 5.1 安装硬件侧 SDK

每个跟随服务都会 import 对应的厂商 SDK。可以装成 Python 包，也可以用环境变量把 SDK 的
`src/` 目录指过去。

| 服务 | SDK | 覆盖变量 |
| --- | --- | --- |
| 臂·STM32 路线（`run_stm32_server.sh`） | `litearm`（litearm-python） | `LITEARM_PY_SRC` |
| 臂·SDK 路线（`run_sdk_server.sh`） | `pylitearm` | — |
| 夹爪（`run_gripper_server.sh`） | `litegrip`（litegrip-python） | `LITEGRIP_SDK` |
| 灵巧手（`run_hand_server.sh`） | linkerbot SDK | `LINKERBOT_SDK` |
| 状态汇总 | `eclipse-zenoh`（`pip install eclipse-zenoh`） | — |

从源码安装的示例：

```bash
python3 -m pip install -e /path/to/litearm-python        # 提供 `litearm`
export LITEARM_PY_SRC=/path/to/litearm-python/src        # 不想 pip 安装时走这条
```

缺 SDK 时，启动脚本会明确报出缺哪个包、找过哪些路径。

### 5.2 起硬件跟随服务

先起跟随服务，再让遥操下发；服务没起来时 `--real` 会连不上。

所有默认值都来自 `litearm-vr-follow/follow.yaml`。命令行参数覆盖文件，文件覆盖内置常量。

**臂**——二选一，互斥：

```bash
# STM32 路线：控制律在固件，走串口。
# --mode j 免调增益；链路验证完再切 jf / mit。
litearm-vr-follow/run_stm32_server.sh --mode j --tcp-port 9001

# SDK 路线：控制律在主机，走 SocketCAN。
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
litearm-vr-follow/run_sdk_server.sh --iface can0 --tcp-port 9001
```

串口不是 `/dev/ttyACM0` 时，改 `follow.yaml` 的 `port`（`/dev/serial/by-id/...` 比
`ttyACM*` 更稳定）。

**夹爪**——先 `can0 up`（经典 CAN，1 Mbps）：

```bash
litearm-vr-follow/run_gripper_server.sh --can can0 --port 9003
```

**灵巧手**：

```bash
litearm-vr-follow/run_hand_server.sh --can can0 --port 9004
```

**双臂 + 桥托管（捷径）**：两条臂都在本机、且由手工起时，
`litearm-vr-bridge/scripts/start.sh` 一条命令就把两个跟随服务、桥和遥操一起起来
（`--stop` 反着停）。默认 `left=/dev/ttyACM0:9001`、`right=/dev/ttyACM1:9002`，用
`FOLLOW_{LEFT,RIGHT}_{SERIAL,PORT}` 覆盖。

### 5.3 遥操接真机

```bash
# 只接臂
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --real

# 臂 + 夹爪
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --real \
    --gripper --gripper-remote 127.0.0.1:9003

# 臂 + 灵巧手
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --real \
    --hand --hand-remote 127.0.0.1:9004
```

单臂默认端口 `9001`；双臂配置左 `9001` / 右 `9002`。必须与跟随服务的 `--tcp-port` 一致
（或显式传 `--real-left` / `--real-right`）。

### 5.4 状态汇总（只读，可并行跑）

把各跟随服务的实测状态按臂身份发布到 zenoh
（`litearm/v4/{armA,armB}/vr_{arm,grip,hand}`）。它不下发命令，也不写电机：

```bash
litearm-vr-follow/run_status_aggregator.sh \
    --arm armA=/dev/serial/by-id/usb-...-if00@127.0.0.1:9001 \
    --gripper armA=can0@127.0.0.1:9003

# 另开一个终端看订阅
python3 litearm-vr-follow/status_listen.py
```

### 5.5 STM32 单臂正装的 120° 转腕复测

遥操通过 `state` 读取跟随端的 `q_min/q_max`，与 URDF 行程取交集后用于 IK。
默认纯转腕约束为手柄平移 5 mm、J1–J4 相对该段起点各 ±20°、关节速度 1 rad/s。
腕部行程不足时，求解优先保持 TCP 位置，允许姿态跟随误差。

#### 更新和启动

按现场修改旧检出目录和 Python 路径。将 `PR_NUMBER` 设为待验收的 PR 编号，从后继仓库拉取该提交，在独立目录启动两端。

```bash
export OLD_REPO="/path/to/current/litearm-vr"
export FIELD_WT="$HOME/litearm-vr-wrist-test"
export TELEOP_DIR="$FIELD_WT/litearm-vr-teleop/teleop"
export RUN_DIR="$HOME/wrist-120/run-120"
export TELEOP_PY="$HOME/miniconda3/envs/metavr-arm/bin/python"

git clone --no-checkout https://github.com/nexform-tech/litearm-vr.git "$FIELD_WT" &&
git -C "$FIELD_WT" fetch origin "refs/pull/${PR_NUMBER:?请设置 PR 编号}/head" &&
git -C "$FIELD_WT" switch --detach FETCH_HEAD &&
mkdir -p "$RUN_DIR" &&
cp "$OLD_REPO/litearm-vr-follow/follow.yaml" "$RUN_DIR/follow.yaml" &&
cp "$OLD_REPO/litearm-vr-teleop/teleop/config/teleop.yaml" "$RUN_DIR/teleop.yaml"
```

使用自定义 `FOLLOW_CONFIG` 或 `--config` 的现场应复制实际生效的文件。
松开 grip 并确认持位后，依次停止旧遥操和旧跟随端。下面三个终端使用同一组路径变量，
沿用已有的 Zenoh bridge 和 STM32 SDK 环境。

终端 A 启动跟随端。

```bash
FOLLOW_CONFIG="$RUN_DIR/follow.yaml" \
  "$FIELD_WT/litearm-vr-follow/run_stm32_server.sh" \
  --port /dev/ttyACM0 --mode jf --tcp-port 9001 \
  2>&1 | tee "$RUN_DIR/follow.log"
```

终端 B 查询限位并采样。确认 `limits.json` 中 `valid` 为真，且上下限各有七个弧度值。
若跟随端报告读取固件限位失败，回包使用跟随配置的范围，固件限位仍待核验。

```bash
python3 "$TELEOP_DIR/scripts/wrist_120_check.py" limits \
  127.0.0.1:9001 "$RUN_DIR/limits.json" &&
python3 "$TELEOP_DIR/scripts/probe.py" sample \
  --addr 127.0.0.1:9001 --hz 50 --out "$RUN_DIR/follow.jsonl"
```

终端 C 启动遥操。示例以法兰为 TCP，安装工具时使用实际工具偏移，并在导出时传入同一组值。

```bash
python3 "$TELEOP_DIR/launch/run_teleop_webxr.py" \
  --metavr-py "$TELEOP_PY" --config "$RUN_DIR/teleop.yaml" \
  --source zenoh --arm-id armA --zenoh-mode peer --zenoh-connect tcp/127.0.0.1:7447 \
  --no-dual --no-side --ik pybullet --control-frame tool --tcp-offset 0 0 0 \
  --no-hand --no-gripper --real --real-left 127.0.0.1:9001 --real-right 127.0.0.1:9001 \
  --probe "$RUN_DIR/sim.jsonl" 2>&1 | tee "$RUN_DIR/teleop.log"
```

#### 采集和判定

1. 空载并清空工作区，急停放在手边。按现场流程将手臂移至 `sim.single_q_home` 附近。
2. 右手握住 grip，静止约 1 秒，然后固定手柄位置，绕握持时的局部 X 轴在约 2 秒内转到 +120°。
   保持约 1.5 秒，再用约 2 秒转回，静止约 1 秒后松开。出现异常扫动时立即松开。
3. 一次采集保留一段握持，避免触发 A 键回零。松开后用 Ctrl-C 结束遥操和采样并导出。

```bash
python3 "$TELEOP_DIR/scripts/wrist_120_check.py" report "$RUN_DIR" \
  --urdf "$TELEOP_DIR/models/litearm_description/urdf/litearm_description.urdf" \
  --tcp-offset 0 0 0
```

- 在 `tcp.csv` 检查手柄峰值接近 120°、手柄平移小于配置阈值，且转动和保持期启用 `wrist_tracking`。
- 在 `joints.csv` 检查遥操目标、跟随端收到的目标和下发指令均位于有效限位内，
  并检查 J1–J4 最大偏移。J7 接近有效边界且保持在边界内时，才能确认到限后的行为。
- `tcp.csv` 的 `measured_fk` 由实测关节角和模型正运动学推算。按台架允许偏差验收
  转动、保持、回转期间的峰值及结束偏移。PyBullet 离线参考值为 J1 峰值 20°、
  J7 最低 −89°、TCP 峰值偏移约 0.02 mm。

保留整个采集目录。需要回退时，先结束新进程，再从旧检出目录按原命令启动。

---

## 6. 停止

```bash
# 桥：先让机械臂就地保持，再收掉受管子进程
litearm-vr-bridge/scripts/up.sh --stop

# 遥操
cd litearm-vr-teleop/teleop && make stop
```

跟随服务在各自的终端里 `Ctrl-C` 停掉。不要让跟随服务继续跑，同时让别的程序接管串口或
CAN 口。

---

## 7. 排障速查

| 现象 | 先看 |
| --- | --- |
| 遥操起来了但机械臂不动 | 头显是否已连上 WebSocket、是否按住了 `grip`；`--real` 时跟随服务是否已起 |
| 连不上 8765 / 9001 | 端口被占；桥占用 8765 时，遥操应用 `--port 8766` |
| 桥 `--doctor` 报 fail | 按提示逐条修（串口/CAN、conda、`DISPLAY`、上游路径等） |
| 串口打不开 | 是否被别的程序占用；用户是否在 `dialout` 组 |
| CAN 打不开 | 接口是否已按 1 Mbps `up`；是否被别的程序占用 |
| 夹爪一上电就闭合 | 这是 `gripper_server` 启动即 `open` 的设计，别加 `--no-startup-open` |
| `ModuleNotFoundError: litearm` / `litegrip` / `linkerbot` | 安装对应 SDK，或设 `LITEARM_PY_SRC` / `LITEGRIP_SDK` / `LINKERBOT_SDK` |
| 遥操报跟踪误差但机械臂本身正常 | 对一下实际下发的 `cmd_q` 与实测 `q`；跟随侧可能在做限速 |
| pybullet 窗口不出现 | 没有图形会话或 `DISPLAY`；在本机的桌面会话里启动 |

更细的排障看各组件的 `README.md` 与其 `docs/`。

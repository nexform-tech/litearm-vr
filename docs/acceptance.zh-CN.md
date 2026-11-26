[English](acceptance.md) | 简体中文

本页记录 VR 遥操链路的验收证据：测了什么、在什么硬件或环境上、实测结果，以及仍未关闭的已知缺口。

## 1. 范围

证据覆盖本仓交付的三个组件：

- `litearm-vr-bridge`：头显接入、Zenoh 转发、状态汇总、掉线兜底、受管进程启停。
- `litearm-vr-teleop`：手柄到关节的遥操、IK、预览与真机客户端。
- `litearm-vr-follow`：STM32 串口链路与 SDK/CAN 链路的真机跟随服务。

## 2. 仿真链路

在参考笔记本 + Pico 头显 + PyBullet DIRECT 上运行：

- teleop 离线套件：80/80 通过，其中锚定跟踪 21/21。
- bridge 离线套件：168/168 通过。
- 锚定行为：固定锚点、100 次往返、跨 180°、不可达目标恢复、操作者领先处理、执行偏差保护。
- 预览跟踪误差（`|ee_after − 目标|`）：中位 0.9–1.6 mm，p90 3.2–8.6 mm。

## 3. 真机链路

使用 `Litearm1.12.3-7J` 机械臂、`/dev/ttyACM0`、单臂正装、STM32 follow 服务 `jf` 模式，follow slew 设为 3.0 rad/s 与 6.0 rad/s²。

- `ctl up` 一条命令启动 follow 与 teleop；头显连接 90 fps，bridge 转发耗时 0.16 ms。
- grip 按下/松开产生 `following` / `paused` 事件，`owner` 在 `none` 与 `vr` 间切换。
- `ctl down` 先就地 latch，再停止 teleop（3.3 s）与 follow（0.46 s），未使用强制停止。
- 按目标速度分组的「目标 vs 实测 TCP」跟踪误差：

| 目标速度 | 中位 | p90 |
| --- | --- | --- |
| < 20 mm/s | 9–11 mm | 17 mm |
| 20–100 mm/s | 12–20 mm | 66 mm |
| > 100 mm/s | 26–50 mm | 107 mm |

- 角度误差：中位 3–7°，p90 9–21°。
- 指令 vs 实测执行误差：中位 12–18 mm。

快速运动时的滞后来自 follow 的 slew 限速，不是 IK 或预览模型；慢速基线为 9–17 mm。

## 4. 部署包

发布包在干净的 Ubuntu 22.04.5 虚拟机中构建并安装：

- 产物：`litearm-vr-bundle_0.2.2_amd64.deb`（110 MB）、`.tar.gz`（151 MB）、SHA-256 文件，以及含源码 commit、SDK pin、DKMS commit 的 `MANIFEST.json`。
- `dpkg -i` 成功；bridge 以系统级服务安装（`User=ubuntu`），active 且 enabled。
- 对主进程 `kill -9` 后，`Restart=always` 在 5 秒内恢复。
- 升级模拟：从 venv 中删除依赖后重装发布包，依赖从 wheelhouse 恢复，`/etc/litearm-vr-bridge.env` 保持不变。

## 5. 已知缺口

| 缺口 | 状态 | 证据 |
| --- | --- | --- |
| 夹爪分片抖动、`open()` 回弹 | 代码已重写为 200 Hz 控制环；待接夹爪做硬件验证 | Issue #11 |
| DKMS `gs_usb` 模块加载 | 可构建并安装；测试 VM 内核缺 `can-dev`，`modprobe` 失败 | Issue #12 |
| 快速运动下的跟随滞后 | follow slew 限速的预期行为，可通过 `max_speed` / `max_accel` 调整 | Issue #10 |
| `litearm-server`（7449）共存 | 设计待实测项 | `litearm-vr-bridge/docs/DESIGN.md` §8 |
| 上位机与 teleop 的 URDF 统一 | 设计待决策项；以 teleop 的 URDF 为准 | `litearm-vr-bridge/docs/DESIGN.md` §8 |

## 6. 复现检查

```bash
# teleop 测试（在 litearm-vr-teleop/teleop 下）
PYTHONPATH=src python3 -m unittest discover -s tests -p "test_*.py"

# bridge 测试（在 litearm-vr-bridge 下）
PYTHONPATH=src python3 -m pytest tests -q

# follow 协议测试（在 litearm-vr-follow 下）
python3 -m unittest test_measured_state.py
```

真机验收按 `QUICKSTART.md` 与 `litearm-vr-bridge/docs/REAL-ARM-BRINGUP.md` 执行。

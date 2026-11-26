[English](accuracy.md) | 简体中文

本页定义 VR 遥操的精度指标：每项指标的含义、测量时的配置、当前的实测基线与复测方法。对外报数和判断一次改动是否把跟手性做坏了，都以本页为准。

## 适用范围与配置

基线于 2026-11-26 在以下配置下测得：

- 机械臂：`Litearm1.12.3-7J`，单臂正装（`--upright`）。
- `tcp_offset = 0`：受控点是法兰原点。未装夹爪，所有数字都描述法兰精度；换成非零偏置后受控点移动，数字随之变化。
- follow：STM32 串口链路（`jf` 模式），限速 `--max-speed 3.0` rad/s、`--max-accel 6.0` rad/s²；其中一轮对比用 `--max-accel 10`。
- 头显：Pico 4 Ultra；桥转发 90 fps，单帧转发耗时 0.16 ms。
- 采样：teleop 探针（sim 流 + follow 采样 50 Hz），每轮 1010 帧 / 18.7 s，共三轮，操作者重复相近动作。

## 指标定义

| 指标 | 定义 | 单位 |
| --- | --- | --- |
| 预览跟踪误差 `E_preview` | 每帧 IK 末端位姿与预览目标的距离：`distance(ee_after, ee_target)` | mm |
| 跟随误差 `E_follow` | 头显目标 TCP 与真机实测 TCP 的距离，按目标速度分档（`<20`、`20–100`、`>100` mm/s） | mm |
| 执行误差 `E_exec` | 指令 TCP（IK 与限速之后）与实测 TCP 的距离 | mm |
| 姿态误差 `E_rot` | 目标与实测 TCP 姿态的夹角 | ° |

统计量取中位、p90（90 分位）与最大值。以中位为口径：每轮操作手势不同，单轮的极大值说明的是一次快甩，而不是系统能力。

`E_preview` 由 `probe.py report` 直接输出；`E_follow`、`E_exec`、`E_rot` 按最近时间戳把 sim 流与 follow 采样对齐后按速度分档统计——四类指标来自同一次录制，一次采集即可。

## 实测基线

| 指标 | 条件 | 中位 | p90 | 最大 |
| --- | --- | --- | --- | --- |
| `E_preview` | 全部帧 | 0.9–1.6 mm | 3.2–8.6 mm | 15.3 mm |
| `E_follow` | `< 20` mm/s | 9–11 mm | 17 mm | — |
| `E_follow` | `20–100` mm/s | 12–20 mm | 66 mm | — |
| `E_follow` | `> 100` mm/s | 26–50 mm | 107 mm | 190.5 mm |
| `E_rot` | 全部帧 | 3–7° | 9–21° | 32.7° |
| `E_exec` | 全部帧 | 12–18 mm | 24–32 mm | — |

区间来自三轮测量，三轮的操作速度不同（目标速度中位 0.16–0.24 m/s）。快速档的误差主要由 follow 的限速拖后造成，属于预期行为，不是 IK 或模型的缺陷；慢速档限速不生效，才是「跟得上时有多准」的有效基线。

## 建议的回归阈值

以下阈值用于发现回退，不作为对客户的验收承诺，对外引用前需产品确认。

| 指标 | 警告 | 失败 |
| --- | --- | --- |
| `E_preview` 中位 | > 2 mm | > 3 mm |
| `E_follow` `< 20` mm/s 中位 | > 15 mm | > 20 mm |
| `E_follow` `20–100` mm/s 中位 | > 25 mm | > 40 mm |
| `E_follow` `> 100` mm/s 中位 | > 60 mm | > 80 mm |
| `E_exec` 中位 | > 20 mm | > 30 mm |
| `E_rot` 中位 | > 8° | > 12° |

## 复测方法

带探针起 teleop，另开一个终端采集 follow 服务：

```bash
# ① teleop 带探针（写 sim 流；正常遥操时不要开）
./teleop/launch/run_teleop_webxr.py --real --real-right 127.0.0.1:9002 \
    --probe /tmp/probe_sim.jsonl

# ② follow 采样（follow 服务是第二段链路的真值）
python3 teleop/scripts/probe.py sample --addr 127.0.0.1:9002 --out /tmp/probe_follow.jsonl

# ③ 动作结束后出归属报告
python3 teleop/scripts/probe.py report --sim /tmp/probe_sim.jsonl --follow /tmp/probe_follow.jsonl
```

配置必须与「适用范围与配置」一致：同样的 `tcp_offset`、同样的限速、同样的安装方式。做参数对比时用固定轨迹（固定幅度与周期）——手动手势的方差大于多数参数差异。

## 注意事项

- 数字随限速变化。调高 `--max-speed`/`--max-accel` 会缩小快速段的滞后，同时抬高峰值误差；调低则相反。
- 全程 `tcp_offset = 0`。装夹爪或加工具偏置后受控点改变，同样的绝对误差对应的角度误差也会不同。
- 目标速度由探针流估计，采样时间抖动会给这个估计带来噪声；误差统计本身不依赖该估计。
- 只测了法兰。夹爪侧精度取决于夹爪及其标定，不在本页范围内。

# follow_server 两版协议差异与 bridge 兼容性

> 背景：把跟随服务从 stm32 版（串口 /dev/ttyACM0，控制律在 STM32 固件）换成
> sdk 版（CAN can0，控制律在主机 pylitearm `JointFollowController`）。两者是互斥的
> 两条控制路径，但对外 TCP/JSON 协议声称"完全一致"——实测**并非完全一致**，本文
> 逐字段核对差异，并给出 bridge / VR 端 / 测试脚本各自的处理结论。

## 1. 结论（TL;DR）

| 对象 | 要不要动 | 说明 |
| --- | --- | --- |
| teleop VR 端（real_arm_client） | 不动 | fire-and-forget，从不读回包字段 |
| bridge `follow.py` | **已改** | `fault`/`flags` 兼容 |
| bridge `bridge.py` | **已改** | `MANAGED_ARM_STATES` 补 stm32 跟随 mode |
| 早上测试脚本 | 要改 | `start` 后改 ping 拿 q0、`flags`→`fault` |
| 重力方向 | **待确认** | 默认 config 仍是正装 base_rpy |

## 2. 两版协议差异

桥对 9001 只用到 `start / hold / latch / stop / ping`，差异集中在回包字段：

| 字段 | stm32 版 | sdk 版 |
| --- | --- | --- |
| `start.q` | 有（读到的当前角） | **无** |
| `ping.state` | `mode_name`：`INIT/MOVE_J/MOVE_P/MOVE_JS/MOVE_MIT/MIT_ALL/EMERGENCY` | `ArmState.value`：`disconnected/moving/following/disabled/fault/...` |
| `ping.flags` | 有（flag 名列表，健康为空） | **无** |
| `ping.fault` | **无** | 有（`[(joint, err), ...]`，健康为空） |
| `ping.q` | 有 | 有 |
| `ping.target` | 有 | 无 |

- stm32 `FLAG_NAMES`：`FAULT / WD_TRIPPED / FB_STALE / TEMP_WARN / POS_VIOL / OVERSPEED`（`uart_litearm.py:43`）。
- sdk `fault` 语义：仅非健康状态码 `[(joint, err), ...]`；`flags` 语义：置位 flag 名列表。**两者健康时都是空列表、非空即有故障**，语义可安全合并。

## 3. VR 端（teleop）：无影响

`scripts/real_arm_client.py` 是 fire-and-forget 设计：

- 只发 `start / latch / movej / q`，**从不发 `ping`**；
- 后台 `_drain_loop` 只 `recv` 丢弃回包、**不解析任何字段**（防 TCP 缓冲堵塞）；
- 从不读 `start.q`、`flags`、`fault`、`target`。

所以上面所有差异它一个都不碰。头显/WebXR 前端更不受影响（只连桥 8765）。

## 4. bridge：本次改动

### 4.1 `follow.py` `_absorb_state`（必须改）

原来只认 `fault`，stm32 模式下 `arm.fault` 恒 `None`。改为 `fault` 取不到就回退 `flags`：

```python
fault = reply.get("fault")
if not isinstance(fault, (list, tuple)):
    fault = reply.get("flags")
if isinstance(fault, (list, tuple)):
    self.fault = list(fault)
```

### 4.2 `bridge.py` `MANAGED_ARM_STATES`（可选，已改）

`owner` 判断里 `arm_state in MANAGED_ARM_STATES` 原来只认 sdk 的 `"following"`，
stm32 版的 `mode_name`（`MOVE_JS/MOVE_MIT/MIT_ALL`）对不上。补上：

```python
MANAGED_ARM_STATES = ("following", "MOVE_JS", "MOVE_MIT", "MIT_ALL")
```

影响面很小：`owner = "vr" if (governor.following or arm_state in MANAGED_ARM_STATES)`，
`governor.following` 才是主力，`arm_state` 只是兜底提示；且 `owner` 只用于状态页
显示，不参与兜底控制（兜底按帧龄 500ms 判）。

## 5. 早上测试脚本需要改的地方

`/tmp/test_follow_1ms.py`、`/tmp/test_follow_random.py` 是按 stm32 版协议写的：

1. **`start` 后拿 q0**：`q0 = r["q"]` → sdk 版 `start` 无 `q`，KeyError。
   改成 `start` 后再 `ping` 一次取 `q0`，或直接用 `Q_HOME`（movej 已到）。
2. **故障检测**：`p.get("flags") or []` / `p['flags']` → sdk 版是 `fault`。
   改成 `p.get("fault") or []`（否则 22s 高速摆动里故障检测静默失效）。
3. **j2 下摆略超软限位**：`A[1]=1.3` 从 q_home 的 -0.5 下摆到 -1.8，而 J2 软限位
   -1.797689（超 0.002 rad）。sdk `joint_follow` 有 `joint_limit_wall`（margin≈3°）会
   推回，不崩但到不了 -1.8。建议 `A[1]` 收到 1.25（下摆 -1.75）。

## 6. 重力方向：待确认

sdk 版 follow_server 默认加载 `default_config_path()` = 随包 `config/litearm.yaml`，
其中 `installation.base_rpy: [0.0, 0.0, 0.0]`（正装），且**没有** `gravity:` 字段。

`gravity_from_installation`（`robot_model.py:11`）优先级：`gravity:[gx,gy,gz]` → 否则
`base_rpy` 变换 → 否则默认 `[0,0,-9.81]`。当前 base_rpy 全 0 → 重力按**正装**方向算。

`gravity_scale`（当前生效 `[2.7, 2.10, 1.75, 1.85, 2.9, 3.1, 1.2]`）只改每关节重力的
**幅度**，不改方向。真机是侧装（roll -90°、gy=+9.8）的话，方向要写
`gravity: [0, 9.81, 0]` 或把 `base_rpy` 写成 roll -90°，否则 G(q) 方向错 → 松手/持位
往错误方向下垂。

> 待确认：用户说的"重力补偿方向已改好"改的是哪个文件哪个字段。若只动了
> `gravity_scale`，方向其实还没改，高速测试会露馅。改对后要么直接改 `litearm.yaml`，
> 要么给 follow_server 加 `--config` 指向改好的文件。

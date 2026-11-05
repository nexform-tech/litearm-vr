# 把 `gripper_server.py` 的 open/close 改成「参照 litegrip-cpp / ros2_control 的控制环」

本文是这次重写方案的评估报告：**改什么、改多少、为什么这么改、怎么验收**。写给维护这套遥操的人。
落地的代码改动在同一个提交里（`litearm-vr-follow/gripper_server.py`）。

参照物（三处，同一套设计的三个层次）：

| 参照 | 位置 | 它在这一层负责什么 |
| --- | --- | --- |
| `litegrip::ControlLoop` | `litegrip-cpp/{include/litegrip/control_loop.hpp,src/control_loop.cpp}`（179+682 行） | **真正的每拍算法**：200 Hz 后台线程、轨迹限速、力矩预算分配、安全门、MIT 帧、看门狗/温度/故障 |
| `LitegripSystem` | `litegrip-ros2/litegrip_ros2_control/src/litegrip_system.cpp`（521 行） | ros2_control 薄壳：生命周期、单位换算、`write()` 只把「目标开度」投给环 |
| `CppLoopGripper` | `lerobot/litearm-lerobot/src/litearm_lerobot/gripper_cpp.py` | 同一套设计的 Python 版（20 Hz，经 C ABI），含夹取的重试/退让启发式 |

---

## 1. 参照实现的核心机制（每拍伪代码）

`control_loop.cpp::hardware_cycle()`（200 Hz，`dt` 用**实测** `elapsed`）：

```
poll 反馈帧 → 反馈看门狗（rx_count 不变 > feedback_timeout_s=0.5 → kNoFeedback）
温度 > 80℃ → 安全停        电机 error_code → 故障处理
target      = 上层 set_target_rad() 投进来的目标（非阻塞，只覆盖一个变量）
trajectory += clamp(target - trajectory, ± max_velocity_rad_s * elapsed)   # 轨迹限速
命令超时（> command_timeout_s=0.2 无新命令）→ 冻结 trajectory（★ 绝不设成实测位置）
kd = min(kd_ceiling, budget / max_feedback_velocity_rad_s)                 # 预算先给阻尼
kp = min(kp_ceiling, (budget - kd*v_b) / max_position_error_rad)           # 剩下的给位置
q_frame = measured + clamp(trajectory - measured, ± budget/kp)             # ★ 帧里最多要这么多力
护栏：越红线的帧**拒绝**（不钳位）；默认拒帧→改发 hold 帧（kHoldTorqueFraction=0.5）
control_mit(q_frame, kp, kd, dq=0, tau=0)                                  # dq 恒 0、tau 恒 0
sleep(period - 用时)
```

关键常量（`control_loop.hpp` + `litegrip_controllers.yaml` + lerobot 侧实际值）：

| 量 | C++ 默认 | lerobot `CppLoopGripper` 实际用 | 含义 |
| --- | --- | --- | --- |
| `control_rate_hz` | 200 | 200（Python 版 20） | 帧流周期 |
| `max_velocity_rad_s` | 1.5（launch 配置） | `max_rate_m_s=0.0975`（≈97 mm/s） | **命令轨迹**推进速度上限（不是 dq 字段） |
| `torque_limit_nm` | 3.5（硬上限） | 1.5 | 力/力矩预算：**夹持力就是它** |
| `kp` / `kd` 上限 | 20 / 0.5（`kp_max=200,kd_max=5`） | 8 / 0.5 | 环自己的增益上限，还会被预算再压 |
| `max_position_error_rad` | −1 → 取红线宽度 | 0.15 | 预算分配用的「最坏位置误差」 |
| `max_feedback_velocity_rad_s` | −1 → **拒发任何运动帧** | 0.5 | 预算分配用的「最坏反馈速度」 |
| `feedback_timeout_s` / `command_timeout_s` | 0.5 / 0.2 | 同 | 掉线 / 命令过期 |
| `temperature_limit_c` | 80 | 80 | 温度上限 |
| 红线 / 目标 | 红线 `[-1.24,-0.01]` rad，机械 `[-1.2728,0.0513]`，开度 0..87 mm | 行程 74 mm、红线内缩 3 mm | 目标钳进「可命令区间」，越红线拒帧 |

**最重要的一条**：上层只投「目标」。`write()` 每周期重投；`on_activate()` 先把目标锁存成**实测**开度
（否则 controller_manager 的初值 0.0 会被当成「全闭目标」）；`on_deactivate()` **保持实测位置、不 disable**
（disable 会掉件）。`set_target_rad()` 中途改目标只覆盖变量 —— 下一拍轨迹就以速度上限朝新目标走，
**≤5 ms 内换向**，没有任何「等这一片跑完」的延迟。

---

## 2. 为什么这套机制能根治我们现场那四个故障

| 现故障（见 `gripper-jitter-analysis.md`） | 现实现的机制 | 控制环为什么不会犯 |
| --- | --- | --- |
| 分片重跑闭合斜坡（§5.1：1 次 close → 每 0.75 s 一次 `grasp()`） | 阻塞式原子动作 + 为「可打断」而分片 | **没有动作、没有分片**：目标是个变量，帧流是连续的 |
| 空闲保活顶死 → ~1.2 s 松弛振荡（§4.3.1） | 保活=位置环，设定值取「最近一次读到的位置」→ 与实际位置有 ~0.7 mm 偏差 → `kp×Δ≈1 Nm` 持续压 | 帧里的 q 是**命令轨迹**（绝对目标），再被 `clamp(trajectory-measured, ±budget/kp)` 夹住：**任何时刻的力都 ≤ 预算**；到达目标后误差→0，力→0（不会顶死） |
| 松手不回弹（§4.3.2：`open()` 在第一个 0.25 s 窗口被判堵转收工） | SDK 的堵转判据 + 一次性动作 | 环**不判堵转、不放弃**：朝目标持续推进，被挡住就停在「力=预算」的平衡点；目标一改立刻反向 |
| 夹持力不可控（`grasp` 的纯力矩保力，工件让位就掉力） | 纯力矩前馈 + 分片重启爬升 | 夹持力 = `kp×(trajectory−measured)`，被 `budget/kp` 的钳位封顶 ⇒ **工件让位时误差变大、力保持**；这就是「设定力夹取」，不需要第二套模式 |

另外顺带解决的：**掉线不再静默**（环持续发帧，故障时发零力矩帧而不是停流，避开 0xD 通信丢失自锁）、
温度/电机故障有统一出口（fault code）、松手延迟从「最多一个分片 0.85 s」降到 **≤5 ms**。

---

## 3. 改动量评估

### 3.1 代码

| 文件 | 改动 | 规模 |
| --- | --- | --- |
| `litearm-vr-follow/gripper_server.py` | **重写 `LiteGripperControl` 的内核**：删掉阻塞动作分派（`_loop` 里的 open/close 分支、`_grasp_until_released`、`_open_with_retry`、`_keepalive`），换成 200 Hz 控制环（轨迹限速、预算分配、帧 q 钳位、看门狗/温度/故障、命令超时冻结） | 删 ~170 行 / 增 ~200 行；**净增 ~30 行**，文件 620 → ~650 行 |
| 同上：`close()` / `open()` / `snapshot()` / `read_now()` / `last_cmd_ratio()` / `GripperServer` / TCP 协议 | **不动**（`close`→投闭合目标、`open`→投张开目标，语义与协议不变） | 0 |
| 同上：CLI | 新增 `--rate-hz/--kp/--kd/--max-pos-error/--max-vel...`；`--speed` 复用为「轨迹速度上限」；`--hold-s` 删除（不再有分片）；`--force` 复用为「力矩预算」（`force_n × 0.1 Nm`） | ~8 个参数 |
| `litearm-vr-follow/test_gripper_shards.py` | **重写成 `test_gripper_loop.py`**：假 SDK + 真实环，断言限速推进、力被预算钳住、目标中途改向 ≤1 拍、停帧→冻结、故障→零力矩帧、未使能不运动 | 删 ~330 行 / 增 ~260 行 |
| `litearm-vr-follow/README.md` | 夹爪服务段改写成控制环语义 + 新参数表 | ~30 行 |
| `litearm-vr-follow/docs/gripper-jitter-analysis.md` | 加一段「最终形态：控制环」+ 指向本文 | ~10 行 |
| `litearm-vr-teleop`（上游遥操） | **完全不用动**（TCP 协议 `close/open/status/ping` 不变） | 0 |
| `litegrip-python` SDK | **不用动**（只用到 `send_mit_frame` 这一个公开入口） | 0 |

### 3.2 风险与取舍

- **增益/预算要重新标定**：环的 kp 上限远小于 SDK 的 `kp=100`（C++ 20、lerobot 实际 8），
  力由预算（`--force`）定。首次上真机建议保守：`--force 10`、`--kp 20`、`--kd 0.5`、`--speed 30`，
  再按手感加。所有量都在 CLI 上，不用改代码。
- **空闲不再「抱死」**：夹爪停在目标处、力≈0，靠机构弹簧/摩擦维持（C++ 的 `safe_stop` 注释写着
  这台夹爪「零力矩 → 弹簧把爪张开」）。好处是不会有顶死振荡；代价是空闲时位置可能被外力推动。
- **`max_feedback_velocity_rad_s` 必须给**（C++ 里不给就拒发任何运动帧，fail-closed）：
  默认取 SDK 的 `MotionConfig.velocity_limit`（1.0 rad/s），不够再调。
- **红线靠标定推导**（这套栈没有 C++ 那个版本化安全基线 JSON）：用 SDK 标定的
  `pos_closed_rad/pos_open_rad` 内缩 `--red-margin-mm`（lerobot 用 3 mm）作为可命令区间，
  **不移植**红线拒帧 + hold 帧那套机器（我们没有基线文件，且它的「顶过接触点」行为与我们要修的
  抖动是同一类风险）。
- **可回退**：当前 `gripper_server.py` 有未提交改动，重写前先备份到 `/tmp/gripper_server.legacy.py`，
  需要时 `git checkout` 或直接用备份。


### 3.3 不移植的部分（明确边界）

| 不移植 | 理由 |
| --- | --- |
| 版本化安全基线 JSON（`safety_baseline`、`SafetyLimits`、红线拒帧 + `stream_hold_frame`） | 这套栈没有这份文件；「拒帧并朝接触点再推」正是我们要避免的行为。改用「轨迹钳进可命令区间」 |
| `dry_run` / `hardware_enable` 双开关、estop 请求位、`enable_request` 标志 | 本服务的启停由 `run_gripper_server.sh` + TCP 命令表达，不需要再加一层 |
| lerobot `CppLoopGripper` 的夹取启发式（`grip_torque_fraction` 退让、`stall_epsilon_mm` 失速判定、`approach_step_mm` 渐进、`recovery` 重连节奏） | 那是在 20 Hz Python + C ABI 下的补丁；200 Hz 环里「力=预算」已经给出了同样的结果。等实测需要再加 |
| 开合比例 0..1 作为命令面 | 本服务对上已经是 `close/open` 两个离散命令 + `status` 里的 `ratio/target_ratio`，保持契约不变 |

---

## 4. 实施与验收

1. 备份旧实现 → 写新环 → `py_compile` → 新单测（假 SDK，不需要硬件）→ 跑 `test_status_msgs.py` 确认契约未破。
2. 现场验收（探针 `probe_gripper.py` 只读复测，判据写死在命令里）：
   - 静止空闲：`cmd` 帧应为 `kp=预算分配值`、`tau=0`、`dq=0`，**且 `q` 停在目标上不动**；实测 tau 在预算内、无周期振荡；
   - 按住：`q` 以 `--speed` 限速推进到闭合侧，撞到限位/工件后 `|st.tau|` 稳定在预算附近（≈`--force×0.1` Nm），**力不随时间衰减**；
   - 松开：目标换到张开侧，`q` **在 ≤2 拍内反向**（CAN 上能看到 dq 符号翻转，不需要等一个分片）；
   - 无 `grasp #`/`续保力 #`/`open #n/3` 这类分片日志（它们随旧实现一起删掉）。
3. 复测通过后，把两处工作区改动一起提交（用户决定是否提交）。

---

## 5. 实施结果（与第 3 节评估的差异）

已按上面第 4 节落地。实际改动与评估基本一致，一处规模差异、若干实现取舍记录如下。

| 项 | 评估 | 实际 |
| --- | --- | --- |
| `gripper_server.py` | 删 ~170 / 增 ~200，净增 ~30 行 | 663 → **672 行**（净 +9）：删掉 `_loop` 的动作分派、`_keepalive`、`_grasp_until_released`、`_open_with_retry`、`_idle_refresh`、`_ensure_ready`（共 ~250 行），换成 `_cycle` + 6 个算式/工具方法（~180 行） |
| 测试 | 重写为 `test_gripper_loop.py` | 新增 **`test_gripper_loop.py`（400 行，10 个用例）**；旧的 `test_gripper_shards.py`（322 行）测的是已删除的分片/保力/重试行为，**已失效**，暂留待决定去留 |
| 契约测试 | 不变 | `test_status_msgs.py` 仍通过（回包只增键不改键） |
| CLI | ~8 个新参数 | 新增 `--rate-hz/--kp/--kd/--max-pos-error-mm/--max-fb-vel/--feedback-timeout/--temp-limit`；`--speed`/`--force` 语义改为「轨迹速度上限」/「力矩预算」；**`--hold-s` 删除**（不再有分片） |
| 上游 teleop | 0 | 0（协议未动） |
| SDK | 0 | 0（只用了公开的 `send_mit_frame`） |

实现时明确**没有**照搬的两处，都在代码注释里写明了理由：

1. **不实现命令超时冻结**（C++ 的 `command_timeout_s=0.2` → 冻结 trajectory）。C++ 那份是因为
   ros2_control 的上层**每周期重投**目标，超时代表「上层死了」；本服务的 `close`/`open` 是
   **离散且粘住**的命令，冻结反而会让轨迹停在半路永远到不了目标。
2. **不实现红线拒帧 + `stream_hold_frame`**。我们没有版本化安全基线，且「拒帧后朝接触点再推」
   与本次要修的抖动是同一类风险。改为：目标只取标定限位、帧内位置被 `预算/kp` 钳住 ——
   于是**帧里要求的力矩天然≤预算**，不需要靠拒帧兜底。
   ⚠ 副作用（有意接受）：夹爪被工件挡住时，帧里的 `q` 会落在标定行程之外（仍是有限力，
   ≤预算）。C++ 因为有红线会拒掉这种帧；我们的安全性质是「力有界」而不是「q 在行程内」。

**未做硬件验证**：以上全部由假 SDK 单测覆盖（`python3 test_gripper_loop.py` → 10/10 通过）。
现场验收需要重启 `gripper_server.py` 后用 `probe_gripper.py` 复测（判据见第 4 节第 2 步），
**在此之前不要把这一版当已验证**。

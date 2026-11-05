# 夹爪抖动：一次 `close` 到底调了几次 `grasp()`？

本文是「头显扳机 → 夹爪 DM 电机」这条链的抖动排查记录与修复说明，写给维护遥操/夹爪的人：
**为什么会抖、抖动在代码里的确切位置、怎么改、怎么验证**。结论对应的补丁就在本仓
（`gripper_server.py`）与上游 `litearm-vr-teleop`（`ws_echo_server.py` 的日志）。

排查对象：`gripper_server.py`（本仓）+ `litearm-vr-teleop/scripts/ws_echo_server.py` +
`litegrip-python` SDK 的 `actions.grasp()`。分析方法：读码 + 用假 SDK 替身数调用次数 +
`can0` 只读抓包解码。**没有**改动现场正在跑的机器人进程。

---

## 1. 结论

1. **是多次调用，但不在 teleop 侧。** 头显按一次扳机 → teleop 只发**一次** `{"cmd":"close"}`
   （边沿触发 + 50ms 消抖），`LiteGripperControl.close()` 也只写一次槽位。多次发生在
   **worker 执行层**：`_grasp_until_released()` 的分片循环**每片重跑一整段 SDK `grasp()`**，
   实测间隔 **0.751 s（1.33 Hz）**，一直持续到松手。
2. **抖动在分片边界**：同一只夹爪被两种控制律轮流接管 —— 位置环 + 速度前馈（约 1.8 Nm）
   ⇄ 纯力矩保力（2.0 Nm）。力是方的，且纯力矩段没有位置项，工件让位后下一片的斜坡再把它
   顶回来，物理上就是**一松一紧地抖**。
3. **现场日志看不见这个多次**：`_grasp_until_released` 用 `first` 标志**只打第一片**的日志，
   teleop 侧 `close`/`open` 一行都不打。所以日志里永远只有一条 `[夹爪] grasp -> …`，
   给人「只调了一次」的错觉。**补丁 2 就是先让这个现象可见。**

如果你的直觉是「调用一次 close/open/grasp 即可」——在**意图层**完全正确，本文的多次发生在
**执行层**，是上一版为了「松手能立刻排上」引入的分片设计带来的副作用（见 §6）。

4. **分片不是抖动的全部。** 现场探针（§4.3）另外抓到两条独立故障，修好分片之后它们
   依然在：① **空闲保活**用配置增益（kp=100）跑位置环，设定值取不到同一时刻的实测位置，
   于是变成一股 ~1 Nm 的持续闭合力，把夹爪顶死在闭合限位上形成 **~1.2 s 松弛振荡**；
   ② **`open()`** 在第一个 0.25 s 采样窗口就被 SDK 判堵转收工 → **松手不回弹**。
   两条都在本仓修掉（§5.2 / §5.3），**不需要改 SDK**。

---

## 2. 调用链与逐环调用次数

```
头显（~90Hz 帧）──ws 8765──▶ TeleopBridge.handle_frame()
                              │  trigger 上升/下降沿（消抖 50ms、最小闭合 0.5s）
                              ▼  1 次 / 每次扳机
                         GripperRemoteClient.close() / .open()   ──TCP 9003──▶
                              ▼  1 次
                    GripperServer.handle() → LiteGripperControl.close()
                              ▼  只写槽位 _target（非阻塞，TCP 立刻回执）
                    worker 线程 _grasp_until_released()
                              ▼  ★ 循环 N 次 —— 抖动的来源
         SDK LiteGrip.grasp(force, hold_s) = _move_to_limit(闭合段) + _hold_force(保力段)
                              ▼  200Hz MIT 帧
                           can0 → 夹爪 DM4310（CAN ID 0x08）
```

| 环节 | 位置 | 一次扳机调用几次 |
| --- | --- | --- |
| 触发边沿判定 | `ws_echo_server.py` `handle_frame()` | **1**（50ms 消抖 + 0.5s 最小闭合） |
| TCP 下发 | `GripperRemoteClient._send()` | **1**（fire-and-forget，无回执、无重发） |
| 收命令 | `LiteGripperControl.close()/open()` | **1**（仅写 `_target`） |
| 执行 | `_grasp_until_released()` | **close：N 次**动作调用；**open：1 次** |
| SDK | `actions.grasp()` | 每片 1 次 `_move_to_limit` + `_hold_force` |

### 单片时长（`--hold-s 0.5` 默认值）

| 段 | 时长 | 依据（SDK） |
| --- | --- | --- |
| 闭合段到「判堵转」 | ≈ 0.25 s | `stall_cycles=5 × sample_interval=0.05` |
| 保力段 | ≈ 0.6 s | `hold_s=0.5` 按 `hold_interval=0.2` 切片，deadline 跨片检查 |
| **合计** | **≈ 0.75 s → 1.33 Hz** | 与实测 0.751 s 吻合 |

---

## 3. 实测证据

### 3.1 数调用次数：一次 `close` → 6 次 `grasp()`（4 秒内）

用假 SDK 替身（不发任何 CAN 帧、只记调用时刻）直接跑 `gripper_server.py` 的真实代码：

```bash
# 等价于测试里的 test_gripper_shards.py（假 SDK + 真实 worker 线程）
python3 test_gripper_shards.py
```

修复前的输出：

```
空闲 1s 内的动作调用： []
[夹爪] grasp -> ok=True reached=False stalled=True q=+0.0000（stalled 且未 reached = 夹到工件，正常）
  t=+0.022s  grasp()
  t=+0.773s  grasp()
  t=+1.524s  grasp()
  t=+2.275s  grasp()
  t=+3.025s  grasp()
  t=+3.776s  grasp()
  t=+4.529s  open()
4s 按住 trigger 期间 grasp() 被调用 6 次
  相邻间隔(s): [0.751, 0.751, 0.751, 0.75, 0.751] → 平均周期 0.751s（约 1.33 Hz）
```

注意日志只有**一行** —— 6 次调用里 5 次是静默的（`first` 标志）。

### 3.2 现场抓包：确认总线上只有这一个 master

`can0` 只读采样 6 s（`SOCK_RAW` + `bind`，不发送）：

```
id=0x008 n=120 / 6s (20 Hz)   命令帧
id=0x018 n=119 / 6s (20 Hz)   电机反馈帧

0x08 解码: q=-0.0010 rad  dq=-0.0073 rad/s  kp=100.0  kd=2.00  tau=-0.0024 Nm
0x18 解码: err=1(已使能) can_id=8  q=-0.0010 rad  tau=-0.022 Nm  T_mos=33  T_coil=31
```

这条 20 Hz 帧正是空闲保活 `_keepalive()`（`send_mit_frame(q=_q, kp=cfg.kp=100, kd=cfg.kd=2.0)`，
`STATUS_HZ=20`）→ 采样时是**空闲保持态**（已使能、停在闭合位）。

`/proc/net/can/rcvlist_fil` 里**只有 1 个** can0 socket（`can_id=0x018 mask=0x7ff`，
`matches=16008`；16008 ÷ 20 Hz ≈ 800 s ≈ 该服务运行时长）⇒ **总线上只有一个 master**，
不存在「两个进程抢同一只夹爪」。顺带排除：兄弟仓 `litearm-collect-zenoh` 的
`collect_teleop_ui.py --gripper-channel can0` 只经 TCP 读快照、不打开硬件（其 docstring 与
CAN socket 计数都证实了这一点）。

解码命令（复现用；`protocol.py` 是纯函数，不需要 CAN）：

```python
import importlib.util, sys
p = "<litegrip-python>/src/litegrip/can/protocol.py"
spec = importlib.util.spec_from_file_location("dmp", p)
m = importlib.util.module_from_spec(spec); sys.modules["dmp"] = m; spec.loader.exec_module(m)
d = bytes.fromhex("7ffd7ff3336667ff")          # 抓到的 0x08 payload
q  = m.uint_to_float((d[0] << 8) | d[1], -12.5, 12.5, 16)
kp = m.uint_to_float(((d[3] & 0xF) << 8) | d[4], 0.0, 500.0, 12)
kd = m.uint_to_float((d[5] << 4) | (d[6] >> 4), 0.0, 5.0, 12)
print(q, kp, kd)                                # -0.0010 100.0 2.0
```

---

## 4. 为什么这就是抖动

分片边界上，控制律与力都在跳：

| 阶段 | 帧里的 kp / kd / τ | 闭合力 |
| --- | --- | --- |
| 闭合段 `_move_to_limit` | kp=100，kd 被力预算压到 ≈1.69，速度前馈 dq≈0.82 rad/s | 预算 `20 N × 0.1 × 0.9 = 1.8 Nm` |
| 保力段 `_hold_force` | kp=0、kd=0，纯前馈 τ | **2.0 Nm**（= 20 N × 0.1） |

1. **力方波**：每片边界力在 1.8 ↔ 2.0 Nm 跳一次（1.33 Hz）。SDK 自己在保力段刻意让力矩从
   飞行值按 20 N/s 爬升以求连续，**而分片把这段连续爬升切成了 0.75 s 一次的锯齿**。
2. **纯力矩段保持不住位置**：SDK 的 `_hold_force` 文档明说「零增益下夹爪可以被外力推动」。
   工件弹性回弹、或闭合侧约 0.010 rad 的粘滑死区一动，夹爪就被顶开一点；**下一片闭合段又把
   指令位置顶回去**（`cmd = pos + lead_cap`）→ 一松一紧。
3. **最坏的坏分支**：某片闭合段若「既没到位也没堵转」（软工件 / 结构缓慢让位），SDK 会走
   `_release()`——**连发 0.2 s 零增益帧，夹持力归零**——返回 `ok=False, cycles=0`。而
   `_grasp_until_released` 不看返回值，立刻再调一次 `grasp()`：工件掉一下、再被重夹。
4. **反向延迟**：松手后要等当前分片跑完（0.75~0.85 s；极端情况下闭合段跑满帧数上界可到
   4.3 s）才轮到 `open()` —— 手感是「松了手还在夹，然后猛地张开」。

另注：空闲时是**第三种**控制律（`_keepalive` 的 kp=100/kd=2 位置保持，与 grasp 的纯力矩保力
不是一回事），一次夹取过程中的边界上其实有 3 种律在切。

---

### 4.3 现场探针实测：另外两个根因（**这才是「闭合后周期抖动 + 松开不回弹」**）

用 `probe_gripper.py`（只读：`can0` 原始帧 + 服务端 `status` 快照）录一段人手操作，
再按「命令 vs 实测」对齐，看到的是**两条独立故障**，与 §4 的分片问题无关 ——
分片修好之后它们依然在。

### 4.3.1 空闲保活 = 一路「设定位取不到实测」的位置环 → ~1.2 s 松弛振荡

操作者**已经松手**（意图 open、`target_ratio=1.0`、`_done="open"`），总线上的命令帧是
20 Hz 的保活帧（`q = 服务端缓存位置, kp=100, kd=2, tau=0`）。把 `svc.q`（服务端缓存）
和 `st.q`（同一条总线帧里电机实际报的位置）并排看：

| t (s) | cmd.q 命令位置 | svc.q 服务端缓存 | st.q 总线实测 | Δ = svc−st | st.tau 实测力矩 |
| --- | --- | --- | --- | --- | --- |
| 2.755 | +0.0608 | +0.0608 | +0.0521 | **+0.0087** | +0.711 Nm |
| 3.055 | +0.0704 | +0.0704 | +0.0566 | **+0.0138** | +0.838 Nm |
| 3.206 | +0.0814 | +0.0795 | +0.0696 | **+0.0099** | +0.926 Nm |
| 3.256 | +0.0830 | +0.0830 | +0.0715 | **+0.0115** | +0.940 Nm |

- 服务端缓存的「位置」比电机**同一时刻实际报的**位置**系统性高 ~0.011 rad ≈ 0.7 mm**；
- 保活帧是位置环，于是 `kp × Δ ≈ 0.8~1.2 Nm` 的**持续闭合力**（对上表实测 tau 一一吻合）；
- 夹爪顶在闭合硬限位上，顶到粘滑释放回弹、再被顶回去 →
  **~1.2 s 周期、幅度 ~6 mm 的来回抖动**（实测实测位置 q 在 −0.03 ↔ +0.10 rad 之间循环）；
- 全程**没有任何新指令**（`target_ratio` 一直是 1.0）——抖动由这一路位置环自己产生。

> ⚠ 这条修正了 §2 表里的一个说法：空闲的 `_keepalive` 并非「无害地保持当前位置」，
> 它带着配置增益（kp=100）在跑，是一个**力源**。
> SDK 侧「缓存位置为何比总线帧高一点」还没定到根（`poll()/update_state()` 的缓存语义），
> 但修复不依赖它：**保活帧不带增益就没有任何力**（MIT 律里 kp=kd=τ=0 就是「别管我」）。

### 4.3.2 `open()` 在第一个 0.25s 采样窗口就被判堵转 → 松手不回弹

松手后 `open()` 确实跑了，但只跑了 **50 帧（254 ms）就收工**：

```
   2.3435 cmd  位置斜坡 q=+0.0227 dq=-0.681 kp=100 kd=2 tau=0    ← open 起点
   ...
   2.5980 cmd  位置斜坡 q=-0.0006 dq=-0.681 ...                  ← 第 50 帧，之后转 20Hz 保活
   实测：q 从 +0.1055（被保力顶进限位 4.8mm）走到 -0.0265，只动了 ~2mm 的净行程
```

`_move_to_limit(press=True)` 的堵转判据要求**第一个 0.25 s 窗口**里净位移
≥ `0.2 × 指令速度 × 0.25 s`（出厂 ≈0.034 rad ≈1.9 mm）。夹爪刚从保力段（1.48 Nm 顶在
限位上）出来，第一口气大半花在顶开残余压紧量上 → 判据判定「堵转」→ `ok=False`。
而服务端当时是「无论 ok 与否都标记完成」，于是**不再尝试**：意图 open、实测停在
闭合侧，随后被 §4.3.1 的保活位置环继续顶死 —— 现场就是「松开后不回弹」。

⚠ 不能改回「不 ok 就不标记完成」：那正是上一版的紧循环重跑（§5.1 的同类问题）。
正确做法是**有界重试**（见 §5.3）。

---

## 5. 修复

### 5.1 分片只续保力，不重跑闭合斜坡（根治）

> ⚠ **§5.1 / §5.2 / §5.3 是过渡修法（在旧架构里打补丁），已被取代**：现在整个
> open/close 已按 `litegrip-cpp` 的 `ControlLoop` 重写成一条 ~200Hz 控制环 ——
> 没有分片、没有保活位置环、没有一次性 `open()`，所以这三处补丁连同它们要修的问题
> 一起不存在了。重写方案与改动量见 [`gripper-control-loop-rewrite.md`](gripper-control-loop-rewrite.md)。
> 下面三小节保留，是为了记录「症状 → 机制 → 当时的修法」这条链，改这一块前值得读一遍。

`_grasp_until_released()` 里：**第一片**跑完整 `grasp()`（闭合段 + 保力段），**第 2..N 片**
只发保力帧（续保力）。分片的目的只是「松手能立刻排上」，续保力同样满足 —— 保力帧本来就
只是「再发 hold_s 秒的纯力矩」。

交接条件照抄 SDK：**只有闭合段 `reached or stalled` 才算真的交给了保力段**；没交接成功时
（SDK 已撤力）下一片必须重跑整段。

实现上借用 SDK 的 `actions._hold_force()`：上游**没有公开的 hold 入口**
（`GripperActions` 只有 open / close / grasp / ramp_to / zero / enable / disable）。
⚠ 这是个**私有名**，故代码里做了一次性检查：取不到就**大声**退回旧行为（每片重跑整段），
而不是静默炸掉 worker 线程 —— 本文件已经吃过两次 SDK 版本漂移的亏（`st.fresh` 不存在、
`refresh_status()` 不存在，后者被 `except` 静静吞掉、保活从来没生效过）。
`test_gripper_shards.py` 里有一条用例专门钉这个私有入口仍然存在。

> 长期方案：在上游 SDK 给 `GripperActions` 加一个公开的 `hold(force_n, hold_s)`（只发保力帧，
> 不做闭合段），本仓改成调它。在那之前，本仓的用法与测试是这条私有名的唯一依赖点。

### 5.2 空闲保活改成零增益（去掉 §4.3.1 的力源）

`_keepalive()` 从「`send_mit_frame(q=_q, kp=config.kp=100, kd=config.kd=2)`」改成
**`send_mit_frame(q=_q, 0, 0)`**：帧照发（防 ~100 ms 通信丢失自锁），但**不产生任何力**。

- 为什么原来那组增益有害：位置环的设定值只能取「最近一次读到的位置」，而它**不可能是
  同一时刻的实测值**；只要有一点偏差（实测 ~0.7 mm），`kp × Δ` 就是一牛·米级的持续力。
- 为什么不改用「配置里的目标端位置」当设定值：夹爪够不到目标时（例如上一动作失败停在
  另一端），误差会是整段行程 × kp → 直接顶到 10 Nm。零增益是唯一不会自己出力的选择。
- 代价：空闲时夹爪是**自由**的（手可以推动）。这不影响功能 —— 扳机按住期间走的是 grasp
  分片（纯力矩保力），空闲期本来就没有东西需要夹。

### 5.3 `open` 改成有界重试（修 §4.3.2 的「不回弹」）

`open` 分派改成 `_open_with_retry()`：最多 `OPEN_TRIES=3` 次，每次失败之间静 `OPEN_RETRY_S=0.3` s
（让残余力矩衰减，否则第二次还在同一个采样窗口里被判堵转），每次尝试都打一行日志，
失败到底就如实报「连续 N 次没到位」并照旧标记完成。

⚠ **必须有界**：无界 = 上一版的紧循环重跑（一次接一次全速斜坡互相打架）。

### 5.4 让现象可见（日志）

- **本仓** `_grasp_until_released()`：**每片**都打一行（带序号、与上一片的间隔 dt、
  `ok/reached/stalled/cycles/q`），结束时打「共 N 片」。
- **teleop 仓** `ws_echo_server.py`：`close` / `open` 各打一行（带 `side`、原始 `trigger` 值、
  闭合保持了多久）。带上 `trigger` 原值是为了顺手排查另一条待查项：阈值无滞回
  （`> 0.5` 即按下，50ms 消抖），手柄停在 0.5 附近反复跨越阈值时也会反复 close/open。

### 5.5 一并修掉的文档漂移

README 的「夹爪服务」段仍写着 `close` 是「直接透传 SDK 的 `close(speed)`…不做保力」，
这是 `close` 改用 `grasp()` 之前的话，已随本次补丁更正。

### 5.6 改动清单

| 仓 / 文件 | 改动 |
| --- | --- |
| `litearm-vr-follow/gripper_server.py` | ①分片只续保力（§5.1）②保活零增益（§5.2）③open 有界重试（§5.3）④逐片日志（§5.4）⑤顶部陈旧说明同步（§5.5） |
| `litearm-vr-follow/test_gripper_shards.py` | 新增回归测试（纯 stdlib，假 SDK 替身） |
| `litearm-vr-follow/README.md` | 夹爪服务段更正（`close` 早已是 `grasp()`）+ 指向本文 |
| `litearm-vr-teleop/teleop/scripts/ws_echo_server.py` | `close` / `open` 各打一行（带 `trigger` 原值） |
| `/tmp/gripper_probe/probe_gripper.py`、`analyze_gripper.py` | 现场只读探针与分析器（**还没入仓**，需要就挪进本仓） |

---

## 6. 这个坑换过五次实现（改前先读）

| 实现 | 现象 |
| --- | --- |
| 自写「纯力矩保力 + open 打断」 | 空爪时纯力矩把夹爪一路挤进闭合硬限位、越挤越死；随后 `open` 第一段脱开位移太短，被 SDK 堵转判据误判成堵转 → **来回寸动** |
| 改用 SDK `close(speed)` / `open(speed)` | 位置环直推，夹到东西也继续往 0 顶 → **压坏工件** |
| `close` 改用 `grasp()` + 分片（上一版） | 分片重跑闭合斜坡 → **分片边界一松一紧**（§5.1 修） |
| 空闲保活用配置增益（kp=100）保持位置 | 设定值取不到同一时刻的实测位置 → 持续 ~1 Nm 顶死 → **~1.2 s 周期抖动**（§5.2 修） |
| `open` 失败就标记完成、不重试 | SDK 第一个 0.25 s 窗口判堵转 → **松手不回弹**（§5.3 修） |

后两条是同一次现场排查（探针实测）抓出来的，**与分片无关**；`open` 那条尤其注意：
既不能「不重试」，也不能「无界重试」（后者是紧循环互相打架）。

五条的根因各不相同，**不要**为了让某一条现象消失就把上一条改回去。

---

## 7. 怎么验证

1. **离线（不需要硬件）**：`python3 test_gripper_shards.py` —— 断言一次 `close` 只调 **1 次**
   `grasp()`、之后全是续保力片；`open` 只调 1 次。
2. **现场日志**：按住扳机，`gripper_server` 应打印

   ```
   [夹爪] grasp #1（+0.000s） -> ok=True reached=False stalled=True q=+0.0000（stalled 且未 reached = 夹到工件，正常）
   [夹爪] 续保力 #2（+0.752s） ok=True cycles=3 q=+0.0000
   [夹爪] 续保力 #3（+0.751s） ok=True cycles=3 q=+0.0000
   ...
   [夹爪] 分片结束：共 N 片
   ```

   修复前只有第一行；`续保力 #2` 之后**不应**再出现 `grasp #`。
   teleop 侧应看到每次扳机各一行 `close` / `open`；若一行变很多行，说明扳机在阈值附近抖动
   （看行里的 `trigger` 值），那是另一条待办（加滞回）。
3. **CAN 抓包**：`candump can0`（或本文 §3.2 的只读脚本）。夹住物体时，修复前能看到周期
   ~0.75 s 的「位置帧段（kp=100/kd≈1.7）+ 纯力矩段（kp=kd=0/τ=2）」交替；修复后应是一段
   连续的纯力矩帧。
4. **上游遥操侧（本仓之外）**：把 `time.monotonic` 打桩成虚拟时钟、用假的 sim/gripper 驱动
   真实的 `TeleopBridge.handle_frame`，喂「按住扳机 180 帧 + 松开 180 帧」⇒ 断言**只有**
   一次 `close`、一次 `open`（实测：`[('close', …), ('open', …)]`）。这条同时也是 §1 第 1 条
   结论（重复不在 teleop 侧）的证据。
5. **空闲不该有力（现场）**：松开扳机、什么都不按，`/tmp/gripper_probe/probe_gripper.py`
   录 20 s，应看到 `cmd` 帧 `kp=0 kd=0 tau=0`（20 Hz）、`st.tau` 在 ±0.05 Nm 内、
   `st.q` 不再来回走（修复前：`kp=100`、`tau` 反复爬到 1 Nm、q 以 ~1.2 s 周期摆 6 mm）。
6. **松手必回弹（现场）**：按一下再松开，`gripper_server` 应打印
   `open #1/3 -> ok=True`（一次成功），实测位置回到张开限位（q ≈ −1.06 rad、ratio≈0.93）。
   若打印 `#2/3`、`#3/3` 才成功，说明残余压紧量仍然偏大（可以把 `OPEN_RETRY_S` 调大一点）。

⚠ 改动只对**重启后**的 `gripper_server.py` 生效；正在跑的进程仍是旧代码。

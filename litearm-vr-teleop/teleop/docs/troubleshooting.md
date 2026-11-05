# 排障手册（沉淀实战经验）

## 环境安装

| 问题 | 原因 | 解法 |
|---|---|---|
| conda 装包 HTTP 000 | 换用的镜像已失效（USTC anaconda 镜像已停用、现 302 跳转） | 换回清华镜像：`--override-channels -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r`（`setup.sh` 默认已用它） |
| GitHub 克隆慢 | 直连被限速 | `https://ghfast.top/https://github.com/<org>/<repo>/archive/refs/heads/<branch>.zip`（注意 LIBERO 默认分支是 master） |
| HuggingFace 全超时 | 域名被墙 | 绕道：assets 从 GitHub 提取，不用 HF 自动下载 |
| `pip install -e .` 后 import 不到 | `find_packages()` 不会把 setup.py 同级目录当包 | 显式 `packages=["xxx"]` + `package_dir={"xxx": "."}`，或补 `__init__.py` 放子目录 |
| pybullet 与 numpy ABI 不匹配 | pybullet 3.2.6 是 numpy 1.x 时代构建 | 用 `pybullet==3.2.7`（numpy 2 时代构建，与 numpy 2.2 兼容） |

## pybullet 仿真

| 问题 | 原因 | 解法 |
|---|---|---|
| 无头环境弹不出 3D 窗口 | GUI 连接依赖 DISPLAY | `LITEARM_NO_VIEWER=1`（或 `--no-viewer`）降级 DIRECT 模式 |
| FK 位姿差 5cm+ | `getLinkState` 的 `[0][1]` 是质心帧（COM），不是 link 原点帧 | 用 `[4][5]`（URDF link 帧，与旧 pylitearm ee_frame 帧同构） |
| IK 解在 7-DOF 冗余间跳变 | 冗余臂多解，种子不连续 | `restPoses=当前 q` + `jointDamping` 拉 null space + `sim.max_step_rad` 限幅（已内置） |
| `createMultiBody` 报 invalid keyword | pybullet 3.2.7 无 `visualShapes` 别名 | 用 `baseVisualShapeIndex` / `linkVisualShapeIndices` |

## RViz2 显示（三坑，实战定位）

| 问题 | 原因 | 解法 |
|---|---|---|
| `robot_state_publisher` 启动即 crash（`Failed to parse global arguments`） | SolidWorks 导出 URDF 是 CRLF 多行 XML，`-p robot_description:="$(cat URDF)"` 传参被 rcl 参数解析器拒绝 | 改用 `--params-file` + YAML（json 转义包裹 URDF，自动去 CRLF） |
| RobotModel 报红（`Error retrieving file [.../xxx.STL]`） | RViz 的 assimp 不认裸绝对路径 mesh | URDF 里 mesh 用 `file://` 前缀：`sed "s|package://litearm_description/meshes/|file:///abs/path/meshes/|g"` |
| 单臂模式 RViz 空白 | run 脚本单臂分支误用双臂 rviz config（Fixed Frame=`column_base_link`，单臂根是 `base_link`） | 单臂用 `models/litearm_description/rviz/display.rviz`（Fixed Frame=`base_link`） |
| Global Status: Warn（Fixed Frame does not exist） | 没有 `robot_state_publisher` 发 TF（通常是上面 crash 导致） | 先修 crash；确认 `ros2 topic info /tf` 的 Publisher count=1 |

## 真机连接（串口路 / litearm SDK）

> 权威脚本在仓库根：`run_stm32_comm_test.sh` / `run_stm32_server.sh`。
> 这里只记**排查口径**，不重复它们的用法。

### 两条路，本机只有一条能跑

| 路 | 脚本 | 串口层依赖 | 本机 |
|---|---|---|---|
| 串口（控制律在 STM32 固件） | `arm_follow_server_stm32.py` | **`litearm` SDK**（[litearm-python](https://github.com/nexform-tech/litearm-python)，只依赖 pyserial） | ✅ 可用 |
| CAN（STM32 只当 CAN 桥） | `arm_follow_server_sdk.py` | `pylitearm` | ❌ **源码在本机不存在** |

判 `pylitearm` 是否可用：`python3 -c "from pylitearm import Arm, ArmState"`。
本机失败的原因是 `~/.local/lib/python3.10/site-packages/__editable__.pylitearm-5.0.0.pth`
指向 `/home/yudao/wkspace/pylitearm/src`，而那个目录**已被删除**（悬空 editable）。
⇒ 用这条路的唯一办法是拿到 `pylitearm` 源码重新 `pip install -e`。

判 `litearm` 是否可用：`python3 -c "import litearm; print(litearm.__version__)"`。
本机已 editable 装给系统 `python3`（`__editable__.litearm_python-2.1.0.pth`），无需安装。

### 启动顺序（**务必按这个顺序**）

```bash
cd /home/yudao/wkspace/vr
./run_stm32_comm_test.sh                              # ① 只被动读状态 + 查固件，不动臂
./run_stm32_server.sh --mode jf --tcp-port 9002       # ② 起 follow server（此时仍不动臂）
# 另一个终端：
cd teleop && ./launch/run_teleop_webxr.py --real --real-right 127.0.0.1:9002   # ③ 起 teleop
```

| 问题 | 原因 | 解法 |
|---|---|---|
| **臂不动，但日志一切正常** | 单臂/双臂的端口对应不一致：单臂**正装**只往 `right` 发（默认 `127.0.0.1:9002`），单臂**侧装**只往 `left` 发（默认 `127.0.0.1:9001`），而 follow server 的默认端口是 9001 | 三选一：server 用 `--tcp-port 9002`；或给 metavr `--real-right 127.0.0.1:9001`；**或（推荐）写进配置** `config/teleop.yaml` 的 `real.right_addr` / `real.left_addr`，省得每次敲 |
| 连不上真机，日志刷 `[真机][right] 连接失败 … Connection refused` | 该端口上**没有进程在监听**（`Errno 111` = 机器可达但端口空着，不是网络不通），即 follow server 没起或端口对不上 | 先起 follow server（上面 ① ② 两步）；用 `ss -tlnp \| grep :9001` 确认它在监听哪个端口；那条日志**每 2s 才刷一行**是设计如此（连接失败后 2s 内不重试，否则每帧 connect 会拖死仿真主循环） |
| **仿真里臂在动、真机不动** | `real.enabled: true` 但桥接从未连上 —— 下发全部失败，而仿真窗口照常跟手 | ⚠ **别用 pybullet 窗口判断真机有没有收到指令**，看终端有没有 `[真机]…已连接`。上一条解决连接问题 |
| `import litearm` 失败 | SDK 不在解释器里 | 脚本会探 `~/github/litearm/litearm-python/src`；换机器用 `LITEARM_PY_SRC=<litearm-python>/src`，或 `pip install -e <litearm-python>` |
| 3s 内未收到状态帧 | 串口被占 / USB 未枚举 / 固件没在跑 | 串口独占：先 `fuser -v /dev/ttyACM0` 看谁占着；`--port` 显式指定 |
| `--mode jf` 被拒（`ERR [08,0]`） | 固件 <1.9.0 没有 JOINT_FOLLOW | 先跑 `./run_stm32_comm_test.sh` 看固件版本，然后退到 `--mode js` 或 `--mode j` |
| 状态帧里 `WD_TRIPPED` | **命令看门狗跳了**：没有任何客户端在按 ≥10Hz 喂帧 ⇒ 固件 fail-soft（降刚度、τ=0） | 正常现象（没在遥操时就会有）；`start` 后帧流恢复即自动解除 |
| 每个关节 `e=01` | 不是故障 —— 逐关节 err 的常态值；SDK 的 `RobotState.faulted` 仍为 False、`joint_fault=0` | 无须处理。真故障看 `faulted` / `joint_fault` / `fail_axes` |
| 使能看起来没生效 | `litearm.Arm.enable()` **在状态帧反映之前就返回**（实测 2~3ms 返回，而 `enabled` 首次为真要 17~48ms） | 本仓库的 `Stm32Arm.enable()` 已在返回后**轮询确认**，别改成直接读 |

### 读状态帧（`./run_stm32_comm_test.sh` 的输出怎么读）

```
mode=MOVE_JS(3) flags=['WD_TRIPPED'] | J1 q=... dq=... τ=... T=../..C e=01
```

- `mode`：固件当前模式。`3`=`MOVE_JS`、`1`=`MOVE_J`、`5`=`MIT_ALL`。
  ⚠ `JOINT_FOLLOW(0x08)` 在状态帧里**仍报 `MIT_ALL`**（mode 字段 3 bit 已满被复用），
  所以**无法从状态帧判断是否在跟随**。
- `flags` 低 6 位：`FAULT / WD_TRIPPED / FB_STALE / TEMP_WARN / POS_VIOL / OVERSPEED`；
  bit6–8 是 mode、bit9 是 `enabled`、bit10 是 `cart_busy`（这三位不进 `flags` 名字列表）。
- `τ` 是实测力矩。fail-soft 时是 0；持位时能看到各关节的重力项。

### ⚠ 第一次握持 grip 会把臂送到 home

`ws_echo_server.py` 在**首次** grip 上升沿会先发 `movej(home)` 再进跟随，`home` 就是
`teleop_sim.py` 的 `SINGLE_Q_HOME`（单臂正装）/[`Q_HOME`]（双臂左臂）。
⇒ **按下之前先比一下当前实测 q 和 home 的差**，差值大的关节会真的转那么多：

```bash
cd /home/yudao/wkspace/vr && python3 - <<'EOF'
from litearm import Arm
from litearm_teleop.teleop_sim import SINGLE_Q_HOME   # 需 PYTHONPATH 指到 teleop/src
a = Arm("/dev/ttyACM0").connect()
q = a.get_state().value.q
print("当前:", [round(v,3) for v in q])
print("home:", [round(v,3) for v in SINGLE_Q_HOME])
print("差  :", [round(h-c,3) for c,h in zip(q, SINGLE_Q_HOME)])
a.close()
EOF
```

（2026-10-14 实测：本机 J5 差 **1.56 rad ≈ 89°**、J6 差 **0.87 rad ≈ 50°** —— 这是一次
**大幅快动**，`sp=0.5` 下大约 1~2 秒走完。工作区务必清空、急停在手边。）

### 定位「关节跳变 / 跟手感不一致」

单看最终关节角分不出是谁造成的。用探针把**整条链每帧**落盘，让数据自己说：

```bash
# ① metavr 侧带 --probe 起（写 sim 流；正常遥操别开）
./teleop/launch/run_teleop_webxr.py --real --real-right 127.0.0.1:9002 \
    --probe /tmp/probe_sim.jsonl
# ② 真机侧另开一个终端采样（follow server 就是第二段链路的真值）
python3 teleop/scripts/probe.py sample --addr 127.0.0.1:9002 --out /tmp/probe_follow.jsonl
# ③ 复现完出归属报告
python3 teleop/scripts/probe.py report --sim /tmp/probe_sim.jsonl --follow /tmp/probe_follow.jsonl
```

各原因在数据上长得**不一样**，报告会直接给结论：

| 现象 | 数据特征 | 归谁 |
|---|---|---|
| 帧外断点（`q_before` ≠ 上一帧 `q_after`） | 关节被**命令**搬走 | `go_home`/双击归位/真机 `movej` —— **不是 IK** |
| 关节大跳而**末端几乎没动** | 工具没跑，关节跳了 | IK 冗余零空间换分支 |
| 关节**不动** | `ik.achieved=False` / `failure=box,residual` | exact 的策略持位 |
| 增益不足但无跳变 | `ik.at_limit=True`（`|v|max` 顶到 `v_limit`） | **速度上限在削答案**（`pin` 还会叠加阻尼衰减） |
| 手在动而关节严格不动 | `zeroed=True` 连续多帧 | 静止死区（出厂 1.8cm/s / 0.18rad/s） |
| 下发目标平滑但实测跟不上 | `wire cmd_q` 平滑、`真机 q` 落后 | 真机侧 slew / 限位 / 看门狗 |

报告开头那行汇总最有信息量，例如：

```
合计 8536 帧：死区置零 1.9%  策略持位 0.0%  速度≥90%上限 19.4%  σ_min<0.050 13.9%  帧外断点 3 次
```

「速度≥90%上限」与「σ_min 小」同时高 = 跟手感不足的主因是**奇异区里任务的可及速度不够**
（答案被盒削掉），而不是 IK 解错 —— 那种情况换求解器不会好，要么降死区/提速度上限，
要么接受「近奇异处要么精确要么停」的 `--ik exact` 策略。

### 停下来 / 后退

| 动作 | 命令 |
|---|---|
| 松手暂停（不切模式，固件帧流就地保持） | 松开 grip |
| 双击 grip | 回 home 并停跟随 |
| 急停（**失能、可能掉臂**） | 给 server 发 `{"cmd":"stop"}` |
| 只停软件 | `pkill -f ws_echo_server`；server 收到 SIGINT/SIGTERM 会先 `movej(当前角)` 就地增刚持位再退出 |

⚠ `stop` 在两版 server 语义不同：串口版是 `CMD_EMERGENCY_STOP`（**失能**），
CAN 版是「安全停止 + 高刚度持位」。别混用。

### 已知边界

- **metavr 对真机是开环的**：它不读真机状态，也不知道真机被 slew 拖住。真机滞后在仿真里看不出来。
- 真机侧还有第二层整形：`ArmFollowServer` 的限位钳制 + `_sticky_hold`（静止保持）+ 梯形 slew。

## 遥操运行时

| 问题 | 原因 | 解法 |
|---|---|---|
| 一条帧都收不到（默认 `--source zenoh`） | 桥没起 / arm 段不一致 / 连的端点不对 | ① 先起 `litearm-vr-bridge`（它 listen `tcp/0.0.0.0:7447`）；② `--arm-id` 必须与桥的 `ZENOH_ARM_ID` 一致（**不一致不报错，只是收不到**）；③ `--zenoh-connect` 要指向那台的 7447；④ 终端没打 `[诊断] 首帧原始数据` 就说明帧没到。退一步用 `--source ws` 让头显直连本机，先确认头显自己在发 |
| `ImportError: 数据源 zenoh 需要 eclipse-zenoh` | metavr-arm 环境没装它（`vr_source` 是延迟 import，只有走 zenoh 时才炸） | `~/venvs/metavr-arm/bin/python -m pip install eclipse-zenoh`（或 `bash environments/setup.sh` 重建）；也可以 `--source ws` 绕过 |
| 终端刷「队列已满…丢弃当前帧」 | IK 跟不上帧率（zenoh 帧队列深度 64） | 偶发（掉帧追赶）可忽略；持续说明本机在别处卡了（RViz2 / 日志 / CPU 被抢），或换更快的 IK 后端 |
| 手柄数据完全冻结 | ① 头显未佩戴/手柄不在视野；② 客户端长时间运行卡死 | ① 确认佩戴+手柄在视野；② 客户端重连 |
| 机械臂乱摆/振荡 | 增量基于实际位姿做全量追赶 + 平滑滞后 | 已修：增量基于上帧目标（手柄停即停）；若复现检查 `robot_moving_H` 逻辑 |
| 陷入奇异 | 臂伸直位等构型 sigma_min→0 | 见 architecture.md"遥操语义"；操作上：暂停→手柄回中立→重新开始 |
| 启动时 `Address already in use`（8765） | ① 旧服务端孤儿进程占端口；② `--source ws` 时与桥抢 8765 | ① `./launch/stop_teleop.sh`（或 pkill -f ws_echo_server）；② 用 `--source ws` 时别让桥同时听 8765，或改用别的 `--port`。⚠ 默认的 zenoh 模式**不占** 8765，启动脚本也不会去查这个端口 |
| pkill 把终端也杀了 | pkill 模式匹配到自身 shell 命令行 | 模式用 `[w]s_echo_server` 且与启动命令**分开执行** |

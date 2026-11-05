# 使用手册

## 可选项都在这一个文件里：`config/teleop.yaml`

全部可调项（含单位、取值范围与说明）都在 `config/teleop.yaml`，包括频率、限幅、
初始构型、轴映射标定值、IK 求解器参数、死区手感、真机地址、灵巧手/夹爪参数。
启动脚本与服务端都读它。

优先级：**命令行 > 配置文件 > 代码出厂常量**。所以：

```bash
cp config/teleop.yaml config/my_arm.yaml   # 改这一份，别动出厂的那份
./launch/run_teleop_webxr.py --config config/my_arm.yaml
./launch/run_teleop_webxr.py --port 9000   # 临时覆盖单项，不改文件
```

- 布尔项都支持 `--x / --no-x`（如 `--dual` / `--no-dual`），配置文件里写了 `true`
  也能从命令行关掉。
- **标着「标定值」的段落**（`mapping`、夹爪行程、`tuning.tcp_offset`）是对着真机量
  出来的，没有重新标定别改 —— 改了不报错，只会让手和臂的对应关系错掉。
- 配置文件里删掉某个键不会报错，该键退回代码里的出厂常量。

## 每次使用流程（中间层 zenoh 手柄遥操）

1. **启动桥**（电脑）：`litearm-vr-bridge`（收头显的 8765，转发到 zenoh
   `litearm/v4/{arm}/vr_frame`）。本仓默认就从这条话题取帧。
2. **启动服务端**（电脑）：
   ```bash
   ./launch/run_teleop_webxr.py      # 单进程：zenoh 订阅 + pybullet 仿真
   ```
   电脑弹出 pybullet 3D 窗口（鼠标拖拽旋转/滚轮缩放），机械臂保持初始位姿；
   终端打印 `数据源: 中间层 zenoh 订阅 litearm/v4/armA/vr_frame（…）`。
   没接桥、想让头显直连本机时改用 `--source ws`（此时头显连 `ws://<电脑IP>:8765/`）。

3. **连接头显客户端**：
   - Quest 客户端连接 `ws://<电脑IP>:8765/`（桥的在听地址，IP 与电脑同一 Wi-Fi 下）
   - 手柄数据开始发送（每帧 JSON，含 leftController/rightController）
   - 收到第一帧时终端会打一行 `[诊断] 首帧原始数据: …` —— 看到它才说明帧真的到了

3. **遥操**：
   | 操作 | 功能 |
   |---|---|
   | 右手手柄 gripActive 按下 | 开始 / 恢复（重新锚定） |
   | 右手手柄 gripActive 松开 | 暂停 |
   | 手柄移动 / 旋转 | 驱动机械臂 |
   | 扳机 | 夹爪（当前仿真忽略夹爪） |

## 场景可视化（pybullet 窗口）

- 机械臂本体（URDF STL 渲染）
- 灰色视觉地面（仅显示，无碰撞）

## 常见问题速查

| 现象 | 处理 |
|---|---|
| 连不上 ws | 检查 IP、电脑/头显同网段、**桥**在跑、端口 8765（`--source ws` 时才是本仓在听） |
| 服务端在跑但一条帧都收不到 | ① 桥起了吗（默认数据源是中间层）；② `--arm-id` 与桥的 `ZENOH_ARM_ID` 是否一致（**不一致不报错，只是收不到**）；③ `--zenoh-connect` 是否指向桥的监听端点（默认 `tcp/127.0.0.1:7447`）；④ 看终端有没有 `[诊断] 首帧原始数据`。退一步：`--source ws` 让头显直连本机，先确认头显自己在发 |
| 机械臂不动 | 先按下 gripActive 开始；手柄需被追踪（放在头显视野内） |
| 机械臂卡住不动 | 松开 gripActive 暂停 → 手柄回中立位 → 重新按下 gripActive 开始（重新锚定）；看终端有无 `[持位]` 预警 |
| 大幅挥手时跟着跟着停住，终端刷 `[持位]…[box]` | 这是 `--ik exact` 的设计语义：手速需要的关节速度超出盒（`exact_ik.max_joint_vel` 与「行程/dt」的交集）⇒ 就地持位，不做部分交付。预警句里的 `起作用的是关节速度上限` / `关节行程` 指明是哪一半在管；后半句说明该位姿**是否**接近奇异 —— σ_min 正常时别往奇异点上找原因。嫌停得太早、或想取消速度这一界，见 `config/teleop.yaml` 的 `exact_ik.max_joint_vel` |
| 运动方向/镜像不对 | 调 `config/teleop.yaml` 的 `mapping` 段（`axis_remap`/`translation_remap`/`rotation_signs`/`dual_*`），改完重跑（见 architecture.md） |
| 改了 `config/teleop.yaml` 没生效 | ① 看启动时打的 `[配置] <路径>` 是不是你改的那份（`--config` 换文件）；② 同一项在命令行上也给了 —— **命令行赢**；③ 键名写错/缩进错了会退回出厂常量且不报错，跑 `make test-config` 会帮你查出来 |
| 换网络后连不上 | 重跑 run_teleop_webxr.py（自动更新 IP）→ 客户端重新连接 |

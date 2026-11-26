English | [简体中文](troubleshooting.zh-CN.md)

# Troubleshooting (field notes)

## Environment setup

| Problem | Cause | Fix |
|---|---|---|
| conda install returns HTTP 000 | The configured mirror is gone (the USTC anaconda mirror is retired and now redirects with 302) | Switch back to the Tsinghua mirror: `--override-channels -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r` (`setup.sh` already uses it by default) |
| GitHub clone is slow | Direct connections are throttled | Use `https://ghfast.top/https://github.com/<org>/<repo>/archive/refs/heads/<branch>.zip` (note that LIBERO's default branch is `master`) |
| HuggingFace always times out | The domain is blocked | Work around it: fetch assets from GitHub instead of letting HF download them |
| `pip install -e .` succeeds but the import fails | `find_packages()` does not treat the directory next to `setup.py` as a package | Set `packages=["xxx"]` and `package_dir={"xxx": "."}` explicitly, or add `__init__.py` under a subdirectory |
| pybullet and numpy have an ABI mismatch | pybullet 3.2.6 was built for numpy 1.x | Use `pybullet==3.2.7` (built for numpy 2 and compatible with numpy 2.2) |

## PyBullet simulation

| Problem | Cause | Fix |
|---|---|---|
| No 3D window in a headless environment | The GUI connection needs `DISPLAY` | Set `LITEARM_NO_VIEWER=1` (or pass `--no-viewer`) to fall back to DIRECT mode |
| FK pose is off by 5 cm or more | `getLinkState` indexes `[0][1]` are the centre-of-mass frame, not the link frame | Use `[4][5]` (the URDF link frame, which matches the old pylitearm `ee_frame`) |
| IK jumps between 7-DOF redundant solutions | The redundant arm has multiple solutions and the seed is not continuous | Use `restPoses=current q` plus `jointDamping` to pull the null space and `sim.max_step_rad` to limit the step (already built in) |
| `createMultiBody` reports an invalid keyword | pybullet 3.2.7 has no `visualShapes` alias | Use `baseVisualShapeIndex` / `linkVisualShapeIndices` |

## RViz2 display (three pitfalls found in the field)

| Problem | Cause | Fix |
|---|---|---|
| `robot_state_publisher` crashes at startup (`Failed to parse global arguments`) | The SolidWorks URDF is CRLF multi-line XML; passing it with `-p robot_description:="$(cat URDF)"` is rejected by the rcl argument parser | Use `--params-file` with a YAML file (JSON-escaped URDF, CRLF removed automatically) |
| RobotModel shows red (`Error retrieving file [.../xxx.STL]`) | RViz's assimp does not accept bare absolute mesh paths | Prefix meshes with `file://` in the URDF: `sed "s|package://litearm_description/meshes/|file:///abs/path/meshes/|g"` |
| RViz is blank in single-arm mode | The single-arm branch of the run script uses the dual-arm RViz config (Fixed Frame=`column_base_link`, while the single-arm root is `base_link`) | Use `models/litearm_description/rviz/display.rviz` for single-arm (Fixed Frame=`base_link`) |
| Global Status: Warn (Fixed Frame does not exist) | No `robot_state_publisher` is publishing TF (usually because of the crash above) | Fix the crash first; confirm `ros2 topic info /tf` reports Publisher count=1 |

## Real-arm connection (serial path / litearm SDK)

> The authoritative scripts live at the repository root: `run_stm32_comm_test.sh` and `run_stm32_server.sh`. This section only records how to diagnose problems; it does not repeat their usage.

### Two paths, only one works on this machine

| Path | Script | Serial or CAN dependency | This machine |
|---|---|---|---|
| Serial (control loop in STM32 firmware) | `arm_follow_server_stm32.py` | **`litearm` SDK** ([litearm-python](https://github.com/nexform-tech/litearm-python), pyserial only) | Yes |
| CAN (STM32 is only a CAN bridge) | `arm_follow_server_sdk.py` | `pylitearm` | No, the source is not present on this machine |

Check whether `pylitearm` is usable: `python3 -c "from pylitearm import Arm, ArmState"`.
On this machine it fails because `~/.local/lib/python3.10/site-packages/__editable__.pylitearm-5.0.0.pth`
points to `/home/yudao/wkspace/pylitearm/src`, and that directory has been deleted (dangling editable install).
The only way to use this path is to obtain the `pylitearm` source and `pip install -e` it again.

Check whether `litearm` is usable: `python3 -c "import litearm; print(litearm.__version__)"`.
On this machine it is installed editable for the system `python3` (`__editable__.litearm_python-2.1.0.pth`); no extra install is needed.

### Start order (follow it exactly)

```bash
cd /home/yudao/wkspace/vr
./run_stm32_comm_test.sh                              # 1. read status and firmware only; does not move the arm
./run_stm32_server.sh --mode jf --tcp-port 9002       # 2. start the follow server; the arm still does not move
# In another terminal:
cd teleop && ./launch/run_teleop_webxr.py --real --real-right 127.0.0.1:9002   # 3. start teleop
```

| Problem | Cause | Fix |
|---|---|---|
| **The arm does not move but the logs look normal** | Single-arm and dual-arm port mapping differs: single-arm **upright** sends only to `right` (default `127.0.0.1:9002`), single-arm **side mount** sends only to `left` (default `127.0.0.1:9001`), while the follow server defaults to 9001 | Pick one: start the server with `--tcp-port 9002`; pass `--real-right 127.0.0.1:9001` to metavr; or (recommended) set `real.right_addr` / `real.left_addr` in `config/teleop.yaml` so it is not typed every time |
| The real arm cannot be reached and the log repeats `[real][right] connection failed ... Connection refused` | No process is listening on that port (`Errno 111` means the host is reachable but the port is empty, not that the network is down), i.e. the follow server is not running or the port does not match | Start the follow server first (steps 1 and 2); confirm the listening port with `ss -tlnp \| grep :9001`. The log repeats only every 2 s by design (no retry for 2 s after a failure; retrying every frame would stall the simulation loop) |
| **The simulated arm moves but the real arm does not** | `real.enabled: true` but the bridge never connected, so every command failed while the preview kept following | Do not use the PyBullet window to judge whether the real arm receives commands; look for `[real] ... connected` in the terminal. Fix the connection with the previous row |
| `import litearm` fails | The SDK is not in the interpreter | The script probes `~/github/litearm/litearm-python/src`; on another machine set `LITEARM_PY_SRC=<litearm-python>/src`, or `pip install -e <litearm-python>` |
| No status frame within 3 s | The serial port is occupied, USB has not enumerated, or the firmware is not running | The serial port is exclusive: check `fuser -v /dev/ttyACM0` for the owner; pass `--port` explicitly |
| `--mode jf` is rejected (`ERR [08,0]`) | Firmware older than 1.9.0 has no `JOINT_FOLLOW` | Run `./run_stm32_comm_test.sh` to read the firmware version, then fall back to `--mode js` or `--mode j` |
| The status frame contains `WD_TRIPPED` | **The command watchdog tripped**: no client is feeding frames at 10 Hz or more, so the firmware goes fail-soft (lower stiffness, τ=0) | This is normal while teleoperation is not running; it clears automatically once the frame stream resumes after `start` |
| Every joint reports `e=01` | Not a fault: this is the normal per-joint `err` value; `RobotState.faulted` is still false and `joint_fault=0` | No action needed. Real faults show up in `faulted`, `joint_fault` or `fail_axes` |
| Enabling seems to have no effect | `litearm.Arm.enable()` **returns before the status frame reflects it** (about 2–3 ms to return, while `enabled` first becomes true after 17–48 ms) | `Stm32Arm.enable()` in this repository already polls until confirmed; do not change it to a direct read |

### Reading the status frame (`./run_stm32_comm_test.sh` output)

```
mode=MOVE_JS(3) flags=['WD_TRIPPED'] | J1 q=... dq=... τ=... T=../..C e=01
```

- `mode`: the current firmware mode. `3`=`MOVE_JS`, `1`=`MOVE_J`, `5`=`MIT_ALL`.
  Note that `JOINT_FOLLOW(0x08)` **still reports `MIT_ALL`** in the status frame
  (the mode field reuses the 3 bits), so the status frame cannot tell whether
  the arm is following.
- Low 6 bits of `flags`: `FAULT / WD_TRIPPED / FB_STALE / TEMP_WARN / POS_VIOL / OVERSPEED`;
  bits 6–8 are the mode, bit 9 is `enabled`, and bit 10 is `cart_busy` (these
  three are not part of the flag-name list).
- `τ` is the measured torque. It is 0 in fail-soft; while holding it shows each
  joint's gravity term.

### The first grip sends the arm home

`ws_echo_server.py` sends `movej(home)` on the **first** grip rising edge before
entering follow mode. `home` is `SINGLE_Q_HOME` (single-arm upright) in
`teleop_sim.py` or `Q_HOME` (dual-arm left arm).
Compare the current measured `q` with home before pressing; joints with a large
difference will really move that far:

```bash
cd /home/yudao/wkspace/vr && python3 - <<'EOF'
from litearm import Arm
from litearm_teleop.teleop_sim import SINGLE_Q_HOME   # PYTHONPATH must point at teleop/src
a = Arm("/dev/ttyACM0").connect()
q = a.get_state().value.q
print("current:", [round(v,3) for v in q])
print("home:   ", [round(v,3) for v in SINGLE_Q_HOME])
print("diff:   ", [round(h-c,3) for c,h in zip(q, SINGLE_Q_HOME)])
a.close()
EOF
```

(Measured on 2026-10-14: J5 differed by **1.56 rad ≈ 89°** and J6 by
**0.87 rad ≈ 50°**; that is a large, fast move, about 1–2 s at `sp=0.5`.
Clear the workspace and keep the emergency stop within reach.)

### Diagnosing joint jumps or inconsistent tracking feel

The final joint angles alone cannot tell which stage caused a jump. Use the
probe to record every stage of the chain per frame and let the data identify it:

```bash
# 1. Start metavr with --probe (writes the sim stream; do not enable it during normal teleoperation)
./teleop/launch/run_teleop_webxr.py --real --real-right 127.0.0.1:9002 \
    --probe /tmp/probe_sim.jsonl
# 2. Sample the real arm from another terminal (the follow server is the ground truth for the second stage)
python3 teleop/scripts/probe.py sample --addr 127.0.0.1:9002 --out /tmp/probe_follow.jsonl
# 3. Produce the attribution report after reproducing the problem
python3 teleop/scripts/probe.py report --sim /tmp/probe_sim.jsonl --follow /tmp/probe_follow.jsonl
```

Each cause looks different in the data, and the report states the conclusion:

| Symptom | Data signature | Attributed to |
|---|---|---|
| Out-of-frame discontinuity (`q_before` does not match the previous frame's `q_after`) | The joint was **commanded** to move | `go_home`, double-click home, or a real-arm `movej` — **not IK** |
| Large joint jump while the **end effector barely moves** | The tool stayed while the joints moved | IK null-space branch switch |
| Joints **do not move** | `ik.achieved=False` or `failure=box,residual` | Exact solver policy hold |
| Insufficient gain without a jump | `ik.at_limit=True` (`|v|max` hits `v_limit`) | **The speed limit is clipping the answer** (`pin` adds damping on top) |
| The hand moves while the joints stay strictly still | `zeroed=True` for consecutive frames | Rest deadband (factory 1.8 cm/s and 0.18 rad/s) |
| Commanded target is smooth but measured joints lag | `wire cmd_q` is smooth, real `q` lags | Real-arm slew, joint limits, or the watchdog |

The summary line at the top of the report carries the most information, for example:

```
total 8536 frames: deadband zeroed 1.9%  policy hold 0.0%  speed >=90% limit 19.4%  sigma_min<0.050 13.9%  out-of-frame discontinuities 3
```

A high share of both "speed >=90% limit" and "sigma_min small" means the main
cause is that the **achievable speed in the singular region is insufficient**
(the answer is clipped by the box), not that IK solved the wrong problem.
Changing the solver will not help; lower the deadband, raise the speed limit,
or accept the `--ik exact` policy of "exact or stop near singularities".

### Stopping and backing off

| Action | Command |
|---|---|
| Release to pause (no mode switch; the firmware holds in place) | Release grip |
| Double-click grip | Return home and stop following |
| Emergency stop (**disables the arm and may drop it**) | Send `{"cmd":"stop"}` to the server |
| Stop only the software | `pkill -f ws_echo_server`; on SIGINT/SIGTERM the server sends `movej(current angles)` to hold stiffly before exiting |

Note that `stop` means different things in the two server versions: the serial
version sends `CMD_EMERGENCY_STOP` (**disables the arm**), while the CAN version
does a "safe stop plus high-stiffness hold". Do not mix them.

### Known boundaries

- **metavr is open-loop with respect to the real arm**: it does not read the
  real arm state and cannot tell that the real arm is held back by slew. The
  real-arm lag is invisible in the simulation.
- The real-arm side adds a second shaping layer: `ArmFollowServer`'s limit
  clamping, `_sticky_hold` (rest hold), and trapezoidal slew.

## Teleop runtime

| Problem | Cause | Fix |
|---|---|---|
| Not a single frame arrives (default `--source zenoh`) | The bridge is not running, the arm segment does not match, or the endpoint is wrong | 1. Start `litearm-vr-bridge` (it listens on `tcp/0.0.0.0:7447`); 2. `--arm-id` must match the bridge's `ZENOH_ARM_ID` (a mismatch is silent and simply receives nothing); 3. `--zenoh-connect` must point at that machine's 7447; 4. if the terminal never prints `[diag] first raw frame`, no frame arrived. As a fallback use `--source ws` and connect the headset directly, to confirm the headset is sending |
| `ImportError: data source zenoh requires eclipse-zenoh` | The metavr-arm environment does not have it (`vr_source` imports lazily, so it only fails on the zenoh path) | `~/venvs/metavr-arm/bin/python -m pip install eclipse-zenoh` (or rebuild with `bash environments/setup.sh`); `--source ws` also bypasses it |
| The terminal repeats "queue full ... dropping current frame" | IK cannot keep up with the frame rate (the zenoh frame queue depth is 64) | Occasional drops while catching up are harmless; a persistent stream means the machine is stalled elsewhere (RViz2, logging, CPU contention) or a faster IK backend is needed |
| Controller data freezes completely | 1. The headset is not worn or the controller is out of view; 2. the client has been running for too long and hung | 1. Confirm the headset is worn and the controller is in view; 2. reconnect the client |
| The arm swings or oscillates | The increment is computed from the actual pose and catches up fully, with smoothing lag | Fixed: the increment is based on the previous target (the arm stops when the hand stops); if it reappears, check the `robot_moving_H` logic |
| The arm falls into a singularity | Configurations such as a fully extended arm drive sigma_min toward 0 | See "teleop semantics" in architecture.md; operationally: pause, return the controller to neutral, and start again |
| `Address already in use` on startup (8765) | 1. An orphaned old server process owns the port; 2. `--source ws` conflicts with the bridge on 8765 | 1. `./launch/stop_teleop.sh` (or `pkill -f ws_echo_server`); 2. do not let the bridge listen on 8765 while using `--source ws`, or use another `--port`. The default zenoh mode **does not use** 8765 and the launcher does not check that port |
| pkill also kills the terminal | The pkill pattern matches its own shell command line | Use the pattern `[w]s_echo_server` and run it **separately** from the launch command |

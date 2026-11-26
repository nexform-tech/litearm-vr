English | [简体中文](README.zh-CN.md)

# litearm-vr-teleop — Quest 3 VR teleoperation (controller pose -> joint angles)

The current control uses a fixed grip anchor and constrained pose tracking; read
the [drift fix and migration notes](teleop/docs/anchored-tracking.md) before
deployment.

A teleoperation system for the LiteArm based on Quest 3 WebXR controllers. The
controller pose arrives through the **middle-layer Zenoh** (forwarded by
[`litearm-vr-bridge`](https://gitee.com/yudao_hz_1/litearm-vr-bridge) from 8765
to `litearm/v4/{arm}/vr_frame`), goes through the teleoperation semantics (fixed
anchored pose tracking) and inverse kinematics, and outputs joint angles for the
LiteArm V4 seven-axis arm: either drawn in simulation or sent over TCP to the
real-arm follow service. `--source ws` falls back to "this repository listens on
8765 and the headset connects directly".

This repository **stops at joint angles**. Turning joint angles into motor
commands belongs to
[`litearm-vr-follow`](https://gitee.com/yudao_hz_1/litearm-vr-follow) (see the
9001 protocol contract in its README).

---

## Data flow

```
Quest 3 controllers (~72 Hz)
      │ ws 8765
      ▼
litearm-vr-bridge ──zenoh litearm/v4/{arm}/vr_frame──▶ this repository (ws_echo_server.py)
      │  (default data source)                                │
      │                          ├─ teleoperation semantics: velocity-style delta mapping
      │                          ├─ IK: pybullet (default) / pinocchio + pink / exact
      │                          │
      │                          ├──▶ pybullet GUI window or RViz2 (view)
      │                          ├──▶ real_arm_client ──TCP 9001/9002──▶ arm_follow_server (motion)
      │                          ├──▶ HandRemoteClient ──TCP 9004──▶ hand_server (dexterous hand)
      │                          └──▶ GripperRemoteClient ──TCP 9003──▶ gripper_server (gripper)
      │
      └──zenoh .../vr_status──▶ litearm-studio
```

- **Default `--source zenoh`**: subscribes to the frame stream forwarded by the
  bridge (its protocol is in `src/litearm_vr_bridge/protocol.py`). The topic is
  `litearm/v4/{arm_id}/vr_frame`; the session defaults to **peer + connect
  `tcp/127.0.0.1:7447`** (the bridge's listening endpoint). A peer that cannot
  connect only retries in the background, so **starting teleop before the
  bridge still works** (it connects within about 1 s once the bridge is up). The
  arm segment and endpoints are configurable: `--arm-id`, `--zenoh-mode`,
  `--zenoh-connect`, `--zenoh-listen`.
- **`--source ws`**: this repository listens on 8765 and the Quest client
  connects directly, bypassing the bridge. Useful when tuning teleoperation
  without a bridge, or on a machine without Zenoh.

---

## Features

- **Four configurations**: single-arm upright (default,
  `litearm_description.urdf`), single-arm horizontal side mount
  (`litearm_side.urdf`), dual-arm side-mount column
  (`dual_litearm_column_description`, mirrored left/right cooperation), and
  dual-arm upright (`dual_litearm_upright.urdf`, two independent vertical arms
  without a column; home and mapping reuse the single-arm upright baseline).
- **Three IK backends** (factory `--ik pybullet`, the metavr default):
  - `pybullet` (factory) — classic `calculateInverseKinematics`, light
    dependencies, behaviour aligned with metavr (best effort: sends even when
    not converged; no rest deadband; only the outer step limit is applied).
  - `pin` — pinocchio + pink, **policy aligned with metavr's `PinkIk`**: layered
    task stack (FrameTask + manipulability task + PostureTask), integrated per
    tick until convergence, QP hard limits on position and velocity. This
    repository supports dual arms where metavr does not.
  - `exact` — exact task velocity (`exact_ik.py`). **No damping**: the twist is
    either generated exactly at the hand's velocity or the arm holds in place;
    **the gain is always 1 or 0 and never varies with pose**. The magnitude is
    bounded by the joint-velocity box, and redundant DOF are used only when the
    box forces it. Failures distinguish "box infeasible / residual too large /
    Jacobian has no direction" and name the joint.
- **Real-arm following** (`--real`): solved joint angles are sent per frame over
  TCP/JSON to `arm_follow_server`; `gripActive` edges map to `start` / `latch`.
  The protocol matches both server versions in the follow repository.
- **Dexterous hand / gripper** (`--hand` / `--gripper`): trigger edges drive the
  remote TCP service; press = fist/close, release = open. The hardware drivers
  live in `litearm-vr-follow`; this repository only sends commands.
- **Two display backends**: pybullet's own 3D window (default, zero
  dependencies) / RViz2 (needs ROS2, can overlay TF).
- **Velocity-style delta teleoperation**: `gripActive` press = start/anchor,
  release = pause; per-frame controller pose deltas drive the arm, and the arm
  stops when the controller stops.

---

## Quick start

### 1. Install the environment (first time)

```bash
bash environments/setup.sh   # rebuild the conda environment metavr-arm
make smoke                   # smoke test (expects SMOKE TEST PASSED)
```

> ROS2 Humble is needed only for RViz display (optional). See
> [Environment setup](docs/environment-setup.md).

### 2. Start the teleoperation service

```bash
./launch/run_teleop_webxr.py    # one command (default: single-arm upright + pybullet IK + pybullet window, no real arm)
make stop                        # stop
```

The default data source is the middle-layer Zenoh, so **start the bridge first**
(it receives the headset on `ws://<PC-IP>:8765` and forwards to
`litearm/v4/armA/vr_frame`):

```bash
# Without the bridge, let the headset connect directly to this machine (local WS listener on 8765)
./launch/run_teleop_webxr.py --source ws
```

### 3. Enable the real arm / end effector as needed

Launcher arguments map one-to-one to `--help` and are passed through:

```bash
./launch/run_teleop_webxr.py --real                        # real arm (joint angles to arm_follow_server)
./launch/run_teleop_webxr.py --real --hand --hand-remote 192.168.66.95:9004   # real arm + dexterous hand
./launch/run_teleop_webxr.py --real --gripper --gripper-remote 192.168.66.95:9003  # real arm + gripper
./launch/run_teleop_webxr.py --dual                        # dual-arm side-mount column
./launch/run_teleop_webxr.py --dual --upright               # dual-arm upright (two independent vertical arms)
./launch/run_teleop_webxr.py --ik exact --deadband-scale 0 # exact task velocity + deadband off
```

`make teleop` behaves the same and passes `ARGS` through:
`make teleop ARGS="--dual --viewer rviz"`.

---

## Configuration

**Every option lives in [`teleop/config/teleop.yaml`](teleop/config/teleop.yaml)**
(units, valid ranges, and why each value was chosen). Both the launcher and the
server read it; use `--config` to select another file:

```bash
cp teleop/config/teleop.yaml teleop/config/my_arm.yaml
./launch/run_teleop_webxr.py --config teleop/config/my_arm.yaml
./launch/run_teleop_webxr.py --port 9000     # the command line only for temporary overrides
```

Priority: **command line > configuration file > factory constants in code**.
Every boolean supports `--x` / `--no-x`, so a `true` in the file can still be
turned off from the command line. Sections: `server` / `vr` (data source: Zenoh
topic and session), `env` / `robot` (single/dual arm, side/upright), `tuning`
(feel), `sim` (rates and limits), `mapping` (**calibration values**: three axis
mappings), `pin_ik` / `exact_ik`, `real` / `effector` / `controller`.

Sections marked as "calibration values" were measured against the real arm; do
not change them without recalibrating. For where to add a new option and the two
"option -> config path" tables, see the
[development guide](teleop/docs/development.md#配置文件configteleopyaml).

---

## Launch details

All arguments are in `./launch/run_teleop_webxr.py --help` (defaults are in the
configuration file above).

### Configurations

| Configuration | Command | Notes |
|---|---|---|
| **Single-arm upright** (default) | `./launch/run_teleop_webxr.py` | Right controller drives one arm |
| Single-arm side mount | `--side` | A single **left** arm: occupies the left slot (left controller, real arm on `:9001`); home/mapping use the dual-arm side-mount left arm. When combined with `--dual`, the dual-arm mode wins |
| Dual-arm side-mount column | `--dual` | Left/right controllers drive left/right arms (mirrored cooperation) |
| Dual-arm upright (no column) | `--dual --upright` | Two independent vertical arms; home/mapping reuse the single-arm upright baseline, right arm mirrored |

### IK backends

| Backend | Argument | Dependencies | Gain (hand -> arm) |
|---|---|---|---|
| **pybullet** (default) | `--ik pybullet` | pybullet only | ~0.44–0.98 (see "Known defect" below) |
| pinocchio + pink | `--ik pin` | `pin` / `pin-pink` / `qpsolvers` / `quadprog` | 0.9998 (integrated per tick until convergence) |
| `exact` exact task velocity | `--ik exact` | none (pure NumPy) | **1.000**, or 0 (hold) |

> **Known defect (pybullet backend)**: it uses absolute iterative IK with a
> 3 mm / 0.05 rad acceptance tolerance, while the target is rebuilt every frame
> from the **already-arrived pose**, so each frame's IK residual is discarded
> permanently. Measured hand-to-arm delivery ratio is only **0.435 / 0.462 /
> 0.480** (5 / 10 / 20 cm/s), i.e. moving the hand 10 cm moves the arm about
> 4.4 cm. It is only a light-dependency fallback; use `pin` or `exact` for
> responsive following.

### Speed and step limits

Two configuration keys (**not** command-line arguments), both in
`teleop/config/teleop.yaml`:

| Key | Factory | Purpose |
|---|---|---|
| `sim.max_step_rad` | `0.2` rad/frame (60 Hz => 12 rad/s) | Per-frame joint step cap, **prevents IK jumps**, independent of speed. Shared by `--ik pybullet` and `--ik pin` (metavr also clamps once after IK dispatch); the factory 0.2 is metavr's value. Under velocity-style deltas the clipped displacement is not made up; measured 0.08 reaches only 40–86% of a fast 5 cm wave |
| `exact_ik.max_joint_vel` | `10.0` rad/s | **Used only by `--ik exact`** (velocity box = intersection with "travel/dt"). Note that metavr has no such quantity, which is why it lives in the `exact_ik:` section rather than global `sim:` |

**Why limit speed**: first, to match the real arm. The real arm is held by
`arm_follow_server`'s trapezoidal slew at `--max-speed 2.0` / `--max-accel 3.0`;
without a simulation limit, a large wave "arrives instantly" in simulation while
the real arm cannot keep up. Second, `--ik exact` derives its criterion from it:
when the joint speed demanded by the hand exceeds the velocity box, the arm
**holds in place** and prints `[hold][box]`, stopping loudly instead of silently
under-delivering.

**Why limit the step**: a safety net independent of speed. When the seven-axis
redundancy switches solutions or passes near a singularity, one frame's solution
can jump by more than 1 rad; `sim.max_step_rad` flattens that jump.

**To disable the speed bound**: raise `exact_ik.max_joint_vel` from the factory
`10.0` to **`99.0`** (at 60 Hz, `99·dt ≈ 1.65 rad/frame`, far above any
reachable joint step => equivalent to disabling). Note three prerequisites:

- **`--ik pin` is unaffected**: its limit is pink's `VelocityLimit`
  (`pin_ik.velocity_limit`, factory `1.0` = the URDF value); relax that too if
  needed.
- **It does not make the real arm faster**: the real chain has two more layers —
  `arm_follow_server`'s `--max-speed` / `--max-accel` and the firmware/motor
  limits. The only effect here is that **the simulation no longer holds because
  of speed**: targets become more aggressive and the real arm still clips them.
- **Do not also relax `sim.max_step_rad`**: it guards against jumps, not speed.

### Display backends

| Backend | Argument | Dependencies | Notes |
|---|---|---|---|
| **pybullet GUI** (default) | `--viewer gui` | Python environment only | Own 3D window; drag to rotate, scroll to zoom |
| RViz2 | `--viewer rviz` | + ROS2 Humble | Shows RobotModel + TF and can overlay more visualisations |

### Real arm / dexterous hand / gripper

| Argument | Default | Notes |
|---|---|---|
| `--real` | off | Enable real-arm bridging (`real_arm_client.py`, joint angles over TCP/JSON) |
| `--real-left` / `--real-right` | `127.0.0.1:9001` / `:9002` | `arm_follow_server` address for the left/right arm |
| `--hand` | off | Enable the dexterous hand (trigger edge -> fist/open) |
| `--hand-remote` | empty | `hand_server.py` address; required with `--hand` (the hand is on another industrial PC's bus) |
| `--gripper` | off | Enable the gripper (trigger edge -> close/open) |
| `--gripper-remote` | empty | `gripper_server.py` address; required with `--gripper` (the gripper is on another industrial PC's bus) |

> The hand and gripper are both consumers of the same trigger edge; **normally
> enable only the one actually mounted on the arm**. Both hardware drivers live
> in `litearm-vr-follow` (`hand_server.py` / `gripper_server.py`); this machine
> sends commands over TCP and **never touches CAN or installs the SDK**.

### Full command-line reference (`launch/run_teleop_webxr.py`)

The launcher **uses only the standard library**, so the system `python3` can run
it; it finds the metavr-arm environment itself. The complete table is in
`./launch/run_teleop_webxr.py --help`.

| Argument | Default | Notes |
|---|---|---|
| `--metavr-py PATH` | auto | Python of the metavr-arm environment; when omitted, resolved via `CONDA_PREFIX` -> `conda info --base` -> common conda/venv roots |
| `--source` | `zenoh` | Data source: `zenoh` = subscribe to the middle-layer frame stream (bridge forwarding) / `ws` = own WS listener with the headset connecting directly |
| `--arm-id` | `armA` | Arm segment in the Zenoh topic (`litearm/v4/{arm_id}/vr_frame`). Must match the bridge's `ZENOH_ARM_ID`; **a wrong value does not error, it simply receives no frames** |
| `--zenoh-mode` | `peer` | Zenoh session mode: `peer` = retry in the background, **connects even if the bridge starts later** / `client` = requires a connection at session start (errors out without the bridge) / `router` |
| `--zenoh-connect` | `tcp/127.0.0.1:7447` | Endpoints Zenoh actively connects to, comma-separated (default = the local bridge's listening endpoint); use that machine's IP when the bridge is remote |
| `--zenoh-listen` | empty | Zenoh listening endpoints (default none: the local bridge already owns 7447) |
| `--host` | `0.0.0.0` | WS listening address (with `--source ws`) |
| `--port` | `8765` | WS listening port (with `--source ws`) |
| `--dual` | off | Dual arm (default single-arm upright); with `--upright` dual-arm upright, otherwise dual-arm side-mount column |
| `--upright` | off | With `--dual`: dual-arm upright (two independent vertical arms, no column); only meaningful with `--dual` |
| `--side` | off | Single-arm side mount = single left arm (left controller, real arm on `:9001`; when combined with `--dual`, dual-arm wins) |
| `--viewer` | `gui` | `gui` = pybullet window, `rviz` = RViz2 (needs ROS2) |
| `--ik` | `pybullet` | `pybullet` (factory, = metavr's `IK_BACKEND`) / `pin` / `exact` |
| `--tcp-offset X Y Z` | `0 0 0` | Tool offset in metres in the tool frame along the tool +z axis: offset of the controlled tool point from the tip link. **Factory `0 0 0` = control the flange directly**; with a gripper or dexterous hand set e.g. `0 0 0.18` to reach the grasp point |
| `--deadband-scale` | `1.0` | Rest deadband threshold multiplier; `0` = off (**lets tracking noise through**). The factory value at 60 Hz equals 1.8 cm/s / 0.18 rad/s |
| `--smooth-alpha` | `0.5` | EMA smoothing weight for new samples, in `(0, 1]`; `1.0` = no smoothing (most responsive, most jittery). Note that `0` freezes the arm |
| `--resume-margin` | `1.0` | Hold hysteresis for `--ik exact` (`1.0` = naive rule) |
| `--task-residual-tolerance` | `1e-3` | Residual ratio cap for `--ik exact` to decide "the task was generated" |
| `--manip-weight` | `1e-2` | Manipulability task weight for `--ik pin` (pushes redundancy toward better-conditioned Jacobians, **preventing** singularities). Factory `1e-2` (metavr keeps this task always on); 0 disables it. Larger values compete with FrameTask |
| `--control-frame` | `tool` | Frame in which rotation deltas are read: `tool` = end-effector local frame (factory/current) / `base_link` = base frame (rotates about **world axes**, same source as translation) |
| `--base-link-signs X Y Z` | `-1 -1 1` | Per-axis rotation signs for `--control-frame base_link` (initial values borrowed from teleop-vr measurements; **not re-measured on this bench**) |
| `--real` | off | Enable real-arm bridging |
| `--real-left` / `--real-right` | `127.0.0.1:9001` / `:9002` | Follow service addresses for the left/right arm |
| `--hand` | off | Enable the dexterous hand (trigger edge -> fist/open) |
| `--hand-remote HOST:PORT` | empty | `hand_server.py` address (required with `--hand`, e.g. `192.168.66.95:9004`) |
| `--gripper` | off | Enable the gripper (trigger edge -> close/open) |
| `--gripper-remote HOST:PORT` | empty | `gripper_server.py` address (required with `--gripper`, e.g. `192.168.66.95:9003`) |

### Direct invocation (without the launcher)

```bash
# Teleoperation service (data source defaults to zenoh; see --help for real/hand/gripper switches)
python scripts/ws_echo_server.py --source zenoh --arm-id armA \
    --zenoh-mode peer --zenoh-connect tcp/127.0.0.1:7447 \
    --viewer gui --ik pin [--dual] [--upright] [--side] [--no-ros] \
    [--real --real-left 127.0.0.1:9001 --real-right 127.0.0.1:9002] \
    [--hand --hand-remote host:port] \
    [--gripper --gripper-remote host:port]

# Headset connects directly to this machine (no bridge, no zenoh)
python scripts/ws_echo_server.py --source ws --host 0.0.0.0 --port 8765 \
    --viewer gui --ik pin [--no-ros]

# Pure simulation (no data source, includes --viewer / --ik / --dual / --upright / --side)
python -m litearm_teleop.teleop_sim --dual --upright --viewer gui --ik pin
```

---

## Teleoperation

| Controller input | Behaviour |
|---|---|
| **gripActive pressed** | Start/anchor (records the current controller pose as reference) |
| Move the controller while held | The arm follows (velocity-style delta mapping); with `REAL=1` it is also sent to the real arm |
| **gripActive released** | Pause |
| **trigger pressed** | Dexterous hand fist / gripper close (needs `HAND=1` / `GRIPPER=1`) |
| **trigger released** | Dexterous hand open / gripper open |

The trigger is analogue: it counts as "pressed" only above a threshold, with
debouncing to filter jitter.

When stuck in a singularity: pause, return the controller to neutral, press
`gripActive` again to re-anchor. (The `IK=pin` backend has LM damping and
null-space regularisation, so it resists singularities much better than
`pybullet`.)

---

## Directory layout

```
litearm-vr-teleop/
└── teleop/                            # teleoperation project: model descriptions + sources + launch/stop scripts
    ├── docs/                          # architecture / environment / user guide / troubleshooting / development
    ├── src/litearm_teleop/            # own sources: data source + teleoperation semantics + IK
    │   ├── teleop_sim.py              # single/dual-arm semantics + axis-mapping constants + IK backend dispatch
    │   ├── vr_source.py               # VR data source: subscribes to the middle-layer zenoh vr_frame topic
    │   ├── pin_ik.py                  # pinocchio + pink differential IK backend
    │   └── exact_ik.py                # exact task-velocity backend (no damping, gain 1 or 0)
    ├── launch/
    │   ├── run_teleop_webxr.py        # one-command launcher (standard library only; arguments replace the old environment variables)
    │   └── stop_teleop.sh             # stop
    ├── scripts/
    │   ├── ws_echo_server.py          # teleoperation service (zenoh subscription or WS listener -> teleop_sim in-process)
    │   ├── real_arm_client.py         # real-arm bridge: joint angles -> arm_follow_server (TCP/JSON)
    │   ├── ros_joint_state.py         # ROS2 /joint_states publication (for RViz)
    │   └── decimate_meshes.py         # STL decimation (generates meshes_lowpoly)
    ├── models/                        # model descriptions (geometry source of truth, each an independent ROS2 package)
    │   ├── litearm_description/       # single arm (the SE package from upstream litearm-description)
    │   │   ├── urdf/litearm_description.urdf   # upright (joint1~7 / ee_link)
    │   │   ├── urdf/litearm_side.urdf # horizontal side mount (derived from the upright model)
    │   │   ├── meshes/                # high-detail STL (for RViz)
    │   │   └── meshes_lowpoly/        # decimated STL (for pybullet rendering)
    │   └── dual_litearm_column_description/    # dual-arm URDF + STL: side-mount column / upright (no column),
    │                                          #   both variants reuse the same macro and mesh
    ├── environments/                  # conda environment definition + one-command rebuild
    ├── tests/                         # smoke tests
    └── Makefile  CHANGELOG.md
```

---

## Environment setup

For a first deployment read [Environment setup](docs/environment-setup.md):
system requirements, dependency list, one-command install, and network mirrors.

```bash
bash environments/setup.sh   # rebuild the conda environment metavr-arm
make smoke                   # smoke test
```

The three entry points (`setup.sh` / `run_teleop_webxr.py` / `Makefile`) all
resolve the **same environment**, so paths do not need to be aligned by hand:

- `setup.sh` asks conda for its root (`conda info --base`) and **does not
  guess** — the environment is created there
- the launcher and `make` resolve in this order: `--metavr-py` (or `METAVR_PY`)
  -> an already-activated environment with the same name -> `conda info --base`
  -> common conda/venv roots

So after `bash environments/setup.sh`, `make smoke` / `make teleop` work
directly without telling them where the environment is. If conda lives in an
unusual place, give a hint:

```bash
export METAVR_PY=~/myconda/envs/metavr-arm/bin/python   # or
make PY_ARM=~/myconda/envs/metavr-arm/bin/python smoke
```

### Runtime dependencies (beyond Python packages)

| Capability | Additional requirement |
|---|---|
| Middle-layer Zenoh data source (**factory** `--source zenoh`) | A local `litearm-vr-bridge` (listening on `tcp/0.0.0.0:7447`) + `eclipse-zenoh` in the environment (already in `environments/metavr-arm-requirements.txt`) |
| RViz2 display | ROS2 Humble |
| Real-arm following | `arm_follow_server` from `litearm-vr-follow` listening on `REAL_LEFT`/`REAL_RIGHT` |
| Dexterous hand | `hand_server.py` from `litearm-vr-follow` listening on `--hand-remote` (this machine needs no CAN/SDK) |
| Gripper | `gripper_server.py` from `litearm-vr-follow` listening on `--gripper-remote` (this machine needs no CAN) |

---

## Documentation index

- [Environment setup](docs/environment-setup.md) — system requirements / dependencies / installation / network mirrors
- [Architecture](docs/architecture.md) — data flow / teleoperation semantics / coordinate mapping
- [User guide](docs/user-guide.md) — daily operation and common questions
- [Troubleshooting](docs/troubleshooting.md) — all field-proven pitfalls
- [Development guide](docs/development.md) — layout / configuration knobs / model changes

English | [简体中文](README.zh-CN.md)

Real-time VR teleoperation for the **LiteArm V4 7-DoF arm**, with optional gripper
and L6 dexterous hand. You move your hands in a **VR headset**; the arm follows.

# LiteArm VR Teleoperation (litearm-vr)

This repository ships the complete pipeline in one place: the WebSocket bridge, the
inverse-kinematics teleop service, and the hardware follow services. Clone it once and
run either a pure simulation or a real robot from the same tree.

---

## What you need

| Item | Requirement |
| --- | --- |
| PC | Ubuntu 20.04 / 22.04, x86_64, graphical desktop, Python 3 |
| Headset | A supported OpenXR VR headset running the LiteArm VR client (closed-source; distributed separately), on the same LAN as the PC |
| Teleop environment | [conda](https://docs.conda.io/) — the teleop service runs in its own `metavr-arm` environment |
| Robot *(optional)* | LiteArm V4 7-DoF arm; optional gripper and L6 dexterous hand |
| Interfaces *(real robot only)* | USB serial (`/dev/ttyACM*`) for the STM32 path, **or** a SocketCAN adapter (`can0`); classic CAN at 1 Mbps |
| Permissions *(real robot only)* | Membership in `dialout`; `sudo` for CAN setup and the one-time bridge install |
| Display *(optional)* | ROS 2 Humble, only for the RViz2 visualization backend |

Hardware is optional. You can bring up the whole pipeline in simulation with just the
PC and a headset — or no headset at all, using the bundled fake frame source.

---

## How it works

```
VR headset (controller poses, ~72 Hz)
        │  WebSocket  ws://<PC-IP>:8765
        ▼
litearm-vr-bridge      receives frames, relays them, reports link status            (optional)
        │  zenoh  litearm/v4/{arm_id}/vr_frame
        ▼
litearm-vr-teleop      controller pose → teleop mapping → inverse kinematics → 7 joint angles
        │  TCP/JSON  :9001 / :9002 (arms)   :9003 (gripper)   :9004 (hand)
        ▼
litearm-vr-follow      joint angles / open-close commands → motors
        ├─ arm_follow_server_stm32.py / _sdk.py   arm   (serial / SocketCAN — mutually exclusive)
        ├─ gripper_server.py                      gripper
        └─ hand_server.py                         dexterous hand
        ▼
LiteArm V4 arm · gripper · L6 hand
```

**The bridge is optional.** Without it, the headset connects directly to the teleop
service on port 8765. With it, the bridge owns 8765, forwards to teleop on 8766, and
adds link-status reporting, frame-loss fallback, and supervised process start/stop.

---

## Repository layout

```
litearm-vr/
├── README.md / README.zh-CN.md            # overview (English / Chinese)
├── QUICKSTART.md / QUICKSTART.zh-CN.md    # step-by-step setup (English / Chinese)
├── litearm-vr-bridge/                     # WebSocket → zenoh bridge, link status, process control
│   ├── scripts/{install.sh,up.sh}         # one-time install / day-to-day start-stop
│   ├── src/litearm_vr_bridge/             # bridge source
│   └── submodules/                        # symlinks → ../litearm-vr-teleop, ../litearm-vr-follow
├── litearm-vr-teleop/                     # headset teleop (controller pose → joint angles)
│   ├── teleop/launch/run_teleop_webxr.py  # one-shot launcher
│   ├── teleop/src/litearm_teleop/         # IK, teleop mapping, simulation
│   └── teleop/config/teleop.yaml          # all options, including calibration
├── litearm-vr-follow/                     # hardware services (joint angles / commands → motors)
│   ├── arm_follow_server_stm32.py / _sdk.py
│   ├── gripper_server.py / hand_server.py
│   ├── status_aggregator.py / status_listen.py
│   └── follow.yaml                        # per-service defaults (ports, interfaces, gains)
├── third_party/litearm-can-dkms/          # vendored CAN driver, GPL-2.0-only (bundle only)
├── packaging/                             # bundle + frozen-teleop build scripts
├── scripts/                               # local and CI checks
└── docs/                                  # documentation index
```

Each component keeps its own README and `docs/`; those are authoritative for internals.

---

## Components

| Component | Responsibility | Main entry point |
| --- | --- | --- |
| **litearm-vr-bridge** | Relays headset frames to teleop over zenoh; reports link status; handles frame-loss fallback and supervised process start/stop | `litearm-vr-bridge/scripts/up.sh` |
| **litearm-vr-teleop** | Controller pose → teleop mapping → inverse kinematics → 7 joint angles; 3D preview (pybullet / RViz2) | `litearm-vr-teleop/teleop/launch/run_teleop_webxr.py` |
| **litearm-vr-follow** | Joint angles / open-close commands → motors; read-only status aggregation published to zenoh | `litearm-vr-follow/run_{stm32,sdk,gripper,hand}_server.sh` |

---

## Channel and port contract

| Edge | Channel | Notes |
| --- | --- | --- |
| headset → bridge | WebSocket `ws://<PC-IP>:8765` | the headset client speaks WebSocket only |
| bridge → teleop | zenoh `litearm/v4/{arm_id}/vr_frame` | `arm_id` defaults to `armA` (`ZENOH_ARM_ID`) |
| bridge ↔ control UI | zenoh `.../vr_status`, `.../vr_rpc` | status subscription, action queryable |
| teleop → arm | TCP `:9001` (left / single) and `:9002` (right) | one JSON object per line |
| teleop → gripper | TCP `:9003` | `{"cmd":"close"}` / `{"cmd":"open"}` |
| teleop → hand | TCP `:9004` | `{"cmd":"close","side":"left"}` |
| follow → status | zenoh `litearm/v4/{armA,armB}/vr_{arm,grip,hand}` | read-only, one topic set per arm |

`arm_id` is where "left / right" lives: a follow service only knows the serial port or
CAN interface it opened, never which arm it is. The identity comes from outside, via the
follow side's `--arm ARM_ID=[IFACE@]HOST:PORT`.

Ports must match on both ends: the follow service's `--tcp-port` (or `follow.yaml`) and
the teleop launcher's `--real-left` / `--real-right` (or `teleop.yaml`).

---

## Quick start

See [QUICKSTART.md](QUICKSTART.md) for the full walkthrough and troubleshooting.
The shortest path:

```bash
git clone https://github.com/nexform-tech/litearm-vr.git
cd litearm-vr

# 1) build the teleop environment (first time; needs conda)
bash litearm-vr-teleop/teleop/environments/setup.sh

# 2) run in pure simulation (no hardware; headset connects to ws://<PC-IP>:8765)
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py

# 3) real robot: start the follow services first, then teleop --real
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
litearm-vr-follow/run_stm32_server.sh --mode j --tcp-port 9001
litearm-vr-follow/run_gripper_server.sh --can can0 --port 9003
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --real \
    --gripper --gripper-remote 127.0.0.1:9003
```

---

## Safety

> **Real hardware moves for real.** Read this before powering the arm.

- Confirm the arm is powered correctly, the workspace is clear, and the e-stop is within
  reach before starting a real-robot session.
- **Serial and CAN are exclusive.** Do not let another program hold the serial port or
  CAN interface while a follow service is running.
- The two arm follow services (`stm32` over serial, `sdk` over SocketCAN) drive the same
  motors and are **mutually exclusive** — never run both.
- On frame loss the bridge commands the arm to hold its measured pose (no MOVE_J jolt).
  A deliberate operator stop still uses the latching stop.
- When the bridge-managed teleop carries `--real`, the bridge does **not** auto-start the
  upstream chain; run `./scripts/up.sh --link` explicitly (add `--yes` for
  non-interactive use).
- Start in simulation, then test with the arm unloaded and at low speed before running a
  full teleoperation session.

---

## Documentation and support

- Step-by-step setup and a troubleshooting table: [QUICKSTART.md](QUICKSTART.md)
- First real-arm bring-up: `litearm-vr-bridge/docs/REAL-ARM-BRINGUP.md`
- Bridge design and deployment: `litearm-vr-bridge/docs/DESIGN.md`, `docs/DEPLOY.md`
- Teleop options and IK backends: `litearm-vr-teleop/README.md`
- Follow service protocol and tuning: `litearm-vr-follow/README.md`
- Tracking model and migration notes: `litearm-vr-teleop/teleop/docs/anchored-tracking.md`

For issues, use the issue tracker of the repository that owns the affected component,
and include the launcher command, the full console output, and your `follow.yaml` /
`teleop.yaml` (redact credentials before posting).

---

## About this repository

This monorepo merges the three formerly separate component repositories
(`litearm-vr-bridge`, `litearm-vr-teleop`, `litearm-vr-follow`) so the whole pipeline
can be cloned and run in one place. Each subdirectory is that component's working tree.

The bridge looks for its upstream at `<bridge>/submodules/litearm-vr-{teleop,follow}`;
in this repository those paths are **symlinks** to the sibling components, so the
defaults work out of the box. If you relocate a component, point `TELEOP_REPO` /
`FOLLOW_REPO` at its root.

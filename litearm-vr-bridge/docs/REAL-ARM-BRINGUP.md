English | [简体中文](REAL-ARM-BRINGUP.zh-CN.md)

# Real-arm bring-up

> This page is for **the person standing in front of the machine**: connecting
> the VR chain to a real arm for the first time, or bringing it up again on a
> new machine. Configuration keys are in [DESIGN](DESIGN.md) section 7, daily
> start/stop is in [README](../README.md), and deployment and troubleshooting are
> in [DEPLOY](DEPLOY.md).
>
> **Read sections 0 and 1 before touching anything.** Section 2 has four
> stages, each with a "what passing looks like"; do not skip stages.

## 0. Choose the machine first: three hard constraints

| Constraint | Why | Consequence if unmet |
| --- | --- | --- |
| **Which machine has the STM32 USB cable** | The real-arm follow service opens `/dev/ttyACM0` (the STM32 is USB CDC) | It does not start — the log shows the upstream's "serial port does not exist" |
| **Which machine has the metavr-arm conda environment** | teleop needs pybullet / pinocchio / pink | It does not start — "cannot find the metavr-arm environment python" |
| **Whether the headset can reach that machine** | The bridge listens on that machine's LAN IP, and the headset connects by IP | Frames do not arrive — the fake headset works, the real one does not |

Note: **the STM32 is not on the "controller".** The device is wherever the USB
cable is plugged in (see the STM32 notes in `litearm-vr`). So "which machine to
debug the real arm on" is determined by where the arm's USB cable is.

There are therefore only three shapes:

- **A. A real Linux machine (industrial PC / development machine)** — satisfies
  all three constraints and is **preferred**. All you need to do is put the
  bridge there: `git clone --recursive` + `git submodule update --init` +
  `./scripts/install.sh --user`.
- **B. WSL / a container** — USB must be passed through (on WSL,
  `usbipd attach --wsl`), and the conda environment must be built there (the
  upstream `environments/setup.sh`). **WSL2 uses NAT by default, so the headset
  cannot connect** (the bridge's `lan_addr` line prints a 198.18.x.x address,
  which belongs to WSL). Either switch WSL to mirrored networking / add port
  forwarding, or use the fake headset for this shape (stage 1 in section 2).
- **C. Running directly on Windows** — pybullet / pinocchio are painful to
  install on Windows; not recommended.

## 1. Decide two things before powering on

### 1.1 Which control law to use while following (`FOLLOW_SERVER_CTRL_MODE`)

The four settings of the upstream `arm_follow_server_stm32.py --mode` (from its
`--help`):

| Value | Meaning |
| --- | --- |
| `jf` | JOINT_FOLLOW 0x08, **firmware servo loop plus feedforward** — the upstream "recommended" setting, **requires firmware >= 1.9.0** |
| `js` | MOVE_JS joint servo |
| `j` | MOVE_J trajectory solving |
| `mit` | MOVE_MIT_ALL passthrough (**no gravity compensation; use with care**) |

The bridge passes **`js`** by default (chosen in a previous session when
"aligning real-arm parameters"). If the board firmware is 1.9.0 or newer, try
the upstream-recommended `jf`:

```bash
FOLLOW_SERVER_CTRL_MODE=jf ./scripts/up.sh --link
```

**Read the firmware version from the board first** (do not guess):

```bash
./submodules/litearm-vr-follow/run_stm32_comm_test.sh /dev/ttyACM0
# Step 2 sends CMD_GET_FIRMWARE(0x41) and prints a version string such as "Litearm1.9.0-*"
```

### 1.2 Where the two environment variables point

The upstream scripts resolve their own environment; the bridge only passes it
down. On this machine export:

```bash
export LITEARM_TOOLS=<tools directory of the litearm-stm32 repository>   # where arm_console.py lives
export TELEOP_SERVER_PY=<envs/metavr-arm/bin/python>            # needed only when conda is not found
```

(The `follow_tools` / `teleop_python` items of `--doctor` check exactly this and
their messages state the search order used.)

### 1.3 Safety prerequisites (follow the warning in `deploy/bridge.env.example`)

The arm is powered, **the workspace is clear**, **the emergency stop is within
reach**, and the gravity-compensation parameters are written.
**The first grip press sends `movej` back to `q_home`** — the arm moves by itself,
not under your control.

## 2. Four stages

### Stage 0: read-only preflight (does not touch hardware)

```bash
./scripts/up.sh --check-only
```

**What passing looks like**: no unexplained FAIL among the 15 items.

Expected items (explain them, do not necessarily clear them):

| Item | When it is FAIL/WARN | What to do |
| --- | --- | --- |
| `hardware` | Serial port not plugged in / not passed through | Plug in USB; on WSL `usbipd attach --wsl --busid <busid>` |
| `follow_tools` | `arm_console.py` not found | `export LITEARM_TOOLS=...` (see 1.2) |
| `teleop_python` | No conda environment | `export TELEOP_SERVER_PY=...`, or run the upstream `environments/setup.sh` |
| `display` | System service has no DISPLAY | Use `install.sh --user`, or `TELEOP_VIEWER=rviz` |
| `submodules` | Not initialised / modified in place | `git submodule update --init`; make upstream changes in their own repositories |
| `env:teleop` / `flags:follow_server` | Upstream renamed variables or arguments | The upstream was modified; check the submodule copy |

### Stage 1: pure simulation — verify only the frame path, do not touch the arm (**works without USB**)

```bash
TELEOP_ARM_HOST= ./scripts/up.sh          # explicitly empty = do not pass --real; up.sh starts the upstreams
```

Without a headset, use the fake headset to push frames:

```bash
python3 scripts/fake_headset.py --status
```

**What passing looks like**:

- The fake headset prints one status line per second: `fps≈90`, frame age in
  single-digit milliseconds, forwarding increasing, cost <1 ms, `teleop=connected`.
- On the teleop side, the model in the pybullet window follows the controller
  (only after grip is pressed).
- `link=following`, `arm=—` (the real arm is not connected yet).

To install a real headset when none is present: in `litearm-vr` run
`./build.sh quest install` (over adb), and enter the address by hand in the
headset's 3D panel — the `ws://<LAN IP>:8765` printed by the bridge.

### Stage 2: hardware connected but no commands — the arm is enabled but not following

First detach the bridge and test the serial port alone:

```bash
./submodules/litearm-vr-follow/run_stm32_comm_test.sh /dev/ttyACM0
```

Then let the bridge start the follow service (**real arm not connected**; teleop
stays in simulation):

```bash
TELEOP_ARM_HOST= ./scripts/up.sh --link    # start follow_server; teleop is still simulation and sends nothing
```

Note: in this stage `teleop.start` sends `start` to the follow service — **the
motors are powered and hold position**. The workspace check (section 1.3) applies
here too; "the real arm is not connected yet" is not an excuse.

**What passing looks like**: the `arm` line of `--ctl status` has `state` and
`q` (the bridge's 4 Hz `ping` succeeds), and `arm.connected=True`.

### Stage 3: real-arm following

```bash
./scripts/up.sh --link        # asks you to confirm "workspace clear, emergency stop in hand" (use --yes when non-interactive)
```

**The first time**: keep your hand on the emergency stop and start with **small
amplitudes** — the fake headset's default radius is only 3 cm.

**What passing looks like**: press grip -> the arm first sends `movej` back to
`q_home`, then follows the controller in small movements; release grip -> pause;
press grip again -> re-anchor from the current position (no jump).

### Stage 4: fallback acceptance (the M2 acceptance item; do not skip)

Stop the frames — the fake headset can reproduce "the headset froze" with one
command:

```bash
python3 scripts/fake_headset.py --drop-after 5 --status
```

**What passing looks like** (watch the bridge log for the times):

| Time | Expected |
| --- | --- |
| 200 ms (`FRAME_WARN_MS`) | `[frame_stall]` warning |
| 500 ms (`FRAME_FALLBACK_MS`) | `[fallback_hold]` — the arm **holds in place** (no movej) |
| After frames resume | `[fallback_cleared]`; the arm keeps holding and **grip must be pressed again** to continue |

A larger check: unplug the headset power / turn off Wi-Fi (= disconnect, not a
freeze); the effect should be the same.

> The fake headset was measured on this machine: `--drop-after` produced
> `frame_stall` at 202 ms and `fallback_hold` at 500 ms, matching the
> configuration (the `hold` failed then because that machine had no
> follow_server connected).

## 3. Troubleshooting table

| Symptom | Look first at |
| --- | --- |
| Headset/fake headset cannot connect | `--doctor`'s `lan_addr` (is the IP right?), `port_bridge` (is it occupied?) |
| fps exists but the arm does not move | `link.forward_enabled` (forwarding switch) -> `bridge.teleop_connected` -> `arm.state` (did it accept `start`?) -> whether the fake headset's per-frame delta clears the upstream deadband |
| Frames arrive and teleop connects, but the simulated model does not move | Did the fake headset press grip (press it after `--warmup`)? Is `headset.grip` true? |
| An upstream does not start | `PROCESS_LOG_DIR/<name>.log` — **the upstream script's own error text is there** |
| The real arm does not move but `arm.state=following` | Wrong `--mode` (see 1.1), or `--kp/--kd` (shared by `jf`/`mit`) not tuned |
| No fallback alarm ever appears | `fallback.guard_enabled`, `FALLBACK_ENABLED` |

## 4. What to do when something goes wrong

- **Do not `pkill` the follow service directly**: once it stops, nobody holds the
  motors. Stop through the bridge: `./scripts/up.sh --stop` (latch first, then
  stop the processes).
- Emergency stop: the state machine's `estop` (sends only `stop`, never `movej`).
- Return to simulation: leave `TELEOP_ARM_HOST=` empty, run
  `./scripts/up.sh --stop`, and start again.
- The bridge **does not restart upstream processes automatically** (they drive
  the arm; a crash is only reported) — see implementation trade-off 2 in the
  README.

## 5. What does not exist yet (it limits this bring-up)

- **The host application has no VR page** (no VR route in `litearm-studio`), and
  M4 headset feedback is not implemented — so this bring-up has **no graphical
  interface** and relies on `--ctl status`, the fake headset's `--status` line,
  the pybullet window, and logs. The "operator view" part of real-arm debugging
  waits for the VR page.
- The fake headset verifies **translation only**; orientation is always the
  identity quaternion (wrist orientation is not covered).
- When the headset cannot connect, the "human in the loop" part of the chain
  (grip semantics, trigger controlling the hand/gripper) cannot be verified.

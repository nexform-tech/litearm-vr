English | [简体中文](QUICKSTART.zh-CN.md)

# Quick Start

Bring the full VR teleoperation pipeline up from scratch — first in simulation, then
on real hardware.

> Run every command **from the repository root** (`litearm-vr/`). Replace `<PC-IP>` with
> this computer's LAN address, and the device names (`/dev/ttyACM0`, `can0`) with the
> ones on your setup.

---

## 0. Prerequisites

**Computer**

- Ubuntu 20.04 / 22.04, x86_64, with a graphical desktop (the pybullet preview needs one).
- Python 3 and [conda](https://docs.conda.io/) (or miniconda). The teleop service runs in
  its own `metavr-arm` environment (Python 3.10).
- **Optional:** ROS 2 Humble, only for the RViz2 visualization backend.

**Headset**

- A supported OpenXR VR headset with the LiteArm VR client (closed-source;
  distributed separately), on the same Wi-Fi / LAN as the computer.

**Real robot (optional)**

- LiteArm V4 7-DoF arm; optionally a gripper and an L6 dexterous hand.
- One arm interface: a USB serial port (`/dev/ttyACM*`, STM32 path) **or** a SocketCAN
  adapter (`can0`, SDK path). Both must not be used at the same time.
- CAN bus configured as classic CAN at 1 Mbps.
- Serial permission: add your user to the `dialout` group and log in again:

  ```bash
  sudo usermod -aG dialout "$USER"
  ```

- `sudo` for CAN interface setup and the one-time bridge install.

Simulation needs no hardware at all.

---

## 1. Get the code

```bash
git clone https://github.com/nexform-tech/litearm-vr.git
cd litearm-vr
```

All three components live in subdirectories. No `--recursive` is needed — they are
working trees, not git submodules.

---

## 2. Build the teleop environment (first time)

```bash
bash litearm-vr-teleop/teleop/environments/setup.sh   # creates/rebuilds conda env metavr-arm
cd litearm-vr-teleop/teleop && make smoke && cd -      # smoke test: prints SMOKE TEST PASSED
```

If conda is not on `PATH`, either activate it or pass `CONDA_ROOT=/path/to/conda` to
`setup.sh`. Corporate networks can override the package mirror with `CONDA_MAIN` and
`CONDA_R`; see the script header.

---

## 3. Run in pure simulation (no hardware)

```bash
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py
```

- Connect the VR headset client to `ws://<PC-IP>:8765`.
- Defaults: single arm, upright mount, pybullet IK, pybullet preview window, **no hardware**.
- Controls: hold `gripActive` to engage / anchor, release to pause; `trigger` edge toggles
  gripper close/open.
- Stop with `Ctrl-C`, or `make stop` inside `litearm-vr-teleop/teleop`.

No headset at hand? With the bridge running, feed it synthetic controller frames:

```bash
python3 litearm-vr-bridge/scripts/fake_headset.py --help
```

---

## 4. Bring up the bridge (optional)

The bridge relays headset frames to teleop over zenoh and adds link-status reporting,
frame-loss fallback, and supervised process start/stop. It is not required for a
simulation run.

```bash
cd litearm-vr-bridge
sudo ./scripts/install.sh     # one-time: venv, systemd unit, /etc/litearm-vr-bridge.env
./scripts/up.sh               # day-to-day: starts the bridge and runs --doctor
cd ..
```

On a graphical session without `sudo`, use `./scripts/install.sh --user`.

For the first real-arm bring-up, follow
`litearm-vr-bridge/docs/REAL-ARM-BRINGUP.md`.

---

## 5. Connect real hardware

> ⚠️ **Real hardware moves for real.** Confirm the arm is powered, the workspace is
> clear, and the e-stop is within reach.

### 5.1 Install the hardware-side SDKs

Each follow service imports a vendor SDK. Install it as a Python package, or point the
matching environment variable at the SDK's `src/` directory.

| Service | SDK | Override variable |
| --- | --- | --- |
| Arm, STM32 path (`run_stm32_server.sh`) | `litearm` (litearm-python) | `LITEARM_PY_SRC` |
| Arm, SDK path (`run_sdk_server.sh`) | `pylitearm` | — |
| Gripper (`run_gripper_server.sh`) | `litegrip` (litegrip-python) | `LITEGRIP_SDK` |
| Hand (`run_hand_server.sh`) | linkerbot SDK | `LINKERBOT_SDK` |
| Status aggregator | `eclipse-zenoh` (`pip install eclipse-zenoh`) | — |

Example, installing from a source checkout:

```bash
python3 -m pip install -e /path/to/litearm-python        # provides `litearm`
export LITEARM_PY_SRC=/path/to/litearm-python/src        # alternative to pip install
```

The launcher prints a clear error naming the missing SDK and the paths it searched.

### 5.2 Start the follow services

Start the follow services **before** teleop; `--real` cannot connect otherwise.

All defaults come from `litearm-vr-follow/follow.yaml`. Command-line flags override the
file, and the file overrides built-in constants.

**Arm** — choose one, they are mutually exclusive:

```bash
# STM32 path: control law in firmware, over serial.
# --mode j skips gain tuning; switch to jf/mit once the chain is verified.
litearm-vr-follow/run_stm32_server.sh --mode j --tcp-port 9001

# SDK path: control law on the host, over SocketCAN.
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
litearm-vr-follow/run_sdk_server.sh --iface can0 --tcp-port 9001
```

If the serial port is not `/dev/ttyACM0`, set `port` in `follow.yaml` (a
`/dev/serial/by-id/...` path is more stable than `ttyACM*`).

**Gripper** — CAN must be up first (classic CAN, 1 Mbps):

```bash
litearm-vr-follow/run_gripper_server.sh --can can0 --port 9003
```

**Dexterous hand:**

```bash
litearm-vr-follow/run_hand_server.sh --can can0 --port 9004
```

**Dual arm, bridge-managed (shortcut).** When both arms are local and started by hand,
`litearm-vr-bridge/scripts/start.sh` starts both follow services, the bridge, and teleop
in one command (`--stop` tears down in reverse). Defaults are
`left=/dev/ttyACM0:9001`, `right=/dev/ttyACM1:9002`; override with
`FOLLOW_{LEFT,RIGHT}_{SERIAL,PORT}`.

### 5.3 Connect teleop to the hardware

```bash
# arm only
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --real

# arm + gripper
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --real \
    --gripper --gripper-remote 127.0.0.1:9003

# arm + dexterous hand
litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --real \
    --hand --hand-remote 127.0.0.1:9004
```

The arm defaults to `9001` for a single arm; a dual-arm configuration uses left `9001` /
right `9002`. These must match the follow services' `--tcp-port` (or pass
`--real-left` / `--real-right` explicitly).

### 5.4 Status aggregation (read-only, safe to run alongside)

Publishes each follow service's measured status to zenoh
(`litearm/v4/{armA,armB}/vr_{arm,grip,hand}`). It never sends commands and never writes
to motors:

```bash
litearm-vr-follow/run_status_aggregator.sh \
    --arm armA=/dev/serial/by-id/usb-...-if00@127.0.0.1:9001 \
    --gripper armA=can0@127.0.0.1:9003

# watch the subscriptions in another terminal
python3 litearm-vr-follow/status_listen.py
```

### 5.5 Validate a 120° wrist turn on a single upright STM32 arm

Teleop reads `q_min/q_max` from the follower's `state` response and intersects
those bounds with the URDF limits for IK. The default wrist-turn settings are
5 mm of hand translation, ±20° of travel from the start of the turn for each
of J1–J4, and a joint speed limit of 1 rad/s. When wrist travel runs out, the
solver prioritizes TCP position and allows orientation error.

#### Update and start the services

Replace the paths for the current checkout and Python environment as needed.
Set `PR_NUMBER` to the pull request under test. Fetch its head into a separate
checkout and run both services from that checkout.

```bash
export OLD_REPO="/path/to/current/litearm-vr"
export FIELD_WT="$HOME/litearm-vr-wrist-test"
export TELEOP_DIR="$FIELD_WT/litearm-vr-teleop/teleop"
export RUN_DIR="$HOME/wrist-120/run-120"
export TELEOP_PY="$HOME/miniconda3/envs/metavr-arm/bin/python"

git clone --no-checkout https://github.com/nexform-tech/litearm-vr.git "$FIELD_WT" &&
git -C "$FIELD_WT" fetch origin "refs/pull/${PR_NUMBER:?请设置 PR 编号}/head" &&
git -C "$FIELD_WT" switch --detach FETCH_HEAD &&
mkdir -p "$RUN_DIR" &&
cp "$OLD_REPO/litearm-vr-follow/follow.yaml" "$RUN_DIR/follow.yaml" &&
cp "$OLD_REPO/litearm-vr-teleop/teleop/config/teleop.yaml" "$RUN_DIR/teleop.yaml"
```

If the installation sets `FOLLOW_CONFIG` or `--config`, copy the active
configuration files. Release the grip and confirm the arm is holding before
stopping the old teleop process and then the old follower. Use the same variables
in each terminal and reuse the existing Zenoh bridge and STM32 SDK environment.

Terminal A starts the follower.

```bash
FOLLOW_CONFIG="$RUN_DIR/follow.yaml" \
  "$FIELD_WT/litearm-vr-follow/run_stm32_server.sh" \
  --port /dev/ttyACM0 --mode jf --tcp-port 9001 \
  2>&1 | tee "$RUN_DIR/follow.log"
```

Terminal B reads the limits and starts sampling. Check that `limits.json` has
`valid: true` and both bound arrays contain seven values in radians. If startup
reports that the firmware limit query failed, the response reflects the follower
configuration and the firmware values still need to be verified.

```bash
python3 "$TELEOP_DIR/scripts/wrist_120_check.py" limits \
  127.0.0.1:9001 "$RUN_DIR/limits.json" &&
python3 "$TELEOP_DIR/scripts/probe.py" sample \
  --addr 127.0.0.1:9001 --hz 50 --out "$RUN_DIR/follow.jsonl"
```

Terminal C starts teleop. This example uses the flange as the TCP. When using
an offset tool, pass its offset in metres here and use the same offset for export.

```bash
python3 "$TELEOP_DIR/launch/run_teleop_webxr.py" \
  --metavr-py "$TELEOP_PY" --config "$RUN_DIR/teleop.yaml" \
  --source zenoh --arm-id armA --zenoh-mode peer --zenoh-connect tcp/127.0.0.1:7447 \
  --no-dual --no-side --ik pybullet --control-frame tool --tcp-offset 0 0 0 \
  --no-hand --no-gripper --real --real-left 127.0.0.1:9001 --real-right 127.0.0.1:9001 \
  --probe "$RUN_DIR/sim.jsonl" 2>&1 | tee "$RUN_DIR/teleop.log"
```

#### Record and review the trial

1. Clear the workspace, remove the payload, and keep the emergency stop within
   reach. Use the site's normal procedure to bring the arm near `sim.single_q_home`.
2. Hold the right grip and keep the controller still for about 1 s. With its
   position fixed, rotate it +120° around its local X axis at grip engagement
   over about 2 s. Hold for about 1.5 s, rotate back over about 2 s, and keep still
   for about 1 s before releasing the grip. Release immediately if the arm sweeps
   unexpectedly.
3. Record one grip interval and avoid pressing A. After releasing the grip,
   stop teleop and the sampler with Ctrl-C, then export the data.

```bash
python3 "$TELEOP_DIR/scripts/wrist_120_check.py" report "$RUN_DIR" \
  --urdf "$TELEOP_DIR/models/litearm_description/urdf/litearm_description.urdf" \
  --tcp-offset 0 0 0
```

- In `tcp.csv`, check that the controller angle peaked near 120°, controller
  translation stayed below the configured threshold, and `wrist_tracking` was
  active during the turn and hold.
- In `joints.csv`, check that the teleop target, the follower's received target,
  and the command sent to the firmware stayed inside the effective limits.
  Check each upstream joint's peak displacement. The trial exercises J7
  saturation when J7 approaches the effective boundary and stays within it.
- The `measured_fk` series in `tcp.csv` is computed using encoder measurements
  and the kinematic model. Compare its peak and final displacement with the
  bench's allowed position error. The offline PyBullet reference reached a
  20° J1 excursion, a −89° J7 minimum, and about 0.02 mm peak TCP displacement.

Keep the entire capture directory. To roll back, stop the new processes and
start the original commands from the previous checkout.

---

## 6. Stop

```bash
# bridge: holds the arm in place first, then reaps its supervised children
litearm-vr-bridge/scripts/up.sh --stop

# teleop
cd litearm-vr-teleop/teleop && make stop
```

Also stop the follow services with `Ctrl-C` in their terminals. Do not leave a follow
service running while another program takes over the serial port or CAN interface.

---

## 7. Troubleshooting cheat sheet

| Symptom | Check first |
| --- | --- |
| teleop runs but the arm does nothing | Is the headset connected to the WebSocket? Is `grip` held? With `--real`, are the follow services up? |
| Cannot connect to 8765 / 9001 | Port already in use; when the bridge owns 8765, teleop should use `--port 8766` |
| Bridge `--doctor` reports a failure | Fix each hint (serial/CAN, conda, `DISPLAY`, upstream paths, …) |
| Serial port will not open | Another process holding it? Is the user in `dialout`? |
| CAN will not open | Is the interface `up` at 1 Mbps? Is another program using it? |
| Gripper closes on power-up | `gripper_server` opens on start by design — do not pass `--no-startup-open` |
| `ModuleNotFoundError: litearm` / `litegrip` / `linkerbot` | Install the SDK or set `LITEARM_PY_SRC` / `LITEGRIP_SDK` / `LINKERBOT_SDK` |
| Teleop reports tracking error but the arm is fine | Check that the applied command (`cmd_q`) matches the measured `q`; the follow side may be limiting slew |
| pybullet window does not appear | No graphical session or `DISPLAY`; run from a desktop session on this computer |

For deeper troubleshooting, see each component's `README.md` and its `docs/`.

English | [简体中文](README.zh-CN.md)

# litearm-vr-follow

The **real-arm hardware driver service** for LiteArm: the arm receives seven
joint angles and follows them; the gripper receives `close` / `open` to drive
its jaws; the dexterous hand receives `close` / `open` plus left/right to make a
fist or open.

It is the layer closest to the hardware in the VR teleoperation chain:

```
litearm-vr-teleop  ──TCP :9001/9002──▶  arm follow service  ──▶  STM32  ──CAN──▶  arm motors
   (solves joint angles)                   (follow execution)
litearm-vr-teleop  ──TCP :9003───────▶  gripper service     ──▶  socketcan ──CAN──▶  gripper DM motor
   (trigger edge)                          (close/open)
litearm-vr-teleop  ──TCP :9004───────▶  dexterous hand service ──▶ linkerbot SDK ─CAN─▶ L6 hand
   (trigger edge)                          (close/open+side)

status_aggregator.py ──TCP :9001/9003─▶ each service (ping/status, read-only) ──Zenoh──▶ litearm/v4/arm/vr_arm
   (aggregates status, 100Hz/20Hz)         (reads status, never drives hardware)          litearm/v4/grip/vr_grip
```

It **does not solve IK and never touches headset data** — whatever the upstream
sends (joint angles or switch commands), the hardware does.

Besides "receive command -> drive hardware", this repository also carries a
**read-only status aggregator**: `status_aggregator.py` polls each service over
TCP, merges the measurements into a `FollowStatus`, and publishes it on Zenoh
(see below).

## Two versions, choose one

| | `arm_follow_server_stm32.py` | `arm_follow_server_sdk.py` |
| --- | --- | --- |
| To the arm | Serial `/dev/ttyACM0` | socketcan `can0` |
| Where the control law runs | In the STM32 firmware | On the host (`pylitearm`) |
| Extra dependency | `arm_console.py` (see below) | `pylitearm` |
| Start | `./run_stm32_server.sh` | `./run_sdk_server.sh` |
| Hardware-free self-test | `./run_stm32_comm_test.sh` | `--dry-run` |

Note: **the two versions are mutually exclusive** — they drive the same motors
and must not run at the same time.

Note: **the real arm really moves**. Before running, confirm the arm is powered,
the workspace is clear, and the emergency stop is within reach.

## Configuration: `follow.yaml`

Every `run_*.sh` takes its parameters from **`follow.yaml`** in the same
directory: edit that file and the scripts start with the new values, **without
passing arguments**:

```bash
./run_sdk_server.sh            # reads the arm_sdk section of follow.yaml
./run_gripper_server.sh        # reads the gripper section
```

Priority is **command line > follow.yaml > factory constants in the script**
(removing a key falls back to the factory value without error). The command line
can still override temporarily:

```bash
./run_sdk_server.sh --tcp-port 9002                       # override arm_sdk.tcp_port
FOLLOW_CONFIG=/path/to/other.yaml ./run_sdk_server.sh     # use another configuration
```

Section to script mapping:

| Section | Script | | Section | Script |
| --- | --- | --- | --- | --- |
| `arm_stm32` | `run_stm32_server.sh` | | `hand` | `run_hand_server.sh` |
| `arm_sdk` | `run_sdk_server.sh` | | `aggregator` | `run_status_aggregator.sh` |
| `gripper` | `run_gripper_server.sh` | | `stm32_comm` | `run_stm32_comm_test.sh` |

Every key and its valid values are documented in the comments of `follow.yaml`.
The parser `follow_config.py` uses only the standard library (no PyYAML);
`python3 follow_config.py --print <section>` shows the command line a section
would expand to.

## Gripper service (`gripper_server.py`)

The gripper DM motor (CAN ID 0x08, DM4310) is driven by `gripper_server.py`,
alongside the arm follow server (both belong to the "receive command, drive
motor" hardware layer):

```bash
./run_gripper_server.sh --can can0 --port 9003         # real gripper
```

Protocol: TCP, one JSON object per line. `{"cmd":"close"}` / `{"cmd":"open"}` /
`{"cmd":"ping"}` (replies `{"ok":true,"q":...,"tau":...}`) / `{"cmd":"status"}`
(**zero I/O**: returns the cached status, plus `enabled`/`error_code`/`fresh`/
`age_ms` and loop diagnostics `fault_code`/`target_rad`/`cmd_rad`/`t_mos`/
`t_coil`, suitable for 20 Hz polling).

`close` / `open` each do exactly one thing: **post a target angle** (`close` ->
calibrated closed limit, `open` -> calibrated open limit) and reply immediately.
The actual drive is **one ~200 Hz control loop** in the service (following
`litegrip-cpp`'s `ControlLoop`; see
[`docs/gripper-control-loop-rewrite.md`](docs/gripper-control-loop-rewrite.md)):
each tick reads feedback, checks the watchdog/temperature/fault, advances the
trajectory at `--speed`, splits the torque budget into kp/kd, clamps the
position in-frame, and sends an MIT frame.

Three points must be understood before changing this code (otherwise the old
problems return):

- **The `q` in the frame is the commanded trajectory, not the latest measured
  position.** Using the latest reading as the setpoint leaves a natural offset
  from the same-instant measurement (about 0.7 mm), and `kp × Δ` becomes a
  persistent force near 1 Nm that jams the gripper against the limit and causes
  the ~1.2 s relaxation oscillation.
- **The force requested in the frame is bounded by geometry**:
  `q_frame = measured + clamp(trajectory − measured, ±budget/kp)`. When the
  gripper hits a workpiece or a hard limit, the requested torque **does not
  exceed the budget** — **the grip force is `--force`**. When the workpiece
  yields, the error grows and the force is held automatically; no separate
  "force hold" mode is needed, and it never "gives up after one attempt".
- **The target is a variable, not a one-shot action**: changing the target
  overrides it, and the next tick moves toward the new target (direction change
  within 5 ms), with no "wait for this shard to finish" delay. **There are no
  shards, retries, or stall criteria.**

Parameters (all can be given on the command line; defaults are the conservative
starting point):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `--speed` | 50 mm/s | Maximum advance speed of the commanded trajectory (**not** the MIT frame's dq field, which is always 0) |
| `--force` | 20 N | **Torque budget** on the closing side (×0.1 = 2 Nm) = grip-force limit |
| `--rate-hz` | 200 | Control-loop period |
| `--kp` / `--kd` | 20 / 0.5 | Gain caps; the actual values are reduced by the budget for the worst case |
| `--max-pos-error-mm` | 10 | "Worst position error" used to allocate the budget: `kp <= budget/this`. **Smaller = stiffer** (for example 5 -> kp≈19.5) |
| `--max-fb-vel` | 1 rad/s | "Worst feedback velocity" used for the budget allocation (sets the kd cap) |
| `--feedback-timeout` | 0.5 s | How long without a new feedback frame counts as disconnected -> send zero-gain frames |
| `--temp-limit` | 80 °C | MOS/coil over-temperature limit -> safe stop (zero-gain frames) |
| `--status-hz` | 20 | Rate at which the status cache is refreshed while **no target has been received yet** (limp) |

> The jitter problem went through four implementations with different root
> causes; **read [`docs/gripper-jitter-analysis.md`](docs/gripper-jitter-analysis.md)
> before changing it**:
> 1. A hand-written "pure torque hold + open interrupt": with an empty gripper,
>    pure torque pushed the jaws into the closed hard limit, harder and harder;
>    then the first `open` segment moved too little and was misjudged as a stall
>    by the SDK, causing back-and-forth inching.
> 2. Switching to the SDK's `close()` / `open()`: the position loop pushed
>    directly and could crush the workpiece.
> 3. Using the SDK's `grasp()` with shards for `close`: each shard re-ran the
>    whole closing ramp, and the shard boundaries alternated loose and tight.
> 4. Idle keep-alive used the configured gain (kp=100) in a position loop: the
>    setpoint could not match the same-instant measurement, jamming the limit
>    and producing a ~1.2 s oscillation.
> Do not revert one of these to make a single symptom disappear.

**Disable at startup**: a DM motor powers up holding its factory zero
(0 ≈ closed), so the service **first sends one 0xFD disable frame** to leave the
gripper limp and avoid "the gripper closes as soon as power is applied".
Disabling only needs a connection, not calibration. **Before the first target
arrives**, the loop stays limp (no enable, no frames) and re-enables on demand
when the first target arrives. Pass `--no-startup-disable` to keep the old
behaviour.

**Open at startup**: the service posts one **open** target after starting. The
headset trigger is a **toggle**: the first press closes, so the gripper must
start open for the first press to mean "close". Note that this homing **does not
count as a received command**, so `target_ratio` in `status` remains `None`
after boot (the contract wants the operator's most recent command). When the
link is down or the gripper is uncalibrated, the target is discarded and the
gripper stays limp. Pass `--no-startup-open` to disable it.

**Never stop the stream silently**: with about 100 ms of bus silence the motor
latches into a 0xD communication-loss fault (after which it can neither report
feedback nor accept commands). Therefore every exit path (disconnect,
over-temperature, motor fault) sends **zero-gain frames** (not silence), for the
same reason as the C++ `safe_stop()`.

Depends on the `litegrip` SDK (the script adds `~/sl/litegrip-python/src` to
`sys.path`; override with `LITEGRIP_SDK`; an installed package needs no
setting). The SDK needs a JSON calibration file describing the travel; the
factory calibration is the default. A calibration measured on this machine must
be given explicitly (`--calib PATH` / `--mount normal|reverse` / environment
variable `LITEGRIP_CALIB`); **it refuses to move when uncalibrated**.

Note: the gripper CAN interface must be up first (1 Mbps classic):
`sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up`

## Dexterous hand service (`hand_server.py`)

The L6 hand is driven by `hand_server.py`, alongside the arm follow server and
the gripper service (the same "receive command, drive motor" hardware layer):

```bash
./run_hand_server.sh --can can0 --port 9004            # real hand
```

Protocol: TCP, one JSON object per line: `{"cmd":"close","side":"left"}` /
`{"cmd":"open","side":"right"}` / `{"cmd":"ping"}`. `side` is `"left"` or
`"right"`, so the two hands make a fist or open independently. It calls
`hand_grasp`'s `left_close/right_close/...` (**blocking**, about 1.5 s), so a
background thread executes queued commands serially (the two hands share one CAN
bus and cannot run in parallel anyway). Depends on the linkerbot SDK
(`hand_grasp` + `src/`), which `run_hand_server.sh` adds to `PYTHONPATH`
(default `$HOME/sl/linkerbot-python-sdk-main`, override with `LINKERBOT_SDK`).

Note: the hand CAN interface must be up first (1 Mbps classic):
`sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up`

## Status aggregator (`status_aggregator.py`)

This completes the **receive half**: it publishes the measured status of the arm
and gripper (q/dq/tau/temperature/fault; gripper position/torque/enabled) to
Zenoh, **split by arm identity**:

| Device | Topic |
| --- | --- |
| Arm | `litearm/v4/{armA,armB}/vr_arm` |
| Gripper | `litearm/v4/{armA,armB}/vr_grip` |
| Dexterous hand (v1 placeholder) | `litearm/v4/{armA,armB}/vr_hand` |

The middle segment `armA`/`armB` is the **arm identity**, aligned with the
bridge topic contract `litearm/v4/{arm_id}/{channel}`. **The follow service does
not know its arm identity** — it only knows the serial port it opened (reply
`port`) or the CAN interface (reply `can`), and never whether it is left or
right. Identity therefore cannot be inferred from replies and must be
**configured externally**:

```bash
# armA=left arm: serial @ follow endpoint; IFACE is only compared with the reply (warn once on mismatch)
./run_status_aggregator.sh \
    --arm armA=/dev/serial/by-id/usb-...-if00@127.0.0.1:9001 \
    --arm armB=/dev/serial/by-id/usb-...-if02@127.0.0.1:9002 \
    --gripper armA=can0@127.0.0.1:9003 --gripper armB=can1@127.0.0.1:9004
./run_status_aggregator.sh --gripper none                       # disable the gripper
./run_status_aggregator.sh --prefix litearm/v4                  # change the topic prefix
python3 status_listen.py                                        # wildcard subscribe for debugging
```

(The prefix `litearm/v4` can be changed with `--prefix`; each message is the
device status itself, with no envelope. The earlier merged **single**
`FollowStatus` (`litearm/v4/follow/status`) is **no longer published**.)

**Read-only**: it only sends `ping` / `status`, **never** `start`/`q`/`movej`/
`home`/`close`/`open`, and **never opens** the serial port or CAN interface. It
can run alongside the teleoperation sender without interference (separate TCP
connections).

- **Message types**: ROS-style `@dataclass` in `status_msgs.py` (`ArmStatus` /
  `GripperStatus` / `HandStatus`; the merged `FollowStatus` wrapper remains only
  for parsing/tests). Serialisation is `json.dumps(asdict(...))` — **not** the
  ROS2 wire format and not interoperable with real ROS2. `ArmStatus` normalises
  the stm32 and sdk `ping` replies (`fault` first, fall back to `flags`; `q` is
  accepted only when `len == 7`).
- **Arm identity `arm_id` (topic middle segment)**: `armA`/`armB`, supplied by
  the **bridge's arm list** through `ARM_ID=` in `--arm`/`--gripper`. The follow
  service itself cannot provide it — it only knows the serial port.
- **Device name `device`**: the physical identity in the payload, taken from the
  reply's serial/CAN interface (the arm stm32 `port`, such as `/dev/ttyACM0`;
  the sdk `iface`; the gripper `can`, such as `can0`), falling back to the
  configured interface when the reply has none. A mismatch with the configured
  `IFACE` produces **one warning** (not blocking).
- **Rates**: publish 100 Hz / arm ping 100 Hz / gripper status 20 Hz by default
  (`--publish-hz` / `--arm-hz` / `--gripper-hz`). The gripper uses the `status`
  cache (zero CAN I/O), so 20 Hz is cheap; arm ping reads the SDK's cached
  frames, also cheap.
- **Timeouts**: when a device has not updated for `--stale-ms` (default 1000),
  its published state sets `connected=false`, but **publishing continues**;
  a broken link reconnects with a 1 s backoff.
- **Zenoh**: uses the same `_base_config()` as `litegrip` — `mode=peer`, no
  multicast scouting/gossip, explicit `listen`/`connect`. Default
  `listen tcp/0.0.0.0:17449` (17447/17448 are already used by arm/gripper
  teleoperation). Subscribers must know this port (`--zenoh-listen` /
  `--zenoh-connect` can change it).

Depends on `zenoh` (pip `eclipse-zenoh`; the script exits with an error if it is
missing).

## External protocol

TCP, one JSON object per line (`\n` separated). Upstreams (`real_arm_client.py`
in `litearm-vr-teleop`, and `litearm-vr-bridge`) speak this contract.

### Commands

| Command | Effect | Reply |
| --- | --- | --- |
| `{"cmd":"start"}` | Start following: enable + read current angles + `following=True` | `{"ok":true,"cmd":"start","q":[7]}`; adds `"already":true` when already following; enable failure returns `{"ok":false,"error":...}` |
| `{"cmd":"q","q":[7]}` | One frame of seven joint targets, followed per frame | `{"ok":true,"cmd":"q"}` |
| `{"cmd":"latch"}` | Hold in place | `{"ok":true,"cmd":"latch"}` |
| `{"cmd":"stop"}` | Emergency stop | `{"ok":true,"cmd":"stop"}` |
| `{"cmd":"movej","q":[7]}` | Joint trajectory to target (**blocks** until `\|q-target\|<tol`) | `{"ok":bool,"cmd":"movej"}` |
| `{"cmd":"home"}` | Return home (**blocks** until `\|q\|≈0`) | — |
| `{"cmd":"ping"}` | Liveness probe | See below |

The difference between `q` and `movej`: **`q` is a per-frame streaming follow
target** (high rate, non-blocking), while **`movej` is a one-shot trajectory**
(blocking, waits for arrival).

> `movej` / `home` **block this connection**. The client either uses a separate
> connection or accepts that this connection is unresponsive meanwhile.

### `ping` reply — fields differ between versions

| Field | stm32 version | sdk version |
| --- | --- | --- |
| `state` | `mode_name`: `INIT` / `MOVE_J` / `MOVE_P` / `MOVE_JS` / `MOVE_MIT` / `MIT_ALL` / `EMERGENCY` | `ArmState.value`: `disconnected` / `moving` / `following` / ... |
| `flags` | Yes: list of set flag names (empty when healthy) | No |
| `fault` | No | Yes: `[(joint, err), ...]` (empty when healthy) |
| `q` | Yes | Yes |
| `target` / `anchor` / `cmd_q` | Yes | No |
| `joints` | Yes: `[(q, dq, tau, t_mos, t_coil, err), ...]` (per-joint temperature/fault) | No |

Consumers must **accept both**: fall back to `flags` when `fault` is absent.
Their semantics match — both are empty when healthy and non-empty on a fault.

## Dependency: `arm_console.py`

The stm32 version imports `from arm_console import (LiteArm, CMD_...)`. That file
is **not in this repository**; it comes from the `tools/` directory of the
[`litearm-stm32`](https://gitee.com/yudao_hz_1/litearm-stm32) repository (the
STM32 0xA5 protocol wrapper plus the `LiteArm` serial class).

Imports are resolved **first in the script's own directory, then through
`PYTHONPATH`**, so either placement works:

```bash
# Option 1: PYTHONPATH (what run_stm32_server.sh does)
export PYTHONPATH=/path/to/litearm-stm32/tools:$PYTHONPATH

# Option 2: copy arm_console.py next to this directory
```

> `run_stm32_server.sh` / `run_stm32_comm_test.sh` look for the `tools`
> directory in this order: `$LITEARM_TOOLS` -> a sibling `litearm-stm32/tools`
> -> `~/sl/litearm-stm32/tools`. When none matches, the script exits with an
> error; point `LITEARM_TOOLS` at it.

The sdk version depends on `pylitearm` (over `can0`). `run_sdk_server.sh`
contains path handling for cmeel `pin 4.1.0`; the comment explains why: to
bypass the old pinocchio shipped with ROS Humble and built against numpy 1.x.

## Running

```bash
# Everyday use: edit follow.yaml and run
./run_sdk_server.sh
./run_gripper_server.sh
./run_hand_server.sh
./run_status_aggregator.sh

# Temporary overrides (command line > follow.yaml)
./run_stm32_server.sh --mode js --tcp-port 9001               # skip gain tuning
./run_sdk_server.sh  --dry-run                                # verify the protocol without hardware
```

`--mode` has four choices matching the firmware's four follow methods: `jf`
(joint follow, **default**) / `js` / `j` / `mit`.

Note: **the serial port is exclusive**. Do not run `cat /dev/ttyACM0` or leave
`arm_console` holding the port while the service runs.

## Safety pitfalls (learned the hard way, do not undo)

- **`latch` must use `movej` to the current angles**, so the firmware increases
  stiffness after arriving in `MOVE_J`. **Do not use `move_js` to hold** — its
  integral feedforward accumulates steady-state error at rest and causes a limit
  cycle.
- `stop` uses the firmware's `EMERGENCY_STOP`.
- Targets outside the joint limits are clamped into range and logged (rate
  limited to one line per 2 s); they never drive hard into a limit.

## Origin

The content was split out of the `vr` aggregate repository (`e3c9145` created
2026-09-16, `dbebe58` updated 2026-10-14). The gripper service was originally
`gripper_server_can.py` on the industrial PC and was folded into this repository
as `gripper_server.py` on 2026-10-15 (the same day it switched to the
`litegrip-python` SDK instead of the hand-written `damiao_socketcan` loop). The
dexterous hand was originally `scripts/hand_control.py` in the teleop repository
(local CAN driver) and moved here the same day as `hand_server.py` under the
same rules. `status_aggregator.py` / `status_msgs.py` / `status_listen.py` were
added on 2026-10-15 (status aggregation plus Zenoh publishing), with
**additive-only** small changes to `arm_follow_server_stm32.py` (`joints` in
ping) and `gripper_server.py` (`status` cache reads). Since then this repository
is the only home for these files; the `vr` repository is dormant.

### Teleoperation feedback and hold without switching modes

`{"cmd":"state","request_id":123}` is a read-only query: it does not enable or
home anything. The reply contains `cmd=state`, the unchanged `request_id`,
`valid`, `q` (seven measured joint angles in radians), and `age_s`. When
invalid, `q`/`age_s` are null; `target`/`cmd_q` are never returned as if they
were measurements. On STM32 the value becomes valid only after consecutive
queries observe the firmware status sequence advance; it is re-validated after
250 ms without progress or after an interrupted query. The SDK path requires
`get_state()` to provide `feedback.joints[].fresh/age_s` for all seven axes; an
older SDK without that metadata explicitly returns invalid.

`{"cmd":"hold"}` discards a distant old teleoperation target and prefers a fresh
measured angle; when feedback is unavailable it holds the currently commanded
values. It keeps the existing follow mode and gains and never calls movej or
re-enables. STM32 clears the velocity feedforward; the SDK keeps its
joint_follow loop (with no feedback the provider returns None and the SDK's
existing convention holds the current `q_cmd`). This command is used when grip
is released or teleoperation input becomes invalid.

The teleoperation client queries asynchronously through `request_state(side)`
(internally limited to 20 Hz); `get_state(side, max_age=0.25)` returns a valid
measured state or None. The client accounts for both the server-reported
feedback age and the request round-trip time, and clears the cache immediately
on disconnect. Deployment must update the server and client together; a backend
that does not support the new query or cannot prove feedback freshness is not
used as a closed-loop measurement source.

Repeated `hold` commands keep the position selected by the first one and do not
re-anchor on every measurement-noise sample; during a hold the client resends
hold as a keep-alive through its background query cycle. On the SDK path a
follow target not updated for more than 250 ms also clears the old target.
The SDK read-only pre-connection requires `connect(read_only=True)`; `start`
converts it to a normal connection.

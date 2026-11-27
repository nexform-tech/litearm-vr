English | [简体中文](README.zh-CN.md)

# litearm-vr-bridge

The resident service that `litearm-studio` (the LiteArm host application) uses
in **VR teleoperation scenarios**.

In one sentence: **the shim between the headset and teleop**. It receives
controller frames from the headset and publishes them on Zenoh for teleop to
solve IK, summarises the link state for the host application, accepts its
actions, and owns dropout fallback and managed-process control.

```
                    ┌──────────── litearm-vr-bridge ────────────┐
headset ──ws 8765──▶│ 1 raw frames ──▶ zenoh litearm/v4/{arm}/vr_frame│ ──▶ teleop (subscriber)
                    │                        │                  │ ──TCP 9001──▶ follow ──▶ real arm
                    │ 2 status      ──▶ .../vr_status (host subscribes)│ ◀── ping / start / latch
                    │ 3 actions     ◀── .../vr_rpc (host queries)     │
                    │ 4 dropout fallback + process start/stop        │
                    └───────────────────────────────────────────┘
```

**Both external edges are on Zenoh** (frames to teleop; status and actions to
and from the host application). Port 8765 serves only the headset (Godot speaks
only WebSocket); the bridge-to-follow link remains TCP 9001.

The host application does not appear on that diagram as a direct peer: it joins
the Zenoh network through a `zenoh-bridge-remote-api` instance (browsers can
only use WebSocket; see `docs/DEPLOY.md`). **`ctl` does not need that router** —
it runs on the same machine as the bridge and connects to the bridge's Zenoh
listening endpoint; see "Quick start" below.

## Status

**M1, M2 and M3 are implemented and covered by tests; the real-arm run is
recorded in [docs/acceptance.md](../docs/acceptance.md).**
The full design is in [docs/DESIGN.md](docs/DESIGN.md), deployment in
[docs/DEPLOY.md](docs/DEPLOY.md), and **read
[docs/REAL-ARM-BRINGUP.md](docs/REAL-ARM-BRINGUP.md) before the first real-arm
connection** (it covers choosing the machine, the control law, and the
step-by-step bring-up).

| Milestone | Status |
| --- | --- |
| M1 forwarding and status (8765 frames to Zenoh `vr_frame`, `vr_status` summary, `forward_cost_ms`) | Done |
| M2 actions and fallback (`teleop.start/stop`, `estop`, 200/500 ms warnings and automatic hold) | Done |
| M3 process control and systemd (`process.start/stop`, `deploy/`, `scripts/`) | Done — `ctl up/down`, latch stop and 90 fps forwarding recorded in [`docs/acceptance.md`](../docs/acceptance.md) §3 |
| M4 headset feedback | Not started (depends on `litearm-vr` wiring `message_received`) |

## Quick start

```bash
sudo ./scripts/install.sh     # one-time: create the venv, install systemd, generate /etc/litearm-vr-bridge.env
                              # (use ./scripts/install.sh --user for a graphical session or without sudo)

./scripts/up.sh               # the everyday command
./scripts/up.sh --link        # attach the real-arm chain (see "real-arm staging" below)
./scripts/up.sh --status      # print one link snapshot
./scripts/up.sh --stop        # stop (it latches in place first, then stops the children)
```

`up.sh` loads the env file, runs the preflight (`--doctor`), starts the bridge,
starts the chain according to the rules, and prints the address to enter on the
headset.

For a **dual-arm real robot** the bridge cannot manage two follow endpoints (it
holds a single `FOLLOW_SERVER_ENDPOINT`). `scripts/start.sh` combines "start two
follow servers plus `up.sh --link`" into one command:

```bash
./scripts/start.sh          # start left/right follow -> bridge + teleop; --stop reverses it; --status prints the state
```

Defaults are `left=/dev/ttyACM0:9001` and `right=/dev/ttyACM1:9002`; override
with `FOLLOW_{LEFT,RIGHT}_{SERIAL,PORT}`. The follow repository is looked up at
a sibling `../litearm-vr-follow` (or set `FOLLOW_DIR=`).

**Real-arm staging.** When teleop drives the real arm (the bridge passes
`--real` to the launcher), `up.sh` **does not** start the upstreams
automatically; that step requires an explicit `--link` (non-interactive
environments also need `--yes`, and it asks you to confirm that the workspace is
clear and the emergency stop is within reach). Simulation (explicitly empty
`TELEOP_ARM_HOST`, so `--real` is not passed) starts everything in one command.

The reason is that the bridge's default stance is "the shim does not change the
existing link behaviour", and the same reasoning as trade-off 2 below (no
automatic restart) applies to automatic startup: coupling "monitoring shim" and
"power on the real-arm control stack" into one action is not appropriate. To
make the bridge leave the upstream processes completely alone, set
`TELEOP_CMD=` and `FOLLOW_SERVER_CMD=` explicitly empty (`config.py` treats that
as "I do not want this process"), and `teleop.start` only opens forwarding.

### Preflight

```bash
./scripts/up.sh --check-only     # preflight only, exits 1 on failure (a normal start runs it too)
```

It asks the field questions in advance: port already in use, submodules not
initialised, upstream modified in place, serial port or CAN missing, pybullet
missing from conda, `arm_console.py` not found, no `DISPLAY` so the pybullet
window cannot open, and so on.

It also performs **two kinds of argument checks**: for the follow service, where
the bridge builds the command line and passes arguments through to the upstream
script, it compares the arguments it will pass with the `add_argument` entries
declared in that script; for teleop, whose arguments became environment
variables read by the upstream launcher, it checks whether the launcher reads
the variables the bridge sets. With the upstream pinned as a submodule, this
guards not only "the on-site upstream is older than the bridge assumes" but also
"the upstream changed its interface" and "someone edited the submodule in
place".

### Trigger

The managed processes are started **only when `teleop.start` arrives**, and the
VR page in the host application was removed per AGENTS.md, so nothing on site
can press that button. `--ctl` fills the gap (`up.sh` uses it too):

```bash
python3 -m litearm_vr_bridge --ctl up       # teleop.start: enable forwarding and start follow_server/teleop
python3 -m litearm_vr_bridge --ctl status   # print the interpreted fields of one snapshot
python3 -m litearm_vr_bridge --ctl down     # teleop.stop (latch) -> process.stop one by one
```

## Where the upstreams come from

The two upstream components the service starts and stops are **submodules** of
this repository, pinned to specific commits:

```
submodules/litearm-vr-teleop   teleop (formerly metavr): controller pose -> 7 joint angles (IK + 3D preview)
submodules/litearm-vr-follow   7 joint angles -> motors (real-arm follow service)
```

```bash
git clone --recursive git@gitee.com:yudao_hz_1/litearm-vr-bridge.git
git submodule update --init          # add this if the clone was made without --recursive
```

The paths are derived from the repository layout (`repo_root()` in
`config.py`), so no configuration is needed by default; set `TELEOP_REPO` and
`FOLLOW_REPO` to the respective repository roots only when the upstreams really
live outside the submodule directories.
**How** to start the upstreams is also delegated to their own launch scripts
(teleop's `launch/run_teleop_webxr.py`, follow's `run_stm32_server.sh`): the
conda environment, `arm_console`, ROS2, and dexterous-hand SDK paths are
resolved there, and the bridge does not duplicate that knowledge. To override
them, export the variables those scripts read (`TELEOP_SERVER_PY`,
`LITEARM_TOOLS`, and so on); child processes inherit them unchanged.

Upgrading an upstream means moving the pin explicitly:

```bash
git -C submodules/litearm-vr-teleop pull
git add submodules/litearm-vr-teleop && git commit -m "chore(submodule): teleop follow <sha>"
```

`--doctor` reports both submodule commits and whether either was modified in
place. In-place changes disappear on the next `submodule update`; make them in
the upstream repository instead.

## Running manually (development / troubleshooting)

```bash
python3 -m pip install --user "websockets>=13"   # websockets.asyncio.* exists from 13 onward
cd litearm-vr-bridge
git submodule update --init                      # the two upstream repositories

PYTHONPATH=src python3 -m litearm_vr_bridge --print-config   # show the effective configuration
PYTHONPATH=src python3 -m litearm_vr_bridge --doctor         # preflight
PYTHONPATH=src python3 -m litearm_vr_bridge                  # run the service in the foreground

PYTHONPATH=src python3 -m unittest discover -s tests -t .    # run the tests (171 items)
```

Minimal configuration (all environment variables are listed in DESIGN section 7):

```bash
BRIDGE_PORT=8765                     # only the headset connects here
ZENOH_ARM_ID=armA                    # topic segment; must match teleop and the host application
ZENOH_LISTEN_ENDPOINTS=tcp/0.0.0.0:7447   # teleop and the host router connect here
TELEOP_ENDPOINT=127.0.0.1:8766       # the upstream teleop switches to --port 8766; no code change needed
FOLLOW_SERVER_ENDPOINT=127.0.0.1:9001
```

`/healthz` is a read-only HTTP endpoint for monitoring (served on the headset
port). **When Zenoh is down it is the only out-of-band channel**, so `up.sh`
uses it for liveness instead of `--ctl status` (which needs Zenoh). The host
application subscribes to `litearm/v4/{arm}/vr_status`.

## Implementation trade-offs vs DESIGN

1. **`events` are not cleared after being pushed once; they are kept by TTL
   (20 s by default) with an upper bound.** The upstream `litearm-studio`
   consumes the status stream by overwrite (`snapshotValue = JSON.parse(...)`)
   and has no consumption acknowledgement; clearing after one push would swallow
   occasional alarms. The `event.clear` action clears them explicitly.
2. **Managed child processes are not restarted automatically.** Both teleop and
   the follow service drive the arm, so restarting them automatically could
   power the arm or home it without an operator (teleop homes on the first grip press).
   The bridge only reports `running/stopped/crashed`; keeping the bridge itself
   alive and restarting it on crashes is left to systemd.
3. **3D preview uses `STATUS_HZ` rather than a second message channel.** The
   host application treats every status message as a complete snapshot, so a
   separate `arm.q` message would be seen as a partial snapshot. A snapshot is
   only a few hundred bytes, so raising `STATUS_HZ` to 20–30 is enough for a
   smooth preview.
4. **`teleop.start` opens the follow service for the operator only on a cold
   start.** Immediately after `teleop.stop`, `estop`, or a fallback, teleop
   still holds the previous anchor (it never saw the grip release). Opening the
   follow service then would make teleop's `q` frames accepted immediately and
   servoed against the old anchor, causing a jump if the controller moved. In
   that case the bridge only enables forwarding and returns a
   `teleop_start_needs_regrip` event; the operator presses grip again to
   re-anchor before following. On a cold start (none of those states has
   occurred) it sends `start` as usual: the follow service's `q_target` is empty
   and it only holds in place until the operator presses grip.

The snapshot also carries several troubleshooting fields beyond DESIGN 4.1
(`bridge.teleop_connected`, `headset.frames_forwarded`/`frames_dropped`,
`link.state`, `processes_detail`, and so on); the host application can treat
them as optional (`litearm-studio` already types them as optional).

## What it is not

- It does not solve IK (that is in teleop).
- It does not drive the arm (that is in `arm_follow_server` / `litearm-server`).
- It is not a user interface (that is in `litearm-studio`).
- It does not push 90 Hz raw controller frames to the frontend (the frontend
  only needs the status summary).

## Related projects

| Project | Relationship |
| --- | --- |
| `litearm-studio` | Host frontend; the only interface consumer of this service |
| `litearm-vr-teleop` | Upstream (submodule): IK relay; the bridge forwards frames to it without modifying its code |
| `litearm-vr-follow` | Downstream (submodule): real-arm follow service; the bridge sends `start/hold/latch/stop` and polls `ping` |
| `litearm-vr` (Godot) | Headset side; the upstream data source of this service |
| `litearm-server` | Arm controller daemon (ws 7449), normally connected directly by the host application |

> Process names in the status snapshot remain `teleop` and `follow_server` to
> match the `TELEOP_*` configuration entries and the host application fields;
> `teleop` means the `litearm-vr-teleop` component.

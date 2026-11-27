English | [简体中文](DEPLOY.zh-CN.md)

# Deployment and operation

> Configuration keys are defined in [DESIGN.md](DESIGN.md) section 7; this page
> only covers how to run the service and how to confirm it is correct.

## 1. Dependencies and installation

The service needs only Python 3.10+ and `websockets>=13` (`websockets.asyncio.*`
and `websockets.http11` exist from 13 onward; version 12 fails to import).

**The two upstream repositories are submodules and must be cloned** (without
them the bridge runs but has nothing to manage):

```bash
git clone --recursive git@gitee.com:yudao_hz_1/litearm-vr-bridge.git
git submodule update --init          # add this if the clone was made without --recursive
```

```bash
# Option A: one command (pull submodules + create venv + write env + install systemd + enable at boot)
sudo ./scripts/install.sh
./scripts/install.sh --user          # graphical session / no sudo: install a user-level service

# Option B: run from source (development, troubleshooting)
cd /opt/litearm/litearm-vr-bridge
python3 -m pip install --user "websockets>=13"
PYTHONPATH=src python3 -m litearm_vr_bridge --print-config   # show the effective configuration
PYTHONPATH=src python3 -m litearm_vr_bridge --doctor         # preflight
PYTHONPATH=src python3 -m litearm_vr_bridge                  # start the service

# Option C: install as a package (provides the litearm-vr-bridge command)
python3 -m pip install --user .
litearm-vr-bridge
```

Run the tests (stdlib unittest, no pytest required):

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -t .    # 171 items
```

## 2. How the upstreams start: their own launch scripts

Both upstreams are submodules, and the bridge **does not assemble their command
lines**; it calls their own launch scripts:

```
submodules/litearm-vr-teleop/teleop/launch/run_teleop_webxr.py  <- the bridge passes --host/--port/--viewer/--real...
submodules/litearm-vr-follow/run_stm32_server.sh                <- the bridge passes --host/--tcp-port/--mode/--port through
```

This keeps the knowledge of the conda environment (`TELEOP_SERVER_PY` ->
activated metavr-arm -> `conda info --base` -> common roots), `arm_console.py`
(`LITEARM_TOOLS`), ROS2, and hand/gripper SDKs in one place upstream, removing a
whole class of "starts by hand but not as a service" failures on site. To
override those paths, export the variables the upstreams read in the env file;
managed child processes inherit them unchanged.

Running by hand (during troubleshooting) uses the same path as the bridge:

```bash
# Before: the headset connected directly to teleop's 8765
submodules/litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --port 8765

# Now: the headset connects to the bridge's 8765, and the bridge forwards frames to teleop's 8766
submodules/litearm-vr-teleop/teleop/launch/run_teleop_webxr.py --port 8766
```

**The upstream needs no code change** (`--port` is its own switch, and the
launcher accepts it). The address entered in the headset's 3D panel remains
`ws://<local-IP>:8765`.

> For the first real-arm connection follow
> [REAL-ARM-BRINGUP.md](REAL-ARM-BRINGUP.md): decide on the machine first
> (whose USB port, where the conda environment lives, whether the headset can
> reach it), then advance stage by stage.

### Notes on running under systemd

The upstream scripts are written for "a person sitting in front of the machine",
so three things need attention when managed (none fatal, all encountered on
site):

- **`run_teleop_webxr.sh` runs `pkill -f ws_echo_server` at startup**: it clears
  processes with the same name first. Normally the bridge is stopping the one it
  started last time; if you are also running one manually for debugging, it is
  cleared too.
- **The hand/gripper CAN interfaces are configured by the upstream scripts with
  `sudo ip link set ...`**: a systemd service does not necessarily have `sudo`
  (and should not). Before enabling `TELEOP_HAND` / `TELEOP_GRIPPER`, configure
  the interface first:
  `sudo ip link set can0 up type can bitrate 1000000` (already-up interfaces are
  left alone).
- **ROS2 publication is decided upstream**: when `/opt/ros/humble/setup.bash` is
  sourced and `rclpy` is available, teleop publishes `/joint_states` (otherwise
  it warns and skips). The bridge has **no** knob to disable it; adding one is an
  upstream change.

## 3. systemd persistence

One command (creates the venv, replaces the `/opt` paths in the unit with the
real paths, and generates the env file):

```bash
sudo ./scripts/install.sh                    # system service
./scripts/install.sh --user                  # user-level service (systemctl --user)
```

Manual installation:

```bash
sudo cp deploy/litearm-vr-bridge.service /etc/systemd/system/
sudo cp deploy/bridge.env.example /etc/litearm-vr-bridge.env
sudoedit /etc/litearm-vr-bridge.env          # at least confirm FOLLOW_SERVER_MODE and the serial/CAN port
                                             # (TELEOP_REPO/FOLLOW_REPO are needed only when the upstreams are outside the submodules)
sudo systemctl daemon-reload
sudo systemctl enable --now litearm-vr-bridge
systemctl status litearm-vr-bridge
journalctl -u litearm-vr-bridge -f
```

`KillMode=mixed` in the unit is deliberate: stopping the service lets the bridge
finish "if following -> latch in place first -> then stop teleop / the follow
service" before the rest are signalled.

**System service vs user service.** teleop's pybullet window
(`TELEOP_VIEWER=gui`) opens only in a graphical session, and a system service
has no `DISPLAY` by default. `install.sh` writes the current session's
`DISPLAY`/`XAUTHORITY` into a drop-in; a manual install must add them, or use
`--user` (a user service inherits the graphical session, and
`loginctl enable-linger $USER` makes it start at boot).

**Env file format**: `KEY=VALUE`, one per line, comments must occupy a **whole
line** (trailing comments are not supported), and values containing spaces may
be quoted. This is not arbitrary — systemd's `EnvironmentFile=` reads it this
way, and `up.sh` uses the same semantics; both must agree. Do not write
`$(...)`: this is not a shell.

Note: early versions of `bridge.env.example` had trailing comments, and with
`EnvironmentFile=` the comments became part of the value, silently ignoring
options such as `TELEOP_SIDE` (an unparsable value falls back to the default).
That is fixed, but re-check any file copied from an old installation.

## 4. Smoke checks (on site)

```bash
# 0. Preflight: ports / upstream scripts / argument checks / serial or CAN / dependencies / graphical session
./scripts/up.sh --check-only

# 1. Is the bridge alive? (HTTP, not WebSocket)
curl -s http://127.0.0.1:8765/healthz | python3 -m json.tool

# 1'. Or let the bridge report: link state, headset, arm, processes, fallback, recent alarms
./scripts/up.sh --status

# 2. Status stream from the host application's perspective (zenoh; the client connects directly, no router)
python3 - <<'PY'
import json, zenoh
cfg = zenoh.Config()
cfg.insert_json5("mode", '"client"')
cfg.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
cfg.insert_json5("scouting/multicast/enabled", "false")
cfg.insert_json5("scouting/gossip/enabled", "false")
seen = []
session = zenoh.open(cfg)
session.declare_subscriber("litearm/v4/armA/vr_status",
    lambda s: seen.append(json.loads(bytes(s.payload))))
import time; time.sleep(0.5)
for snap in seen[-3:]:
    print(snap["owner"], snap["link"]["state"], snap["headset"]["fps"],
          snap["headset"]["latency_ms"], snap["bridge"]["forward_cost_ms"])
session.close()
PY

# 3. After the headset connects, expect fps ~90 and a single-digit last_frame_age_ms
```

Interpretation:

| Symptom | Meaning |
| --- | --- |
| `bridge.forward_cost_ms` > 1 ms | This service is stalled (CPU, logging, contention) |
| `headset.latency_ms` grows while `forward_cost_ms` is normal | The delay is in the headset or the wireless link |
| `headset.online=false` with `last_frame_age_ms` rising | The headset dropped or disconnected |
| `fallback.active=true` | No frame for over 500 ms; the arm is held in place |
| `link.forward_enabled=false` | Someone pressed "stop teleoperation" (the headset still sends, but nothing is forwarded) |

## 5. Cold start to teleoperation (M3)

```bash
./scripts/up.sh            # start the bridge -> preflight -> print the headset address
./scripts/up.sh --link     # real-arm chain: send teleop.start and start the upstreams per configuration
```

Both upstreams default to the submodules, so no paths are needed; when they live
elsewhere set `TELEOP_REPO` / `FOLLOW_REPO` (or the two `*_CMD` variables).

After `--link`, the bridge performs these steps in order (the `teleop.start`
branch of `bridge.handle_action`):

1. Start `follow_server` (9001) and **wait until its port really accepts
   connections** before continuing (on real hardware it waits for an STM32
   status frame, which takes seconds; doing this out of order sends `start` to a
   port with no listener and loses it).
2. Start `teleop` (8766) — with the real arm (`--real`) it first emits a yellow
   warning event; teleop **does not home automatically** at startup, and only
   the first grip press sends `movej` back to `q_home` before following.
3. Open frame forwarding and send `start` to the follow service.
4. The operator puts on the headset and presses the right grip to start
   following.

> By default `up.sh` performs this step automatically only in **simulation**;
> when the generated teleop command carries `--real`, it stops and asks you to
> confirm the workspace is clear and the emergency stop is within reach before
> running `--link`. To keep the bridge away from the upstream processes
> entirely, set `TELEOP_CMD=` / `FOLLOW_SERVER_CMD=` explicitly empty.

> To keep teleop away from the real arm: set `TELEOP_ARM_HOST=` explicitly
> empty, and it runs pybullet simulation only.

**Re-entering teleoperation** (after `--ctl down`, `teleop.stop`, or an
emergency stop): sending `teleop.start` again (`./scripts/up.sh --link` or
`--ctl up`) only opens frame forwarding and returns a
`teleop_start_needs_regrip` event in the status stream; teleop still holds the
previous anchor, so the operator must **press grip again** (release, then press)
to re-anchor and continue following. Recovery after a headset dropout works the
same way. This avoids the jump of "chasing the controller from an old anchor".

## 6. Troubleshooting

Run `./scripts/up.sh --check-only` first: ports, submodules, both argument
checks, serial/CAN, dependencies, and DISPLAY are all reported at once.

- **The headset cannot connect**: confirm `BRIDGE_PORT` is free, the firewall
  allows it, and the headset panel address is `<local-IP>:8765`.
- **An upstream does not start**: check `processes_detail` for `exit_code` and
  `PROCESS_LOG_DIR/<name>.log` — the upstream launch script's own errors
  ("arm_console.py not found", "cannot find the metavr-arm environment python")
  are in that log; fix them there.
- **`--doctor` reports `env:teleop` / `flags:follow_server`**: the upstream no
  longer accepts the arguments (or environment variables) we pass — usually a
  submodule modified in place, or an upstream interface change; the preflight
  names the specific items.
- **`--link` reports "the follow service did not accept start"**: follow_server
  started but is not listening on 9001 (crashed, serial port missing, or still
  waiting for an STM32 status frame for more than 8 s). Its log is in
  `PROCESS_LOG_DIR/follow_server.log`.
- **The status page shows fps but the arm does not move**: check
  `link.forward_enabled` (stopped?), `bridge.teleop_connected` (is the forwarding
  target present?), and `arm.state` (did the follow service accept `start`?).
- **No fallback alarm ever appears**: check `FALLBACK_ENABLED`;
  `fallback.guard_enabled` in the snapshot reports it faithfully.
- **`process.*` returns "not configured"**: neither path is available — the
  submodules were not initialised (`git submodule update --init`) and
  `TELEOP_REPO` / `FOLLOW_REPO` were not given; or that process's `*_CMD` was
  set explicitly empty (see the notes in section 4 of the env example).
- **The follow service keeps failing ping**: `arm.error` contains the reason
  (connection refused / timeout / malformed reply).
- **Logs are too noisy**: `LOG_LEVEL=WARNING`; failures while the follow service
  is unavailable are already rate-limited to one line per 5 s.

## 7. Known boundaries

- The bridge **does not solve IK and does not drive the arm**; fallback only
  sends `hold` / `stop`, never `movej` / `home`.
- Only **one** headset connection is forwarded at a time (a later one replaces
  the earlier one, and the log says so).
- Managed processes **do not restart automatically**: they drive the arm, so a
  crash is only reported and a human decides whether to restart (persistence and
  crash restart for the bridge itself are systemd's job).
- There is no authentication on the LAN (DESIGN 8.4). Port 9001 has always been
  "whoever connects can control the arm"; do not expose it to untrusted
  networks.

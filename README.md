# LiteArm VR Host Application Guide

[English](README.md) | [简体中文](README.zh-CN.md)

Applies to: the on-machine VR teleoperation environment. The current machine is an **upright single arm + gripper** (`arms = "single"`, `side = false`, `grip = "can0@127.0.0.1:9003"`).

> All user paths in this document are written as environment variables or relative paths, never bound to a username:
> `$HOME`, `${XDG_CONFIG_HOME:-$HOME/.config}`, `${XDG_STATE_HOME:-$HOME/.local/state}`.
> Inside the repository, relative paths are always used.

## Download and install

The Debian package for this release is attached to the [Releases](https://github.com/nexform-tech/litearm-vr/releases) page:

```bash
sudo apt install ./litearm-vr-bundle_0.1.0_amd64.deb
```

The package installs the bridge, upstream teleop/follow, the frozen teleop binary, the SDKs, the `litearm-can` DKMS driver (requires `dkms`, `build-essential` and matching kernel headers), udev rules and systemd units; `postinst` configures the DKMS module and installs the bridge as a user-level systemd service for the installing user.

Health check after installation:

```bash
/opt/litearm/litearm-vr-bridge/scripts/up.sh --doctor
```

---

## 0. Set environment variables first (once per new terminal)

```bash
export STUDIO_DIR="${HOME}/vr/litearm-vr-bridge/studio"      # host application source/build directory (ships with the bridge repo)
export BRIDGE_DIR="${HOME}/vr/litearm-vr-bridge"              # bridge source checkout (includes the preset scripts)
export BRIDGE_INSTALL="${LITEARM_BRIDGE_INSTALL:-/opt/litearm/litearm-vr-bridge}"  # installed bridge
export BRIDGE_CONFIG_DIR="${XDG_CONFIG_HOME:-${HOME}/.config}/litearm-vr-bridge"
export BRIDGE_STATE_DIR="${XDG_STATE_HOME:-${HOME}/.local/state}/litearm-vr-bridge"
export VR_STUDIO="${HOME}/.local/bin/litearm-vr-only"          # launcher
```

You can put these in `~/.bashrc` so that new terminals do not need them set again.

---

## 1. Launch the host application

### Graphical interface

Search for **"LiteArm VR"** in the application menu and open it.

### Terminal

```bash
"${VR_STUDIO}"
```

The launcher first makes sure the bridge service is running, then opens a window with this application's frontend.

> The source daemon requires `pywebview`; when it is missing the launcher prints an explicit prompt:
> `python3 -m pip install --user pywebview`.

### Simulation dry run (does not touch the real arm)

```bash
"${VR_STUDIO}" --fake
```

> ⚠ **Do not use the original "LiteArm Studio" icon for VR teleoperation.**
> That is the full version; its daemon connects to and occupies `/dev/ttyACM0` automatically. During VR teleoperation this serial port must be left to the bridge's follow service.
> This launcher passes `--port /dev/ttyDOES_NOT_EXIST` precisely so the daemon does not grab the serial port.
> The status, joint angles and motor loads on the VR page all come from the bridge's zenoh channel; the daemon does not need to connect to the arm itself.

---

## 2. Connect the headset and start teleoperation

Headset WebSocket address (the machine's IP is detected automatically):

```bash
echo "ws://$(hostname -I | awk '{print $1}'):8765"
```

For example, if it prints `ws://192.168.31.6:8765`, enter that into the headset panel.

Order of operations:

1. Open the host application and check the **"臂映射" (Arm Mapping)** column on the right:
   - Arm: `armA`
   - Gripper: `can0`
   - Serial port: leave empty = auto-detect; you can also select `/dev/serial/by-id/...` manually
2. Once the headset is online, click **"▶ 开始遥操" (Start Teleoperation)**.
3. The bridge brings up automatically:
   - the follow service (occupies the arm's serial port)
   - the gripper service (`can0:9003`)
4. Headset controller:
   - **press grip = start following, release grip = pause**
   - **press trigger = close gripper (make a fist), release trigger = open gripper**

> The first grip press runs `movej` back to the zero position first, then starts following. This is the arm's own motion, not real-time controller input.
> Make sure the arm is powered, nobody is in the workspace, and the emergency stop is within reach before pressing.

If teleoperation exited unexpectedly earlier (the page shows "兜底持位", fallback hold): **click "开始遥操" once more** to bring it back up.

---

## 3. Open / close the gripper

The gripper does not need to be started separately — **it is brought up together with follow when you click "开始遥操"**, and the bridge verifies that it is actually enabled.

| Action | Controller |
| --- | --- |
| Close | **press trigger** |
| Open | **release trigger** |

Notes:

- This page has **no button to open/close the gripper manually** (the manual control page was removed as required); the gripper is controlled only by the headset trigger.
- The **Gripper** row under "电机负载" (Motor Loads) in the left column shows the current reading; "臂映射" (Arm Mapping) on the right shows which CAN port it uses.
- If the gripper does not move, check:
  ```bash
  ip -br link | grep can0
  tail -40 "${BRIDGE_STATE_DIR}/logs/gripper_server_armA.log"
  ```
  If `can0` is absent, or the log says it is not enabled, check the CAN box/cable first.

---

## 4. Editing the configuration file

Configuration file:

```bash
"${BRIDGE_CONFIG_DIR}/config.toml"
```

Priority, from highest to lowest:

1. `overrides.json` (items changed on the host application's "链路配置" (Link Configuration) page)
2. `config.toml`
3. Factory defaults in the code

### 4.1 Use a preset (first choice when changing the machine model)

The preset scripts live in the **source checkout** (the installed 0.1.0 package does not include this script):

```bash
cd "${BRIDGE_DIR}"
./scripts/preset.sh list
./scripts/preset.sh single-upright-grip     # upright single arm + gripper
```

The script will: back up the old configuration → validate → write `${BRIDGE_CONFIG_DIR}/config.toml` → restart the bridge.
Backup files are named `config.toml.bak-YYYYmmdd-HHMMSS`.

### 4.2 Edit by hand

```bash
nano "${BRIDGE_CONFIG_DIR}/config.toml"
systemctl --user restart litearm-vr-bridge
```

Minimal configuration for the current machine:

```toml
[machine]
arms = "single"
side = false          # upright
real = true

[arms.A]
grip = "can0@127.0.0.1:9003"
```

It is recommended to pin the serial port (to avoid auto-detection ambiguity):

```bash
ls /dev/serial/by-id/
```

Write the STM32 path you see (TOML **does not** expand environment variables, so an actual path is required):

```toml
[arms.A]
device = "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_XXXX-if01"
grip = "can0@127.0.0.1:9003"
```

Restart after editing:

```bash
systemctl --user restart litearm-vr-bridge
```

### 4.3 Change settings on the host application page

The **"链路配置" (Link Configuration)** button in the top-right corner of the VR page edits these directly:

- `machine.arms` / `machine.side` / `machine.real`
- `arms.A.device` / `arms.A.grip`
- `follow.mode` / `follow.ctrl_mode`
- `bridge.log_level`, etc.

Items changed on the page are written to:

```bash
"${BRIDGE_CONFIG_DIR}/overrides.json"
```

It takes **precedence over** `config.toml`. To make `config.toml` take effect again:

- click "恢复默认" (Restore Defaults) in the dialog, or
- ```bash
  rm -f "${BRIDGE_CONFIG_DIR}/overrides.json"
  systemctl --user restart litearm-vr-bridge
  ```

Items such as `machine.arms` require a bridge restart; the dialog marks them `apply=restart`.

### 4.4 Verify the change

```bash
# Effective configuration (including overrides)
PYTHONPATH="${BRIDGE_INSTALL}/src:${BRIDGE_INSTALL}/vendor" \
  python3 -m litearm_vr_bridge --print-config

# Health check: configuration source / serial port / CAN / processes / parameters
cd "${BRIDGE_INSTALL}" && ./scripts/up.sh --doctor
```

In `--doctor`, `7447` / `8765` showing "in use" is normal — the bridge itself is listening.

---

## 5. Full configuration options

### 5.1 File locations and precedence

Searched in order; the first one found wins:

| Order | Location |
| --- | --- |
| 1 | `--config PATH` (explicitly specified on the command line) |
| 2 | `${XDG_CONFIG_HOME:-$HOME/.config}/litearm-vr-bridge/config.toml` |
| 3 | `/etc/litearm-vr-bridge.toml` |

If none of the three exists = all factory defaults are used, and the bridge still starts (a valid state).

Items changed on the host application's "链路配置" page are written to:

```bash
"${XDG_CONFIG_HOME:-${HOME}/.config}/litearm-vr-bridge/overrides.json"
```

Precedence: **`overrides.json` > `config.toml` > factory defaults**.

> ⚠ A TOML file **does not expand** environment variables such as `$HOME`. The `${...}` in this document is only used in shell commands;
> paths written into `config.toml` must be actual paths (for example `/dev/serial/by-id/...`).

### 5.2 Apply modes

| Apply mode | Meaning |
| --- | --- |
| `live` | Takes effect immediately |
| `respec` | Rebuilds the command line of the affected managed processes; refuses while the link is busy and applies once it becomes idle |
| `restart` | Requires a bridge restart after saving: `systemctl --user restart litearm-vr-bridge` |
| Locked in UI | Cannot be changed on the page; must be written into `config.toml`. Safety / network / executable-type settings, to avoid breaking things or losing contact remotely |

### 5.3 `[machine]` — what this machine is

| Key | Type / values | Default | Apply | Editable in UI | Description |
| --- | --- | --- | --- | --- | --- |
| `real` | bool | `true` | respec | Yes | `true` = connect to the real arm; `false` = run pybullet simulation only, does not touch the arm |
| `side` | bool | `false` | respec | Yes | Single-arm mounting direction: `false` = upright, `true` = side-mounted; only effective for a single arm |
| `viewer` | enum: `gui` / `rviz` | `gui` | respec | Yes | Viewer for upstream teleop; `rviz` requires ROS2 |
| `hand` | bool | `false` | respec | Yes | Dexterous hand switch |
| `hand_remote` | text: `host:9004` | empty | respec | Yes | Dexterous hand service address (when `hand = true`) |
| `gripper` | bool | `false` | respec | Yes | Upstream-level gripper switch (usually paired with `gripper_remote`) |
| `gripper_remote` | text: `host:9003` | empty | respec | Yes | Gripper service address (when `gripper = true`) |
| `upright` | bool | `false` | respec | Yes | Upright mounting for dual arms; only effective when `arms = "dual"` |
| `arms` | enum: `single` / `dual` | `single` | restart | Yes | Arm configuration; takes effect after restarting the bridge |
| `server_py` | text | empty = auto-detect | restart | No (locked) | Interpreter for the upstream teleop server (python from the metavr-arm environment) |

> The current single-arm gripper uses `[arms.A].grip = "can0@127.0.0.1:9003"`.
> `machine.gripper` / `gripper_remote` are upstream-level switches and usually should **not** duplicate `[arms.A].grip`.

### 5.4 `[follow]` — how the follow service runs

| Key | Type / values | Default | Apply | Editable in UI | Description |
| --- | --- | --- | --- | --- | --- |
| `mode` | enum: `stm32` / `sdk` | `stm32` | respec | Yes | `stm32` = serial; `sdk` = CAN |
| `ctrl_mode` | text: `jf` / `js` / `j` / `mit` | `js` | respec | Yes | Corresponds to upstream `--mode`; `jf` requires board firmware ≥ 1.9.0 |
| `serial` | text | empty | respec | Yes | Shorthand for the hardware port of a single arm; **must not** be set together with `[arms.*].device` |
| `dry_run` | bool | `false` | respec | Yes | Solve normally but do not send commands to the arm |

### 5.5 `[arms.A]` / `[arms.B]` — ports for each arm

For a single arm, write only `[arms.A]`; for dual arms, A = left (left controller), B = right (right controller).

| Key | Type / values | Default | Apply | Editable in UI | Description |
| --- | --- | --- | --- | --- | --- |
| `device` | text | empty = auto-detect | respec | Yes | Arm hardware port; `/dev/serial/by-id/...` is recommended, `ttyACM*` can drift |
| `grip` | text: `can0@host:9003` | empty = no gripper | respec | Yes | CAN port@service address for this arm's gripper; when the address is local, the bridge manages the gripper service as well |
| `host` | text | empty = local machine | respec | Yes | Host where the follow service runs (for cross-machine deployments) |
| `port` | int | A=`9001`, B=`9002` | respec | Yes | Follow service port |
| `id` | text | A=`armA`, B=`armB` | restart | No (locked) | Arm identity / middle segment of the zenoh topic; changing it disconnects the host application |

Example for the current machine:

```toml
[arms.A]
# device = "/dev/serial/by-id/usb-...-if01"   # empty = auto-detect; pin it to the actual by-id path when possible
grip = "can0@127.0.0.1:9003"
```

### 5.6 `[safety]` — real-hardware acknowledgement gate

| Key | Type / values | Default | Apply | Editable in UI | Description |
| --- | --- | --- | --- | --- | --- |
| `require_real_ack` | bool | `false` | restart | No (locked) | When `true`, `teleop.start` must carry `real_ack: true`, otherwise it is refused; the page cannot turn it off by itself |

### 5.7 `[bridge]` — the bridge itself

| Key | Type / values | Default | Apply | Editable in UI | Description |
| --- | --- | --- | --- | --- | --- |
| `host` | text | `0.0.0.0` | restart | No (locked) | WebSocket listen address |
| `port` | int | `8765` | restart | No (locked) | WebSocket port the headset connects to |
| `arm_id` | text | `armA` | restart | No (locked) | Middle segment of the zenoh topic; must match teleop and the host application |
| `zenoh_listen` | text | `tcp/127.0.0.1:7447` | restart | No (locked) | zenoh listen endpoint; change to `tcp/0.0.0.0:7447` for a host application on another machine |
| `zenoh_connect` | text | empty | restart | No (locked) | zenoh endpoint to actively connect to; leave empty for same-machine deployment |
| `log_level` | enum: `DEBUG` / `INFO` / `WARNING` / `ERROR` | `INFO` | live | Yes | Log level, effective immediately |
| `log_format` | enum: `text` / `json` | `text` | restart | Yes | Log format |
| `log_file` | text | empty | restart | Yes | Also write logs to a file |
| `manifest_file` | text | empty = same directory as `arms.json` | restart | Yes | Arm mapping record file |

### 5.8 `[upstream]` — which upstream processes are managed

| Key | Type / values | Default | Apply | Editable in UI | Description |
| --- | --- | --- | --- | --- | --- |
| `manage` | bool or list | `true` | restart | Yes | `true` / `false`, or an explicit list such as `["teleop", "follow", "gripper", "status"]` |
| `teleop_repo` | text | unset = use the submodule in the repo | restart | No (locked) | Upstream teleop path |
| `follow_repo` | text | unset = use the submodule in the repo | restart | No (locked) | Upstream follow path |
| `teleop_frozen` | text | unset = use the frozen artifact in the package if present | restart | No (locked) | Frozen teleop executable; an empty string = force not to use it |

### 5.9 `[upstream.env]` — environment variables for managed child processes

This section is the **only** place whose contents actually become environment variables; key names are arbitrary and values must be strings:

```toml
[upstream.env]
LITEARM_PY_SRC = "/opt/litearm/follow"     # read by follow's run_stm32_server.sh
LITEGRIP_SDK = "/opt/litearm/litegrip/src" # read by the gripper service
CMEEL_SITE = "/opt/cmeel"                  # read in sdk mode
TELEOP_PYTHONPATH = "/opt/extra"           # extra import path
# GRIPPER_SERVER_CMD = "..."               # custom gripper service command
```

> This section is **locked in the UI** and can only be written in `config.toml`; the host application's "链路配置" does not provide editing.
> The bridge's own keys (`machine.*` / `follow.*` / `arms.*` / `bridge.*` / `upstream.*`) **must not** be written here;
> putting them here raises an immediate error naming the offending key.

### 5.10 Cross-key rules

- `side` is only meaningful for `arms = "single"`.
- `[follow] serial` and `[arms.*].device` are two spellings of the same thing; **set only one**.
- A single arm recognizes only `[arms.A]`; only dual arms write `[arms.B]`.
- `machine.arms`, `upstream.manage`, `bridge.log_format` / `log_file` / `manifest_file`
  require a **bridge restart**.
- A wrong section / key / value raises an **immediate error naming it**; it does not silently fall back to the default.

---

## 6. Status / logs / restart

```bash
# Service status
systemctl --user status litearm-vr-bridge

# Link summary: headset frame rate, arm device, follow, fallback state
cd "${BRIDGE_INSTALL}" && ./scripts/up.sh --status

# Live log
journalctl --user -u litearm-vr-bridge -f

# Per-process logs
ls "${BRIDGE_STATE_DIR}/logs/"
tail -f "${BRIDGE_STATE_DIR}/logs/teleop.log"
tail -f "${BRIDGE_STATE_DIR}/logs/follow_server_armA.log"
tail -f "${BRIDGE_STATE_DIR}/logs/gripper_server_armA.log"

# Restart the bridge
systemctl --user restart litearm-vr-bridge
```

---

## 7. FAQ

| Symptom | Handling |
| --- | --- |
| "开始遥操" is greyed out | The headset is not online, or the bridge is offline; check "VR 链路" (VR Link) on the page and `up.sh --status` first |
| The headset cannot connect to `ws://<IP>:8765` | Confirm the headset and this machine are on the same subnet; `ss -ltnp \| grep 8765`; check whether this machine's IP has changed |
| The page shows "跟随服务报错: Connection refused" | This is normal before "开始遥操" is clicked; follow only starts listening on 9001 after starting |
| teleop exits unexpectedly / the page shows "兜底持位" | Click "开始遥操" again; if it keeps exiting, check `teleop.log` (common cause: the teleop pybullet window was closed manually) |
| The gripper does not move | `ip -br link \| grep can0`; check `gripper_server_armA.log`; confirm that `grip = "can0@..."` is in the configuration |
| `ls /dev/serial/by-id/` is empty | The arm is not plugged in properly or was disconnected, or there is a USB cable / power problem; replug and check again |
| The serial port is occupied and follow will not start | `cd "${BRIDGE_INSTALL}" && ./scripts/up.sh --doctor`; it names the process holding the port |
| `config.toml` changes do not take effect | Check whether `overrides.json` overrides it (see 4.3) |

---

## 8. Safety notes

- The first grip press performs a **`movej` back to the zero position**; confirm that nobody is in the workspace and the emergency stop is within reach.
- The **emergency stop** on the page sends a stop command to follow.
- In real-hardware mode (`real = true`), "开始遥操" really drives the arm; when trying out the interface, use `"${VR_STUDIO}" --fake` first.
- Releasing grip only pauses following, it does not cut power; if the arm needs to stay put, let it hold position itself.

# LiteArm VR Host Application Guide

[English](README.md) | [简体中文](README.zh-CN.md)

Applies to: the on-machine VR teleoperation environment. The current machine is an **upright single arm + gripper** (`arms = "single"`, `side = false`, `grip = "can0@127.0.0.1:9003"`).

> **Nothing to configure by hand — no environment variables to set.** The packages pin every path, so this document always writes the defaults.
> You only touch `XDG_CONFIG_HOME` / `XDG_STATE_HOME` when the configuration or state has to live somewhere else (shared account, read-only home).

| Purpose | Default location |
| --- | --- |
| Bridge (program + scripts + SDKs) | `/opt/litearm/litearm-vr-bridge` |
| Bridge configuration | `~/.config/litearm-vr-bridge/config.toml` (no file = all factory defaults) |
| Overrides written by the host application | `~/.config/litearm-vr-bridge/overrides.json` |
| Arm mapping record (serial port / gripper CAN) | `~/.config/litearm-vr-bridge/arms.json` |
| Managed-process logs | `~/.local/state/litearm-vr-bridge/logs/` |
| Host application | Application menu **LiteArm VR**, or the `litearm-studio` command |

> The bridge reads **no** environment variables (the single exception is `[upstream.env]` inside `config.toml`, see 5.9).
> The old `/etc/litearm-vr-bridge.env` is no longer read; if it is still around, delete it.
> `LITEARM_PY_SRC` / `LITEGRIP_SDK`, which the upstream launch scripts look for, are written into the systemd unit by the install script — you never export them.

---

## Download and install

Delivery is one directory: the **bridge deb** (its `Depends` carries the CAN driver `litearm-can-dkms`, shipped alongside it), plus the **host application** `litearm-studio` deb. See the [Releases](https://github.com/nexform-tech/litearm-vr/releases) page.

```bash
sudo apt install ./litearm-vr-bridge_0.1.2_amd64.deb ./litearm-can-dkms_0.1.2_all.deb
```

- Needs network access for `dkms` and matching kernel headers (`linux-headers-$(uname -r)`). **No git, no source checkout required.**
- With Secure Boot enabled the system refuses the unsigned CAN driver; disable it or sign the module.
- `postinst` configures the DKMS module and installs the bridge as a **user-level** systemd service for the user who ran the install, then starts it.
  When it cannot tell which user that is (nobody logged in, no terminal `sudo`) it does not guess — it prints the one command to run as yourself:
  `/opt/litearm/litearm-vr-bridge/scripts/install.sh`.
- Upgrade: the same command; the bridge restarts itself and an existing `config.toml` is **not** overwritten.

The host application (the UI) is a separate package — install it from [the litearm-studio releases](https://github.com/nexform-tech/litearm-studio/releases):

```bash
sudo apt install ./litearm-studio_<version>_amd64.deb
```

After that `litearm-studio` is on `PATH` and **LiteArm VR** appears in the application menu. The window is drawn by the system's GTK / WebKitGTK (pulled in as apt dependencies); no browser or `pywebview` to install.

Confirm the service is running after installation:

```bash
systemctl --user status litearm-vr-bridge
journalctl --user -u litearm-vr-bridge -n 50 --no-pager   # why it failed to start
```

> The package's `scripts/` also ships a few helpers (`up.sh` health check / status,
> `preset.sh` machine presets, `ports.sh` boards and serial ports). They are thin wrappers
> around the `systemctl` / `journalctl` commands used below — **nothing in this document
> depends on them being present**.

---

## 1. Launch the host application

### Graphical interface

Search for **"LiteArm VR"** in the application menu and open it.

### Terminal

```bash
litearm-studio
```

It prints the listening address and opens a window. Launching it again while an instance is running does not start a second process — it raises that window.
**Closing the window exits the UI**; the bridge is a separate systemd service and keeps running (see §6 to stop it).

### Simulation dry run (does not touch the real arm)

```bash
litearm-studio --fake
```

> ⚠ During VR teleoperation the arm's serial port must be left to the bridge's follow service.
> If the application's own daemon grabs it (symptom: the page cannot reach the arm, `follow` will not start), launch it without an arm:
>
> ```bash
> litearm-studio --port /dev/ttyDOES_NOT_EXIST
> ```
>
> The status, joint angles and motor loads on the VR page all come from the bridge's zenoh channel; the UI does not need to connect to the arm itself.

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
  tail -40 ~/.local/state/litearm-vr-bridge/logs/gripper_server_armA.log
  ```
  If `can0` is absent, or the log says it is not enabled, check the CAN box / cable first (`dkms status` should list `litearm-can` as `installed`).

---

## 4. Editing the configuration file

Configuration file:

```bash
~/.config/litearm-vr-bridge/config.toml
```

Priority, from highest to lowest:

1. `overrides.json` (items changed on the host application's "链路配置" (Link Configuration) page)
2. `config.toml`
3. Factory defaults in the code

### 4.1 Use a preset (first choice when changing the machine model)

The preset script lives in the package's `scripts/`:

```bash
/opt/litearm/litearm-vr-bridge/scripts/preset.sh list
/opt/litearm/litearm-vr-bridge/scripts/preset.sh single-upright-grip   # upright single arm + gripper
```

The script will: back up the old configuration → validate → write `~/.config/litearm-vr-bridge/config.toml` → restart the bridge; backups are named `config.toml.bak-YYYYmmdd-HHMMSS`.
Without the script (minimal install / source checkout) just edit `config.toml` by hand as in 4.2.

### 4.2 Edit by hand

```bash
nano ~/.config/litearm-vr-bridge/config.toml
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
ip -br link | grep can                     # the CAN port the gripper is on
```

(The package also ships `scripts/ports.sh`, which lists all of these in one table and can watch plug events with `-w`; the two commands above are enough without it.)
Write the STM32 path you see (TOML **does not** expand `$HOME` or similar, so an actual path is required):

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
~/.config/litearm-vr-bridge/overrides.json
```

It takes **precedence over** `config.toml`. To make `config.toml` take effect again:

- click "恢复默认" (Restore Defaults) in the dialog, or
- ```bash
  rm -f ~/.config/litearm-vr-bridge/overrides.json
  systemctl --user restart litearm-vr-bridge
  ```

Items such as `machine.arms` require a bridge restart; the dialog marks them `apply=restart`.

### 4.4 Verify the change

```bash
systemctl --user restart litearm-vr-bridge
systemctl --user status litearm-vr-bridge
journalctl --user -u litearm-vr-bridge -n 30 --no-pager
```

The startup log spells out, line by line: the headset entry point and zenoh endpoints, each arm's
serial port and follow endpoint, the managed-process command lines, and **which items are overridden
by `overrides.json`** (it prints the corresponding value in `config.toml`). When "my change did
nothing", read those lines first.

### 4.5 Advanced: swapping `follow.yaml` / `teleop.yaml` (gripper force, feel, calibration)

The bridge's own settings live in `config.toml`; **gripper / follow settings live in `follow.yaml` and
teleop settings in `teleop.yaml`**. The packaged copies sit under `/opt` and an upgrade overwrites them,
so the procedure is "copy them into your home directory + tell the bridge to read that copy":

```bash
mkdir -p ~/litearm-config
cp -n /opt/litearm/litearm-vr-bridge/submodules/litearm-vr-follow/follow.yaml \
      ~/litearm-config/follow.yaml
cp -n /opt/litearm/litearm-vr-bridge/submodules/litearm-vr-teleop/teleop/config/teleop.yaml \
      ~/litearm-config/teleop.yaml
```

Then append these lines to the **end** of `~/.config/litearm-vr-bridge/config.toml`
(`[upstream.env]` is the section name — keep the brackets):

```toml
[upstream.env]
FOLLOW_CONFIG = "/home/<your-user>/litearm-config/follow.yaml"
TELEOP_CONFIG = "/home/<your-user>/litearm-config/teleop.yaml"
```

```bash
systemctl --user restart litearm-vr-bridge
```

- `FOLLOW_CONFIG` is read by follow's own launch scripts (follow / gripper / status aggregator all read it); `TELEOP_CONFIG` is translated by the bridge into the upstream launcher's `--config`. Values **must be absolute paths**: `~` and `$HOME` are not expanded, and a wrong value fails immediately.
- This section only accepts **names the upstream launch scripts know** (besides these two, e.g. `LITEARM_PY_SRC`); the bridge's own keys (`machine.*` …) are rejected with a pointer to the right section.
- An upgrade only overwrites the two YAML files under `/opt`; whatever is in `~/litearm-config/` is untouched — no need to touch the packaged copies or to create symlinks.
- Restart the bridge after editing; the teleop process is only started when you click "开始遥操" on the page.

Frequently changed items (defaults taken from the two YAML files in the current package):

| What you want | File | Key | Default |
| --- | --- | --- | --- |
| Gripper gripping force limit (N) | `follow.yaml` | `gripper.force` | `20.0` (grip force ≈ this value; ×0.1 = Nm torque budget) |
| Gripper open/close speed limit (mm/s) | `follow.yaml` | `gripper.speed` | `100.0` |
| Gripper moves the wrong way | `follow.yaml` | `gripper.mount = "reverse"` | unset (`normal`) |
| Gripper motor replaced, CAN ID changed | `follow.yaml` | `gripper.can_id` | `"0x08"` |
| Gripper closes by itself at power-up | `follow.yaml` | `gripper.no_startup_open` | `false` |
| Gripper over-temperature protection | `follow.yaml` | `gripper.temp_limit` | `80` (°C) |
| Arm jitters while the controller is still | `teleop.yaml` | `tuning.deadband_scale` | `0.1` (`1.0` = raw thresholds; `0.0` = deadband off, tracking noise passes straight through — do not) |
| Following feels drifty / sluggish | `teleop.yaml` | `tuning.smooth_alpha` | `0.2` (`1.0` = most responsive and jitteriest, `0.5` = more responsive; **never 0**) |
| Grasp point height changed | `teleop.yaml` | `tuning.tcp_offset` | `[0.0, 0.0, 0.18]` (metres, calibrated) |
| Joint jumps when swinging fast | `teleop.yaml` | `sim.max_step_rad` | `0.2` (rad/frame) |
| Switch IK backend | `teleop.yaml` | `tuning.ik` | `pybullet` (also `pin` / `exact`) |
| Controller directions wrong | `teleop.yaml` | `mapping.*` | calibrated values — do not tweak by feel |

> Every key of the two YAML files (including `tuning.notch` / `twist_accel`, `pin_ik`, `aggregator`, …)
> is covered by the "LiteArm VR Bridge parameter reference" (`litearm-vr-bridge-config.md`) shipped
> with the delivery. Sections marked "calibrated" were measured against a real arm; re-calibrate when
> changing arms or grippers.

---

## 5. Full configuration options

### 5.1 File locations and precedence

Searched in order; the first one found wins:

| Order | Location |
| --- | --- |
| 1 | `--config PATH` (explicitly specified on the command line) |
| 2 | `~/.config/litearm-vr-bridge/config.toml` |
| 3 | `/etc/litearm-vr-bridge.toml` |

If none of the three exists = all factory defaults are used, and the bridge still starts (a valid state).

Items changed on the host application's "链路配置" page are written to:

```bash
~/.config/litearm-vr-bridge/overrides.json
```

Precedence: **`overrides.json` > `config.toml` > factory defaults**.

> ⚠ A TOML file **does not expand** environment variables such as `$HOME`.
> Paths written into `config.toml` must be actual paths (for example `/dev/serial/by-id/...`).

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
| `server_py` | text | empty = auto-detect | restart | No (locked) | Interpreter for the upstream teleop server; the package ships a frozen teleop, so leave it empty normally |

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
`[arms.A]` defaults to the **left (side-mounted)** slot; a single **upright** arm must also state
`side = false` so the arm takes the right slot (that is what the `preset.sh` presets do). A swapped slot
does not raise an error — it just means the wrong controller drives the arm.

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
| `teleop_repo` | text | unset = use the copy inside the package | restart | No (locked) | Upstream teleop path; the package ships it (pinned submodule) |
| `follow_repo` | text | unset = use the copy inside the package | restart | No (locked) | Upstream follow path; the package ships it (pinned submodule) |
| `teleop_frozen` | text | unset = use the frozen artifact in the package if present | restart | No (locked) | Frozen teleop executable; an empty string = force not to use it |

### 5.9 `[upstream.env]` — environment variables for managed child processes (normally unnecessary)

The install already writes these into the systemd unit, so day to day this section is unused:

```bash
grep '^Environment=' ~/.config/systemd/user/litearm-vr-bridge.service
# Environment=PYTHONPATH=/opt/litearm/litearm-vr-bridge/src:/opt/litearm/litearm-vr-bridge/vendor:/opt/litearm/litearm-vr-bridge/vendor-upstream
# Environment=LITEGRIP_SDK=/opt/litearm/litearm-vr-bridge/sdk/litegrip-python/src
# Environment=LITEARM_PY_SRC=/opt/litearm/litearm-vr-bridge/sdk/litearm-python/src
```

Write into it only in these cases (key names are arbitrary, values must be strings); the full recipe
for swapping `follow.yaml` / `teleop.yaml` is in 4.5:

```toml
[upstream.env]
# Swap the two upstream YAML files (absolute paths only; never edit the packaged copies,
# an upgrade overwrites them)
# FOLLOW_CONFIG = "/home/me/litearm-config/follow.yaml"
# TELEOP_CONFIG = "/home/me/litearm-config/teleop.yaml"
# Your own source checkout / SDK, or a specific teleop interpreter
# LITEARM_PY_SRC = "/opt/litearm/follow"
# LITEGRIP_SDK = "/opt/litearm/litegrip/src"
# TELEOP_SERVER_PY = "/home/me/miniconda3/envs/metavr-arm/bin/python"
# Custom managed commands
# GRIPPER_SERVER_CMD = "..."
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
# Service status / restart / stop
systemctl --user status litearm-vr-bridge
systemctl --user restart litearm-vr-bridge
systemctl --user stop litearm-vr-bridge

# Live log (last 50 lines, then follow)
journalctl --user -u litearm-vr-bridge -n 50 -f

# Per-process logs
ls ~/.local/state/litearm-vr-bridge/logs/
tail -f ~/.local/state/litearm-vr-bridge/logs/teleop.log
tail -f ~/.local/state/litearm-vr-bridge/logs/follow_server_armA.log
tail -f ~/.local/state/litearm-vr-bridge/logs/gripper_server_armA.log
```

The link summary (headset frame rate, arm device, follow, fallback state) is on the VR page; when the
page will not come up, read the bridge's own output with `journalctl`.

> If the bridge was installed as a **system-level** service (unattended machines), drop `--user` from the commands above and use `journalctl -u litearm-vr-bridge` for the journal.

---

## 7. FAQ

| Symptom | Handling |
| --- | --- |
| "开始遥操" is greyed out | The headset is not online, or the bridge is offline; check "VR 链路" (VR Link) on the page and `systemctl --user status litearm-vr-bridge` first |
| The headset cannot connect to `ws://<IP>:8765` | Confirm the headset and this machine are on the same subnet; `ss -ltnp \| grep 8765`; check whether this machine's IP has changed |
| The page shows "跟随服务报错: Connection refused" | This is normal before "开始遥操" is clicked; follow only starts listening on 9001 after starting |
| teleop exits unexpectedly / the page shows "兜底持位" | Click "开始遥操" again; if it keeps exiting, check `teleop.log` (common cause: the teleop pybullet window was closed manually) |
| The gripper does not move | `ip -br link \| grep can0`; check `gripper_server_armA.log`; confirm that `grip = "can0@..."` is in the configuration; `dkms status` should list `litearm-can` as `installed` |
| `ls /dev/serial/by-id/` is empty | The arm is not plugged in properly or was disconnected, or there is a USB cable / power problem; replug and check again |
| The serial port is occupied and follow will not start | `sudo fuser -v /dev/ttyACM*` shows who holds it; the host application's daemon may have grabbed it too (see §1); the bridge's own account of it is in `journalctl --user -u litearm-vr-bridge -n 100` |
| `config.toml` changes do not take effect | Check whether `overrides.json` overrides it (see 4.3) |
| No service unit after installing | `systemctl --user status litearm-vr-bridge`; if the unit does not exist, `postinst` failed to install it: run the package's `scripts/install.sh` once as your own user (that is the command it printed), or reinstall the deb |

---

## 8. Safety notes

- The first grip press performs a **`movej` back to the zero position**; confirm that nobody is in the workspace and the emergency stop is within reach.
- The **emergency stop** on the page sends a stop command to follow.
- In real-hardware mode (`real = true`), "开始遥操" really drives the arm; when trying out the interface, use `litearm-studio --fake` first.
- Releasing grip only pauses following, it does not cut power; if the arm needs to stay put, let it hold position itself.

English | [简体中文](acceptance.zh-CN.md)

This page records the acceptance evidence for the VR teleoperation stack: what was tested, on which hardware or environment, the measured results, and the known gaps that remain open.

## 1. Scope

The evidence covers the three components delivered by this repository:

- `litearm-vr-bridge`: headset ingress, Zenoh forwarding, status summary, dropout fallback, and managed-process start/stop.
- `litearm-vr-teleop`: controller-to-joint teleoperation, IK, preview, and the real-arm client.
- `litearm-vr-follow`: the real-arm follow service for the STM32 serial path and the SDK/CAN path.

## 2. Simulation chain

Run on the reference laptop with the Pico headset and PyBullet DIRECT:

- Teleop offline suite: 80/80 tests pass, including 21/21 anchored-tracking tests.
- Bridge offline suite: 168/168 tests pass.
- Anchor behaviour: fixed anchor, 100 round trips, cross-180° rotation, unreachable-target recovery, operator-lead handling, and execution-deviation protection.
- Preview tracking error (`|ee_after − target|`): median 0.9–1.6 mm, p90 3.2–8.6 mm.

## 3. Real-arm chain

Tested with a `Litearm1.12.3-7J` arm on `/dev/ttyACM0`, single-arm upright mount, STM32 follow service in `jf` mode, and follow slew set to 3.0 rad/s and 6.0 rad/s².

- `ctl up` starts the follow service and teleop with one command; the headset connected at 90 fps and the bridge reported 0.16 ms forwarding cost.
- Grip press/release produced `following`/`paused` events and `owner` changed from `none` to `vr`.
- `ctl down` latched in place first, then stopped teleop (3.3 s) and the follow service (0.46 s) without force.
- Target-vs-measured TCP tracking error by target speed:

| Target speed | Median | p90 |
| --- | --- | --- |
| < 20 mm/s | 9–11 mm | 17 mm |
| 20–100 mm/s | 12–20 mm | 66 mm |
| > 100 mm/s | 26–50 mm | 107 mm |

- Angle error: median 3–7°, p90 9–21°.
- Command-vs-measured execution error: median 12–18 mm.

The fast-motion lag comes from the follow slew limit, not from IK or the preview model. The slow-motion baseline is 9–17 mm.

## 4. Deployment bundle

The release bundle was built and installed in a clean Ubuntu 22.04.5 virtual machine:

- Artifacts: `litearm-vr-bundle_0.2.2_amd64.deb` (110 MB), `.tar.gz` (151 MB), SHA-256 sidecars, and `MANIFEST.json` with the source commit, SDK pins, and DKMS commit.
- `dpkg -i` succeeded; the bridge service was installed as a system service (`User=ubuntu`), active and enabled.
- A `kill -9` on the main process was recovered by `Restart=always` within 5 seconds.
- Upgrade simulation: removing a dependency from the venv and reinstalling the bundle restored it from the wheelhouse, while `/etc/litearm-vr-bridge.env` was preserved.

## 5. Known gaps

| Gap | Status | Evidence |
| --- | --- | --- |
| Gripper shard jitter, `open()` bounce | Code rewritten to a 200 Hz control loop; hardware verification pending a connected gripper | Issue #11 |
| DKMS `gs_usb` module load | Builds and installs; the test VM kernel lacks `can-dev`, so `modprobe` fails | Issue #12 |
| Follow lag during fast motion | Follow slew limit; expected behaviour, adjustable through `max_speed` / `max_accel` | Issue #10 |
| `litearm-server` (7449) coexistence | Design item pending hardware measurement | `litearm-vr-bridge/docs/DESIGN.md` §8 |
| URDF parity between host and teleop | Design item; teleop's URDF is the reference | `litearm-vr-bridge/docs/DESIGN.md` §8 |

## 6. Reproducing the checks

```bash
# Teleop tests (from litearm-vr-teleop/teleop)
PYTHONPATH=src python3 -m unittest discover -s tests -p "test_*.py"

# Bridge tests (from litearm-vr-bridge)
PYTHONPATH=src python3 -m pytest tests -q

# Follow protocol tests (from litearm-vr-follow)
python3 -m unittest test_measured_state.py
```

Real-arm acceptance follows `QUICKSTART.md` and `litearm-vr-bridge/docs/REAL-ARM-BRINGUP.md`.

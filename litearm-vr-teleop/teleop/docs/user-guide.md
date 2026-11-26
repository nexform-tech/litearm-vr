English | [简体中文](user-guide.zh-CN.md)

# User guide

## All options live in one file: `config/teleop.yaml`

Every tunable option, including units, valid ranges and descriptions, is in
`config/teleop.yaml`: rates, limits, initial pose, axis-mapping calibration,
IK solver parameters, deadband feel, real-arm addresses, and dexterous-hand and
gripper parameters. Both the launcher and the server read it.

Priority: **command line > configuration file > code defaults**. Therefore:

```bash
cp config/teleop.yaml config/my_arm.yaml   # edit this copy, leave the shipped file alone
./launch/run_teleop_webxr.py --config config/my_arm.yaml
./launch/run_teleop_webxr.py --port 9000   # override one item without editing the file
```

- Boolean entries support `--x` and `--no-x` (for example `--dual` and
  `--no-dual`); a `true` value in the file can still be turned off from the
  command line.
- Sections marked as **calibration values** (`mapping`, gripper travel, and
  `tuning.tcp_offset`'s grasp point when a gripper is fitted) were measured
  against the real arm. Do not change them without recalibrating: nothing
  errors, but the hand-to-arm correspondence becomes wrong.
- Removing a key from the configuration file does not error; that key falls
  back to the code default.

## Per-session flow (zenoh controller teleoperation)

1. **Start the bridge** (PC): `litearm-vr-bridge` (receives the headset on 8765
   and forwards to the Zenoh topic `litearm/v4/{arm}/vr_frame`). This repository
   reads frames from that topic by default.
2. **Start the server** (PC):
   ```bash
   ./launch/run_teleop_webxr.py      # one process: zenoh subscription + pybullet simulation
   ```
   A pybullet 3D window opens (drag to rotate, scroll to zoom) and the arm holds
   its initial pose. The terminal prints
   `data source: zenoh subscription litearm/v4/armA/vr_frame (...)`.
   To let the headset connect directly without the bridge, use `--source ws`
   (the headset then connects to `ws://<PC-IP>:8765/`).

3. **Connect the headset client**:
   - The Quest client connects to `ws://<PC-IP>:8765/` (the bridge's listening
     address, on the same Wi-Fi as the PC).
   - Controller data starts flowing (one JSON frame per update, containing
     `leftController` and `rightController`).
   - When the first frame arrives the terminal prints
     `[diag] first raw frame: ...`; only then are frames really arriving.

3. **Teleoperate**:
   | Action | Function |
   |---|---|
   | Right controller `gripActive` pressed | Start / resume (re-anchor) |
   | Right controller `gripActive` released | Pause |
   | Move / rotate the controller | Drive the arm |
   | Trigger | Gripper (the simulation currently ignores it) |

## Scene visualisation (pybullet window)

- The arm itself (URDF STL rendering)
- A grey visual ground plane (display only, no collision)

## Quick troubleshooting

| Symptom | Action |
|---|---|
| Cannot connect over ws | Check the IP, that the PC and headset are on the same subnet, that the **bridge** is running, and port 8765 (this repository listens only with `--source ws`) |
| The server runs but receives no frames | 1. Is the bridge running (the default source is the middle layer)? 2. Does `--arm-id` match the bridge's `ZENOH_ARM_ID` (a mismatch is silent and simply receives nothing)? 3. Does `--zenoh-connect` point at the bridge's listening endpoint (default `tcp/127.0.0.1:7447`)? 4. Does the terminal print `[diag] first raw frame`? As a fallback use `--source ws` and connect the headset directly, to confirm the headset is sending |
| The arm does not move | Press `gripActive` first; the controller must be tracked (keep it in the headset's field of view) |
| The arm is stuck | Release `gripActive` to pause, return the controller to neutral, press `gripActive` again to re-anchor; watch the terminal for `[hold]` warnings |
| During large hand motions the arm stops and the terminal repeats `[hold]...[box]` | This is the designed `--ik exact` semantics: the joint speed required by the hand exceeds the box (the intersection of `exact_ik.max_joint_vel` and "travel / dt"), so the controller holds in place instead of delivering a partial result. The warning names whether the joint speed limit or the joint travel is active; the second half says whether the pose is near a singularity, so do not look for a singularity cause when `sigma_min` is normal. To stop later or remove the speed bound, adjust `exact_ik.max_joint_vel` in `config/teleop.yaml` |
| Motion direction or mirroring is wrong | Adjust the `mapping` section of `config/teleop.yaml` (`axis_remap`, `translation_remap`, `rotation_signs`, `dual_*`) and restart (see `architecture.md`) |
| Changes to `config/teleop.yaml` have no effect | 1. Check that the `[config] <path>` line at startup points at the file you edited (`--config` selects another file); 2. the same item was also given on the command line, and the **command line wins**; 3. a misspelled key or wrong indentation silently falls back to the default; `make test-config` reports it |
| Cannot connect after changing networks | Re-run `run_teleop_webxr.py` (it refreshes the IP), then reconnect the client |

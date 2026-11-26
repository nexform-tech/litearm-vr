English | [简体中文](accuracy.zh-CN.md)

This page defines the precision metrics used for VR teleoperation, the configuration they were measured under, the current measured baselines, and how to reproduce a measurement. It is the reference for quoting numbers to customers and for judging whether a change made tracking worse.

## Scope and configuration

Baselines were measured on 2026-11-26 under this configuration:

- Arm: `Litearm1.12.3-7J`, single-arm upright mount (`--upright`).
- `tcp_offset = 0`: the controlled point is the flange origin. No gripper was fitted, so all numbers describe flange precision. A non-zero offset moves the controlled point and changes the numbers.
- Follow: STM32 serial path (`jf` mode) with slew limits `--max-speed 3.0` rad/s and `--max-accel 6.0` rad/s². One comparison round used `--max-accel 10`.
- Headset: Pico 4 Ultra; the bridge forwarded frames at 90 fps with 0.16 ms per-frame cost.
- Sampling: teleop probe (sim stream plus follow samples at 50 Hz), 1010 frames over 18.7 s per round, three rounds with the operator repeating similar motions.

## Metric definitions

| Metric | Definition | Unit |
| --- | --- | --- |
| Preview tracking error `E_preview` | Distance between the IK end-effector pose and the target pose in the preview, per frame: `distance(ee_after, ee_target)` | mm |
| Follow error `E_follow` | Distance between the headset target TCP and the measured arm TCP, grouped by target speed (`< 20`, `20–100`, `> 100` mm/s) | mm |
| Execution error `E_exec` | Distance between the commanded TCP (after IK and slew) and the measured TCP | mm |
| Orientation error `E_rot` | Angle between the target and measured TCP orientation | ° |

Statistics are the median, the p90 (90th percentile) and the maximum. The median is the headline number: operator gestures differ between rounds, so a single round's maximum says more about one fast flick than about the system.

`E_preview` is printed directly by `probe.py report`. `E_follow`, `E_exec` and `E_rot` join the sim stream with the follow samples by nearest timestamp and aggregate per speed bin; the same streams produce all four metrics, so one recording session is enough.

## Measured baseline

| Metric | Condition | Median | p90 | Max |
| --- | --- | --- | --- | --- |
| `E_preview` | all frames | 0.9–1.6 mm | 3.2–8.6 mm | 15.3 mm |
| `E_follow` | `< 20` mm/s | 9–11 mm | 17 mm | — |
| `E_follow` | `20–100` mm/s | 12–20 mm | 66 mm | — |
| `E_follow` | `> 100` mm/s | 26–50 mm | 107 mm | 190.5 mm |
| `E_rot` | all frames | 3–7° | 9–21° | 32.7° |
| `E_exec` | all frames | 12–18 mm | 24–32 mm | — |

Ranges span the three rounds, which differed in operator speed (target median 0.16–0.24 m/s). The fast-motion bin is dominated by the follow slew limit lagging behind the teleop target; it is expected behaviour, not an IK or model defect. Slow motion, where the slew limit is not binding, is the meaningful baseline for "how precise is the arm when it keeps up".

## Suggested regression limits

These thresholds are for catching regressions, not for customer acceptance. Confirm them with product before quoting.

| Metric | Warn | Fail |
| --- | --- | --- |
| `E_preview` median | > 2 mm | > 3 mm |
| `E_follow` `< 20` mm/s median | > 15 mm | > 20 mm |
| `E_follow` `20–100` mm/s median | > 25 mm | > 40 mm |
| `E_follow` `> 100` mm/s median | > 60 mm | > 80 mm |
| `E_exec` median | > 20 mm | > 30 mm |
| `E_rot` median | > 8° | > 12° |

## Reproducing the measurement

Start teleop with the probe enabled and sample the follow service in a second terminal:

```bash
# ① teleop with the probe (writes the sim stream; do not use during normal teleoperation)
./teleop/launch/run_teleop_webxr.py --real --real-right 127.0.0.1:9002 \
    --probe /tmp/probe_sim.jsonl

# ② follow sample (the follow service is the second hop's ground truth)
python3 teleop/scripts/probe.py sample --addr 127.0.0.1:9002 --out /tmp/probe_follow.jsonl

# ③ attribution report after the motion
python3 teleop/scripts/probe.py report --sim /tmp/probe_sim.jsonl --follow /tmp/probe_follow.jsonl
```

Keep the configuration identical to the scope section: same `tcp_offset`, same slew limits, same mount. For parameter comparisons use a fixed trajectory (fixed amplitude and period) — hand-repeated gestures vary more than the parameters being compared.

## Caveats

- Numbers move with the slew limits. Raising `--max-speed`/`--max-accel` shrinks the fast-motion lag and grows the peak error; lower them and the reverse happens.
- `tcp_offset = 0` throughout. With a gripper or a tool offset the controlled point changes and the same absolute errors mean different angular errors.
- The target speed is estimated from the probe stream; sampling-time jitter adds noise to that estimate. Error statistics do not depend on the estimate itself.
- Only the flange was measured. Gripper-side accuracy depends on the gripper and its calibration and is out of scope here.

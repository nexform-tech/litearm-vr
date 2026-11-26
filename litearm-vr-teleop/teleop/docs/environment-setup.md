English | [简体中文](environment-setup.zh-CN.md)

# Environment setup and configuration

> This page is for a **first deployment or new user**: preparing a PC from
> scratch to run the LiteArm VR teleoperation stack. If the machine is already
> configured, read the [User guide](user-guide.md) instead.

## 1. System requirements

| Item | Requirement | Notes |
|---|---|---|
| Operating system | Ubuntu 22.04 LTS | ROS2 Humble is officially supported |
| ROS2 | Humble (optional) | Needed only for RViz2 visualisation; pybullet IK works without ROS2 |
| Python environment | conda (`metavr-arm`, Python 3.10) | The teleoperation simulation runtime |
| Headset | Meta Quest 3 | WebXR browser with controllers, on the same Wi-Fi as the PC |

> **Minimum setup**: the `metavr-arm` Python environment alone is enough to run
> teleoperation IK (without 3D display).
> **Full setup**: the Python environment plus ROS2 Humble (RViz2 shows the arm
> model).

## 2. Environment composition (two dependency layers)

### 2.1 Python environment `metavr-arm` (required)

The only runtime dependency of the teleoperation simulation, created by
`environments/setup.sh`:

| Dependency | Version | Purpose |
|---|---|---|
| numpy | 2.2.x | Numerical computation |
| scipy | 1.15.x | Rotation vectors and homogeneous transforms |
| pybullet | `==3.2.7` | Arm IK and simulation (**must stay at 3.2.7**; 3.2.6 is ABI-incompatible with numpy 2) |
| websockets | 16.x | WebSocket teleoperation service (the `--source ws` fallback) |
| eclipse-zenoh | `>=1.10` | **Default data source**: subscribes to the `vr_frame` topic forwarded by `litearm-vr-bridge` |
| pin | `>=2.6.3` | pinocchio models and Jacobians for differential IK (the `--ik pin` backend; the default backend is `pybullet`) |
| pin-pink | `>=4.4.0` | pink task/QP framework for differential IK (the PyPI name is `pin-pink`; `pink` is an unrelated formatter) |
| qpsolvers + quadprog | `>=4.3.1` | QP solver backend for pink |
| loop-rate-limiters | — | Control-loop throttling for pink |

The single source of truth for the dependency list is
`environments/metavr-arm-requirements.txt`.

> `setup.sh` prints the interpreter path after creating the environment. The
> three entry points (`setup.sh`, `launch/run_teleop_webxr.py`, `Makefile`)
> resolve that path with the same logic, so `make smoke` works right after
> installation. If the environment lives elsewhere, pass
> `--metavr-py <that environment's python>` (or
> `export METAVR_PY=<that environment's python>`).

### 2.2 ROS2 Humble (optional, RViz2 visualisation only)

- `robot_state_publisher`: reads the URDF and publishes TF
- `rviz2`: shows RobotModel and TF

Without ROS2, `launch/run_teleop_webxr.py` falls back to pure pybullet IK
(no 3D window).

## 3. One-command installation

```bash
# 1. Python environment (conda must already be installed; the environment is created under conda's own root)
bash environments/setup.sh

# 2. ROS2 Humble (optional, Ubuntu 22.04)
# Follow the official instructions: https://docs.ros.org/en/humble/Installation.html
# Install robot_state_publisher and rviz2:
sudo apt install ros-humble-robot-state-publisher ros-humble-rviz2
```

`environments/setup.sh` does three things: rebuild the conda environment,
install the pip dependencies and `litearm-teleop` in editable mode, then verify
the imports.

## 4. Verify the installation

```bash
# Python environment smoke test (no window)
make smoke
# Expected final line: SMOKE TEST PASSED

# Full chain (RViz2 display plus teleoperation service)
make teleop     # or ./launch/run_teleop_webxr.py (single arm) / ... --dual (dual arm)
```

## 5. Network notes (mirrors used on this machine)

| Resource | Available channel |
|---|---|
| conda packages | Tsinghua mirror (the USTC anaconda mirror is retired): `--override-channels -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r` |
| pip packages | Tsinghua PyPI (works normally) |
| GitHub | `ghfast.top` mirror zip (mind the default branch) |
| HuggingFace / Meta | Unreachable |

> The mirrors are baked into `environments/setup.sh` (Tsinghua by default) and
> can be overridden with the `CONDA_MAIN` and `CONDA_R` environment variables;
> machines with normal network access can switch back to `defaults`.

## 6. Quick start (after configuration)

```bash
make teleop          # one command: clean leftovers -> robot_state_publisher + rviz2 -> teleoperation service (zenoh subscription by default)
                     # frames need a source first: run litearm-vr-bridge in another terminal (it receives the headset on ws 8765)
make teleop ARGS="--source ws"   # no bridge: this repository listens on 8765 and the headset connects directly
make stop            # stop
```

See the [User guide](user-guide.md) and [Troubleshooting](troubleshooting.md).

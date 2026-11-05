English | [简体中文](setup.zh-CN.md)

This guide installs the LiteArm VR stack from source on a workstation and verifies it.

# Setup

## Prerequisites

- Ubuntu 20.04 or 22.04, x86_64, Python 3.10 or newer, git.
- Conda for the teleop service in a source checkout. The release bundle ships a
  frozen teleop and does not need conda.
- Optional: Docker and `dpkg-deb` to build the bundle.

## 1. Install the bridge

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -e "./litearm-vr-bridge[dev]"
```

## 2. Install the teleop environment

```bash
bash litearm-vr-teleop/teleop/environments/setup.sh
```

For the test suite, install the teleop dependencies in the active Python:

```bash
pip install numpy scipy pyyaml websockets eclipse-zenoh pybullet
```

## 3. Verify

```bash
bash scripts/ci.sh all
```

Expected result: the bridge unit tests, the teleop tests, and the follow pure
tests all pass.

## 4. Build the bundle

```bash
bash packaging/build-bundle.sh 0.2.2
```

See [Packaging](../packaging/README.md) for prerequisites and outputs.

## Troubleshooting

**`ModuleNotFoundError: websockets`**

Activate the virtual environment or install the dependency:
`pip install websockets`.

**PyBullet cannot open a window**

Use `--viewer rviz` or a machine with a desktop. The test suite uses PyBullet
DIRECT mode.

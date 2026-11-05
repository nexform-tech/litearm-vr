[English](setup.md) | 简体中文

本指南说明如何在工作站上从源码安装 LiteArm VR 栈并验证。

# 安装

## 前置条件

- Ubuntu 20.04 或 22.04，x86_64，Python 3.10 及以上，git。
- 源码方式运行 teleop 时需要 conda。发布 bundle 自带冻结 teleop，不需要 conda。
- 可选：构建 bundle 需要 Docker 与 `dpkg-deb`。

## 1. 安装桥

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -e "./litearm-vr-bridge[dev]"
```

## 2. 安装 teleop 环境

```bash
bash litearm-vr-teleop/teleop/environments/setup.sh
```

跑测试时，在当前 Python 里装 teleop 依赖：

```bash
pip install numpy scipy pyyaml websockets eclipse-zenoh pybullet
```

## 3. 验证

```bash
bash scripts/ci.sh all
```

预期结果：桥单测、teleop 测试、follow 纯逻辑测试全部通过。

## 4. 构建 bundle

```bash
bash packaging/build-bundle.sh 0.2.2
```

前置条件与产物见 [打包与发布](../packaging/README.zh-CN.md)。

## 故障排查

**`ModuleNotFoundError: websockets`**

激活虚拟环境，或安装依赖：`pip install websockets`。

**PyBullet 开不了窗口**

改用 `--viewer rviz`，或在带图形桌面的机器上跑。测试套件使用 PyBullet
DIRECT 模式。

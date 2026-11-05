[English](README.md) | 简体中文

# 打包与发布

`packaging/build-bundle.sh` 生成客户实际安装的单个 `.deb`，包含桥、冻结版
teleop、跟随服务、固定版本的 Python SDK、离线 wheelhouse，以及 vendored 的
CAN DKMS 驱动。

上位机（`litearm-vr-studio`）是独立仓库，不包含在本 bundle 里。

## 前置条件

- Linux x86_64，装有 `bash`、`git`、`python3`、`pip`、`dpkg-deb` 和 Docker。
- 首次构建需要联网：脚本要拉取固定版本的 SDK 和离线 wheelhouse。
- 约 2 GB 空闲磁盘，用于冻结 teleop 和 staging 目录。

冻结 teleop 在 Ubuntu 22.04 + Python 3.10 的 Docker 镜像里构建，产物链接的
glibc 与目标机同代。镜像会复用；改了 Dockerfile 时通过
`build-frozen-teleop.sh --rebuild-image` 重建。

## 构建

```bash
bash packaging/build-bundle.sh 0.2.2
```

产物：

```
packaging/out/0.2.2/
├── litearm-vr-bundle_0.2.2_amd64.deb
├── litearm-vr-bundle_0.2.2_amd64.deb.sha256
├── litearm-vr-bundle_0.2.2_amd64.tar.gz
├── litearm-vr-bundle_0.2.2_amd64.tar.gz.sha256
└── MANIFEST.json
```

`.deb` 是 Ubuntu/Debian 的支持路径。`.tar.gz` 是同一 payload 去掉 Debian 控制脚本；
解到 `/` 后运行 `/usr/lib/litearm-vr-bundle/portable-install.sh`。它面向 glibc
2.35+ 且不使用 apt 的 Linux 发行版，属于实验性支持；目标机仍需 `python3`、
`python3-venv`、`dkms`、`build-essential` 和匹配的内核头文件。

`MANIFEST.json` 记录源码 commit、SDK pin、DKMS commit、产物哈希和组件列表。
安装命令写在自动生成的 release notes 里；release workflow 会把整个输出目录挂到
GitHub Release。

## 脚本做了什么

1. 按 `sdk-pins.env` 拉取 `litearm-python` 与 `litegrip-python`。
2. 拉取 `build-bundle.sh` 里固定版本的离线 wheelhouse。
3. 没有现成产物时，用 `build-frozen-teleop.sh` 构建
   `packaging/work/frozen-src/dist/teleop-frozen/`。
4. 从 `third_party/litearm-can-dkms/` 构建 vendored DKMS 包。
5. 组装 bundle，给安装脚本和运行时打冻结 teleop 补丁，写出 `.deb`、便携
   `.tar.gz`、它们的 `.sha256` 和 `MANIFEST.json`。

## 环境变量覆盖

| 变量 | 用途 |
| --- | --- |
| `LITEARM_BUILD_CLEAN=1` | 构建前清空 `packaging/work/` |
| `LITEARM_FROZEN_DIR=<dir>` | 复用已有的 `teleop-frozen` 目录 |
| `LITEARM_CAN_REPO=<dir>` | 换一份 `litearm-can-dkms` 源码树 |
| `LITEARM_CAN_DEB=<file>` | 使用预编译的 `litearm-can-dkms` `.deb` |
| `LITEARM_WHEELS_DIR=<dir>` | 使用预置的 wheelhouse |
| `LITEARM_SKIP_FROZEN_BUILD=1` | 缺冻结产物时直接失败，不现场构建 |

## CI 与发布

- `.github/workflows/ci.yml` 跑组件测试；`package` job 可以在 Actions 页面
  手动构建一个 bundle。
- `.github/workflows/release.yml` 调用 NexForm 中央 release workflow。它产生
  新 tag 后，`package` job 构建 bundle 并把四个输出文件挂到 GitHub Release。

DKMS 源码 vendored 在 `third_party/litearm-can-dkms/`，许可证是 GPL-2.0-only；
仓库其余部分是 Apache-2.0。许可证文本见 `THIRD_PARTY_NOTICES.md`，以及
bundle 里的 `/usr/share/doc/litearm-vr-bundle/`。

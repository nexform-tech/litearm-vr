English | [简体中文](README.zh-CN.md)

# Packaging and release

`packaging/build-bundle.sh` builds the single `.deb` that customers install.
It contains the bridge, the frozen teleop, the follow services, the pinned
Python SDKs, the offline wheelhouse, and the vendored CAN DKMS driver.

The host application (`litearm-vr-studio`) is a separate repository and is not
part of this bundle.

## Prerequisites

- Linux x86_64 with `bash`, `git`, `python3`, `pip`, `dpkg-deb`, and Docker.
- Network access the first time: the script fetches the pinned SDK commits and
  the offline wheelhouse.
- About 2 GB of free disk space for the frozen teleop and the staging tree.

The frozen teleop is built in a Docker image based on Ubuntu 22.04 and Python
3.10, so the binary links the same glibc generation as the target machines.
The image is reused on later builds; pass `--rebuild-image` through
`build-frozen-teleop.sh` when the Dockerfile changes.

## Build

```bash
bash packaging/build-bundle.sh 0.2.2
```

Output:

```
packaging/out/0.2.2/
├── litearm-vr-bundle_0.2.2_amd64.deb
├── litearm-vr-bundle_0.2.2_amd64.deb.sha256
├── litearm-vr-bundle_0.2.2_amd64.tar.gz
├── litearm-vr-bundle_0.2.2_amd64.tar.gz.sha256
└── MANIFEST.json
```

The `.deb` is the supported Ubuntu/Debian install. The `.tar.gz` carries the
same payload without the Debian control scripts; extract it at `/` and run
`/usr/lib/litearm-vr-bundle/portable-install.sh`. It targets glibc 2.35+ Linux
distributions that do not use apt and is experimental; the target still needs
`python3`, `python3-venv`, `dkms`, `build-essential`, and matching kernel
headers.

`MANIFEST.json` records the source commit, the SDK pins, the DKMS commit, the
artifact hashes, and the component list. The generated release notes contain
the install commands; the release workflow attaches this output directory to
the GitHub release.

## What the script does

1. Fetches `litearm-python` and `litegrip-python` at the commits in
   `sdk-pins.env`.
2. Fetches the offline wheelhouse pinned in `build-bundle.sh`.
3. Builds `packaging/work/frozen-src/dist/teleop-frozen/` with
   `build-frozen-teleop.sh` when it is not already present.
4. Builds the vendored DKMS package from
   `third_party/litearm-can-dkms/`.
5. Assembles the bundle, applies the install/runtime patches for the frozen
   teleop, and writes the `.deb`, the portable `.tar.gz`, their `.sha256`
   sidecars, and `MANIFEST.json`.

## Environment overrides

| Variable | Purpose |
| --- | --- |
| `LITEARM_BUILD_CLEAN=1` | Remove `packaging/work/` before building |
| `LITEARM_FROZEN_DIR=<dir>` | Reuse an existing `teleop-frozen` directory |
| `LITEARM_CAN_REPO=<dir>` | Use another `litearm-can-dkms` source tree |
| `LITEARM_CAN_DEB=<file>` | Use a prebuilt `litearm-can-dkms` `.deb` |
| `LITEARM_WHEELS_DIR=<dir>` | Use a pre-populated wheelhouse |
| `LITEARM_SKIP_FROZEN_BUILD=1` | Fail instead of building the frozen teleop |

## CI and release

- `.github/workflows/ci.yml` runs the component tests. Its `package` job can
  build a bundle manually from the Actions tab.
- `.github/workflows/release.yml` calls the central NexForm release workflow.
  When that workflow creates a new tag, the `package` job builds the bundle and
  attaches the four output files to the GitHub release.

The DKMS source is vendored under `third_party/litearm-can-dkms/` and is
GPL-2.0-only. The rest of the repository is Apache-2.0. See
`THIRD_PARTY_NOTICES.md` and the bundle's `/usr/share/doc/litearm-vr-bundle/`
directory for the license texts.

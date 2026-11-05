# Third-party notices

This repository contains NexForm application code under the Apache-2.0 license
(see `LICENSE`) and the following third-party components.

## Vendored source

### litearm-can-dkms

- Location: `third_party/litearm-can-dkms/`
- License: GPL-2.0-only (full text in `third_party/litearm-can-dkms/LICENSE`)
- Upstream: `litearm-can-dkms`, commit
  `f3648a8489dc120ba0c5e28b69921fe2107add1b`
- Copyright: 2013-2016 Geschwister Schneider Technologie-, Entwicklungs- und
  Vertriebs UG (Haftungsbeschränkt); 2016 Hubert Denkmair; 2023 Pengutronix,
  Marc Kleine-Budde
- The driver is derived from the Linux kernel `gs_usb` driver.

The GPL-2.0-only license applies to this directory only. It does not change the
license of the NexForm application code.

## Teleop model packages

The URDF and STL packages under `litearm-vr-teleop/teleop/models/` declare BSD
in their `package.xml`. See each package's `package.xml` for the exact terms.

## Runtime dependencies bundled in release artifacts

The `litearm-vr-bundle` `.deb` carries the frozen teleop runtime and an offline
wheelhouse. The main components and their upstream licenses are:

| Component | License |
| --- | --- |
| Python | PSF-2.0 |
| pinocchio | BSD-2-Clause |
| pybullet | zlib |
| eclipse-zenoh | EPL-2.0 (check the wheel metadata for the exact variant) |
| websockets | BSD-3-Clause |
| numpy | BSD-3-Clause |
| scipy | BSD-3-Clause |
| pyserial | BSD-3-Clause |
| setuptools | MIT |
| wheel | MIT |
| packaging | Apache-2.0 OR BSD-2-Clause |
| tomli | MIT |

The release build installs packages from their upstream wheels; each wheel
carries its own license metadata. Before publishing a release, include the
license texts required by those packages in the release notes or in the bundle's
documentation directory.

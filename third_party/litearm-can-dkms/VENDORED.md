# Vendored litearm-can-dkms

This directory is a source snapshot of the LiteArm CAN DKMS driver used by the
`litearm-vr-bundle` package. It is not NexForm application code and is not
covered by the repository's Apache-2.0 license.

- Upstream: the internal `litearm-can-dkms` repository
- Source commit: `f3648a8489dc120ba0c5e28b69921fe2107add1b` (also in `SOURCE_COMMIT`)
- Vendored on: 2026-11-05
- License: GPL-2.0-only (see `LICENSE` and `debian/copyright`)
- Build entry point: `build-deb.sh`; the bundle build calls it and merges the
  resulting `.deb` payload into `litearm-vr-bundle`.

The driver is derived from the Linux kernel `gs_usb` driver. The copyright
holders are listed in `debian/copyright` and in the SPDX headers of `src/*.c`.

## Updating

Replace this tree with the upstream source at the new commit, update
`SOURCE_COMMIT`, and verify the standalone package still builds:

```bash
bash third_party/litearm-can-dkms/build-deb.sh 0.0.0-test
dpkg-deb -c third_party/litearm-can-dkms/dist/litearm-can-dkms_0.0.0-test_all.deb
```

The bundle build copies this directory into its work tree before building, so
the vendored tree stays clean.

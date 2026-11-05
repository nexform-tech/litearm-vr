#!/bin/sh
# Build litearm-can-dkms_<version>_all.deb into ./dist
# usage: ./build-deb.sh [version]   (default: 1.0.0)
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
VERSION="${1:-1.0.0}"
STAGE="$(mktemp -d)"
chmod 755 "$STAGE"
trap 'rm -rf "$STAGE"' EXIT
SRC="$STAGE/usr/src/litearm-can-$VERSION"

mkdir -p "$SRC" "$STAGE/DEBIAN" "$STAGE/lib/udev/rules.d" "$STAGE/lib/systemd/system" "$STAGE/usr/lib/litearm" "$STAGE/usr/share/doc/litearm-can-dkms"
cp "$HERE"/src/gs_usb-5.15.c "$HERE"/src/gs_usb-6.8.c "$HERE"/dkms/Makefile "$SRC/"
install -m 644 "$HERE/LICENSE" "$SRC/LICENSE"
install -m 755 "$HERE/dkms/pre-build.sh" "$SRC/pre-build.sh"
install -m 644 "$HERE/debian/copyright" "$STAGE/usr/share/doc/litearm-can-dkms/copyright"
sed "s/@VERSION@/$VERSION/" "$HERE/dkms/dkms.conf" > "$SRC/dkms.conf"
install -m 644 "$HERE/deploy/60-litearm-can.rules" "$STAGE/lib/udev/rules.d/"
install -m 644 "$HERE/deploy/litearm-can-up@.service" "$STAGE/lib/systemd/system/"
install -m 755 "$HERE/bin/litearm-can-up" "$HERE/bin/litearm-can-selftest" "$STAGE/usr/lib/litearm/"
sed "s/@VERSION@/$VERSION/" "$HERE/debian/control" > "$STAGE/DEBIAN/control"
for s in postinst prerm; do
    sed "s/@VERSION@/$VERSION/" "$HERE/debian/$s" > "$STAGE/DEBIAN/$s"
    chmod 755 "$STAGE/DEBIAN/$s"
done
mkdir -p "$HERE/dist"
dpkg-deb --root-owner-group --build "$STAGE" "$HERE/dist/litearm-can-dkms_${VERSION}_all.deb"

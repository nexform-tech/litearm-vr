#!/bin/sh
# Pick the gs_usb baseline that matches the kernel being built for.
# DKMS exports $kernelver; fall back to uname -r when run by hand.
kver="${kernelver:-$(uname -r)}"
case "$kver" in
    4.*|5.*) src=gs_usb-5.15.c ;;
    *)       src=gs_usb-6.8.c ;;
esac
echo "litearm-can: kernel $kver -> $src"
cp -f "$src" gs_usb.c

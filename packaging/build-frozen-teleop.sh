#!/usr/bin/env bash
# Build the frozen teleop (PyInstaller onedir) from this repository's teleop source.
#
# The product code comes from litearm-vr-teleop/; the build tooling lives in
# litearm-vr-teleop/teleop/packaging/frozen/. The build runs in Docker on
# Ubuntu 22.04 + Python 3.10 so the binary links the same glibc generation as the
# target machines.
#
# Usage:
#   bash packaging/build-frozen-teleop.sh
#   bash packaging/build-frozen-teleop.sh --rebuild-image
#
# Output: packaging/work/frozen-src/dist/teleop-frozen/
set -euo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$HERE/.." && pwd)"
SRC="$ROOT/litearm-vr-teleop"
TOOLING="$SRC/teleop/packaging/frozen"
WORK="$HERE/work/frozen-src"

die() { echo "错误：$*" >&2; exit 1; }
note() { echo "[frozen] $*"; }

[ -d "$SRC/teleop" ] || die "缺少 teleop 源码：$SRC/teleop"
[ -d "$TOOLING" ] || die "缺少冻结构建工具：$TOOLING"
command -v docker >/dev/null || die "没装 docker（构建需要它，跑不需要）"
command -v python3 >/dev/null || die "需要 python3"

rm -rf "$WORK"
mkdir -p "$WORK"
note "复制 teleop 源码到 $WORK"
tar -C "$SRC" --exclude-vcs --exclude='__pycache__' --exclude='.pytest_cache' \
    --exclude='dist' --exclude='packaging-build' -cf - . | tar -xf - -C "$WORK"

# Keep the build tooling in the temp tree even if the checkout predates it.
mkdir -p "$WORK/teleop/packaging/frozen"
cp -a "$TOOLING"/. "$WORK/teleop/packaging/frozen/"

# 冻结后 __file__ 落在发行目录里，模型路径必须能由 LITEARM_TELEOP_MODELS 指过来。
python3 - "$WORK/teleop/src/litearm_teleop/teleop_sim.py" <<'PY'
import sys
from pathlib import Path

p = Path(sys.argv[1])
s = p.read_text(encoding="utf-8")

old = 'SINGLE_URDF_DIR = os.path.join(_PROJECT_DIR, "models", "litearm_description")'
new = '''# 模型根目录。冻结成自带解释器的可执行文件之后，__file__ 落在发行目录里，
# 那个"往上数三层"够不到仓库里的 models/ —— 用 LITEARM_TELEOP_MODELS 指过来。
_MODELS_DIR = (os.environ.get("LITEARM_TELEOP_MODELS", "").strip()
               or os.path.join(_PROJECT_DIR, "models"))
SINGLE_URDF_DIR = os.path.join(_MODELS_DIR, "litearm_description")'''
if old not in s:
    sys.exit("teleop_sim.py: SINGLE_URDF_DIR not found")
s = s.replace(old, new, 1)

old2 = '''DUAL_URDF_DIR = os.path.join(_PROJECT_DIR, "models",
                             "dual_litearm_column_description")'''
new2 = '''DUAL_URDF_DIR = os.path.join(_MODELS_DIR,
                             "dual_litearm_column_description")'''
if old2 not in s:
    sys.exit("teleop_sim.py: DUAL_URDF_DIR not found")
p.write_text(s.replace(old2, new2, 1), encoding="utf-8")
print("patched", p)
PY

note "在 Docker 里构建冻结 teleop"
if [ "${1:-}" = "--rebuild-image" ]; then
  bash "$WORK/teleop/packaging/frozen/build.sh" --rebuild-image
else
  bash "$WORK/teleop/packaging/frozen/build.sh"
fi

OUT="$WORK/dist/teleop-frozen"
[ -x "$OUT/teleop-server" ] || die "构建完成但没有 $OUT/teleop-server"
note "产物：$OUT（$(du -sh "$OUT" | cut -f1)）"

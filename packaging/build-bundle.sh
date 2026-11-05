#!/usr/bin/env bash
# Build litearm-vr-bundle_<ver>_amd64.deb from this repository.
#
# The bundle contains:
#   /opt/litearm/litearm-vr-bridge   bridge + teleop/follow + SDKs + frozen teleop
#   /usr/src/litearm-can-<ver>       vendored CAN DKMS sources
#   /usr/lib/litearm-vr-bundle       install hooks
#
# The studio .deb is published from litearm-vr-studio and is not part of this bundle.
#
# Usage: bash packaging/build-bundle.sh <version>
#
# Environment overrides:
#   LITEARM_BUILD_CLEAN=1       remove packaging/work before building
#   LITEARM_FROZEN_DIR=<dir>    reuse a built teleop-frozen directory
#   LITEARM_CAN_REPO=<dir>      use another litearm-can-dkms source tree
#   LITEARM_CAN_DEB=<file>      use a prebuilt litearm-can-dkms deb
#   LITEARM_WHEELS_DIR=<dir>    use a pre-populated wheelhouse
#   LITEARM_SKIP_FROZEN_BUILD=1 fail instead of building the frozen teleop
#
# Requirements: bash, git, python3, pip, dpkg-deb, docker, network for SDK/wheel fetch.
set -euo pipefail

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  exit 0
fi

VERSION="${1:?usage: build-bundle.sh <version>}"
VERSION="${VERSION#v}"
case "$VERSION" in
  [0-9]*.[0-9]*.[0-9]*) ;;
  *) echo "bad version: $VERSION" >&2; exit 2 ;;
esac

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SRC="$(cd -- "$HERE/.." && pwd)"
CANREPO="${LITEARM_CAN_REPO:-$SRC/third_party/litearm-can-dkms}"
OUT="$HERE/out/$VERSION"
WORK="$HERE/work"
STAGE="$WORK/stage"
SDKROOT="$WORK/sdks"
CANSRC="$WORK/can-src"
FROZEN_SRC="${LITEARM_FROZEN_DIR:-$WORK/frozen-src/dist/teleop-frozen}"
WHEELS="${LITEARM_WHEELS_DIR:-$HERE/wheels}"
CAN_DEB="${LITEARM_CAN_DEB:-}"
PKGNAME="litearm-vr-bundle"
PKG="$STAGE/opt/litearm/litearm-vr-bridge"

die() { echo "错误：$*" >&2; exit 1; }
note() { echo "[build] $*"; }

[ -d "$CANREPO" ] || die "缺少 DKMS 源码：$CANREPO"
[ -f "$CANREPO/build-deb.sh" ] || die "缺少 $CANREPO/build-deb.sh"

if [ "${LITEARM_BUILD_CLEAN:-0}" = 1 ]; then
  rm -rf "$WORK"
fi
rm -rf "$STAGE" "$OUT"
mkdir -p "$PKG" "$OUT" "$STAGE/DEBIAN" "$STAGE/usr/lib/$PKGNAME"

# ── 0. 依赖：SDK、wheelhouse、冻结 teleop、DKMS ─────────────────────────────
fetch_sdks() {
  # shellcheck disable=SC1091
  . "$HERE/sdk-pins.env"
  mkdir -p "$SDKROOT"
  for name in litearm-python litegrip-python; do
    dst="$SDKROOT/$name"
    case "$name" in
      litearm-python) ref="$LITEARM_PYTHON_REF" ;;
      litegrip-python) ref="$LITEGRIP_PYTHON_REF" ;;
    esac
    [ -n "$ref" ] || die "sdk-pins.env 缺少 $name 的 pin"
    if [ -d "$dst/src" ]; then
      continue
    fi
    rm -rf "$dst"
    mkdir -p "$dst"
    git -C "$dst" init -q
    git -C "$dst" remote add origin "https://github.com/nexform-tech/$name.git"
    if ! git -C "$dst" fetch -q --depth 1 origin "$ref"; then
      git -C "$dst" fetch -q origin || die "拉取 $name 失败"
      git -C "$dst" checkout -q "$ref" || die "检出 $name@$ref 失败"
    else
      git -C "$dst" checkout -q FETCH_HEAD
    fi
    [ -d "$dst/src" ] || die "$name 没有 src/"
  done
}

fetch_wheels() {
  if compgen -G "$WHEELS/*.whl" >/dev/null; then
    return 0
  fi
  command -v python3 >/dev/null || die "需要 python3 来下载 wheelhouse"
  mkdir -p "$WHEELS"
  note "下载离线 wheelhouse"
  python3 -m pip download --quiet --only-binary=:all: \
    --python-version 310 --implementation cp \
    --abi cp310 --abi abi3 --abi none \
    --platform manylinux_2_17_x86_64 \
    -d "$WHEELS" \
    "websockets==16.1.1" "eclipse-zenoh==1.10.2" "tomli==2.5.0" \
    "pyserial==3.5" "setuptools==84.2.1" "wheel==0.48.0" "packaging==26.3" \
    || die "下载 wheelhouse 失败（需要网络）"
  find "$WHEELS" -name '*.tar.gz' -delete
  ls "$WHEELS"/*.whl >/dev/null 2>&1 || die "wheelhouse 为空"
}

build_frozen() {
  if [ -x "$FROZEN_SRC/teleop-server" ]; then
    return 0
  fi
  if [ "${LITEARM_SKIP_FROZEN_BUILD:-0}" = 1 ]; then
    die "缺少冻结 teleop：$FROZEN_SRC/teleop-server"
  fi
  note "构建冻结版 teleop（Docker）"
  bash "$HERE/build-frozen-teleop.sh"
  [ -x "$FROZEN_SRC/teleop-server" ] || die "冻结 teleop 构建失败：$FROZEN_SRC/teleop-server"
}

prepare_can() {
  if [ -n "$CAN_DEB" ]; then
    [ -f "$CAN_DEB" ] || die "LITEARM_CAN_DEB 不存在：$CAN_DEB"
    return 0
  fi
  rm -rf "$CANSRC"
  mkdir -p "$CANSRC"
  tar -C "$CANREPO" --exclude-vcs --exclude='./dist' --exclude='__pycache__' -cf - . \
    | tar -xf - -C "$CANSRC"
  note "构建 CAN DKMS deb"
  ( cd "$CANSRC" && ./build-deb.sh "$VERSION" >/dev/null )
  CAN_DEB="$CANSRC/dist/litearm-can-dkms_${VERSION}_all.deb"
  [ -f "$CAN_DEB" ] || die "DKMS 没产出 $CAN_DEB"
}

if [ -n "${LITEARM_CAN_DEB:-}" ]; then
  CAN_COMMIT="prebuilt:$(basename "$LITEARM_CAN_DEB")"
else
  CAN_COMMIT="$(cat "$CANREPO/SOURCE_COMMIT" 2>/dev/null || git -C "$CANREPO" rev-parse HEAD 2>/dev/null || echo unknown)"
fi

fetch_sdks
fetch_wheels
build_frozen
prepare_can

# ── 1. 三个组件 ──────────────────────────────────────────────────────────────
note "1/8 复制源码（bridge / teleop / follow）"
tar -C "$SRC/litearm-vr-bridge" --exclude-vcs --exclude='__pycache__' \
    --exclude='.pytest_cache' --exclude='logs' --exclude='.venv*' \
    --exclude='*.egg-info' -cf - . \
  | tar -xf - -C "$PKG"
mkdir -p "$PKG/submodules"
# 本仓的 submodules/ 里是两个指向兄弟目录的软链；包里要换成真实拷贝。
rm -f "$PKG/submodules/litearm-vr-teleop" "$PKG/submodules/litearm-vr-follow"
for c in litearm-vr-teleop litearm-vr-follow; do
  mkdir -p "$PKG/submodules/$c"
  extra=()
  if [ "$c" = litearm-vr-teleop ]; then
    # 冻结构建工具只在仓库里用，不进运行时安装目录。
    extra+=(--exclude='./teleop/packaging')
  fi
  tar -C "$SRC/$c" --exclude-vcs --exclude='__pycache__' --exclude='.pytest_cache' \
      --exclude='dist' --exclude='logs' --exclude='*.egg-info' "${extra[@]}" -cf - . \
    | tar -xf - -C "$PKG/submodules/$c"
done

# ── 2. SDK（follow 真正碰硬件要 import litearm / litegrip） ──────────────────
note "2/8 复制 SDK 源码"
mkdir -p "$PKG/sdk"
for s in litearm-python litegrip-python; do
  [ -d "$SDKROOT/$s/src" ] || die "缺少 SDK：$SDKROOT/$s"
  tar -C "$SDKROOT" --exclude-vcs --exclude='__pycache__' --exclude='./tests' \
      -cf - "$s" | tar -xf - -C "$PKG/sdk"
done

# ── 2b. 冻结版 teleop（由本仓 teleop 源码构建） ─────────────────────────────
[ -x "$FROZEN_SRC/teleop-server" ] || die "缺少冻结 teleop：$FROZEN_SRC/teleop-server（先跑 packaging/build-frozen-teleop.sh）"
note "2b/8 带上冻结 teleop（$(du -sh "$FROZEN_SRC" | cut -f1)）"
mkdir -p "$PKG/frozen-teleop"
cp -a "$FROZEN_SRC"/. "$PKG/frozen-teleop"/

# ── 3. 离线依赖 ─────────────────────────────────────────────────────────────
note "3/8 带上离线依赖（wheelhouse + vendor-upstream）"
[ -d "$WHEELS" ] || die "缺少 $WHEELS（先下载 wheel）"
mkdir -p "$PKG/wheelhouse" "$PKG/vendor-upstream"
cp "$WHEELS"/*.whl "$PKG/wheelhouse/"
python3 - "$WHEELS/pyserial-3.5-py2.py3-none-any.whl" "$PKG/vendor-upstream" <<'PY'
import sys, zipfile
from pathlib import Path
wheel, dst = Path(sys.argv[1]), Path(sys.argv[2])
with zipfile.ZipFile(wheel) as z:
    z.extractall(dst)
PY

# ── 4. 安装脚本离线补丁 ─────────────────────────────────────────────────────
note "4/8 给 install.sh 打离线补丁（预置 venv + 系统级 User=）"
python3 - "$PKG/scripts/install.sh" <<'PY'
import sys
from pathlib import Path
p = Path(sys.argv[1])
s = p.read_text(encoding="utf-8")

old_venv = '''if [ ! -x "$ROOT/.venv/bin/python" ]; then
  python3 -m venv "$ROOT/.venv" || die "建 venv 失败（需要 python3-venv 包？）"
fi
"$ROOT/.venv/bin/pip" install -q --upgrade pip >/dev/null 2>&1 || true
"$ROOT/.venv/bin/pip" install -q -e "$ROOT" \\
  || die "装依赖失败（离线？）：$ROOT/.venv/bin/pip install -e $ROOT"'''
new_venv = '''if [ -f "$ROOT/.venv/.bundle-prebuilt" ]; then
  note 依赖 "使用安装包预置的 .venv（离线，不联网）"
else
  if [ ! -x "$ROOT/.venv/bin/python" ]; then
    python3 -m venv "$ROOT/.venv" || die "建 venv 失败（需要 python3-venv 包？）"
  fi
  if [ -d "$ROOT/wheelhouse" ]; then
    "$ROOT/.venv/bin/pip" install -q --no-index --find-links "$ROOT/wheelhouse" "$ROOT" \\
      || die "从 wheelhouse 装依赖失败"
    touch "$ROOT/.venv/.bundle-prebuilt"
  else
    "$ROOT/.venv/bin/pip" install -q --upgrade pip >/dev/null 2>&1 || true
    "$ROOT/.venv/bin/pip" install -q -e "$ROOT" \\
      || die "装依赖失败（离线？）：$ROOT/.venv/bin/pip install -e $ROOT"
  fi
fi'''
if old_venv not in s:
    sys.exit("install.sh: venv block not found")
s = s.replace(old_venv, new_venv, 1)

old_sed = '''sed -e "s#/opt/litearm/litearm-vr-bridge#$ROOT#g" \\
    -e "/^Environment=PYTHONPATH=/d" \\
    -e "s#^ExecStart=.*#ExecStart=$ROOT/.venv/bin/python -m litearm_vr_bridge#" \\
    "$UNIT_SRC" > "$UNIT_DST"'''
new_sed = '''SED_USER=""
[ "$MODE" = system ] && SED_USER="User=$RUN_USER"
sed -e "s#/opt/litearm/litearm-vr-bridge#$ROOT#g" \\
    -e "/^Environment=PYTHONPATH=/d" \\
    -e "s#^ExecStart=.*#ExecStart=$ROOT/.venv/bin/python -m litearm_vr_bridge#" \\
    -e "s#^\\[Service\\][[:space:]]*#[Service]\\n${SED_USER}#" \\
    "$UNIT_SRC" > "$UNIT_DST"'''
if old_sed not in s:
    sys.exit("install.sh: unit sed block not found")
s = s.replace(old_sed, new_sed, 1)
p.write_text(s, encoding="utf-8")
print("patched", p)
PY

# SDK / pyserial 指路（跟随进程会读）
cat >> "$PKG/deploy/bridge.env.example" <<'EOF'

# ── 安装包自带（离线） ──
# follow 的启动脚本认这两个变量；pyserial 在 vendor-upstream/ 里。
LITEARM_PY_SRC=/opt/litearm/litearm-vr-bridge/sdk/litearm-python/src
LITEGRIP_SDK=/opt/litearm/litearm-vr-bridge/sdk/litegrip-python/src
PYTHONPATH=/opt/litearm/litearm-vr-bridge/vendor-upstream
EOF

# ── 4b. bridge：优先启动冻结 teleop ─────────────────────────────────────────
note "4b/8 给 bridge 打 frozen-teleop 启动补丁"
python3 - "$PKG/src/litearm_vr_bridge/config.py" "$PKG/src/litearm_vr_bridge/preflight.py" <<'PY'
import sys
from pathlib import Path

cfg = Path(sys.argv[1])
pre = Path(sys.argv[2])

s = cfg.read_text(encoding="utf-8")

anchor = '''    root = _bridge_root(env)
    return (root / default_rel) if root is not None else None
'''
helper = anchor + '''

def _frozen_teleop(env: dict[str, str], teleop_dir: Path) -> Path | None:
    """冻结版 teleop 可执行文件：安装包里有就用它，现场没有才回退 conda。

    显式 `TELEOP_FROZEN=<path>` 指路；不设时找安装包里的
    `<bridge-root>/frozen-teleop/teleop-server`。
    """
    explicit = (env.get("TELEOP_FROZEN") or "").strip()
    if explicit:
        p = Path(os.path.expanduser(explicit))
        return p if p.is_file() and os.access(p, os.X_OK) else None
    root = _bridge_root(env)
    if root is None:
        return None
    p = root / "frozen-teleop" / "teleop-server"
    return p if p.is_file() and os.access(p, os.X_OK) else None
'''
if anchor not in s:
    sys.exit("config.py: _repo_dir tail not found")
s = s.replace(anchor, helper, 1)

old_spec = '''    return ProcessSpec(
        name="teleop",
        argv=tuple(_launcher_argv(env, cfg, teleop_dir)),
        cwd=_str(env, "TELEOP_CWD", "") or None,
        env=_child_env(env),
        kind=KIND_TELEOP,
        probe_script=str(teleop_dir / TELEOP_SCRIPT),
    )'''
new_spec = '''    child_env = _child_env(env)
    frozen = _frozen_teleop(env, teleop_dir)
    if frozen is not None:
        # 冻结版自带解释器；共享库在它旁边，模型仍从 teleop 源码树取。
        child_env["LD_LIBRARY_PATH"] = (
            str(frozen.parent)
            + (os.pathsep + child_env["LD_LIBRARY_PATH"]
               if child_env.get("LD_LIBRARY_PATH") else "")
        )
        child_env.setdefault(
            "LITEARM_TELEOP_MODELS", str(teleop_dir / "teleop" / "models")
        )
    return ProcessSpec(
        name="teleop",
        argv=tuple(_launcher_argv(env, cfg, teleop_dir)),
        cwd=_str(env, "TELEOP_CWD", "") or None,
        env=child_env,
        kind=KIND_TELEOP,
        probe_script=str(teleop_dir / TELEOP_SCRIPT),
    )'''
if old_spec not in s:
    sys.exit("config.py: _teleop_spec tail not found")
s = s.replace(old_spec, new_spec, 1)

old_argv = '''    argv = [
        _str(env, "TELEOP_LAUNCH_PY", "python3"),
        str(teleop_dir / TELEOP_LAUNCHER),
        "--host", _str(env, "TELEOP_BIND_HOST", "127.0.0.1"),
        "--port", str(cfg.teleop_port),
    ]
    server_py = (env.get("TELEOP_SERVER_PY") or "").strip()
    if server_py:
        argv += ["--metavr-py", server_py]
'''
new_argv = '''    frozen = _frozen_teleop(env, teleop_dir)
    if frozen is not None:
        # 冻结版自带解释器：参数与 launcher 透传给服务端的一致，但没有 --metavr-py。
        argv = [
            str(frozen),
            "--config", str(teleop_dir / "teleop" / "config" / "teleop.yaml"),
            "--host", _str(env, "TELEOP_BIND_HOST", "127.0.0.1"),
            "--port", str(cfg.teleop_port),
        ]
    else:
        argv = [
            _str(env, "TELEOP_LAUNCH_PY", "python3"),
            str(teleop_dir / TELEOP_LAUNCHER),
            "--host", _str(env, "TELEOP_BIND_HOST", "127.0.0.1"),
            "--port", str(cfg.teleop_port),
        ]
        server_py = (env.get("TELEOP_SERVER_PY") or "").strip()
        if server_py:
            argv += ["--metavr-py", server_py]
'''
if old_argv not in s:
    sys.exit("config.py: _launcher_argv head not found")
s = s.replace(old_argv, new_argv, 1)
cfg.write_text(s, encoding="utf-8")

p = pre.read_text(encoding="utf-8")
old = '''    if spec is None:
        return Check("teleop_python", INFO, "未托管 teleop，跳过依赖检查")

    if spec.kind != KIND_TELEOP:'''
new = '''    if spec is None:
        return Check("teleop_python", INFO, "未托管 teleop，跳过依赖检查")

    if spec.argv:
        exe0 = Path(os.path.expanduser(spec.argv[0]))
        if (exe0.name == "teleop-server" and exe0.parent.name == "frozen-teleop"
                and exe0.is_file()):
            return Check("teleop_python", OK,
                         f"冻结版 teleop：{exe0}（自带解释器，不需要 metavr-arm）")

    if spec.kind != KIND_TELEOP:'''
if old not in p:
    sys.exit("preflight.py: _check_teleop_python head not found")
pre.write_text(p.replace(old, new, 1), encoding="utf-8")
print("patched frozen teleop support")
PY

# ── 4c. teleop：冻结版的模型路径（LITEARM_TELEOP_MODELS） ──────────────────
note "4c/8 给 teleop 打模型路径补丁（冻结版）"
python3 - "$PKG/submodules/litearm-vr-teleop/teleop/src/litearm_teleop/teleop_sim.py" <<'PY'
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

# ── 5. build-info ───────────────────────────────────────────────────────────
note "5/8 写 build-info.json"
cat > "$PKG/build-info.json" <<EOF
{
  "package": "$PKGNAME",
  "version": "$VERSION",
  "built_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "source": "$(basename "$SRC")",
  "source_commit": "$(git -C "$SRC" rev-parse HEAD)",
  "source_dirty": $( [ -n "$(git -C "$SRC" status --porcelain)" ] && echo true || echo false ),
  "components": ["litearm-vr-bridge", "litearm-vr-teleop", "litearm-vr-follow"],
  "teleop_frozen": true,
  "teleop_frozen_built_from": "this repository's teleop source",
  "bundled_sdk": {
    "litearm-python": "$LITEARM_PYTHON_REF",
    "litegrip-python": "$LITEGRIP_PYTHON_REF"
  },
  "can_dkms_commit": "$CAN_COMMIT",
  "can_dkms_license": "GPL-2.0-only",
  "wheelhouse": "cp310/abi3 offline wheels (websockets, eclipse-zenoh, tomli, setuptools, wheel, pyserial)"
}
EOF

# ── 6. CAN DKMS payload ─────────────────────────────────────────────────────
note "6/8 合并 CAN DKMS payload"
CANDEB="$CAN_DEB"
[ -f "$CANDEB" ] || die "找不到 DKMS deb：$CANDEB"
dpkg-deb -x "$CANDEB" "$STAGE"
CANCTL="$WORK/canctl"
mkdir -p "$CANCTL"
dpkg-deb -e "$CANDEB" "$CANCTL"
install -m 755 "$CANCTL/postinst" "$STAGE/usr/lib/$PKGNAME/can-postinst"
install -m 755 "$CANCTL/prerm" "$STAGE/usr/lib/$PKGNAME/can-prerm"

# ── 7. DEBIAN 控制文件 ──────────────────────────────────────────────────────
note "7/8 写 DEBIAN/{control,postinst,prerm,postrm}"
cat > "$STAGE/DEBIAN/control" <<EOF
Package: $PKGNAME
Version: $VERSION
Section: utils
Priority: optional
Architecture: amd64
Maintainer: LiteArm Team <support@litearm.com>
Installed-Size: $(du -sk "$STAGE" | cut -f1)
Depends: python3 (>= 3.10), python3-venv, dkms, build-essential
Recommends: linux-headers-generic | linux-headers-realtime
Conflicts: litearm-vr-bridge, litearm-can-dkms
Replaces: litearm-vr-bridge, litearm-can-dkms
Description: LiteArm VR bridge + CAN DKMS driver (combined package)
 Single-package delivery built from the LiteArm VR stack repository: the VR
 teleop bridge plus frozen teleop, upstream follow, SDKs, offline wheelhouse,
 CAN driver sources, udev rules and systemd units.
 .
 postinst configures the CAN DKMS module and installs the bridge as a
 systemd service for the installing user.
 .
 Health check: /opt/litearm/litearm-vr-bridge/scripts/up.sh --doctor
EOF

cat > "$STAGE/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e
[ -x /usr/lib/litearm-vr-bundle/can-postinst ] && /usr/lib/litearm-vr-bundle/can-postinst configure || true
[ -x /usr/lib/litearm-vr-bundle/bridge-postinst ] && /usr/lib/litearm-vr-bundle/bridge-postinst "$@" || true
exit 0
EOF

cat > "$STAGE/DEBIAN/prerm" <<'EOF'
#!/bin/sh
set -e
case "$1" in
    remove|upgrade|deconfigure)
        [ -x /usr/lib/litearm-vr-bundle/can-prerm ] \
            && /usr/lib/litearm-vr-bundle/can-prerm remove || true
        ;;
esac
exit 0
EOF

cat > "$STAGE/DEBIAN/postrm" <<'EOF'
#!/bin/sh
# $1 = remove | purge | upgrade | ...
set -e
case "$1" in
    remove|purge) ;;
    *) exit 0 ;;
esac

if [ -f /etc/systemd/system/litearm-vr-bridge.service ]; then
    systemctl disable --now litearm-vr-bridge >/dev/null 2>&1 || true
fi

RUNAS="$(command -v runuser || command -v sudo || true)"
for home in /home/* /root; do
    unit="$home/.config/systemd/user/litearm-vr-bridge.service"
    [ -f "$unit" ] || continue
    u="$(basename "$home")"
    uid="$(id -u "$u" 2>/dev/null || echo)"
    if [ -n "$uid" ] && [ -n "$RUNAS" ]; then
        "$RUNAS" -u "$u" -- env "XDG_RUNTIME_DIR=/run/user/$uid" \
            systemctl --user disable --now litearm-vr-bridge >/dev/null 2>&1 || true
    fi
    if [ "$1" = "purge" ]; then
        rm -f "$unit"
        echo "[litearm-vr-bridge] 已停掉、禁用并删掉 $u 的用户级单元"
    else
        echo "[litearm-vr-bridge] 已停掉并禁用 $u 的用户级服务（单元文件仍在 $unit）"
    fi
done

for path in "/etc/litearm-vr-bridge.toml" "/etc/litearm-vr-bridge.env" "/home/*/.config/litearm-vr-bridge"; do
    for p in $path; do
        [ -e "$p" ] || continue
        echo "[litearm-vr-bridge] 保留（是数据，卸载不动）：$p"
    done
done
exit 0
EOF

cat > "$STAGE/usr/lib/$PKGNAME/bridge-postinst" <<'EOF'
#!/bin/sh
# 装完自动把桥装成 systemd 服务。先预置离线 venv（root 做），再以操作员身份装服务。
set -e

PKG=/opt/litearm/litearm-vr-bridge
[ -d "$PKG" ] || exit 0

OP="${SUDO_USER:-}"
if [ -z "$OP" ] || [ "$OP" = "root" ]; then
    OP="$(getent passwd | awk -F: '$3 >= 1000 && $3 < 65534 {print $1; exit}')"
fi

manual() {
    cat <<MSG

litearm-vr-bridge 代码已装到 $PKG。
$1

用**你自己的账号**跑一次（不要 sudo）：

    $PKG/scripts/install.sh --user

依赖已在安装包的 wheelhouse/ 里（不联网）。无人值守的机器：
    sudo $PKG/scripts/install.sh --system

体检：$PKG/scripts/up.sh --doctor

MSG
    exit 0
}

prebuild_venv() {
    if [ -x "$PKG/.venv/bin/python" ]; then
        touch "$PKG/.venv/.bundle-prebuilt"
        return 0
    fi
    python3 -m venv "$PKG/.venv" || return 1
    "$PKG/.venv/bin/python" - "$PKG/wheelhouse" "$PKG/src" <<'PY' || return 1
import sys, zipfile
from pathlib import Path
import sysconfig
wh, src = Path(sys.argv[1]), sys.argv[2]
site = Path(sysconfig.get_paths()["purelib"])
site.mkdir(parents=True, exist_ok=True)
for wheel in sorted(wh.glob("*.whl")):
    with zipfile.ZipFile(wheel) as z:
        z.extractall(site)
(site / "litearm_vr_bridge_src.pth").write_text(src + "\n", encoding="utf-8")
PY
    "$PKG/.venv/bin/python" -c 'import litearm_vr_bridge, websockets, zenoh' || return 1
    chmod -R a+rX "$PKG/.venv"
    touch "$PKG/.venv/.bundle-prebuilt"
}

if ! prebuild_venv; then
    manual "（预置离线 venv 失败：本机 Python 与 wheelhouse 不匹配？日志见上。）"
fi

[ -n "$OP" ] || manual "（没认出该装给谁：没人在终端里 sudo 装，系统里也没有常规用户。）"
UID_OP="$(id -u "$OP" 2>/dev/null || echo)"
[ -n "$UID_OP" ] || manual "（认出来的用户 $OP 取不到 uid。）"

LOG=/tmp/litearm-vr-bridge-install.log
RUNAS="$(command -v runuser || command -v sudo || true)"

echo "[litearm-vr-bridge] 把服务装给用户 $OP（uid=$UID_OP）"

# 一、用户级（首选：图形会话、~/.config 天然对得上）
if [ -n "$RUNAS" ] && "$RUNAS" -u "$OP" -- env "XDG_RUNTIME_DIR=/run/user/$UID_OP" \
        "$PKG/scripts/install.sh" --user >"$LOG" 2>&1; then
    loginctl enable-linger "$OP" >/dev/null 2>&1 || true
    cat <<MSG

[litearm-vr-bridge] 装好了：**用户级** systemd 服务，跑在 $OP 名下，已在运行。
  体检：$PKG/scripts/up.sh --doctor
  状态：$PKG/scripts/up.sh --status
  起链路：在上位机（litearm-vr-studio）的 VR 页上点「开始遥操」

  夹爪的 CAN：已随本安装包提供，CAN 盒**插上就自动 up**（比特率默认 1000000；
  要改写在 /etc/default/litearm-can 里，然后重插一次盒子）。

MSG
    exit 0
fi

# 二、系统级（退回：无人值守 / 那人没有活动会话）
echo "[litearm-vr-bridge] 用户级装不上（多半是 $OP 没有活动会话），退回系统级…"
if SUDO_USER="$OP" "$PKG/scripts/install.sh" --system >>"$LOG" 2>&1; then
    cat <<MSG

[litearm-vr-bridge] 装好了：**系统级** systemd 服务（User=$OP），已在运行。
  ⚠ 系统服务拿不到图形会话 —— 要看 pybullet 预览窗口的话：
      sudo systemctl edit litearm-vr-bridge
      加 Environment=DISPLAY=:0 与 Environment=XAUTHORITY=/home/$OP/.Xauthority
      或改用用户级：sudo -u $OP $PKG/scripts/install.sh --user
  体检：$PKG/scripts/up.sh --doctor

MSG
    exit 0
fi

manual "（自动装服务没成功，日志在 $LOG。）"
EOF

chmod 755 "$STAGE/DEBIAN/postinst" "$STAGE/DEBIAN/prerm" "$STAGE/DEBIAN/postrm" \
          "$STAGE/usr/lib/$PKGNAME/bridge-postinst" \
          "$STAGE/usr/lib/$PKGNAME/can-postinst" \
          "$STAGE/usr/lib/$PKGNAME/can-prerm"

# ── 7b. 许可证与第三方声明 ──────────────────────────────────────────────────
note "7b/8 写入许可证与第三方声明"
DOCDIR="$STAGE/usr/share/doc/$PKGNAME"
mkdir -p "$DOCDIR"
cp "$SRC/LICENSE" "$DOCDIR/LICENSE"
cp "$SRC/NOTICE" "$DOCDIR/NOTICE"
cp "$SRC/THIRD_PARTY_NOTICES.md" "$DOCDIR/THIRD_PARTY_NOTICES.md"
cp "$HERE/debian/copyright" "$DOCDIR/copyright"

# 便携安装入口：tar.gz 解到 / 之后运行它，等价于 deb 的 postinst。
cat > "$STAGE/usr/lib/$PKGNAME/portable-install.sh" <<'EOF'
#!/bin/sh
set -e
[ -x /usr/lib/litearm-vr-bundle/can-postinst ] \
    && /usr/lib/litearm-vr-bundle/can-postinst configure || true
[ -x /usr/lib/litearm-vr-bundle/bridge-postinst ] \
    && /usr/lib/litearm-vr-bundle/bridge-postinst configure || true
exit 0
EOF
chmod 755 "$STAGE/usr/lib/$PKGNAME/portable-install.sh"

# ── 8. 出包 ─────────────────────────────────────────────────────────────────
note "8/8 打 deb 和 portable tarball"
DEB="$OUT/${PKGNAME}_${VERSION}_amd64.deb"
TARBALL="$OUT/${PKGNAME}_${VERSION}_amd64.tar.gz"
dpkg-deb --build --root-owner-group "$STAGE" "$DEB" >/dev/null
tar -C "$STAGE" --exclude='./DEBIAN' -czf "$TARBALL" .
( cd "$OUT" && sha256sum "$(basename "$DEB")" > "$(basename "$DEB").sha256" )
( cd "$OUT" && sha256sum "$(basename "$TARBALL")" > "$(basename "$TARBALL").sha256" )
DEB_SHA="$(sha256sum "$DEB" | cut -d' ' -f1)"
TARBALL_SHA="$(sha256sum "$TARBALL" | cut -d' ' -f1)"
cat > "$OUT/MANIFEST.json" <<EOF
{
  "package": "$PKGNAME",
  "version": "$VERSION",
  "built_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "source": "$(basename "$SRC")",
  "source_commit": "$(git -C "$SRC" rev-parse HEAD)",
  "source_dirty": $( [ -n "$(git -C "$SRC" status --porcelain)" ] && echo true || echo false ),
  "components": ["litearm-vr-bridge", "litearm-vr-teleop", "litearm-vr-follow"],
  "sdk_pins": {
    "litearm-python": "$LITEARM_PYTHON_REF",
    "litegrip-python": "$LITEGRIP_PYTHON_REF"
  },
  "can_dkms_commit": "$CAN_COMMIT",
  "can_dkms_license": "GPL-2.0-only",
  "artifacts": [
    {"name": "$(basename "$DEB")", "sha256": "$DEB_SHA", "format": "deb"},
    {"name": "$(basename "$TARBALL")", "sha256": "$TARBALL_SHA", "format": "tar.gz"}
  ]
}
EOF
ls -lh "$DEB" "$TARBALL"

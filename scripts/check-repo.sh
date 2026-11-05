#!/usr/bin/env bash
# Repository-level checks that do not need the component test dependencies.
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

fail() { echo "check failed: $*" >&2; exit 1; }

required=(
  LICENSE
  NOTICE
  README.md
  README.zh-CN.md
  QUICKSTART.md
  QUICKSTART.zh-CN.md
  THIRD_PARTY_NOTICES.md
  third_party/litearm-can-dkms/LICENSE
  third_party/litearm-can-dkms/SOURCE_COMMIT
)
for file in "${required[@]}"; do
  [ -s "$file" ] || fail "missing or empty: $file"
done

[ "$(sed -n '1p' README.md)" = 'English | [简体中文](README.zh-CN.md)' ] \
  || fail "README.md language switcher"
[ "$(sed -n '1p' README.zh-CN.md)" = '[English](README.md) | 简体中文' ] \
  || fail "README.zh-CN.md language switcher"
[ "$(sed -n '1p' QUICKSTART.md)" = 'English | [简体中文](QUICKSTART.zh-CN.md)' ] \
  || fail "QUICKSTART.md language switcher"
[ "$(sed -n '1p' QUICKSTART.zh-CN.md)" = '[English](QUICKSTART.md) | 简体中文' ] \
  || fail "QUICKSTART.zh-CN.md language switcher"

if grep -Rqs "LicenseRef-Proprietary" --include='*.toml' --include='*.md' .; then
  fail "LicenseRef-Proprietary is still present"
fi

if grep -Rqs "litearm-vr-headset" README.md README.zh-CN.md QUICKSTART.md QUICKSTART.zh-CN.md; then
  fail "public docs link to the private headset repository"
fi

python3 - <<'PY'
import json
from pathlib import Path

for path in (Path(".releaserc.json"), Path(".nexform-standards.json")):
    if path.exists():
        json.loads(path.read_text(encoding="utf-8"))
PY

while IFS= read -r -d '' script; do
  bash -n "$script"
done < <(find packaging scripts litearm-vr-bridge litearm-vr-teleop \
  litearm-vr-follow third_party/litearm-can-dkms -type f -name '*.sh' -print0)

source_commit="$(tr -d '[:space:]' < third_party/litearm-can-dkms/SOURCE_COMMIT)"
case "$source_commit" in
  [0-9a-f][0-9a-f]*)
    [ "${#source_commit}" -eq 40 ] || fail "SOURCE_COMMIT is not a 40-char SHA"
    ;;
  *)
    fail "SOURCE_COMMIT is not a hex SHA"
    ;;
esac

python3 - <<'PY'
import re
from pathlib import Path

files = [
    Path("README.md"),
    Path("README.zh-CN.md"),
    Path("QUICKSTART.md"),
    Path("QUICKSTART.zh-CN.md"),
    Path("docs/index.md"),
    Path("docs/index.zh-CN.md"),
    Path("docs/host-application.md"),
    Path("docs/host-application.zh-CN.md"),
    Path("docs/setup.md"),
    Path("docs/setup.zh-CN.md"),
    Path("packaging/README.md"),
    Path("packaging/README.zh-CN.md"),
]
missing = []
for path in files:
    text = path.read_text(encoding="utf-8")
    for target in re.findall(r"\]\(([^)]+)\)", text):
        if target.startswith(("http://", "https://", "#", "mailto:")):
            continue
        target = target.split("#", 1)[0]
        if not target:
            continue
        if not (path.parent / target).resolve().exists():
            missing.append(f"{path}: {target}")
if missing:
    raise SystemExit("broken links:\n" + "\n".join(missing))
PY

echo "repository checks OK"

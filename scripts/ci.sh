#!/usr/bin/env bash
# Local and CI checks for the litearm-vr monorepo.
#
# The workflow installs the component dependencies first; this script only runs
# the checks. Usage:
#
#   bash scripts/ci.sh bridge
#   bash scripts/ci.sh teleop
#   bash scripts/ci.sh follow
#   bash scripts/ci.sh all
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

run_bridge() {
  ( cd litearm-vr-bridge && python -m unittest discover -s tests -t . )
}

run_teleop() {
  python -m compileall -q litearm-vr-teleop/teleop/src \
    litearm-vr-teleop/teleop/scripts litearm-vr-teleop/teleop/launch
  (
    cd litearm-vr-teleop
    export PYTHONPATH=teleop/src
    export LITEARM_NO_VIEWER=1
    for test in teleop/tests/test_*.py; do
      echo "== $test"
      python "$test"
    done
  )
}

run_follow() {
  (
    cd litearm-vr-follow
    python -m compileall -q .
    for script in run_*.sh; do
      bash -n "$script"
    done
    python test_status_msgs.py
    python test_gripper_loop.py
    python test_measured_state.py
  )
}

case "${1:-all}" in
  bridge) run_bridge ;;
  teleop) run_teleop ;;
  follow) run_follow ;;
  all)
    run_bridge
    run_teleop
    run_follow
    ;;
  *)
    echo "usage: $0 [bridge|teleop|follow|all]" >&2
    exit 2
    ;;
esac

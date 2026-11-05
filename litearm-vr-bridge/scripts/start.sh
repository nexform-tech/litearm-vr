#!/usr/bin/env bash
# 双臂真机一键起停（本机配置：left=ttyACM0/9001，right=ttyACM1/9002）。
#
#   ./scripts/start.sh              # 起两个 follow server → 桥 + teleop(--link)
#   ./scripts/start.sh --stop       # 停桥/teleop → 停两个 follow server
#   ./scripts/start.sh --status     # 看一眼现状（follow 端口 + 桥链路）
#   ./scripts/start.sh -- --no-venv # `--` 之后的参数原样转给 up.sh
#
# 为什么要有它：桥只持有**一个** follow 端点（FOLLOW_SERVER_ENDPOINT），管不了两臂。
# 本机把 FOLLOW_SERVER_CMD 留空（见 ~/.config/litearm-vr-bridge/env），于是两个
# follow server 得手工起 —— 本脚本就是把那两步 + up.sh --link 合成一条。
#
# 前提：
#   1) 机械臂已上电、工作区无人、急停在手（首次按 grip 会 movej 回 q_home，动作很大）。
#   2) 在**图形会话**的终端里跑 —— GUI teleop 需要 DISPLAY/XAUTHORITY/DBUS/XDG_RUNTIME_DIR。
#   3) follow 仓与 bridge 仓是同级目录（monorepo 布局）；否则用 FOLLOW_DIR= 指定。
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${LOG_DIR:-$ROOT/logs}"

# 双臂参数。默认值与 ~/.config/litearm-vr-bridge/env 的
# TELEOP_REAL_LEFT(9001=left) / TELEOP_ARM_HOST(9002=right) 对应；改了 env 就同步改这里。
LEFT_SERIAL="${FOLLOW_LEFT_SERIAL:-/dev/ttyACM0}"
RIGHT_SERIAL="${FOLLOW_RIGHT_SERIAL:-/dev/ttyACM1}"
LEFT_PORT="${FOLLOW_LEFT_PORT:-9001}"
RIGHT_PORT="${FOLLOW_RIGHT_PORT:-9002}"
FOLLOW_MODE="${FOLLOW_MODE:-jf}"

ACTION=up
EXTRA=()

die() { echo "错误：$*" >&2; exit 1; }
note() { echo "[$1] ${*:2}"; }

usage() {
  sed -n '2,9p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  cat <<'EOF'

可用环境变量：
  FOLLOW_DIR            follow 仓路径（默认找 ../litearm-vr-follow）
  FOLLOW_LEFT_SERIAL   左臂串口（默认 /dev/ttyACM0）
  FOLLOW_RIGHT_SERIAL  右臂串口（默认 /dev/ttyACM1）
  FOLLOW_LEFT_PORT     左臂 TCP 端口（默认 9001）
  FOLLOW_RIGHT_PORT    右臂 TCP 端口（默认 9002）
  FOLLOW_MODE          follow 跟随模式（默认 jf）
  LOG_DIR              日志/pid 目录（默认 <bridge>/logs）
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --stop) ACTION=stop; shift ;;
    --status) ACTION=status; shift ;;
    -h|--help) usage; exit 0 ;;
    *) EXTRA+=("$1"); shift ;;   # 其余原样转给 up.sh
  esac
done

# ── follow 仓定位 ───────────────────────────────────────────────────────────
locate_follow() {
  local c
  if [ -n "${FOLLOW_DIR:-}" ]; then
    [ -d "$FOLLOW_DIR" ] || die "FOLLOW_DIR 不存在：$FOLLOW_DIR"
    return
  fi
  for c in "$ROOT/../litearm-vr-follow" "$ROOT/litearm-vr-follow"; do
    if [ -f "$c/run_stm32_server.sh" ]; then FOLLOW_DIR="$(cd -- "$c" && pwd)"; return; fi
  done
  die "找不到 follow 仓（找过 ../litearm-vr-follow）。用 FOLLOW_DIR=<路径> 指定，或确认 monorepo 布局。"
}

# ── 探针 ────────────────────────────────────────────────────────────────────
port_listening() {
  python3 - "$1" <<'PY' >/dev/null 2>&1
import socket, sys
s = socket.socket()
s.settimeout(0.3)
try:
    s.connect(("127.0.0.1", int(sys.argv[1])))
    sys.exit(0)
except OSError:
    sys.exit(1)
finally:
    s.close()
PY
}

bridge_alive() {
  python3 - "${BRIDGE_PORT:-8765}" <<'PY' >/dev/null 2>&1
import sys, urllib.request
try:
    with urllib.request.urlopen(f"http://127.0.0.1:{sys.argv[1]}/healthz", timeout=1.0) as r:
        sys.exit(0 if r.status == 200 else 1)
except Exception:
    sys.exit(1)
PY
}

follow_pid() {  # <left|right> -> echo pid 或空
  local f="$LOG_DIR/follow-$1.pid"
  [ -f "$f" ] || return 0
  local pid; pid="$(cat "$f" 2>/dev/null || true)"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && echo "$pid"
  return 0
}

# ── 起 / 停 follow server ───────────────────────────────────────────────────
start_follow() {  # <name> <serial> <port>
  local name="$1" serial="$2" port="$3"
  local pidfile="$LOG_DIR/follow-$name.pid" log="$LOG_DIR/follow-$name.out"
  local pid; pid="$(follow_pid "$name")"
  if [ -n "$pid" ]; then note follow "$name 已在运行（pid $pid）"; return 0; fi
  if port_listening "$port"; then
    note follow "$name 已在监听 :$port（非本脚本所起），跳过"; return 0
  fi
  [ -e "$serial" ] || note 警告 "$name 串口不存在：$serial（USB 没枚举？服务会连不上）"
  mkdir -p "$LOG_DIR"
  note follow "启动 $name：$serial → :$port（日志 $log）"
  nohup bash "$FOLLOW_DIR/run_stm32_server.sh" \
    --mode "$FOLLOW_MODE" --port "$serial" --tcp-port "$port" \
    >>"$log" 2>&1 </dev/null &
  echo $! >"$pidfile"
}

stop_follow() {  # <name>
  local name="$1" pidfile="$LOG_DIR/follow-$name.pid"
  local pid; pid="$(follow_pid "$name")"
  rm -f "$pidfile"
  if [ -z "$pid" ]; then note follow "$name 没在跑"; return 0; fi
  note follow "停止 $name pid=$pid（先 movej 当前角就地持位，再退出）"
  kill -TERM "$pid" 2>/dev/null || true
  local i=0
  while kill -0 "$pid" 2>/dev/null && [ "$i" -lt 75 ]; do sleep 0.2; i=$((i + 1)); done
  if kill -0 "$pid" 2>/dev/null; then
    note 警告 "$name 15 秒没退出，SIGKILL"
    kill -KILL "$pid" 2>/dev/null || true
  fi
}

wait_port() {  # <port>
  local i=0
  while [ "$i" -lt 50 ]; do
    port_listening "$1" && return 0
    sleep 0.2; i=$((i + 1))
  done
  return 1
}

# ── 分派 ────────────────────────────────────────────────────────────────────

if [ "$ACTION" = status ]; then
  for side in left right; do
    port=$([ "$side" = left ] && echo "$LEFT_PORT" || echo "$RIGHT_PORT")
    pid="$(follow_pid "$side")"
    if [ -n "$pid" ]; then note follow "$side 运行中 pid=$pid :$port"
    elif port_listening "$port"; then note follow "$side :$port 有服务（非本脚本所起）"
    else note follow "$side 未运行"; fi
  done
  "$ROOT/scripts/up.sh" --status
  exit $?
fi

if [ "$ACTION" = stop ]; then
  # 先停桥（它会 latch + 收掉 teleop），再停 follow —— 反过来会让 arm 在 teleop 还活着时掉链
  "$ROOT/scripts/up.sh" --stop "${EXTRA[@]+"${EXTRA[@]}"}"
  stop_follow left
  stop_follow right
  exit 0
fi

# ── up ──────────────────────────────────────────────────────────────────────
locate_follow

start_follow left "$LEFT_SERIAL" "$LEFT_PORT"
start_follow right "$RIGHT_SERIAL" "$RIGHT_PORT"

wait_port "$LEFT_PORT"  || note 警告 "左臂 :$LEFT_PORT 还没起来，看 $LOG_DIR/follow-left.out（teleop 会 2s 一次重连）"
wait_port "$RIGHT_PORT" || note 警告 "右臂 :$RIGHT_PORT 还没起来，看 $LOG_DIR/follow-right.out（teleop 会 2s 一次重连）"
note follow "两个 follow server 就绪"

LINK_ARGS=(--link --yes)
# 桥已经在跑时，up.sh 的 doctor 会把它自己的端口(8765/7447)当成被占用而 FAIL —— 跳过预检
if bridge_alive; then
  case " ${EXTRA[*]-} " in *" --no-check "*) ;; *) LINK_ARGS+=(--no-check) ;; esac
fi

echo
note 一键 "启动桥 + teleop："
"$ROOT/scripts/up.sh" "${LINK_ARGS[@]}" "${EXTRA[@]+"${EXTRA[@]}"}"

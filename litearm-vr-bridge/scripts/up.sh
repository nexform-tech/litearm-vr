#!/usr/bin/env bash
# litearm-vr-bridge 一键启动。
#
#   ./scripts/up.sh              # 预检 → 起桥 → 拉链路（见下面的"真机分档"）
#   ./scripts/up.sh --link       # 显式接上真机链路（拉起 follow_server + teleop）
#   ./scripts/up.sh --no-link    # 只起桥，不碰上游
#   ./scripts/up.sh --status     # 看一眼链路现状
#   ./scripts/up.sh --stop       # 停桥（走它自己的"先 latch 再收子进程"流程）
#
# 真机分档（重要）：生成的 teleop 命令里带 `--real` 时，本脚本**不会**自动拉起上游 ——
# 那一步要人明确敲 `--link`（非交互环境还要 `--yes`）。仿真（不带 --real）才一键到底。
# 理由见 README：一条命令把"监控垫片"和"真机控制栈上电"耦合成一个动作，不合适。
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

LOG_DIR="${LOG_DIR:-$ROOT/logs}"
PIDFILE="$LOG_DIR/bridge.pid"
BOOT_LOG="$LOG_DIR/bridge.out"

ENV_FILE=""
USE_VENV=1
DO_CHECK=1
USE_SERVICE=auto
LINK=auto          # auto | yes | no
ASSUME_YES=0
ACTION=up          # up | stop | status
PY=""

die() { echo "错误：$*" >&2; exit 1; }
note() { echo "[$1] ${*:2}"; }

usage() {
  sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  cat <<'EOF'

选项：
  --env FILE     指定 env 文件（默认依次找 ./bridge.env、/etc/litearm-vr-bridge.env、
                 ~/.config/litearm-vr-bridge/env）
  --link         接上真机链路：发 teleop.start，按配置拉起 follow_server + teleop
  --no-link      只起桥，不发 teleop.start
  --yes, -y      真机确认不问（非交互环境必须给）
  --no-service   不用 systemd，就在本机后台起一个（日志与 pid 落在 logs/）
  --no-venv      不用 .venv，用系统 python3
  --no-check     跳过 --doctor 预检
  --stop         停桥
  --status       打印一帧链路快照
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --env)        [ $# -ge 2 ] || die "--env 需要文件路径"; ENV_FILE="$2"; shift 2 ;;
    --link)       LINK=yes; shift ;;
    --no-link)    LINK=no; shift ;;
    --yes|-y)     ASSUME_YES=1; shift ;;
    --no-service) USE_SERVICE=no; shift ;;
    --no-venv)    USE_VENV=0; shift ;;
    --no-check)   DO_CHECK=0; shift ;;
    --stop)       ACTION=stop; shift ;;
    --status)     ACTION=status; shift ;;
    -h|--help)    usage; exit 0 ;;
    *)            die "未知参数：$1（-h 看用法）" ;;
  esac
done

# ── 配置 ────────────────────────────────────────────────────────────────────

load_env() {
  local file="$ENV_FILE" candidate
  if [ -z "$file" ]; then
    for candidate in "$ROOT/bridge.env" /etc/litearm-vr-bridge.env \
                     "$HOME/.config/litearm-vr-bridge/env"; do
      if [ -f "$candidate" ]; then file="$candidate"; break; fi
    done
  fi
  if [ -z "$file" ]; then
    note 配置 "没有 env 文件，用默认值（现场部署请跑 ./scripts/install.sh 生成一份）"
    return
  fi
  [ -f "$file" ] || die "env 文件不存在：$file"
  note 配置 "加载 $file"
  parse_env "$file"
}

# 按 systemd EnvironmentFile 的语义读 KEY=VALUE：一行一个、整行注释、
# 值可加引号、**不执行任何东西**。
#
# 不能用 `.` source：那样 `FOLLOW_SERVER_CMD=python3 /path/x.py --port 9001` 里的
# 空格会被当成命令分隔，而且文件里出现 $(...) 就会被执行 —— 同一个文件 systemd 也读，
# 两边必须是一套语义（systemd 不支持行尾注释，所以示例里注释都单独占行）。
parse_env() {
  local file="$1" line key value
  while IFS= read -r line || [ -n "$line" ]; do
    line="${line#"${line%%[![:space:]]*}"}"           # 去左空白
    case "$line" in ''|'#'*) continue ;; esac
    case "$line" in *=*) ;; *) continue ;; esac
    key="${line%%=*}"
    value="${line#*=}"
    key="${key%"${key##*[![:space:]]}"}"              # 去右空白
    case "$key" in *[!A-Za-z0-9_]*) note 警告 "跳过不规范的行：$key"; continue ;; esac
    value="${value%$'\r'}"                            # 容忍 CRLF
    case "$value" in
      \"*\") value="${value#\"}"; value="${value%\"}" ;;
      \'*\') value="${value#\'}"; value="${value%\'}" ;;
    esac
    export "$key=$value"
  done < "$file"
}

pick_python() {
  if [ "$USE_VENV" = 1 ]; then
    if [ ! -x "$ROOT/.venv/bin/python" ]; then
      note 准备 "创建虚拟环境 .venv（首次会慢一点）"
      if python3 -m venv "$ROOT/.venv" >/dev/null 2>&1; then
        "$ROOT/.venv/bin/pip" install -q -e "$ROOT" >/dev/null 2>&1 \
          || "$ROOT/.venv/bin/pip" install -q "websockets>=13" "eclipse-zenoh>=1.0" >/dev/null 2>&1 \
          || true
      fi
    fi
    if [ -x "$ROOT/.venv/bin/python" ]; then PY="$ROOT/.venv/bin/python"; return; fi
    note 准备 "venv 建不起来，回落到系统 python3"
  fi
  PY="$(command -v python3)" || die "找不到 python3"
  "$PY" -c 'import websockets, zenoh' >/dev/null 2>&1 \
    || die '当前 python3 缺依赖：python3 -m pip install --user "websockets>=13" "eclipse-zenoh>=1.0"（或去掉 --no-venv）'
}

load_env
pick_python
# venv 里装了包就不用 PYTHONPATH；不然按 DEPLOY.md 的老办法指到 src/
if ! "$PY" -c 'import litearm_vr_bridge' >/dev/null 2>&1; then
  export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
fi

bridge() { "$PY" -m litearm_vr_bridge "$@"; }
# 判活走 `/healthz`（头显那个口上的只读 HTTP）而不是 `--ctl status`：后者要连 zenoh，
# 而"zenoh 挂了"恰好是我们最需要还能判活的时候。zenoh 的实情由 `--ctl status` 与
# `/healthz` 的 `zenoh` 段回答。
bridge_alive() {
  "$PY" - "${BRIDGE_PORT:-8765}" <<'EOF' >/dev/null 2>&1
import sys, urllib.request
try:
    with urllib.request.urlopen(f"http://127.0.0.1:{sys.argv[1]}/healthz", timeout=1.0) as r:
        sys.exit(0 if r.status == 200 else 1)
except Exception:
    sys.exit(1)
EOF
}

if [ "$ACTION" = status ]; then
  bridge --ctl status
  exit $?
fi

# ── 停 ──────────────────────────────────────────────────────────────────────

service_kind() {
  if [ -f "$HOME/.config/systemd/user/litearm-vr-bridge.service" ]; then
    echo user
  elif [ -f /etc/systemd/system/litearm-vr-bridge.service ]; then
    echo system
  else
    echo none
  fi
}

stop_local() {
  [ -f "$PIDFILE" ] || return 1
  local pid i=0
  pid="$(cat "$PIDFILE" 2>/dev/null || true)"
  rm -f "$PIDFILE"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null || return 1

  note 桥 "停止本地实例 pid=$pid（它会先 latch 就地持位，再收掉子进程）"
  kill -TERM "$pid" 2>/dev/null || true
  while kill -0 "$pid" 2>/dev/null && [ "$i" -lt 50 ]; do sleep 0.2; i=$((i + 1)); done
  if kill -0 "$pid" 2>/dev/null; then
    note 警告 "10 秒还没退出，SIGKILL"
    kill -KILL "$pid" 2>/dev/null || true
  fi
  return 0
}

if [ "$ACTION" = stop ]; then
  stopped=0
  case "$(service_kind)" in
    user)
      if systemctl --user stop litearm-vr-bridge; then
        stopped=1
        note 桥 "已停止（用户级 systemd）"
      else
        note 警告 "停用户级服务失败：systemctl --user status litearm-vr-bridge"
      fi ;;
    system)
      if systemctl stop litearm-vr-bridge 2>/dev/null; then
        stopped=1
        note 桥 "已停止（系统 systemd）"
      elif [ "$(id -u)" != 0 ]; then
        note 警告 "停系统服务需要权限：sudo systemctl stop litearm-vr-bridge"
      fi ;;
  esac
  if stop_local; then stopped=1; fi
  if [ "$stopped" = 0 ]; then note 桥 "没有找到在跑的桥"; fi
  exit 0
fi

# ── 预检 ────────────────────────────────────────────────────────────────────

if [ "$DO_CHECK" = 1 ]; then
  if ! bridge --doctor; then
    die "预检有 FAIL，先修好再启动（或者 --no-check 跳过，但你知道自己在做什么）"
  fi
fi

# ── 起桥 ────────────────────────────────────────────────────────────────────

start_local() {
  mkdir -p "$LOG_DIR"
  if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
    note 桥 "本地实例已在运行（pid $(cat "$PIDFILE")）"
    return
  fi
  note 桥 "本机后台启动，日志：$BOOT_LOG"
  nohup "$PY" -m litearm_vr_bridge >>"$BOOT_LOG" 2>&1 </dev/null &
  echo $! >"$PIDFILE"
}

wait_bridge() {
  local i=0
  while [ "$i" -lt 50 ]; do
    bridge_alive && return 0
    sleep 0.2
    i=$((i + 1))
  done
  return 1
}

if bridge_alive; then
  note 桥 "已在运行"
else
  kind="$(service_kind)"
  if [ "$USE_SERVICE" = no ]; then kind=none; fi
  case "$kind" in
    user)
      systemctl --user start litearm-vr-bridge || die "用户级服务起不来：systemctl --user status litearm-vr-bridge"
      note 桥 "已启动（用户级 systemd）" ;;
    system)
      if [ "$(id -u)" = 0 ]; then
        systemctl start litearm-vr-bridge || die "系统服务起不来：systemctl status litearm-vr-bridge"
      elif ! systemctl start litearm-vr-bridge 2>/dev/null; then
        die "起不了系统服务（需要 root）。二选一：
       sudo systemctl start litearm-vr-bridge
       ./scripts/up.sh --no-service        # 本机后台起一个"
      fi
      note 桥 "已启动（系统 systemd）" ;;
    none)
      start_local ;;
  esac

  wait_bridge || die "桥起来了但没响应，看 $BOOT_LOG（或 journalctl -u litearm-vr-bridge）"
  note 桥 "就绪"
fi

# ── 拉链路 ──────────────────────────────────────────────────────────────────

# 桥的 zenoh 监听端口：从 ZENOH_LISTEN_ENDPOINTS（tcp/host:port）里取最后一段
zenoh_port() {
  local ep="${ZENOH_LISTEN_ENDPOINTS:-tcp/0.0.0.0:7447}"
  echo "${ep##*:}"
}

lan_ip() {
  "$PY" - <<'PY' 2>/dev/null || true
import socket
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    sock.connect(("8.8.8.8", 80))
    print(sock.getsockname()[0])
except OSError:
    pass
PY
}

teleop_is_real() {
  # 桥自己算好的：teleop 是不是接真机（REAL=1，或自定义命令里带 --real）
  bridge --print-config 2>/dev/null | "$PY" -c '
import json, sys
try:
    cfg = json.load(sys.stdin)
except ValueError:
    cfg = {}
print("1" if cfg.get("real_arm") else "0")
'
}

confirm_real() {
  [ "$ASSUME_YES" = 1 ] && return 0
  echo
  echo "  ⚠  即将拉起 teleop 并接真机（--real）：首次按 grip 会 movej 回 q_home，然后跟随手柄。"
  echo "     确认：机械臂已上电、工作区无人、急停在手。"
  if [ -t 0 ]; then
    printf "     继续？[y/N] "
    read -r answer
    case "$answer" in
      y|Y|yes|YES) return 0 ;;
      *) echo "     已取消（桥还在跑，随时可以再敲 --link）。"; return 1 ;;
    esac
  fi
  die "非交互环境接真机必须显式确认：加 --yes"
}

real_arm=0
[ "$(teleop_is_real)" = 1 ] && real_arm=1

link_code=0
case "$LINK" in
  no)
    note 链路 "只起桥，没有接上游（--no-link）" ;;
  yes)
    if [ "$real_arm" = 1 ]; then confirm_real || exit 0; fi
    echo
    bridge --ctl up || link_code=$?
    ;;
  auto)
    if [ "$real_arm" = 1 ]; then
      echo
      note 链路 "检测到 teleop 接了真机（--real）：不自动拉起上游。"
      echo "        确认工作区无人、急停在手之后，敲："
      echo "            ./scripts/up.sh --link"
    else
      echo
      note 链路 "仿真模式：自动拉起上游"
      bridge --ctl up || link_code=$?
    fi
    ;;
esac

# 链路那步失败也把头显地址讲完 —— 那几行恰恰是操作员最需要的
echo
note 头显 "面板里填 ws://$(lan_ip):${BRIDGE_PORT:-8765}"
echo "        看门狗：>${FRAME_WARN_MS:-200}ms 无帧告警，>${FRAME_FALLBACK_MS:-500}ms 自动 hold 就地保持"
echo "        现状：./scripts/up.sh --status      停止：./scripts/up.sh --stop"
echo
# 上位机那侧要跑的东西。浏览器说不了原生 zenoh，得经这台 router 进来 ——
# 所以它不是"可选的调优"，没有它上位机连不上。teleop 与 ctl 不需要它。
note 上位机 "在**上位机那台机器**上跑（deploy/zenoh-router.service 就是它）："
echo "            zenoh-bridge-remote-api --connect tcp/$(lan_ip):$(zenoh_port) \\"
echo "                --ws-port 10000 --no-multicast-scouting"
echo "        然后 VR 页里填：ws://$(lan_ip):10000   arm_id=${ZENOH_ARM_ID:-armA}"
echo "        ⚠ 10000 要放行；arm_id 必须与这里、与 teleop 的一致"
exit "$link_code"

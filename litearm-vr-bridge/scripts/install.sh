#!/usr/bin/env bash
# litearm-vr-bridge 一次性安装：建 venv、写 env、装 systemd 单元、开机自启。
#
#   sudo ./scripts/install.sh          # 系统服务（默认；与 deploy/ 里的单元一致）
#        ./scripts/install.sh --user   # 用户级服务（图形会话 / 没有 sudo 时更合适）
#
# 装完之后的日常操作都在 up.sh：
#   ./scripts/up.sh                    # 起桥（真机分档见 up.sh 的说明）
#   ./scripts/up.sh --link             # 接上真机链路
#
# 幂等：重复跑不会覆盖已有的 env 文件，只会刷新单元。
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
UNIT_NAME="litearm-vr-bridge.service"

MODE=system
DO_ENABLE=1
ENV_SRC="$ROOT/deploy/bridge.env.example"

die() { echo "错误：$*" >&2; exit 1; }
note() { echo "[$1] ${*:2}"; }

usage() {
  cat <<'EOF'
用法：install.sh [--system|--user] [--env-src FILE] [--no-enable]

  --system        装成系统服务（默认，需要 root）
  --user          装成用户级服务（systemctl --user，别用 sudo 跑）
  --env-src FILE  用哪个文件做 env 模板（默认 deploy/bridge.env.example）
  --no-enable     只装，不 enable --now
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --system)    MODE=system; shift ;;
    --user)      MODE=user; shift ;;
    --env-src)   [ $# -ge 2 ] || die "--env-src 需要文件路径"; ENV_SRC="$2"; shift 2 ;;
    --no-enable) DO_ENABLE=0; shift ;;
    -h|--help)   usage; exit 0 ;;
    *)           die "未知参数：$1（-h 看用法）" ;;
  esac
done

if [ "$MODE" = system ]; then
  [ "$(id -u)" = 0 ] || die "装系统服务需要 root：sudo $0 $*（或者用 --user）"
  RUN_USER="${SUDO_USER:-root}"
  ENV_DST=/etc/litearm-vr-bridge.env
  UNIT_SRC="$ROOT/deploy/litearm-vr-bridge.service"
  UNIT_DST="/etc/systemd/system/$UNIT_NAME"
else
  [ "$(id -u)" != 0 ] || die "--user 不能用 sudo/root 跑（那会装成 root 的用户服务）"
  RUN_USER="$(id -un)"
  ENV_DST="$HOME/.config/litearm-vr-bridge/env"
  UNIT_SRC="$ROOT/deploy/litearm-vr-bridge.user.service"
  UNIT_DST="$HOME/.config/systemd/user/$UNIT_NAME"
fi

sysctl_() { if [ "$MODE" = user ]; then systemctl --user "$@"; else systemctl "$@"; fi; }

# ── 1. 上游 submodule ───────────────────────────────────────────────────────

# 桥要起停的上游（teleop / follow）是仓库里的 submodule。装了服务却忘了 init，
# 现场的症状是"桥起来了但什么都没托管" —— 在这里一次做掉。
note 1/6 "上游 submodule（teleop / follow）"
need_submodule=0
for sub in submodules/litearm-vr-teleop submodules/litearm-vr-follow; do
  [ -e "$ROOT/$sub/.git" ] || need_submodule=1
done
if [ "$need_submodule" = 0 ]; then
  note 上游 "已就位"
elif [ ! -e "$ROOT/.git" ]; then
  note 警告 "不是 git 工作区，拉不了 submodule —— 确认 $ROOT/submodules/ 下两个仓库都在"
elif [ "$MODE" = system ] && [ -n "${SUDO_USER:-}" ] && [ "$(stat -c %U "$ROOT" 2>/dev/null)" = "$SUDO_USER" ]; then
  # 别用 root 拉：submodule 里的文件会变成 root 属主，之后操作员自己 git pull 就费劲了
  note 上游 "以 $SUDO_USER 身份拉取"
  sudo -u "$SUDO_USER" git -C "$ROOT" submodule update --init \
    || note 警告 "submodule 拉取失败（网络？），先手工跑：git -C $ROOT submodule update --init"
else
  git -C "$ROOT" submodule update --init \
    || note 警告 "submodule 拉取失败（网络？），先手工跑：git -C $ROOT submodule update --init"
fi

# ── 2. venv ─────────────────────────────────────────────────────────────────

note 2/6 "准备虚拟环境 $ROOT/.venv"
if [ ! -x "$ROOT/.venv/bin/python" ]; then
  python3 -m venv "$ROOT/.venv" || die "建 venv 失败（需要 python3-venv 包？）"
fi
"$ROOT/.venv/bin/pip" install -q --upgrade pip >/dev/null 2>&1 || true
"$ROOT/.venv/bin/pip" install -q -e "$ROOT" \
  || die "装依赖失败（离线？）：$ROOT/.venv/bin/pip install -e $ROOT"
# 装完自检一下：import 不通说明环境有问题，别等到开机才发现
"$ROOT/.venv/bin/python" -c 'import litearm_vr_bridge, websockets, zenoh' \
  || die "venv 里 import 不了，检查 pip 输出"

# ── 3. env 文件 ─────────────────────────────────────────────────────────────

note 3/6 "配置文件 $ENV_DST"
mkdir -p "$(dirname "$ENV_DST")"
if [ -f "$ENV_DST" ]; then
  note 配置 "已存在，保持不动（要重置就自己删掉再跑）"
else
  cp "$ENV_SRC" "$ENV_DST"
  [ "$MODE" = system ] && chmod 644 "$ENV_DST"
  note 配置 "已生成 —— 至少要确认 FOLLOW_SERVER_MODE（stm32 / sdk）和串口/CAN 口"
fi

# 受管进程的 stdout/stderr 落在 PROCESS_LOG_DIR
if [ "$MODE" = system ]; then
  mkdir -p /var/log/litearm-vr-bridge
  chown "$RUN_USER" /var/log/litearm-vr-bridge 2>/dev/null || true
fi

# ── 4. systemd 单元 ─────────────────────────────────────────────────────────

note 4/6 "安装单元 $UNIT_DST"
mkdir -p "$(dirname "$UNIT_DST")"
# 把示例单元里的 /opt 路径换成真实路径，ExecStart 指向 venv（这样不用再设 PYTHONPATH，
# 也就不会把它透传给 teleop / follow_server 这些子进程）。
sed -e "s#/opt/litearm/litearm-vr-bridge#$ROOT#g" \
    -e "/^Environment=PYTHONPATH=/d" \
    -e "s#^ExecStart=.*#ExecStart=$ROOT/.venv/bin/python -m litearm_vr_bridge#" \
    "$UNIT_SRC" > "$UNIT_DST"

# ── 5. 图形会话（pybullet 窗口） ────────────────────────────────────────────

note 5/6 "图形会话（teleop 的 pybullet 窗口要用）"
if [ "$MODE" = system ]; then
  xauth="$(getent passwd "$RUN_USER" | cut -d: -f6)/.Xauthority"
  if [ -f "$xauth" ]; then
    mkdir -p "/etc/systemd/system/$UNIT_NAME.d"
    cat > "/etc/systemd/system/$UNIT_NAME.d/10-display.conf" <<EOF
# 由 scripts/install.sh 生成：让受管的 teleop 能在操作员桌面上开 pybullet 窗口。
# 若窗口仍起不来，试：xhost +SI:localuser:root  （或改用 ./scripts/install.sh --user）
[Service]
Environment=DISPLAY=${DISPLAY:-:0}
Environment=XAUTHORITY=$xauth
EOF
    note 图形 "已写入 $UNIT_NAME.d/10-display.conf（DISPLAY=${DISPLAY:-:0}，用户 $RUN_USER）"
  else
    note 图形 "没找到 $xauth，跳过；要 pybullet 窗口就手工加 DISPLAY/XAUTHORITY，或改用 --user"
  fi
else
  note 图形 "用户级服务继承当前会话；窗口起不来就补：systemctl --user import-environment DISPLAY XAUTHORITY"
fi

# ── 6. 生效 ─────────────────────────────────────────────────────────────────

if [ "$DO_ENABLE" = 1 ]; then
  note 6/6 "启用并启动"
  sysctl_ daemon-reload
  sysctl_ enable --now litearm-vr-bridge
  if [ "$MODE" = user ]; then
    loginctl enable-linger "$RUN_USER" 2>/dev/null \
      || note 提示 "开机自启要 sudo loginctl enable-linger $RUN_USER"
  fi
  sleep 1
  sysctl_ --no-pager status litearm-vr-bridge | head -12 || true
else
  note 6/6 "跳过启用（--no-enable）"
  sysctl_ daemon-reload || true
fi

cat <<EOF

装好了。接下来：

  1) 改配置：${EDITOR:-vi} $ENV_DST
     至少要确认 FOLLOW_SERVER_MODE（stm32 / sdk）、串口或 CAN 口，以及接不接真机
     （上游的 conda 环境由 submodules/litearm-vr-teleop 里的启动脚本自己找，
       找不到就 export TELEOP_SERVER_PY=<envs/metavr-arm/bin/python>）

  2) 体检：./scripts/up.sh --doctor

  3) 起链路：./scripts/up.sh            # 仿真会自动拉起上游
             ./scripts/up.sh --link     # 接真机（会先要你确认工作区无人、急停在手）

  日志：$( [ "$MODE" = system ] && echo "journalctl -u litearm-vr-bridge -f" || echo "journalctl --user -u litearm-vr-bridge -f" )
EOF

#!/usr/bin/env bash
# 容器内的一步（由 build.sh 把仓库根挂 /workspace 进来跑）：
#   打包 → 收共享库 → 断言关键包 → 自检（--help / 两条数据源各无头起一次 / 收 SIGTERM）。
#
# ⚠ 这个脚本只负责「产出一个自带解释器的 teleop-server」；它**不碰** src/、launch/、
#   config/ —— 那些是仓库自己的代码。冻结之后模型仍从仓库取（LITEARM_TELEOP_MODELS）。
set -euo pipefail

REPO=/workspace
PROJECT="$REPO/teleop"
SRC="$PROJECT/src"
SCRIPTS="$PROJECT/scripts"
HERE="$PROJECT/packaging/frozen"
BUILD_DIR="$REPO/packaging-build/frozen"     # PyInstaller 中间产物（.gitignore）
OUT="$REPO/dist/teleop-frozen"               # 最终产物
LOG="$BUILD_DIR/pyinstaller.log"

rm -rf "$BUILD_DIR" "$OUT"
mkdir -p "$BUILD_DIR"

# ---------------------------------------------------------------------------
# 1) PyInstaller（在容器里现装：镜像里没带它，几秒钟；要固化再挪进 Dockerfile）
# ---------------------------------------------------------------------------
# --user 是因为构建容器以宿主 uid 跑（见 build.sh 的 --user），写不了
# /usr/local/lib/python3.10/dist-packages；装完用 `python3 -m PyInstaller` 调，
# 不依赖 PATH 里有没有那个 console script。
echo "[1/5] 安装 PyInstaller"
python3 -m pip install --user --quiet --no-warn-script-location pyinstaller
echo "      $(python3 -m PyInstaller --version)"

# ---------------------------------------------------------------------------
# 2) 打包（onedir）
# ---------------------------------------------------------------------------
# --onedir（不是 --onefile）：LD_LIBRARY_PATH 要指向「可执行文件所在目录」，
#   onefile 运行时才解压到 /tmp，那条路对不上调用方的契约。
#
# --contents-directory .：**关键**。PyInstaller 6 默认把数据/共享库塞进
#   `<名字>/_internal/`，而契约要求「共享库与可执行文件同目录、不要子目录」；
#   给 `.` 就是退回 5.x 的平铺布局（此时 sys._MEIPASS 也正好 = 可执行文件所在目录）。
#   注：PyInstaller 仍会把「靠 $ORIGIN 相对路径找兄弟库」的那几坨（cmeel.prefix/lib、
#   numpy.libs、scipy.libs）连同目录层级一起摆好，并在顶层留**相对符号链接**；
#   于是 soname 在顶层就能解析，[3/5] 会验证这些链接一个都不许悬空。
#
# 不用 PyInstaller 的 `--strip`：它一视同仁地剥掉符号表，而 numpy 的 `*.libs` 里
#   那个 OpenBLAS 被剥完会加载不了（`libscipy_openblas64_-56d6093b.so: ELF load
#   command address/offset not page-aligned`，2026-10-16 实测，四道自检当场变红）。
#   调试信息那一坨确实值钱（pybullet 的 .so 136 MB → `strip --strip-debug` 后 9.4 MB），
#   但要在**只删调试节**的前提下做，见 [3/5] 的「剥调试节」那一步。
#
# --collect-all：编译型扩展与「运行时才 import」的东西，PyInstaller 的静态分析
#   看不见，必须整包收：
#     pinocchio/eigenpy/coal/hppfcl  cmeel 轮子，装在 cmeel 前缀（非标准路径）
#     pink/qpsolvers/quadprog/loop_rate_limiters  pin IK 那一串；
#        quadprog 是 qpsolvers 在 try/except 里探到的动态后端
#     zenoh                         出厂数据源，函数体内 import；abi3 扩展
#     websockets                    用 lazy_import（__getattr__ + __import__）在
#        运行时才 import `websockets.asyncio` / `.asyncio.server` —— 漏了的症状是
#        「服务起得来、一到 websockets.serve 就 ModuleNotFoundError」
#     pybullet/litearm_teleop       本仓代码与仿真库（静态 import 也能收到，
#        collect-all 是防它内部有动态 import 的保险）
# --hidden-import real_arm_client/probe：scripts/ 下的顶层模块，
#   `--real` / `--probe` 时才 import，静态分析收不到。
# --exclude-module rclpy/ros_joint_state：ROS2 不冻（与启动脚本的说明一致：冻结产物里
#   没有 rclpy，服务端自己降级成「不发 /joint_states」），排除掉省得为收不到的 ROS 刷警告。
#   另有几个常见瘦身项（tkinter/matplotlib/PIL/IPython/pytest/setuptools/pkg_resources）
#   **一个都没被收进来**（量过：产物顶层压根没有这些目录），所以不加空转的排除项。
echo "[2/5] PyInstaller onedir 打包"
t0=$SECONDS
python3 -m PyInstaller \
    --noconfirm --clean --onedir \
    --contents-directory . \
    --name teleop-server \
    --distpath "$BUILD_DIR/dist" \
    --workpath "$BUILD_DIR/work" \
    --specpath "$BUILD_DIR" \
    --paths "$SRC" --paths "$SCRIPTS" \
    --collect-all litearm_teleop \
    --collect-all pinocchio \
    --collect-all eigenpy \
    --collect-all coal \
    --collect-all hppfcl \
    --collect-all pink \
    --collect-all qpsolvers \
    --collect-all quadprog \
    --collect-all loop_rate_limiters \
    --collect-all zenoh \
    --collect-all websockets \
    --collect-all pybullet \
    --hidden-import real_arm_client \
    --hidden-import probe \
    --exclude-module rclpy \
    --exclude-module ros_joint_state \
    "$SCRIPTS/ws_echo_server.py" >"$LOG" 2>&1 || {
        echo "      失败：PyInstaller 退出码非 0，日志尾部："
        tail -n 40 "$LOG"
        exit 1
    }
echo "      用时 $((SECONDS - t0))s（完整日志 $LOG）"

PKG="$BUILD_DIR/dist/teleop-server"
[ -x "$PKG/teleop-server" ] || { echo "错误：没找到 PyInstaller 产物 $PKG/teleop-server"; exit 1; }

# ---------------------------------------------------------------------------
# 3) 摆成最终布局：可执行文件 + 共享库都**能在同一层被找到**
# ---------------------------------------------------------------------------
echo "[3/5] 摆成最终布局 $OUT"
mkdir -p "$OUT"
cp -a "$PKG"/. "$OUT"/

# 收共享库：cmeel 前缀（libpinocchio_default/libcoal/libboost_*/liburdfdom_*…）、
# numpy.libs、scipy.libs —— PyInstaller 的依赖分析能收一部分，剩下的由闭包补齐。
python3 "$HERE/collect_libs.py" "$OUT"

# 瘦身：只删调试节（`strip --strip-debug`，不动符号表和动态节）。
# 值钱的是 pybullet 那个 .so：136 MB → 9.4 MB，整包 427 MB → 298 MB（-30%，2026-10-16 实测）。
# 之所以不用 PyInstaller 的 `--strip`：那个连符号表一起剥，OpenBLAS 会加载不了
# （见文件头 [2/5] 那段注释）。剥完紧接着就是四道自检，弄坏了当场红。
# ⚠ 跳过 `*.libs/`：那是 numpy/scipy 轮子里自带的大页对齐二进制（OpenBLAS、gfortran）。
#   容器里的 binutils 2.38 剥完会让 PT_LOAD 的地址/偏移不再页对齐，加载器直接报
#   「libscipy_openblas64_-56d6093b.so: ELF load command address/offset not page-aligned」，
#   numpy 起不来（同一份文件在 Debian 的 binutils 2.44 下剥完却没事）。这点调试信息不值当，
#   反正大头的 127 MB 在 pybullet 那个 .so 上。
echo "      剥调试节（跳过 *.libs/）"
n_stripped=0; saved_kb=0
while IFS= read -r f; do
    case "$f" in *.libs/*) continue;; esac
    head -c 4 "$f" 2>/dev/null | grep -q ELF || continue
    before=$(stat -c%s "$f")
    strip --strip-debug "$f" 2>/dev/null || continue
    n_stripped=$((n_stripped + 1))
    saved_kb=$((saved_kb + (before - $(stat -c%s "$f")) / 1024))
done < <(find "$OUT" -type f)
echo "        $n_stripped 个 ELF 去掉调试节，省 ${saved_kb} KB"

# 关键包断言：**缺了就是构建失败，不产出残缺产物**（照 litearm-studio 的
# packaging/build.py 的做法）。这里判的是「冻出来的东西真能干活」的两根柱子：
#   pinocchio = --ik pin 的后端（它不在 pink 也在不了）；
#   pybullet   = 仿真本体（模型/机器人都在它里面跑）。
# 只看 Python 包在不在没用 —— 它们链接的原生库（libpinocchio_default …）才是
# 运行时真正 dlopen 的东西，所以两边都要有。
echo "      关键包断言"
assert_file() {  # $1=说明 $2=find 模式
    local hit
    hit=$(find "$OUT" -name "$2" -print -quit)
    if [ -z "$hit" ]; then
        echo "      失败：产物里没有 $1（$2）—— 构建失败，不出残缺产物"
        exit 1
    fi
    echo "        $1 ← ${hit#"$OUT"/}（$(du -h "$hit" | cut -f1)）"
}
assert_file "pinocchio 扩展" "pinocchio_pywrap*.so"
assert_file "pinocchio 原生库" "libpinocchio_default.so*"
assert_file "pybullet 扩展" "pybullet*.so"
assert_file "内嵌解释器" "libpython3.10.so.1.0"

# 顶层那一堆是**相对符号链接**（PyInstaller 用来保住目录层级又让 soname 在顶层可解析）。
# 绝对路径或悬空的链接 = 拷到客户机上必然找不到库，判失败。
bad_abs=$(find "$OUT" -type l -lname '/*' | head -5)
dangling=$(find "$OUT" -xtype l | head -5)
if [ -n "$bad_abs" ] || [ -n "$dangling" ]; then
    echo "      失败：产物里有指向外部的或悬空的符号链接："
    printf '        %s\n' $bad_abs $dangling
    exit 1
fi
echo "        符号链接 $(find "$OUT" -type l | wc -l) 个，全部相对且可解析"

# ---------------------------------------------------------------------------
# 4) 自检：--help（不需要模型，但会把 numpy/pybullet/websockets 全 import 一遍）
# ---------------------------------------------------------------------------
echo "[4/5] 自检 --help（不给 LD_LIBRARY_PATH）"
env -u LD_LIBRARY_PATH "$OUT/teleop-server" --help >"$BUILD_DIR/help.txt" 2>&1 || {
    echo "      失败：--help 非 0 退出，尾部："
    tail -n 20 "$BUILD_DIR/help.txt"
    exit 1
}
grep -E '^  --(config|source|viewer|ik|real|dual|arm-id)' "$BUILD_DIR/help.txt" | sed 's/^/      /'

# ---------------------------------------------------------------------------
# 5) 自检：无头起服务（真加载 URDF + 真建 pin IK 求解器），再收 SIGTERM
# ---------------------------------------------------------------------------
# 在**产物目录之外**（/tmp）跑，且只给它自己这一层当 LD_LIBRARY_PATH —— 容器里
# 装着全套依赖，从 /tmp 跑才看得出「有没有偷偷用到宿主的库」。
#
# 容器里没有 DISPLAY，所以走 --viewer rviz（= pybullet DIRECT，无窗口）。
# ⚠ 注意 `LITEARM_NO_VIEWER` 这个环境变量**当前代码里没有任何地方读**（只在
#   tests/ 与 docs/ 里出现），别指望它能把 GUI 关掉 —— 要无窗口只能给
#   --viewer rviz，或在配置里写 robot.viewer: rviz。给了它反而会走 GUI，
#   在没有 X11 的环境里直接 "Error opening X11 library libX11.so.6" 死掉。
#
# 为什么要 script(1)：**Python 的 stdout 重定向到文件时是块缓冲的**，服务端那几行
#   就绪信息会卡在缓冲区里，`grep 日志` 永远等不到（PyInstaller 的引导器还会忽略
#   PYTHONUNBUFFERED）。套一个伪终端（-f 立刻落盘、-e 把子进程退出码带出来）就
#   按行刷新了。这不是冻结带来的毛病：不冻的源码那样跑也一样。
echo "[5/5] 自检 无头冒烟（DIRECT + --ik pin + 从 LITEARM_TELEOP_MODELS 读 URDF）"
SMOKE_DIR=/tmp/frozen-smoke
rm -rf "$SMOKE_DIR"
mkdir -p "$SMOKE_DIR"
cp -a "$OUT"/. "$SMOKE_DIR"/

# 服务端自己的 pid：按 argv[0] 精确匹配。不能用 `pgrep -f 命令串` —— script(1) 的
# cmdline 里也含那串，会先撞上它，然后拿错的 pid 去看 /proc/<pid>/maps。
server_pids() {
    local p first
    for p in /proc/[0-9]*; do
        [ -r "$p/cmdline" ] || continue
        first=""
        IFS= read -r -d '' first <"$p/cmdline" 2>/dev/null || true
        [ "$first" = "$SMOKE_DIR/teleop-server" ] && printf '%s ' "${p#/proc/}"
    done
    return 0
}

# $1=名字 $2=就绪标志 $3..=数据源那一段参数
smoke() {
    local name=$1 marker=$2; shift 2
    local log="$BUILD_DIR/smoke-$name.log"
    local cmd pids pid i rc=0

    cmd="env -u LD_LIBRARY_PATH LITEARM_TELEOP_MODELS=$PROJECT/models LD_LIBRARY_PATH=$SMOKE_DIR"
    cmd="$cmd $SMOKE_DIR/teleop-server --config $PROJECT/config/teleop.yaml"
    cmd="$cmd --no-real --no-ros --viewer rviz --ik pin $*"
    script -q -f -e -c "$cmd" "$log" >/dev/null 2>"$BUILD_DIR/smoke-$name.err" &
    local spid=$!

    # appeared：区分「还没起来」和「起来又死了」—— 第一轮循环时 script(1) 才刚 fork，
    # 子进程多半还不存在，那时候不能当成失败收摊。
    local ready=0 appeared=0
    for i in $(seq 1 60); do
        if grep -q "$marker" "$log" 2>/dev/null; then ready=1; break; fi
        pids=$(server_pids)
        if [ -n "$pids" ]; then appeared=1; elif [ "$appeared" = 1 ]; then break; fi
        sleep 1
    done
    if [ "$ready" != 1 ]; then
        echo "      失败[$name]：60s 内没到「$marker」（进程出现过=$appeared）；日志："
        tail -n 30 "$log"
        tail -n 5 "$BUILD_DIR/smoke-$name.err" 2>/dev/null | sed 's/^/      /'
        kill -9 $(server_pids) 2>/dev/null || true
        kill -9 "$spid" 2>/dev/null || true
        exit 1
    fi
    grep -aE "\[配置\]|末端偏移量|遥操仿真已就绪|数据源:" "$log" | sed 's/^/      /'

    # pin IK 是否**真的**建起来了：pinocchio 只在 PinIKSolver.__init__ 里 import，
    # 所以它的 .so 出现在进程映射表里 = pin 后端确实构造过（不是回退 pybullet）。
    pids=$(server_pids)
    local hits=0
    for pid in $pids; do
        hits=$((hits + $(grep -c pinocchio "/proc/$pid/maps" 2>/dev/null || echo 0)))
    done
    echo "      [$name] pid=$pids；/proc/<pid>/maps 里 pinocchio 命中 $hits 行"
    if [ "$hits" -lt 1 ]; then
        echo "      失败[$name]：--ik pin 起了服务，但进程里没有 pinocchio —— 后端没真正用上"
        kill -9 $pids 2>/dev/null || true
        kill -9 "$spid" 2>/dev/null || true
        exit 1
    fi
    grep -m1 -E "libpinocchio_pywrap|pinocchio_pywrap|libpinocchio_default" "/proc/$(echo "$pids" | cut -d' ' -f1)/maps" \
        | sed 's/^/        /'

    kill -TERM $pids 2>/dev/null || true
    wait "$spid" || rc=$?
    echo "      [$name] SIGTERM 后退出码 $rc（143 = 被 TERM 掉，默认信号语义）"
    [ "$rc" = 143 ] || { echo "      失败[$name]：退出码不是 143"; exit 1; }
    if grep -aqE "Traceback" "$log"; then
        echo "      失败[$name]：日志里有 Traceback："
        grep -an "Traceback" "$log" | tail -n 5
        exit 1
    fi
}

# 就绪标志取「服务真的开始收帧了」那一行，不是 --viewer 之前的那句 —— ws 那行
# 只在 websockets.serve() 真的绑上端口后才打，zenoh 那行要 zenoh 会话开起来。
# ws = 验收用的那条路；zenoh = **出厂**数据源（默认值），它才是客户现场走的。
smoke ws "数据源: 自建 WS 监听" --source ws --port 18766
smoke zenoh "数据源: 中间层 zenoh 订阅" --source zenoh --zenoh-mode peer \
    --zenoh-connect tcp/127.0.0.1:7447 --port 18767

echo
echo "产物就绪：$OUT/teleop-server（整棵 $OUT 一起发）"
du -sh "$OUT"

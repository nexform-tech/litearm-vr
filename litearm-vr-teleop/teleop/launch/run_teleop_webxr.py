#!/usr/bin/env python3
"""WebSocket 手柄遥操一键启动（pybullet 逆解 + 可选 RViz2 显示）。

**只用标准库**，所以可以直接用系统 python3 跑 —— 它自己负责找到 metavr-arm 那个
解释器（见 `--metavr-py` 与下方解析顺序）。参数取代了原来的环境变量，`--help` 即总表。

配置：读 `config/teleop.yaml`（`--config` 换一份），**命令行显式给的项覆盖它**。
配置里也有 teleop_sim 那一层的选项（限幅 / home / 轴映射 / 求解器参数），那些由
服务端读同一个文件生效（`--config` 会转发给它）。

它做五件事，然后前台跑 `scripts/ws_echo_server.py`：
  1. 解析配置与 metavr-arm 解释器
  2. 备环境：source ROS2 humble（有就 source）
  3. 起显示后端（`--viewer rviz` 时：robot_state_publisher + rviz2）
  4. 清残留进程、查端口
  5. 打印操作提示，前台启动服务；退出时收拾干净（含临时 URDF/YAML）

用法:
  ./launch/run_teleop_webxr.py --help
  ./launch/run_teleop_webxr.py --dual --viewer rviz
  ./launch/run_teleop_webxr.py --ik exact --deadband-scale 0
  ./launch/run_teleop_webxr.py --config /path/to/other.yaml
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

# 本脚本在 <项目根>/launch/ 下，项目根是它上一级
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HOME = os.path.expanduser("~")

# 配置文件读取器在 src/litearm_teleop/config.py —— 它**只用标准库**，所以这里能
# 在系统 python3 下 import（本脚本刻意不依赖第三方包）。litearm_teleop 的
# __init__.py 是空的，import 它不会带出 pybullet/numpy。
sys.path.insert(0, os.path.join(PROJECT_DIR, "src"))
from litearm_teleop.config import DEFAULT_PATH as CONFIG_DEFAULT_PATH  # noqa: E402
from litearm_teleop.config import get as cfg_get  # noqa: E402
from litearm_teleop.config import load as cfg_load  # noqa: E402
from litearm_teleop.vr_source import (DEFAULT_ARM_ID as VR_DEFAULT_ARM_ID)  # noqa: E402
from litearm_teleop.vr_source import (  # noqa: E402
    DEFAULT_SOURCE as VR_DEFAULT_SOURCE, DEFAULT_ZENOH_CONNECT as VR_DEFAULT_ZENOH_CONNECT,
    DEFAULT_ZENOH_LISTEN as VR_DEFAULT_ZENOH_LISTEN,
    DEFAULT_ZENOH_MODE as VR_DEFAULT_ZENOH_MODE)
from litearm_teleop.vr_source import frame_topic as vr_frame_topic  # noqa: E402
from litearm_teleop.vr_source import parse_endpoints  # noqa: E402

# 出厂末端偏移量（米，本体系）= 本台架夹爪的法兰→抓取点距离，沿 tip link 的 +z。
# **权威定义在 `src/litearm_teleop/teleop_sim.py` 的 `DEFAULT_TCP_OFFSET`**；本脚本
# 只用标准库、不能 import 它，所以这里刻意重复一份，由 tests/test_teleop_sim.py
# 的检查 [6] 守着两边不漂移。配置文件里的 tuning.tcp_offset 优先于它。
DEFAULT_TCP_OFFSET = (0.0, 0.0, 0.0)

# 清理与端口检查针对的进程命令行子串。用 pgrep -f 匹配，故写成能在本脚本
# 源码里原样出现也不误伤自己的字面量（本脚本自身命令行不含这些串）。
STALE_PATTERNS = ("litearm_teleop.teleop_sim", "ws_echo_server",
                  "robot_state_publisher", "rviz2")
TEARDOWN_PATTERNS = ("robot_state_publisher", "rviz2", "ws_echo_server")


def _log(msg=""):
    print(msg, flush=True)


def run(cmd, **kw):
    """跑一条命令，返回 CompletedProcess。

    命令**不存在**时返回 returncode=127 而不是抛异常 —— 等价于 shell 里的
    `cmd 2>/dev/null || true`，让调用方自己判存在性（conda / ss / ip / pgrep
    都可能没装，而缺了它们不该让启动崩掉）。
    """
    try:
        return subprocess.run(cmd, capture_output=True, text=True, **kw)
    except (FileNotFoundError, NotADirectoryError):
        return subprocess.CompletedProcess(cmd, 127, "", "")


# --------------------------------------------------------------------------
# 1. metavr-arm 解释器
# --------------------------------------------------------------------------
def resolve_python(explicit):
    """按顺序解析：显式指定 → 已激活的同名 conda 环境 → 问 conda → 探常见根。

    与 `environments/setup.sh`、`Makefile` 找的是同一个环境。换机器不想 export
    就传 `--metavr-py /path/to/envs/metavr-arm/bin/python`。
    """
    if explicit:
        if not os.access(explicit, os.X_OK):
            _log("[错误] --metavr-py 指向的不是可执行文件: %s" % explicit)
            sys.exit(1)
        return explicit

    prefix = os.environ.get("CONDA_PREFIX")
    if prefix and os.path.basename(prefix) == "metavr-arm":
        candidate = os.path.join(prefix, "bin", "python")
        if os.access(candidate, os.X_OK):
            return candidate

    roots = []
    base = run(["conda", "info", "--base"]).stdout.strip()
    if base:
        roots.append(base)
    # 末项 ~/venvs 是对本机（venv 装的 metavr-arm 在 ~/venvs/metavr-arm）的补充，
    # 原来的 .sh 只探 conda 根，在本机命不中。
    roots += [os.path.join(HOME, "sl", "miniconda3"), os.path.join(HOME, "miniconda3"),
              os.path.join(HOME, "opt", "anaconda3"), os.path.join(HOME, "anaconda3"),
              os.path.join(HOME, "venvs")]
    for root in roots:
        candidate = os.path.join(root, "envs", "metavr-arm", "bin", "python")
        if os.access(candidate, os.X_OK):
            return candidate
    # venv 的布局没有 envs/ 这一层
    candidate = os.path.join(HOME, "venvs", "metavr-arm", "bin", "python")
    if os.access(candidate, os.X_OK):
        return candidate

    _log("[错误] 找不到 metavr-arm 环境的 python")
    _log("       先跑 bash environments/setup.sh，"
         "或用 --metavr-py <该环境的 python 路径> 指定")
    sys.exit(1)


# --------------------------------------------------------------------------
# 2. 环境：ROS2 与 PYTHONPATH
# --------------------------------------------------------------------------
def source_ros(env):
    """有 ROS2 humble 就把它 source 进 `env`（等价于 .sh 里的 source）。

    做法是让一个子 shell source 后倒出整个环境再合并 —— 一个进程里拿不到
    setup.bash 的全部副作用（PATH / AMENT_PREFIX_PATH / PYTHONPATH / ...）。
    """
    setup = "/opt/ros/humble/setup.bash"
    if not os.path.isfile(setup):
        _log("[警告] 未找到 ROS2 humble，RViz2 显示不可用（仍可跑 pybullet 逆解）")
        return False
    out = run(["bash", "-c",
               "set +u; source %s >/dev/null 2>&1; env -0" % setup], env=env).stdout
    for item in out.split("\0"):
        if "=" in item:
            key, _, value = item.partition("=")
            env[key] = value
    return True


def check_package(python, env):
    """确认 `python` 里 import 到的 litearm_teleop 就是**本检出**这一份。

    `main()` 在调用本函数前已把本检出的 `src` 顶到 `env["PYTHONPATH"]` 最前。
    setuptools 的 editable finder 是 append 到 `sys.meta_path` 末尾的（**优先级低于**
    `PathFinder`），所以 PYTHONPATH 里更靠前的 src 会被先命中 ⇒ 正常情况下这里必然
    通过，且**不要求先 pip install -e**，换/复制检出也不用重装。

    留作兜底：万一环境里有更高优先级的机制（自定义 finder、系统级安装）抢先解析到
    别的检出，就在起任何东西之前拦住 —— 否则会一声不响地跑**别处的代码**，比报错难查。

    期望目录是 `<项目根>/src`（服务端也只有这一份源码可跑）。
    """
    probe = ("import litearm_teleop, os; "
             "print(os.path.dirname(os.path.dirname(litearm_teleop.__file__)))")
    done = run([python, "-c", probe], env=env)
    found = done.stdout.strip()
    expected = os.path.join(PROJECT_DIR, "src")
    if done.returncode != 0 or not found:
        _log("[错误] %s 里 import 不到 litearm_teleop" % python)
        _log("       本检出 src 已置于 PYTHONPATH，仍 import 不到说明解释器本身异常：")
        _log("       %s -c 'import sys; print(sys.executable)'" % python)
        sys.exit(1)
    if os.path.realpath(found) != os.path.realpath(expected):
        _log("[错误] import 到的 litearm_teleop 不是本检出这一份")
        _log("       实际: %s" % found)
        _log("       期望: %s" % expected)
        _log("       本检出 src 已置顶 PYTHONPATH 却仍被抢先 —— 环境里有更高优先级的")
        _log("       litearm_teleop 安装。卸掉它即可：")
        _log("       %s -m pip uninstall -y litearm-teleop" % python)
        sys.exit(1)
    _log("[环境] litearm_teleop: %s" % found)


# --------------------------------------------------------------------------
# 3. 显示后端（rviz）：重写 mesh 路径 + 生成参数 YAML + 起两个进程
# --------------------------------------------------------------------------
def prepare_rviz_assets(dual, side, upright, state):
    """把 URDF 的 package:// mesh 路径换成 file:// URI，并生成 ROS2 参数 YAML。

    mesh 必须走 file:// —— RViz 的 assimp 对裸绝对路径报红。参数 YAML 是必需的：
    URDF 常含 CRLF 且是多行，没法用 -p 直接传。
    """
    if dual:
        root = os.path.join(PROJECT_DIR, "models", "dual_litearm_column_description")
        urdf_src = os.path.join(root, "urdf", "dual_litearm_upright.urdf" if upright
                                else "dual_litearm_column.urdf")
        pkg_prefix = "package://dual_litearm_column_description/meshes/"
        mesh_dir = os.path.join(root, "meshes")
        rviz_config = os.path.join(root, "rviz", "display.rviz")
    else:
        root = os.path.join(PROJECT_DIR, "models", "litearm_description")
        urdf_src = os.path.join(root, "urdf",
                                "litearm_side.urdf" if side
                                else "litearm_description.urdf")
        pkg_prefix = "package://litearm_description/meshes/"
        mesh_dir = os.path.join(root, "meshes")
        rviz_config = os.path.join(root, "rviz", "display.rviz")

    with open(urdf_src, "r", encoding="utf-8") as f:
        urdf = f.read().replace(pkg_prefix, "file://" + mesh_dir + "/")
    fd, tmp_urdf = tempfile.mkstemp(prefix="litearm_ros_", suffix=".urdf", dir="/tmp")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(urdf)
    state["tmp"].append(tmp_urdf)
    _log("[准备] URDF mesh 路径已替换: %s" % tmp_urdf)

    # 参数 YAML：先归一化换行，再用 json 转义整个 URDF 文本
    normalized = urdf.replace("\r\n", "\n").replace("\r", "\n")
    fd, tmp_yaml = tempfile.mkstemp(prefix="litearm_param_", suffix=".yaml", dir="/tmp")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write("/**:\n  ros__parameters:\n    robot_description: "
                + json.dumps(normalized) + "\n")
    state["tmp"].append(tmp_yaml)
    _log("[准备] ROS2 参数 YAML: %s" % tmp_yaml)
    return tmp_yaml, rviz_config


def start_rviz(tmp_yaml, rviz_config, state, env):
    if shutil.which("ros2", path=env.get("PATH")) is None:
        _log("[警告] 找不到 ros2 命令，rviz 模式下不会有任何显示（服务仍可跑）")
        return
    procs = state["procs"]
    procs.append(subprocess.Popen(
        ["ros2", "run", "robot_state_publisher", "robot_state_publisher",
         "--ros-args", "--params-file", tmp_yaml],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env))
    _log("[启动] robot_state_publisher PID=%d" % procs[-1].pid)
    if os.path.isfile(rviz_config):
        procs.append(subprocess.Popen(
            ["ros2", "run", "rviz2", "rviz2", "-d", rviz_config],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env))
        _log("[启动] rviz2 PID=%d" % procs[-1].pid)
    else:
        _log("[警告] 未找到 RViz 配置，跳过 rviz2")
    time.sleep(2)


# --------------------------------------------------------------------------
# 4. 进程与接口
# --------------------------------------------------------------------------
def kill_matching(patterns, keep_pid=None):
    """按命令行子串杀进程（含本次会话之前的残留），返回是否杀到了。"""
    keep = {os.getpid()}
    if keep_pid:
        keep.add(keep_pid)
    victims = []
    for pattern in patterns:
        out = run(["pgrep", "-f", pattern]).stdout.split()
        for token in out:
            pid = int(token)
            if pid not in keep:
                victims.append(pid)
    killed = False
    for pid in sorted(set(victims)):
        try:
            os.kill(pid, signal.SIGTERM)
            killed = True
        except OSError:
            pass
    return killed


def port_in_use(port):
    out = run(["ss", "-tln"]).stdout
    for line in out.splitlines()[1:]:
        fields = line.split()
        if len(fields) >= 4 and fields[3].rsplit(":", 1)[-1] == str(port):
            return True
    return False


def wifi_ip():
    """供 Quest 客户端连接提示：第一个非回环、非 docker 的全局 IPv4。"""
    out = run(["ip", "-4", "-o", "addr", "show", "scope", "global"]).stdout
    for line in out.splitlines():
        fields = line.split()
        if "inet" not in fields:
            continue
        address = fields[fields.index("inet") + 1].split("/")[0]
        if address.startswith("127.") or address.startswith("172.17."):
            continue
        return address
    return ""


# --------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------
# CLI 选项 → (配置文件路径, 出厂常量)。出厂常量只在配置文件里连键都没有时才用；
# 它是这份表的第二真源，由 tests/test_config.py 守着与代码常量一致。
OPTION_PATHS = {
    "metavr_py": ("env.metavr_py", None),
    "source": ("vr.source", VR_DEFAULT_SOURCE),
    "arm_id": ("vr.arm_id", VR_DEFAULT_ARM_ID),
    "zenoh_mode": ("vr.zenoh_mode", VR_DEFAULT_ZENOH_MODE),
    "zenoh_connect": ("vr.zenoh_connect", VR_DEFAULT_ZENOH_CONNECT),
    "zenoh_listen": ("vr.zenoh_listen", VR_DEFAULT_ZENOH_LISTEN),
    "host": ("server.host", "0.0.0.0"),
    "port": ("server.port", 8765),
    "dual": ("robot.dual", False),
    "upright": ("robot.upright", False),
    "side": ("robot.side", False),
    "viewer": ("robot.viewer", "gui"),
    "probe": ("robot.probe", None),
    "ik": ("tuning.ik", "pybullet"),
    "tcp_offset": ("tuning.tcp_offset", list(DEFAULT_TCP_OFFSET)),
    "deadband_scale": ("tuning.deadband_scale", 1.0),
    "smooth_alpha": ("tuning.smooth_alpha", 0.5),
    "resume_margin": ("tuning.resume_margin", 1.0),
    "task_residual_tolerance": ("tuning.task_residual_tolerance", 1e-3),
    "manip_weight": ("tuning.manip_weight", 0.0),
    "control_frame": ("tuning.control_frame", "tool"),
    "base_link_signs": ("tuning.base_link_signs", None),
    "real": ("real.enabled", False),
    "real_left": ("real.left_addr", "127.0.0.1:9001"),
    "real_right": ("real.right_addr", "127.0.0.1:9002"),
    "hand": ("effector.hand", False),
    "hand_remote": ("effector.hand_remote", None),
    "gripper": ("effector.gripper", False),
    "gripper_remote": ("effector.gripper_remote", None),
}


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="LiteArm VR 遥操一键启动（pybullet 逆解 + 可选 RViz2 显示）",
        epilog="各项的默认值写在 config/teleop.yaml 里（含单位与取值范围）；"
               "命令行只用来临时覆盖。配置文件解析器见 src/litearm_teleop/config.py。")
    ap.add_argument("--config", default=None, metavar="PATH",
                    help="配置文件（默认 config/teleop.yaml）；命令行显式给的项更优先")

    env_group = ap.add_argument_group("环境")
    env_group.add_argument("--metavr-py", metavar="PATH",
                           help="metavr-arm 环境的 python 路径（不传则按配置/自动解析）")

    svc = ap.add_argument_group("服务与数据源")
    svc.add_argument("--source", choices=["zenoh", "ws"],
                     help="VR 数据源：zenoh=订阅中间层帧流（默认，帧由 "
                          "litearm-vr-bridge 从 8765 转发到 zenoh）/ "
                          "ws=自建 WS 监听（头显直连 --host:--port）")
    svc.add_argument("--arm-id", metavar="ARM",
                     help="zenoh 话题里的 arm 段（默认 armA）；要与桥的 ZENOH_ARM_ID 一致")
    svc.add_argument("--zenoh-mode", choices=["client", "peer", "router"],
                     help="zenoh 会话模式（默认 peer：连不上只后台重试，桥后起也能接上；client 要求开会话时就已连上）")
    svc.add_argument("--zenoh-connect", metavar="EP[,EP…]",
                     help="zenoh 主动连的端点（默认 tcp/127.0.0.1:7447 = 同机的桥）")
    svc.add_argument("--zenoh-listen", metavar="EP[,EP…]",
                     help="zenoh 监听端点（默认空；同机的桥已占 7447）")
    svc.add_argument("--host", help="WS 监听地址（--source ws 时用）")
    svc.add_argument("--port", type=int, help="WS 监听端口（--source ws 时用）")

    shape = ap.add_argument_group("构型与显示")
    shape.add_argument("--dual", action=argparse.BooleanOptionalAction,
                       help="双臂（默认单臂正装）；配 --upright 为双臂正装，"
                            "否则双臂侧装立柱")
    shape.add_argument("--upright", action=argparse.BooleanOptionalAction,
                       help="与 --dual 同用：双臂正装（两只独立竖直臂、无立柱）；"
                            "不传则双臂侧装立柱")
    shape.add_argument("--side", action=argparse.BooleanOptionalAction,
                       help="单臂侧装 = 单左臂（左手柄驱动、home/映射取双臂侧装左臂，"
                            "真机走 :9001；--dual 时忽略）")
    shape.add_argument("--viewer", choices=["gui", "rviz"],
                       help="gui=pybullet 窗口 / rviz=RViz2（需 ROS2）")
    shape.add_argument("--probe", metavar="JSONL",
                       help="数据探针：把每帧各阶段的量写成 JSONL（诊断关节跳变用，"
                            "见 scripts/probe.py；正常遥操别开）")

    tune = ap.add_argument_group("求解与手感")
    tune.add_argument("--ik", choices=["pin", "pybullet", "exact"],
                      help="pybullet=calculateInverseKinematics（出厂）/ "
                           "pin=pinocchio+pink（metavr 策略）/ "
                           "exact=精确任务速度（无阻尼，增益 1 或 0）")
    tune.add_argument("--tcp-offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                      help="末端偏移量(米，本体系)：控制的工具点相对 tip link 的偏移"
                           "（沿工具轴 +z）；传 0 0 0 = 直接控法兰")
    tune.add_argument("--deadband-scale", type=float, metavar="S",
                      help="静止死区阈值乘数；0=关掉死区（会放行追踪噪声）。"
                           "出厂值在 60Hz 下等于平移 1.8cm/s、旋转 0.18rad/s")
    tune.add_argument("--smooth-alpha", type=float, metavar="A",
                      help="位姿低通在60Hz下的新样本权重，取值 (0, 1]；1.0=不平滑")
    tune.add_argument("--resume-margin", type=float, metavar="M",
                      help="--ik exact 的持位滞回，1.0=朴素规则")
    tune.add_argument("--task-residual-tolerance", type=float, metavar="T",
                      help="--ik exact 判定「任务被产生」的残差比例上限")
    tune.add_argument("--manip-weight", type=float, metavar="W",
                      help="--ik pin 的操纵度任务权重（把冗余推向雅可比更良态的构型，"
                           "预防奇异）；0=关闭（出厂）。调大会牺牲工具精度")
    tune.add_argument("--control-frame", choices=["tool", "base_link"],
                      help="旋转增量读在哪个系：tool=末端局部系（出厂，现状）/ "
                           "base_link=基座系（绕世界轴转，与平移同源）")
    tune.add_argument("--base-link-signs", type=float, nargs=3, metavar=("X", "Y", "Z"),
                      help="--control-frame base_link 用的逐轴旋转符号（未重测时勿乱改）")

    real = ap.add_argument_group("真机")
    real.add_argument("--real", action=argparse.BooleanOptionalAction, help="启用真机桥接")
    real.add_argument("--real-left", help="左臂 arm_follow_server 地址")
    real.add_argument("--real-right", help="右臂 arm_follow_server 地址")

    end = ap.add_argument_group("灵巧手与夹爪")
    end.add_argument("--hand", action=argparse.BooleanOptionalAction,
                     help="灵巧手（trigger 边沿 → 握拳/张开，经 --hand-remote 走 TCP）")
    end.add_argument("--hand-remote", metavar="HOST:PORT",
                     help="灵巧手服务 hand_server.py 地址（--hand 时必填）")
    end.add_argument("--gripper", action=argparse.BooleanOptionalAction,
                     help="夹爪（trigger 边沿 → 闭合/张开，经 --gripper-remote 走 TCP）")
    end.add_argument("--gripper-remote", metavar="HOST:PORT",
                     help="夹爪服务 gripper_server.py 地址（--gripper 时必填）")
    args = ap.parse_args(argv)

    args.config = args.config or CONFIG_DEFAULT_PATH
    cfg = load_config(args.config)
    resolve_options(args, cfg)
    return args, cfg


def load_config(path):
    """读配置文件。文件缺失/写坏都**不静默**，但也不至于让启动崩掉。"""
    if not os.path.isfile(path):
        if path != CONFIG_DEFAULT_PATH:
            _log("[错误] 配置文件不存在: %s" % path)
            sys.exit(1)
        _log("[警告] 未找到出厂配置 %s，本次全部用出厂常量" % CONFIG_DEFAULT_PATH)
        return {}
    try:
        return cfg_load(path)
    except ValueError as error:
        _log("[错误] 配置文件解析失败: %s" % path)
        _log("       %s" % error)
        sys.exit(1)


def resolve_options(args, cfg):
    """把配置文件里的值填进**命令行没显式给过**的项。

    优先级：命令行 > 配置文件 > 出厂常量。argparse 里所有选项的 default 都是
    `None`（没有 `default=`，因为本脚本要为「缺省」留出与「显式给 False/0」不同
    的含义），所以「属性为 None」就等于「没给过」。
    """
    for dest, (path, factory) in OPTION_PATHS.items():
        if getattr(args, dest, None) is None:
            setattr(args, dest, cfg_get(cfg, path, factory))
    return args


def _endpoint_note(name, value):
    """`，connect=a,b` 这样的一小段提示；没有端点就是空串。"""
    endpoints = parse_endpoints(value)
    return "" if not endpoints else "，%s=%s" % (name, ",".join(endpoints))


def build_server_argv(args, hand_ready, gripper_ready):
    """组装服务端命令行。

    `--config` 一定转发 —— 服务端要用它读**本脚本不认识**的那一层选项
    （sim / mapping / pin_ik / exact_ik / controller）。本脚本认识的那些也照旧显式
    转发一份（值已按「命令行 > 配置文件」解析过），这样服务端的优先级规则与这里一致，
    不用去猜哪一边赢。

    ⚠ 灵巧手/夹爪/真机的**开关**必须显式传 `--hand/--no-hand` 等：本脚本已经按配置
    校验过「开关必须配地址」，若只传「开」而不传「关」，服务端会自己去读配置文件里那个
    `effector.hand: true` —— 于是同一个开关有了两处判断，迟早会不一致。
    `hand_ready` 是本脚本校验通过（开关与地址成对）后的结论。
    """
    argv = ["--config", args.config,
            "--source", args.source,
            "--arm-id", args.arm_id,
            "--zenoh-mode", args.zenoh_mode,
            "--host", args.host, "--port", str(args.port), "--viewer", args.viewer]
    # 端点按逗号拼成一个参数（服务端也按逗号切，两边同一套写法）；空的就不传 ——
    # 传空串会与「配置文件里没写」混在一起。
    for flag, value in (("--zenoh-connect", args.zenoh_connect),
                        ("--zenoh-listen", args.zenoh_listen)):
        endpoints = parse_endpoints(value)
        if endpoints:
            argv += [flag, ",".join(endpoints)]
    if args.dual:
        argv.append("--dual")
    if args.upright:
        argv.append("--upright")
    if args.side:
        argv.append("--side")
    argv += ["--ik", args.ik,
             "--deadband-scale", str(args.deadband_scale),
             "--smooth-alpha", str(args.smooth_alpha),
             "--resume-margin", str(args.resume_margin),
             "--task-residual-tolerance", str(args.task_residual_tolerance)]
    if args.manip_weight:
        argv += ["--manip-weight", str(args.manip_weight)]
    if args.control_frame != "tool":
        argv += ["--control-frame", args.control_frame]
    if args.base_link_signs is not None:
        argv += ["--base-link-signs"] + [str(v) for v in args.base_link_signs]
    if args.tcp_offset is not None:
        argv += ["--tcp-offset"] + [str(v) for v in args.tcp_offset]
    if args.probe:
        argv += ["--probe", args.probe]
    if args.real:
        argv += ["--real", "--real-left", args.real_left,
                 "--real-right", args.real_right]
    else:
        argv.append("--no-real")
    argv += (["--hand", "--hand-remote", args.hand_remote] if hand_ready
             else ["--no-hand"])
    argv += (["--gripper", "--gripper-remote", args.gripper_remote] if gripper_ready
             else ["--no-gripper"])
    return argv


def main(argv=None):
    args, cfg = parse_args(argv)
    state = {"procs": [], "tmp": []}

    # SIGTERM 默认会直接终止进程，`finally` 就不跑了 ⇒ 临时文件残留、子进程成孤儿。
    # 转成异常，走与 Ctrl-C 同一条清理路径。
    def _on_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    python = resolve_python(args.metavr_py)
    env = dict(os.environ)
    _log("[配置] %s" % args.config)
    _log("[环境] 解释器: %s" % python)

    source_ros(env)
    # 把**本检出**的 src 顶到 PYTHONPATH 最前，让 env 里任何 editable 安装都让位。
    # setuptools 的 editable finder 是 append 到 sys.meta_path 末尾的（优先级低于
    # PathFinder），所以 PYTHONPATH 里更靠前的 src 会被先找到 ⇒ 任意检出都能跑自己
    # 这份代码，无需先 pip install -e，也不会串到隔壁检出。
    src_dir = os.path.join(PROJECT_DIR, "src")
    env["PYTHONPATH"] = src_dir + (
        os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    check_package(python, env)

    mode = ("双臂正装" if (args.dual and args.upright)
            else ("双臂侧装" if args.dual
                  else ("单臂侧装（左臂）" if args.side else "单臂正装")))
    if args.dual and args.side:
        _log("[警告] 同时给了 --dual 与 --side，按双臂处理（--side 仅单臂时生效）")
    if args.upright and not args.dual:
        _log("[警告] --upright 仅在 --dual 时生效，已忽略（仍按单臂处理）")

    ip_address = wifi_ip()
    if not ip_address:
        _log("[警告] 未检测到局域网 IP，请确认已连接 Wi-Fi")

    # ---------- 清残留 + 端口检查 + 全程 try ----------
    def cleanup():
        # 先精确终止本次起的进程，再按模式清扫（与 .sh 的 pkill 兜底一致）
        for proc in state["procs"]:
            if proc.poll() is None:
                proc.terminate()
        for proc in state["procs"]:
            try:
                proc.wait(timeout=3)
            except Exception:
                proc.kill()
        kill_matching(TEARDOWN_PATTERNS, keep_pid=os.getpid())
        for path in state["tmp"]:
            try:
                os.remove(path)
            except OSError:
                pass
        _log("[已停止] 遥操服务已退出")

    try:
        # 逐条清残留，每条给它自己的回收时间（端口要等它真正释放）
        for pattern, pause in (("litearm_teleop.teleop_sim", 2.0),
                               ("ws_echo_server", 1.0),
                               ("robot_state_publisher", 1.0),
                               ("rviz2", 1.0)):
            if kill_matching([pattern], keep_pid=os.getpid()):
                time.sleep(pause)
        if args.source == "ws" and port_in_use(args.port):
            _log("[错误] 端口 %d 仍被占用，请手动排查: ss -tlnp | grep :%d"
                 % (args.port, args.port))
            sys.exit(1)

        # ---------- 显示后端 ----------
        if args.viewer == "rviz":
            tmp_yaml, rviz_config = prepare_rviz_assets(args.dual, args.side,
                                                        args.upright, state)
            start_rviz(tmp_yaml, rviz_config, state, env)

        # ---------- 灵巧手 ----------
        # 实现已移出本仓：本机只当 TCP 客户端，CAN/SDK 在电机那台工控机上，所以这里
        # 没有 CAN 要准备，只有「开关 + 地址」要校验成对 —— 只开开关不给地址的话，
        # 服务端会静默什么都不做，操作者会把「按了没反应」当成硬件问题。
        hand_ready = False
        if args.hand:
            if not args.hand_remote:
                _log("[错误] --hand 需要 --hand-remote HOST:PORT"
                     "（灵巧手服务 hand_server.py 跑在电机所在的工控机上）")
                sys.exit(1)
            hand_ready = True

        # ---------- 夹爪 ----------
        gripper_ready = False
        if args.gripper:
            if not args.gripper_remote:
                _log("[错误] --gripper 需要 --gripper-remote HOST:PORT"
                     "（夹爪服务 gripper_server.py 跑在电机所在的工控机上）")
                sys.exit(1)
            gripper_ready = True

        # ---------- 启动前提示 ----------
        _log()
        _log("[启动] 遥操服务 数据源=%s" % args.source)
        if args.source == "zenoh":
            _log("  zenoh 帧流: %s（arm_id=%s）" % (vr_frame_topic(args.arm_id), args.arm_id))
            _log("  zenoh 会话: mode=%s%s%s"
                 % (args.zenoh_mode, _endpoint_note("connect", args.zenoh_connect),
                    _endpoint_note("listen", args.zenoh_listen)))
            _log("  ⚠ 需先起 litearm-vr-bridge（它收头显的 8765，再转发到这条话题）")
        else:
            _log("  WS 监听: ws://%s:%d（头显直连本机）" % (args.host, args.port))
        _log("  模式: %s" % mode)
        _log("  IK: %s" % {"pin": "pinocchio + pink 微分 IK（抗奇异/防关节跳变）",
                           "pybullet": "pybullet calculateInverseKinematics（回退）",
                           "exact": "精确任务速度（无阻尼，增益恒为 1 或 0）"}[args.ik])
        _log("  显示: %s" % ("RViz2（RobotModel + TF，/joint_states）"
                            if args.viewer == "rviz"
                            else "pybullet GUI 窗口（鼠标拖拽旋转/滚轮缩放）"))
        if args.tcp_offset is not None:
            _log("  末端偏移: %s m（本体系：控制该工具点而非法兰）"
                 % " ".join(str(v) for v in args.tcp_offset))
        if args.deadband_scale != 1.0:
            _log("  静止死区: 阈值 ×%.2f%s"
                 % (args.deadband_scale,
                    "（已关闭 —— 手柄追踪噪声会原样放行）" if args.deadband_scale == 0.0
                    else ""))
        if args.real:
            _log("  真机桥接: 已启用（左臂 %s，右臂 %s）" % (args.real_left, args.real_right))
        else:
            _log("  真机桥接: 关闭（设 --real 启用）")
        if hand_ready:
            _log("  灵巧手: 已启用（远端 TCP=%s，trigger 按下=握拳，松开=张开）"
                 % args.hand_remote)
        else:
            _log("  灵巧手: 关闭（设 --hand 启用）")
        if gripper_ready:
            _log("  夹爪: 已启用（远端 TCP=%s，trigger 按下=闭合，松开=张开）"
                 % args.gripper_remote)
        else:
            _log("  夹爪: 关闭（设 --gripper 启用）")
        _log("  Quest 客户端连接: ws://%s:%d" % (ip_address or "<本机IP>", args.port))
        _log("  遥操语义: gripActive 按下=启动，松开=暂停；A 键=平滑回零；trigger 按下=握拳，松开=张开")
        _log()

        # ---------- 前台跑服务 ----------
        # 用 Popen 而不是 call：要把服务本身也纳入 state["procs"]，否则收到
        # SIGTERM/SIGINT 时清理够不着它（它和本脚本同进程组，但信号只发给本脚本）。
        server = "%s/scripts/ws_echo_server.py" % PROJECT_DIR
        argv = [python, server] + build_server_argv(args, hand_ready, gripper_ready)
        server_proc = subprocess.Popen(argv, cwd=PROJECT_DIR, env=env)
        state["procs"].append(server_proc)
        code = server_proc.wait()

        _log()
        _log("==============================================")
        if args.source == "zenoh":
            _log("  数据源: 中间层 zenoh %s" % vr_frame_topic(args.arm_id))
            _log("  头显连 ws://%s:%d（litearm-vr-bridge 收帧后转发到上面这条话题）"
                 % (ip_address or "<本机IP>", 8765))
        else:
            _log("  Quest 端操作:")
            _log("  1. 客户端连接 ws://%s:%d" % (ip_address or "<本机IP>", args.port))
        _log("  手柄 gripActive 按下=启动  松开=暂停；A 键=平滑回零；trigger 按下=握拳  松开=张开")
        _log("==============================================")
        return code
    finally:
        cleanup()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)

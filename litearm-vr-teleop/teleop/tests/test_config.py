#!/usr/bin/env python3
"""配置文件（`config/teleop.yaml`）与其读取器的单测。只依赖 numpy，秒级。

守三件事：
  1. **配置文件里的出厂值 == 代码常量。** 这是本套配置最容易出事的地方：配置文件
     写着一份默认值、代码里还有一份常量，两边一漂就是「文档说的和跑的不一样」，
     而且不会报错。所以逐个比，比到矩阵/向量元素。
  2. 解析器本身（缩进、流式序列、注释、引号、null、带冒号的值、错误必须报）。
  3. 优先级：**命令行 > 配置文件 > 出厂常量**，且「命令行没给过」与「显式给了
     出厂值」必须区分得开（否则命令行的默认值会盖掉配置文件）。

用法: python tests/test_config.py
"""
import importlib.util
import os
import sys

import numpy as np

from litearm_teleop.config import get, load, parse, set_path
from litearm_teleop.pin_ik import (PINK_DT, PINK_ERR_TOL, PINK_LM_DAMPING,
                                   PINK_MANIP_RATE, PINK_MAX_ITERS,
                                   PINK_POSTURE_COST)
from litearm_teleop.teleop_sim import (BASE_LINK_ROTATION_SIGNS, CONTROL_FRAME,
                                       CTRL_AXIS_REMAP, CTRL_TRANSLATION_REMAP,
                                       DEFAULT_TCP_OFFSET, DELTA_DEADBAND_R_IN,
                                       DELTA_DEADBAND_R_OUT, DELTA_DEADBAND_T_IN,
                                       DELTA_DEADBAND_T_OUT,
                                       DUAL_TRANSLATION_REMAP_LEFT,
                                       DUAL_TRANSLATION_REMAP_RIGHT, FREQ,
                                       IK_POS_TOL, IK_ROT_TOL, MAX_JOINT_VEL,
                                       MAX_STEP_RAD, MIRROR_SIGNS,
                                       NEAR_SINGULAR_SIGMA,
                                       Q_HOME, Q_LOWPASS_TAU, REST_CONFIRM_FRAMES,
                                       ROTATION_AXIS_SIGNS, SIDE_CONFIG,
                                       SINGLE_Q_HOME, SMOOTH_ALPHA, SimConfig,
                                       Tuning, UPRIGHT_CONFIG,
                                       sim_config_from_config, tuning_from_config)
from litearm_teleop.vr_source import (DEFAULT_ARM_ID, DEFAULT_SOURCE,
                                      DEFAULT_ZENOH_CONNECT, DEFAULT_ZENOH_LISTEN,
                                      DEFAULT_ZENOH_MODE, frame_topic,
                                      parse_endpoints)

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _launcher():
    """把启动脚本当模块执行（它只用标准库，import 它不会带出 pybullet）。"""
    path = os.path.join(PROJECT_DIR, "launch", "run_teleop_webxr.py")
    spec = importlib.util.spec_from_file_location("_launcher_cfg", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parser():
    """解析器：本项目实际用到的写法都要对，用不到的写法要**报错**而不是读错。"""
    data = parse("""
# 整行注释
top: 1                  # 行尾注释
nested:
  a: 2.5
  deeper:
    b: true
  quoted: "0x08"        # 十六进制靠引号，避免被当数字读错
flow: [1, 2, 3]
matrix: [[1, 0, 0], [0, 1, 0], [0, 0, -1]]
uri: 127.0.0.1:9001
blank:
tilde: ~
text: gui
""")
    assert data["top"] == 1
    assert data["nested"]["a"] == 2.5 and data["nested"]["deeper"]["b"] is True
    assert data["nested"]["quoted"] == "0x08"
    assert data["flow"] == [1, 2, 3]
    assert data["matrix"] == [[1, 0, 0], [0, 1, 0], [0, 0, -1]]
    assert data["uri"] == "127.0.0.1:9001"      # 只切第一个冒号
    assert data["blank"] is None and data["tilde"] is None
    assert data["text"] == "gui"
    print("[1] 解析器：嵌套 / 流式序列 / 注释 / 引号 / null / 带冒号的值")

    # get/set_path 的语义：缺失 → default，显式 null 也 → default
    assert get(data, "nested.a") == 2.5
    assert get(data, "nested.nope", "D") == "D"
    assert get(data, "tilde", "D") == "D"
    set_path(data, "x.y.z", 7)
    assert get(data, "x.y.z") == 7
    print("[2] get/set_path：点路径读写，缺失与显式 null 都退回 default")

    for bad, why in (("a: 1\n  b: 2\n", "缩进突进"),
                     ("- item\n", "块序列"),
                     ("a: [1, 2\n", "序列没闭合"),
                     ("  a: 1\n", "顶层缩进"),
                     ("a\n", "没有冒号")):
        try:
            parse(bad)
            raise AssertionError("应拒绝：%s" % why)
        except ValueError:
            pass
    print("[3] 解析器：不支持的写法一律报错，不静默读错")


def test_yaml_matches_constants():
    """配置文件里的出厂值必须与代码常量逐位相同。"""
    cfg = load()
    sim = sim_config_from_config(cfg)
    tuning = tuning_from_config(cfg)

    for name, value, const in (("sim.freq", sim.freq, FREQ),
                               ("exact_ik.max_joint_vel", sim.max_joint_vel, MAX_JOINT_VEL),
                               ("sim.max_step_rad", sim.max_step_rad, MAX_STEP_RAD),
                               ("sim.q_lowpass_tau", sim.q_lowpass_tau, Q_LOWPASS_TAU),
                               ("sim.ik_pos_tol", sim.ik_pos_tol, IK_POS_TOL),
                               ("sim.ik_rot_tol", sim.ik_rot_tol, IK_ROT_TOL),
                               ("sim.near_singular_sigma", sim.near_singular_sigma,
                                NEAR_SINGULAR_SIGMA),
                               ("tuning.rest_confirm_frames", tuning.rest_confirm_frames,
                                REST_CONFIRM_FRAMES),
                               ("tuning.smooth_alpha", tuning.smooth_alpha,
                                SMOOTH_ALPHA),
                               # pin 策略：数值必须与 pin_ik.py 的 metavr 常量一致
                               ("pin_ik.lm_damping", sim.pin_lm_damping,
                                PINK_LM_DAMPING),
                               ("pin_ik.posture_cost", sim.pin_posture_cost,
                                PINK_POSTURE_COST),
                               ("pin_ik.manip_rate", sim.pin_manip_rate,
                                PINK_MANIP_RATE),
                               ("pin_ik.dt", sim.pin_dt, PINK_DT),
                               ("pin_ik.max_iters", sim.pin_max_iters,
                                PINK_MAX_ITERS),
                               ("pin_ik.err_tol", sim.pin_err_tol, PINK_ERR_TOL),
                               ("tuning.control_frame", tuning.control_frame,
                                CONTROL_FRAME),
                               ("tuning.deadband.t_in", tuning.deadband_t_in,
                                DELTA_DEADBAND_T_IN),
                               ("tuning.deadband.t_out", tuning.deadband_t_out,
                                DELTA_DEADBAND_T_OUT),
                               ("tuning.deadband.r_in", tuning.deadband_r_in,
                                DELTA_DEADBAND_R_IN),
                               ("tuning.deadband.r_out", tuning.deadband_r_out,
                                DELTA_DEADBAND_R_OUT)):
        assert value == const, "%s 与代码常量漂移了: %r != %r" % (name, value, const)

    for name, value, const in (("sim.q_home", sim.q_home, Q_HOME),
                               ("sim.single_q_home", sim.single_q_home, SINGLE_Q_HOME),
                               ("sim.mirror_signs", sim.mirror_signs, MIRROR_SIGNS),
                               ("mapping.axis_remap", sim.axis_remap, CTRL_AXIS_REMAP),
                               ("mapping.translation_remap", sim.translation_remap,
                                CTRL_TRANSLATION_REMAP),
                               ("mapping.rotation_signs", sim.rotation_signs,
                                ROTATION_AXIS_SIGNS),
                               ("mapping.base_link_rotation_signs",
                                sim.base_link_rotation_signs, BASE_LINK_ROTATION_SIGNS),
                               ("mapping.dual_translation_remap_left",
                                sim.dual_translation_remap_left,
                                DUAL_TRANSLATION_REMAP_LEFT),
                               ("mapping.dual_translation_remap_right",
                                sim.dual_translation_remap_right,
                                DUAL_TRANSLATION_REMAP_RIGHT),
                               ("mapping.dual_rotation_signs_left",
                                sim.dual_rotation_signs_left,
                                SIDE_CONFIG["left"]["rotation_signs"]),
                               ("mapping.dual_rotation_signs_right",
                                sim.dual_rotation_signs_right,
                                SIDE_CONFIG["right"]["rotation_signs"]),
                               ("mapping.upright_translation_remap_left",
                                sim.upright_translation_remap_left,
                                UPRIGHT_CONFIG["left"]["translation_remap"]),
                               ("mapping.upright_translation_remap_right",
                                sim.upright_translation_remap_right,
                                UPRIGHT_CONFIG["right"]["translation_remap"]),
                               ("mapping.upright_rotation_signs_left",
                                sim.upright_rotation_signs_left,
                                UPRIGHT_CONFIG["left"]["rotation_signs"]),
                               ("mapping.upright_rotation_signs_right",
                                sim.upright_rotation_signs_right,
                                UPRIGHT_CONFIG["right"]["rotation_signs"])):
        assert np.array_equal(value, const), \
            "%s 与代码常量漂移了:\n%r\n%r" % (name, value, const)

    # `SimConfig()` / `Tuning()` 的出厂默认也必须还是原值（配置层不许改动出厂行为）
    default_sim = SimConfig()
    assert default_sim.freq == FREQ and default_sim.max_joint_vel == MAX_JOINT_VEL
    assert np.array_equal(default_sim.q_home, Q_HOME)
    assert np.array_equal(default_sim.dual_rotation_signs_right,
                          SIDE_CONFIG["right"]["rotation_signs"])
    assert Tuning().deadband_t_in == DELTA_DEADBAND_T_IN
    assert Tuning().rest_confirm_frames == REST_CONFIRM_FRAMES
    print("[4] 配置文件出厂值 == 代码常量；SimConfig()/Tuning() 默认也未被改动")

    # 启动脚本自己那份 tcp_offset 副本（它只用标准库，不能 import teleop_sim）
    launcher = _launcher()
    assert tuple(launcher.DEFAULT_TCP_OFFSET) == tuple(DEFAULT_TCP_OFFSET)
    assert tuple(get(cfg, "tuning.tcp_offset", None)) == tuple(DEFAULT_TCP_OFFSET), \
        "配置文件里的 tcp_offset 与出厂值漂移了"
    print("[5] tcp_offset：启动脚本副本与配置文件都 == 出厂值 %s"
          % " ".join(str(v) for v in DEFAULT_TCP_OFFSET))
    return launcher


def test_precedence(launcher):
    """命令行 > 配置文件 > 出厂常量；且「没给过」要与「显式给了值」区分开。"""
    args, cfg = launcher.parse_args([])
    assert args.port == get(cfg, "server.port") and args.ik == get(cfg, "tuning.ik")
    assert args.upright == get(cfg, "robot.upright")
    print("[6] 不给命令行 → 全部取自 config/teleop.yaml（port=%r ik=%r）"
          % (args.port, args.ik))

    args, _ = launcher.parse_args(["--port", "9000", "--ik", "exact", "--dual"])
    assert args.port == 9000 and args.ik == "exact" and args.dual is True
    print("[7] 命令行显式项覆盖配置文件（port=9000 ik=exact dual=True）")

    # 改一份配置文件的临时副本，验证「配置文件能压过出厂常量」
    import tempfile
    text = open(os.path.join(PROJECT_DIR, "config", "teleop.yaml"),
                encoding="utf-8").read()
    text = text.replace("port: 8765", "port: 9123").replace("dual: false",
                                                            "dual: true")
    fd, path = tempfile.mkstemp(suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(text)
    try:
        args, _ = launcher.parse_args(["--config", path])
        assert args.port == 9123 and args.dual is True, (args.port, args.dual)
        # 命令行仍然压得过它，且 --no-dual 能把配置文件里的 true 关掉
        args, _ = launcher.parse_args(["--config", path, "--port", "9000",
                                       "--no-dual"])
        assert args.port == 9000 and args.dual is False
    finally:
        os.remove(path)
    print("[8] 配置文件 > 出厂常量；命令行 > 配置文件；--no-X 能关掉配置里的 true")


def test_option_paths_exist():
    """启动脚本/服务端的「选项 → 配置路径」表必须都指向配置文件里真实存在的键。

    写错一个路径不会报错，只会让那个选项**永远读不到配置**（静默失效）。
    ⚠ 这里查的是**键在不在**，不是「值非空」：`probe: ~` 这类「出厂就是关」的键
    值本来就是 null，用 `get` 带哨兵值去判会把它们误判成不存在。
    """
    def has_path(data, path):
        node = data
        for key in path.split("."):
            if not isinstance(node, dict) or key not in node:
                return False
            node = node[key]
        return True

    cfg = load()
    launcher = _launcher()
    missing = [path for path, _factory in launcher.OPTION_PATHS.values()
               if not has_path(cfg, path)]
    assert not missing, "启动脚本引用了配置文件里不存在的键: %r" % missing

    # 服务端的表在 scripts/ws_echo_server.py 里，用 ast 读出来（它 import 时就要
    # pybullet/websockets，纯文本解析更省事且不做多余副作用）。
    import ast
    server = os.path.join(PROJECT_DIR, "scripts", "ws_echo_server.py")
    with open(server, encoding="utf-8") as f:
        tree = ast.parse(f.read())
    table = None
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") \
                == "_OPTION_PATHS":
            table = ast.literal_eval(node.value)
    assert table, "在 ws_echo_server.py 里没找到 _OPTION_PATHS"
    missing = [path for path in table.values() if not has_path(cfg, path)]
    assert not missing, "服务端引用了配置文件里不存在的键: %r" % missing
    print("[9] 两张「选项 → 配置路径」表(%d + %d 项)都指向真实存在的键"
          % (len(launcher.OPTION_PATHS), len(table)))


def test_launcher_argv_accepted_by_server(launcher):
    """启动脚本拼出来的 argv 必须能被服务端的参数解析器接住。

    两边是两套 argparse，靠 `build_server_argv` 手写拼字符串连着。写错一个选项名
    （或加了 `--no-X` 而对面没定义）在真机上表现为「启动脚本自己跑得好好的，服务端
    随后 exit 2 退出，只剩一堆已起的 rviz 进程」—— 所以在这里钉住。
    """
    import importlib.util as ilu
    path = os.path.join(PROJECT_DIR, "scripts", "ws_echo_server.py")
    spec = ilu.spec_from_file_location("_server_cfg", path)
    server = ilu.module_from_spec(spec)
    spec.loader.exec_module(server)

    # 覆盖到每一种分支：三种构型、base_link 坐标系带符号、真机与两个远端执行器
    cases = [
        ([], False, False),
        (["--dual", "--viewer", "rviz", "--ik", "exact"], True, True),
        (["--dual", "--upright"], False, False),
        (["--real", "--hand", "--hand-remote", "10.0.0.5:9004",
          "--gripper", "--gripper-remote", "10.0.0.5:9003",
          "--control-frame", "base_link", "--base-link-signs", "1", "-1", "1",
          "--probe", "/tmp/x.jsonl"], True, True),
        (["--gripper", "--gripper-remote", "10.0.0.5:9003", "--no-hand"], False, True),
        # 数据源走 WS 回退，且 zenoh 那一组显式给成别的值（转发要原样到达）
        (["--source", "ws", "--arm-id", "armB", "--zenoh-mode", "peer",
          "--zenoh-connect", "tcp/10.0.0.9:7447,tcp/10.0.0.9:7448",
          "--zenoh-listen", "tcp/0.0.0.0:7448"], False, False),
    ]
    for argv, hand_ready, gripper_ready in cases:
        args, _cfg = launcher.parse_args(argv)
        forwarded = launcher.build_server_argv(args, hand_ready, gripper_ready)
        parsed = server._build_parser().parse_args(forwarded)
        # 命令行显式给了的项必须原样到达服务端（否则优先级会静默反转）
        if args.port != 8765:
            assert parsed.port == args.port
        assert parsed.host == args.host
        assert parsed.viewer == args.viewer
        # 数据源那一组同理：source/arm_id/mode 总是显式转发，端点是空就不传
        assert parsed.source == args.source
        assert parsed.arm_id == args.arm_id
        assert parsed.zenoh_mode == args.zenoh_mode
        assert parse_endpoints(getattr(parsed, "zenoh_connect", None)) \
            == parse_endpoints(args.zenoh_connect)
        assert parse_endpoints(getattr(parsed, "zenoh_listen", None)) \
            == parse_endpoints(args.zenoh_listen)
        assert bool(getattr(parsed, "dual", False)) == args.dual
        assert bool(getattr(parsed, "upright", False)) == args.upright
        assert parsed.ik == args.ik
        assert parsed.config == args.config
        # 开关是**显式**传的：服务端不该有「配置说开、其实没准备好」的机会
        assert bool(getattr(parsed, "hand", False)) == hand_ready
        assert bool(getattr(parsed, "gripper", False)) == gripper_ready
        assert bool(getattr(parsed, "real", False)) == args.real
        if hand_ready:
            assert parsed.hand_remote == args.hand_remote
        if gripper_ready:
            assert parsed.gripper_remote == args.gripper_remote
    print("[10] 启动脚本拼的 argv 全部被服务端解析器接住（含 --no-hand/--no-real）")


def test_package_preflight(launcher):
    """启动前的 `litearm_teleop` 归属检查：必须指向本检出，否则指名道姓地报错。

    这条守卫的存在理由是本机真撞过一次：项目目录搬走之后 editable 安装里写死的旧
    绝对路径还在，服务端抛 ModuleNotFoundError，看起来像服务端自己崩了。
    """
    # 正常路径：跑测试的解释器里 litearm_teleop 就指向本检出，不该抛
    launcher.check_package(sys.executable, dict(os.environ))

    # 「跑得成功但什么也没印」等价于 import 不到 → 必须退出（用 /bin/true 冒充解释器）
    try:
        launcher.check_package("/bin/true", dict(os.environ))
        raise AssertionError("import 不到时应当退出")
    except SystemExit as stopped:
        assert stopped.code == 1
    print("[11] litearm_teleop 归属检查：正常放行，import 不到则指名报错并退出")


def test_vr_source_config():
    """数据源（`vr.*`）的出厂值必须与 `vr_source.py` 的常量一致。

    [9] 只保证「键在配置文件里存在」，这里保证「值对得上」—— 两者漂了不会报错，
    只会表现为「桥在发、这儿一条也订不到」。
    """
    cfg = load()
    assert get(cfg, "vr.source") == DEFAULT_SOURCE
    assert get(cfg, "vr.arm_id") == DEFAULT_ARM_ID
    assert get(cfg, "vr.zenoh_mode") == DEFAULT_ZENOH_MODE
    assert parse_endpoints(get(cfg, "vr.zenoh_connect")) == DEFAULT_ZENOH_CONNECT
    # zenoh_listen 出厂留空（= 不监听）：同机的桥已经占了 7447，两边都听会撞
    assert parse_endpoints(get(cfg, "vr.zenoh_listen", None)) == DEFAULT_ZENOH_LISTEN \
        == (), "出厂不该监听任何端点（同机的桥已占 7447）"

    # 话题：必须是桥约定里的那条（arm_id 段跟着配置走）
    assert frame_topic(get(cfg, "vr.arm_id")) == "litearm/v4/armA/vr_frame"
    print("[12] vr.*：source/arm_id/zenoh_mode/zenoh_connect 出厂值 == 代码常量，"
          "listen 出厂为空；话题 = %s" % frame_topic(get(cfg, "vr.arm_id")))


def main():
    for key, value in Tuning().tracking.items():
        assert get(load(), "tracking." + key) == value
    for invalid in ({"input_timeout": 0}, {"max_joint_velocity": float("nan")}, {"typo": 1}):
        try:
            Tuning(tracking=invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid tracking settings accepted")
    test_parser()
    launcher = test_yaml_matches_constants()
    test_precedence(launcher)
    test_option_paths_exist()
    test_launcher_argv_accepted_by_server(launcher)
    test_package_preflight(launcher)
    test_vr_source_config()
    print("CONFIG TESTS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())

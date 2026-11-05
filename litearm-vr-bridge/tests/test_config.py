from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from litearm_vr_bridge.config import (
    KIND_CUSTOM,
    KIND_FOLLOW_SDK,
    KIND_FOLLOW_STM32,
    KIND_TELEOP,
    Config,
    parse_endpoint,
    repo_root,
)

TELEOP_LAUNCHER = "submodules/litearm-vr-teleop/teleop/launch/run_teleop_webxr.py"
TELEOP_SCRIPT = "submodules/litearm-vr-teleop/teleop/scripts/ws_echo_server.py"
FOLLOW_LAUNCHER = "submodules/litearm-vr-follow/run_stm32_server.sh"
FOLLOW_SDK_LAUNCHER = "submodules/litearm-vr-follow/run_sdk_server.sh"
FOLLOW_SCRIPT = "submodules/litearm-vr-follow/arm_follow_server_stm32.py"
FOLLOW_SDK_SCRIPT = "submodules/litearm-vr-follow/arm_follow_server_sdk.py"


class ParseEndpointTest(unittest.TestCase):
    def test_host_port(self) -> None:
        self.assertEqual(parse_endpoint("192.168.31.108:8765", 1), ("192.168.31.108", 8765))

    def test_scheme_and_path_ignored(self) -> None:
        self.assertEqual(parse_endpoint("ws://127.0.0.1:8766/ws", 1), ("127.0.0.1", 8766))

    def test_missing_port_uses_default(self) -> None:
        self.assertEqual(parse_endpoint("127.0.0.1", 9001), ("127.0.0.1", 9001))
        self.assertEqual(parse_endpoint("127.0.0.1:", 9001), ("127.0.0.1", 9001))

    def test_ipv6(self) -> None:
        self.assertEqual(parse_endpoint("[::1]:9001", 1), ("::1", 9001))

    def test_invalid_port_raises(self) -> None:
        with self.assertRaises(ValueError):
            parse_endpoint("127.0.0.1:abc", 1)

    def test_empty_raises(self) -> None:
        with self.assertRaises(ValueError):
            parse_endpoint("   ", 1)


class RepoRootTest(unittest.TestCase):
    def test_finds_this_repo(self) -> None:
        """从 src/litearm_vr_bridge/config.py 往上，第一个带 pyproject.toml 的就是仓库根。"""
        root = repo_root()
        assert root is not None
        self.assertTrue((root / "pyproject.toml").is_file())
        self.assertTrue((root / "src" / "litearm_vr_bridge").is_dir())

    def test_none_outside_a_repo(self) -> None:
        """以非 editable 方式装进 site-packages 时推不出来 —— 那时要靠 BRIDGE_ROOT。"""
        with tempfile.TemporaryDirectory() as tmp:
            deep = Path(tmp) / "lib" / "python3.13" / "site-packages" / "x.py"
            deep.parent.mkdir(parents=True)
            self.assertIsNone(repo_root(deep))


class ConfigFromEnvTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        for rel in (
            "submodules/litearm-vr-teleop/teleop/launch",
            "submodules/litearm-vr-teleop/teleop/scripts",
            "submodules/litearm-vr-follow",
        ):
            (self.root / rel).mkdir(parents=True)

    def env(self, **over: str) -> dict[str, str]:
        env = {"BRIDGE_ROOT": str(self.root)}
        env.update(over)
        return env

    def test_defaults(self) -> None:
        cfg = Config.from_env(self.env())
        self.assertEqual((cfg.bridge_host, cfg.bridge_port), ("0.0.0.0", 8765))
        self.assertEqual(cfg.teleop_endpoint, "127.0.0.1:8766")
        self.assertEqual(cfg.follow_endpoint, "127.0.0.1:9001")
        self.assertEqual(cfg.frame_warn_ms, 200.0)
        self.assertEqual(cfg.frame_fallback_ms, 500.0)
        self.assertEqual(cfg.status_hz, 10.0)
        self.assertEqual(cfg.arm_source, "follow_server")
        self.assertTrue(cfg.fallback_enabled)
        self.assertEqual(cfg.teleop_url, "ws://127.0.0.1:8766/")
        # 有仓库布局（submodule 就位）时，两个上游默认都受管
        self.assertEqual(set(cfg.processes), {"teleop", "follow_server"})
        self.assertEqual(cfg.repo_root, self.root)

    def test_env_overrides(self) -> None:
        cfg = Config.from_env(
            self.env(
                BRIDGE_PORT="8865",
                TELEOP_ENDPOINT="127.0.0.1:9766",
                FOLLOW_SERVER_ENDPOINT="192.168.31.139:9001",
                FRAME_WARN_MS="150",
                FRAME_FALLBACK_MS="400",
                STATUS_HZ="20",
                FALLBACK_ENABLED="0",
                LOG_LEVEL="debug",
                ARM_SOURCE="controller",
            )
        )
        self.assertEqual(cfg.bridge_port, 8865)
        self.assertEqual(cfg.teleop_endpoint, "127.0.0.1:9766")
        self.assertEqual(cfg.follow_endpoint, "192.168.31.139:9001")
        self.assertEqual(cfg.frame_warn_ms, 150.0)
        self.assertEqual(cfg.frame_fallback_ms, 400.0)
        self.assertEqual(cfg.status_hz, 20.0)
        self.assertFalse(cfg.fallback_enabled)
        self.assertEqual(cfg.log_level, "debug")
        self.assertEqual(cfg.arm_source, "controller")

    def test_invalid_numbers_fall_back_to_defaults(self) -> None:
        cfg = Config.from_env(self.env(BRIDGE_PORT="x", STATUS_HZ="abc"))
        self.assertEqual(cfg.bridge_port, 8765)
        self.assertEqual(cfg.status_hz, 10.0)

    # ── 默认命令：上游自己的启动脚本 ────────────────────────────────────────

    def test_default_commands_run_upstream_launchers(self) -> None:
        """桥调上游自己的 launcher，不自己拼 ws_echo_server.py 的命令行 ——
        找解释器 / source ROS2 / 起 rviz / 清残留 / 查端口都在那边。"""
        cfg = Config.from_env(self.env())

        teleop = cfg.processes["teleop"]
        self.assertEqual(
            teleop.argv,
            ("python3", str(self.root / TELEOP_LAUNCHER),
             "--host", "127.0.0.1", "--port", "8766",
             "--viewer", "gui",
             # 未设置 TELEOP_ARM_HOST = 默认接本服务的 follow_server
             "--real", "--real-right", "127.0.0.1:9001"),
        )
        self.assertEqual(teleop.kind, KIND_TELEOP)
        self.assertEqual(teleop.probe_script, str(self.root / TELEOP_SCRIPT))

        follow = cfg.processes["follow_server"]
        self.assertEqual(
            follow.argv,
            ("bash", str(self.root / FOLLOW_LAUNCHER),
             "--host", "127.0.0.1", "--tcp-port", "9001",
             "--mode", "js", "--port", "/dev/ttyACM0"),
        )
        self.assertEqual(follow.kind, KIND_FOLLOW_STM32)
        self.assertEqual(follow.probe_script, str(self.root / FOLLOW_SCRIPT))

    def test_teleop_argv_is_the_upstream_contract(self) -> None:
        """传的是 launcher `--help` 那张表里的参数，不是我们自创的。

        上游 2026-10-15 起参数取代了环境变量，所以这里断言命令行，不断言 env。
        """
        argv = Config.from_env(self.env()).processes["teleop"].argv
        self.assertEqual(argv[argv.index("--host") + 1], "127.0.0.1")
        self.assertEqual(argv[argv.index("--port") + 1], "8766")
        self.assertEqual(argv[argv.index("--viewer") + 1], "gui")
        self.assertNotIn("--side", argv)
        self.assertNotIn("--dual", argv)
        self.assertNotIn("--ik", argv)  # 不设就用上游默认（pinocchio 微分 IK）
        self.assertNotIn("--hand", argv)
        self.assertNotIn("--metavr-py", argv)  # 留空 = launcher 自己解析

    def test_teleop_env_carries_no_launch_settings(self) -> None:
        """启动设置全走命令行，子进程 env 里不再有 WS_PORT/DUAL/REAL 那一套。"""
        env = Config.from_env(self.env()).processes["teleop"].env
        for key in ("WS_HOST", "WS_PORT", "VIEWER", "DUAL", "SIDE", "REAL",
                    "HAND", "GRIPPER", "IK"):
            self.assertNotIn(key, env)

    def test_teleop_endpoint_reaches_the_launcher(self) -> None:
        cfg = Config.from_env(self.env(TELEOP_ENDPOINT="127.0.0.1:9766"))
        argv = cfg.processes["teleop"].argv
        self.assertEqual(argv[argv.index("--port") + 1], "9766")

    def test_teleop_py_becomes_teleop_py_flag(self) -> None:
        cfg = Config.from_env(self.env(TELEOP_SERVER_PY="/opt/conda/envs/metavr-arm/bin/python"))
        argv = cfg.processes["teleop"].argv
        self.assertEqual(argv[argv.index("--metavr-py") + 1],
                         "/opt/conda/envs/metavr-arm/bin/python")

    def test_side_viewer_and_ik(self) -> None:
        cfg = Config.from_env(
            self.env(TELEOP_SIDE="1", TELEOP_VIEWER="rviz", TELEOP_IK="pybullet")
        )
        argv = cfg.processes["teleop"].argv
        self.assertIn("--side", argv)
        self.assertNotIn("--dual", argv)
        self.assertEqual(argv[argv.index("--viewer") + 1], "rviz")
        self.assertEqual(argv[argv.index("--ik") + 1], "pybullet")

    def test_empty_teleop_arm_host_keeps_sim_only(self) -> None:
        cfg = Config.from_env(self.env(TELEOP_ARM_HOST=""))
        argv = cfg.processes["teleop"].argv
        self.assertNotIn("--real", argv)
        self.assertNotIn("--real-right", argv)

    def test_real_left_override(self) -> None:
        cfg = Config.from_env(self.env(TELEOP_REAL_LEFT="127.0.0.1:9002"))
        argv = cfg.processes["teleop"].argv
        self.assertEqual(argv[argv.index("--real-left") + 1], "127.0.0.1:9002")

    def test_hand_and_gripper(self) -> None:
        cfg = Config.from_env(
            self.env(
                TELEOP_HAND="1",
                TELEOP_GRIPPER="1",
                TELEOP_GRIPPER_REMOTE="192.168.31.9:9100",
            )
        )
        argv = cfg.processes["teleop"].argv
        self.assertIn("--hand", argv)
        self.assertEqual(argv[argv.index("--hand-remote") + 1], "127.0.0.1:9004")
        self.assertIn("--gripper", argv)
        self.assertEqual(argv[argv.index("--gripper-remote") + 1], "192.168.31.9:9100")
        # 驱动移到 follow 仓之后，本地 CAN 参数不再存在
        self.assertNotIn("--hand-can", argv)
        self.assertNotIn("--gripper-can", argv)

    def test_stm32_serial_and_ctrl_mode(self) -> None:
        cfg = Config.from_env(
            self.env(FOLLOW_SERVER_SERIAL="/dev/ttyUSB0", FOLLOW_SERVER_CTRL_MODE="mit")
        )
        follow = cfg.processes["follow_server"]
        self.assertEqual(follow.argv[follow.argv.index("--port") + 1], "/dev/ttyUSB0")
        self.assertEqual(follow.argv[follow.argv.index("--mode") + 1], "mit")

    def test_sdk_mode_and_dry_run(self) -> None:
        cfg = Config.from_env(
            self.env(FOLLOW_SERVER_MODE="sdk", FOLLOW_SERVER_CAN="can1",
                     FOLLOW_SERVER_DRY_RUN="1")
        )
        follow = cfg.processes["follow_server"]
        self.assertEqual(follow.argv, ("bash", str(self.root / FOLLOW_SDK_LAUNCHER),
                                       "--host", "127.0.0.1", "--tcp-port", "9001",
                                       "--iface", "can1", "--dry-run"))
        self.assertEqual(follow.kind, KIND_FOLLOW_SDK)
        self.assertEqual(follow.probe_script, str(self.root / FOLLOW_SDK_SCRIPT))

    # ── 自定义命令与不托管 ──────────────────────────────────────────────────

    def test_explicit_commands_win(self) -> None:
        cfg = Config.from_env(
            self.env(TELEOP_CMD="python3 /tmp/other_server.py --port 8766 --no-viewer")
        )
        teleop = cfg.processes["teleop"]
        self.assertEqual(
            teleop.argv,
            ("python3", "/tmp/other_server.py", "--port", "8766", "--no-viewer"),
        )
        self.assertEqual(teleop.kind, KIND_CUSTOM)
        self.assertIsNone(teleop.probe_script)

    def test_blank_command_disables_process(self) -> None:
        cfg = Config.from_env(self.env(TELEOP_CMD="   "))
        self.assertNotIn("teleop", cfg.processes)
        self.assertIn("follow_server", cfg.processes)

    def test_without_repo_layout_nothing_is_managed(self) -> None:
        """推不出仓库根（非源码运行 / 没 init）时不猜路径，交给 *_CMD。"""
        with mock.patch("litearm_vr_bridge.config.repo_root", return_value=None):
            cfg = Config.from_env({})
        self.assertEqual(cfg.processes, {})
        self.assertIsNone(cfg.repo_root)

    def test_bridge_root_points_elsewhere(self) -> None:
        with mock.patch("litearm_vr_bridge.config.repo_root", return_value=Path("/nope")):
            cfg = Config.from_env(self.env())
        self.assertEqual(cfg.repo_root, self.root)
        self.assertEqual(cfg.processes["teleop"].argv[1], str(self.root / TELEOP_LAUNCHER))

    def test_real_arm_property(self) -> None:
        self.assertTrue(Config.from_env(self.env()).real_arm)
        self.assertFalse(Config.from_env(self.env(TELEOP_ARM_HOST="")).real_arm)
        self.assertTrue(
            Config.from_env(self.env(TELEOP_CMD="python3 /x.py --real")).real_arm
        )
        self.assertFalse(Config.from_env(self.env(TELEOP_CMD="")).real_arm)

    def test_process_label_shows_the_launcher(self) -> None:
        label = Config.from_env(self.env()).processes["teleop"].label()
        self.assertIn(TELEOP_LAUNCHER, label)
        self.assertIn("--port 8766", label)

    def test_split_repos_layout(self) -> None:
        """上游不在 submodule 里时，`TELEOP_REPO` / `FOLLOW_REPO` 指到各仓根。"""
        cfg = Config.from_env(
            {
                "TELEOP_REPO": "/opt/litearm-vr-teleop",
                "FOLLOW_REPO": "/opt/litearm-vr-follow",
            }
        )
        self.assertEqual(set(cfg.processes), {"teleop", "follow_server"})
        teleop = cfg.processes["teleop"].argv
        self.assertIn("/opt/litearm-vr-teleop/teleop/launch/run_teleop_webxr.py", teleop)
        follow = cfg.processes["follow_server"].argv
        self.assertIn("/opt/litearm-vr-follow/run_stm32_server.sh", follow)

    def test_repo_override_wins_over_submodule_layout(self) -> None:
        cfg = Config.from_env(self.env(TELEOP_REPO="/t", FOLLOW_REPO="/f"))
        self.assertIn("/t/teleop/launch/run_teleop_webxr.py",
                      cfg.processes["teleop"].argv)
        self.assertIn("/f/run_stm32_server.sh",
                      cfg.processes["follow_server"].argv)

    def test_teleop_dual_upright_flag(self) -> None:
        cfg = Config.from_env(self.env(TELEOP_DUAL="1", TELEOP_DUAL_UPRIGHT="1"))
        teleop = cfg.processes["teleop"].argv
        self.assertIn("--dual", teleop)
        self.assertIn("--upright", teleop)

    def test_teleop_upright_ignored_without_dual(self) -> None:
        cfg = Config.from_env(self.env(TELEOP_DUAL_UPRIGHT="1"))
        self.assertNotIn("--upright", cfg.processes["teleop"].argv)

    def test_hand_remote_flag_defaults_to_follow_host(self) -> None:
        cfg = Config.from_env(
            self.env(TELEOP_HAND="1", FOLLOW_SERVER_ENDPOINT="192.168.31.139:9001")
        )
        teleop = cfg.processes["teleop"].argv
        self.assertIn("--hand", teleop)
        self.assertEqual(teleop[teleop.index("--hand-remote") + 1], "192.168.31.139:9004")

    def test_hand_remote_flag_override(self) -> None:
        cfg = Config.from_env(self.env(TELEOP_HAND="1",
                                       TELEOP_HAND_REMOTE="10.0.0.9:9104"))
        teleop = cfg.processes["teleop"].argv
        self.assertEqual(teleop[teleop.index("--hand-remote") + 1], "10.0.0.9:9104")

    def test_gripper_remote_flag_defaults_to_follow_host(self) -> None:
        cfg = Config.from_env(self.env(TELEOP_GRIPPER="1"))
        teleop = cfg.processes["teleop"].argv
        self.assertIn("--gripper", teleop)
        self.assertEqual(teleop[teleop.index("--gripper-remote") + 1], "127.0.0.1:9003")
        # 远端模式不再出现本地 CAN 参数
        self.assertNotIn("--gripper-can", teleop)
        self.assertNotIn("--gripper-can-id", teleop)

    def test_hand_flag_absent_by_default(self) -> None:
        teleop = Config.from_env(self.env()).processes["teleop"].argv
        self.assertNotIn("--hand", teleop)
        self.assertNotIn("--gripper", teleop)

    # ── 旧的 METAVR_* 变量名（metavr → teleop 改名留下） ────────────────────

    def test_legacy_blank_cmd_still_disables_the_process(self) -> None:
        """**这条是安全用例**：旧写法 `METAVR_CMD=` 的语义是"这个进程我不要"。

        改名之后如果只认新名字，这个键会被静默忽略、回落到默认命令 —— 一个本该
        不托管 teleop 的真机现场会把它拉起来。
        """
        cfg = Config.from_env(self.env(METAVR_CMD=""))
        self.assertNotIn("teleop", cfg.processes)
        self.assertEqual(cfg.legacy_env, ("METAVR_CMD",))

    def test_legacy_blank_arm_host_still_means_sim_only(self) -> None:
        """**同样是安全用例**：旧写法 `METAVR_ARM_HOST=` 的语义是"只跑仿真"。

        忽略掉就默认接真机 —— 那就成了"配置看着是仿真，实际动真臂"。
        """
        cfg = Config.from_env(self.env(METAVR_ARM_HOST=""))
        argv = cfg.processes["teleop"].argv
        self.assertNotIn("--real", argv)
        self.assertFalse(cfg.real_arm)
        self.assertEqual(cfg.legacy_env, ("METAVR_ARM_HOST",))

    def test_legacy_names_are_reported_for_migration(self) -> None:
        cfg = Config.from_env(self.env(METAVR_SIDE="1", METAVR_VIEWER="rviz"))
        self.assertEqual(cfg.legacy_env, ("METAVR_SIDE", "METAVR_VIEWER"))
        argv = cfg.processes["teleop"].argv
        self.assertIn("--side", argv)
        self.assertEqual(argv[argv.index("--viewer") + 1], "rviz")

    def test_legacy_py_maps_to_server_py(self) -> None:
        """`METAVR_PY` 的尾巴也变了：它现在叫 `TELEOP_SERVER_PY`（不是 TELEOP_PY）。"""
        cfg = Config.from_env(self.env(METAVR_PY="/opt/conda/envs/metavr-arm/bin/python"))
        argv = cfg.processes["teleop"].argv
        self.assertEqual(argv[argv.index("--metavr-py") + 1],
                         "/opt/conda/envs/metavr-arm/bin/python")

    def test_new_name_wins_and_is_not_reported_as_legacy(self) -> None:
        """两个都在 = 迁移途中，不该一直告警。"""
        cfg = Config.from_env(self.env(METAVR_SIDE="0", TELEOP_SIDE="1"))
        self.assertIn("--side", cfg.processes["teleop"].argv)
        self.assertEqual(cfg.legacy_env, ())

    def test_clean_config_reports_no_legacy_names(self) -> None:
        self.assertEqual(Config.from_env(self.env()).legacy_env, ())

    def test_with_overrides_does_not_touch_original(self) -> None:
        cfg = Config.from_env(self.env())
        other = cfg.with_overrides(bridge_port=9999, status_hz=None)
        self.assertEqual(other.bridge_port, 9999)
        self.assertEqual(cfg.bridge_port, 8765)


if __name__ == "__main__":
    unittest.main()

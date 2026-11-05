"""预检（--doctor）：造一个临时仓库布局（含假 submodule）+ 假外部世界，把每类判定走一遍。

重点覆盖两种"对拍"——它们是把"上游被就地改过 / 换了接口"这种现场故障提前变成
一条警告的那个检查：
  * `flags:*`：命令行还是我们拼的（follow，参数透传），比对脚本声明的 `--flag`
  * `env:*`  ：参数变成环境变量的（teleop），比对上游启动脚本读不读我们设的 `$VAR`
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from litearm_vr_bridge.config import Config
from litearm_vr_bridge.preflight import (
    FAIL,
    INFO,
    OK,
    WARN,
    Check,
    Probes,
    SubmoduleState,
    _CONDA_ROOTS,
    _submodule_state,
    format_report,
    has_failures,
    run_checks,
)

TELEOP_DIR = "submodules/litearm-vr-teleop"
FOLLOW_DIR = "submodules/litearm-vr-follow"
# teleop 仓 2026-10-15 起代码在 `teleop/` 子目录里，launcher 从 .sh 换成了 .py
TELEOP_LAUNCHER = f"{TELEOP_DIR}/teleop/launch/run_teleop_webxr.py"
TELEOP_SCRIPTS = f"{TELEOP_DIR}/teleop/scripts"
FOLLOW_LAUNCHER = f"{FOLLOW_DIR}/run_stm32_server.sh"

# 假的上游 launcher：把上游那张参数表抄了一遍（flag 对拍要读它）。
# 上游用的是 BooleanOptionalAction，所以 --dual/--side 这些还能带 --no- 前缀。
TELEOP_LAUNCHER_SRC = """
import argparse
p = argparse.ArgumentParser()
p.add_argument("--config")
p.add_argument("--metavr-py")
p.add_argument("--host")
p.add_argument("--port", type=int)
for flag in ("--dual", "--upright", "--side", "--real", "--hand", "--gripper"):
    p.add_argument(flag, action=argparse.BooleanOptionalAction)
p.add_argument("--viewer", choices=["gui", "rviz"])
p.add_argument("--probe")
p.add_argument("--ik", choices=["pin", "pybullet", "exact"])
p.add_argument("--tcp-offset", type=float, nargs=3)
p.add_argument("--deadband-scale", type=float)
p.add_argument("--real-left")
p.add_argument("--real-right")
p.add_argument("--hand-remote")
p.add_argument("--gripper-remote")
p.parse_args()
"""

FOLLOW_STM32_LAUNCHER_SRC = """
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
LITEARM_TOOLS="${LITEARM_TOOLS:-$(dirname "$PWD")/litearm-stm32/tools}"
export PYTHONPATH="$LITEARM_TOOLS${PYTHONPATH:+:$PYTHONPATH}"
exec python3 arm_follow_server_stm32.py "$@"
"""

FOLLOW_SDK_LAUNCHER_SRC = """
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec python3 arm_follow_server_sdk.py "$@"
"""

# 上游 .py 的参数声明（flag 对拍要读它）
TELEOP_SRC = """
import argparse
p = argparse.ArgumentParser()
for flag in ("--host", "--port", "--dual", "--side", "--real", "--real-right", "--viewer"):
    p.add_argument(flag)
p.add_argument("--no-ros", action="store_true")
p.parse_args()
"""

FOLLOW_STM32_SRC = """
import argparse
p = argparse.ArgumentParser()
p.add_argument("--host")
p.add_argument("--tcp-port", type=int)
p.add_argument("--mode", choices=["jf", "js", "j", "mit"], default="jf")
p.add_argument("--port", default="/dev/ttyACM0")
p.parse_args()
"""

FOLLOW_SDK_SRC = """
import argparse
p = argparse.ArgumentParser()
p.add_argument("--host")
p.add_argument("--tcp-port", type=int)
p.add_argument("--iface", default="can0")
p.add_argument("--dry-run", action="store_true")
p.parse_args()
"""


def probes(**over: object) -> Probes:
    """默认"submodule 就位、网络一律通、依赖一律齐"，单测不该依赖真实网卡和端口。"""
    params: dict[str, object] = {
        "bind_ok": lambda host, port: True,
        "connect_ok": lambda host, port: True,
        "health": lambda host, port: None,
        "import_probe": lambda exe, modules, timeout: (True, "齐备"),
        "lan_ip": lambda: "10.0.0.5",
        "which": lambda cmd: f"/usr/bin/{cmd}",
        "conda_base": lambda: None,
        "submodule_state": lambda path: SubmoduleState(
            present=True, tracked=True, sha="abc1234"
        ),
    }
    params.update(over)
    return Probes(**params)  # type: ignore[arg-type]


def find(checks: list[Check], name: str) -> Check:
    for check in checks:
        if check.name == name:
            return check
    raise AssertionError(f"没有 {name} 这项：{[c.name for c in checks]}")


class PreflightTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        (self.root / TELEOP_SCRIPTS).mkdir(parents=True)
        (self.root / Path(TELEOP_LAUNCHER).parent).mkdir(parents=True)
        (self.root / FOLLOW_DIR).mkdir(parents=True)
        (self.root / "tools").mkdir()
        (self.root / "tools" / "arm_console.py").write_text("", encoding="utf-8")
        self.device = self.root / "ttyACM0"
        self.device.write_text("", encoding="utf-8")
        self.probes = probes()
        self.write_repo()

    def write_repo(
        self,
        *,
        launcher: str = TELEOP_LAUNCHER_SRC,
        teleop: str = TELEOP_SRC,
        follow: str = FOLLOW_STM32_SRC,
        sdk: bool = False,
    ) -> None:
        (self.root / TELEOP_LAUNCHER).write_text(launcher, encoding="utf-8")
        (self.root / TELEOP_SCRIPTS / "ws_echo_server.py").write_text(
            teleop, encoding="utf-8"
        )
        if sdk:
            (self.root / FOLLOW_DIR / "run_sdk_server.sh").write_text(
                FOLLOW_SDK_LAUNCHER_SRC, encoding="utf-8"
            )
            (self.root / FOLLOW_DIR / "arm_follow_server_sdk.py").write_text(
                follow, encoding="utf-8"
            )
        else:
            (self.root / FOLLOW_LAUNCHER).write_text(
                FOLLOW_STM32_LAUNCHER_SRC, encoding="utf-8"
            )
            (self.root / FOLLOW_DIR / "arm_follow_server_stm32.py").write_text(
                follow, encoding="utf-8"
            )

    def env(self, **over: str) -> dict[str, str]:
        env = {
            "BRIDGE_ROOT": str(self.root),
            "TELEOP_SERVER_PY": "/usr/bin/python3",
            "FOLLOW_SERVER_MODE": "stm32",
            "FOLLOW_SERVER_SERIAL": str(self.device),
            "LITEARM_TOOLS": str(self.root / "tools"),
        }
        env.update(over)
        return env

    def config(self, **over: str) -> Config:
        return Config.from_env(self.env(**over))

    def run_checks(self, **over: str) -> list[Check]:
        """造配置 + 跑预检，两边用的是同一份环境变量（生产里都是 os.environ）。"""
        env = self.env(**over)
        return run_checks(Config.from_env(env), probes=self.probes, env=env)

    # ── 上游对拍 ────────────────────────────────────────────────────────────

    def test_contract_all_match(self) -> None:
        checks = self.run_checks()
        self.assertEqual(find(checks, "flags:teleop").status, OK)
        self.assertEqual(find(checks, "flags:follow_server").status, OK)
        self.assertEqual(find(checks, "submodules").status, OK)
        self.assertFalse(has_failures(checks))

    def test_zenoh_installed_and_importable(self) -> None:
        self.assertEqual(find(self.run_checks(), "zenoh").status, OK)

    def test_zenoh_port_free_is_ok(self) -> None:
        self.assertEqual(find(self.run_checks(), "zenoh_port").status, OK)

    def test_zenoh_port_busy_fails_with_a_pointer_to_the_culprit(self) -> None:
        """绑不上是**静默**故障：会话照开、发布者照声明，只是谁也连不进来。

        同机最常见的原因是 litearm-server 也监听 7447，所以提示里要点名。
        """
        self.probes = probes(bind_ok=lambda host, port: False)
        check = find(self.run_checks(), "zenoh_port")
        self.assertEqual(check.status, FAIL)
        self.assertIn("7447", check.hint)
        self.assertIn("litearm-server", check.hint)

    def test_zenoh_port_skipped_when_not_listening(self) -> None:
        check = find(self.run_checks(ZENOH_LISTEN_ENDPOINTS=""), "zenoh_port")
        self.assertEqual(check.status, INFO)

    def test_legacy_env_is_flagged(self) -> None:
        self.assertEqual(find(self.run_checks(), "legacy_env").status, OK)
        check = find(self.run_checks(METAVR_CMD=""), "legacy_env")
        self.assertEqual(check.status, WARN)
        self.assertIn("METAVR_CMD", check.detail)
        self.assertIn("TELEOP_CMD", check.hint)

    def test_flag_contract_lists_what_we_pass(self) -> None:
        check = find(self.run_checks(), "flags:teleop")
        for flag in ("--host", "--port", "--viewer", "--real", "--real-right"):
            self.assertIn(flag, check.detail)

    def test_flag_contract_catches_renamed_flag(self) -> None:
        """上游把 --real / --real-right 改名了：我们传的就是白传，真机不会动。"""
        self.write_repo(
            launcher=TELEOP_LAUNCHER_SRC.replace('"--real", ', "")
            .replace('p.add_argument("--real-right")', 'p.add_argument("--arm-right")')
        )
        check = find(self.run_checks(), "flags:teleop")
        self.assertEqual(check.status, WARN)
        self.assertIn("--real", check.detail)
        self.assertIn("--real-right", check.detail)
        # 只认整个词：--real-left 不该被当成 --real 的替身
        self.assertNotIn("--real-left", check.detail)

    def test_flag_contract_choice_mismatch_is_flagged(self) -> None:
        """stm32 的 --mode 只认 jf/j，而本服务默认传 js。"""
        self.write_repo(follow=FOLLOW_STM32_SRC.replace('"jf", "js", ', ""))
        check = find(self.run_checks(), "flags:follow_server")
        self.assertEqual(check.status, WARN)
        self.assertIn("--mode=js", check.detail)

    def test_flag_contract_skipped_for_custom_command(self) -> None:
        """FOLLOW_SERVER_CMD 换成非 .py 的东西时不该瞎报。"""
        check = find(
            self.run_checks(FOLLOW_SERVER_CMD="bash /opt/run_server.sh --tcp-port 9001"),
            "flags:follow_server",
        )
        self.assertEqual(check.status, INFO)

    # ── submodule ───────────────────────────────────────────────────────────

    def test_submodule_missing_fails(self) -> None:
        self.probes = probes(submodule_state=lambda p: SubmoduleState(present=False))
        checks = self.run_checks()
        check = find(checks, "submodules")
        self.assertEqual(check.status, FAIL)
        self.assertIn("git submodule update --init", check.hint or "")
        self.assertTrue(has_failures(checks))

    def test_submodule_dirty_warns(self) -> None:
        self.probes = probes(
            submodule_state=lambda p: SubmoduleState(
                present=True, tracked=True, sha="abc1234", dirty=True
            )
        )
        checks = self.run_checks()
        check = find(checks, "submodules")
        self.assertEqual(check.status, WARN)
        self.assertIn("就地改过", check.detail)
        self.assertFalse(has_failures(checks))

    def test_copied_submodule_without_git_is_not_a_failure(self) -> None:
        """tarball / rsync 部署：没有 .git，文件却齐全 —— 能跑，别拦。"""
        self.probes = probes(submodule_state=lambda p: SubmoduleState(present=True))
        checks = self.run_checks()
        check = find(checks, "submodules")
        self.assertEqual(check.status, INFO)
        self.assertIn("不是 git 检出", check.detail)
        self.assertFalse(has_failures(checks))
        # 文件在不在是 process:* 那几项管的，它们照常通过
        self.assertEqual(find(checks, "process:teleop").status, OK)
        self.assertEqual(find(checks, "process:follow_server").status, OK)

    def test_submodule_versions_are_reported(self) -> None:
        check = find(self.run_checks(), "submodules")
        self.assertIn("litearm-vr-teleop@abc1234", check.detail)
        self.assertIn("litearm-vr-follow@abc1234", check.detail)

    def test_submodule_check_without_repo_root(self) -> None:
        cfg = Config(bridge_host="127.0.0.1", bridge_port=8765)
        check = find(run_checks(cfg, probes=self.probes), "submodules")
        self.assertEqual(check.status, WARN)
        self.assertIn("BRIDGE_ROOT", check.hint or "")

    def test_submodule_state_real_probe(self) -> None:
        """真实现：没 init 的 submodule 是个**空目录**；拷进来的那份没有 .git。"""
        empty = self.root / "submodules" / "not-init"
        empty.mkdir(parents=True)
        self.assertFalse(_submodule_state(empty).present)

        # 拷目录 / 打包部署：有文件、没有 .git —— 文件在，就算"在"
        copied = _submodule_state(self.root / FOLLOW_DIR)
        self.assertTrue(copied.present)
        self.assertFalse(copied.tracked)

        # init 过的 submodule，`.git` 是指向 .git/modules/... 的文件
        (empty / ".git").write_text("gitdir: /nowhere\n", encoding="utf-8")
        (empty / "x.py").write_text("", encoding="utf-8")
        broken = _submodule_state(empty)
        self.assertTrue(broken.present)
        self.assertTrue(broken.tracked)
        self.assertIsNone(broken.sha)  # 不是真仓库，读不出 sha

    @unittest.skipUnless(shutil.which("git"), "没有 git")
    def test_submodule_state_detects_local_edits(self) -> None:
        repo = self.root / "submodules" / "dirty"
        repo.mkdir(parents=True)
        env = {
            "PATH": "/usr/bin:/bin",
            "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@example.com",
            "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@example.com",
        }
        for args in (("init", "-q"), ("add", "-A"), ("commit", "-qm", "x", "--allow-empty")):
            subprocess.run(["git", "-C", str(repo), *args], check=True,
                           capture_output=True, env=env)
        self.assertFalse(_submodule_state(repo).dirty)

        (repo / "local.patch").write_text("现场手改\n", encoding="utf-8")
        self.assertTrue(_submodule_state(repo).dirty)

    # ── 进程与硬件 ──────────────────────────────────────────────────────────

    def test_missing_launcher_fails(self) -> None:
        (self.root / TELEOP_LAUNCHER).unlink()
        check = find(self.run_checks(), "process:teleop")
        self.assertEqual(check.status, FAIL)
        self.assertIn("启动脚本不存在", check.detail)
        self.assertIn("git submodule update --init", check.hint or "")

    def test_missing_upstream_script_fails(self) -> None:
        """submodule 只 init 了一半：启动脚本在，真正的 .py 没了。"""
        (self.root / FOLLOW_DIR / "arm_follow_server_stm32.py").unlink()
        check = find(self.run_checks(), "process:follow_server")
        self.assertEqual(check.status, FAIL)
        self.assertIn("上游脚本不存在", check.detail)

    def test_serial_present_and_missing(self) -> None:
        self.assertEqual(find(self.run_checks(), "hardware").status, OK)
        cfg = self.config(FOLLOW_SERVER_SERIAL=str(self.root / "nope"))
        self.assertEqual(find(run_checks(cfg, probes=self.probes), "hardware").status, FAIL)

    def test_sdk_mode_checks_can_interface(self) -> None:
        self.write_repo(follow=FOLLOW_SDK_SRC, sdk=True)
        check = find(
            self.run_checks(FOLLOW_SERVER_MODE="sdk", FOLLOW_SERVER_CAN="can-does-not-exist"),
            "hardware",
        )
        self.assertEqual(check.status, FAIL)
        self.assertIn("can-does-not-exist", check.detail)

    def test_follow_tools_uses_upstream_search_order(self) -> None:
        """没设 LITEARM_TOOLS 时，按上游顺序退到"follow 仓库同级"。"""
        sibling = self.root / "submodules" / "litearm-stm32" / "tools"
        sibling.mkdir(parents=True)
        (sibling / "arm_console.py").write_text("", encoding="utf-8")
        check = find(self.run_checks(LITEARM_TOOLS=""), "follow_tools")
        self.assertEqual(check.status, OK)
        self.assertIn("litearm-stm32", check.detail)

    def test_follow_tools_missing_fails(self) -> None:
        check = find(self.run_checks(LITEARM_TOOLS=str(self.root / "nope")), "follow_tools")
        self.assertEqual(check.status, FAIL)
        self.assertIn("LITEARM_TOOLS", check.hint or "")

    def test_follow_tools_not_needed_for_sdk(self) -> None:
        self.write_repo(follow=FOLLOW_SDK_SRC, sdk=True)
        check = find(self.run_checks(FOLLOW_SERVER_MODE="sdk"), "follow_tools")
        self.assertEqual(check.status, INFO)

    def test_teleop_python_resolved_like_upstream(self) -> None:
        with mock.patch("litearm_vr_bridge.preflight._CONDA_ROOTS", ()):
            check = find(self.run_checks(), "teleop_python")
        self.assertEqual(check.status, OK)
        self.assertIn("/usr/bin/python3", check.detail)

    def test_teleop_python_warns_without_conda_env(self) -> None:
        """找不到 metavr-arm 时给指路，不是一路瞎猜。"""
        with mock.patch("litearm_vr_bridge.preflight._CONDA_ROOTS", ()):
            check = find(self.run_checks(TELEOP_SERVER_PY=""), "teleop_python")
        self.assertEqual(check.status, WARN)
        self.assertIn("TELEOP_SERVER_PY", check.hint or "")

    def test_teleop_python_found_in_conda_root(self) -> None:
        """上游的第三/四步：conda 根下的 envs/metavr-arm。"""
        base = self.root / "miniconda3"
        exe = base / "envs" / "metavr-arm" / "bin" / "python"
        exe.parent.mkdir(parents=True)
        exe.write_text("", encoding="utf-8")
        with mock.patch("litearm_vr_bridge.preflight._CONDA_ROOTS", (str(base),)):
            check = find(self.run_checks(TELEOP_SERVER_PY=""), "teleop_python")
        self.assertEqual(check.status, OK)
        self.assertIn("metavr-arm", check.detail)

    def test_unmanaged_processes_are_skipped(self) -> None:
        """两个上游都不托管时，不该冒出一堆假的 FAIL。"""
        checks = self.run_checks(TELEOP_CMD="", FOLLOW_SERVER_CMD="")
        self.assertEqual(find(checks, "hardware").status, INFO)
        self.assertIn("未托管", find(checks, "follow_tools").detail)
        self.assertFalse(
            [c for c in checks if c.name.split(":")[0] in ("process", "env", "flags")]
        )
        self.assertFalse(has_failures(checks))

    # ── 端口与图形 ──────────────────────────────────────────────────────────

    def test_port_busy_by_something_else_fails(self) -> None:
        self.probes = probes(bind_ok=lambda h, p: False)
        checks = self.run_checks(TELEOP_CMD="", FOLLOW_SERVER_CMD="")
        check = find(checks, "port_bridge")
        self.assertEqual(check.status, FAIL)
        self.assertTrue(has_failures(checks))

    def test_port_busy_by_our_own_bridge_is_ok(self) -> None:
        health = {"version": "0.1.0", "headset_online": False}
        self.probes = probes(bind_ok=lambda h, p: False, health=lambda h, p: health)
        checks = self.run_checks(TELEOP_CMD="", FOLLOW_SERVER_CMD="")
        self.assertEqual(find(checks, "port_bridge").status, OK)

    def test_display_warning_only_when_gui(self) -> None:
        self.assertEqual(find(self.run_checks(DISPLAY=""), "display").status, WARN)
        self.assertEqual(find(self.run_checks(DISPLAY=":0"), "display").status, OK)
        self.assertEqual(find(self.run_checks(TELEOP_VIEWER="rviz"), "display").status, INFO)

    def test_real_arm_warning(self) -> None:
        check = find(self.run_checks(), "real_arm")
        self.assertEqual(check.status, WARN)
        self.assertIn("127.0.0.1:9001", check.detail)  # 说清楚挂到哪条臂上

        sim = find(self.run_checks(TELEOP_ARM_HOST=""), "real_arm")
        self.assertEqual(sim.status, INFO)
        self.assertIn("不会碰真机", sim.detail)

    def test_lan_addr_line(self) -> None:
        check = find(self.run_checks(BRIDGE_PORT="8765"), "lan_addr")
        self.assertIn("ws://10.0.0.5:8765", check.detail)

    # ── 报告 ────────────────────────────────────────────────────────────────

    def test_report_groups_and_concludes(self) -> None:
        (self.root / TELEOP_LAUNCHER).unlink()
        checks = self.run_checks()
        report = format_report(checks)
        self.assertIn("[FAIL]", report)
        self.assertIn("必须修好", report)
        self.assertIn("→", report)  # 带修复提示
        self.assertTrue(has_failures(checks))

    def test_report_all_clear(self) -> None:
        checks = self.run_checks(DISPLAY=":0")
        # 只剩 real_arm 那条 WARN（接真机的提醒）
        self.assertFalse(has_failures(checks))
        self.assertIn("确认后可以启动", format_report(checks))


if __name__ == "__main__":
    unittest.main()

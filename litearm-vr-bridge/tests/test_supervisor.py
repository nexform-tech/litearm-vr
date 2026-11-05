from __future__ import annotations

import asyncio
import sys
import tempfile
import unittest
from pathlib import Path

from litearm_vr_bridge.config import ProcessSpec
from litearm_vr_bridge.supervisor import ProcessSupervisor

SLEEPER = ProcessSpec(
    name="sleeper",
    argv=(sys.executable, "-c", "import time; time.sleep(60)"),
)
EXITER = ProcessSpec(
    name="exiter",
    argv=(sys.executable, "-c", "import sys; sys.exit(3)"),
)


class SupervisorTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.exits: list[tuple[str, int | None, bool]] = []
        self.sup = ProcessSupervisor(
            {"sleeper": SLEEPER, "exiter": EXITER},
            log_dir=Path(self._tmp.name),
            on_exit=lambda name, code, expected: self.exits.append((name, code, expected)),
        )

    async def asyncTearDown(self) -> None:
        await self.sup.stop_all()
        self._tmp.cleanup()

    async def test_unconfigured_process_is_reported(self) -> None:
        reply = await self.sup.start("teleop")
        self.assertFalse(reply["ok"])
        self.assertIn("未配置", reply["error"])
        self.assertEqual(self.sup.statuses(), {"sleeper": "stopped", "exiter": "stopped"})

    async def test_start_stop_roundtrip(self) -> None:
        reply = await self.sup.start("sleeper")
        self.assertTrue(reply["ok"])
        self.assertGreater(reply["pid"], 0)
        self.assertTrue(self.sup.running("sleeper"))
        self.assertEqual(self.sup.statuses()["sleeper"], "running")
        self.assertTrue(Path(reply["log"]).exists())

        again = await self.sup.start("sleeper")
        self.assertTrue(again.get("already_running"))
        self.assertEqual(self.sup.details()["sleeper"]["starts"], 1)

        stopped = await self.sup.stop("sleeper")
        self.assertTrue(stopped["ok"])
        self.assertFalse(self.sup.running("sleeper"))
        self.assertEqual(self.sup.statuses()["sleeper"], "stopped")
        self.sup.refresh()
        self.assertEqual(self.exits, [])  # 主动停止由 stop() 自己收割，不算"意外退出"

    async def test_second_stop_is_noop(self) -> None:
        await self.sup.start("sleeper")
        await self.sup.stop("sleeper")
        reply = await self.sup.stop("sleeper")
        self.assertTrue(reply.get("already_stopped"))

    async def test_crash_harvested_with_event(self) -> None:
        await self.sup.start("exiter")
        for _ in range(50):
            self.sup.refresh()
            if self.exits:
                break
            await asyncio.sleep(0.05)
        self.assertEqual(self.exits, [("exiter", 3, False)])
        self.assertEqual(self.sup.statuses()["exiter"], "crashed")
        self.assertIn("3", self.sup.details()["exiter"]["last_error"] or "")

    async def test_uptime_freezes_after_exit(self) -> None:
        """已退出的进程不能继续"涨运行时长"——界面照它显示。"""
        await self.sup.start("exiter")
        for _ in range(50):
            self.sup.refresh()
            if self.exits:
                break
            await asyncio.sleep(0.05)
        first = self.sup.details()["exiter"]["uptime_s"]
        await asyncio.sleep(0.6)
        self.sup.refresh()
        second = self.sup.details()["exiter"]["uptime_s"]
        self.assertEqual(first, second)

    async def test_unconfigured_names_listed(self) -> None:
        self.assertEqual(sorted(self.sup.unconfigured()), ["follow_server", "teleop"])

    async def test_stop_all_ignores_unknown(self) -> None:
        await self.sup.start("sleeper")
        await self.sup.stop_all()
        self.assertFalse(self.sup.running("sleeper"))


if __name__ == "__main__":
    unittest.main()

"""`--ctl`：给运行中的桥发动作（快捷启动的"扳机"）。

其中 `test_start_waits_until_follow_server_listens` 守的是一个真实修过的回归：
`teleop.start` 冷启动时，`start` 指令必须等 follow_server 真的 listen 之后再发。
之前 governor 一被调用就把命令排进队列，旁路看门狗 50ms 一轮抢在
`_ensure_processes` 还在 await 的时候发出去 —— 真机上 follow_server 要等 STM32
状态帧、几秒后才 listen，那条 start 就打在没监听的端口上丢了，操作员按 grip 不动。
"""

from __future__ import annotations

import asyncio
import contextlib
import os
import signal
import sys
import socket
import tempfile
import unittest
from pathlib import Path
from typing import Any, AsyncIterator

from litearm_vr_bridge import ctl
from litearm_vr_bridge.bridge import Bridge
from litearm_vr_bridge.config import Config, ProcessSpec

sys.path.insert(0, str(Path(__file__).resolve().parent))
from helpers import FakeFollowServer, free_port  # noqa: E402

# 模拟真机：先"连 STM32"几秒，再 listen；把收到的命令追加到文件供断言
SLOW_FOLLOW = """
import json, socket, sys, time
port, delay, logfile = int(sys.argv[1]), float(sys.argv[2]), sys.argv[3]

def record(text):
    with open(logfile, "a", encoding="utf-8") as handle:
        handle.write(text + "\\n")

time.sleep(delay)
server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("127.0.0.1", port))
server.listen(5)
record("listening")
while True:
    conn, _ = server.accept()
    stream = conn.makefile("rw")
    for line in stream:
        command = json.loads(line).get("cmd")
        record(command)
        stream.write(json.dumps({"ok": True, "state": "MOVE_JS", "q": [0.0] * 7, "flags": []}) + "\\n")
        stream.flush()
"""


@contextlib.asynccontextmanager
async def running_bridge(**over: Any) -> AsyncIterator[tuple[Bridge, FakeFollowServer]]:
    follow = FakeFollowServer()
    await follow.start()
    tmp = tempfile.TemporaryDirectory()
    params: dict[str, Any] = dict(
        bridge_host="127.0.0.1",
        bridge_port=free_port(),
        teleop_host="127.0.0.1",
        teleop_port=free_port(),
        follow_host="127.0.0.1",
        follow_port=follow.port,
        status_hz=50.0,
        ping_hz=20.0,
        processes={},
        process_log_dir=Path(tmp.name),
    )
    params.update(over)
    bridge = Bridge(Config(**params))
    await bridge.start()
    try:
        yield bridge, follow
    finally:
        await bridge.close()
        await follow.stop()
        tmp.cleanup()


# ── 上游启动脚本的形状：bash 起个 python 子进程（**不是 exec**），退出时 pkill 清理 ──
#
# 真实的 `run_teleop_webxr.sh` 就是这样：bash 在前、ws_echo_server.py 在后（R1 里还有
# rviz/robot_state_publisher）。桥必须能把**这一组**连根收掉，而不是只收掉 bash 留个孤儿
# 抱着 8766 不放。follow 那边的 wrapper 是 `exec`（单进程），所以只有 teleop 有这个问题。
FAKE_TELEOP_CHILD = """
import asyncio, os, sys
from pathlib import Path
from websockets.asyncio.server import serve

port, pidfile = int(sys.argv[1]), sys.argv[2]
Path(pidfile).write_text(str(os.getpid()), encoding="utf-8")

async def handler(conn):
    async for _message in conn:   # 收帧就行，内容不看
        pass

async def main():
    async with serve(handler, "127.0.0.1", port):
        await asyncio.Future()

asyncio.run(main())
"""

# 注意：下面这串要过 str.format，所以 shell 的 { } 得写成 {{ }}（trap 那两行）
FAKE_TELEOP_WRAPPER = """#!/usr/bin/env bash
set -euo pipefail
cleanup() {{ pkill -f "[f]ake_teleop_child" 2>/dev/null || true; }}
trap cleanup EXIT
echo "[fake-teleop] 起来了"
python3 "{child}" "{port}" "{pidfile}" &
wait $!
"""


def _child_alive(pid: int, marker: str) -> bool:
    """pid 还在、而且确实是那个子进程（照 cmdline 认，防 pid 复用）。"""
    try:
        cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().decode("utf-8", "replace")
    except OSError:
        return False
    return marker in cmdline


def _port_listening(port: int) -> bool:
    """这个端口有没有人在听（连上就断，不发数据）。"""
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.3):
            return True
    except OSError:
        return False


class TeleopWrapperLifecycleTest(unittest.IsolatedAsyncioTestCase):
    """启停"顺不顺滑"：上游是 bash → python 两层时，能不能起得来、收得干净、再起得来。"""

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.dir = Path(self._tmp.name)
        self.child = self.dir / "fake_teleop_child.py"
        self.child.write_text(FAKE_TELEOP_CHILD, encoding="utf-8")
        self.pidfile = self.dir / "child.pid"

    def spec(self, port: int) -> ProcessSpec:
        wrapper = self.dir / "run_fake_teleop.sh"
        wrapper.write_text(
            FAKE_TELEOP_WRAPPER.format(child=self.child, port=port, pidfile=self.pidfile),
            encoding="utf-8",
        )
        return ProcessSpec(name="teleop", argv=("bash", str(wrapper)))

    async def _wait(self, predicate, timeout: float = 5.0) -> bool:
        deadline = asyncio.get_running_loop().time() + timeout
        while asyncio.get_running_loop().time() < deadline:
            if predicate():
                return True
            await asyncio.sleep(0.05)
        return predicate()

    def _pid(self) -> int:
        """pidfile 里的 pid；文件还没写完/不存在时返回 0（写文件不是原子的，调用方先 _wait）。"""
        try:
            return int(self.pidfile.read_text(encoding="utf-8").strip() or 0)
        except (OSError, ValueError):
            return 0

    async def test_wrapper_tree_starts_reaps_and_restarts(self) -> None:
        port = free_port()
        async with running_bridge(teleop_port=port, processes={"teleop": self.spec(port)}) \
                as (bridge, _follow):
            # 起：bash 在跑，子进程也在，并且在 listen
            result = await asyncio.wait_for(ctl.ctl_up(bridge.cfg, 10.0), timeout=20.0)
            self.assertEqual(result.code, 0, "\n".join(result.lines))
            self.assertTrue(await self._wait(lambda: self.pidfile.exists()))
            pid = self._pid()
            self.assertTrue(_child_alive(pid, "fake_teleop_child"))
            # 子进程真的**在监听**（不只是"还活着"）。改造前这里看的是
            # teleop_connected —— 那是"桥连上了 teleop 的 WS 口"，而桥现在不连那个口
            # 了（帧走 zenoh，teleop 反过来订）。所以直接探端口。
            self.assertTrue(
                await self._wait(lambda: _port_listening(port)),
                f"假 teleop 的子进程没在 {port} 上监听",
            )

            # 停：连根收掉 —— 子进程不能留下
            result = await asyncio.wait_for(ctl.ctl_down(bridge.cfg, 10.0), timeout=20.0)
            self.assertEqual(result.code, 0, "\n".join(result.lines))
            self.assertTrue(
                await self._wait(lambda: not _child_alive(pid, "fake_teleop_child")),
                f"停完之后子进程还活着（pid={pid}）—— 孤儿会抱着 {port} 不放",
            )
            self.assertEqual(bridge.snapshot()["processes"]["teleop"], "stopped")

            # 再起：端口没被孤儿占住，所以第二次照样能起来
            result = await asyncio.wait_for(ctl.ctl_up(bridge.cfg, 10.0), timeout=20.0)
            self.assertEqual(result.code, 0, "\n".join(result.lines))
            self.assertTrue(
                await self._wait(lambda: 0 < self._pid() != pid),
                "重启之后 pidfile 还是旧的 —— 新子进程没起来？",
            )
            self.assertTrue(
                await self._wait(lambda: _port_listening(port)),
                "重启之后子进程没在监听",
            )

    async def test_crashed_upstream_can_be_started_again(self) -> None:
        """上游自己崩了：如实报 crashed（不自动重启），再 up 能拉起来。"""
        port = free_port()
        async with running_bridge(teleop_port=port, processes={"teleop": self.spec(port)}) \
                as (bridge, _follow):
            result = await asyncio.wait_for(ctl.ctl_up(bridge.cfg, 10.0), timeout=20.0)
            self.assertEqual(result.code, 0, "\n".join(result.lines))
            self.assertTrue(await self._wait(lambda: self._pid() > 1))

            os.kill(self._pid(), signal.SIGKILL)  # 模拟上游自己挂掉
            self.assertTrue(
                await self._wait(
                    lambda: bridge.snapshot()["processes"]["teleop"] == "crashed"
                ),
                f"没报 crashed：{bridge.snapshot()['processes']}",
            )

            result = await asyncio.wait_for(ctl.ctl_up(bridge.cfg, 10.0), timeout=20.0)
            self.assertEqual(result.code, 0, "\n".join(result.lines))
            self.assertEqual(bridge.snapshot()["processes"]["teleop"], "running")


class CtlTest(unittest.IsolatedAsyncioTestCase):
    async def test_status_prints_link_line(self) -> None:
        async with running_bridge() as (bridge, _follow):
            result = await ctl.ctl_status(bridge.cfg, 2.0)
            self.assertEqual(result.code, 0)
            self.assertIn("链路", "\n".join(result.lines))

    async def test_up_opens_forwarding_and_starts_follow(self) -> None:
        async with running_bridge() as (bridge, follow):
            result = await ctl.ctl_up(bridge.cfg, timeout_s=6.0)
            self.assertEqual(result.code, 0, "\n".join(result.lines))
            self.assertIn("start", follow.commands)

    async def test_down_latches_then_reports(self) -> None:
        async with running_bridge() as (bridge, follow):
            await ctl.ctl_up(bridge.cfg, timeout_s=6.0)
            result = await ctl.ctl_down(bridge.cfg, timeout_s=6.0)
            self.assertEqual(result.code, 0, "\n".join(result.lines))
            self.assertIn("latch", follow.commands)

    async def test_bridge_not_running_exits_2(self) -> None:
        cfg = Config(bridge_host="127.0.0.1", bridge_port=free_port())
        result = await ctl.ctl_status(cfg, 1.0)
        self.assertEqual(result.code, 2)
        self.assertIn("没连上桥", "\n".join(result.lines))

    async def test_start_waits_until_follow_server_listens(self) -> None:
        """回归：start 必须在 follow_server listen 之后才下发。"""
        port = free_port()
        with tempfile.TemporaryDirectory() as tmp:
            logfile = Path(tmp) / "follow.log"
            spec = ProcessSpec(
                name="follow_server",
                argv=(sys.executable, "-c", SLOW_FOLLOW, str(port), "1.0", str(logfile)),
            )
            async with running_bridge(follow_port=port, processes={"follow_server": spec}) \
                    as (bridge, _follow):
                result = await ctl.ctl_up(bridge.cfg, timeout_s=15.0)
                self.assertEqual(result.code, 0, "\n".join(result.lines))
                received = logfile.read_text(encoding="utf-8").split()
                self.assertIn("listening", received)
                self.assertIn("start", received)

    async def test_up_reports_failure_when_follow_server_dies(self) -> None:
        """跟随服务起来就崩：要给非 0 退出码，不能报成功。"""
        spec = ProcessSpec(
            name="follow_server",
            argv=(sys.executable, "-c", "raise SystemExit(3)"),
        )
        async with running_bridge(
            follow_port=free_port(), processes={"follow_server": spec}
        ) as (bridge, _follow):
            result = await ctl.ctl_up(bridge.cfg, timeout_s=8.0)
            self.assertEqual(result.code, 1, "\n".join(result.lines))


if __name__ == "__main__":
    unittest.main()

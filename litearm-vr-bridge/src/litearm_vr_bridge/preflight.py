"""启动前体检（`--doctor`）。

现场最贵的故障不是"起不来"，而是"起来了但不知道为什么不动"：端口被别的程序占了、
submodule 忘了 init、conda env 路径不对、串口没插、`arm_console.py` 找不到、上游
启动脚本换了参数名（我们传的它不认）、系统服务里没有 DISPLAY 所以 pybullet
窗口起不来。这些都能在启动前问清楚。

上游从单仓（`VR_REPO`）换成 submodule 之后，预检的参照物也跟着变了：

* **进程怎么起**交给上游启动脚本，所以"参数对拍"看的是那个脚本认不认我们传的
  `--flag`：teleop 对的是 launcher 自己的 argparse，follow 对的是它最终跑到的
  那个 `.py`（命令行由桥拼、参数透传）。
* 上游版本被 submodule pin 住了，"现场那份上游比本服务假设的老"这个故障模式结构性
  消失，取而代之的是"现场把 submodule 就地改了" —— 所以多了 submodule 版本/脏污检查。
* 解释器与 `arm_console` 由上游脚本自己解析，桥不再拼这两样。但**检查项还在**：
  按上游同一套顺序去*验证*（`_resolve_teleop_python` / `_check_follow_tools`），
  查得到就报，查不到就提示怎么指路。检查不参与启动，启动永远走上游那一条路。

写法与 `governor.py` / `metrics.py` 同类：**判定逻辑不直接碰外部世界**，外部世界通过
`Probes` 注入。这样单测能造一个临时仓库布局、假串口、假端口，不需要真机。

判定结果只有四种：ok / warn / fail / info —— `fail` 才拦启动（见 `has_failures`），
`warn` 是"你确认就行"。
"""

from __future__ import annotations

import json
import os
import re
import shutil
import socket
import subprocess
import sys
from collections import Counter
from dataclasses import dataclass
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Callable

from .config import (
    KIND_CUSTOM,
    KIND_FOLLOW_SDK,
    KIND_FOLLOW_STM32,
    KIND_TELEOP,
    SUBMODULES,
    Config,
    ProcessSpec,
)

__all__ = [
    "Check",
    "Probes",
    "SubmoduleState",
    "run_checks",
    "format_report",
    "has_failures",
    "OK",
    "WARN",
    "FAIL",
    "INFO",
]

OK = "ok"
WARN = "warn"
FAIL = "fail"
INFO = "info"

# 排序用：先报坏的
_LEVEL = {FAIL: 0, WARN: 1, OK: 2, INFO: 3}
_LABEL = {OK: "ok  ", WARN: "WARN", FAIL: "FAIL", INFO: "info"}
_PREFIX = 7  # "[WARN] "

# `websockets.asyncio.*` / `websockets.http11` 是 13.x 起才有的 API
MIN_WEBSOCKETS = 13

_WILDCARD_HOSTS = ("0.0.0.0", "::", "", "*")
_NAME_WIDTH = 18

# 上游遥操脚本探测 conda 根的候选（照它自己的顺序）
_CONDA_ROOTS = ("~/sl/miniconda3", "~/miniconda3", "~/opt/anaconda3", "~/anaconda3")
_METAVR_ENV = "metavr-arm"

_SUBMODULE_INIT_HINT = (
    "git submodule update --init  （clone 时漏了 --recursive 也会这样）；"
    "不是 git 检出的话，把两个仓库的文件放到 submodules/ 下即可"
)


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str
    hint: str | None = None

    @property
    def level(self) -> int:
        return _LEVEL.get(self.status, 3)


@dataclass(frozen=True)
class SubmoduleState:
    """一个 submodule 的现场状态。

    两个问题分开问：`present` 只问"文件在不在"（桥能不能跑取决于这个），`tracked`
    问"是不是 git 检出"（版本核对与脏污检查需要它）。tarball / rsync 部署没有 `.git`，
    文件却是全的 —— 那是合法的部署方式，不该拦启动。
    """

    present: bool
    tracked: bool = False
    sha: str | None = None
    dirty: bool = False


# ── 外部世界（单测里替换） ──────────────────────────────────────────────────


@dataclass
class Probes:
    """预检要碰的外部世界。默认是真实现，单测注入假的。"""

    bind_ok: Callable[[str, int], bool] = lambda h, p: _bind_ok(h, p)
    connect_ok: Callable[[str, int], bool] = lambda h, p: _connect_ok(h, p)
    health: Callable[[str, int], dict | None] = lambda h, p: _health_json(h, p)
    import_probe: Callable[[str, tuple[str, ...], float], tuple[bool, str]] = (
        lambda exe, mods, t: _import_probe(exe, mods, t)
    )
    lan_ip: Callable[[], str | None] = lambda: _lan_ip()
    which: Callable[[str], str | None] = staticmethod(shutil.which)  # type: ignore[assignment]
    conda_base: Callable[[], str | None] = lambda: _conda_base()
    submodule_state: Callable[[Path], SubmoduleState] = lambda p: _submodule_state(p)


def _bind_ok(host: str, port: int) -> bool:
    """端口能不能拿来监听。

    带 SO_REUSEADDR，和 `websockets.serve` 保持一致：否则刚停掉的桥留下的
    TIME_WAIT 会让探测误报"被占用"，而实际上服务是能起来的。这不会漏掉真冲突 ——
    SO_REUSEADDR 让不开一个正在 LISTEN 的 socket（那需要 SO_REUSEPORT）。
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((host, port))
        return True
    except OSError:
        return False


def _connect_ok(host: str, port: int, timeout: float = 0.3) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _health_json(host: str, port: int, timeout: float = 1.0) -> dict | None:
    """直接发一条 HTTP/1.0 GET，不依赖 urllib 的重定向/异常链。"""
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            sock.sendall(
                f"GET /healthz HTTP/1.0\r\nHost: {host}\r\nConnection: close\r\n\r\n".encode()
            )
            chunks: list[bytes] = []
            while True:
                try:
                    data = sock.recv(4096)
                except (OSError, socket.timeout):
                    break
                if not data:
                    break
                chunks.append(data)
    except OSError:
        return None
    head, _, body = b"".join(chunks).partition(b"\r\n\r\n")
    if b" 200 " not in head.split(b"\r\n", 1)[0]:
        return None
    try:
        payload = json.loads(body.decode("utf-8", "replace"))
    except (ValueError, TypeError):
        return None
    return payload if isinstance(payload, dict) else None


def _import_probe(exe: str, modules: tuple[str, ...], timeout_s: float) -> tuple[bool, str]:
    code = "import " + ", ".join(modules)
    try:
        proc = subprocess.run(
            [exe, "-c", code],
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f"{type(exc).__name__}: {exc}"
    if proc.returncode == 0:
        return True, "依赖齐备（" + ", ".join(modules) + "）"
    tail = (proc.stderr or proc.stdout or "").strip().splitlines()
    return False, (tail[-1] if tail else f"退出码 {proc.returncode}")


def _lan_ip() -> str | None:
    """取默认路由的出口 IP —— 头显要填的就是它。

    连一个 UDP 地址不会真的发包，只是为了问内核"你走哪张网卡出去"。
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(0.5)
            sock.connect(("8.8.8.8", 80))
            ip = str(sock.getsockname()[0])
    except OSError:
        return None
    return ip if ip and not ip.startswith("127.") else None


def _conda_base() -> str | None:
    """问 conda 自己的根在哪（上游脚本也是这么找 metavr-arm 的）。"""
    exe = shutil.which("conda")
    if exe is None:
        return None
    try:
        proc = subprocess.run(
            [exe, "info", "--base"], capture_output=True, text=True, timeout=5.0
        )
    except (OSError, subprocess.SubprocessError):
        return None
    base = (proc.stdout or "").strip()
    return base if proc.returncode == 0 and base else None


def _submodule_state(path: Path) -> SubmoduleState:
    """submodule 在不在、是哪个 commit、有没有被就地改过。

    "在不在"看**目录里有没有东西**，不看 `.git`：没 init 的 submodule 是个空目录，
    而拷目录 / 打包部署的那份没有 `.git`、文件却齐全，一样能跑 —— 拿 `.git` 当判据
    会把后者误判成"没 init"从而拦住启动。有没有 git 只决定"版本核不核得了"。
    """
    try:
        if not path.is_dir() or next(path.iterdir(), None) is None:
            return SubmoduleState(present=False)
    except OSError:  # 权限之类，当成读不出来处理，别让预检崩
        return SubmoduleState(present=False)

    if not (path / ".git").exists():
        return SubmoduleState(present=True)

    sha = _git(path, "rev-parse", "--short", "HEAD")
    status = _git(path, "status", "--porcelain")
    if sha is None or status is None:  # .git 在但 git 读不出来（坏了 / 没有 git）
        return SubmoduleState(present=True, tracked=True, sha=sha)
    return SubmoduleState(present=True, tracked=True, sha=sha, dirty=bool(status.strip()))


def _git(path: Path, *args: str) -> str | None:
    try:
        proc = subprocess.run(
            ["git", "-C", str(path), *args],
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if proc.returncode != 0:
        return None
    return proc.stdout.strip()


# ── 对外入口 ────────────────────────────────────────────────────────────────


def run_checks(
    cfg: Config,
    *,
    probes: Probes | None = None,
    env: dict[str, str] | None = None,
) -> list[Check]:
    world = probes if probes is not None else Probes()
    environ = os.environ if env is None else env

    checks = [
        _check_python(),
        _check_websockets(),
        _check_zenoh(),
        _check_zenoh_port(cfg, world),
        _check_legacy_env(cfg),
        _check_submodules(cfg, world),
        _check_bridge_port(cfg, world),
        _check_link_targets(cfg, world),
    ]
    for name, spec in cfg.processes.items():
        checks.append(_check_process_files(name, spec, world))
        checks.append(_check_process_contract(name, spec))
    checks.append(_check_hardware(cfg))
    checks.append(_check_follow_tools(cfg, environ))
    checks.append(_check_teleop_python(cfg, world, environ))
    checks.append(_check_display(cfg, environ))
    checks.append(_check_lan_addr(cfg, world))
    checks.append(_check_real_arm(cfg))
    return checks


def _check_legacy_env(cfg: Config) -> Check:
    """配置里还有没有 `METAVR_*` 这种旧名字。

    WARN 而不是 FAIL：桥认旧名字（见 config.LEGACY_ENV_PREFIX），所以**现在能用**；
    但其中两个键的语义是"不要这样做"（`METAVR_CMD=` 留空、`METAVR_ARM_HOST=` 留空），
    留着旧名字等于把"我明确不要"这件事寄托在一个将来会删的兼容层上。
    """
    if not cfg.legacy_env:
        return Check("legacy_env", OK, "配置里没有旧变量名")
    return Check(
        "legacy_env", WARN, "还在用旧变量名：" + "、".join(cfg.legacy_env),
        "改成同名的 TELEOP_*（如 METAVR_CMD → TELEOP_CMD、METAVR_PY → "
        "TELEOP_SERVER_PY）。桥暂时仍认旧名字，但以后会删",
    )


def has_failures(checks: list[Check]) -> bool:
    return any(check.status == FAIL for check in checks)


def format_report(checks: list[Check]) -> str:
    counts = Counter(check.status for check in checks)
    lines = [
        "预检：{} 项（fail {} / warn {} / ok {} / info {}）".format(
            len(checks), counts[FAIL], counts[WARN], counts[OK], counts[INFO]
        ),
        "",
    ]
    indent = " " * (_PREFIX + _NAME_WIDTH)
    for check in sorted(checks, key=lambda c: c.level):
        lines.append(
            f"[{_LABEL.get(check.status, check.status)}] "
            f"{check.name:<{_NAME_WIDTH}} {check.detail}"
        )
        if check.hint:
            lines.append(f"{indent}→ {check.hint}")
    lines.append("")

    if counts[FAIL]:
        lines.append(f"结论：有 {counts[FAIL]} 项必须修好才能启动。")
    elif counts[WARN]:
        lines.append(f"结论：没有致命问题，但有 {counts[WARN]} 项警告，确认后可以启动。")
    else:
        lines.append("结论：全部通过。")
    return "\n".join(lines)


# ── 逐项检查 ────────────────────────────────────────────────────────────────


def _check_python() -> Check:
    major, minor = sys.version_info[:2]
    ok = (major, minor) >= (3, 10)
    detail = f"{sys.version.split()[0]}（{sys.executable}）"
    if ok:
        return Check("python", OK, detail)
    return Check("python", FAIL, detail, "本服务需要 Python 3.10+")


def _check_websockets() -> Check:
    try:
        raw = version("websockets")
    except PackageNotFoundError:
        return Check(
            "websockets", FAIL, "未安装", 'python3 -m pip install "websockets>=13"'
        )
    except Exception as exc:  # noqa: BLE001 — 元数据坏了不该让预检崩
        return Check("websockets", WARN, f"版本读不出来：{exc}")

    match = re.match(r"(\d+)", raw or "")
    major = int(match.group(1)) if match else 0
    if major >= MIN_WEBSOCKETS:
        return Check("websockets", OK, raw)
    return Check(
        "websockets",
        FAIL,
        f"{raw} 太老（需要 >={MIN_WEBSOCKETS}）",
        "本服务用的是 websockets.asyncio.* / http11，13.x 起才有",
    )


def _check_zenoh() -> Check:
    """我们的对外接口现在全在 zenoh 上，没装它这个服务等于没人看得见。"""
    try:
        raw = version("eclipse-zenoh")
    except PackageNotFoundError:
        return Check(
            "zenoh", FAIL, "未安装",
            'python3 -m pip install "eclipse-zenoh>=1.0"',
        )
    except Exception as exc:  # noqa: BLE001 — 元数据坏了不该让预检崩
        return Check("zenoh", WARN, f"版本读不出来：{exc}")
    try:
        import zenoh  # noqa: F401
    except Exception as exc:  # noqa: BLE001
        return Check("zenoh", FAIL, f"{raw} 装了但 import 不了：{exc}")
    return Check("zenoh", OK, raw)


def _check_zenoh_port(cfg: Config, world: Probes) -> Check:
    """桥要 listen 的那个端点绑不绑得上。

    **这条防的是一个静默故障**：zenoh 绑不上监听时只在日志里说一句，会话照样开着、
    发布者照样声明 —— 于是"桥在跑、但谁也连不进来"。同机最容易撞的是
    `litearm-server`（它默认也监听 `tcp/127.0.0.1:7447`）。
    """
    endpoints = cfg.zenoh_listen_endpoints
    if not endpoints:
        return Check("zenoh_port", INFO, "没配监听端点（只做 client），跳过")
    host, port = _parse_tcp_endpoint(endpoints[0])
    if host is None:
        return Check("zenoh_port", INFO, f"认不出端点 {endpoints[0]!r}，跳过")
    probe_host = "127.0.0.1" if host in ("0.0.0.0", "::", "", "*") else host
    if world.bind_ok(probe_host, port):
        return Check("zenoh_port", OK, f"{host}:{port} 可用")
    return Check(
        "zenoh_port",
        FAIL,
        f"{host}:{port} 绑不上（多半被占了）",
        "`ss -tlnp | grep %d` 看是谁。⚠ litearm-server 默认也监听 tcp/127.0.0.1:7447 —— "
        "同机部署时把 ZENOH_LISTEN_ENDPOINTS 改到别的端口" % port,
    )


def _parse_tcp_endpoint(endpoint: str) -> tuple[str | None, int]:
    """`tcp/0.0.0.0:7447` → `("0.0.0.0", 7447)`；认不出来返回 (None, 0)。"""
    proto, _, addr = endpoint.partition("/")
    if proto not in ("tcp", "ws", "udp"):
        return None, 0
    host, _, port_s = addr.rpartition(":")
    try:
        return host, int(port_s)
    except ValueError:
        return None, 0


def _check_submodules(cfg: Config, world: Probes) -> Check:
    """上游 submodule 在不在、是哪个 commit、有没有被就地改过。

    pin 住之后，"上游比桥假设的老"这个故障模式结构性消失，剩下要防的是两件事：
    忘了 init（桥会没有任何受管进程），以及现场有人直接在 submodule 里改代码
    （`git submodule update` 一跑就没了，改动得回它自己的仓库做）。
    """
    root = cfg.repo_root
    if root is None:
        return Check(
            "submodules", WARN, "推导不出仓库根（不是从源码跑的？）",
            "export BRIDGE_ROOT=<桥仓库根>；或用 TELEOP_CMD / FOLLOW_SERVER_CMD 自己写命令",
        )

    missing: list[str] = []
    untracked: list[str] = []
    dirty: list[str] = []
    versions: list[str] = []
    for rel in SUBMODULES:
        state = world.submodule_state(root / rel)
        name = Path(rel).name
        if not state.present:
            missing.append(rel)
            continue
        if not state.tracked:
            untracked.append(name)
            continue
        versions.append(f"{name}@{state.sha or '?'}")
        if state.dirty:
            dirty.append(name)

    if missing:
        return Check(
            "submodules", FAIL, "没 init：" + "、".join(missing), _SUBMODULE_INIT_HINT
        )

    detail = "；".join(
        part for part in (
            "、".join(versions),
            "不是 git 检出（跳过版本核对）：" + "、".join(untracked) if untracked else "",
        ) if part
    )
    if dirty:
        return Check(
            "submodules", WARN, f"{detail}；{'、'.join(dirty)} 被就地改过",
            "上游的改动要走它自己的仓库；这里的改动下次 submodule update 就没了",
        )
    if untracked:
        return Check(
            "submodules", INFO, detail,
            "文件齐就能跑；但这样核对不了上游版本、也看不出有没有被改过",
        )
    return Check("submodules", OK, detail)


def _check_bridge_port(cfg: Config, world: Probes) -> Check:
    host, port = cfg.bridge_host, cfg.bridge_port
    if world.bind_ok(host, port):
        return Check("port_bridge", OK, f"{host}:{port} 可用")

    health = world.health(_client_host(host), port)
    if _looks_like_bridge(health):
        return Check(
            "port_bridge", OK,
            f"{host}:{port} 上是本服务（v{health.get('version')}）—— 已在运行",
        )
    return Check(
        "port_bridge", FAIL, f"{host}:{port} 被别的程序占用",
        f"查占用：ss -tlnp | grep {port}；或改 BRIDGE_PORT",
    )


def _check_link_targets(cfg: Config, world: Probes) -> Check:
    parts: list[str] = []
    missing: list[str] = []
    for label, host, port in (
        ("teleop", cfg.teleop_host, cfg.teleop_port),
        ("follow_server", cfg.follow_host, cfg.follow_port),
    ):
        alive = world.connect_ok(host, port)
        parts.append(f"{label} {'在监听' if alive else '未监听'}")
        if not alive:
            missing.append(label)
    detail = f"{cfg.teleop_endpoint} / {cfg.follow_endpoint}：" + "，".join(parts)
    if not missing:
        return Check("link_targets", OK, detail)
    return Check(
        "link_targets", INFO, detail,
        "未监听不算错：桥托管时会在 teleop.start 拉起；不托管请先自己启动它们",
    )


def _check_process_files(name: str, spec: ProcessSpec, world: Probes) -> Check:
    problems: list[str] = []
    argv0 = spec.argv[0] if spec.argv else ""
    if not argv0:
        problems.append("命令为空")
    elif "/" in argv0:
        path = os.path.expanduser(argv0)
        if not os.path.isfile(path):
            problems.append(f"解释器不存在：{path}")
        elif not os.access(path, os.X_OK):
            problems.append(f"解释器不可执行：{path}")
    elif world.which(argv0) is None:
        problems.append(f"PATH 里找不到 {argv0}")

    # 上游启动脚本 + 它最终会跑到的那个 .py，两个都得在：只看前者的话，
    # submodule 只 init 了一半（缺文件）也能"通过"，等到真起进程才炸
    # follow 的启动入口是 `.sh`，teleop 的是 `.py`（2026-10-15 起）
    launcher = _launcher_of(spec) or _launcher_py_of(spec)
    for label, target in (("启动脚本", launcher), ("上游脚本", _target_path(spec))):
        if target and not os.path.isfile(target):
            problems.append(f"{label}不存在：{target}")
    if spec.cwd and not os.path.isdir(os.path.expanduser(spec.cwd)):
        problems.append(f"工作目录不存在：{spec.cwd}")

    label = f"process:{name}"
    if problems:
        missing_upstream = any("submodules/" in p for p in problems)
        hint = _SUBMODULE_INIT_HINT if missing_upstream else f"检查 {name.upper()}_CMD / BRIDGE_ROOT"
        return Check(label, FAIL, "；".join(problems), hint)
    return Check(label, OK, spec.label())


def _check_process_contract(name: str, spec: ProcessSpec) -> Check:
    """对拍：我们让它按什么跑，它认不认。

    上游被 submodule pin 住之后，这条防的不再是"上游比自己老"，而是"上游换了接口"
    （改了参数名）和"现场手改过 submodule"。两种命令对的是两个不同的文件：

    * teleop：接口在**上游 launcher 的 argparse** 上（2026-10-15 起命令行取代了
      环境变量），所以读 launcher 本身；
    * follow：命令行由桥拼、参数透传给上游脚本，所以读它最终会跑到的那个 `.py`。
    """
    if spec.kind == KIND_TELEOP:
        # teleop 的接口在**上游 launcher 的 argparse** 上（2026-10-15 起命令行取代了
        # 环境变量）。所以要读 launcher 本身，而不是它背后跑的 ws_echo_server.py ——
        # 后者是 launcher 调起来的，我们一个参数都不直接传给它。
        return _check_flag_contract(name, spec, script=_launcher_py_of(spec))
    return _check_flag_contract(name, spec)


def _launcher_py_of(spec: ProcessSpec) -> str | None:
    """argv 里那个上游 launcher（`.py`，teleop 用；follow 那边是 `.sh`）。

    跳过 argv[0]（解释器），免得把 `python3` 这种当成 launcher。
    """
    for arg in spec.argv[1:]:
        if arg.endswith(".py"):
            return os.path.expanduser(arg)
    return None


def _check_flag_contract(name: str, spec: ProcessSpec, *, script: str | None = None) -> Check:
    label = f"flags:{name}"
    script = script or _target_path(spec)
    if not script:
        return Check(label, INFO, "自定义命令里没有 .py 脚本，跳过参数对拍")
    if not os.path.isfile(script):
        return Check(label, INFO, f"脚本不存在（{script}），跳过参数对拍")

    try:
        source = Path(script).read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        return Check(label, INFO, f"读不了脚本：{exc}")

    declared = _declared_flags(source)
    if not declared:
        return Check(label, INFO, "脚本里没抓到 add_argument，跳过参数对拍")

    # add_argument 未必是字面量（`for f in (...): p.add_argument(f)` 这种抓不到），
    # 所以"这个参数存不存在"退一步看整个文件里有没有出现过这个字符串 —— 只要出现过
    # 就不报警，避免误报；接口改过的文件里连字符串都没了，照样抓得住。
    mentioned = set(_FLAG_LITERAL_RE.findall(source))

    unknown: list[str] = []
    bad_choice: list[str] = []
    argv = list(spec.argv)
    for index, token in enumerate(argv):
        if not token.startswith("--"):
            continue
        flag = token.split("=", 1)[0]
        if flag not in declared:
            if flag not in mentioned:
                unknown.append(flag)
            continue
        choices = declared[flag]
        value = token.split("=", 1)[1] if "=" in token else (
            argv[index + 1] if index + 1 < len(argv) else None
        )
        if choices and value is not None and not value.startswith("--") and value not in choices:
            bad_choice.append(f"{flag}={value}（只认 {'/'.join(choices)}）")

    if unknown or bad_choice:
        detail = []
        if unknown:
            detail.append("脚本不认这些参数：" + " ".join(unknown))
        if bad_choice:
            detail.append("取值不在脚本的 choices 里：" + "，".join(bad_choice))
        return Check(
            label, WARN, "；".join(detail),
            f"{script} 可能被就地改过（上游换了接口）；核对 submodule 里的这份",
        )
    passed = sorted({token.split("=", 1)[0] for token in argv if token.startswith("--")})
    return Check(label, OK, f"我们传的 {len(passed)} 个参数，脚本都认（{' '.join(passed)}）")


def _check_hardware(cfg: Config) -> Check:
    spec = cfg.processes.get("follow_server")
    if spec is None:
        return Check("hardware", INFO, "未托管 follow_server，跳过串口/CAN 检查")
    argv = list(spec.argv)
    variant = _follow_variant(spec)

    if variant == "sdk":
        iface = _value_of(argv, "--iface") or "can0"
        sysfs = Path("/sys/class/net") / iface
        if not sysfs.exists():
            return Check(
                "hardware", FAIL, f"CAN 网卡 {iface} 不存在",
                f"接上 CAN 盒后：sudo ip link set {iface} up type can bitrate 1000000",
            )
        try:
            state = (sysfs / "operstate").read_text(encoding="utf-8").strip()
        except OSError as exc:
            return Check("hardware", WARN, f"CAN 网卡 {iface} 状态读不出来：{exc}")
        if state == "up":
            return Check("hardware", OK, f"CAN {iface} 已 up")
        return Check("hardware", FAIL, f"CAN {iface} 存在但状态是 {state}",
                     f"sudo ip link set {iface} up type can bitrate 1000000")

    if variant == "stm32":
        device = _value_of(argv, "--port") or "/dev/ttyACM0"
        if not os.path.exists(device):
            return Check(
                "hardware", FAIL, f"串口 {device} 不存在",
                "检查 STM32 是否上电、USB 是否插好；或改 FOLLOW_SERVER_SERIAL",
            )
        if not os.access(device, os.R_OK | os.W_OK):
            return Check("hardware", FAIL, f"串口 {device} 不可读写",
                         "把运行用户加进 dialout 组：sudo usermod -aG dialout $USER")
        return Check("hardware", OK, f"串口 {device} 可读写")

    return Check("hardware", INFO, "跟随服务是自定义命令（认不出哪一版），跳过硬件检查")


def _check_follow_tools(cfg: Config, env: dict[str, str]) -> Check:
    """STM32 版 follow_server 要 import `arm_console.py`（litearm-stm32/tools）。

    桥不再自己拼 PYTHONPATH（那是上游 `run_stm32_server.sh` 的事），这里只是**按
    上游同一套顺序去验证**它找不找得到 —— 只在预检里用，不参与启动：
      1) 环境里的 `LITEARM_TOOLS`
      2) follow 仓库同级的 `litearm-stm32/tools`
      3) `~/sl/litearm-stm32/tools`
    """
    spec = cfg.processes.get("follow_server")
    if spec is None:
        return Check("follow_tools", INFO, "未托管 follow_server，跳过")
    if _follow_variant(spec) != "stm32":
        return Check("follow_tools", INFO, "非 stm32 版跟随服务，不需要 arm_console")

    candidates: list[Path] = []
    explicit = (env.get("LITEARM_TOOLS") or "").strip()
    if explicit:
        candidates.append(Path(os.path.expanduser(explicit)))
    launcher = _launcher_of(spec)
    if launcher:
        candidates.append(Path(launcher).parent.parent / "litearm-stm32" / "tools")
    candidates.append(Path(os.path.expanduser("~/sl/litearm-stm32/tools")))

    for candidate in candidates:
        if (candidate / "arm_console.py").is_file():
            return Check("follow_tools", OK, f"arm_console.py 在 {candidate}")
    return Check(
        "follow_tools", FAIL,
        "这几处都没有 arm_console.py：" + "、".join(str(c) for c in candidates),
        "设 LITEARM_TOOLS=<litearm-stm32/tools 的路径>（上游启动脚本认这个变量）",
    )


def _check_teleop_python(cfg: Config, world: Probes, env: dict[str, str]) -> Check:
    spec = cfg.processes.get("teleop")
    if spec is None:
        return Check("teleop_python", INFO, "未托管 teleop，跳过依赖检查")

    if spec.kind != KIND_TELEOP:
        # 自定义命令：解释器就是我们写在 argv[0] 的那个
        exe = os.path.expanduser(spec.argv[0]) if spec.argv else ""
        if not os.path.isfile(exe):
            return Check("teleop_python", INFO,
                         f"解释器 {exe} 不存在，跳过 import 检查（见 process:teleop）")
    else:
        exe = _resolve_teleop_python(env, world) or ""
        if not exe:
            return Check(
                "teleop_python", WARN, "找不到 metavr-arm 环境的 python",
                "上游启动脚本也会找同一个：export TELEOP_SERVER_PY=<envs/metavr-arm/bin/python>",
            )

    ok, detail = world.import_probe(exe, ("pybullet", "numpy"), 10.0)
    if ok:
        return Check("teleop_python", OK, f"{exe}：{detail}")
    return Check(
        "teleop_python", WARN, f"{exe}：{detail}",
        "teleop 依赖 pybullet/numpy；先跑上游 environments/setup.sh 建环境，"
        "或用 TELEOP_SERVER_PY 指到装齐了的那份",
    )


def _resolve_teleop_python(env: dict[str, str], world: Probes) -> str | None:
    """按上游 launcher 的同一套顺序找 metavr-arm 的解释器。

    **只用于检查**：真正起进程时是上游 launcher 自己解析的（`teleop.yaml` 的
    `env.metavr_py`，或命令行 `--metavr-py`，再不然自动探）。顺序照抄上游，是为了
    让"检查的那个解释器"和"实际上跑的那个"是同一个人。

      1) `TELEOP_SERVER_PY`
      2) 已激活的同名 conda 环境（`CONDA_PREFIX`）
      3) `conda info --base` 下面的 envs/metavr-arm
      4) 常见 conda 根
    """
    explicit = (env.get("TELEOP_SERVER_PY") or "").strip()
    if explicit:
        return os.path.expanduser(explicit)

    prefix = (env.get("CONDA_PREFIX") or "").strip()
    if prefix and Path(prefix).name == _METAVR_ENV:
        exe = Path(prefix) / "bin" / "python"
        if exe.is_file():
            return str(exe)

    roots: list[str] = []
    base = world.conda_base()
    if base:
        roots.append(base)
    roots.extend(os.path.expanduser(p) for p in _CONDA_ROOTS)
    for root in roots:
        exe = Path(root) / "envs" / _METAVR_ENV / "bin" / "python"
        if exe.is_file():
            return str(exe)
    return None


def _check_display(cfg: Config, env: dict[str, str]) -> Check:
    spec = cfg.processes.get("teleop")
    if spec is None:
        return Check("display", INFO, "未托管 teleop，跳过 DISPLAY 检查")
    if _viewer_of(spec) != "gui":
        return Check("display", INFO, "teleop 不用 pybullet 窗口，跳过 DISPLAY 检查")
    display = (env.get("DISPLAY") or "").strip()
    wayland = (env.get("WAYLAND_DISPLAY") or "").strip()
    if display or wayland:
        return Check("display", OK, f"DISPLAY={display or wayland}")
    return Check(
        "display", WARN, "没有 DISPLAY：teleop 的 pybullet 窗口起不来",
        "系统服务请补 Environment=DISPLAY=:0 与 XAUTHORITY=，或改用 install.sh --user",
    )


def _check_lan_addr(cfg: Config, world: Probes) -> Check:
    ip = world.lan_ip()
    if not ip:
        return Check("lan_addr", WARN, "没找到非 loopback 的 IPv4 地址",
                     "头显要连本机 LAN 地址，确认网卡已连接")
    return Check("lan_addr", INFO, f"头显面板填 ws://{ip}:{cfg.bridge_port}")


def _check_real_arm(cfg: Config) -> Check:
    spec = cfg.processes.get("teleop")
    if spec is None:
        return Check("real_arm", INFO, "未托管 teleop，未知是否接真机")
    if not cfg.real_arm:
        return Check("real_arm", INFO, "teleop 只跑仿真，不会碰真机")
    target = _value_of(list(spec.argv), "--real-right") or cfg.follow_endpoint
    return Check(
        "real_arm", WARN, f"teleop 接真机：--real → {target}",
        "首次按 grip 会 movej 回 q_home 并跟随真机；确认已上电、工作区无人、急停在手",
    )


# ── 小工具 ──────────────────────────────────────────────────────────────────


def _client_host(host: str) -> str:
    """监听地址可能是 0.0.0.0，客户端要连 loopback。"""
    return "127.0.0.1" if host in _WILDCARD_HOSTS else host


def _looks_like_bridge(payload: dict | None) -> bool:
    if not isinstance(payload, dict):
        return False
    return "version" in payload and "headset_online" in payload


def _launcher_of(spec: ProcessSpec) -> str | None:
    """argv 里那个上游启动脚本（`*.sh`）。"""
    for arg in spec.argv:
        if arg.endswith(".sh"):
            return os.path.expanduser(arg)
    return None


def _target_path(spec: ProcessSpec) -> str | None:
    """这条命令最终会跑到的那个 .py。

    默认命令走 `bash run_xxx.sh`，光看 argv 只知道包了一层，所以配置里显式带了
    `probe_script`；`*_CMD` 自定义命令就从 argv 里认。
    """
    if spec.probe_script:
        return os.path.expanduser(spec.probe_script)
    for arg in spec.argv:
        if arg.endswith(".py"):
            return os.path.expanduser(arg)
    return None


def _follow_variant(spec: ProcessSpec) -> str | None:
    """这条 follow_server 命令是 stm32 版还是 sdk 版（自定义命令按文件名认）。"""
    if spec.kind == KIND_FOLLOW_STM32:
        return "stm32"
    if spec.kind == KIND_FOLLOW_SDK:
        return "sdk"
    if spec.kind != KIND_CUSTOM:
        return None
    for arg in spec.argv:
        if "arm_follow_server_stm32.py" in arg:
            return "stm32"
        if "arm_follow_server_sdk.py" in arg:
            return "sdk"
    return None


def _viewer_of(spec: ProcessSpec) -> str | None:
    """teleop 用哪个显示后端 —— 2026-10-15 起是命令行参数，不再走环境变量。"""
    return _value_of(list(spec.argv), "--viewer")


def _value_of(argv: list[str], flag: str) -> str | None:
    for index, token in enumerate(argv):
        if token == flag and index + 1 < len(argv):
            return argv[index + 1]
        if token.startswith(flag + "="):
            return token.split("=", 1)[1]
    return None


_DECL_RE = re.compile(r"add_argument\(\s*[\"'](--[A-Za-z0-9_-]+)[\"']([^)]*)")
_FLAG_LITERAL_RE = re.compile(r"[\"'](--[A-Za-z0-9_-]+)[\"']")
_CHOICES_RE = re.compile(r"choices\s*=\s*\[([^\]]*)\]")
_QUOTED_RE = re.compile(r"[\"']([^\"']+)[\"']")


def _declared_flags(source: str) -> dict[str, list[str]]:
    """脚本里声明的 `--flag` → 它的 choices（没有就是空列表）。"""
    declared: dict[str, list[str]] = {}
    for flag, rest in _DECL_RE.findall(source):
        match = _CHOICES_RE.search(rest)
        declared[flag] = _QUOTED_RE.findall(match.group(1)) if match else []
    return declared



"""配置：环境变量为底，命令行可覆盖。

命名与默认值见 docs/DESIGN.md 第 7 节。

两件事在这里定死，其余都从环境变量来：

* **上游在哪** —— 受管的上游是两个 submodule（teleop / follow），路径从 `__file__`
  推出来（`repo_root()`）。布局是仓库的一部分，不是"这台机器的特性"，所以默认值
  不进 env 文件；上游确实放在 submodule 之外时才用 `TELEOP_REPO` / `FOLLOW_REPO`
  指路，那是部署特性。
* **上游怎么起** —— 交给上游自己的启动脚本（teleop 的
  `launch/run_teleop_webxr.py`、follow 的 `run_*_server.sh`）。conda 环境、
  `arm_console`、ROS2、灵巧手 SDK 这些知识在上游那边只有一份，桥复制一份就是
  "手工能起、服务起不来"这类现场故障的源头。
  桥只管这台机器的特性：监听在哪、串口/CAN 口、正装还是侧装、接不接真机。
"""

from __future__ import annotations

import os
import shlex
from dataclasses import dataclass, field, replace
from pathlib import Path

from .protocol import DEFAULT_ARM_ID

__all__ = [
    "Config",
    "ProcessSpec",
    "LEGACY_STATUS_PATHS",
    "SUBMODULES",
    "TELEOP_DIR",
    "FOLLOW_DIR",
    "KIND_TELEOP",
    "KIND_FOLLOW_STM32",
    "KIND_FOLLOW_SDK",
    "KIND_CUSTOM",
    "repo_root",
    "parse_endpoint",
    "format_endpoint",
    "LEGACY_ENV_PREFIX",
]

# ── 环境变量的旧名字（metavr → teleop 改名留下的） ──────────────────────────
#
# 2026-10 之前桥管上游叫 metavr，现场那份 /etc/litearm-vr-bridge.env 里写的是
# `METAVR_*`。改名之后那些键会被**静默忽略**，而其中两个的"缺席"是会出事的：
#
#   * `METAVR_CMD=`（显式留空）的语义是"这个进程我不要"（见 `_build_processes`）；
#     忽略掉就回落到默认命令，把一个本该不托管 teleop 的真机现场把它拉起来。
#   * `METAVR_ARM_HOST=`（显式留空）的语义是"只跑仿真"；忽略掉就默认接真机。
#
# 所以旧名字**继续认**，并且记在 `Config.legacy_env` 里，由启动日志和 `--doctor`
# 提示迁移。新名字优先；两个都在时不告警（那是迁移途中的正常状态）。
LEGACY_ENV_PREFIX = "METAVR_"

# 前缀换了、但尾巴也变了的特例（不是简单的前缀替换）
_LEGACY_ALIASES = {"METAVR_PY": "TELEOP_SERVER_PY"}

# 上位机**曾经**用来订状态的路径。状态口迁到 zenoh 之后 8765 只服务头显，
# 但这些路径不能只是"不再特判" —— 头显分支是"新连接顶替旧连接"，一个缓存了旧页面
# 的上位机连上来会**顶掉真正的头显**并开始往帧路径里灌它自己的空 socket。
# 所以要在 `_process_request` 里明确回绝（见 headset.py）。
LEGACY_STATUS_PATHS: tuple[str, ...] = ("/ws/status", "/ws", "/status")

DEFAULT_BRIDGE_HOST = "0.0.0.0"
DEFAULT_BRIDGE_PORT = 8765
DEFAULT_TELEOP_PORT = 8766
DEFAULT_FOLLOW_SERVER_PORT = 9001

# ── 上游 submodule 在仓库里的位置（见 .gitmodules） ─────────────────────────

SUBMODULE_DIR = "submodules"
TELEOP_DIR = f"{SUBMODULE_DIR}/litearm-vr-teleop"
FOLLOW_DIR = f"{SUBMODULE_DIR}/litearm-vr-follow"

# ⚠ teleop 仓 2026-10-15 重构：代码整体移进了 `teleop/` 子目录，启动脚本也从
#   `scripts/run_teleop_webxr.sh` 换成 `launch/run_teleop_webxr.py`。上游把环境准备
#   （找 metavr-arm 解释器、source ROS2、起 rviz、清残留进程、查端口）全收编进了那个
#   launcher，启动参数也从环境变量改成了命令行。所以桥**调它**，不再自己拼
#   `ws_echo_server.py` 的命令行 —— 那等于把上游的环境知识复制一份。
TELEOP_SUBDIR = "teleop"

# ⚠ 下面两组是**上游仓内**的相对路径，不是相对桥仓库的 —— 上游仓根由
# `_repo_dir()` 解析（`TELEOP_REPO` / `FOLLOW_REPO` 可覆写，默认取 submodule）。
TELEOP_LAUNCHER = f"{TELEOP_SUBDIR}/launch/run_teleop_webxr.py"
TELEOP_SCRIPT = f"{TELEOP_SUBDIR}/scripts/ws_echo_server.py"
FOLLOW_LAUNCHERS = {
    "stm32": "run_stm32_server.sh",
    "sdk": "run_sdk_server.sh",
}
FOLLOW_SCRIPTS = {
    "stm32": "arm_follow_server_stm32.py",
    "sdk": "arm_follow_server_sdk.py",
}

# `--doctor` 会挨个检查这两个 submodule 在不在、是哪个 commit、有没有被就地改过
SUBMODULES: tuple[str, ...] = (TELEOP_DIR, FOLLOW_DIR)

# 受管进程是哪一种上游：preflight 按它决定该去查什么（串口？CAN？conda 环境？）
KIND_TELEOP = "teleop"
KIND_FOLLOW_STM32 = "follow-stm32"
KIND_FOLLOW_SDK = "follow-sdk"
KIND_CUSTOM = "custom"

# 上游 follow_server 的 --mode 默认是 jf；桥这边一直显式传 js（免调增益跟随）。
# 现场要换控制律用 FOLLOW_SERVER_CTRL_MODE 覆盖，别改代码。
DEFAULT_FOLLOW_CTRL_MODE = "js"

_SCHEMES = ("ws://", "wss://", "tcp://", "http://", "https://")


def parse_endpoint(value: str, default_port: int) -> tuple[str, int]:
    """把 `host:port` / `ws://host:port/path` 解析成 (host, port)。

    端口缺省时用 default_port；解析不出端口则抛 ValueError。
    """
    raw = (value or "").strip()
    for scheme in _SCHEMES:
        if raw.lower().startswith(scheme):
            raw = raw[len(scheme):]
            break
    raw = raw.split("/", 1)[0].strip()
    if not raw:
        raise ValueError(f"空地址: {value!r}")

    if raw.startswith("["):  # IPv6 字面量 [::1]:9001
        host, _, rest = raw[1:].partition("]")
        port_s = rest.lstrip(":")
        port = int(port_s) if port_s else default_port
        return host, port

    if ":" in raw:
        host, _, port_s = raw.rpartition(":")
        if not port_s:
            return host, default_port
        try:
            return host, int(port_s)
        except ValueError as exc:
            raise ValueError(f"端口非法: {value!r}") from exc
    return raw, default_port


def format_endpoint(host: str, port: int) -> str:
    return f"{host}:{port}"


def repo_root(start: Path | None = None) -> Path | None:
    """桥自己所在的仓库根 —— 上游 submodule 就在它下面。

    从 `__file__` 往上找第一个带 `.git` / `pyproject.toml` 的目录。以非 editable
    方式装进 site-packages 时找不到，那种情况下默认命令推导不出来（status 里会
    说明原因），用 BRIDGE_ROOT 指路，或者干脆用 `*_CMD` 自己写命令行。
    """
    here = (start or Path(__file__)).resolve()
    for parent in here.parents:
        if (parent / ".git").exists() or (parent / "pyproject.toml").is_file():
            return parent
    return None


@dataclass(frozen=True)
class ProcessSpec:
    """一个受管进程：名字 + 启动命令。"""

    name: str
    argv: tuple[str, ...]
    cwd: str | None = None
    env: dict[str, str] = field(default_factory=dict)
    # 这条命令属于哪一种上游；`*_CMD` 自定义的算 custom
    kind: str = KIND_CUSTOM
    # 已知仓库布局时，这条命令最终会跑到的那个 .py —— 参数对拍与硬件检查要看它
    # （命令本身是 `bash run_xxx.sh`，光看 argv 看不出是哪一版 follow_server）
    probe_script: str | None = None

    def label(self) -> str:
        """给人/界面看的一行。

        env 拼在前面：不然状态页和日志里只剩一条光秃秃的命令，看不出我们到底让它
        按什么参数跑、又额外带了什么环境。
        """
        env_part = " ".join(f"{k}={v}" for k, v in sorted(self.env.items()))
        cmd = " ".join(self.argv)
        return f"{env_part} {cmd}" if env_part else cmd


@dataclass(frozen=True)
class Config:
    # 头显连这里（只想服务头显 —— 上位机走 zenoh，不经过这个口）
    bridge_host: str = DEFAULT_BRIDGE_HOST
    bridge_port: int = DEFAULT_BRIDGE_PORT

    # 对内目标
    teleop_host: str = "127.0.0.1"
    teleop_port: int = DEFAULT_TELEOP_PORT
    follow_host: str = "127.0.0.1"
    follow_port: int = DEFAULT_FOLLOW_SERVER_PORT

    # 阈值与频率
    frame_warn_ms: float = 200.0
    frame_fallback_ms: float = 500.0
    status_hz: float = 10.0
    ping_hz: float = 4.0
    event_ttl_ms: float = 20_000.0
    forward_timeout_ms: float = 500.0
    fallback_enabled: bool = True

    # 真机状态来源（第 8 节待实测：实测前默认 follow_server）
    arm_source: str = "follow_server"

    # ── zenoh（对外接口：帧给 teleop、状态与动作给上位机） ──────────────────
    #
    # `arm_id` 是话题里那一段，**必须与 teleop / 上位机填的一致**；它同时决定了
    # 本服务与 `litearm-server` 在不在同一棵话题树上（都是 `litearm/v4/{arm}/...`）。
    zenoh_arm_id: str = DEFAULT_ARM_ID
    # peer：既连出去（router）也让别人连进来（teleop、以及同机的 ctl）
    zenoh_mode: str = "peer"
    # 听在这。默认全网卡，头显那台机器之外的 router 才连得进来；只想本机用就改 127.0.0.1
    zenoh_listen_endpoints: tuple[str, ...] = ("tcp/0.0.0.0:7447",)
    # 主动连谁。留空 = 只等对端连进来（现场通常由 zenoh-bridge-remote-api 反向连本机）
    zenoh_connect_endpoints: tuple[str, ...] = ()
    # 动作的应答上限（秒）。冷启动 teleop.start 要等端口 + 等上游，约 14s，别调小
    zenoh_reply_timeout_s: float = 20.0

    log_level: str = "INFO"
    log_format: str = "text"
    log_file: Path | None = None
    stop_processes_on_exit: bool = True

    # 受管进程（M3）
    processes: dict[str, ProcessSpec] = field(default_factory=dict)
    process_log_dir: Path = Path("logs")

    # 桥自己所在的仓库根（上游 submodule 在它下面）；推不出来时为 None
    repo_root: Path | None = None

    # 这次从环境里认出来的遗留变量名（`METAVR_*`）；空 = 已迁干净。
    # 启动日志与 `--doctor` 靠它提示迁移，见 LEGACY_ENV_PREFIX 那段注释。
    legacy_env: tuple[str, ...] = ()

    @property
    def teleop_endpoint(self) -> str:
        return format_endpoint(self.teleop_host, self.teleop_port)

    @property
    def follow_endpoint(self) -> str:
        return format_endpoint(self.follow_host, self.follow_port)

    @property
    def teleop_url(self) -> str:
        return f"ws://{self.teleop_endpoint}/"

    @property
    def status_interval_s(self) -> float:
        return 1.0 / max(self.status_hz, 0.1)

    @property
    def ping_interval_s(self) -> float:
        return 1.0 / max(self.ping_hz, 0.1)

    @property
    def real_arm(self) -> bool:
        """teleop 是不是接真机（`--real`）。

        `up.sh` 靠它决定"要不要先让人确认工作区无人、急停在手，再拉起上游"；
        自定义命令（`TELEOP_CMD`）里的 `--real` 也算数。
        """
        spec = self.processes.get("teleop")
        if spec is None:
            return False
        return spec.env.get("REAL") == "1" or "--real" in spec.argv

    def with_overrides(self, **kwargs: object) -> "Config":
        clean = {k: v for k, v in kwargs.items() if v is not None}
        return replace(self, **clean)  # type: ignore[arg-type]

    @classmethod
    def from_env(cls, env: dict[str, str] | None = None) -> "Config":
        e, legacy = _absorb_legacy_env(os.environ if env is None else env)

        host = _str(e, "BRIDGE_HOST", DEFAULT_BRIDGE_HOST)
        port = _int(e, "BRIDGE_PORT", DEFAULT_BRIDGE_PORT)

        teleop_host, teleop_port = parse_endpoint(
            _str(e, "TELEOP_ENDPOINT", f"127.0.0.1:{DEFAULT_TELEOP_PORT}"),
            DEFAULT_TELEOP_PORT,
        )
        follow_host, follow_port = parse_endpoint(
            _str(e, "FOLLOW_SERVER_ENDPOINT", f"127.0.0.1:{DEFAULT_FOLLOW_SERVER_PORT}"),
            DEFAULT_FOLLOW_SERVER_PORT,
        )

        cfg = cls(
            bridge_host=host,
            bridge_port=port,
            teleop_host=teleop_host,
            teleop_port=teleop_port,
            follow_host=follow_host,
            follow_port=follow_port,
            frame_warn_ms=_float(e, "FRAME_WARN_MS", 200.0),
            frame_fallback_ms=_float(e, "FRAME_FALLBACK_MS", 500.0),
            status_hz=_float(e, "STATUS_HZ", 10.0),
            ping_hz=_float(e, "PING_HZ", 4.0),
            event_ttl_ms=_float(e, "EVENT_TTL_MS", 20_000.0),
            forward_timeout_ms=_float(e, "FORWARD_TIMEOUT_MS", 500.0),
            fallback_enabled=_bool(e, "FALLBACK_ENABLED", True),
            arm_source=_str(e, "ARM_SOURCE", "follow_server"),
            log_level=_str(e, "LOG_LEVEL", "INFO"),
            log_format=_str(e, "LOG_FORMAT", "text"),
            log_file=_opt_path(e.get("LOG_FILE")),
            process_log_dir=Path(_str(e, "PROCESS_LOG_DIR", "logs")),
            stop_processes_on_exit=_bool(e, "STOP_PROCESSES_ON_EXIT", True),
            zenoh_arm_id=_str(e, "ZENOH_ARM_ID", DEFAULT_ARM_ID),
            zenoh_mode=_zenoh_mode(e),
            zenoh_listen_endpoints=_endpoints(e, "ZENOH_LISTEN_ENDPOINTS",
                                              ("tcp/0.0.0.0:7447",)),
            zenoh_connect_endpoints=_endpoints(e, "ZENOH_CONNECT_ENDPOINTS", ()),
            zenoh_reply_timeout_s=_float(e, "ZENOH_REPLY_TIMEOUT_S", 20.0),
            repo_root=_bridge_root(e),
            legacy_env=legacy,
        )
        return cfg.with_overrides(processes=_build_processes(e, cfg))


def _absorb_legacy_env(env: dict[str, str]) -> tuple[dict[str, str], tuple[str, ...]]:
    """把遗留的 `METAVR_*` 折成 `TELEOP_*`。

    返回 `(折好的 env, 用到的旧名字)`。新名字优先；两个都在时不算"用了旧的"
    —— 那是迁移途中的正常状态，不该一直告警。

    为什么非认不可：见 `LEGACY_ENV_PREFIX` 那段 —— 有两个键的**缺席**会改变行为，
    而且都是往危险的方向（把不托管的进程拉起来、把仿真变成接真机）。
    """
    out = dict(env)
    used: list[str] = []
    for key in env:
        if not key.startswith(LEGACY_ENV_PREFIX):
            continue
        new = _LEGACY_ALIASES.get(key) or ("TELEOP_" + key[len(LEGACY_ENV_PREFIX):])
        if new in env:
            continue
        out[new] = env[key]
        used.append(key)
    return out, tuple(sorted(used))


def _build_processes(env: dict[str, str], cfg: Config) -> dict[str, ProcessSpec]:
    """从环境变量构造受管进程。

    默认命令 = **上游自己的启动脚本**，位置由仓库布局（submodule）推出；
    `TELEOP_CMD` / `FOLLOW_SERVER_CMD` 给完整命令行可以整个换掉。
    显式留空（变量在、内容为空）= 这个进程桥不托管；推导不出布局（没 init
    submodule、也不是从源码跑的）同样不托管，bridge.py 会把原因写进日志。

    上游仓库根可以显式指路（`TELEOP_REPO` / `FOLLOW_REPO`），不然按桥自己的
    submodule 布局推 —— 现场把上游放在别处时才需要前者。
    """
    specs: dict[str, ProcessSpec] = {}
    teleop = _teleop_spec(env, cfg, _repo_dir(env, "TELEOP_REPO", TELEOP_DIR))
    if teleop is not None:
        specs["teleop"] = teleop
    follow = _follow_spec(env, cfg, _repo_dir(env, "FOLLOW_REPO", FOLLOW_DIR))
    if follow is not None:
        specs["follow_server"] = follow
    return specs


def _bridge_root(env: dict[str, str]) -> Path | None:
    """桥仓库根：BRIDGE_ROOT 可以显式指路；没设就按 `__file__` 推。"""
    explicit = (env.get("BRIDGE_ROOT") or "").strip()
    if explicit:
        return Path(os.path.expanduser(explicit))
    return repo_root()


def _repo_dir(env: dict[str, str], key: str, default_rel: str) -> Path | None:
    """上游仓库根目录。

    `<KEY>`（`TELEOP_REPO` / `FOLLOW_REPO`）显式给了就用它 —— 现场把上游放在
    submodule 之外时才需要。没给就按桥的 submodule 布局推；推不出来（没 init
    submodule、也不是从源码树里跑的）返回 None，表示这个进程桥不托管。

    ⚠ 别把"指路的**空值**"当成"推导不出来"：显式留空走的是 `*_CMD` 那条路
    （见 `_build_processes` 的注释），语义是"这个进程我不要"。
    """
    explicit = (env.get(key) or "").strip()
    if explicit:
        return Path(os.path.expanduser(explicit.rstrip("/")))
    root = _bridge_root(env)
    return (root / default_rel) if root is not None else None


def _teleop_spec(
    env: dict[str, str], cfg: Config, teleop_dir: Path | None
) -> ProcessSpec | None:
    custom = _split_cmd(env.get("TELEOP_CMD"))
    if custom:
        return ProcessSpec(
            name="teleop", argv=tuple(custom), cwd=_str(env, "TELEOP_CWD", "") or None
        )
    if env.get("TELEOP_CMD") is not None or teleop_dir is None:
        return None
    return ProcessSpec(
        name="teleop",
        argv=tuple(_launcher_argv(env, cfg, teleop_dir)),
        cwd=_str(env, "TELEOP_CWD", "") or None,
        env=_child_env(env),
        kind=KIND_TELEOP,
        probe_script=str(teleop_dir / TELEOP_SCRIPT),
    )


def _launcher_argv(env: dict[str, str], cfg: Config, teleop_dir: Path) -> list[str]:
    """跑上游 launcher（`teleop/launch/run_teleop_webxr.py`）的命令行。

    上游 2026-10-15 起**参数取代了环境变量**（`--help` 即总表）：默认值写在
    `teleop/config/teleop.yaml`，命令行显式给的最优先。所以桥这边只拼命令行 ——
    找 metavr-arm 解释器、source ROS2、起 rviz、清残留进程、查端口仍归 launcher，
    那套环境知识在上游只有一份，桥复制一份就是"手工能起、服务起不来"的来源。

    launcher 只用标准库，随便哪个 python3 都能跑；要指定 metavr-arm 那个解释器
    用 `TELEOP_SERVER_PY`（转成 `--metavr-py`）或直接写进 teleop.yaml。
    """
    argv = [
        _str(env, "TELEOP_LAUNCH_PY", "python3"),
        str(teleop_dir / TELEOP_LAUNCHER),
        "--host", _str(env, "TELEOP_BIND_HOST", "127.0.0.1"),
        "--port", str(cfg.teleop_port),
    ]
    server_py = (env.get("TELEOP_SERVER_PY") or "").strip()
    if server_py:
        argv += ["--metavr-py", server_py]

    # 构型：上游默认单臂正装；当前真机是单臂水平侧装，所以每次都讲清楚
    if _bool(env, "TELEOP_DUAL", False):
        argv.append("--dual")
        if _bool(env, "TELEOP_DUAL_UPRIGHT", False):
            argv.append("--upright")
    elif _bool(env, "TELEOP_SIDE", False):
        argv.append("--side")

    argv += ["--viewer", _viewer(env)]

    ik = (env.get("TELEOP_IK") or "").strip()
    if ik:
        argv += ["--ik", ik]

    # 真机桥接：显式留空 TELEOP_ARM_HOST = 只跑仿真；未设置 = 默认接本服务的 follow_server
    arm_host = _arm_host(env, cfg)
    if arm_host:
        # 单臂侧装时仿真里的臂名就是 right（teleop_sim），真机挂在这条上
        argv += ["--real", "--real-right", arm_host]
    real_left = (env.get("TELEOP_REAL_LEFT") or "").strip()
    if real_left:
        argv += ["--real-left", real_left]

    # 末端执行器：驱动都在 follow 仓（hand_server.py / 夹爪服务），teleop 只经 TCP 发指令，
    # 默认落在 follow 那台机器上。上游从 CAN 改成远端之后，这里的 `*_CAN*` 就没有意义了。
    if _bool(env, "TELEOP_HAND", False):
        argv += ["--hand", "--hand-remote",
                 _str(env, "TELEOP_HAND_REMOTE", f"{cfg.follow_host}:9004")]
    if _bool(env, "TELEOP_GRIPPER", False):
        argv += ["--gripper", "--gripper-remote",
                 _str(env, "TELEOP_GRIPPER_REMOTE", f"{cfg.follow_host}:9003")]
    return argv


def _follow_spec(
    env: dict[str, str], cfg: Config, follow_dir: Path | None
) -> ProcessSpec | None:
    custom = _split_cmd(env.get("FOLLOW_SERVER_CMD"))
    if custom:
        return ProcessSpec(
            name="follow_server",
            argv=tuple(custom),
            cwd=_str(env, "FOLLOW_SERVER_CWD", "") or None,
        )
    if env.get("FOLLOW_SERVER_CMD") is not None or follow_dir is None:
        return None

    mode = _follow_mode(env)
    # 参数原样透传给上游脚本（它 exec 到真正的 server），所以硬件检查还能从 argv
    # 里拿到串口 / CAN 口；arm_console 的 PYTHONPATH 由上游脚本自己拼。
    argv = [
        "bash", str(follow_dir / FOLLOW_LAUNCHERS[mode]),
        "--host", _str(env, "FOLLOW_SERVER_BIND_HOST", "127.0.0.1"),
        "--tcp-port", str(cfg.follow_port),
    ]
    if mode == "stm32":
        argv += ["--mode", _str(env, "FOLLOW_SERVER_CTRL_MODE", DEFAULT_FOLLOW_CTRL_MODE),
                 "--port", _str(env, "FOLLOW_SERVER_SERIAL", "/dev/ttyACM0")]
    else:
        argv += ["--iface", _str(env, "FOLLOW_SERVER_CAN", "can0")]
        if _bool(env, "FOLLOW_SERVER_DRY_RUN", False):
            argv.append("--dry-run")
    return ProcessSpec(
        name="follow_server",
        argv=tuple(argv),
        cwd=_str(env, "FOLLOW_SERVER_CWD", "") or None,
        kind=KIND_FOLLOW_STM32 if mode == "stm32" else KIND_FOLLOW_SDK,
        probe_script=str(follow_dir / FOLLOW_SCRIPTS[mode]),
    )


def _child_env(env: dict[str, str], *extra_paths: str) -> dict[str, str]:
    """透传给受管进程的额外环境变量（PYTHONPATH 之类）。

    上游自己拼环境（conda / ROS2 / arm_console），所以这里默认什么都不加；
    留着是为了 `TELEOP_PYTHONPATH` 这类逃生口，以及少见的就地调试。
    """
    parts: list[str] = []
    for key in ("PYTHONPATH", "TELEOP_PYTHONPATH"):
        value = (env.get(key) or "").strip()
        if value:
            parts.append(value)
    parts.extend(p for p in extra_paths if p)
    out: dict[str, str] = {}
    if parts:
        out["PYTHONPATH"] = os.pathsep.join(parts)
    return out


def _follow_mode(env: dict[str, str]) -> str:
    mode = _str(env, "FOLLOW_SERVER_MODE", "stm32").strip().lower()
    return mode if mode in FOLLOW_LAUNCHERS else "stm32"


def _arm_host(env: dict[str, str], cfg: Config) -> str:
    """真机 follow_server 的地址。显式留空 `TELEOP_ARM_HOST=` = 只跑仿真。"""
    if "TELEOP_ARM_HOST" in env:
        return (env.get("TELEOP_ARM_HOST") or "").strip()
    return cfg.follow_endpoint


def _viewer(env: dict[str, str]) -> str:
    viewer = _str(env, "TELEOP_VIEWER", "gui")
    return viewer if viewer in ("gui", "rviz") else "gui"


def _split_cmd(value: str | None) -> list[str] | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        return shlex.split(text)
    except ValueError:
        return None


_ZENOH_MODES = ("peer", "client", "router")


def _zenoh_mode(env: dict[str, str]) -> str:
    """zenoh 会话模式。认不出来就用 peer —— 那是本服务唯一说得通的一种：

    peer 既连出去又让人连进来（teleop 与同机的 ctl 都靠它）；client 只能连出去，
    会把"等对端连进来"这条路掐掉。
    """
    mode = _str(env, "ZENOH_MODE", "peer").strip().lower()
    return mode if mode in _ZENOH_MODES else "peer"


def _endpoints(env: dict[str, str], key: str, default: tuple[str, ...]) -> tuple[str, ...]:
    """逗号分隔的端点表。

    **显式留空（变量在、内容为空）= 空表**，不是"用默认值" —— 与 `*_CMD` 那套语义
    一致：留空是一个决定，不是没配置。所以要"不听任何端口"就写 `X=`。
    """
    if key not in env:
        return default
    raw = (env.get(key) or "").strip()
    if not raw:
        return ()
    return tuple(part.strip() for part in raw.split(",") if part.strip())


def _str(env: dict[str, str], key: str, default: str) -> str:
    value = env.get(key)
    if value is None:
        return default
    value = value.strip()
    return value if value else default


def _int(env: dict[str, str], key: str, default: int) -> int:
    raw = (env.get(key) or "").strip()
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError:
        return default


def _float(env: dict[str, str], key: str, default: float) -> float:
    raw = (env.get(key) or "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _bool(env: dict[str, str], key: str, default: bool) -> bool:
    raw = (env.get(key) or "").strip().lower()
    if not raw:
        return default
    if raw in ("1", "true", "yes", "y", "on"):
        return True
    if raw in ("0", "false", "no", "n", "off"):
        return False
    return default


def _opt_path(value: str | None) -> Path | None:
    text = (value or "").strip()
    return Path(text) if text else None

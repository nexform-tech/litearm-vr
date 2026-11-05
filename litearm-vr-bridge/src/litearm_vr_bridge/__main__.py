"""命令行入口：`litearm-vr-bridge` / `python3 -m litearm_vr_bridge`。

环境变量为底（DESIGN 第 7 节），命令行可覆盖常用几项。
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import signal
from pathlib import Path
from typing import Any, Sequence

from . import __version__
from . import protocol
from .bridge import Bridge
from .config import Config, parse_endpoint
from .ctl import run_ctl
from .logging_setup import setup_logging
from .preflight import format_report, has_failures, run_checks

log = logging.getLogger("litearm_vr_bridge")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="litearm-vr-bridge",
        description="LiteArm VR 遥操链路垫片：收头显帧 → 转发 teleop → 汇总状态 → 掉线兜底",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--version", action="version", version=__version__)
    parser.add_argument("--host", default=None, help="监听地址（BRIDGE_HOST）")
    parser.add_argument("--port", type=int, default=None, help="监听端口（BRIDGE_PORT）")
    parser.add_argument("--teleop-endpoint", default=None,
                        help="转发目标 host:port（TELEOP_ENDPOINT）")
    parser.add_argument("--follow-endpoint", default=None,
                        help="跟随服务 host:port（FOLLOW_SERVER_ENDPOINT）")
    parser.add_argument("--status-hz", type=float, default=None,
                        help="状态推送频率；3D 预览想更平滑可提到 20~30（STATUS_HZ）")
    parser.add_argument("--warn-ms", type=float, default=None,
                        help="帧流异常告警阈值（FRAME_WARN_MS）")
    parser.add_argument("--fallback-ms", type=float, default=None,
                        help="兜底持位阈值（FRAME_FALLBACK_MS）")
    parser.add_argument("--no-fallback", action="store_true",
                        help="关闭自动兜底（只告警，不自动 hold）")
    parser.add_argument("--log-level", default=None, help="DEBUG/INFO/WARNING/ERROR")
    parser.add_argument("--log-format", default=None, choices=["text", "json"])
    parser.add_argument("--log-file", default=None, help="额外写一份日志文件")
    parser.add_argument("--print-config", action="store_true",
                        help="打印生效配置后退出（排障用）")
    parser.add_argument("--doctor", action="store_true",
                        help="启动前体检：端口/上游脚本/参数对拍/串口与 CAN/依赖，"
                             "有 fail 时退出码为 1（不启动服务）")
    parser.add_argument("--ctl", choices=["up", "down", "status"], default=None,
                        help="给运行中的桥发动作：up=teleop.start（并拉起上游进程）、"
                             "down=teleop.stop+latch 并停掉受管进程、status=打印一帧快照")
    parser.add_argument("--ctl-timeout", type=float, default=None,
                        help="--ctl 等待回执的超时秒数")
    return parser


def build_config(args: argparse.Namespace) -> Config:
    cfg = Config.from_env()
    warn_ms = args.warn_ms if args.warn_ms is not None else cfg.frame_warn_ms
    fallback_ms = args.fallback_ms if args.fallback_ms is not None else (
        max(warn_ms, cfg.frame_fallback_ms)
    )
    if fallback_ms < warn_ms:
        raise ValueError(
            f"FRAME_FALLBACK_MS（{fallback_ms:g}）不能小于 FRAME_WARN_MS（{warn_ms:g}）"
        )
    return cfg.with_overrides(
        bridge_host=args.host,
        bridge_port=args.port,
        teleop_host=_host_part(args.teleop_endpoint, cfg.teleop_host),
        teleop_port=_port_part(args.teleop_endpoint, cfg.teleop_port),
        follow_host=_host_part(args.follow_endpoint, cfg.follow_host),
        follow_port=_port_part(args.follow_endpoint, cfg.follow_port),
        status_hz=args.status_hz,
        frame_warn_ms=warn_ms,
        frame_fallback_ms=fallback_ms,
        fallback_enabled=False if args.no_fallback else None,
        log_level=args.log_level,
        log_format=args.log_format,
        log_file=Path(args.log_file) if args.log_file else None,
    )


def _host_part(endpoint: str | None, default: str) -> str | None:
    if not endpoint:
        return None
    return parse_endpoint(endpoint, 0)[0]


def _port_part(endpoint: str | None, default: int) -> int | None:
    if not endpoint:
        return None
    port = parse_endpoint(endpoint, default)[1]
    return port if port > 0 else None


def _config_summary(cfg: Config) -> dict[str, Any]:
    return {
        "bridge": f"{cfg.bridge_host}:{cfg.bridge_port}",
        "zenoh_arm_id": cfg.zenoh_arm_id,
        "status_topic": protocol.status_topic(cfg.zenoh_arm_id),
        "rpc_topic": protocol.rpc_topic(cfg.zenoh_arm_id),
        "frame_topic": protocol.frame_topic(cfg.zenoh_arm_id),
        "status_hz": cfg.status_hz,
        "teleop_endpoint": cfg.teleop_endpoint,
        "follow_server_endpoint": cfg.follow_endpoint,
        "frame_warn_ms": cfg.frame_warn_ms,
        "frame_fallback_ms": cfg.frame_fallback_ms,
        "fallback_enabled": cfg.fallback_enabled,
        "ping_hz": cfg.ping_hz,
        "event_ttl_ms": cfg.event_ttl_ms,
        "forward_timeout_ms": cfg.forward_timeout_ms,
        "arm_source": cfg.arm_source,
        # up.sh 靠它分档：接真机就不自动拉起上游，要先让人确认
        "real_arm": cfg.real_arm,
        "log_level": cfg.log_level,
        "log_format": cfg.log_format,
        "log_file": str(cfg.log_file) if cfg.log_file else None,
        "stop_processes_on_exit": cfg.stop_processes_on_exit,
        "process_log_dir": str(cfg.process_log_dir),
        "processes": {name: spec.label() for name, spec in cfg.processes.items()},
    }


async def _run(cfg: Config) -> None:
    bridge = Bridge(cfg)
    loop = asyncio.get_running_loop()
    stop = loop.create_future()

    def _request_stop(signame: str) -> None:
        if not stop.done():
            log.info("收到 %s，准备退出 ...", signame)
            stop.set_result(signame)

    for signame in ("SIGINT", "SIGTERM"):
        sig = getattr(signal, signame, None)
        if sig is None:
            continue
        try:
            loop.add_signal_handler(sig, _request_stop, signame)
        except (NotImplementedError, RuntimeError):
            pass

    await bridge.start()
    try:
        await stop
    finally:
        await bridge.close()


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(list(argv) if argv is not None else None)
    try:
        cfg = build_config(args)
    except ValueError as exc:
        print(f"配置错误：{exc}")
        return 2

    # --doctor / --ctl 是给人看的（也常被 up.sh 拿去判读），输出只走 stdout：
    # 不配日志，免得 INFO 行混进报告里。
    if args.doctor:
        checks = run_checks(cfg)
        print(format_report(checks))
        return 1 if has_failures(checks) else 0

    if args.ctl:
        result = asyncio.run(run_ctl(args.ctl, cfg, timeout_s=args.ctl_timeout))
        print("\n".join(result.lines))
        return result.code

    setup_logging(cfg.log_level, cfg.log_format, cfg.log_file)

    if args.print_config:
        print(json.dumps(_config_summary(cfg), ensure_ascii=False, indent=2))
        return 0

    try:
        asyncio.run(_run(cfg))
    except KeyboardInterrupt:
        log.info("已中断")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""假头显：没有 Quest 的时候，用它把"帧"喂给桥。

真机联调时它顶替头显，也用来复现"头显卡住 / 掉线"的兜底场景。桥对数据连接没有身份
要求（只要不是 `/ws/status`、`/ws`、`/status` 这三个路径，其余一律当头显），帧格式就是
`litearm-vr/docs/PROTOCOL.md` 里那份。**只发最新一帧**，与真头显一致。

用法（现场流程见 docs/REAL-ARM-BRINGUP.md）：

    python3 scripts/fake_headset.py                   # 90Hz 推帧，1 秒后按 grip，小幅画圆
    python3 scripts/fake_headset.py --status          # 顺带订状态话题，每秒打一行
    python3 scripts/fake_headset.py --pattern hold    # 只推静止帧：验"帧通了但臂不动"
    python3 scripts/fake_headset.py --no-grip         # 全程不按 grip：只验转发，不启动遥操
    python3 scripts/fake_headset.py --drop-after 5    # 5 秒后停发（连接不断）→ 复现兜底 latch
    python3 scripts/fake_headset.py --duration 20     # 推 20 秒后断开退出（= 拔线）

⚠ 桥那边若是真机档（`--real`），这一推就是**真机在动**：先确认工作区无人、急停在手。
   默认幅度 3cm、周期 4 秒、1 秒后才按 grip —— 先小后大。

关于幅度：上游 teleop 是**速度式 delta 映射**（`teleop_sim.py` 的 `_step_teleop`），
第一帧只当锚点，之后只看帧间增量；而且带静止死区
（`litearm_teleop/teleop_sim.py:75` `DELTA_DEADBAND_T_OUT = 3.0e-4` m/帧）——
增量太小它当你没动。所以启动时会打印实际每帧增量并与死区比一下，好对着调
`--radius` / `--period` / `--hz`。
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import time

# 手柄在 OpenXR local-floor 空间里的初始位姿（右手，单位米）。绝对坐标不重要 ——
# 上游第一帧只当锚点，之后只看增量，所以随便给个"人伸手"的位置就行。
BASE_POSE = (0.25, 1.05, -0.35)

# 上游 teleop 的静止死区（退出静止的阈值），用来判断我们推的幅度动不动的了臂。
# 出处：litearm_teleop/teleop_sim.py:75。改上游就得跟着改这里。
DEADBAND_T_OUT = 3.0e-4  # m/帧


def pose_at(t: float, *, pattern: str, radius: float, period: float,
            base: tuple[float, float, float] = BASE_POSE) -> tuple[float, float, float]:
    """t 秒时手柄应该在的位置。"""
    if pattern == "hold":
        return base
    w = 2.0 * math.pi * t / max(period, 1e-6)
    if pattern == "circle":  # 竖直面内画圆
        return (base[0] + radius * math.cos(w), base[1], base[2] + radius * math.sin(w))
    return (base[0] + radius * math.sin(w), base[1], base[2])  # line：左右来回


def controller_frame(pos: tuple[float, float, float], *, grip: bool,
                     trigger: float = 0.0, tracked: bool = True) -> dict:
    """一个手柄的字段（姿态一律单位四元数：只验平移，不验腕部姿态）。"""
    x, y, z = pos
    return {
        "tracked": tracked,
        "position": {"x": x, "y": y, "z": z},
        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0},
        "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "gripActive": grip,
        "trigger": trigger,
    }


def frame_json(pos: tuple[float, float, float], *, grip: bool, trigger: float = 0.0,
               ts_ms: int | None = None, tracked: bool = True) -> str:
    """一帧头显数据（字段与 litearm-vr/docs/PROTOCOL.md 一致）。

    左手也发，但不按 grip —— 遥操只用右手（单臂侧装时仿真的臂名就是 right）。
    """
    if ts_ms is None:
        ts_ms = int(time.time() * 1000)
    return json.dumps(
        {
            "timestamp": int(ts_ms),
            "leftController": controller_frame(pos, grip=False, tracked=tracked),
            "rightController": controller_frame(
                pos, grip=grip, trigger=trigger, tracked=tracked
            ),
        },
        separators=(",", ":"),
    )


def per_frame_delta(radius: float, period: float, hz: float) -> float:
    """最坏情况下的每帧增量（米）——圆/线的速度是 2πr/T，除以帧率。"""
    if period <= 0 or hz <= 0:
        return 0.0
    return 2.0 * math.pi * radius / period / hz


def _pose_str(pos: tuple[float, float, float]) -> str:
    return "({:.3f}, {:.3f}, {:.3f})".format(*pos)


async def publish(args: argparse.Namespace) -> int:
    import websockets  # 延迟导入：帧构造那几个函数不依赖它（测试要 import 本文件）

    url = f"ws://{args.host}:{args.port}/"
    delta = per_frame_delta(args.radius, args.period, args.hz)
    print(f"[假头显] 连接 {url} …")
    try:
        ws = await websockets.connect(url, max_queue=None)
    except OSError as exc:
        print(f"[假头显] 连不上桥（{exc}）—— 桥起来了吗？./scripts/up.sh", flush=True)
        return 2

    async with ws:
        print(
            f"[假头显] 已连上：{args.hz:g}Hz，{args.pattern}，半径 {args.radius * 100:g}cm，"
            f"周期 {args.period:g}s，起始位姿 {_pose_str(BASE_POSE)}"
        )
        if args.pattern != "hold":
            print(
                f"[假头显] 每帧增量 ≈ {delta * 1000:.2f} mm"
                f"（上游死区 {DEADBAND_T_OUT * 1000:.2f} mm/帧）"
            )
            if delta <= DEADBAND_T_OUT:
                print(
                    "[假头显] ⚠ 低于死区：上游会当你没动。调大 --radius，或调小 --period / --hz"
                )
        if args.no_grip:
            print("[假头显] --no-grip：只推帧，不按 grip（遥操不会启动）")
        else:
            print(f"[假头显] {args.warmup:g}s 后按 grip（= 启动遥操，真机档会先 movej 回 home）")

        start = time.monotonic()
        interval = 1.0 / max(args.hz, 0.1)
        deadline = start + args.duration if args.duration else None
        next_at = start
        sent = 0
        dropped_at: float | None = None

        while True:
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break
            if args.drop_after and now - start >= args.drop_after:
                if dropped_at is None:
                    dropped_at = now
                    print(
                        "[假头显] 停发帧（连接保持）—— 桥应当在 FRAME_FALLBACK_MS（默认 500ms）"
                        "内 latch 就地持位：看 --status 或桥的日志",
                        flush=True,
                    )
                # 连接留着、一帧不发：这是"头显卡住"，不是拔线
                await asyncio.sleep(0.5)
                continue

            t = now - start
            grip = (not args.no_grip) and t >= args.warmup
            await ws.send(frame_json(pose_at(t, pattern=args.pattern, radius=args.radius,
                                            period=args.period), grip=grip))
            sent += 1

            next_at += interval
            delay = next_at - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                next_at = time.monotonic()  # 落后太多就重排，不追帧

        print(f"[假头显] 结束：共发 {sent} 帧" + ("（后半段是停发状态）" if dropped_at else ""))
        return 0


def _status_topic(args: argparse.Namespace) -> str:
    return f"litearm/v4/{args.arm_id}/vr_status"


def watch_status(args: argparse.Namespace) -> None:
    """订状态话题，每秒打一行 —— 一个命令就能看清"帧有没有到、臂认没认"。

    走 zenoh（原来的 `ws /ws/status` 已经没有了）。用的是 client 直连桥的监听端点，
    和 `ctl` 一样**不需要 router** —— 排查现场时少一个会坏的东西。
    """
    import zenoh

    zcfg = zenoh.Config()
    zcfg.insert_json5("mode", '"client"')
    zcfg.insert_json5("connect/endpoints", f'["{args.zenoh_endpoint}"]')
    zcfg.insert_json5("scouting/multicast/enabled", "false")
    zcfg.insert_json5("scouting/gossip/enabled", "false")

    last = [0.0]

    def on_sample(sample) -> None:
        now = time.monotonic()
        if now - last[0] < args.status_every:
            return
        last[0] = now
        try:
            snap = json.loads(bytes(sample.payload))
        except (ValueError, TypeError):
            return
        print("[状态] " + _status_line(snap), flush=True)

    try:
        session = zenoh.open(zcfg)
    except Exception as exc:  # noqa: BLE001
        print(f"[状态] 订阅失败（{exc}）", flush=True)
        return
    try:
        session.declare_subscriber(_status_topic(args), on_sample)
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        session.close()


def _status_line(snap: dict) -> str:
    bridge = snap.get("bridge") or {}
    headset = snap.get("headset") or {}
    arm = snap.get("arm") or {}
    link = snap.get("link") or {}
    fallback = snap.get("fallback") or {}
    processes = snap.get("processes") or {}

    def num(value: object, unit: str = "") -> str:
        return "—" if value is None else f"{value:g}{unit}" if isinstance(value, (int, float)) else str(value)

    parts = [
        f"fps={num(headset.get('fps'))}",
        f"帧龄={num(headset.get('last_frame_age_ms'), 'ms')}",
        f"转发={num(headset.get('frames_forwarded'))}",
        f"丢帧={num(headset.get('frames_dropped'))}",
        f"耗时={num(bridge.get('forward_cost_ms'), 'ms')}",
        f"teleop={'已连' if bridge.get('teleop_connected') else '未连'}",
        f"臂={arm.get('state') or '—'}",
        f"链路={link.get('state')}",
        f"转发开关={'开' if link.get('forward_enabled') else '关'}",
        f"兜底={'生效' if fallback.get('active') else '否'}",
    ]
    procs = " ".join(f"{name}={state}" for name, state in sorted(processes.items()))
    return "  ".join(parts) + (f"  进程 {procs}" if procs else "")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="假头显：往桥推手柄帧（没有 Quest 时用）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--host", default="127.0.0.1", help="桥的地址")
    p.add_argument("--port", type=int, default=8765, help="桥的端口（BRIDGE_PORT）")
    p.add_argument("--hz", type=float, default=90.0, help="推帧频率（真头显 ~90）")
    p.add_argument("--pattern", choices=["circle", "line", "hold"], default="circle",
                   help="circle/line 会动；hold 只推静止帧")
    p.add_argument("--radius", type=float, default=0.03, help="运动半径（米）")
    p.add_argument("--period", type=float, default=4.0, help="画一圈/来回一次要几秒")
    p.add_argument("--warmup", type=float, default=1.0, help="几秒后按 grip")
    p.add_argument("--no-grip", action="store_true", help="全程不按 grip")
    p.add_argument("--drop-after", type=float, default=0.0,
                   help="多少秒后停发帧（连接保持）—— 复现兜底，0=不停")
    p.add_argument("--duration", type=float, default=0.0,
                   help="推多少秒后断开退出（= 拔线），0=一直推")
    p.add_argument("--status", action="store_true",
                   help="同时订状态话题打状态行（走 zenoh，不需要 router）")
    p.add_argument("--zenoh-endpoint", default="tcp/127.0.0.1:7447",
                   help="桥的 zenoh 监听端点（默认 tcp/127.0.0.1:7447）")
    p.add_argument("--arm-id", default="armA", help="话题里的 arm 段（默认 armA）")
    p.add_argument("--status-every", type=float, default=1.0, help="状态行间隔（秒）")
    return p


async def main_async(args: argparse.Namespace) -> int:
    tasks = [asyncio.create_task(publish(args))]
    if args.status:
        # watch_status 是阻塞的（zenoh 的 Python 接口是同步的），扔到线程里跑
        tasks.append(asyncio.create_task(asyncio.to_thread(watch_status, args)))
    done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    for task in pending:
        task.cancel()
    return next((t.result() for t in done if not t.cancelled() and t.exception() is None), 0)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\n[假头显] 已中断")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())

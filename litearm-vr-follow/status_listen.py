#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""极小的 Zenoh 订阅端（验证/调试用，系统里没有 `z_sub`）。

订阅状态 topic（默认通配 `litearm/v4/**`，即各设备**各一条**），把每条设备本体
JSON 打到 stdout。key 形如 `{prefix}/{arm_id}/vr_*`：末段（`vr_arm`/`vr_grip`/`vr_hand`）
辨识设备，中段是臂身份（`armA`/`armB`）；回退按 payload 形状。

配置照搬 `litegrip` 的 `_base_config()`：`mode=peer` + 关发现 + 显式 `listen`/`connect`。
本机与发布端同为 peer 时，**两端要有一端 listen、另一端 connect**：默认订阅端也
listen（`tcp/0.0.0.0:17450`）并 connect 发布端（`tcp/127.0.0.1:17449`）。

用法：
  python3 status_listen.py                                   # 通配订阅 litearm/v4/**
  python3 status_listen.py --key 'litearm/v4/arm/vr_arm'     # 只订臂
  python3 status_listen.py --listen none --connect tcp/127.0.0.1:17449
"""
from __future__ import annotations

import argparse
import json
import sys
import time


def build_config(listen, connect, scouting, mode):
    import zenoh

    c = zenoh.Config()
    c.insert_json5("mode", json.dumps(mode))
    if not scouting:
        c.insert_json5("scouting/multicast/enabled", "false")
        c.insert_json5("scouting/gossip/enabled", "false")
    c.insert_json5("listen/endpoints", json.dumps(listen))
    c.insert_json5("connect/endpoints", json.dumps(connect))
    return c


def _fmt(key, d):
    """按 topic 末段（`vr_*`，回退 payload 形状）选臂/夹爪/手的打印格式。

    key 形如 `{prefix}/{arm_id}/vr_*`：中段是臂身份，末段是设备通道。
    """
    parts = key.split("/")
    arm_id = parts[-2] if len(parts) >= 2 else ""
    leaf = parts[-1] if parts else ""
    if leaf == "vr_grip" or (isinstance(d.get("q"), (int, float)) and "tau" in d):
        return (f"grip[{arm_id}] {d.get('device')} q={d.get('q')} tau={d.get('tau')}"
                f" en={d.get('enabled')}"
                f"{'' if d.get('connected') else '(断)'}")
    if leaf == "vr_hand" or "angles" in d:
        return (f"hand[{arm_id}] {d.get('device')} angles={d.get('angles')}"
                f"{'' if d.get('connected') else '(断)'}")
    return (f"arm[{arm_id}] {d.get('device')} src={d.get('source') or '?'}"
            f"{'' if d.get('connected') else '(断)'}"
            f" q={d.get('q')}"
            f"{'' if not d.get('fault') else ' fault=' + str(d['fault'])}"
            f"{'' if not d.get('flags') else ' flags=' + str(d['flags'])}")


def main(argv=None):
    p = argparse.ArgumentParser(description="订阅 follow 状态 topic（调试用）")
    p.add_argument("--key", default=None,
                   help="Zenoh key 表达式（默认通配 litearm/v4/**，可重复）",
                   action="append")
    p.add_argument("--listen", action="append", default=None,
                   help="listen 端点（可重复）；值为 'none' 表示不 listen")
    p.add_argument("--connect", action="append", default=["tcp/127.0.0.1:17449"],
                   help="connect 端点（可重复）")
    p.add_argument("--zenoh-scouting", action="store_true")
    p.add_argument("--zenoh-mode", default="peer", choices=("peer", "client"))
    p.add_argument("--raw", action="store_true", help="只打 JSON，不加工")
    args = p.parse_args(argv)
    if not args.key:
        args.key = ["litearm/v4/**"]

    listen = args.listen if args.listen is not None else ["tcp/0.0.0.0:17450"]
    listen = [] if listen and listen[0] == "none" else listen

    try:
        import zenoh
    except ImportError:
        print("错误: 未找到 zenoh 模块。请先 pip install eclipse-zenoh",
              file=sys.stderr)
        return 1

    cfg = build_config(listen, args.connect, args.zenoh_scouting, args.zenoh_mode)
    session = zenoh.open(cfg)
    start = time.monotonic()
    n = 0

    def on_sample(sample):
        nonlocal n
        n += 1
        key = str(sample.key_expr)
        text = bytes(sample.payload).decode("utf-8", "replace")
        if args.raw:
            print(f"[{time.monotonic() - start:8.3f}s] {key} {text}", flush=True)
            return
        try:
            d = json.loads(text)
        except ValueError:
            print(f"[{time.monotonic() - start:8.3f}s] {key} (非 JSON) {text}",
                  flush=True)
            return
        print(f"[{time.monotonic() - start:8.3f}s] {key} {_fmt(key, d)}",
              flush=True)

    subs = [session.declare_subscriber(k, on_sample) for k in args.key]
    print(f"订阅 {' '.join(args.key)}（Ctrl-C 退出）…", flush=True)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subs:
            try:
                sub.undeclare()
            except Exception:  # noqa: BLE001
                pass
        session.close()
    print(f"\n收到 {n} 条后退出", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

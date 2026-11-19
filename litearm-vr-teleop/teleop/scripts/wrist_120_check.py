#!/usr/bin/env python3
"""Read-only limit query and CSV export for one single-arm wrist replay."""

import argparse
import csv
import json
import math
from pathlib import Path
import socket
import time
import xml.etree.ElementTree as ET


def vector(values, size):
    return (isinstance(values, list) and len(values) == size
            and all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in values))


def read_limits(address, output):
    host, port = address.rsplit(":", 1)
    with socket.create_connection((host, int(port)), timeout=2.0) as sock:
        with sock.makefile("rwb") as stream:
            for request_id in range(30):
                stream.write(json.dumps({"cmd": "state", "request_id": request_id}).encode()
                             + b"\n")
                stream.flush()
                line = stream.readline()
                if not line:
                    raise SystemExit("跟随端已断开")
                state = json.loads(line)
                lo, hi = state.get("q_min"), state.get("q_max")
                if (not vector(lo, 7) or not vector(hi, 7)
                        or any(a > b for a, b in zip(lo, hi))):
                    raise SystemExit("跟随端缺少有效的 q_min/q_max")
                if (state.get("request_id") == request_id and state.get("valid") is True
                        and vector(state.get("q"), 7)):
                    break
                time.sleep(0.05)
            else:
                raise SystemExit("未收到新鲜关节反馈，先处理跟随端状态")
    text = json.dumps(state, ensure_ascii=False, indent=2)
    output.write_text(text + "\n", encoding="utf-8")
    print(text)


def read_rows(path):
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def write_rows(path, columns, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)


def rotate(quat, value):
    # PyBullet quaternion order is x, y, z, w.
    norm = math.hypot(*quat)
    x, y, z, w = (v / norm for v in quat)
    vx, vy, vz = value
    a, b, c = 2 * (y*vz - z*vy), 2 * (z*vx - x*vz), 2 * (x*vy - y*vx)
    return vx + w*a + y*c - z*b, vy + w*b + z*a - x*c, vz + w*c + x*b - y*a


def report(directory, urdf, tcp_offset):
    state = json.loads((directory / "limits.json").read_text(encoding="utf-8"))
    joints = {j.attrib["name"]: j.find("limit") for j in ET.parse(urdf).getroot().findall("joint")}
    model = [joints[f"joint{i}"].attrib for i in range(1, 8)]
    lo = [max(value, float(item["lower"])) for value, item in zip(state["q_min"], model)]
    hi = [min(value, float(item["upper"])) for value, item in zip(state["q_max"], model)]
    if any(a > b for a, b in zip(lo, hi)):
        raise SystemExit("跟随端和 URDF 限位没有交集")

    sim = [row for row in read_rows(directory / "sim.jsonl") if row.get("arm") == "right"]
    if not sim:
        raise SystemExit("遥操探针无右臂轨迹")
    start, end = sim[0]["t"], sim[-1]["t"]
    active_follow = [row for row in read_rows(directory / "follow.jsonl")
                     if start <= row["t"] <= end and row.get("following") is True]
    traces = [
        ("teleop", sim, "q_after"),
        ("follow_target", active_follow, "target"),
        ("follow_cmd", active_follow, "cmd_q"),
        ("follow_measured", active_follow, "q"),
    ]
    joint_rows = []
    for source, data, field in traces:
        joint_rows.extend((row["t"] - start, source, *row[field])
                          for row in data if vector(row.get(field), 7))
    write_rows(directory / "joints.csv",
               ["time_s", "source", *[f"j{i}_rad" for i in range(1, 8)]], sorted(joint_rows))

    anchor = sim[0]["ik"]["raw_target_tcp"]
    origin_q = sim[0]["hand_q"]
    hand_span = 0.0
    hand_shift = 0.0
    tcp_rows = []
    for row in sim:
        info = row["ik"]
        motion = abs(sum(a*b for a, b in zip(origin_q, row["hand_q"])))
        denom = math.hypot(*origin_q) * math.hypot(*row["hand_q"])
        angle = math.degrees(2 * math.acos(min(1.0, motion / denom)))
        hand_span = max(hand_span, angle)
        hand_shift = max(hand_shift, math.dist(sim[0]["hand_p"], row["hand_p"]))
        offset = rotate(row["ee_after_q"], tcp_offset)
        command_tcp = [a+b for a, b in zip(row["ee_after_p"], offset)]
        positions = [("target", info["target_tcp"]), ("teleop_tcp", command_tcp)]
        if vector(info.get("measured_q"), 7):
            positions.append(("measured_fk", info["measured_tcp"]))
        tcp_rows.extend((row["t"] - start, source, *pos, math.dist(pos, anchor) * 1000,
                         angle, bool(info.get("wrist_tracking")), bool(info.get("position_only")))
                        for source, pos in positions)
    write_rows(directory / "tcp.csv", ["time_s", "source", "x_m", "y_m", "z_m",
                                       "from_anchor_mm", "hand_angle_deg", "wrist_tracking",
                                       "position_only"], tcp_rows)

    print("手柄峰值 %.2f°，平移 %.2f mm，纯转腕帧 %d/%d" %
          (hand_span, hand_shift * 1000, sum(bool(r["ik"].get("wrist_tracking")) for r in sim),
           len(sim)))
    print("有效 J7 行程 %.3f° 到 %.3f°" % (math.degrees(lo[6]), math.degrees(hi[6])))
    missing = []
    for source, _, _ in traces:
        joint = [row[2:] for row in joint_rows if row[1] == source]
        if not joint:
            missing.append(source)
            continue
        start_q = sim[0]["q_before"] if source == "teleop" else joint[0]
        swing = [math.degrees(max(abs(q[i] - start_q[i]) for q in joint)) for i in range(4)]
        excess = max(0.0, *(max(a-v, v-b) for q in joint for v, a, b in zip(q, lo, hi)))
        j7 = [math.degrees(q[6]) for q in joint]
        print(f"{source}: J1–J4 最大偏移 {[round(v, 3) for v in swing]}°，"
              f"J7 范围 [{min(j7):.3f}, {max(j7):.3f}]°，最大越界 {excess:.6f} rad")
    for source in ("teleop_tcp", "measured_fk"):
        values = [row[5] for row in tcp_rows if row[1] == source]
        if values:
            print(f"{source}: TCP 峰值 {max(values):.3f} mm，结束偏移 {values[-1]:.3f} mm")
        else:
            missing.append(source)
    if missing:
        raise SystemExit("缺少轨迹: " + ", ".join(missing))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    limits = sub.add_parser("limits")
    limits.add_argument("address")
    limits.add_argument("output", type=Path)
    run = sub.add_parser("report")
    run.add_argument("directory", type=Path)
    run.add_argument("--urdf", type=Path, required=True)
    run.add_argument("--tcp-offset", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    args = parser.parse_args()
    if args.command == "limits":
        read_limits(args.address, args.output)
    else:
        report(args.directory, args.urdf, args.tcp_offset)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""遥操数据探针：把「手 → 轴映射 → 平滑 → 死区 → 目标 → IK → 关节 → 末端 → 真机」
整条链每帧落盘，然后按跳变**归属原因**。

为什么需要它：单看最终关节角分不出一次跳变是谁造成的。各原因在数据上长得**不一样**：

| 现象 | 数据特征 |
|---|---|
| IK 换了分支（零空间/多解） | 关节角大跳，但**末端几乎没动** |
| 近奇异 + 阻尼 | 末端没动够（增益 < 1），关节角**没有**跳变 |
| 策略持位（exact 的盒/残差门、pin 的 QP 失败） | 关节角**不动**，`ik.failure` / `achieved=False` |
| 持位恢复 | 恢复那一帧把积压的手部动作**一次性放出来** → 突然大跳 |
| 静止死区 | 手在动而关节角**严格不动**（`zeroed=True`），解冻后猛追 |
| 限幅/步长钳制 | `ik.at_limit` / `ik.clipped` = True |
| 真机侧（slew / 限位 / 看门狗） | 下发目标平滑，但实测 `q` 跟不上或跳 |

用法（三段）：

    # ① metavr 侧：带 --probe 起（写 sim 流）
    python scripts/ws_echo_server.py --probe /tmp/probe_sim.jsonl --real ...
    # ② 真机侧：另开一个终端采样 follow server（它自己就是第二段链路的真值）
    python scripts/probe.py sample --addr 127.0.0.1:9002 --out /tmp/probe_follow.jsonl
    # ③ 复现完之后出报告
    python scripts/probe.py report --sim /tmp/probe_sim.jsonl --follow /tmp/probe_follow.jsonl
"""
import argparse
import json
import socket
import sys
import threading
import time

#: sim 流单文件上限（行）。60Hz 下一小时约 21.6 万帧 —— 超过就停写并告警，
#: 免得复现一整晚把磁盘写满。
MAX_LINES = 250000


def _round(obj, nd=6):
    """递归把浮点压到 nd 位 —— 遥操量的精度远用不到 float 全位，不压的话
    一小时能写掉上百 MB（60Hz × 每帧几十个数）。"""
    if isinstance(obj, float):
        return round(obj, nd)
    if isinstance(obj, dict):
        return {k: _round(v, nd) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_round(v, nd) for v in obj]
    return obj


class JsonlWriter:
    """sim 侧探针回调：每帧一行 JSON。`__call__(arm, rec)` 与 `Arm._emit_probe` 对接。"""

    def __init__(self, path, max_lines=MAX_LINES):
        self._f = open(path, "w", encoding="utf-8")
        self._n = 0
        self._max = max_lines
        self._warned = False

    def __call__(self, arm, rec):
        if self._n >= self._max:
            if not self._warned:
                self._warned = True
                print("[探针] 已达 %d 行上限, 停止写入（复现请分段跑）" % self._max,
                      flush=True)
            return
        rec["i"] = self._n
        rec["arm"] = getattr(arm, "name", "?")
        self._f.write(json.dumps(_round(rec), separators=(",", ":")) + "\n")
        self._n += 1
        if self._n % 200 == 0:
            self._f.flush()

    def close(self):
        try:
            self._f.flush()
            self._f.close()
        except OSError:
            pass


class FollowSampler:
    """按固定频率向 follow server 发 `{"cmd":"ping"}` 并落盘。

    follow server 就是第二段链路的真值源：它的回包里
    `target` = metavr 实际送达的原始目标、`anchor` = 静止保持锚点、
    `cmd` = 经梯形 slew 后的下发目标、`q` = 真机实测。
    于是「metavr 发了什么 / server 决定发什么 / 臂实际到哪」三段可以对齐。
    """

    def __init__(self, addr, path, hz=50.0):
        host, _, port = addr.partition(":")
        self._addr = (host, int(port))
        self._out = JsonlWriter(path)
        self._period = 1.0 / max(1.0, hz)
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._t.start()

    def _loop(self):
        n = 0
        while not self._stop.is_set():
            t0 = time.time()
            try:
                with socket.create_connection(self._addr, timeout=1.0) as s:
                    f = s.makefile("rw")
                    while not self._stop.is_set():
                        f.write('{"cmd":"ping"}\n')
                        f.flush()
                        line = f.readline()
                        if not line:
                            break
                        r = json.loads(line)
                        self._out(None, {"t": time.time(), "state": r.get("state"),
                                         "flags": r.get("flags"), "q": r.get("q"),
                                         "target": r.get("target"),
                                         "anchor": r.get("anchor"),
                                         "cmd_q": r.get("cmd_q"),
                                         "following": r.get("following"),
                                         "mode": r.get("mode")})
                        n += 1
                        if n == 1:
                            print("[探针] follow 采样已连上 %s" % (self._addr,), flush=True)
                        time.sleep(max(0.0, self._period - (time.time() - t0)))
                        t0 = time.time()
            except (OSError, ValueError) as e:
                if not self._stop.is_set():
                    print("[探针] follow 采样断开(%s), 1s 后重连" % e, flush=True)
                    time.sleep(1.0)

    def stop(self):
        self._stop.set()
        self._t.join(timeout=2.0)
        self._out.close()


# --------------------------------------------------------------------------
# 报告
# --------------------------------------------------------------------------
def load(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    pass
    rows.sort(key=lambda r: r.get("t", 0.0))
    return rows


def span(rows, key):
    vals = [r.get(key) for r in rows if r.get(key) is not None]
    if not vals:
        return None
    return (min(vals), max(vals))


def top_jumps(rows, key, k=12):
    """返回 [(|Δ|∞, i)]，i 是本序列内下标。"""
    out = []
    prev = None
    for i, r in enumerate(rows):
        v = r.get(key)
        if v is None:
            continue
        if prev is not None:
            out.append((max(abs(a - b) for a, b in zip(v, prev)), i))
        prev = v
    out.sort(reverse=True)
    return out[:k]


def nearest(rows, t):
    """二分找 t 最近的样本。"""
    if not rows:
        return None
    lo, hi = 0, len(rows) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if rows[mid].get("t", 0.0) < t:
            lo = mid + 1
        else:
            hi = mid
    best = rows[lo]
    if lo > 0 and abs(rows[lo - 1].get("t", 0.0) - t) < abs(best.get("t", 0.0) - t):
        best = rows[lo - 1]
    return best


def verdict(rec, fol, prev_rec=None, sigma_th=0.05):
    """给一帧的跳变归因。返回原因列表（空 = 看起来正常）。

    ⚠ 先判**帧外**断点：若本帧的 `q_before` 不等于上一帧的 `q_after`，说明关节状态
    是被 `_step_teleop` 之外的命令改的（`go_home`/双击归位/真机 `movej`）——
    这种"跳变"是**被命令的**，与 IK / 奇异策略无关，必须与求解器跳变分开报，
    否则会把「归位」误诊成「IK 乱跳」。
    """
    why = []
    ik = rec.get("ik") or {}
    dq = [a - b for a, b in zip(rec["q_after"], rec["q_before"])]
    dq_inf = max(abs(v) for v in dq)
    mag_t = rec.get("mag_t", 0.0)
    mag_r = rec.get("mag_r", 0.0)
    held_before = ik.get("held_before", False)
    achieved = ik.get("achieved", ik.get("ok", True))

    if prev_rec is not None:
        gap = max(abs(a - b) for a, b in zip(rec["q_before"], prev_rec["q_after"]))
        if gap > 1e-6:
            why.append("帧外命令改了关节状态（Δ=%.3f）→ go_home/归位/真机 movej, "
                       "**不是 IK 或奇异问题**" % gap)
            return why

    if ik.get("backend") == "exact":
        if not achieved:
            why.append("策略持位: failure=%s residual=%.3g worst_joint=%s 超出=%.3f"
                       % (ik.get("failure"), ik.get("residual", 0.0),
                          ik.get("worst_joint"), ik.get("worst_excess", 0.0)))
        elif held_before:
            why.append("持位恢复: 上一帧还在持位, 本帧把积压的手部动作放了出来")
    elif ik.get("backend") == "pin":
        if not ik.get("ok", True):
            why.append("pin QP 失败 → 就地持位")
        vmax, vlim = ik.get("v_absmax", 0.0), ik.get("v_limit", 0.0)
        if ik.get("at_limit"):
            why.append("关节速度顶到上限 (|v|max=%.3f / %.3f) → 是盒在决定答案"
                       % (vmax, vlim))
        elif vlim and vmax > 0.9 * vlim:
            why.append("关节速度逼近上限 (|v|max=%.3f / %.3f) → 答案被削过, "
                       "不是 IK 解出来的那么大" % (vmax, vlim))
    elif ik.get("backend") == "pybullet":
        if not ik.get("ok", True):
            why.append("pybullet IK 未收敛 (err_pos=%.4f err_rot=%.4f) → 就地持位"
                       % (ik.get("err_pos", 0.0), ik.get("err_rot", 0.0)))
        if ik.get("clipped"):
            why.append("步长钳制 ±%.4f rad 在起作用" % ik.get("dq_max", 0.0))

    if rec.get("zeroed"):
        why.append("死区: 本帧增量被严格置零(手在动而臂不动)")
    sig = rec.get("sigma_min_tcp")
    if sig is not None and sig < sigma_th:
        why.append("近奇异: σ_min(TCP)=%.4f < %.3f" % (sig, sigma_th))

    # 关节大跳但末端没动 ⇒ 冗余零空间换了分支，工具其实没乱跑
    ee = rec.get("ee_after_p")
    if ee and dq_inf > 0.05:
        # 用关节尺度粗估"这跳该带来多大末端位移"：0.25 m/rad 是 7 轴臂的量级
        if dq_inf * 0.25 < 0.005 and max(abs(v) for v in dq) > 0.15:
            why.append("末端几乎没动 → 冗余零空间换分支(工具没跑, 但关节跳了)")
    if fol:
        dtarget = fol.get("target")
        dq_cmd = fol.get("cmd_q")
        if dq_cmd is not None and mag_t + mag_r < 1e-6 and dq_inf > 1e-6:
            why.append("手不动但目标在动 → 查上游/锚定")
    if not why:
        why.append("正常尺度(可能是操作者本身的大动作)")
    return why


def report(sim_path, follow_path, top=12, sigma_th=0.05):
    sim = load(sim_path)
    fol = load(follow_path) if follow_path else []
    print("=== 载入 ===")
    print("  sim 帧数 = %d   时间跨度 %.1f s" % (len(sim), (sim[-1]["t"] - sim[0]["t"]) if len(sim) > 1 else 0.0))
    if fol:
        print("  follow 样本 = %d  时间跨度 %.1f s" % (len(fol), fol[-1]["t"] - fol[0]["t"]))
    if not sim:
        return 1
    dts = [r.get("dt", 0.0) for r in sim if r.get("dt")]
    if dts:
        dts_sorted = sorted(dts)
        print("  sim dt: 最小 %.4f  中位 %.4f  p95 %.4f  max %.4f  (FREQ=60 ⇒ 理想 0.0167)"
              % (dts_sorted[0], dts_sorted[len(dts) // 2],
                 dts_sorted[int(len(dts) * 0.95)], dts_sorted[-1]))
    n = len(sim)
    zeroed = sum(1 for r in sim if r.get("zeroed"))
    held = sum(1 for r in sim
               if not (r.get("ik") or {}).get("achieved", (r.get("ik") or {}).get("ok", True)))
    atlim = sum(1 for r in sim
                if (r.get("ik") or {}).get("at_limit")
                or ((r.get("ik") or {}).get("v_limit") and
                    (r.get("ik") or {}).get("v_absmax", 0) > 0.9 * (r.get("ik") or {})["v_limit"]))
    nearsing = sum(1 for r in sim
                   if (r.get("sigma_min_tcp") or 9) < sigma_th)
    external = sum(1 for i in range(1, n)
                   if max(abs(a - b) for a, b in
                          zip(sim[i]["q_before"], sim[i - 1]["q_after"])) > 1e-6)
    print("  合计 %d 帧：死区置零 %.1f%%  策略持位 %.1f%%  "
          "速度≥90%%上限 %.1f%%  σ_min<%.3f %.1f%%  帧外断点 %d 次"
          % (n, 100.0 * zeroed / n, 100.0 * held / n, 100.0 * atlim / n,
             sigma_th, 100.0 * nearsing / n, external))
    print("  ↑「速度≥90%上限」与「σ_min 小」同时高 ⇒ 跟手感不足的主因是奇异区里"
          "任务的**可及速度不够**（被盒削），不是 IK 解错")

    # tracking error: 到位的末端 vs 本帧命令的末端目标。这是"牺牲工具精度"的**直接**度量
    # —— 操纵度任务（--manip-weight）会与 FrameTask 抢优先级，只有这个数能证明亏没亏。
    errs = [max(abs(a - b) for a, b in zip(r["ee_after_p"], r["tgt_p"]))
            for r in sim if r.get("tgt_p") and r.get("ee_after_p")]
    if errs:
        errs_sorted = sorted(errs)
        print("  末端跟踪误差 |ee_after − ee_target|: 中位 %.5f m  p90 %.5f m  max %.5f m"
              "（仅 pin/pybullet 有目标；对比两次会话即知 --manip-weight 亏了多少精度）"
              % (errs_sorted[len(errs) // 2], errs_sorted[int(len(errs) * 0.9)],
                 errs_sorted[-1]))

    print("\n=== 各序列自身的最大跳变（谁先跳谁嫌疑大）===")
    for label, rows, key in (("sim 关节 q_after", sim, "q_after"),
                             ("wire target(server 收到)", fol, "target"),
                             ("wire cmd_q(slew 后)", fol, "cmd_q"),
                             ("真机实测 q", fol, "q")):
        if not rows:
            print("  %-26s (无数据)" % label)
            continue
        tj = top_jumps(rows, key, k=3)
        if not tj:
            print("  %-26s (无有效序列)" % label)
            continue
        print("  %-26s 前3大 |Δq|∞: %s"
              % (label, ", ".join("%.3f" % v for v, _ in tj)))

    print("\n=== sim 流里最大的 %d 次关节跳变（含归属）===" % top)
    for val, i in top_jumps(sim, "q_after", k=top):
        rec = sim[i]
        prev_rec = sim[i - 1] if i > 0 else None
        fol_near = nearest(fol, rec["t"]) if fol else None
        print("\n  #%d  t=%.3f  |Δq|∞=%.4f rad  σ_min(TCP)=%s" %
              (rec.get("i", i), rec["t"],
               val, "%.4f" % rec["sigma_min_tcp"] if rec.get("sigma_min_tcp") is not None else "?"))
        print("     手: |Δp|=%.5f m |Δθ|=%.5f rad   死区置零=%s   resting=%s"
              % (rec.get("mag_t", 0), rec.get("mag_r", 0),
                 rec.get("zeroed"), rec.get("resting")))
        print("     帧内 Δq = [%s]" % " ".join("%+.3f" % (a - b) for a, b in
                                               zip(rec["q_after"], rec["q_before"])))
        print("     ik  = %s" % json.dumps(rec.get("ik") or {}, ensure_ascii=False))
        if fol_near:
            print("     真机: state=%s flags=%s following=%s"
                  % (fol_near.get("state"), fol_near.get("flags"),
                     fol_near.get("following")))
        for w in verdict(rec, fol_near, prev_rec, sigma_th):
            print("     ⇒ %s" % w)
    return 0


def main():
    ap = argparse.ArgumentParser(description="遥操数据探针（采样 + 归属报告）")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("sample", help="按频率采集 follow server 的 ping（真机侧真值）")
    s.add_argument("--addr", default="127.0.0.1:9002", help="follow server host:port")
    s.add_argument("--out", required=True, help="输出 JSONL 路径")
    s.add_argument("--hz", type=float, default=50.0, help="采样频率")

    r = sub.add_parser("report", help="把 sim 流与 follow 流合并出归属报告")
    r.add_argument("--sim", required=True, help="sim 侧 JSONL（ws_echo_server --probe）")
    r.add_argument("--follow", default=None, help="follow 侧 JSONL（probe.py sample）")
    r.add_argument("--top", type=int, default=12, help="列出前多少次跳变")
    r.add_argument("--sigma", type=float, default=0.05, help="近奇异告警阈值")

    args = ap.parse_args()
    if args.cmd == "sample":
        smp = FollowSampler(args.addr, args.out, args.hz)
        smp.start()
        print("[探针] 采样中 → %s   (Ctrl-C 结束)" % args.out, flush=True)
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            smp.stop()
            print("\n[探针] 已停止, 写至 %s" % args.out)
        return 0
    return report(args.sim, args.follow, args.top, args.sigma)


if __name__ == "__main__":
    sys.exit(main())

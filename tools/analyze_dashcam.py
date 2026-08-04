#!/usr/bin/env python3
"""行车记录仪分析工具。

用法:
    python3 tools/analyze_dashcam.py logs/dashcam_20260803_221746.csv

输出:
    - 按键事件表(从同目录 _events.log 读取)
    - pitch/roll/速度统计、pitch 大摆动的时段
    - 按键持续时段的姿态表现(例如 W 按住期间 pitch 峰峰值)
    可选 --plot 画 pitch/roll/keys 曲线。
"""

import argparse
import csv
import glob
import math
import os


def load_events(csv_path):
    ev_path = csv_path.replace(".csv", "_events.log")
    if not os.path.exists(ev_path):
        return []
    events = []
    for line in open(ev_path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) >= 3:
            events.append((float(parts[0]), parts[1], parts[2]))
    return events


def held_intervals(events, key):
    """把按下/松开事件还原成 (t0,t1) 持续区间。"""
    intervals = []
    start = None
    for t, k, act in events:
        if k != key:
            continue
        if act == "PRESS" and start is None:
            start = t
        elif act == "RELEASE" and start is not None:
            intervals.append((start, t))
            start = None
    if start is not None:
        intervals.append((start, 1e9))
    return intervals


def stats(name, vals):
    if not vals:
        return f"{name}: n=0"
    avg = sum(vals) / len(vals)
    pk = max(abs(v) for v in vals)
    return f"{name}: n={len(vals)} mean={avg:+.2f} min={min(vals):+.2f} max={max(vals):+.2f} peak|.|={pk:.2f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--pitch-thresh", type=float, default=8.0,
                    help="pitch 摆动分析阈值(度)")
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.csv)))
    if not rows:
        print("empty csv")
        return
    events = load_events(args.csv)

    print(f"== {os.path.basename(args.csv)}  rows={len(rows)} events={len(events)}")
    print("按键事件:")
    for t, k, act in events:
        print(f"  t={t:7.2f}  {k:1s} {act}")

    for key, name in (("W", "前进"), ("S", "后退"), ("A", "左转"), ("D", "右转")):
        ivs = held_intervals(events, key)
        if not ivs:
            continue
        print(f"\n{key}({name}) 持续 {len(ivs)} 段:")
        for i, (t0, t1) in enumerate(ivs[:8]):
            seg = [r for r in rows if t0 <= float(r["sim_t"]) <= t1]
            if not seg:
                continue
            pit = [float(r["pit_deg"]) for r in seg]
            roll = [float(r["roll_deg"]) for r in seg]
            vh = [float(r["body_v"]) for r in seg]
            xh = [float(r["xh"]) for r in seg]
            print(f"  [{i}] t={t0:.2f}-{min(t1, float(seg[-1]['sim_t'])):.2f}s "
                  f"pitch 峰峰={max(pit)-min(pit):.1f}° peak|pit|={max(abs(v) for v in pit):.1f}° "
                  f"vh={min(vh):+.2f}..{max(vh):+.2f} xh={xh[0]:+.2f}->{xh[-1]:+.2f}")

    print("\n全段统计:")
    print(stats("pitch", [float(r["pit_deg"]) for r in rows]))
    print(stats("roll", [float(r["roll_deg"]) for r in rows]))
    print(stats("body_v", [float(r["body_v"]) for r in rows]))
    print(stats("tp", [float(r["tp_deg"]) for r in rows]))

    # pitch 大摆动时段: |pitch| 超过阈值且变化剧烈
    print(f"\npitch>={args.pitch_thresh}° 的时段:")
    active = False
    start = None
    for r in rows:
        p = abs(float(r["pit_deg"]))
        if p >= args.pitch_thresh and not active:
            active = True
            start = float(r["sim_t"])
        elif p < args.pitch_thresh and active:
            print(f"  {start:.2f}s - {float(r['sim_t']):.2f}s "
                  f"(keys={r['keys']}, manual={r['manual']}, vh={r['body_v']})")
            active = False
    if active:
        print(f"  {start:.2f}s - end")

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("no matplotlib")
            return
        t = [float(r["sim_t"]) for r in rows]
        pit = [float(r["pit_deg"]) for r in rows]
        roll = [float(r["roll_deg"]) for r in rows]
        vh = [float(r["body_v"]) for r in rows]
        keys = [r["keys"] for r in rows]
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        axes[0].plot(t, pit, label="pitch"); axes[0].axhline(0, color="k", lw=0.5)
        axes[0].plot(t, roll, label="roll"); axes[0].legend(); axes[0].set_ylabel("deg")
        axes[1].plot(t, vh); axes[1].set_ylabel("body_v (m/s)")
        for r in rows:
            k = r["keys"]
            if k != "-":
                axes[2].axvspan(float(r["sim_t"]), float(r["sim_t"]) + 0.05,
                                color="orange", alpha=0.5)
        axes[2].set_ylabel("keys held"); axes[2].set_xlabel("sim_t (s)")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()

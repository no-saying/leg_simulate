"""Run five-second manual commands and report measured motion from CSV."""
import csv
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
DEST = ROOT / "logs" / "manual_verification"
DEST.mkdir(parents=True, exist_ok=True)


def run(name, sequence, duration, windows, extra_env=None):
    env = dict(os.environ, MANUAL_TEST=sequence, MIXDBG="1", SWDBG="1")
    env.update(extra_env or {})
    result = subprocess.run([str(ROOT / "controller"), "--headless", str(duration)],
                            cwd=ROOT, env=env, capture_output=True, text=True, check=True)
    (DEST / (name + ".log")).write_text(result.stdout + result.stderr)
    path = re.search(r"\[Dashcam\] (logs/.*)_events.log", result.stdout)
    if not path:
        raise RuntimeError("Missing CSV log")
    source = ROOT / (path.group(1) + ".csv")
    shutil.copyfile(source, DEST / (name + ".csv"))
    with source.open() as f:
        rows = [{k: float(v) for k, v in row.items() if k != "keys"}
                for row in csv.DictReader(f)]
    summaries = []
    failures = []
    for label, start, end in windows:
        chunk = [r for r in rows if start <= r["sim_t"] < end]
        first = min(rows, key=lambda r: abs(r["sim_t"] - start))
        last = min(rows, key=lambda r: abs(r["sim_t"] - end))
        tail = [r for r in chunk if r["sim_t"] >= end - 1]
        delta_yaw = sum((b["yaw_deg"] - a["yaw_deg"] + 180) % 360 - 180
                        for a, b in zip(chunk, chunk[1:]))
        speed = sum(math.hypot(b["bx"] - a["bx"], b["by"] - a["by"])
                    for a, b in zip(tail, tail[1:])) / max(tail[-1]["sim_t"] - tail[0]["sim_t"], .001)
        path_length = sum(math.hypot(b["bx"] - a["bx"], b["by"] - a["by"])
                          for a, b in zip(chunk, chunk[1:]))
        excursion = max(math.hypot(r["bx"] - first["bx"], r["by"] - first["by"])
                        for r in chunk)
        forward_speed = sum((b["bx"] - a["bx"]) * math.cos(math.radians(a["yaw_deg"])) +
                            (b["by"] - a["by"]) * math.sin(math.radians(a["yaw_deg"]))
                            for a, b in zip(tail, tail[1:])) / max(tail[-1]["sim_t"] - tail[0]["sim_t"], .001)
        summary = dict(command=label, displacement_m=round(math.hypot(last["bx"] - first["bx"], last["by"] - first["by"]), 3),
                       dx_m=round(last["bx"] - first["bx"], 3), yaw_delta_deg=round(delta_yaw, 1),
                       tail_body_speed_ms=round(speed, 3),
                       tail_forward_speed_ms=round(forward_speed, 3),
                       path_length_m=round(path_length, 3), max_excursion_m=round(excursion, 3),
                       tail_yaw_rate=round(sum(r["yaw_rate"] for r in tail) / len(tail), 3),
                       tail_wheel_speed=round(sum(r["wv_ms"] for r in tail) / len(tail), 3),
                       leg_len_m=round(last["leg_len"], 4), leg_target=round(last["le_man"], 4),
                       max_pitch_deg=round(max(abs(r["pit_deg"]) for r in chunk), 2),
                       max_roll_deg=round(max(abs(r["roll_deg"]) for r in chunk), 2),
                       swing_ref_span_m=round(max(r["sw"] for r in chunk) - min(r["sw"] for r in chunk), 4),
                       lqr_mean_mix=round(sum(r["lqr_mix"] for r in chunk) / len(chunk), 3),
                       fallen=bool(max(r["fallen"] for r in chunk)))
        if summary["fallen"] or max(summary["max_pitch_deg"], summary["max_roll_deg"]) > 10:
            failures.append(label + ": lost balance")
        if label in ("W", "S"):
            direction = -1 if label == "W" else 1
            if forward_speed * direction < .3 or path_length < .5:
                failures.append(label + ": no sustained translation in commanded direction")
        if label in ("A", "D", "spin", "spin_reverse"):
            direction = -1 if label in ("D", "spin_reverse") else 1
            if summary["tail_yaw_rate"] * direction < .4 or delta_yaw * direction < 60:
                failures.append(label + ": no sustained rotation in commanded direction")
            if label in ("spin", "spin_reverse") and excursion > .2:
                failures.append(label + ": spin drift exceeds 0.2 m")
        summaries.append(summary)
        print(name, json.dumps(summary), flush=True)
    (DEST / (name + ".json")).write_text(json.dumps(summaries, indent=2))
    if failures:
        raise AssertionError(name + ": " + "; ".join(failures))
    return summaries


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "individual"
    if mode == "sequence":
        run("sequence", "n@3,f@8,l@13,b@18,r@23,u@26,d@30,ds@35,d@38", 38,
            [("W", 3, 8), ("A", 8, 13), ("S", 13, 18), ("D", 18, 23),
             ("E", 23, 26), ("Q", 26, 30), ("spin", 30, 35), ("release", 35, 38)])
    elif mode in ("spin", "spin-long"):
        spin_duration = 20 if mode == "spin-long" else 5
        for extension in (-.04, -.02, 0, .02, .04):
            height = "u" if extension < 0 else "d" if extension > 0 else "n"
            for direction, command in ((1, "s"), (-1, "z")):
                end = 10 + spin_duration
                run(f"{mode}_{extension}_{direction}",
                    f"n@3,{height}@10,{command}@{end},n@{end + 5}", end + 5,
                    [("spin" if direction > 0 else "spin_reverse", 10, end),
                     ("release", end + 2, end + 5)], {"LE_EXT": str(abs(extension))})
    elif mode == "range":
        for height in ("u", "d"):
            for key, command in (("W", "f"), ("A", "l"), ("S", "b"), ("D", "r"), ("spin", "s")):
                run(height + "_" + key,
                    f"n@3,{height}@10,{command}@15,n@20", 20,
                    [(key, 10, 15), ("release", 17, 20)])
    elif mode in ("stability", "stability-all"):
        extensions = (-.04, -.02, 0, .02, .04)
        if mode == "stability-all":
            extensions = [round(-.04 + .08 * i / 17, 6) for i in range(18)]
        for extension in extensions:
            height = "u" if extension < 0 else "d" if extension > 0 else "n"
            env = {"LE_EXT": str(abs(extension))}
            name = "static_" + str(extension)
            result = run(name, f"n@3,{height}@20", 20, [("idle", 15, 20)], env)
            if result[0]["displacement_m"] > .08 or result[0]["tail_body_speed_ms"] > .08:
                raise AssertionError(name + ": stationary drift")
            for direction in (-1, 1):
                pulse_env = dict(env, PITCH_PULSE_TEST=f"10,0.15,{direction * .15}")
                name = f"pulse_{extension}_{direction}"
                result = run(name, f"n@3,{height}@22", 22,
                             [("disturbance", 10, 13), ("recovered", 17, 22)], pulse_env)
                if result[-1]["tail_body_speed_ms"] > .08 or result[-1]["max_pitch_deg"] > 1:
                    raise AssertionError(name + ": failed to settle after disturbance")
    else:
        for key, command in (("W", "f"), ("A", "l"), ("S", "b"), ("D", "r"), ("spin", "s")):
            run(key, f"n@3,{command}@8,n@11", 11, [(key, 3, 8), ("release", 8, 11)])
        run("height", "n@3,u@6,d@10,d@13", 13, [("E", 3, 6), ("Q", 6, 10)])

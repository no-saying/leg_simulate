#!/usr/bin/env python3
"""近场原点收敛参数扫描。

用法:
    python3 sweep_nearfield.py [--dur 30] [--tag sweep1]

每个配置通过 EXTRA_CFLAGS 以 -D 覆盖 control_config.h 里的宏，重新编译后
跑一次无界面仿真，解析日志并输出汇总指标。
"""

import argparse
import glob
import os
import re
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CTRL = os.path.join(ROOT, "control")
BIN = os.path.join(ROOT, "controller")
LOGS = os.path.join(ROOT, "logs")


def parse_log(path):
    rows = []
    pat = re.compile(
        r"\[t=([\d.]+)\] pit=([+-][\d.]+)d .*?"
        r"xh=([+-][\d.]+) es=([+-][\d.]+) ox=([+-][\d.]+) "
        r"oy=([+-][\d.]+) of=([+-][\d.]+) vh=([+-][\d.]+) "
        r"tp=([+-][\d.]+)d .*?pt=([\d.]+)"
    )
    for line in open(path, errors="ignore"):
        m = pat.search(line)
        if m:
            rows.append(
                (
                    float(m.group(1)),
                    float(m.group(2)),
                    float(m.group(3)),
                    float(m.group(8)),
                    float(m.group(9)),
                    float(m.group(10)),
                )
            )
    return rows


def metrics(rows, dur):
    if not rows:
        return {}
    x = [r[2] for r in rows]
    v = [r[3] for r in rows]
    t_end = max(r[0] for r in rows)
    out = {
        "max_abs_x_all": max(abs(vv) for vv in x),
        "max_abs_v_all": max(abs(vv) for vv in v),
        "final_x": x[-1],
    }
    for lo in (15, 20, 25):
        seg = [r for r in rows if r[0] >= lo]
        if seg:
            sx = [r[2] for r in seg]
            sv = [r[3] for r in seg]
            out[f"max_abs_x_tail{lo}"] = max(abs(vv) for vv in sx)
            out[f"mean_abs_x_tail{lo}"] = sum(abs(vv) for vv in sx) / len(sx)
            out[f"max_abs_v_tail{lo}"] = max(abs(vv) for vv in sv)
            out[f"frac_in_0p1_tail{lo}"] = sum(1 for vv in sx if abs(vv) < 0.10) / len(sx)
            out[f"frac_in_0p15_tail{lo}"] = sum(1 for vv in sx if abs(vv) < 0.15) / len(sx)
    # 首次收敛时间: 之后一直 |x|<0.15
    conv_t = None
    for i in range(len(rows) - 1, -1, -1):
        if abs(rows[i][2]) >= 0.15:
            conv_t = rows[i][0]
            break
    out["last_excursion_0p15_t"] = conv_t if conv_t is not None else 0.0
    return out


def build(defines):
    cflags = " ".join(f"-D{k}={v}" for k, v in defines.items())
    env = dict(os.environ)
    env["EXTRA_CFLAGS"] = cflags
    r = subprocess.run(
        ["make", "-B", "-C", CTRL],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        env=env,
    )
    if r.returncode != 0:
        raise RuntimeError("build failed: " + r.stderr.decode())


_build_lock = threading.Lock()


def run_one(name, defines, dur):
    with _build_lock:
        build(defines)
        binpath = os.path.join(LOGS, f"bin_{name}")
        subprocess.run(["cp", BIN, binpath], check=True)
    log = os.path.join(LOGS, f"sweep_{name}_{dur}s.log")
    with open(log, "wb") as f:
        subprocess.run(
            [binpath, "--headless", str(dur)],
            cwd=ROOT,
            stdout=f,
            stderr=subprocess.DEVNULL,
            check=True,
        )
    rows = parse_log(log)
    return name, defines, metrics(rows, dur)


def fmt(name, m, dur):
    t20 = m.get("max_abs_x_tail20")
    t25 = m.get("max_abs_x_tail25")
    mean25 = m.get("mean_abs_x_tail25")
    conv = m.get("last_excursion_0p15_t")
    v25 = m.get("max_abs_v_tail25")
    frac = m.get("frac_in_0p1_tail25")
    return (
        f"{name:32s} all={m['max_abs_x_all']:+.2f} "
        f"t15={m.get('max_abs_x_tail15', float('nan')):+.2f} "
        f"t20={t20:+.2f} t25={t25:+.2f} mean25={mean25:.2f} "
        f"v25={v25:.2f} frac<0.1={frac:.2f} last>0.15={conv:.1f}s"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dur", type=int, default=30)
    ap.add_argument("--tag", default="near")
    ap.add_argument("--jobs", type=int, default=3)
    args = ap.parse_args()

    configs = []

    # 基准
    configs.append(("baseline", {}))

    # A: 近场刹车目标俯仰上限
    for mx in ("0.060f", "0.100f", "0.200f", "0.250f"):
        configs.append((f"stopmax_{mx}", {"SIMPLE_ORIGIN_STOP_PITCH_MAX": mx}))

    # B: neutral 滤波时间常数
    for tau in ("0.120f", "0.300f", "0.500f"):
        configs.append((f"tau_{tau}", {"SIMPLE_NEUTRAL_TAU": tau}))

    # C: park 阻尼
    for pd in ("18.0f", "24.0f"):
        configs.append((f"park_{pd}", {"SIMPLE_PARK_VEL_DAMP": pd}))

    # D: 近场直接轮扭矩
    configs.append(
        (
            "dirtau",
            {
                "SIMPLE_ORIGIN_DIRECT_TAU_ENABLE": "1",
                "SIMPLE_ORIGIN_STOP_TAU_KP": "10.0f",
                "SIMPLE_ORIGIN_STOP_TAU_MAX": "4.5f",
                "SIMPLE_ORIGIN_CREEP_TAU_KP": "14.0f",
                "SIMPLE_ORIGIN_CREEP_TAU_MAX": "2.0f",
            },
        )
    )
    configs.append(
        (
            "dirtau_soft",
            {
                "SIMPLE_ORIGIN_DIRECT_TAU_ENABLE": "1",
                "SIMPLE_ORIGIN_STOP_TAU_KP": "14.0f",
                "SIMPLE_ORIGIN_STOP_TAU_MAX": "6.0f",
                "SIMPLE_ORIGIN_CREEP_TAU_KP": "18.0f",
                "SIMPLE_ORIGIN_CREEP_TAU_MAX": "2.5f",
            },
        )
    )

    # E: 近场窗口半径
    for pos in ("0.220f", "0.280f"):
        configs.append((f"stoppos_{pos}", {"SIMPLE_ORIGIN_STOP_POS": pos}))

    results = []
    with ThreadPoolExecutor(max_workers=args.jobs) as ex:
        futures = [ex.submit(run_one, name, defs, args.dur) for name, defs in configs]
        for fut in futures:
            name, defs, m = fut.result()
            results.append((name, m))
            print(fmt(name, m, args.dur), flush=True)

    results.sort(key=lambda kv: kv[1].get("max_abs_x_tail25", 1e9))
    print("\n=== 按 tail25 max|x| 排序 ===")
    for name, m in results:
        print(fmt(name, m, args.dur))


if __name__ == "__main__":
    main()

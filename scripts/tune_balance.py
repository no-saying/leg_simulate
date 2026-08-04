#!/usr/bin/env python3
"""平衡控制器的分阶段自动调参脚本。

阶段必须拆开:
  pitch  - 先调 pitch 内环，此时关闭 yaw 和回原点。
  origin - pitch 合格后再调位置到 target_pitch 的规划。
  yaw    - pitch 和 origin 都能站住后再调 yaw。

脚本通过编译期 CFLAGS 注入参数，重编译控制器，运行 headless 仿真，
解析标准日志并给候选参数排序。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
LOG_DIR = ROOT / "logs" / "tuning"

BASE_CFLAGS = ["-O2", "-Wall", "-Wextra"]

LOG_RE = re.compile(
    r"\[t=(?P<t>[0-9.]+)\].*?"
    r"pit=(?P<pit>[+-]?[0-9.]+)d.*?"
    r"roll=(?P<roll>[+-]?[0-9.]+)d.*?"
    r"yaw=(?P<yaw>[+-]?[0-9.]+)d ye=(?P<ye>[+-]?[0-9.]+)d.*?"
    r"yr=(?P<yr>[+-]?[0-9.]+).*?"
    r"wv=(?P<wv>[+-]?[0-9.]+).*?"
    r"bx=(?P<bx>[+-]?[0-9.]+) by=(?P<by>[+-]?[0-9.]+) bz=(?P<bz>[+-]?[0-9.]+).*?"
    r"xh=(?P<xh>[+-]?[0-9.]+).*?"
    r"vh=(?P<vh>[+-]?[0-9.]+).*?"
    r"wt=(?P<wt>[0-9.]+).*?sl=(?P<sl>[0-9.]+)"
)


@dataclass(frozen=True)
class Candidate:
    name: str
    flags: dict[str, str]


def fmt_flag(name: str, value: str) -> str:
    return f"-D{name}={value}"


def parse_define(text: str) -> tuple[str, str]:
    if "=" not in text:
        raise argparse.ArgumentTypeError(f"expected NAME=VALUE, got {text!r}")
    name, value = text.split("=", 1)
    if not name or not value:
        raise argparse.ArgumentTypeError(f"expected NAME=VALUE, got {text!r}")
    return name, value


def pitch_candidates() -> list[Candidate]:
    # 低/中控制权参数组。每次只动少数物理含义明确的旋钮，便于解释排序结果。
    raw = [
        ("p420d100v2t90r700", 420, 100, 2, 90, 700),
        ("p400d100v4t80r550", 400, 100, 4, 80, 550),
        ("p390d100v4t80r520", 390, 100, 4, 80, 520),
        ("p390d95v5t78r500", 390, 95, 5, 78, 500),
        ("p380d95v4t80r520", 380, 95, 4, 80, 520),
        ("p380d105v4t78r500", 380, 105, 4, 78, 500),
        ("p370d100v5t76r480", 370, 100, 5, 76, 480),
        ("p370d90v5t76r480", 370, 90, 5, 76, 480),
        ("p360d90v4t75r500", 360, 90, 4, 75, 500),
        ("p360d105v5t75r450", 360, 105, 5, 75, 450),
        ("p340d100v6t70r420", 340, 100, 6, 70, 420),
        ("p340d85v5t72r450", 340, 85, 5, 72, 450),
        ("p320d95v6t70r400", 320, 95, 6, 70, 400),
        ("p320d80v5t70r420", 320, 80, 5, 70, 420),
        ("p300d95v7t65r360", 300, 95, 7, 65, 360),
        ("p300d80v7t65r360", 300, 80, 7, 65, 360),
        ("p280d85v8t62r340", 280, 85, 8, 62, 340),
        ("p280d70v6t65r380", 280, 70, 6, 65, 380),
        ("p260d80v8t60r320", 260, 80, 8, 60, 320),
    ]
    out: list[Candidate] = []
    for name, kp, kd, vel, tau, rate in raw:
        flags = {
            "SIMPLE_PITCH_KP": f"{kp:.1f}f",
            "SIMPLE_PITCH_KD": f"{kd:.1f}f",
            "SIMPLE_VEL_DAMP": f"{vel:.1f}f",
            "SIMPLE_TAU_START": f"{tau:.1f}f",
            "SIMPLE_TAU_RUN": f"{tau:.1f}f",
            "SIMPLE_RATE_START": f"{rate:.1f}f",
            "SIMPLE_RATE_RUN": f"{rate:.1f}f",
            "SIMPLE_ORIGIN_ENABLE": "0",
            "SIMPLE_PITCH_RATE_TAU": "0.0f",
            "SIMPLE_YAW_MAX": "0.0f",
        }
        out.append(Candidate(name, flags))

    # 轮速压制阶段: yaw/origin 保持关闭，只改变 pitch 内环权限和共模速度阻尼/刹车。
    wheel_raw = [
        ("p380d95v8t76r460_park20_spin12", 380, 95, 8, 76, 460, 20, 12),
        ("p380d95v10t74r430_park24_spin16", 380, 95, 10, 74, 430, 24, 16),
        ("p370d95v10t72r400_park28_spin18", 370, 95, 10, 72, 400, 28, 18),
        ("p370d100v12t70r380_park30_spin22", 370, 100, 12, 70, 380, 30, 22),
        ("p360d95v12t68r360_park32_spin24", 360, 95, 12, 68, 360, 32, 24),
        ("p360d105v14t66r340_park34_spin28", 360, 105, 14, 66, 340, 34, 28),
        ("p340d100v14t64r320_park36_spin30", 340, 100, 14, 64, 320, 36, 30),
        ("p340d110v16t62r300_park38_spin34", 340, 110, 16, 62, 300, 38, 34),
        ("p320d105v18t60r280_park40_spin38", 320, 105, 18, 60, 280, 40, 38),
    ]
    for name, kp, kd, vel, tau, rate, park, spin in wheel_raw:
        flags = {
            "SIMPLE_PITCH_KP": f"{kp:.1f}f",
            "SIMPLE_PITCH_KD": f"{kd:.1f}f",
            "SIMPLE_VEL_DAMP": f"{vel:.1f}f",
            "SIMPLE_TAU_START": f"{tau:.1f}f",
            "SIMPLE_TAU_RUN": f"{tau:.1f}f",
            "SIMPLE_RATE_START": f"{rate:.1f}f",
            "SIMPLE_RATE_RUN": f"{rate:.1f}f",
            "SIMPLE_PARK_VEL_DAMP": f"{park:.1f}f",
            "SIMPLE_SPIN_DAMP": f"{spin:.1f}f",
            "SIMPLE_ORIGIN_ENABLE": "0",
            "SIMPLE_PITCH_RATE_TAU": "0.0f",
            "SIMPLE_YAW_MAX": "0.0f",
        }
        out.append(Candidate(name, flags))

    gov_raw = [
        ("gov_p380d95v4_g2s18_tau60min20", 380, 95, 4, 80, 520, 1.8, 2.0, 60, 20),
        ("gov_p380d95v4_g4s18_tau55min18", 380, 95, 4, 80, 520, 1.8, 4.0, 55, 18),
        ("gov_p380d95v4_g6s18_tau50min18", 380, 95, 4, 80, 520, 1.8, 6.0, 50, 18),
        ("gov_p380d95v5_g4s15_tau55min18", 380, 95, 5, 78, 500, 1.5, 4.0, 55, 18),
        ("gov_p370d95v5_g4s15_tau50min16", 370, 95, 5, 76, 480, 1.5, 4.0, 50, 16),
        ("gov_p370d100v5_g5s14_tau50min16", 370, 100, 5, 76, 460, 1.4, 5.0, 50, 16),
        ("gov_p360d100v6_g5s14_tau48min16", 360, 100, 6, 74, 440, 1.4, 5.0, 48, 16),
        ("gov_p360d105v6_g6s12_tau46min14", 360, 105, 6, 72, 420, 1.2, 6.0, 46, 14),
        ("gov_p340d105v7_g6s12_tau44min14", 340, 105, 7, 70, 400, 1.2, 6.0, 44, 14),
        ("gov_p340d110v8_g8s10_tau42min12", 340, 110, 8, 68, 380, 1.0, 8.0, 42, 12),
    ]
    for name, kp, kd, vel, tau, rate, start, gain, gov_tau, min_tau in gov_raw:
        flags = {
            "SIMPLE_PITCH_KP": f"{kp:.1f}f",
            "SIMPLE_PITCH_KD": f"{kd:.1f}f",
            "SIMPLE_VEL_DAMP": f"{vel:.1f}f",
            "SIMPLE_TAU_START": f"{tau:.1f}f",
            "SIMPLE_TAU_RUN": f"{tau:.1f}f",
            "SIMPLE_RATE_START": f"{rate:.1f}f",
            "SIMPLE_RATE_RUN": f"{rate:.1f}f",
            "SIMPLE_WHEEL_GOV_ENABLE": "1",
            "SIMPLE_WHEEL_GOV_START": f"{start:.2f}f",
            "SIMPLE_WHEEL_GOV_KP": f"{gain:.1f}f",
            "SIMPLE_WHEEL_GOV_TAU": f"{gov_tau:.1f}f",
            "SIMPLE_WHEEL_GOV_MIN_TAU": f"{min_tau:.1f}f",
            "SIMPLE_ORIGIN_ENABLE": "0",
            "SIMPLE_PITCH_RATE_TAU": "0.0f",
            "SIMPLE_YAW_MAX": "0.0f",
        }
        out.append(Candidate(name, flags))

    sched_raw = [
        ("sched_n260d70_far380d95_v4", 260, 70, 380, 95, 4, 80, 520, 0.035, 0.140),
        ("sched_n280d75_far380d95_v4", 280, 75, 380, 95, 4, 80, 520, 0.035, 0.140),
        ("sched_n300d80_far380d95_v4", 300, 80, 380, 95, 4, 80, 520, 0.035, 0.140),
        ("sched_n300d85_far390d100_v4", 300, 85, 390, 100, 4, 80, 520, 0.030, 0.150),
        ("sched_n320d85_far390d100_v5", 320, 85, 390, 100, 5, 78, 500, 0.030, 0.150),
        ("sched_n320d90_far400d105_v5", 320, 90, 400, 105, 5, 78, 500, 0.030, 0.160),
        ("sched_n300d95_far400d110_v5", 300, 95, 400, 110, 5, 78, 500, 0.025, 0.160),
        ("sched_n260d95_far380d110_v6", 260, 95, 380, 110, 6, 76, 480, 0.025, 0.160),
    ]
    for name, nkp, nkd, fkp, fkd, vel, tau, rate, near, far in sched_raw:
        flags = {
            "SIMPLE_GAIN_SCHED_ENABLE": "1",
            "SIMPLE_GAIN_NEAR_RAD": f"{near:.3f}f",
            "SIMPLE_GAIN_FAR_RAD": f"{far:.3f}f",
            "SIMPLE_PITCH_KP_NEAR": f"{nkp:.1f}f",
            "SIMPLE_PITCH_KD_NEAR": f"{nkd:.1f}f",
            "SIMPLE_PITCH_KP": f"{fkp:.1f}f",
            "SIMPLE_PITCH_KD": f"{fkd:.1f}f",
            "SIMPLE_VEL_DAMP": f"{vel:.1f}f",
            "SIMPLE_TAU_START": f"{tau:.1f}f",
            "SIMPLE_TAU_RUN": f"{tau:.1f}f",
            "SIMPLE_RATE_START": f"{rate:.1f}f",
            "SIMPLE_RATE_RUN": f"{rate:.1f}f",
            "SIMPLE_ORIGIN_ENABLE": "0",
            "SIMPLE_PITCH_RATE_TAU": "0.0f",
            "SIMPLE_YAW_MAX": "0.0f",
        }
        out.append(Candidate(name, flags))

    leg_raw = [
        ("leg_p380d95_l18i00d04", 380, 95, 4, 80, 520, 1.8, 0.0, 0.040, 8.0, 0.012),
        ("leg_p380d95_l16i00d04", 380, 95, 4, 80, 520, 1.6, 0.0, 0.040, 7.0, 0.012),
        ("leg_p380d95_l14i00d035", 380, 95, 4, 80, 520, 1.4, 0.0, 0.035, 6.0, 0.014),
        ("leg_p370d95_l16i00d04", 370, 95, 5, 78, 500, 1.6, 0.0, 0.040, 7.0, 0.012),
        ("leg_p370d100_l16i00d05", 370, 100, 5, 78, 500, 1.6, 0.0, 0.050, 7.0, 0.012),
        ("leg_p360d100_l14i00d05", 360, 100, 6, 76, 480, 1.4, 0.0, 0.050, 6.0, 0.014),
        ("leg_p360d105_l13i00d06", 360, 105, 6, 76, 460, 1.3, 0.0, 0.060, 6.0, 0.014),
        ("leg_p340d105_l12i00d06", 340, 105, 7, 74, 440, 1.2, 0.0, 0.060, 5.5, 0.016),
    ]
    for name, kp, kd, vel, tau, rate, lkp, lki, lkd, lmax, ldb in leg_raw:
        flags = {
            "SIMPLE_PITCH_KP": f"{kp:.1f}f",
            "SIMPLE_PITCH_KD": f"{kd:.1f}f",
            "SIMPLE_VEL_DAMP": f"{vel:.1f}f",
            "SIMPLE_TAU_START": f"{tau:.1f}f",
            "SIMPLE_TAU_RUN": f"{tau:.1f}f",
            "SIMPLE_RATE_START": f"{rate:.1f}f",
            "SIMPLE_RATE_RUN": f"{rate:.1f}f",
            "SIMPLE_ORIGIN_ENABLE": "0",
            "SIMPLE_PITCH_RATE_TAU": "0.0f",
            "SIMPLE_YAW_MAX": "0.0f",
            "LEG_ANGLE_KP": f"{lkp:.2f}f",
            "LEG_ANGLE_KI": f"{lki:.2f}f",
            "LEG_ANGLE_KD": f"{lkd:.3f}f",
            "LEG_ANGLE_MAX": f"{lmax:.1f}f",
            "LEG_ANGLE_DEADBAND": f"{ldb:.3f}f",
        }
        out.append(Candidate(name, flags))
    return out


def origin_candidates() -> list[Candidate]:
    # Second stage: position-to-target-pitch planning. Keep the current yaw
    # loop enabled so longitudinal target-pitch tuning matches the GUI/default
    # controller instead of drifting through uncontrolled heading changes.
    # The pitch loop baseline is the source default unless overridden with
    # --define. Gains here are deliberately stronger than the old placeholder
    # set because pitch-only drift is slow but accumulates past 0.5 m.
    raw = [
        ("origin_off", -1.0, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000),
        ("origin_neg_p030d020i004m030", -1.0, 0.030, 0.020, 0.004, 0.030, 0.014, 0.020, 0.020),
        ("origin_neg_p050d030i006m045", -1.0, 0.050, 0.030, 0.006, 0.045, 0.022, 0.028, 0.028),
        ("origin_neg_p080d040i010m070", -1.0, 0.080, 0.040, 0.010, 0.070, 0.034, 0.038, 0.034),
        ("origin_neg_p100d050i012m087", -1.0, 0.100, 0.050, 0.012, 0.087, 0.042, 0.046, 0.038),
        ("origin_pos_p020d015i003m025", 1.0, 0.020, 0.015, 0.003, 0.025, 0.010, 0.016, 0.016),
        ("origin_pos_p030d020i004m030", 1.0, 0.030, 0.020, 0.004, 0.030, 0.014, 0.020, 0.020),
        ("origin_pos_p040d025i005m040", 1.0, 0.040, 0.025, 0.005, 0.040, 0.018, 0.024, 0.024),
        ("origin_pos_p050d030i006m045", 1.0, 0.050, 0.030, 0.006, 0.045, 0.022, 0.028, 0.028),
        ("origin_pos_p060d035i006m050", 1.0, 0.060, 0.035, 0.006, 0.050, 0.026, 0.030, 0.030),
        ("origin_pos_p070d040i008m060", 1.0, 0.070, 0.040, 0.008, 0.060, 0.030, 0.034, 0.032),
        ("origin_pos_p080d045i010m070", 1.0, 0.080, 0.045, 0.010, 0.070, 0.034, 0.038, 0.034),
        ("origin_pos_legheavy_p060", 1.0, 0.060, 0.035, 0.006, 0.055, 0.050, 0.050, 0.045),
        ("origin_pos_legheavy_p080", 1.0, 0.080, 0.045, 0.008, 0.070, 0.060, 0.060, 0.050),
    ]
    return [
        Candidate(
            name,
            {
                "SIMPLE_ORIGIN_ENABLE": "1",
                "SIMPLE_ORIGIN_SIGN": f"{sgn:.1f}f",
                "SIMPLE_ORIGIN_ACTIVE_PITCH_RAD": "0.280f",
                "SIMPLE_ORIGIN_PITCH_KP": f"{kp:.3f}f",
                "SIMPLE_ORIGIN_PITCH_KD": f"{kd:.3f}f",
                "SIMPLE_ORIGIN_PITCH_KI": f"{ki:.3f}f",
                "SIMPLE_ORIGIN_PITCH_MAX": f"{mx:.3f}f",
                "SIMPLE_ORIGIN_LEG_KP": f"{lkp:.3f}f",
                "SIMPLE_ORIGIN_LEG_KD": f"{lkd:.3f}f",
                "SIMPLE_ORIGIN_LEG_MAX": f"{lmx:.3f}f",
            },
        )
        for name, sgn, kp, kd, ki, mx, lkp, lkd, lmx in raw
    ]


def yaw_candidates() -> list[Candidate]:
    raw = [
        ("yaw_pos06r08i00m12", 1.0, 0.60, 0.80, 0.00, 1.2, 0.8),
        ("yaw_pos08r08i00m14", 1.0, 0.80, 0.80, 0.00, 1.4, 0.8),
        ("yaw_pos10r09i00m15", 1.0, 1.00, 0.90, 0.00, 1.5, 0.8),
        ("yaw_pos12r10i00m18", 1.0, 1.20, 1.00, 0.00, 1.8, 0.8),
        ("yaw_pos14r10i00m20", 1.0, 1.40, 1.00, 0.00, 2.0, 0.8),
        ("yaw_pos16r11i00m22", 1.0, 1.60, 1.10, 0.00, 2.2, 0.8),
        ("yaw_pos10r09i04m15", 1.0, 1.00, 0.90, 0.04, 1.5, 1.0),
        ("yaw_pos12r10i04m18", 1.0, 1.20, 1.00, 0.04, 1.8, 1.0),
        ("yaw_pos14r10i04m20", 1.0, 1.40, 1.00, 0.04, 2.0, 1.0),
        ("yaw_pos16r11i04m22", 1.0, 1.60, 1.10, 0.04, 2.2, 1.0),
        ("yaw_pos12r10i08m18", 1.0, 1.20, 1.00, 0.08, 1.8, 1.0),
        ("yaw_pos14r11i08m20", 1.0, 1.40, 1.10, 0.08, 2.0, 1.0),
        ("yaw_pos16r12i08m24", 1.0, 1.60, 1.20, 0.08, 2.4, 1.0),
        ("yaw_pos18r12i08m26", 1.0, 1.80, 1.20, 0.08, 2.6, 1.0),
        ("yaw_pos20r14i08m28", 1.0, 2.00, 1.40, 0.08, 2.8, 1.0),
    ]
    return [
        Candidate(
            name,
            {
                "SIMPLE_ORIGIN_ENABLE": "0",
                "SIMPLE_YAW_SIGN": f"{sgn:.1f}f",
                "SIMPLE_YAW_KP": f"{pos_kp:.2f}f",
                "SIMPLE_YAW_KD": f"{rate_kp:.2f}f",
                "SIMPLE_YAW_POS_KP": f"{pos_kp:.2f}f",
                "SIMPLE_YAW_RATE_KP": f"{rate_kp:.2f}f",
                "SIMPLE_YAW_KI": f"{ki:.2f}f",
                "SIMPLE_YAW_MAX": f"{mx:.1f}f",
                "SIMPLE_YAW_I_MAX": f"{imx:.1f}f",
            },
        )
        for name, sgn, pos_kp, rate_kp, ki, mx, imx in raw
    ]


def candidates_for(stage: str) -> list[Candidate]:
    if stage == "pitch":
        return pitch_candidates()
    if stage == "origin":
        return origin_candidates()
    if stage == "yaw":
        return yaw_candidates()
    raise ValueError(stage)


def run(cmd: list[str], *, cwd: Path, stdout=None) -> None:
    proc = subprocess.run(cmd, cwd=cwd, stdout=stdout, stderr=subprocess.STDOUT, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"command failed ({proc.returncode}): {' '.join(cmd)}")


def build(flags: dict[str, str]) -> None:
    cflags = BASE_CFLAGS + [fmt_flag(k, v) for k, v in sorted(flags.items())]
    run(["make", "-B", "-C", "control", f"CFLAGS={' '.join(cflags)}"], cwd=ROOT, stdout=subprocess.DEVNULL)


def run_headless(duration: float, log_path: Path) -> None:
    with log_path.open("w", encoding="utf-8") as f:
        run(["./controller", "--headless", f"{duration:.3f}"], cwd=ROOT, stdout=f)


def parse_log(path: Path, tail_window: float) -> dict[str, float]:
    rows: list[dict[str, float]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = LOG_RE.search(line)
        if not m:
            continue
        row = {k: float(v) for k, v in m.groupdict().items()}
        row["r"] = math.hypot(row["bx"], row["by"])
        rows.append(row)

    if not rows:
        return {"ok": 0.0, "score": float("inf"), "samples": 0.0}

    final = rows[-1]
    end_t = final["t"]
    tail = [r for r in rows if r["t"] >= end_t - tail_window]
    if not tail:
        tail = rows

    def max_abs(key: str, data=rows) -> float:
        return max(abs(r[key]) for r in data)

    def mean_abs(key: str, data=tail) -> float:
        return sum(abs(r[key]) for r in data) / len(data)

    def rms(key: str, data=tail) -> float:
        return math.sqrt(sum(r[key] * r[key] for r in data) / len(data))

    metrics = {
        "ok": 1.0,
        "samples": float(len(rows)),
        "final_t": final["t"],
        "final_r": final["r"],
        "final_bx": final["bx"],
        "final_by": final["by"],
        "final_bz": final["bz"],
        "final_pitch": final["pit"],
        "final_roll": final["roll"],
        "final_ye": final["ye"],
        "final_yr": final["yr"],
        "final_xh": final["xh"],
        "final_vh": final["vh"],
        "final_wv": final["wv"],
        "max_abs_pitch": max_abs("pit"),
        "max_abs_roll": max_abs("roll"),
        "max_abs_ye": max_abs("ye"),
        "max_abs_yr": max_abs("yr"),
        "max_abs_wv": max_abs("wv"),
        "max_r": max(r["r"] for r in rows),
        "min_bz": min(r["bz"] for r in rows),
        "max_bz": max(r["bz"] for r in rows),
        "tail_rms_pitch": rms("pit"),
        "tail_abs_roll": mean_abs("roll"),
        "tail_abs_yr": mean_abs("yr"),
        "tail_abs_wv": mean_abs("wv"),
        "tail_abs_vh": mean_abs("vh"),
        "tail_rms_xh": rms("xh"),
        "tail_rms_r": rms("r"),
    }
    return metrics


def score(stage: str, m: dict[str, float]) -> float:
    if not m.get("ok"):
        return float("inf")

    fall_penalty = 0.0
    if m["min_bz"] < 0.035:
        fall_penalty += 800.0 * (0.035 - m["min_bz"])
    if m["max_bz"] > 0.22:
        fall_penalty += 250.0 * (m["max_bz"] - 0.22)
    if m["max_abs_pitch"] > 65.0:
        fall_penalty += 6.0 * (m["max_abs_pitch"] - 65.0)
    if m["max_abs_roll"] > 35.0:
        fall_penalty += 4.0 * (m["max_abs_roll"] - 35.0)

    if stage == "pitch":
        return (
            3.5 * m["tail_rms_pitch"]
            + 0.70 * m["max_abs_pitch"]
            + 0.28 * m["tail_abs_wv"]
            + 0.080 * m["max_abs_wv"]
            + 0.05 * m["tail_abs_vh"]
            + fall_penalty
        )
    if stage == "origin":
        return (
            18.0 * m["tail_rms_xh"]
            + 8.0 * abs(m["final_xh"])
            + 1.5 * m["tail_rms_pitch"]
            + 0.06 * m["tail_abs_wv"]
            + 0.40 * m["tail_rms_r"]
            + fall_penalty
        )
    if stage == "yaw":
        return (
            2.5 * m["tail_abs_yr"]
            + 0.25 * abs(m["final_ye"])
            + 1.5 * m["tail_rms_pitch"]
            + 0.04 * m["tail_abs_wv"]
            + fall_penalty
        )
    raise ValueError(stage)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=["pitch", "origin", "yaw"], default="pitch")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--tail-window", type=float, default=3.0)
    parser.add_argument("--limit", type=int, default=0, help="run only the first N candidates")
    parser.add_argument("--define", action="append", type=parse_define, default=[], help="extra compile define NAME=VALUE")
    parser.add_argument("--top", type=int, default=6)
    parser.add_argument("--leave-candidate-binary", action="store_true", help="do not rebuild the source-default binary after tuning")
    args = parser.parse_args(argv)

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    stage_dir = LOG_DIR / f"{args.stage}_{stamp}"
    stage_dir.mkdir(parents=True, exist_ok=True)

    extra_flags = dict(args.define)
    candidates = candidates_for(args.stage)
    if args.limit > 0:
        candidates = candidates[: args.limit]

    rows: list[dict[str, object]] = []
    for i, cand in enumerate(candidates, 1):
        flags = dict(cand.flags)
        flags.update(extra_flags)
        print(f"[{i}/{len(candidates)}] build/run {cand.name}", flush=True)
        log_path = stage_dir / f"{cand.name}_{args.duration:.1f}s.log"
        try:
            build(flags)
            run_headless(args.duration, log_path)
            metrics = parse_log(log_path, args.tail_window)
            cand_score = score(args.stage, metrics)
        except Exception as exc:  # keep batch running and record failure
            metrics = {"ok": 0.0, "error": str(exc)}
            cand_score = float("inf")
        row: dict[str, object] = {
            "rank": 0,
            "name": cand.name,
            "score": cand_score,
            "log": str(log_path.relative_to(ROOT)),
            "flags": " ".join(fmt_flag(k, v) for k, v in sorted(flags.items())),
        }
        row.update(metrics)
        rows.append(row)

    rows.sort(key=lambda r: float(r["score"]))
    for idx, row in enumerate(rows, 1):
        row["rank"] = idx

    csv_path = stage_dir / "summary.csv"
    fieldnames = sorted({key for row in rows for key in row.keys()})
    preferred = ["rank", "name", "score", "log", "flags"]
    fieldnames = preferred + [f for f in fieldnames if f not in preferred]
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nsummary: {csv_path.relative_to(ROOT)}")
    for row in rows[: args.top]:
        print(
            "#{rank} {name} score={score:.2f} "
            "tail_pit={tail_rms_pitch:.1f} max_pit={max_abs_pitch:.1f} "
            "tail_wv={tail_abs_wv:.1f} max_wv={max_abs_wv:.1f} "
            "tail_yr={tail_abs_yr:.1f} final_r={final_r:.2f} log={log}".format(**row)
        )
    if not args.leave_candidate_binary:
        print("\nrebuilding source-default controller", flush=True)
        build({})
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

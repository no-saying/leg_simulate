#!/usr/bin/env python3
"""
模型拆分: 把 12 状态 x 5 输入的整体模型拆成两个物理子系统
================================================================
整体辨识模型的耦合通道(wheel_diff_v 等) R² 低(≈0.5), 叠加 LQR 会
破坏三环 yaw/roll 稳定(实测 yaw 自转失控)。按物理子系统拆分后:

  1. pitch 子系统  (7 状态, 3 输入)
       x = [pitch, pitch_rate, leg_swing, leg_swing_rate,
            leg_len, leg_len_rate, wheel_v]
       u = [wheel_common_tau, leg_swing_ref, leg_len_ref]
       -> LQR 只输出 wheel_tau / leg_swing / leg_len (可信通道)

  2. roll/差速子系统 (5 状态, 2 输入)
       x = [roll, roll_rate, leg_len_diff, leg_len_diff_rate, wheel_diff_v]
       u = [wheel_diff_tau, leg_len_diff_ref]
       -> 独立 LQR, 只输出 yaw_tau / leg_len_diff

  3. 按腿长分段: 每个子系统分别在 9 个腿长段内拟合 A/B/K。

输出: logs/split_models_latest.npz / .json
"""

import json
import os
import time

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(PROJECT_ROOT, "logs", "three_loop_model_latest.npz")
OUT_JSON = os.path.join(PROJECT_ROOT, "logs", "split_models_latest.json")
OUT_NPZ = os.path.join(PROJECT_ROOT, "logs", "split_models_latest.npz")

# 状态/输入索引(与 identify_three_loop.py 的 IDX 一致)
IDX = {
    "pitch": 0, "pitch_rate": 1, "roll": 2, "roll_rate": 3,
    "leg_swing": 4, "leg_swing_rate": 5,
    "leg_len": 6, "leg_len_rate": 7,
    "leg_len_diff": 8, "leg_len_diff_rate": 9,
    "wheel_v": 10, "wheel_diff_v": 11,
}

PITCH_COLS = [IDX["pitch"], IDX["pitch_rate"], IDX["leg_swing"],
              IDX["leg_swing_rate"], IDX["leg_len"], IDX["leg_len_rate"],
              IDX["wheel_v"]]
PITCH_INPUTS = [0, 2, 3]     # wheel_common_tau, leg_swing_ref, leg_len_ref
ROLL_COLS = [IDX["roll"], IDX["roll_rate"], IDX["leg_len_diff"],
             IDX["leg_len_diff_rate"], IDX["wheel_diff_v"]]
ROLL_INPUTS = [1, 4]         # wheel_diff_tau, leg_len_diff_ref

# LQR 权重: pitch 子系统主权重, roll 子系统独立
Q_PITCH = np.diag([900.0, 12.0, 100.0, 2.0, 60.0, 1.0, 2.0])
R_PITCH = np.diag([0.20, 50.0, 12.0])
Q_ROLL = np.diag([500.0, 10.0, 160.0, 2.0, 1.0])
R_ROLL = np.diag([2.0, 10.0])

LEG_LEN_BINS = np.arange(-0.30, 0.11, 0.04)


def solve_dare(A, B, Q, R, max_iter=5000, tol=1e-10):
    P = Q.copy()
    for _ in range(max_iter):
        Pn = Q + A.T @ P @ A - A.T @ P @ B @ np.linalg.solve(
            R + B.T @ P @ B, B.T @ P @ A)
        if np.linalg.norm(Pn - P, "fro") < tol:
            P = Pn
            break
        P = Pn
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


def fit_subsystem(name, sens, u, cols, in_idx, q, r, valid, dt):
    """拟合单个子系统: 返回 dict(A,B,c,K,r2,trim) 或 None"""
    X = sens[:, cols]
    Uk = u[:, in_idx]
    n, m = len(cols), len(in_idx)
    idx = np.where(valid[:-1] & valid[1:])[0]
    if len(idx) < 60:
        print(f"  {name}: 样本不足 ({len(idx)}), 跳过")
        return None
    Xk, Xn = X[idx], X[idx + 1]
    Ukk = Uk[idx]
    Phi = np.hstack([Xk, Ukk, np.ones((len(idx), 1))])
    theta, _, _, _ = np.linalg.lstsq(Phi, Xn, rcond=None)
    A = theta[:n, :].T
    B = theta[n:n + m, :].T
    c = theta[n + m, :]
    pred = Phi @ theta
    ss_res = np.sum((Xn - pred) ** 2, axis=0)
    ss_tot = np.sum((Xn - np.mean(Xn, axis=0)) ** 2, axis=0)
    r2 = 1.0 - ss_res / np.maximum(ss_tot, 1e-12)
    trim = np.mean(X[valid], axis=0)
    K = solve_dare(A, B, q, r)
    poles = np.linalg.eigvals(A)
    print(f"  {name}: n={n} m={m} trans={len(idx)} "
          f"R2={np.round(r2,3)} max|K|={np.abs(K).max():.2f}")
    return {
        "A": A, "B": B, "c": c, "K": K, "r2": r2,
        "x_trim": trim, "poles_real": np.real(poles).tolist(),
        "poles_imag": np.imag(poles).tolist(),
        "transitions": int(len(idx)),
    }


def main():
    d = np.load(SRC)
    sens, u = d["sens"], d["u"]
    z = d["z"]
    dt = d["dt_model"] if "dt_model" in d else 0.01
    valid = np.isfinite(z) & (np.abs(sens[:, IDX["pitch"]]) < 0.105) & \
            (np.abs(sens[:, IDX["roll"]]) < 0.14)
    leg_len = sens[:, IDX["leg_len"]]
    print(f"数据: {len(sens)} 样本, 有效 {int(valid.sum())}, dt={dt}")

    # 全局(不分段)子系统
    global_models = {
        "pitch": fit_subsystem("pitch(全局)", sens, u, PITCH_COLS, PITCH_INPUTS,
                               Q_PITCH, R_PITCH, valid, dt),
        "roll": fit_subsystem("roll(全局)", sens, u, ROLL_COLS, ROLL_INPUTS,
                              Q_ROLL, R_ROLL, valid, dt),
    }

    # 腿长分段子系统
    piecewise = {}
    for bi in range(len(LEG_LEN_BINS) - 1):
        lo, hi = LEG_LEN_BINS[bi], LEG_LEN_BINS[bi + 1]
        key = f"leglen_{lo:+.2f}_{hi:+.2f}"
        seg = valid & (leg_len >= lo) & (leg_len < hi)
        if int(seg.sum()) < 30:
            piecewise[key] = {"pitch": None, "roll": None,
                              "range": [lo, hi], "samples": int(seg.sum())}
            continue
        print(f"  [{lo:+.2f},{hi:+.2f}] samples={int(seg.sum())}")
        piecewise[key] = {
            "pitch": fit_subsystem(f"  pitch[{lo:+.2f},{hi:+.2f}]",
                                   sens, u, PITCH_COLS, PITCH_INPUTS,
                                   Q_PITCH, R_PITCH, seg, dt),
            "roll": fit_subsystem(f"  roll[{lo:+.2f},{hi:+.2f}]",
                                  sens, u, ROLL_COLS, ROLL_INPUTS,
                                  Q_ROLL, R_ROLL, seg, dt),
            "range": [lo, hi],
            "samples": int(seg.sum()),
        }

    def ser(m):
        if m is None:
            return None
        return {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                for k, v in m.items()}

    payload = {
        "generated_at": time.strftime("%Y%m%d_%H%M%S"),
        "source": "three_loop_model_latest.npz",
        "dt": float(dt),
        "pitch_state_names": ["pitch", "pitch_rate", "leg_swing",
                              "leg_swing_rate", "leg_len", "leg_len_rate",
                              "wheel_v"],
        "pitch_input_names": ["wheel_common_tau", "leg_swing_ref", "leg_len_ref"],
        "roll_state_names": ["roll", "roll_rate", "leg_len_diff",
                             "leg_len_diff_rate", "wheel_diff_v"],
        "roll_input_names": ["wheel_diff_tau", "leg_len_diff_ref"],
        "global": {k: ser(v) for k, v in global_models.items()},
        "piecewise": {k: {"pitch": ser(v["pitch"]), "roll": ser(v["roll"]),
                          "range": v["range"], "samples": v["samples"]}
                      for k, v in piecewise.items()},
    }
    with open(OUT_JSON, "w") as f:
        json.dump(payload, f, indent=2)

    # npz: 各段 pitch/roll 的 A/B/K/trim 扁平保存
    save = {
        "dt": np.array([dt]),
        "pitch_cols": np.array(PITCH_COLS),
        "roll_cols": np.array(ROLL_COLS),
    }
    for tag, m in [("g_pitch", global_models["pitch"]),
                   ("g_roll", global_models["roll"])]:
        if m:
            for k, v in m.items():
                save[f"{tag}_{k}"] = np.array(v)
    for key, item in piecewise.items():
        for sub in ("pitch", "roll"):
            m = item[sub]
            if m:
                for k, v in m.items():
                    save[f"{key}_{sub}_{k}"] = np.array(v)
    np.savez(OUT_NPZ, **save)
    print(f"\n拆分模型 -> {OUT_JSON}")
    print(f"           -> {OUT_NPZ}")


if __name__ == "__main__":
    main()

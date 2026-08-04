#!/usr/bin/env python3
"""辨识后局部 LQR/MPC 模型的离线闭环仿真。

模型是局部偏差模型:

    x_dev[k+1] = A x_dev[k] + B u_extra[k] + c
    u_extra[k] = -K x_dev[k]

这个脚本不调用 MuJoCo，只在把 LQR/MPC 接入真实控制器前检查数值稳定性、
分段切换和输入限幅是否合理。
"""

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = str(PROJECT_ROOT / "logs" / "trim_lqr_model_latest.npz")
DEFAULT_LIMITS = np.array([25.0, 0.018, 0.010, 0.010], dtype=float)
# 输入顺序必须和 identify_discrete.py 导出的 B 矩阵一致。
INPUT_NAMES = ["wheel_common_tau", "leg_swing_ref", "leg_len_ref", "leg_len_diff_ref"]
PIECEWISE_MODELS = {
    # inner 负责 0~2.5deg，pos/neg 负责 2.5~5deg 两侧分段。
    "inner": "compact_abs_0_2p5",
    "pos": "compact_pos_2p5_5",
    "neg": "compact_neg_2p5_5",
}
LEGLEN_BLEND_RAD_DEFAULT = 0.008


def decode_names(arr):
    return [str(x) for x in arr.tolist()]


def default_initial_state(names, preset):
    x0 = np.zeros(len(names), dtype=float)
    idx = {name: i for i, name in enumerate(names)}

    def set_if(name, value):
        if name in idx:
            x0[idx[name]] = value

    if preset == "pitch":
        set_if("pitch", math.radians(3.0))
        set_if("pitch_rate", 0.0)
    elif preset == "pitch_neg":
        set_if("pitch", math.radians(-3.0))
        set_if("pitch_rate", 0.0)
    elif preset == "roll":
        set_if("roll", math.radians(3.0))
        set_if("roll_rate", 0.0)
    elif preset == "mixed":
        set_if("pitch", math.radians(3.0))
        set_if("pitch_rate", 0.4)
        set_if("roll", math.radians(2.0))
        set_if("roll_rate", -0.3)
        set_if("leg_swing", 0.006)
        set_if("leg_len", 0.004)
        set_if("leg_len_diff", -0.004)
        set_if("wheel_v", 0.25)
        set_if("wheel_diff_v", -0.10)
    elif preset == "mixed_neg":
        set_if("pitch", math.radians(-3.0))
        set_if("pitch_rate", -0.4)
        set_if("roll", math.radians(-2.0))
        set_if("roll_rate", 0.3)
        set_if("leg_swing", -0.006)
        set_if("leg_len", 0.004)
        set_if("leg_len_diff", 0.004)
        set_if("wheel_v", -0.25)
        set_if("wheel_diff_v", 0.10)
    elif preset == "zero":
        pass
    elif preset == "leglen":
        set_if("pitch", math.radians(1.0))
        set_if("pitch_rate", 0.15)
        set_if("leg_len", -0.030)
        set_if("leg_len_rate", 0.0)
        set_if("wheel_v", 0.10)
    else:
        raise ValueError(f"unknown preset {preset!r}")
    return x0


def load_model(path, model_name):
    data = np.load(path, allow_pickle=True)
    prefix = f"{model_name}_"
    required = [prefix + k for k in ["A", "B", "c", "K", "r2", "state_names", "x_trim"]]
    missing = [k for k in required if k not in data.files]
    if missing:
        raise RuntimeError(f"{path} does not contain {model_name!r} model keys: {missing}")
    A = data[prefix + "A"]
    B = data[prefix + "B"]
    c = data[prefix + "c"]
    K = data[prefix + "K"]
    r2 = data[prefix + "r2"]
    names = decode_names(data[prefix + "state_names"])
    x_trim = data[prefix + "x_trim"]
    if K.size == 0:
        raise RuntimeError(f"{model_name!r} model has no K gain")
    return A, B, c, K, r2, names, x_trim


def load_model_dict(path, model_name):
    A, B, c, K, r2, names, x_trim = load_model(path, model_name)
    return {
        "name": model_name,
        "A": A,
        "B": B,
        "c": c,
        "K": K,
        "r2": r2,
        "names": names,
        "x_trim": x_trim,
    }


def load_leglen_models(path):
    """按 NPZ 里的 *_leg_len_range 元数据加载全部腿长分段 compact 模型。"""
    data = np.load(path, allow_pickle=True)
    keys = sorted(
        set(k[: -len("_leg_len_range")] for k in data.files if k.endswith("_leg_len_range"))
    )
    models = []
    for key in keys:
        model = load_model_dict(path, key)
        rng = data[f"{key}_leg_len_range"]
        center = data[f"{key}_expansion_leg_len"]
        model["leg_len_range"] = [float(rng[0]), float(rng[1])]
        model["center"] = float(np.asarray(center).reshape(-1)[0])
        models.append(model)
    models.sort(key=lambda m: m["leg_len_range"][0])
    if not models:
        raise RuntimeError(f"{path} contains no leg-length piecewise models")
    return models


def solve_bias(B, c, limits):
    u_bias, *_ = np.linalg.lstsq(B, -c, rcond=None)
    u_clip = np.clip(u_bias, -limits, limits)
    return u_bias, u_clip, B @ u_clip + c


def simulate(A, B, c, K, x0, steps, limits, use_c, u_bias=None):
    n = A.shape[0]
    m = B.shape[1]
    xs = np.zeros((steps + 1, n), dtype=float)
    us = np.zeros((steps, m), dtype=float)
    raw_us = np.zeros((steps, m), dtype=float)
    sat = np.zeros((steps, m), dtype=bool)
    xs[0] = x0
    cc = c if use_c else np.zeros_like(c)
    if u_bias is None:
        u_bias = np.zeros(B.shape[1], dtype=float)
    for k in range(steps):
        u_raw = u_bias - K @ xs[k]
        u = np.clip(u_raw, -limits, limits)
        raw_us[k] = u_raw
        us[k] = u
        sat[k] = np.abs(u_raw) > limits + 1e-12
        xs[k + 1] = A @ xs[k] + B @ u + cc
        if not np.all(np.isfinite(xs[k + 1])):
            xs[k + 1 :] = np.nan
            break
    return xs, us, raw_us, sat


def smoothstep(edge0, edge1, x):
    if edge1 <= edge0:
        return 1.0 if x >= edge1 else 0.0
    s = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return float(s * s * (3.0 - 2.0 * s))


def solve_reference_feedforward(model, x_ref_abs, limits):
    x_ref_dev = x_ref_abs - model["x_trim"]
    rhs = (np.eye(model["A"].shape[0]) - model["A"]) @ x_ref_dev - model["c"]
    u_ref, *_ = np.linalg.lstsq(model["B"], rhs, rcond=None)
    u_clip = np.clip(u_ref, -limits, limits)
    residual = model["B"] @ u_clip - rhs
    return u_ref, u_clip, residual


def compute_model_control(model, x_abs, x_ref_abs, limits, linearized=False):
    if linearized:
        u_raw = -model["K"] @ (x_abs - x_ref_abs)
        return u_raw, np.clip(u_raw, -limits, limits)
    _u_ref_raw, u_ref_clip, _residual = solve_reference_feedforward(model, x_ref_abs, limits)
    x_dev = x_abs - model["x_trim"]
    x_ref_dev = x_ref_abs - model["x_trim"]
    u_raw = u_ref_clip - model["K"] @ (x_dev - x_ref_dev)
    return u_raw, np.clip(u_raw, -limits, limits)


def predict_abs_next(model, x_abs, x_ref_abs, u, linearized=False):
    if linearized:
        return x_ref_abs + model["A"] @ (x_abs - x_ref_abs) + model["B"] @ u
    x_dev = x_abs - model["x_trim"]
    x_dev_next = model["A"] @ x_dev + model["B"] @ u + model["c"]
    return model["x_trim"] + x_dev_next


def target_state_from_inner_trim(names, inner_trim, mode, pitch_ref=0.0):
    target = inner_trim.copy()
    idx = {name: i for i, name in enumerate(names)}
    if mode == "local_trim":
        return target
    if mode not in ("zero_state", "handoff"):
        raise ValueError(f"unknown piecewise target mode {mode!r}")
    if "pitch" in idx:
        target[idx["pitch"]] = pitch_ref
    for name in ["pitch", "pitch_rate", "roll", "roll_rate", "leg_swing_rate", "leg_len_rate", "leg_len_diff_rate", "wheel_v", "wheel_diff_v"]:
        if name in idx:
            target[idx[name]] = 0.0
    if "pitch" in idx:
        target[idx["pitch"]] = pitch_ref
    return target


def simulate_piecewise(models, x0_dev, refs_abs, steps, limits, dt, alpha_rate, du_limits, blend_lo_deg, blend_hi_deg, linearized):
    names = models["inner"]["names"]
    idx = {name: i for i, name in enumerate(names)}
    if "pitch" not in idx:
        raise RuntimeError("piecewise compact model has no pitch state")
    for key, model in models.items():
        if model["names"] != names:
            raise RuntimeError(f"piecewise model {key} state ordering differs from inner model")

    n = len(names)
    m = models["inner"]["B"].shape[1]
    xs = np.zeros((steps + 1, n), dtype=float)
    us = np.zeros((steps, m), dtype=float)
    raw_us = np.zeros((steps, m), dtype=float)
    sat = np.zeros((steps, m), dtype=bool)
    alphas = np.zeros(steps + 1, dtype=float)
    alpha_targets = np.zeros(steps, dtype=float)
    du = np.zeros((steps, m), dtype=float)

    xs[0] = refs_abs["inner"] + x0_dev
    pitch0 = xs[0, idx["pitch"]]
    alphas[0] = smoothstep(math.radians(blend_lo_deg), math.radians(blend_hi_deg), abs(pitch0))
    prev_u = None
    max_dalpha = max(0.0, alpha_rate) * dt
    max_du = np.asarray(du_limits, dtype=float) if du_limits is not None else None

    for k in range(steps):
        pitch = xs[k, idx["pitch"]]
        outer_key = "pos" if pitch >= 0.0 else "neg"
        inner = models["inner"]
        outer = models[outer_key]
        alpha_target = smoothstep(math.radians(blend_lo_deg), math.radians(blend_hi_deg), abs(pitch))
        alpha_prev = alphas[k]
        if max_dalpha > 0.0:
            alpha = alpha_prev + float(np.clip(alpha_target - alpha_prev, -max_dalpha, max_dalpha))
        else:
            alpha = alpha_target
        alphas[k] = alpha
        alpha_targets[k] = alpha_target

        u_inner_raw, _u_inner = compute_model_control(inner, xs[k], refs_abs["inner"], limits, linearized)
        u_outer_raw, _u_outer = compute_model_control(outer, xs[k], refs_abs[outer_key], limits, linearized)
        u_raw = (1.0 - alpha) * u_inner_raw + alpha * u_outer_raw
        u = np.clip(u_raw, -limits, limits)
        if max_du is not None and prev_u is not None:
            u = prev_u + np.clip(u - prev_u, -max_du, max_du)
        raw_us[k] = u_raw
        us[k] = u
        sat[k] = np.abs(u_raw) > limits + 1e-12
        if prev_u is not None:
            du[k] = u - prev_u
        prev_u = u.copy()

        x_next_inner = predict_abs_next(inner, xs[k], refs_abs["inner"], u, linearized)
        x_next_outer = predict_abs_next(outer, xs[k], refs_abs[outer_key], u, linearized)
        xs[k + 1] = (1.0 - alpha) * x_next_inner + alpha * x_next_outer
        if not np.all(np.isfinite(xs[k + 1])):
            xs[k + 1 :] = np.nan
            alphas[k + 1 :] = np.nan
            break
        alphas[k + 1] = alpha
    return xs, us, raw_us, sat, alphas, alpha_targets, du


def leglen_membership(leg_len, lo, hi, blend):
    """软边界隶属度：段内为 1，跨边界 blend 宽度内平滑衰减，之后按段间归一化。"""
    if lo <= leg_len <= hi:
        return 1.0
    if leg_len < lo:
        return smoothstep(lo - blend, lo, leg_len)
    return 1.0 - smoothstep(hi, hi + blend, leg_len)


def simulate_leglen(models, x0_dev, refs_abs, anchor_key, steps, limits, dt, alpha_rate, du_limits, blend, linearized):
    """按当前 leg_len 在腿长分段模型间平滑切换的离线闭环仿真。

    参考点取各段自己的 x_trim（local trim），即每段控制器维持本段工作点；
    切换权重按相邻段边界 blend 宽度平滑，并加权重变化速率限制避免抖动。
    """
    names = models[anchor_key]["names"]
    idx = {name: i for i, name in enumerate(names)}
    if "leg_len" not in idx:
        raise RuntimeError("leg-length compact model has no leg_len state")
    for key, model in models.items():
        if model["names"] != names:
            raise RuntimeError(f"leg-length model {key} state ordering differs from anchor model")

    n = len(names)
    m = models[anchor_key]["B"].shape[1]
    xs = np.zeros((steps + 1, n), dtype=float)
    us = np.zeros((steps, m), dtype=float)
    raw_us = np.zeros((steps, m), dtype=float)
    sat = np.zeros((steps, m), dtype=bool)
    weights = np.zeros((steps + 1, len(models)), dtype=float)
    weight_targets = np.zeros((steps, len(models)), dtype=float)
    du = np.zeros((steps, m), dtype=float)

    xs[0] = refs_abs[anchor_key] + x0_dev
    max_dw = max(0.0, alpha_rate) * dt
    weights[0] = leglen_weight_vector(xs[0, idx["leg_len"]], models, blend)
    s = float(weights[0].sum())
    weights[0] = weights[0] / s if s > 1e-12 else 1.0 / len(models)
    prev_w = weights[0].copy()
    prev_u = None
    max_du = np.asarray(du_limits, dtype=float) if du_limits is not None else None

    for k in range(steps):
        w_target = leglen_weight_vector(xs[k, idx["leg_len"]], models, blend)
        weight_targets[k] = w_target
        if max_dw > 0.0:
            w = prev_w + np.clip(w_target - prev_w, -max_dw, max_dw)
            s = float(w.sum())
            w = w / s if s > 1e-12 else w_target
        else:
            w = w_target
        weights[k] = w

        u_raw = np.zeros(m)
        x_next = np.zeros(n)
        for i, (key, model) in enumerate(models.items()):
            u_i_raw, _ = compute_model_control(model, xs[k], refs_abs[key], limits, linearized)
            u_raw += w[i] * u_i_raw
            x_next += w[i] * predict_abs_next(model, xs[k], refs_abs[key], u_raw, linearized)

        u = np.clip(u_raw, -limits, limits)
        if max_du is not None and prev_u is not None:
            u = prev_u + np.clip(u - prev_u, -max_du, max_du)
        raw_us[k] = u_raw
        us[k] = u
        sat[k] = np.abs(u_raw) > limits + 1e-12
        if prev_u is not None:
            du[k] = u - prev_u
        prev_u = u.copy()
        xs[k + 1] = x_next
        if not np.all(np.isfinite(xs[k + 1])):
            xs[k + 1 :] = np.nan
            weights[k + 1 :] = np.nan
            break
        weights[k + 1] = w
        prev_w = w.copy()
    return xs, us, raw_us, sat, weights, weight_targets, du


def leglen_weight_vector(leg_len, models, blend):
    """按当前 leg_len 计算各分段模型权重（软边界、归一化）。"""
    w = np.asarray(
        [
            leglen_membership(leg_len, m["leg_len_range"][0], m["leg_len_range"][1], blend)
            for m in models.values()
        ]
    )
    s = float(w.sum())
    return w / s if s > 1e-12 else np.ones(len(models)) / len(models)


def simulate_leglen_sweep(models, steps, limits, dt, alpha_rate, du_limits, blend, linearized,
                          leg_lo=-0.130, leg_hi=-0.060, ramp_s=5.0):
    """腿长参考慢速扫掠测试：leg_len 从 leg_lo 线性扫到 leg_hi，
    验证分段模型按实测 leg_len 在边界处平滑切换且闭环稳定。
    参考点取零状态投影(pitch=0/wheel=0)，leg_len 跟随扫掠轨迹。
    """
    keys = list(models.keys())
    anchor_key = keys[len(keys) // 2]
    names = models[anchor_key]["names"]
    idx = {name: i for i, name in enumerate(names)}
    if "leg_len" not in idx:
        raise RuntimeError("leg-length compact model has no leg_len state")
    n = len(names)
    m = models[anchor_key]["B"].shape[1]
    xs = np.zeros((steps + 1, n), dtype=float)
    us = np.zeros((steps, m), dtype=float)
    raw_us = np.zeros((steps, m), dtype=float)
    sat = np.zeros((steps, m), dtype=bool)
    weights = np.zeros((steps + 1, len(models)), dtype=float)
    weight_targets = np.zeros((steps, len(models)), dtype=float)
    du = np.zeros((steps, m), dtype=float)
    leg_refs = np.zeros(steps + 1, dtype=float)

    def refs_at(leg_ref):
        refs = {}
        for key in keys:
            ref = target_state_from_inner_trim(names, models[key]["x_trim"], "zero_state", 0.0)
            ref[idx["leg_len"]] = leg_ref
            refs[key] = ref
        return refs

    leg_refs[0] = leg_lo
    refs0 = refs_at(leg_lo)
    xs[0] = refs0[anchor_key]
    weights[0] = leglen_weight_vector(xs[0, idx["leg_len"]], models, blend)
    prev_w = weights[0].copy()
    prev_u = None
    max_dw = max(0.0, alpha_rate) * dt
    max_du = np.asarray(du_limits, dtype=float) if du_limits is not None else None

    for k in range(steps):
        t = k * dt
        frac = min(1.0, max(0.0, t / max(ramp_s, 1e-9)))
        leg_ref = leg_lo + frac * (leg_hi - leg_lo)
        leg_refs[k] = leg_ref
        refs_abs = refs_at(leg_ref)

        w_target = leglen_weight_vector(xs[k, idx["leg_len"]], models, blend)
        weight_targets[k] = w_target
        if max_dw > 0.0:
            w = prev_w + np.clip(w_target - prev_w, -max_dw, max_dw)
            s = float(w.sum())
            w = w / s if s > 1e-12 else w_target
        else:
            w = w_target
        weights[k] = w

        u_raw = np.zeros(m)
        x_next = np.zeros(n)
        for i, key in enumerate(keys):
            u_i_raw, _ = compute_model_control(models[key], xs[k], refs_abs[key], limits, linearized)
            u_raw += w[i] * u_i_raw
            x_next += w[i] * predict_abs_next(models[key], xs[k], refs_abs[key], u_raw, linearized)

        u = np.clip(u_raw, -limits, limits)
        if max_du is not None and prev_u is not None:
            u = prev_u + np.clip(u - prev_u, -max_du, max_du)
        raw_us[k] = u_raw
        us[k] = u
        sat[k] = np.abs(u_raw) > limits + 1e-12
        if prev_u is not None:
            du[k] = u - prev_u
        prev_u = u.copy()
        xs[k + 1] = x_next
        if not np.all(np.isfinite(xs[k + 1])):
            xs[k + 1 :] = np.nan
            weights[k + 1 :] = np.nan
            break
        weights[k + 1] = w
        prev_w = w.copy()
        t_next = (k + 1) * dt
        leg_refs[k + 1] = leg_lo + min(1.0, t_next / max(ramp_s, 1e-9)) * (leg_hi - leg_lo)
    return xs, us, raw_us, sat, weights, weight_targets, du, leg_refs


def state_limits_for_report(names):
    limits = {}
    for name in names:
        if name in ("pitch", "roll"):
            limits[name] = math.radians(5.0 if name == "pitch" else 10.0)
        elif name.endswith("_rate"):
            limits[name] = 8.0
        elif name in ("leg_swing", "leg_len", "leg_len_diff"):
            limits[name] = 0.08
        elif name in ("wheel_v", "wheel_diff_v"):
            limits[name] = 5.0
    return limits


def summarize(label, xs, us, sat, names, dt):
    finite = np.all(np.isfinite(xs), axis=1)
    last = int(np.where(finite)[0][-1]) if np.any(finite) else 0
    x_final = xs[last]
    x0_norm = float(np.linalg.norm(xs[0]))
    xf_norm = float(np.linalg.norm(x_final))
    max_abs = np.nanmax(np.abs(xs[: last + 1]), axis=0)
    sat_frac = np.mean(sat[:max(1, last)], axis=0) if len(sat) else np.zeros(us.shape[1])

    print(f"\n[{label}] simulated {last * dt:.2f}s ({last} steps)")
    print(f"  ||x0||={x0_norm:.5g}  ||xf||={xf_norm:.5g}  ratio={xf_norm / max(x0_norm, 1e-12):.4g}")
    print("  input saturation fraction:")
    for name, frac in zip(INPUT_NAMES[: us.shape[1]], sat_frac):
        print(f"    {name:18s}: {100.0 * frac:6.2f}%")
    print("  max |state deviation|:")
    report_limits = state_limits_for_report(names)
    for name, val in zip(names, max_abs):
        suffix = ""
        if name in ("pitch", "roll"):
            suffix = f" ({math.degrees(val):.2f} deg)"
        flag = ""
        if name in report_limits and val > report_limits[name]:
            flag = "  exceeds-report-limit"
        print(f"    {name:18s}: {val:+.5f}{suffix}{flag}")

    # A simple machine-readable-ish verdict for quick scans.
    grew = xf_norm > 1.2 * max(x0_norm, 1e-9)
    heavy_sat = np.any(sat_frac > 0.20)
    print(f"  verdict: {'GROWS' if grew else 'decays/holds'}; {'SATURATING' if heavy_sat else 'low saturation'}")


def summarize_piecewise(xs_abs, us, raw_us, sat, alphas, du, names, dt):
    finite = np.all(np.isfinite(xs_abs), axis=1)
    last = int(np.where(finite)[0][-1]) if np.any(finite) else 0
    idx = {name: i for i, name in enumerate(names)}
    pitch = xs_abs[: last + 1, idx["pitch"]] if "pitch" in idx else np.zeros(last + 1)
    roll = xs_abs[: last + 1, idx["roll"]] if "roll" in idx else np.zeros(last + 1)
    sat_frac = np.mean(sat[:max(1, last)], axis=0) if len(sat) else np.zeros(us.shape[1])
    max_du = np.nanmax(np.abs(du[:max(1, last)]), axis=0) if len(du) else np.zeros(us.shape[1])
    max_raw = np.nanmax(np.abs(raw_us[:max(1, last)]), axis=0) if len(raw_us) else np.zeros(us.shape[1])

    print(f"\n[piecewise smooth] simulated {last * dt:.2f}s ({last} steps)")
    print(f"  final pitch={math.degrees(pitch[-1]):+.4f} deg  roll={math.degrees(roll[-1]):+.4f} deg")
    print(
        f"  max |pitch|={math.degrees(np.nanmax(np.abs(pitch))):.4f} deg  "
        f"max |roll|={math.degrees(np.nanmax(np.abs(roll))):.4f} deg"
    )
    print(f"  alpha: initial={alphas[0]:.3f} max={np.nanmax(alphas[:last+1]):.3f} final={alphas[last]:.3f}")
    print("  input saturation fraction:")
    for name, frac in zip(INPUT_NAMES[: us.shape[1]], sat_frac):
        print(f"    {name:18s}: {100.0 * frac:6.2f}%")
    print("  max input step after slew:")
    for name, val in zip(INPUT_NAMES[: us.shape[1]], max_du):
        print(f"    {name:18s}: {val:+.5f}")
    print("  max raw input command:")
    for name, val in zip(INPUT_NAMES[: us.shape[1]], max_raw):
        print(f"    {name:18s}: {val:+.5f}")


def summarize_leglen(xs_abs, us, raw_us, sat, weights, du, names, dt, model_keys):
    finite = np.all(np.isfinite(xs_abs), axis=1)
    last = int(np.where(finite)[0][-1]) if np.any(finite) else 0
    idx = {name: i for i, name in enumerate(names)}
    leg_len = xs_abs[: last + 1, idx["leg_len"]] if "leg_len" in idx else np.zeros(last + 1)
    pitch = xs_abs[: last + 1, idx["pitch"]] if "pitch" in idx else np.zeros(last + 1)
    roll = xs_abs[: last + 1, idx["roll"]] if "roll" in idx else np.zeros(last + 1)
    sat_frac = np.mean(sat[:max(1, last)], axis=0) if len(sat) else np.zeros(us.shape[1])
    max_du = np.nanmax(np.abs(du[:max(1, last)]), axis=0) if len(du) else np.zeros(us.shape[1])
    max_raw = np.nanmax(np.abs(raw_us[:max(1, last)]), axis=0) if len(raw_us) else np.zeros(us.shape[1])
    w_used = weights[: last + 1]
    dom = np.argmax(w_used, axis=1) if w_used.ndim == 2 else np.zeros(last + 1, dtype=int)

    print(f"\n[leglen smooth] simulated {last * dt:.2f}s ({last} steps)")
    print(f"  final leg_len={leg_len[-1]:+.4f} rad  pitch={math.degrees(pitch[-1]):+.4f} deg  roll={math.degrees(roll[-1]):+.4f} deg")
    print(f"  leg_len range: [{np.nanmin(leg_len):+.4f}, {np.nanmax(leg_len):+.4f}]")
    print(f"  max |leg_len dev|={np.nanmax(np.abs(leg_len - np.mean(leg_len))):.4f} rad")
    print("  dominant segment occupancy:")
    for i, key in enumerate(model_keys):
        print(f"    {key:20s}: {100.0 * float(np.mean(dom == i)):6.1f}%  max_weight={np.nanmax(w_used[:, i]):.3f}")
    print("  input saturation fraction:")
    for name, frac in zip(INPUT_NAMES[: us.shape[1]], sat_frac):
        print(f"    {name:18s}: {100.0 * frac:6.2f}%")
    print("  max input step after slew:")
    for name, val in zip(INPUT_NAMES[: us.shape[1]], max_du):
        print(f"    {name:18s}: {val:+.5f}")
    print("  max raw input command:")
    for name, val in zip(INPUT_NAMES[: us.shape[1]], max_raw):
        print(f"    {name:18s}: {val:+.5f}")


def summarize_leglen_sweep(xs_abs, us, raw_us, sat, weights, du, names, dt, model_keys, leg_refs, leg_len_ranges):
    finite = np.all(np.isfinite(xs_abs), axis=1)
    last = int(np.where(finite)[0][-1]) if np.any(finite) else 0
    idx = {name: i for i, name in enumerate(names)}
    leg_len = xs_abs[: last + 1, idx["leg_len"]]
    refs = leg_refs[: last + 1]
    t = np.arange(last + 1) * dt
    sat_frac = np.mean(sat[:max(1, last)], axis=0) if len(sat) else np.zeros(us.shape[1])
    w = weights[: last + 1]
    dom = np.argmax(w, axis=1)

    print(f"\n[leglen sweep] simulated {last * dt:.2f}s ({last} steps)")
    print(f"  leg_len ref: {refs[0]:+.4f} -> {refs[-1]:+.4f} rad")
    print(f"  final leg_len={leg_len[-1]:+.4f} rad  tracking err={leg_len[-1] - refs[-1]:+.4f} rad")
    print(f"  max |tracking err|={np.nanmax(np.abs(leg_len - refs)):.4f} rad")
    print("  dominant segment occupancy:")
    for i, key in enumerate(model_keys):
        print(f"    {key:20s}: {100.0 * float(np.mean(dom == i)):6.1f}%  max_weight={np.nanmax(w[:, i]):.3f}")
    print("  boundary crossings (time when next segment weight first exceeds 0.5):")
    for i in range(len(model_keys) - 1):
        cross = np.where(w[1:, i + 1] > 0.5)[0]
        if len(cross) == 0:
            print(f"    {model_keys[i]} -> {model_keys[i+1]}: not crossed within window")
            continue
        c = cross[0] + 1
        expect_lo = leg_len_ranges[i + 1][0]
        print(
            f"    {model_keys[i]} -> {model_keys[i+1]}: t={t[c]:.2f}s leg_len={leg_len[c]:+.4f} "
            f"(segment boundary starts at {expect_lo:+.4f})"
        )
    print("  input saturation fraction:")
    for name, frac in zip(INPUT_NAMES[: us.shape[1]], sat_frac):
        print(f"    {name:18s}: {100.0 * frac:6.2f}%")
    print("  max input step after slew:")
    for name, val in zip(INPUT_NAMES[: us.shape[1]], np.nanmax(np.abs(du[:max(1, last)]), axis=0)):
        print(f"    {name:18s}: {val:+.5f}")


def plot_result(out, t, names, results):
    n = len(names)
    fig_h = max(8.0, 0.35 * n + 5.0)
    fig, axes = plt.subplots(3, 1, figsize=(12, fig_h), dpi=140, sharex=False)

    ax = axes[0]
    for label, xs, _us in results:
        for name in ["pitch", "roll"]:
            if name in names:
                i = names.index(name)
                ax.plot(t, np.degrees(xs[:, i]), label=f"{name} {label}")
    ax.axhline(5.0, color="tab:red", lw=0.8, alpha=0.4)
    ax.axhline(-5.0, color="tab:red", lw=0.8, alpha=0.4)
    ax.set_ylabel("angle dev (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    ax = axes[1]
    for i, name in enumerate(names):
        if name in ("pitch", "roll"):
            continue
        y = results[0][1][:, i]
        if np.nanmax(np.abs(y)) < 1e-9:
            continue
        ax.plot(t, y, label=name)
    ax.set_ylabel("state dev")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=7, ncol=3)

    ax = axes[2]
    tu = t[:-1]
    for label, _xs, us in results:
        for i, name in enumerate(INPUT_NAMES[: us.shape[1]]):
            ax.plot(tu, us[:, i], label=f"{name} {label}")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("u_extra")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=7, ncol=2)

    fig.suptitle("Identified Trim LQR Closed-Loop Simulation")
    fig.tight_layout()
    fig.savefig(out)
    print(f"\nwrote plot: {out}")


def plot_piecewise_result(out, t, names, xs_abs, us, raw_us, alphas, alpha_targets, du):
    idx = {name: i for i, name in enumerate(names)}
    fig, axes = plt.subplots(5, 1, figsize=(12, 13), dpi=140, sharex=True)

    ax = axes[0]
    for name in ["pitch", "roll"]:
        if name in idx:
            ax.plot(t, np.degrees(xs_abs[:, idx[name]]), label=name)
    ax.axhline(5.0, color="tab:red", lw=0.8, alpha=0.4)
    ax.axhline(-5.0, color="tab:red", lw=0.8, alpha=0.4)
    ax.axhline(2.5, color="tab:orange", lw=0.8, alpha=0.4)
    ax.axhline(-2.5, color="tab:orange", lw=0.8, alpha=0.4)
    ax.set_ylabel("angle (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    ax = axes[1]
    for name in ["pitch_rate", "roll_rate", "wheel_v", "wheel_diff_v"]:
        if name in idx:
            ax.plot(t, xs_abs[:, idx[name]], label=name)
    ax.set_ylabel("rate / wheel")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8, ncol=2)

    ax = axes[2]
    ax.plot(t, alphas, label="alpha used")
    ax.plot(t[:-1], alpha_targets, "--", label="alpha target", alpha=0.75)
    ax.set_ylabel("blend")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    ax = axes[3]
    for i, name in enumerate(INPUT_NAMES[: us.shape[1]]):
        ax.plot(t[:-1], us[:, i], label=name)
    ax.set_ylabel("u")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=7, ncol=2)

    ax = axes[4]
    for i, name in enumerate(INPUT_NAMES[: du.shape[1]]):
        ax.plot(t[:-1], du[:, i], label=f"d{name}")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("du / step")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=7, ncol=2)

    fig.suptitle("Piecewise Compact Trim LQR With Smooth Switching")
    fig.tight_layout()
    fig.savefig(out)
    print(f"\nwrote plot: {out}")


def plot_leglen_result(out, t, names, xs_abs, us, raw_us, weights, weight_targets, du, model_keys, leg_refs=None):
    idx = {name: i for i, name in enumerate(names)}
    fig, axes = plt.subplots(5, 1, figsize=(12, 13), dpi=140, sharex=True)

    ax = axes[0]
    for name in ["pitch", "roll"]:
        if name in idx:
            ax.plot(t, np.degrees(xs_abs[:, idx[name]]), label=name)
    ax.axhline(5.0, color="tab:red", lw=0.8, alpha=0.4)
    ax.axhline(-5.0, color="tab:red", lw=0.8, alpha=0.4)
    ax.set_ylabel("angle (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    ax = axes[1]
    if "leg_len" in idx:
        ax.plot(t, xs_abs[:, idx["leg_len"]], label="leg_len")
        if leg_refs is not None:
            ax.plot(t, leg_refs, "--", color="tab:red", label="leg_len ref", alpha=0.8)
    for name in ["pitch_rate", "roll_rate", "wheel_v", "wheel_diff_v"]:
        if name in idx:
            ax.plot(t, xs_abs[:, idx[name]], label=name)
    ax.set_ylabel("leg_len / rate / wheel")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8, ncol=2)

    ax = axes[2]
    for i, key in enumerate(model_keys):
        ax.plot(t, weights[:, i], label=key)
    ax.plot(t[:-1], weight_targets[:, 0], "--", color="gray", alpha=0.5, label="target w0")
    ax.set_ylabel("segment weight")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=7, ncol=2)

    ax = axes[3]
    for i, name in enumerate(INPUT_NAMES[: us.shape[1]]):
        ax.plot(t[:-1], us[:, i], label=name)
    ax.set_ylabel("u")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=7, ncol=2)

    ax = axes[4]
    for i, name in enumerate(INPUT_NAMES[: du.shape[1]]):
        ax.plot(t[:-1], du[:, i], label=f"d{name}")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("du / step")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=7, ncol=2)

    fig.suptitle("Leg-Length Piecewise Compact Trim LQR With Smooth Switching")
    fig.tight_layout()
    fig.savefig(out)
    print(f"\nwrote plot: {out}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-file", default=DEFAULT_MODEL)
    parser.add_argument("--model", default="compact",
                        help="Model prefix in the NPZ, e.g. compact, compact_abs_0_2p5, compact_abs_2p5_5")
    parser.add_argument("--piecewise", action="store_true",
                        help="Use compact_abs_0_2p5 plus +/- outer models with smooth pitch-band blending")
    parser.add_argument("--leglen", action="store_true",
                        help="Use compact_leglen_* models with smooth blending by measured leg_len")
    parser.add_argument("--leglen-blend-rad", type=float, default=LEGLEN_BLEND_RAD_DEFAULT,
                        help="Blend half-width across leg-length segment boundaries in rad")
    parser.add_argument("--leglen-sweep", action="store_true",
                        help="Sweep leg_len reference across all leg-length segments and watch model switching")
    parser.add_argument("--sweep-ramp", type=float, default=5.0,
                        help="Ramp duration for leg_len sweep in seconds")
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--preset", choices=["pitch", "pitch_neg", "roll", "mixed", "mixed_neg", "zero", "leglen"], default="mixed")
    parser.add_argument("--limits", type=float, nargs=4, default=DEFAULT_LIMITS.tolist(),
                        metavar=("WHEEL_NM", "SWING_RAD", "LEN_RAD", "DIFF_RAD"))
    parser.add_argument("--du-limits", type=float, nargs=4, default=[6.0, 0.006, 0.004, 0.004],
                        metavar=("D_WHEEL", "D_SWING", "D_LEN", "D_DIFF"),
                        help="Per-model-step input slew limits for piecewise mode")
    parser.add_argument("--alpha-rate", type=float, default=8.0,
                        help="Piecewise blend alpha slew rate in 1/s")
    parser.add_argument("--blend-lo-deg", type=float, default=2.2)
    parser.add_argument("--blend-hi-deg", type=float, default=2.8)
    parser.add_argument("--piecewise-target", choices=["zero_state", "handoff", "local_trim"], default="handoff",
                        help="Reference for all scheduled piecewise controllers")
    parser.add_argument("--piecewise-affine", action="store_true",
                        help="Use identified c/x_trim affine offsets in piecewise mode; default is small-signal linearized simulation")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    path = Path(args.model_file)
    dt = 0.01
    # Prefer stored dt if the JSON/NPZ producer includes it later; current NPZ
    # stores only matrices, so use the identification decimation dt.
    steps = max(1, int(round(args.duration / dt)))
    limits = np.asarray(args.limits, dtype=float)

    if args.piecewise:
        models = {key: load_model_dict(path, model_name) for key, model_name in PIECEWISE_MODELS.items()}
        names = models["inner"]["names"]
        if models["inner"]["B"].shape[1] != len(limits):
            raise RuntimeError(f"model input count {models['inner']['B'].shape[1]} != limits count {len(limits)}")
        x0_dev = default_initial_state(names, args.preset)
        if args.piecewise_target == "handoff":
            refs_abs = {
                "inner": target_state_from_inner_trim(names, models["inner"]["x_trim"], "zero_state", 0.0),
                "pos": target_state_from_inner_trim(names, models["inner"]["x_trim"], "zero_state", math.radians(args.blend_lo_deg)),
                "neg": target_state_from_inner_trim(names, models["inner"]["x_trim"], "zero_state", -math.radians(args.blend_lo_deg)),
            }
        else:
            ref = target_state_from_inner_trim(names, models["inner"]["x_trim"], args.piecewise_target, 0.0)
            refs_abs = {"inner": ref, "pos": ref, "neg": ref}
        xs_abs, us, raw_us, sat, alphas, alpha_targets, du = simulate_piecewise(
            models,
            x0_dev,
            refs_abs,
            steps,
            limits,
            dt,
            args.alpha_rate,
            args.du_limits,
            args.blend_lo_deg,
            args.blend_hi_deg,
            not args.piecewise_affine,
        )
        print(f"loaded: {path}")
        print("piecewise models:")
        for key, model in models.items():
            ref_key = key if key in refs_abs else "inner"
            _u_bias_raw, u_bias_clip, residual = solve_reference_feedforward(model, refs_abs[ref_key], limits)
            print(
                f"  {key:5s}: {model['name']:20s} R2_mean={np.mean(model['r2']):.4f} "
                f"c_norm={np.linalg.norm(model['c']):.5g} "
                f"u_ref={np.array2string(u_bias_clip, precision=5, suppress_small=True)} "
                f"resid={np.linalg.norm(residual):.5g}"
            )
        print(f"states: {names}")
        print(f"x_ref_inner: {np.array2string(refs_abs['inner'], precision=5, suppress_small=True)}")
        if args.piecewise_target == "handoff":
            print(f"x_ref_pos:   {np.array2string(refs_abs['pos'], precision=5, suppress_small=True)}")
            print(f"x_ref_neg:   {np.array2string(refs_abs['neg'], precision=5, suppress_small=True)}")
        print(f"input limits: {np.array2string(limits, precision=5)}")
        print(
            f"blend band: {args.blend_lo_deg:.2f}..{args.blend_hi_deg:.2f} deg, "
            f"alpha_rate={args.alpha_rate:.2f}/s, du_limits={np.array2string(np.asarray(args.du_limits), precision=5)}, "
            f"target={args.piecewise_target}, dynamics={'affine' if args.piecewise_affine else 'linearized'}"
        )
        summarize_piecewise(xs_abs, us, raw_us, sat, alphas, du, names, dt)

        out = args.out
        if out is None:
            out = str(PROJECT_ROOT / "logs" / f"trim_lqr_sim_piecewise_{args.preset}.png")
        t = np.arange(steps + 1) * dt
        plot_piecewise_result(out, t, names, xs_abs, us, raw_us, alphas, alpha_targets, du)
        return

    if args.leglen:
        models_list = load_leglen_models(path)
        model_keys = [m["name"] for m in models_list]
        models = {m["name"]: m for m in models_list}
        anchor_name = model_keys[len(model_keys) // 2]
        names = models[anchor_name]["names"]
        if models[anchor_name]["B"].shape[1] != len(limits):
            raise RuntimeError(
                f"model input count {models[anchor_name]['B'].shape[1]} != limits count {len(limits)}"
            )
        x0_dev = default_initial_state(names, args.preset)
        refs_abs = {m["name"]: m["x_trim"].copy() for m in models_list}
        xs_abs, us, raw_us, sat, weights, weight_targets, du = simulate_leglen(
            models,
            x0_dev,
            refs_abs,
            anchor_name,
            steps,
            limits,
            dt,
            args.alpha_rate,
            args.du_limits,
            args.leglen_blend_rad,
            not args.piecewise_affine,
        )
        print(f"loaded: {path}")
        print("leg-length piecewise models:")
        for key in model_keys:
            m = models[key]
            print(
                f"  {key:18s}: leg_len range [{m['leg_len_range'][0]:+.4f},{m['leg_len_range'][1]:+.4f}] "
                f"center={m['center']:+.4f} R2_mean={np.mean(m['r2']):.4f}"
            )
        print(f"states: {names}")
        print(
            f"anchor: {anchor_name} x_ref={np.array2string(refs_abs[anchor_name], precision=5, suppress_small=True)}"
        )
        print(
            f"input limits: {np.array2string(limits, precision=5)} "
            f"leglen_blend={args.leglen_blend_rad:.4f} rad alpha_rate={args.alpha_rate:.2f}/s, "
            f"dynamics={'affine' if args.piecewise_affine else 'linearized'}"
        )
        summarize_leglen(xs_abs, us, raw_us, sat, weights, du, names, dt, model_keys)

        out = args.out
        if out is None:
            out = str(PROJECT_ROOT / "logs" / f"trim_lqr_sim_leglen_{args.preset}.png")
        t = np.arange(steps + 1) * dt
        plot_leglen_result(out, t, names, xs_abs, us, raw_us, weights, weight_targets, du, model_keys)
        return

    if args.leglen_sweep:
        models_list = load_leglen_models(path)
        model_keys = [m["name"] for m in models_list]
        models = {m["name"]: m for m in models_list}
        names = models[model_keys[len(model_keys) // 2]]["names"]
        if models[model_keys[0]]["B"].shape[1] != len(limits):
            raise RuntimeError(
                f"model input count {models[model_keys[0]]['B'].shape[1]} != limits count {len(limits)}"
            )
        leg_lo = models_list[0]["center"]
        leg_hi = models_list[-1]["center"]
        ramp_s = min(args.sweep_ramp, args.duration * 0.8)
        xs_abs, us, raw_us, sat, weights, weight_targets, du, leg_refs = simulate_leglen_sweep(
            models,
            steps,
            limits,
            dt,
            args.alpha_rate,
            args.du_limits,
            args.leglen_blend_rad,
            not args.piecewise_affine,
            leg_lo=leg_lo,
            leg_hi=leg_hi,
            ramp_s=ramp_s,
        )
        leg_len_ranges = [m["leg_len_range"] for m in models_list]
        print(f"loaded: {path}")
        print("leg-length piecewise models (by range order):")
        for m in models_list:
            print(
                f"  {m['name']:18s}: leg_len range [{m['leg_len_range'][0]:+.4f},{m['leg_len_range'][1]:+.4f}] "
                f"center={m['center']:+.4f} R2_mean={np.mean(m['r2']):.4f}"
            )
        print(f"states: {names}")
        print(
            f"input limits: {np.array2string(limits, precision=5)} "
            f"leglen_blend={args.leglen_blend_rad:.4f} rad alpha_rate={args.alpha_rate:.2f}/s "
            f"ramp={ramp_s:.2f}s, dynamics={'affine' if args.piecewise_affine else 'linearized'}"
        )
        summarize_leglen_sweep(
            xs_abs, us, raw_us, sat, weights, du, names, dt, model_keys, leg_refs, leg_len_ranges
        )

        out = args.out
        if out is None:
            out = str(PROJECT_ROOT / "logs" / "trim_lqr_sim_leglen_sweep.png")
        t = np.arange(steps + 1) * dt
        plot_leglen_result(out, t, names, xs_abs, us, raw_us, weights, weight_targets, du, model_keys, leg_refs)
        return

    A, B, c, K, r2, names, x_trim = load_model(path, args.model)
    if B.shape[1] != len(limits):
        raise RuntimeError(f"model input count {B.shape[1]} != limits count {len(limits)}")

    x0 = default_initial_state(names, args.preset)
    u_bias_raw, u_bias_clip, bias_residual = solve_bias(B, c, limits)
    xs_affine, us_affine, raw_affine, sat_affine = simulate(A, B, c, K, x0, steps, limits, use_c=True)
    xs_bias, us_bias, raw_bias, sat_bias = simulate(
        A, B, c, K, x0, steps, limits, use_c=True, u_bias=u_bias_clip
    )
    xs_linear, us_linear, raw_linear, sat_linear = simulate(A, B, c, K, x0, steps, limits, use_c=False)

    print(f"loaded: {path}")
    print(f"model: {args.model}")
    print(f"states: {names}")
    print(f"R2: {np.array2string(r2, precision=4)}")
    print(f"x_trim: {np.array2string(x_trim, precision=5, suppress_small=True)}")
    print(f"input limits: {np.array2string(limits, precision=5)}")
    print(f"u_bias least-squares raw:     {np.array2string(u_bias_raw, precision=5, suppress_small=True)}")
    print(f"u_bias clipped to limits:     {np.array2string(u_bias_clip, precision=5, suppress_small=True)}")
    print(f"affine residual after bias:   ||B*u_bias+c||={np.linalg.norm(bias_residual):.5g}")
    summarize("affine with identified c", xs_affine, us_affine, sat_affine, names, dt)
    summarize("affine with clipped u_bias", xs_bias, us_bias, sat_bias, names, dt)
    summarize("linear c=0", xs_linear, us_linear, sat_linear, names, dt)

    out = args.out
    if out is None:
        out = str(PROJECT_ROOT / "logs" / f"trim_lqr_sim_{args.model}_{args.preset}.png")
    t = np.arange(steps + 1) * dt
    plot_result(
        out,
        t,
        names,
        [
            ("affine", xs_affine, us_affine),
            ("bias", xs_bias, us_bias),
            ("linear", xs_linear, us_linear),
        ],
    )


if __name__ == "__main__":
    main()

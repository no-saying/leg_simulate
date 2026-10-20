"""Evaluate baseline or a trained policy with identical slope-spin tasks."""

import argparse
import itertools
import json
from pathlib import Path

import numpy as np
import torch
from spin_anchor_env import TASK_VERSION as ANCHOR_TASK_VERSION
from spin_anchor_env import AnchorSpinEnv
from spin_asymmetric_env import AsymmetricSpinEnv
from spin_asymmetric_env import TASK_VERSION as ASYMMETRIC_TASK_VERSION
from spin_mixed_env import TASK_VERSION as MIXED_TASK_VERSION
from spin_mixed_env import MixedTerrainSpinEnv, validation_cases
from spin_rl_env import TASK_VERSION, SpinRLBatchEnv
from spin_return_env import ReturnTrainingSpinEnv
from spin_standard_env import TASK_VERSION as STANDARD_TASK_VERSION
from spin_standard_env import StandardSpinEnv
from spin_travel_env import TASK_VERSION as TRAVEL_TASK_VERSION
from spin_travel_env import TravelSpinEnv
from spin_wrench_env import WrenchSpinEnv, ReturnWrenchSpinEnv
from stable_baselines3 import PPO


def evaluate(model, env):
    obs = env.reset()
    if model is not None and model.observation_space != env.observation_space:
        raise ValueError("Checkpoint observation contract differs from the current spin task")
    results = {}
    while len(results) < env.num_envs:
        actions = np.zeros((env.num_envs, *env.action_space.shape), np.float32) if model is None else model.predict(
            obs, deterministic=True)[0]
        obs, _, dones, infos = env.step(actions)
        for i in np.flatnonzero(dones):
            if i not in results:
                results[int(i)] = {**infos[i]["spin_metrics"], "return": infos[i]["episode"]["r"]}
    return [results[i] for i in range(env.num_envs)]


def summary(rows):
    stats = {
        "cases": len(rows),
        "completed": sum(not r["failed"] and not r["stalled"] for r in rows),
        "mean_return": float(np.mean([r["return"] for r in rows])),
        "mean_roll_rms_deg": float(np.mean([r["roll_rms_deg"] for r in rows])),
        "mean_max_drift_m": float(np.mean([r["max_drift_m"] for r in rows])),
        "mean_duration_s": float(np.mean([r["duration_s"] for r in rows])),
        "mean_yaw_angle_ratio": float(np.mean([r["yaw_angle_ratio"] for r in rows])),
        "min_yaw_angle_ratio": float(np.min([r["yaw_angle_ratio"] for r in rows])),
    }
    def band(row, standard=False):
        length = row["standard_requested_initial_m"] if standard else 0.117-row["extension"]
        return "short" if length < 0.09 else "long" if length > 0.165 else "middle"
    def bands(standard=False):
        return {name: {"cases": len(group),
               "completed": sum(not r["failed"] and not r["stalled"] for r in group),
               "mean_return": float(np.mean([r["return"] for r in group])),
               "mean_roll_rms_deg": float(np.mean([r["roll_rms_deg"] for r in group])),
               "mean_max_drift_m": float(np.mean([r["max_drift_m"] for r in group])),
               "mean_yaw_angle_ratio": float(np.mean([r["yaw_angle_ratio"] for r in group]))}
        for name in ("short", "middle", "long")
        if (group := [r for r in rows if band(r, standard) == name])}
    if "standard_requested_initial_m" in rows[0]:
        stats["leg_band_basis"] = "initial mapped uphill-leg request"
        stats["by_leg_band"] = bands(standard=True)
        stats["by_nominal_common_leg_band"] = bands()
        stats["mean_standard_actual_m"] = float(np.mean([r["standard_actual_mean_m"] for r in rows]))
        stats["mean_standard_infeasible_fraction"] = float(np.mean([
            r["standard_infeasible_fraction"] for r in rows]))
    else:
        stats["by_leg_band"] = bands()
    if "handover_requested" in rows[0]:
        observed = {"pid": 0, "lqr": 0, "mixed": 0, "no_takeover": 0}
        for row in rows:
            mix = row["takeover_actual_lqr_mix"]
            observed["no_takeover" if mix is None else
                     "pid" if mix <= 0.001 else "lqr" if mix >= 0.999 else "mixed"] += 1
        stats["actual_takeovers"] = observed
        stats["boundary_takeovers"] = sum(
            min(r["takeover_wheel_zones"]) >= 0 and
            len(set(r["takeover_wheel_zones"])) > 1 for r in rows)
        stats["by_location"] = {
            location: {"cases": len(group),
                       "completed": sum(not r["failed"] and not r["stalled"] for r in group),
                       "mean_roll_rms_deg": float(np.mean([r["roll_rms_deg"] for r in group]))}
            for location in sorted({r["terrain_location"] for r in rows})
            if (group := [r for r in rows if r["terrain_location"] == location])}
    if "bounded_spin_pass" in rows[0]:
        stats.update(
            bounded_spin_passed=sum(r["bounded_spin_pass"] for r in rows),
            mean_inside_fraction=float(np.mean([r["inside_fraction"] for r in rows])),
            mean_final_drift_m=float(np.mean([r["final_drift_m"] for r in rows])),
            return_attempts=sum(r["return_attempts"] for r in rows),
            return_successes=sum(r["return_successes"] for r in rows),
            unrecovered_cases=sum(r["unrecovered"] for r in rows),
            mean_odometry_peak_error_m=float(np.mean([r["odometry_peak_error_m"] for r in rows])))
        for name, group_stats in stats["by_leg_band"].items():
            group = [r for r in rows if band(r, standard=True) == name]
            group_stats.update(
                bounded_spin_passed=sum(r["bounded_spin_pass"] for r in group),
                mean_inside_fraction=float(np.mean([r["inside_fraction"] for r in group])),
                mean_final_drift_m=float(np.mean([r["final_drift_m"] for r in group])),
                unrecovered_cases=sum(r["unrecovered"] for r in group))
    if 'full_rl_control_seconds' in rows[0]:
        stats['mean_full_rl_control_seconds'] = float(np.mean([r['full_rl_control_seconds'] for r in rows]))
        stats['native_torque_leak_peak_ctrl'] = max(r['native_torque_leak_peak_ctrl'] for r in rows)
    if 'rl_pitch_rms_deg' in rows[0]:
        stats['mean_rl_pitch_rms_deg'] = float(np.mean([r['rl_pitch_rms_deg'] for r in rows]))
        stats['max_rl_pitch_peak_deg'] = max(r['rl_pitch_peak_deg'] for r in rows)
    return stats


def evaluate_cases(model, cases, terrain=None, *, batch_size=16, threads=8,
                   seed=1000, pid_only=False, travel=False, standard=False, anchor=False,
                   episode_seconds=None, disturbances=True, asymmetric=False, control_mode='residual',
                   torque_loop='ideal', linkage_map='native', return_stage=None, reference_mode='spin',
                   reward_profile='spin'):
    """Bound the number of native controller libraries and MuJoCo scenes in memory."""
    if batch_size < 1 or threads < 1 or not cases:
        raise ValueError("Positive batch size, threads and nonempty cases required")
    if return_stage is not None and not asymmetric:
        raise ValueError('Return gate evaluation requires asymmetric control')
    rows, provenance = [], None
    for start in range(0, len(cases), batch_size):
        chunk = cases[start:start+batch_size]
        env_type = (ReturnWrenchSpinEnv if return_stage is not None and control_mode == 'wrench' else ReturnTrainingSpinEnv if return_stage is not None else WrenchSpinEnv if control_mode == 'wrench' else AsymmetricSpinEnv if asymmetric else AnchorSpinEnv if anchor else StandardSpinEnv if standard else TravelSpinEnv if travel
                    else SpinRLBatchEnv if terrain is None else MixedTerrainSpinEnv)
        kwargs = {} if terrain is None else {"terrain_cases": terrain[start:start+batch_size]}
        if episode_seconds is not None:
            kwargs["episode_seconds"] = episode_seconds
        if anchor or asymmetric:
            kwargs["disturbances"] = disturbances
        if asymmetric:
            kwargs['torque_loop' if control_mode == 'wrench' else 'residual_torque_loop'] = torque_loop
            if control_mode == 'wrench':
                kwargs['reward_profile'] = reward_profile
            if control_mode == 'residual':
                kwargs['linkage_map'] = linkage_map
        if return_stage is not None:
            kwargs.update(return_stage=return_stage, gate_trials=True)
        env = env_type(len(chunk), min(threads, len(chunk)), cases=chunk,
                       pid_only=pid_only, reference_mode=reference_mode, **kwargs)
        try:
            env.seed(seed+start)
            rows.extend(evaluate(model, env))
            provenance = env.provenance
        finally:
            env.close()
        print(f"EVAL cases={len(rows)}/{len(cases)}", flush=True)
    return rows, provenance


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--pid-only", action="store_true")
    parser.add_argument('--control-mode', choices=('residual', 'wrench'), default='residual')
    parser.add_argument('--reference-mode', choices=('spin', 'hold'), default='spin')
    parser.add_argument('--reward-profile', choices=('spin', 'pitch'), default='spin')
    parser.add_argument('--torque-loop', choices=('ideal', 'open', 'pi'), default='ideal')
    parser.add_argument('--linkage-map', choices=('native', 'exact'), default='native')
    parser.add_argument("--terrain", choices=("mixed", "plane", "travel", "standard", "anchor", "asymmetric"), default="mixed")
    parser.add_argument("--episode-seconds", type=float)
    parser.add_argument("--no-disturbances", action="store_true")
    parser.add_argument("--slopes", nargs="+", type=float, default=[0, 3, 5, 8])
    parser.add_argument("--extensions", nargs="+", type=float, default=[-0.093, -0.04, 0, 0.04, 0.070])
    parser.add_argument("--headings", nargs="+", type=float, default=[0, 90])
    parser.add_argument("--boundary-offsets", nargs="+", type=float, default=[-0.12, 0, 0.12])
    parser.add_argument("--callback-grid", action="store_true",
                        help="Use the exact callback headings and zero boundary offsets")
    parser.add_argument('--screen-grid', action='store_true',
                        help='30 cases: all locations, all three lengths, both directions at 8deg/LQR')
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--out", type=Path, default=Path("logs/rl_training/spin_eval.json"))
    args = parser.parse_args()
    if args.reward_profile == 'pitch' and (args.control_mode != 'wrench' or args.reference_mode != 'hold'):
        parser.error('Pitch rewards require wrench control and hold reference')
    if args.control_mode == 'wrench' and args.terrain != 'asymmetric':
        parser.error('Wrench control requires asymmetric terrain')
    if args.control_mode == 'wrench' and args.torque_loop == 'open':
        parser.error('Open-loop motor comparison is available for residual control only')
    if args.torque_loop != 'ideal' and args.terrain != 'asymmetric':
        parser.error('Torque loop requires asymmetric terrain')
    if args.linkage_map != 'native' and (args.terrain != 'asymmetric' or args.control_mode == 'wrench'):
        parser.error('Exact remapping requires asymmetric residual control')
    if args.screen_grid and args.terrain == 'plane':
        parser.error('Screen grid requires mixed terrain')
    torch.set_num_threads(1)
    model = PPO.load(args.model, device="cpu") if args.model else None
    cases = [(slope, np.deg2rad(heading), ext, direction)
             for slope, ext, heading, direction in itertools.product(
                 args.slopes, args.extensions, args.headings, [-1, 1])]
    mixed = args.terrain != "plane"
    terrain = None
    if mixed:
        cases, terrain = (validation_cases(slopes=(8,), handovers=('lqr',)) if args.screen_grid else
                          validation_cases(args.slopes, args.extensions,
                                           None if args.callback_grid else np.deg2rad(args.headings),
                                           (0,) if args.callback_grid else args.boundary_offsets))
    results, provenance = evaluate_cases(model, cases, terrain, batch_size=args.batch_size,
                                         threads=args.threads, seed=args.seed, pid_only=args.pid_only,
                                         travel=args.terrain == "travel", standard=args.terrain == "standard",
                                         anchor=args.terrain == "anchor", episode_seconds=args.episode_seconds,
                                         asymmetric=args.terrain == "asymmetric",
                                         control_mode=args.control_mode,
                                         torque_loop=args.torque_loop,
                                         linkage_map=args.linkage_map,
                                         reference_mode=args.reference_mode,
                                         reward_profile=args.reward_profile,
                                         disturbances=not args.no_disturbances)
    for row in results:
        print(json.dumps(row), flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    version = (ASYMMETRIC_TASK_VERSION if args.terrain == "asymmetric" else ANCHOR_TASK_VERSION if args.terrain == "anchor" else STANDARD_TASK_VERSION
               if args.terrain == "standard" else TRAVEL_TASK_VERSION
               if args.terrain == "travel" else MIXED_TASK_VERSION if mixed else TASK_VERSION)
    args.out.write_text(json.dumps({"task": provenance.get('task', version),
                                   "evaluation_grid": 'screen' if args.screen_grid else 'callback' if args.callback_grid else 'custom',
                                   "provenance": provenance, "seed": args.seed,
                                   "model": str(args.model) if args.model else None,
                                   "summary": summary(results), "results": results}, indent=2)+"\n")
    print(json.dumps(summary(results)), flush=True)


if __name__ == "__main__":
    main()

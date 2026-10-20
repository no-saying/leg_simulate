"""Record deployed controller state alongside actual slope-spin motion."""

import argparse
import csv
import ctypes
import itertools
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import spin_rl_env
from spin_rl_env import SpinRLBatchEnv, attitude
from stable_baselines3 import PPO

GETTERS = {
    "manual": ("balance_get_manual_active", ctypes.c_int),
    "post_manual": ("balance_get_post_manual", ctypes.c_int),
    "spin": ("balance_get_manual_spin", ctypes.c_int),
    "neutral": ("balance_get_neutral", ctypes.c_float),
    "swing_ref": ("balance_get_leg_swing", ctypes.c_float),
    "swing_diff_ref": ("balance_get_leg_swing_diff", ctypes.c_float),
    "hold_pos": ("balance_get_hold_pos", ctypes.c_float),
    "wheel_body_v": ("balance_get_wheel_body_v", ctypes.c_float),
    "imu_v": ("balance_get_slip_inertial_v", ctypes.c_float),
    "slip_yaw": ("balance_get_slip_yaw_blend", ctypes.c_float),
    "contact": ("balance_get_contact_mode", ctypes.c_float),
    "ext_ref": ("balance_get_leg_extend", ctypes.c_float),
    "ext_manual": ("balance_get_leg_extend_manual", ctypes.c_float),
    "obstacle_retract": ("balance_get_obstacle_retract", ctypes.c_float),
}


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--xml", type=Path)
    parser.add_argument("--pid-only", action="store_true")
    parser.add_argument("--define", action="append", default=[], metavar="MACRO=NUMBER")
    parser.add_argument("--slopes", type=float, nargs="+", default=[0, 3])
    parser.add_argument("--extensions", type=float, nargs="+", default=[-0.093, 0.070])
    parser.add_argument("--directions", type=int, choices=[-1, 1], nargs="+", default=[-1, 1])
    parser.add_argument("--duration", type=float, default=20)
    parser.add_argument("--recovery-seconds", type=float, default=4)
    parser.add_argument("--reference", choices=["spin", "hold"], default="spin")
    parser.add_argument("--pitch-kick", type=float, default=0,
                        help="Diagnostic body pitch angular velocity increment in rad/s")
    parser.add_argument("--kick-time", type=float, default=5)
    parser.add_argument("--snapshot-times", type=float, nargs="*", default=[])
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if not np.isfinite([args.pitch_kick, args.kick_time]).all() or args.kick_time < 0:
        parser.error("Kick and time must be finite; time must be nonnegative")
    if args.xml:
        spin_rl_env.XML = str(args.xml.resolve())
    torch.set_num_threads(1)
    cases = [(s, 0, e, d) for s, e, d in itertools.product(
        args.slopes, args.extensions, args.directions)]
    defines = dict(value.split("=", 1) for value in args.define)
    env = SpinRLBatchEnv(len(cases), min(8, len(cases)), cases=cases, pid_only=args.pid_only,
                        controller_defines=defines, episode_seconds=args.duration,
                        reference_mode=args.reference, recovery_seconds=args.recovery_seconds)
    model = PPO.load(args.model, device="cpu") if args.model else None
    libraries, getters = [], []
    for i in range(len(cases)):
        library = ctypes.CDLL(str(Path(env.control._directory.name)/f"controller_{i}.so"))
        functions = {}
        for name, (symbol, result_type) in GETTERS.items():
            function = getattr(library, symbol)
            function.argtypes, function.restype = [], result_type
            functions[name] = function
        for name, symbol, result_type in (
            ("retract", "balance_get_obstacle_leg_retract", ctypes.c_float),
            ("mismatch", "balance_get_wheel_mismatch", ctypes.c_float),
            ("obstacle", "balance_get_wheel_obstacle", ctypes.c_int),
        ):
            function = getattr(library, symbol)
            function.argtypes, function.restype = [ctypes.c_int], result_type
            for right, side in enumerate(("left", "right")):
                functions[f"{side}_{name}"] = lambda f=function, r=right: f(r)
        allocation = library.robot_control_get_leg_allocation
        allocation.argtypes, allocation.restype = [ctypes.POINTER(ctypes.c_float)], None
        state = (ctypes.c_float*4)()
        for k, name in enumerate(("left_ext_applied", "right_ext_applied", "height_lift", "roll_limited")):
            def read_allocation(f=allocation, buffer=state, index=k):
                f(buffer)
                return buffer[index]
            functions[name] = read_allocation
        libraries.append(library)
        getters.append(functions)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    results, rows, snapshots, kicks = {}, [], [], []
    kicked = np.zeros(len(cases), dtype=bool)
    snapshot_steps = {round(t/env.policy_dt) for t in args.snapshot_times}
    try:
        env.seed(1000)
        obs = env.reset()
        while len(results) < len(cases):
            for i in range(len(cases)):
                if (args.pitch_kick and i not in results and not kicked[i]
                        and env.steps[i]*env.policy_dt >= args.kick_time):
                    q, gyro, _ = env._imu()
                    _, _, yaw, _, _ = attitude(q, gyro)
                    # Free-joint angular velocity is local; kick the horizontal
                    # pitch axis at the current heading without changing pose.
                    rotation = np.empty(9)
                    mujoco.mju_quat2Mat(rotation, env.qpos[i, 3:7])
                    axis = np.array([-np.sin(yaw[i]), np.cos(yaw[i]), 0])
                    env.qvel[i, 3:6] += rotation.reshape(3, 3).T@axis*args.pitch_kick
                    env.batch.forward(ids=np.array([i], dtype=np.int32))
                    kicked[i] = True
                    kicks.append({"case": i, "time": float(env.steps[i]*env.policy_dt),
                                  "pitch_rate_delta_rad_s": args.pitch_kick})
            native = env.control.spin_observation()
            q, gyro, _ = env._imu()
            roll, pitch, _, roll_rate, yaw_rate = attitude(q, gyro)
            for i, (slope, _, ext, direction) in enumerate(cases):
                if i in results:
                    continue
                if env.steps[i] in snapshot_steps:
                    snapshots.append((i, env.steps[i]*env.policy_dt,
                                      env.qpos[i].copy(), env.qvel[i].copy(), env.ctrl[i].copy(),
                                      env.floor_quat[i, env.floor].copy(),
                                      env.floor_friction[i, env.floor].copy()))
                row = {"case": i, "time": env.steps[i]*env.policy_dt,
                       "slope": slope, "extension": ext, "direction": direction,
                       "pitch_deg": np.rad2deg(pitch[i]), "roll_deg": np.rad2deg(roll[i]),
                       "roll_rate": roll_rate[i], "yaw_rate": yaw_rate[i],
                       "yaw_target": env.yaw_target[i], "est_v": native[i, 19],
                       "world_speed": np.linalg.norm(env.qvel[i, :2]),
                       "drift": np.linalg.norm(env.qpos[i, :2]-env.start_xy[i]),
                       "left_length": native[i, 11], "right_length": native[i, 12],
                       "left_swing": native[i, 15], "right_swing": native[i, 16],
                       "left_rate": native[i, 9], "right_rate": native[i, 10],
                       "common_nm": 5*(env.ctrl[i, 2]+env.ctrl[i, 5]),
                       "yaw_nm": 5*(env.ctrl[i, 5]-env.ctrl[i, 2]),
                       "lqr_mix": native[i, 27],
                       **{name: function() for name, function in getters[i].items()}}
                rows.append(row)
            actions = np.zeros((env.num_envs, 5), np.float32) if model is None else model.predict(
                obs, deterministic=True)[0]
            obs, _, done, infos = env.step(actions)
            for i in np.flatnonzero(done):
                if i not in results:
                    results[int(i)] = infos[i]["spin_metrics"]
        with args.out.with_suffix(".csv").open("w") as output:
            writer = csv.DictWriter(output, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        args.out.with_suffix(".json").write_text(json.dumps(list(results.values()), indent=2)+"\n")
        args.out.with_suffix(".meta.json").write_text(json.dumps({
            "provenance": env.provenance, "seed": 1000,
            "model": str(args.model) if args.model else None,
            "cases": cases,
            "duration_s": args.duration, "reference": args.reference, "kicks": kicks,
        }, indent=2)+"\n")
        if snapshots:
            np.savez_compressed(args.out.with_suffix(".npz"), **{
                key: np.asarray([row[j] for row in snapshots]) for j, key in enumerate(
                    ("case", "time", "qpos", "qvel", "ctrl", "floor_quat", "floor_friction"))})
        for result in results.values():
            print(json.dumps({k: result[k] for k in (
                "slope_deg", "extension", "direction", "failure_reasons", "failure_phase",
                "yaw_angle_rad", "max_drift_m")}))
    finally:
        env.close()


if __name__ == "__main__":
    main()

"""Trace return actions, reference allocation and physical motion on matched trials."""

import argparse
import csv
import ctypes
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from eval_spin_rl import summary
from spin_anchor_env import heading_rotate
from spin_return_env import ReturnTrainingSpinEnv, return_velocity_reference
from spin_rl_env import attitude
from stable_baselines3 import PPO


REFERENCE_FIELDS = ('native_swing_ref', 'rl_swing_offset', 'final_swing_ref',
                    'left_swing_ref', 'right_swing_ref', 'swing_deadzone',
                    'swing_kp', 'swing_kd', 'left_swing', 'right_swing',
                    'native_speed_target', 'native_pitch_target')
MIX_FIELDS = ('native_common', 'native_yaw', 'rl_common', 'rl_yaw',
              'common_before_caps', 'common_after_caps', 'left_wheel_nm', 'right_wheel_nm')


def reference_readers(env):
    libraries, readers = [], []
    for i in range(env.num_envs):
        library = ctypes.CDLL(str(Path(env.control._directory.name)/f'controller_{i}.so'))
        getter = library.robot_control_get_spin_reference_audit
        getter.argtypes, getter.restype = [ctypes.POINTER(ctypes.c_float)], None
        buffer = (ctypes.c_float*12)()
        def read(function=getter, output=buffer):
            function(output)
            return np.array(output)
        readers.append(read)
        libraries.append(library)
    return libraries, readers


def probe_action(actions, env, mode):
    if mode == 'policy':
        return actions
    result = actions.copy()
    if mode == 'zero':
        return np.zeros_like(result)
    _, _, yaw, _, _ = attitude(*env._imu()[:2])
    position = heading_rotate(env.odometry.position, -yaw)
    velocity = heading_rotate(env.odometry.velocity, -yaw)
    forward = np.clip((-position[:, 0]-.7*velocity[:, 0])/.25, -1, 1)
    # Probes replace only the selected channel, retaining native balance.
    channel = 3 if mode.startswith('wheel') else 2
    result[:, channel] = forward*(1 if mode.endswith('plus') else -1)
    return result


def run(model, args, mode):
    cases = [(args.slope, .7 if location not in ('toe', 'crest') else np.pi/2,
              .117-length, direction) for location in args.locations
             for length in args.lengths for direction in (-1, 1)]
    terrain = [dict(location=location, handover=args.handover, offset=0,
                    takeover_time=0, fade=.3, dropout=False)
               for location in args.locations for _ in range(2*len(args.lengths))]
    env = ReturnTrainingSpinEnv(len(cases), min(args.threads, len(cases)),
        cases=cases, terrain_cases=terrain, gate_trials=True, return_stage=args.stage,
        episode_seconds=args.seconds, disturbances=False, linkage_map='exact')
    records, finished = [], {}
    try:
        env.seed(args.seed)
        obs = env.reset()
        libraries, readers = reference_readers(env)
        while len(finished) < env.num_envs:
            native = env.control.spin_observation().copy()
            mix = env.control.wheel_mix_audit().copy()
            filtered = env.control.spin_actions().copy()
            q, gyro, _ = env._imu()
            roll, pitch, yaw, _, yaw_rate = attitude(q, gyro)
            displacement = env.qpos[:, :2]-env.start_xy
            local = heading_rotate(displacement, -yaw)
            estimated = heading_rotate(env.odometry.position, -yaw)
            velocity = heading_rotate(env.qvel[:, :2], -yaw)
            reference = heading_rotate(return_velocity_reference(displacement), -yaw)
            raw = model.predict(obs, deterministic=True)[0]
            mirrored = obs.copy()
            mirrored[:, :448].reshape(-1, 8, 56)[:, :, 52:54] *= -1
            counterfactual = model.predict(mirrored, deterministic=True)[0]
            actions = probe_action(raw, env, mode)
            for i in range(env.num_envs):
                if i in finished:
                    continue
                row = dict(case=i, time=float(env.steps[i]*env.policy_dt),
                           location=env.location[i], length=.117-env.ext[i],
                           direction=int(env.direction[i]),
                           true_body_x=local[i, 0], true_body_y=local[i, 1],
                           estimated_body_x=estimated[i, 0], estimated_body_y=estimated[i, 1],
                           body_vx=velocity[i, 0], body_vy=velocity[i, 1],
                           return_vx=reference[i, 0], return_vy=reference[i, 1],
                           radius=float(np.linalg.norm(displacement[i])),
                           yaw_rate=yaw_rate[i], yaw_target=env.yaw_target[i],
                           roll_deg=np.rad2deg(roll[i]), pitch_deg=np.rad2deg(pitch[i]),
                           lqr_mix=native[i, 27])
                row.update(zip(MIX_FIELDS, mix[i]))
                row.update(zip(REFERENCE_FIELDS, readers[i]()))
                for k in range(5):
                    row[f'policy_{k}'] = raw[i, k]
                    row[f'action_{k}'] = actions[i, k]
                    row[f'filtered_{k}'] = filtered[i, k]
                    row[f'mirrored_position_action_{k}'] = counterfactual[i, k]
                records.append(row)
            obs, _, done, infos = env.step(actions)
            for i in np.flatnonzero(done):
                if i not in finished:
                    finished[int(i)] = {**infos[i]['spin_metrics'], 'return': infos[i]['episode']['r']}
        prefix = args.out.with_name(args.out.name+'_'+mode)
        with prefix.with_suffix('.csv').open('w') as output:
            writer = csv.DictWriter(output, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
        rows = [finished[i] for i in range(env.num_envs)]
        result = dict(mode=mode, model=str(args.model),
            model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
            seed=args.seed, provenance=env.provenance, summary=summary(rows), results=rows)
        prefix.with_suffix('.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(dict(mode=mode, summary=result['summary'])), flush=True)
    finally:
        env.close()


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--modes', nargs='+', default=['policy'], choices=(
        'policy', 'zero', 'wheel_plus', 'wheel_minus', 'swing_plus', 'swing_minus'))
    parser.add_argument('--locations', nargs='+', default=['lower_flat', 'ramp', 'toe'])
    parser.add_argument('--lengths', nargs='+', type=float, default=[.047, .117, .210])
    parser.add_argument('--handover', choices=('pid', 'lqr'), default='lqr')
    parser.add_argument('--seconds', type=float, default=18)
    parser.add_argument('--slope', type=float, default=3)
    parser.add_argument('--stage', type=int, choices=(0, 1, 2), default=0)
    parser.add_argument('--threads', type=int, default=8)
    parser.add_argument('--seed', type=int, default=2000)
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(1)
    model = PPO.load(args.model, device='cpu')
    for mode in args.modes:
        run(model, args, mode)


if __name__ == '__main__':
    main()

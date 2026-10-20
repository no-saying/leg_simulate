"""Score real spin/return behavior with physical pushes and sensor-only actors."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from eval_spin_rl import evaluate, summary
from spin_asymmetric_env import AsymmetricSpinEnv
from spin_asymmetric_policy import ACTOR_SIZE
from spin_mixed_env import HANDOVERS
from spin_wrench_control import TORQUE_LIMIT, WrenchTorqueMapper
from stable_baselines3 import PPO


class FrozenSensorActor:
    """Evaluate an old actor on current sensors without changing its weights."""

    def __init__(self, model, observation_space):
        self.model = model
        self.observation_space = observation_space
        if model.observation_space.shape[0] not in (ACTOR_SIZE, observation_space.shape[0]):
            raise ValueError('Unsupported actor observation size')

    def predict(self, obs, deterministic=True):
        return self.model.predict(obs[:, :self.model.observation_space.shape[0]],
                                  deterministic=deterministic)


class ClosedLoopAuditEnv(AsymmetricSpinEnv):
    def __init__(self, *args, push_at=7., push_velocity=(.15, -.10), **kwargs):
        self.audit_push_at = push_at
        self.audit_push_velocity = push_velocity
        super().__init__(*args, **kwargs)
        self.wrench_mapper = WrenchTorqueMapper(self.model)
        self.mix_sum = np.zeros((self.num_envs, 8))
        self.mix_count = np.zeros(self.num_envs)
        self.opposition_count = np.zeros(self.num_envs)
        self.active_residual_count = np.zeros(self.num_envs)
        self.cap_count = np.zeros(self.num_envs)
        self.body_wrench_peak = np.zeros((self.num_envs, 6))
        self.mapping_invalid_count = np.zeros(self.num_envs)

    def _after_physics_step(self):
        super()._after_physics_step()
        audit = self.control.wheel_mix_audit()
        self.mix_sum += np.abs(audit)
        self.mix_count += 1
        active = np.abs(audit[:, 2]) > .005
        self.active_residual_count += active
        self.opposition_count += active & (audit[:, 0]*audit[:, 2] < 0)
        self.cap_count += np.abs(audit[:, 4]-audit[:, 5]) > 1e-5

    def _before_policy_step(self):
        super()._before_policy_step()
        torque = np.clip(self.actuator_force*self.model.actuator_gear[:, 0], -TORQUE_LIMIT, TORQUE_LIMIT)
        wrench, _, valid, _ = self.wrench_mapper.equivalent_body_wrench(
            self.qpos[:, self.qa[:4]], torque, self.axle_lateral)
        self.body_wrench_peak[:] = np.maximum(self.body_wrench_peak, np.where(
            valid.all(axis=1)[:, None], np.abs(wrench), 0))
        self.mapping_invalid_count += ~valid.all(axis=1)

    def _reset_context(self, ids):
        super()._reset_context(ids)
        self.push_time[ids] = self.audit_push_at
        self.push_velocity[ids] = self.audit_push_velocity
        for value in (self.mix_sum, self.mix_count, self.opposition_count,
                      self.active_residual_count, self.cap_count, self.body_wrench_peak,
                      self.mapping_invalid_count):
            value[ids] = 0

    def _extra_metrics(self, i):
        return {**super()._extra_metrics(i),
                'true_final_displacement_xy_m': (self.qpos[i, :2]-self.start_xy[i]).tolist(),
                'estimated_final_displacement_xy_m': self.odometry.position[i].tolist(),
                'final_wheel_confidence': self.odometry.wheel_confidence[i].tolist(),
                'push_time_s': self.audit_push_at,
                'push_delta_v_nominal_mps': list(self.audit_push_velocity),
                'mean_abs_wheel_mix_nm': (self.mix_sum[i]/max(self.mix_count[i], 1)).tolist(),
                'active_common_residual_seconds': float(self.active_residual_count[i]*self.dt),
                'native_opposes_rl_common_fraction': float(self.opposition_count[i]/max(self.active_residual_count[i], 1)),
                'common_speed_cap_fraction': float(self.cap_count[i]/max(self.mix_count[i], 1)),
                'equivalent_body_wrench_peak': self.body_wrench_peak[i].tolist(),
                'invalid_wrench_mapping_frames': int(self.mapping_invalid_count[i])}


def run(args, model, controller, friction):
    cases = [(args.slope, .7 if loc not in ('toe', 'crest') else np.pi/2,
              .117-length, direction)
             for loc in args.locations for length in args.lengths for direction in (-1, 1)]
    terrain = [dict(location=loc, handover=args.handover, offset=0, takeover_time=0,
                    fade=.3, dropout=False)
               for loc in args.locations for _ in range(2*len(args.lengths))]
    rows, provenance = [], None
    for start in range(0, len(cases), args.batch_size):
        chunk = cases[start:start+args.batch_size]
        env = ClosedLoopAuditEnv(len(chunk), min(args.threads, len(chunk)), cases=chunk,
                                terrain_cases=terrain[start:start+len(chunk)],
                                episode_seconds=args.seconds, disturbances=True,
                                push_at=args.push_at, push_velocity=args.push_velocity,
                                residual_torque_loop=args.torque_loop,
                                linkage_map=args.linkage_map,
                                directional_reference=args.reference_fusion == 'directional')
        try:
            geoms = [env.model.geom(n).id for n in (
                'floor', 'spin_ramp', 'spin_upper_flat', 'L_Wheel_mesh_1', 'R_Wheel_mesh_1')]
            env.model.geom_friction[geoms, 0] = friction
            env.floor_friction[:, geoms, 0] = friction
            env.seed(args.seed+start)
            actor = None if controller == 'zero' else FrozenSensorActor(model, env.observation_space)
            rows.extend(evaluate(actor, env))
            provenance = env.provenance
        finally:
            env.close()
        print(f'{controller} friction={friction} cases={len(rows)}/{len(cases)}', flush=True)
    return dict(controller=controller, friction=friction, provenance=provenance,
                summary=summary(rows), results=rows)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--model', type=Path,
                        default=Path('logs/rl_training/spin_anchor_v6_resume131k_to1m.zip'))
    parser.add_argument('--controllers', nargs='+', choices=('zero', 'frozen'), default=['zero', 'frozen'])
    parser.add_argument('--frictions', type=float, nargs='+', default=[1, .3])
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--slope', type=float, default=8)
    parser.add_argument('--lengths', type=float, nargs='+', default=[.047, .117, .210])
    parser.add_argument('--locations', nargs='+', default=['lower_flat', 'ramp', 'toe'],
                        choices=('lower_flat', 'ramp', 'toe', 'crest', 'upper_flat'))
    parser.add_argument('--handover', choices=HANDOVERS, default='lqr')
    parser.add_argument('--torque-loop', choices=('ideal', 'open', 'pi'), default='ideal')
    parser.add_argument('--linkage-map', choices=('native', 'exact'), default='native')
    parser.add_argument('--reference-fusion', choices=('directional', 'all-required'), default='directional')
    parser.add_argument('--push-at', type=float, default=7)
    parser.add_argument('--push-velocity', type=float, nargs=2, default=[.15, -.10])
    parser.add_argument('--batch-size', type=int, default=12)
    parser.add_argument('--threads', type=int, default=3)
    parser.add_argument('--seed', type=int, default=1000)
    parser.add_argument('--out', type=Path, default=Path('logs/rl_training/spin_closed_loop_audit.json'))
    args = parser.parse_args()
    if args.batch_size < 1 or args.threads < 1:
        parser.error('batch-size and threads must be positive')
    if not np.isfinite(args.frictions).all() or min(args.frictions) <= 0:
        parser.error('frictions must be finite and positive')
    torch.set_num_threads(1)
    model = PPO.load(args.model, device='cpu') if 'frozen' in args.controllers else None
    groups = []
    for controller in args.controllers:
        for friction in args.frictions:
            group = run(args, model, controller, friction)
            groups.append(group)
            print(json.dumps({k: v for k, v in group.items() if k in ('controller', 'friction', 'summary')}), flush=True)
    output = dict(model=str(args.model) if model is not None else None,
                  model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest() if model is not None else None,
                  seconds=args.seconds, seed=args.seed,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), groups=groups)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, indent=2)+'\n')


if __name__ == '__main__':
    main()

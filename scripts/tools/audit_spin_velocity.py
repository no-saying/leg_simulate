"""Compare directional sensor fusion on identical spin/hold trajectories.

All world state and contacts below are scoring signals only. Every shadow
estimator receives only calibrated IMU and the native encoder observations.
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spin_anchor_env import AnchorSpinEnv, SpinOdometry, heading_rotate
from spin_imu import aligned_control_attitude, imu_attitude_extrinsic, multiply
from spin_leg_geometry import EncoderLegGeometry
from spin_rl_env import attitude
from spin_sensor_odometry import DirectionalSpinOdometry, InnovationSpinOdometry, center_acceleration, wheel_kinematics
from stable_baselines3 import PPO


class VelocityAuditEnv(AnchorSpinEnv):
    def __init__(self, *args, sensor_stress=False, sensor_seed=417, compare_reference=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.sensor_stress = sensor_stress
        self.compare_reference = compare_reference
        self.sensor_rng = np.random.default_rng(sensor_seed)
        self.warmup_velocity = np.zeros((self.num_envs, 3))
        self.original_warmup = self.control.warmup

        def measured_warmup(*args, **kwargs):
            delta = np.zeros_like(self.warmup_velocity)
            self.original_warmup(*args, **kwargs, imu_delta=delta)
            self.warmup_velocity += delta

        self.control.warmup = measured_warmup

    def _reset_ids(self, ids):
        self.warmup_velocity[ids] = 0
        super()._reset_ids(ids)

    def _place_on_terrain(self, ids):
        # Scene construction rotates the already moving robot. Match that reset
        # transform using measured yaw change; real deployment never teleports.
        _, _, before, _, _ = attitude(*self._imu()[:2])
        super()._place_on_terrain(ids)
        _, _, after, _, _ = attitude(*self._imu()[:2])
        self.warmup_velocity[ids, :2] = heading_rotate(
            self.warmup_velocity[:, :2], after-before)[ids]

    def reset(self):
        obs = super().reset()
        self.extrinsic = imu_attitude_extrinsic(self.model)
        self.geometry = EncoderLegGeometry(self.model)
        self.finished = np.zeros(self.num_envs, bool)
        self.shadows = {'v7': SpinOdometry(self.num_envs, self.axle_lateral)}
        self.shadows['directional'] = DirectionalSpinOdometry(self.num_envs, self.axle_lateral)
        for name in ('support_spin', 'exact_geometry', 'exact_equal'):
            self.shadows[name] = SpinOdometry(self.num_envs, self.axle_lateral)
        self.shadows['warmup_initialized'] = SpinOdometry(self.num_envs, self.axle_lateral)
        warm = self.shadows['warmup_initialized']
        warm.velocity[:] = self.warmup_velocity[:, :2]
        warm.previous_gyro[:] = self._imu()[1]
        warm.ready[:] = True
        versions = [('innovation', True)]
        if self.compare_reference:
            versions.append(('innovation_all_required', False))
        for prefix, directional in versions:
            for suffix in ('', '_white', '_residual') if self.sensor_stress else ('',):
                name = prefix+suffix
                self.shadows[name] = InnovationSpinOdometry(
                    self.num_envs, self.axle_lateral, directional_reference=directional)
                self.shadows[name].initialize_velocity(self.warmup_velocity[:, :2], self._imu()[1])
        self.stats = {name: dict(peak=np.zeros(self.num_envs),
                                 velocity_sq=np.zeros((self.num_envs, 2))) for name in self.shadows}
        self.sample_count = np.zeros(self.num_envs)
        self.wheel_sq = np.zeros((self.num_envs, 2))
        self.corrected_wheel_sq = np.zeros((self.num_envs, 2))
        self.invalid_geometry = np.zeros((self.num_envs, 2))
        self.sideways_sq = np.zeros(self.num_envs)
        self.previous_gyro = self._imu()[1].copy()
        self.initial_velocity = self.qvel[:, :2].copy()
        self.initial_warmup_error = self.warmup_velocity[:, :2]-self.initial_velocity
        self.integral = np.zeros((self.num_envs, 2))
        self.imu_dv_sq = np.zeros(self.num_envs)
        self.native_mismatch_sq = np.zeros((self.num_envs, 2))
        self.contact_sum = np.zeros((self.num_envs, 2))
        self.contact_count = np.zeros((self.num_envs, 2))
        return obs

    def _before_policy_step(self):
        super()._before_policy_step()
        active = ~self.finished[:, None]
        loaded = (self.contact_values[:, :, 0] > 1) & active
        self.contact_sum += loaded*self.contact_values[:, :, 1]
        self.contact_count += loaded

    def _after_physics_step(self):
        super()._after_physics_step()
        _, gyro, acc = self._imu()
        q = aligned_control_attitude(self.sensordata[:, :4], self.extrinsic)
        native = self.control.spin_observation()
        frames = self.control.frames
        positions = np.column_stack([frames[k+'_pos'] for k in ('lf', 'lr', 'rf', 'rr')])
        velocities = np.column_stack([frames[k+'_vel'] for k in ('lf', 'lr', 'rf', 'rr')])
        corrected, valid = self.geometry.observation(native, positions, velocities)
        rotation_only, _ = self.geometry.observation(native, positions, velocities, exact_position=False)
        forward, sideways, yaw = wheel_kinematics(q, gyro, native, self.axle_lateral)
        active = ~self.finished
        truth = heading_rotate(self.qvel[:, :2], -yaw)
        alpha = (gyro-self.previous_gyro)/self.dt
        self.previous_gyro[:] = gyro
        self.integral += center_acceleration(q, gyro, acc, alpha)*self.dt
        self.imu_dv_sq[active] += np.sum((self.integral-self.qvel[:, :2]+self.initial_velocity)**2, axis=1)[active]
        self.wheel_sq[active] += (forward-truth[:, :1])[active]**2
        corrected_forward, _, _ = wheel_kinematics(q, gyro, corrected, self.axle_lateral)
        self.corrected_wheel_sq[active] += (corrected_forward-truth[:, :1])[active]**2
        self.invalid_geometry[active] += (~valid)[active]
        self.sideways_sq[active] += (sideways.mean(axis=1)-truth[:, 1])[active]**2
        self.native_mismatch_sq[active] += native[active, 20:22]**2
        self.sample_count[active] += 1
        delta = self.qpos[:, :2]-self.start_xy
        measurements = {}
        if self.sensor_stress:
            # Illustrative per-sample sensor stresses, not measured hardware specs.
            noisy_gyro = gyro+self.sensor_rng.normal(0, .002, gyro.shape)
            noisy_acc = acc+self.sensor_rng.normal(0, .03, acc.shape)
            measurements['innovation_white'] = (q, noisy_gyro, noisy_acc)
            tilt = np.deg2rad(.05)/2
            measurements['innovation_residual'] = (
                multiply(q, [np.cos(tilt), np.sin(tilt), 0, 0]),
                noisy_gyro+np.array([.0005, -.0003, .0004]),
                noisy_acc+np.array([.01, -.006, .008]))
            measurements['innovation_all_required_white'] = measurements['innovation_white']
            measurements['innovation_all_required_residual'] = measurements['innovation_residual']
        for name, odom in self.shadows.items():
            measurement = rotation_only if name == 'support_spin' else corrected.copy() if (
                name.startswith(('exact_', 'innovation'))) else native
            if name == 'exact_equal':
                measurement[:, 20:22] = 0
            odom.step(*measurements.get(name, (q, gyro, acc)), measurement, self.dt)
            row = self.stats[name]
            row['peak'][active] = np.maximum(row['peak'], np.linalg.norm(odom.position-delta, axis=1))[active]
            row['velocity_sq'][active] += heading_rotate(odom.velocity-self.qvel[:, :2], -yaw)[active]**2

    def _extra_metrics(self, i):
        result = super()._extra_metrics(i)
        if self.finished[i]:
            return result
        count = self.sample_count[i]
        result['velocity_audit'] = dict(
            initial_velocity_mps=self.initial_velocity[i].tolist(),
            warmup_velocity_error_mps=self.initial_warmup_error[i].tolist(),
            wheel_forward_rmse_mps=np.sqrt(self.wheel_sq[i]/count).tolist(),
            corrected_wheel_forward_rmse_mps=np.sqrt(self.corrected_wheel_sq[i]/count).tolist(),
            invalid_geometry_fraction=(self.invalid_geometry[i]/count).tolist(),
            sideways_pseudo_rmse_mps=float(np.sqrt(self.sideways_sq[i]/count)),
            native_mismatch_rms_mps=np.sqrt(self.native_mismatch_sq[i]/count).tolist(),
            mean_loaded_contact_slip_mps=(self.contact_sum[i]/np.maximum(self.contact_count[i], 1)).tolist(),
            imu_velocity_delta_rmse_mps=float(np.sqrt(self.imu_dv_sq[i]/count)),
            estimators={name: dict(peak_error_m=float(self.stats[name]['peak'][i]),
                                   final_error_m=float(np.linalg.norm(odom.position[i]-(self.qpos[i, :2]-self.start_xy[i]))),
                                   velocity_heading_rmse_mps=np.sqrt(self.stats[name]['velocity_sq'][i]/count).tolist())
                        for name, odom in self.shadows.items()})
        self.finished[i] = True
        return result


def run(args, friction):
    cases = [(8, args.heading if args.heading is not None else np.pi/2 if loc == 'toe' else .7,
              .117-length, direction)
             for loc in args.locations for length in args.lengths
             for direction in (-1, 1)]
    terrain = [dict(location=loc, handover='lqr', offset=0, takeover_time=0, fade=.3, dropout=False)
               for loc in args.locations for _ in range(2*len(args.lengths))]
    env = VelocityAuditEnv(len(cases), 3, cases=cases, terrain_cases=terrain,
                          episode_seconds=args.seconds, reference_mode=args.reference, disturbances=False,
                          sensor_stress=args.sensor_stress, sensor_seed=args.sensor_seed,
                          compare_reference=args.compare_reference)
    try:
        # Equal-priority MuJoCo contacts take the larger friction coefficient.
        # Lowering only the floor leaves the default wheel coefficient at 1.
        surfaces = [env.model.geom(name).id for name in ('floor', 'spin_ramp', 'spin_upper_flat')]
        wheels = [env.model.geom(name).id for name in ('L_Wheel_mesh_1', 'R_Wheel_mesh_1')]
        friction_geoms = surfaces+wheels
        env.model.geom_friction[friction_geoms, 0] = friction
        env.floor_friction[:, friction_geoms, 0] = friction
        env.seed(1000)
        obs = env.reset()
        model = PPO.load(args.model, device='cpu') if args.model else None
        results = {}
        ticks, switched = 0, False
        while len(results) < len(cases):
            if args.switch_friction is not None and not switched and ticks*env.policy_dt >= args.switch_at:
                env.model.geom_friction[friction_geoms, 0] = args.switch_friction
                env.floor_friction[:, friction_geoms, 0] = args.switch_friction
                switched = True
            actions = np.zeros((len(cases), 5), np.float32) if model is None else model.predict(obs, deterministic=True)[0]
            obs, _, done, infos = env.step(actions)
            ticks += 1
            for i in np.flatnonzero(done):
                if i not in results:
                    results[i] = infos[i]['spin_metrics']
        rows = [dict(location=place['location'], nominal_length_m=.117-case[2], direction=case[3],
                     friction=friction, duration_s=results[i]['duration_s'],
                     switch_friction=args.switch_friction, switch_at_s=args.switch_at if args.switch_friction is not None else None,
                     failure_reasons=results[i]['failure_reasons'], **results[i]['velocity_audit'])
                for i, (case, place) in enumerate(zip(cases, terrain))]
        return rows, env.provenance
    finally:
        env.close()


def summarize(rows):
    return {name: dict(mean_peak_m=float(np.mean([r['estimators'][name]['peak_error_m'] for r in rows])),
                       worst_peak_m=max(r['estimators'][name]['peak_error_m'] for r in rows),
                       mean_final_m=float(np.mean([r['estimators'][name]['final_error_m'] for r in rows])))
            for name in rows[0]['estimators']}


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--frictions', type=float, nargs='+', default=[.3, 1])
    parser.add_argument('--reference', choices=('hold', 'spin'), default='spin')
    parser.add_argument('--lengths', type=float, nargs='+', default=[.047, .117, .210])
    parser.add_argument('--locations', nargs='+', default=['lower_flat', 'ramp', 'toe'],
                        choices=('lower_flat', 'ramp', 'toe', 'crest', 'upper_flat'))
    parser.add_argument('--heading', type=float, help='Heading in radians, overrides location defaults')
    parser.add_argument('--switch-friction', type=float)
    parser.add_argument('--switch-at', type=float, default=4)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--sensor-stress', action='store_true',
                        help='Add shadow-only white-noise and residual-bias IMU profiles')
    parser.add_argument('--sensor-seed', type=int, default=417)
    parser.add_argument('--compare-reference', action='store_true',
                        help='Score the prior all-measurements-required observer on the same samples')
    parser.add_argument('--out', type=Path, default=Path('logs/rl_training/spin_velocity_audit.json'))
    args = parser.parse_args()
    torch.set_num_threads(1)
    rows = []
    for friction in args.frictions:
        group, provenance = run(args, friction)
        rows.extend(group)
        print(json.dumps(dict(friction=friction, summary=summarize(group))), flush=True)
    result = dict(reference=args.reference, seconds=args.seconds, provenance=provenance,
                  compare_reference=args.compare_reference,
                  observer_source_sha256=hashlib.sha256(
                      Path(__file__).parents[1].joinpath('spin_sensor_odometry.py').read_bytes()).hexdigest(),
                  sensor_stress=dict(enabled=args.sensor_stress, seed=args.sensor_seed,
                                     accel_white_sigma_mps2=.03, gyro_white_sigma_radps=.002,
                                     residual_accel_bias_mps2=[.01, -.006, .008],
                                     residual_gyro_bias_radps=[.0005, -.0003, .0004],
                                     residual_body_roll_error_deg=.05,
                                     warmup='clean measured initialization; stress starts at anchor'),
                  audit_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest() if args.model else None,
                  summary=summarize(rows), results=rows)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()

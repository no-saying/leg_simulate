"""Compare legacy and aligned IMU frames on identical physical trajectories.

Simulator velocity is used only to score estimates, never as estimator input.
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
from spin_imu import aligned_control_attitude, imu_attitude_extrinsic
from spin_rl_env import attitude
from stable_baselines3 import PPO


def center_acceleration(q, gyro, acc, alpha):
    offset = np.array([0, -0.020, 0])
    center = acc-np.cross(alpha, offset)-np.cross(gyro, np.cross(gyro, offset))
    roll, pitch, yaw, _, _ = attitude(q, gyro)
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    horizontal = np.column_stack((cp*center[:, 0]+sp*(sr*center[:, 1]+cr*center[:, 2]),
                                  cr*center[:, 1]-sr*center[:, 2]))
    return heading_rotate(horizontal, yaw)


class AuditEnv(AnchorSpinEnv):
    def reset(self):
        obs = super().reset()
        self.audit = {}
        self.audit_finished = np.zeros(self.num_envs, bool)
        self.audit_start = self.qvel[:, :2].copy()
        self.audit_previous_gyro = self._imu()[1].copy()
        self.audit_extrinsic = imu_attitude_extrinsic(self.model)
        for name in ("legacy", "aligned", "aligned_geometry"):
            lateral = self.axle_lateral if name == 'aligned_geometry' else (-.1361375, .1361375)
            self.audit[name] = dict(odom=SpinOdometry(self.num_envs, lateral=lateral),
                                    integral=np.zeros((self.num_envs, 2)),
                                    peak=np.zeros(self.num_envs),
                                    dv_sq=np.zeros(self.num_envs), samples=np.zeros(self.num_envs))
        return obs

    def _after_physics_step(self):
        super()._after_physics_step()
        q, gyro, acc = self._imu()
        aligned = aligned_control_attitude(self.sensordata[:, :4], self.audit_extrinsic)
        alpha = (gyro-self.audit_previous_gyro)/self.dt
        self.audit_previous_gyro[:] = gyro
        native = self.control.spin_observation()
        delta = self.qpos[:, :2]-self.start_xy
        active = ~self.audit_finished
        for name, orientation in (("legacy", q), ("aligned", aligned), ('aligned_geometry', aligned)):
            row = self.audit[name]
            row['odom'].step(orientation, gyro, acc, native, self.dt)
            row['integral'][active] += center_acceleration(orientation, gyro, acc, alpha)[active]*self.dt
            dv_error = row['integral']-(self.qvel[:, :2]-self.audit_start)
            row['dv_sq'][active] += np.sum(dv_error**2, axis=1)[active]
            row['peak'][active] = np.maximum(row['peak'], np.linalg.norm(row['odom'].position-delta, axis=1))[active]
            row['samples'][active] += 1

    def audit_row(self, i):
        return {name: dict(imu_integral_velocity_rmse_mps=float(np.sqrt(v['dv_sq'][i]/v['samples'][i])),
                           odometry_peak_error_m=float(v['peak'][i]),
                           final_odometry_error_m=float(np.linalg.norm(
                               v['odom'].position[i]-(self.qpos[i, :2]-self.start_xy[i]))))
                for name, v in self.audit.items()}

    def _extra_metrics(self, i):
        result = super()._extra_metrics(i)
        if not self.audit_finished[i]:
            result['odometry_audit'] = self.audit_row(i)
            self.audit_finished[i] = True
        return result


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--out', type=Path, default=Path('logs/rl_training/imu_frame_audit.json'))
    args = parser.parse_args()
    torch.set_num_threads(1)
    cases = [(8, np.pi/2 if loc == 'toe' else .7, ext, 1)
             for loc in ('lower_flat', 'ramp', 'toe') for ext in (.070, 0, -.093)]
    terrain = [dict(location=loc, handover='lqr', offset=0, takeover_time=0, fade=.3, dropout=False)
               for loc in ('lower_flat', 'ramp', 'toe') for _ in range(3)]
    env = AuditEnv(len(cases), 3, cases=cases, terrain_cases=terrain,
                   episode_seconds=args.seconds, disturbances=False)
    try:
        env.seed(1000)
        obs = env.reset()
        model = PPO.load(args.model, device='cpu') if args.model else None
        results = {}
        while len(results) < len(cases):
            action = np.zeros((len(cases), 5), np.float32) if model is None else model.predict(
                obs, deterministic=True)[0]
            obs, _, done, infos = env.step(action)
            for i in np.flatnonzero(done):
                if i not in results:
                    results[i] = infos[i]['spin_metrics']
        rows = []
        for i, (case, place) in enumerate(zip(cases, terrain)):
            row = dict(location=place['location'], nominal_length_m=.117-case[2],
                       duration_s=results[i]['duration_s'], failure_reasons=results[i]['failure_reasons'],
                       **results[i]['odometry_audit'])
            rows.append(row)
        result = dict(seconds=args.seconds, model=str(args.model),
                      model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest() if args.model else None,
                      audit_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      provenance=env.provenance, results=rows)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(rows, indent=2))
    finally:
        env.close()


if __name__ == '__main__':
    main()

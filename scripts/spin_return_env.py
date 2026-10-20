"""Opt-in return curriculum; deployment observations and strict evaluation stay V7."""

import hashlib
from pathlib import Path

import numpy as np
from spin_anchor_env import ANCHOR_RADIUS, ESCAPE_RADIUS, RETURN_RADIUS, heading_rotate
from spin_asymmetric_env import AsymmetricSpinEnv
from spin_rl_env import attitude, spin_reference


RETURN_STAGES = (
    dict(name='return_direction', yaw_scale=.45, deadline=15., tolerance=.20,
         slope=3., offset_probability=.75, offset_range=(.20, .38)),
    dict(name='return_while_spinning', yaw_scale=.75, deadline=10., tolerance=.13,
         slope=5., offset_probability=.60, offset_range=(.20, .40)),
    dict(name='bounded_spin', yaw_scale=1., deadline=6., tolerance=.08,
         slope=8., offset_probability=.35, offset_range=(.20, .40)),
)


def return_velocity_reference(displacement, speed_limit=.25, gain=1.0):
    displacement = np.asarray(displacement)
    radius = np.linalg.norm(displacement, axis=1)
    speed = speed_limit*np.tanh(gain*radius/speed_limit)
    return -displacement*(speed/np.maximum(radius, 1e-9))[:, None]


def return_tracking_cost(displacement, velocity, tolerance):
    error = (velocity-return_velocity_reference(displacement))/np.asarray(tolerance)[..., None]
    # Huber tails retain a learning signal outside the initial comfort region.
    absolute = np.abs(error)
    return -2*np.sum(np.where(absolute <= 1, .5*error**2, absolute-.5), axis=1)


def return_gate_passes(stats):
    attempts = stats.get('return_attempts', 0)
    return (attempts >= max(6, stats['cases']//2)
            and stats.get('return_successes', 0)/max(attempts, 1) >= .70
            and stats['completed']/stats['cases'] >= .70
            and stats['mean_yaw_angle_ratio'] >= .80
            and stats['mean_roll_rms_deg'] <= 3.
            and all(band['completed']/band['cases'] >= .50
                    and band['mean_yaw_angle_ratio'] >= .70
                    for band in stats.get('by_leg_band', {}).values()))


class ReturnCurriculumMixin:
    """Known relative target commands create return trials without moving the robot."""

    def __init__(self, *args, return_stage=0, gate_trials=False, **kwargs):
        if return_stage not in range(len(RETURN_STAGES)):
            raise ValueError('Return stage must be 0, 1 or 2')
        super().__init__(*args, **kwargs)
        self.return_stage = return_stage
        self.return_max_slope = self.max_slope_deg
        self.gate_trials = gate_trials
        self.stage_per_episode = np.zeros(self.num_envs, dtype=int)
        self.unscaled_yaw = np.zeros(self.num_envs)
        self.target_offset = np.zeros((self.num_envs, 2))
        self.reward_totals['return_velocity'] = np.zeros(self.num_envs)
        self.provenance.update(
            task='slope_spin_return_curriculum_v8', reward_version='return_velocity_huber_v1',
            return_stages=list(RETURN_STAGES), return_stage=return_stage,
            stage_scope='new episodes only; 20 percent earlier stages retained',
            return_target='known relative command; same offset initializes sensor odometry',
            return_velocity_limit_mps=.25, return_velocity_gain_per_s=1.,
            strict_acceptance='unchanged AsymmetricSpinEnv, no target offset, full yaw, 6s deadline',
            return_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        self.set_return_stage(return_stage)

    def set_return_stage(self, stage):
        if stage not in range(len(RETURN_STAGES)):
            raise ValueError('Return stage must be 0, 1 or 2')
        self.return_stage = stage
        self.max_slope_deg = min(self.return_max_slope, RETURN_STAGES[stage]['slope'])
        self.provenance['return_stage'] = stage

    def _reset_context(self, ids):
        super()._reset_context(ids)
        _, _, yaw, _, _ = attitude(*self._imu()[:2])
        for i in ids:
            rng = self.rng[i]
            stage = (int(rng.integers(self.return_stage)) if self.return_stage > 0
                     and not self.gate_trials and rng.random() < .20 else self.return_stage)
            self.stage_per_episode[i] = stage
            settings = RETURN_STAGES[stage]
            self.unscaled_yaw[i] = 0
            offset = np.zeros((1, 2))
            if self.gate_trials:
                offset[0, 0] = .35 if i % 2 == 0 else -.35
            elif rng.random() < settings['offset_probability']:
                distance = rng.uniform(*settings['offset_range'])
                angle = rng.uniform(-np.pi, np.pi)
                offset[0] = distance*np.array([np.cos(angle), np.sin(angle)])
            self.target_offset[i] = heading_rotate(offset, yaw[i:i+1])[0]
        # A commanded relative target is known on hardware. No true displacement
        # measurement or simulator velocity is copied into the actor's estimator.
        self.start_xy[ids] += self.target_offset[ids]
        self.odometry.position[ids] -= self.target_offset[ids]

    def _spin_reference(self, elapsed):
        reference, phase = spin_reference(
            elapsed, self.episode_steps*self.policy_dt, self.ext, self.direction,
            self.unscaled_yaw, self.dt, self.recovery_seconds)
        self.unscaled_yaw[:] = reference
        scales = np.array([s['yaw_scale'] for s in RETURN_STAGES])[self.stage_per_episode]
        return reference*scales, phase

    def _reward(self, *args):
        _, terms = super()._reward(*args)
        displacement = self.qpos[:, :2]-self.start_xy
        tolerance = np.array([s['tolerance'] for s in RETURN_STAGES])[self.stage_per_episode]
        terms['return_velocity'] = return_tracking_cost(displacement, self.qvel[:, :2], tolerance)
        # The velocity reference already brakes at the target; avoid charging
        # intended inward motion a second time through a total-speed penalty.
        terms['drift'] = np.zeros(self.num_envs)
        return 1+sum(terms.values()), terms

    def _failure_reasons(self, finite, roll, pitch, drift):
        reasons = super()._failure_reasons(finite, roll, pitch, drift) & ~8
        deadline = np.array([s['deadline'] for s in RETURN_STAGES])[self.stage_per_episode]
        return reasons | (8*((drift > ESCAPE_RADIUS) | (self.excursion_age > deadline)))

    def _extra_metrics(self, i):
        return {**super()._extra_metrics(i),
                'return_training_stage': int(self.stage_per_episode[i]),
                'initial_target_offset_m': float(np.linalg.norm(self.target_offset[i])),
                'training_return_deadline_s': RETURN_STAGES[self.stage_per_episode[i]]['deadline'],
                'strict_reference_radius_m': ANCHOR_RADIUS,
                'strict_return_radius_m': RETURN_RADIUS}


class ReturnTrainingSpinEnv(ReturnCurriculumMixin, AsymmetricSpinEnv):
    """Residual-control compatibility path for the archived curriculum experiment."""

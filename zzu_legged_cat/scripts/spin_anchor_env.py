"""V6: bounded spin about a fixed start, with sensor-derived return direction."""

import hashlib
from pathlib import Path

import gymnasium as gym
import numpy as np
from spin_rl_env import HISTORY, attitude
from spin_standard_env import StandardSpinEnv
from spin_travel_env import OBSERVATION_SCALE

TASK_VERSION = "slope_spin_anchor_v6"
FRAME_SIZE = 56
ANCHOR_RADIUS = 0.30
RETURN_RADIUS = 0.15
ESCAPE_RADIUS = 1.0
RETURN_DEADLINE = 6.0
OBSERVATION_SCALE = [*OBSERVATION_SCALE, ANCHOR_RADIUS, ANCHOR_RADIUS, 0.5, 0.5]


def heading_rotate(vectors, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack((c*vectors[:, 0]-s*vectors[:, 1],
                            s*vectors[:, 0]+c*vectors[:, 1]))


class SpinOdometry:
    """Planar wheel/IMU fusion; no simulator position, velocity or contact input."""

    def __init__(self, count, lateral=(-0.1361375, 0.1361375)):
        self.lateral = np.asarray(lateral, dtype=float)
        self.position = np.zeros((count, 2))
        self.velocity = np.zeros((count, 2))
        self.previous_gyro = np.zeros((count, 3))
        self.ready = np.zeros(count, bool)

    def reset(self, ids):
        self.position[ids] = self.velocity[ids] = self.previous_gyro[ids] = 0
        self.ready[ids] = False

    def step(self, q, gyro, acc, native, dt):
        roll, pitch, yaw, _, _ = attitude(q, gyro)
        cp, sp, cr, sr = np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
        alpha = np.where(self.ready[:, None], (gyro-self.previous_gyro)/dt, 0)
        self.previous_gyro[:] = gyro
        # Same CAD mounting and off-center IMU compensation as the native observer.
        gx, gy, gz = gyro.T
        offset_y = -0.020
        center_acc = acc.copy()
        center_acc[:, 0] -= offset_y*(gy*gx-alpha[:, 2])
        center_acc[:, 1] += offset_y*(gz*gz+gx*gx)
        center_acc[:, 2] -= offset_y*(gy*gz+alpha[:, 0])
        horizontal_acc = np.column_stack((
            cp*center_acc[:, 0]+sp*(sr*center_acc[:, 1]+cr*center_acc[:, 2]),
            cr*center_acc[:, 1]-sr*center_acc[:, 2]))
        lengths, swings = native[:, 11:13], native[:, 15:17]
        length_rates, swing_rates = native[:, 13:15], native[:, 17:19]
        lateral = self.lateral
        rotation = (cp[:, None]*(-gy[:, None]*lengths-gz[:, None]*lateral)
                    + (sp*sr)[:, None]*(gz[:, None]*swings+gx[:, None]*lengths)
                    + (sp*cr)[:, None]*(gx[:, None]*lateral-gy[:, None]*swings))
        forward = (-0.043*(native[:, 9:11]-gy[:, None])-cp[:, None]*swing_rates
                   + (sp*cr)[:, None]*length_rates-rotation)
        weights = 1/(1+(native[:, 20:22]/0.12)**2)
        vx = np.sum(weights*forward, axis=1)/np.maximum(weights.sum(axis=1), 1e-6)
        vy = -np.mean(cr[:, None]*(gz[:, None]*swings+gx[:, None]*lengths)
                      + sr[:, None]*(gy[:, None]*swings+length_rates), axis=1)
        wheel_velocity = heading_rotate(np.column_stack((vx, vy)), yaw)
        old_velocity = self.velocity.copy()
        predicted = self.velocity+heading_rotate(horizontal_acc, yaw)*dt
        trust = np.clip(weights.mean(axis=1), 0.05, 1)
        # Long inertial integration magnifies small acceleration biases during
        # spin. Reconcile frequently with yaw/leg-compensated wheel kinematics.
        gain = dt/(0.015/np.maximum(trust, 0.3)+dt)
        self.velocity[:] = predicted+gain[:, None]*(wheel_velocity-predicted)
        self.velocity[~self.ready] = wheel_velocity[~self.ready]
        self.velocity[:] = np.clip(self.velocity, -5, 5)
        self.position += 0.5*(old_velocity+self.velocity)*dt
        self.ready[:] = True

    def features(self, yaw):
        return np.column_stack((heading_rotate(self.position, -yaw)/ANCHOR_RADIUS,
                                heading_rotate(self.velocity, -yaw)/0.5))


def anchor_reward_terms(displacement, velocity, excursion_age):
    radius = np.linalg.norm(displacement, axis=1)
    radial_speed = np.sum(displacement*velocity, axis=1)/np.maximum(radius, 1e-6)
    outside = np.maximum(radius-ANCHOR_RADIUS, 0)/ANCHOR_RADIUS
    # Every tick charges the current radius. Stopping far away remains expensive.
    # The signed progress term is a potential difference, so repeated in/out
    # motion cannot collect return bonuses without paying for outward travel.
    return {
        "anchor_position": -4*np.minimum((radius/ANCHOR_RADIUS)**2, 16),
        "anchor_boundary": -12*np.minimum(outside**2, 16),
        "anchor_outward": -4*(radius > RETURN_RADIUS)*np.minimum(
            (np.maximum(radial_speed, 0)/0.20)**2, 9),
        "anchor_progress": -4*np.minimum(radius/ANCHOR_RADIUS, 4)*radial_speed/ANCHOR_RADIUS,
        "anchor_timeout": -4*(radius > ANCHOR_RADIUS)*np.minimum(excursion_age/RETURN_DEADLINE, 2),
    }


class AnchorSpinEnv(StandardSpinEnv):
    failure_cost_per_second = 500
    training_reward_scale = 0.05

    def __init__(self, *args, disturbances=True, **kwargs):
        kwargs.setdefault("episode_seconds", 30)
        kwargs.setdefault("recovery_seconds", 0)
        super().__init__(*args, **kwargs)
        self.odometry = SpinOdometry(self.num_envs)
        self.disturbances = disturbances
        self.push_time = np.zeros(self.num_envs)
        self.push_velocity = np.zeros((self.num_envs, 2))
        self.push_applied = np.zeros(self.num_envs, bool)
        self.excursion_active = np.zeros(self.num_envs, bool)
        self.excursion_age = np.zeros(self.num_envs)
        self.return_dwell = np.zeros(self.num_envs)
        self.return_attempts = np.zeros(self.num_envs, int)
        self.return_successes = np.zeros(self.num_envs, int)
        self.return_time_total = np.zeros(self.num_envs)
        self.return_time_peak = np.zeros(self.num_envs)
        self.outside_seconds = np.zeros(self.num_envs)
        self.radius_integral = np.zeros(self.num_envs)
        self.odom_error_peak = np.zeros(self.num_envs)
        self.final_radius = np.zeros(self.num_envs)
        self.tail_radius_peak = np.zeros(self.num_envs)
        self.history = np.zeros((self.num_envs, HISTORY, FRAME_SIZE), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (FRAME_SIZE*HISTORY,), np.float32)
        for name in anchor_reward_terms(np.zeros((1, 2)), np.zeros((1, 2)), np.zeros(1)):
            self.reward_totals[name] = np.zeros(self.num_envs)
        self.provenance.update(
            task=TASK_VERSION, reward_version="anchor_v6", frame_size=FRAME_SIZE,
            anchor_radius_m=ANCHOR_RADIUS, return_radius_m=RETURN_RADIUS,
            escape_radius_m=ESCAPE_RADIUS, return_deadline_s=RETURN_DEADLINE,
            return_dwell_s=0.5, actor_position_source="wheel/IMU/leg-kinematic odometry",
            actor_world_truth_inputs=False, disturbances=disturbances,
            position_cost_at_30cm_per_s=4, position_cost_at_60cm_per_s=28,
            remaining_failure_cost_per_s=self.failure_cost_per_second,
            training_reward_scale=self.training_reward_scale,
            reward_source_sha256=hashlib.sha256(b"".join(
                Path(__file__).with_name(name).read_bytes() for name in (
                    "spin_anchor_env.py", "spin_standard_env.py", "spin_travel_env.py",
                    "spin_mixed_env.py", "spin_rl_env.py"))).hexdigest())

    def _reset_context(self, ids):
        super()._reset_context(ids)
        self.odometry.reset(ids)
        for values in (self.push_applied, self.excursion_active, self.excursion_age,
                       self.return_dwell, self.return_attempts, self.return_successes,
                       self.return_time_total, self.return_time_peak, self.outside_seconds,
                       self.radius_integral, self.odom_error_peak, self.final_radius,
                       self.tail_radius_peak):
            values[ids] = 0
        for i in ids:
            r = self.rng[i]
            self.push_time[i] = r.uniform(7, 12) if self.disturbances and r.random() < 0.5 else np.inf
            angle, speed = r.uniform(-np.pi, np.pi), r.uniform(0.10, 0.25)
            self.push_velocity[i] = speed*np.array([np.cos(angle), np.sin(angle)])

    def _frame_obs(self):
        _, _, yaw, _, _ = attitude(*self._imu()[:2])
        return np.column_stack((super()._frame_obs(),
                                np.clip(self.odometry.features(yaw), -20, 20))).astype(np.float32)

    def _before_policy_step(self):
        super()._before_policy_step()
        push = (~self.push_applied) & (self.steps*self.policy_dt >= self.push_time)
        if push.any():
            self._apply_push(push)
            self.push_applied[push] = True

    def _apply_push(self, push):
        # Legacy V6 impulse retained for checkpoint comparisons. Sensor-based
        # tasks override this with forces that appear in the IMU measurements.
        self.qvel[push, :2] += self.push_velocity[push]
        self.batch.forward(ids=np.flatnonzero(push).astype(np.int32))

    def _after_physics_step(self):
        self.odometry.step(*self._imu(), self.control.spin_observation(), self.dt)

    def _reward(self, *args):
        _, terms = super()._reward(*args)
        delta = self.qpos[:, :2]-self.start_xy
        radius = np.linalg.norm(delta, axis=1)
        outside = radius > ANCHOR_RADIUS
        first = outside & ~self.excursion_active
        self.return_attempts[first] += 1
        self.excursion_active |= first
        self.excursion_age += self.excursion_active*self.dt
        self.return_dwell[:] = np.where(
            self.excursion_active & (radius <= RETURN_RADIUS), self.return_dwell+self.dt, 0)
        returned = self.return_dwell >= 0.5
        self.return_successes[returned] += 1
        self.return_time_total[returned] += self.excursion_age[returned]
        self.return_time_peak[returned] = np.maximum(
            self.return_time_peak[returned], self.excursion_age[returned])
        self.excursion_active[returned] = False
        self.excursion_age[returned] = self.return_dwell[returned] = 0
        self.outside_seconds += outside*self.dt
        self.radius_integral += radius*self.dt
        self.final_radius[:] = radius
        tail = (self.steps*self.policy_dt >= self.episode_steps*self.policy_dt-5)
        self.tail_radius_peak[:] = np.maximum(self.tail_radius_peak, radius*tail)
        error = np.linalg.norm(self.odometry.position-delta, axis=1)
        self.odom_error_peak[:] = np.maximum(self.odom_error_peak, error)
        # Replace V5's capped position/speed cost and peak charge with persistent
        # bounded-area costs. Do not penalize inward return speed as drift.
        terms["drift"] = -0.25*np.minimum((args[5]/0.4)**2, 9)
        terms["drift_peak"] = np.zeros(self.num_envs)
        terms.update(anchor_reward_terms(delta, self.qvel[:, :2], self.excursion_age))
        return 1+sum(terms.values()), terms

    def _failure_reasons(self, finite, roll, pitch, drift):
        reasons = super()._failure_reasons(finite, roll, pitch, drift)
        escaped = (drift > ESCAPE_RADIUS) | (self.excursion_age > RETURN_DEADLINE)
        return reasons | (8*escaped)

    def _extra_metrics(self, i):
        duration = max(self.steps[i]*self.policy_dt, self.dt)
        return {**super()._extra_metrics(i),
                "anchor_radius_m": ANCHOR_RADIUS,
                "final_drift_m": float(self.final_radius[i]),
                "mean_drift_m": float(self.radius_integral[i]/duration),
                "outside_seconds": float(self.outside_seconds[i]),
                "inside_fraction": float(1-self.outside_seconds[i]/duration),
                "return_attempts": int(self.return_attempts[i]),
                "return_successes": int(self.return_successes[i]),
                "unrecovered": bool(self.excursion_active[i]),
                "max_return_seconds": float(self.return_time_peak[i]),
                "mean_return_seconds": float(self.return_time_total[i]/max(self.return_successes[i], 1)),
                "tail_max_drift_m": float(self.tail_radius_peak[i]),
                "odometry_peak_error_m": float(self.odom_error_peak[i]),
                "push_applied": bool(self.push_applied[i]),
                "bounded_spin_pass": bool(
                    not self.failure_mask[i] and self.stall_seconds[i] <= 2
                    and not self.excursion_active[i] and self.final_radius[i] <= ANCHOR_RADIUS
                    and self.tail_radius_peak[i] <= ANCHOR_RADIUS
                    and self.outside_seconds[i]/duration <= 0.1
                    and self.yaw_integral[i]/self.yaw_reference_integral[i] >= 0.8
                    if abs(self.yaw_reference_integral[i]) > 1e-6 else False)}

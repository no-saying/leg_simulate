"""Exclusive six-motor spin control with encoder-based linkage allocation."""

from pathlib import Path

import gymnasium as gym
import numpy as np
from spin_asymmetric_env import AsymmetricSpinEnv, PRIVILEGED_SIZE
from spin_mixed_env import MixedTerrainSpinEnv
from spin_rl_env import LEG_MID, SpinRLBatchEnv, attitude, euler_rates
from spin_imu import aligned_control_attitude
from spin_return_env import ReturnCurriculumMixin
from spin_wrench_control import ACTION_NAMES, ACTION_SCALE, TORQUE_LIMIT, WrenchTorqueMapper
from wheel_legged.core.frames import projected_up
from wheel_legged.core.observations import compact_frame, FRAME_SIZE, OBS_SCALE
from wheel_legged.core.rewards import pitch_balance_reward
from wheel_legged.provenance import implementation_digest

TASK_VERSION = 'slope_spin_wrench_compact_v9'
HISTORY = 6  # Five previous frames plus the current frame.
HISTORY_LATENT_SIZE = 3
HISTORY_HIDDEN_SIZE = 128
ACTOR_SIZE = FRAME_SIZE*HISTORY
FRAME_NAMES = (
    'gyro_x', 'gyro_y', 'gyro_z', 'gravity_x', 'gravity_y', 'gravity_z',
    'command_forward_speed', 'command_yaw_rate', 'command_leg_length',
    'left_front_angle', 'left_rear_angle', 'right_front_angle', 'right_rear_angle',
    'left_front_rate', 'left_rear_rate', 'right_front_rate', 'right_rear_rate',
    'left_wheel_rate', 'right_wheel_rate',
    *('previous_'+name for name in ACTION_NAMES),
)


class WrenchSpinEnv(AsymmetricSpinEnv):
    def __init__(self, *args, torque_loop='ideal', reward_profile='spin', **kwargs):
        if torque_loop != 'ideal':
            raise ValueError('Exclusive control uses ideal motors; torque PI is disabled')
        if reward_profile not in ('spin', 'pitch'):
            raise ValueError('Expected spin or pitch reward profile')
        self.reward_profile = reward_profile
        if reward_profile == 'pitch':
            kwargs.setdefault('reference_mode', 'hold')
            if kwargs['reference_mode'] != 'hold':
                raise ValueError('Pitch pretraining requires zero yaw reference (hold)')
            kwargs.setdefault('max_slope_deg', 0)
            kwargs.setdefault('fixed_heading', 0)
            kwargs.setdefault('leg_ext_range', (-.010, .010))
            kwargs.setdefault('episode_seconds', 12)
            kwargs['disturbances'] = False
            self.failure_cost_per_second = 0
            self.training_reward_scale = 1.
        super().__init__(*args, **kwargs)
        self.mapper = WrenchTorqueMapper(self.model)
        self.torque_loop = torque_loop
        self.direct_action = np.zeros((self.num_envs, 6))
        self.last_action = np.zeros((self.num_envs, 6), np.float32)
        self.forward_target = np.zeros(self.num_envs)
        self.applied_torque = np.zeros((self.num_envs, 6))
        self.native_torque = np.zeros((self.num_envs, 6))
        self.jacobian_valid = np.ones((self.num_envs, 2), bool)
        self.virtual_body_wrench = np.zeros((self.num_envs, 6))
        self.full_control_seconds = np.zeros(self.num_envs)
        self.native_leak_peak = np.zeros(self.num_envs)
        self.rl_owner = np.zeros(self.num_envs, bool)
        self.owner_switches = np.zeros(self.num_envs, int)
        self.native_update_seconds = np.zeros(self.num_envs)
        self.rl_pitch_sq = np.zeros(self.num_envs)
        self.rl_pitch_peak = np.zeros(self.num_envs)
        self.rl_pitch_seconds = np.zeros(self.num_envs)
        self.native_previous_output = np.zeros((self.num_envs, 6))
        self.output_rate_limit = np.array([300., 300., 40., 300., 300., 40.])
        self.history = np.zeros((self.num_envs, HISTORY, FRAME_SIZE), np.float32)
        self.action_space = gym.spaces.Box(-1, 1, (6,), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (ACTOR_SIZE+PRIVILEGED_SIZE,), np.float32)
        self.provenance.update(
            task=TASK_VERSION, residual_control=False, frame_size=FRAME_SIZE,
            actor_size=ACTOR_SIZE, critic_size=ACTOR_SIZE+PRIVILEGED_SIZE,
            action_names=list(ACTION_NAMES), action_scale=ACTION_SCALE.tolist(),
            action_units=['N']*4+['Nm']*2, torque_limits_nm=TORQUE_LIMIT.tolist(),
            torque_control='binary ownership of all six motors; no weighted mixing',
            native_during_takeover='not stepped while RL owns outputs; sensor estimation continues independently',
            handover_smoothing='physical torque slew limit; no concurrent controller outputs',
            torque_slew_limit_nm_per_s=self.output_rate_limit.tolist(),
            body_frame='x forward, y left, z up',
            history_frames=HISTORY, history_seconds=HISTORY*self.policy_dt,
            history_past_frames=HISTORY-1, history_lookback_seconds=(HISTORY-1)*self.policy_dt,
            history_latent_size=HISTORY_LATENT_SIZE,
            history_hidden_size=HISTORY_HIDDEN_SIZE,
            actor_architecture='past 5 x 25 -> FC128 -> FC3; current 25 + latent 3 -> FC256 -> FC256 -> 6',
            actor_frame_features=list(FRAME_NAMES),
            attitude_representation='projected gravity in calibrated body frame; no absolute yaw',
            command_features=['forward_speed_mps', 'yaw_rate_radps', 'leg_length_m'],
            joint_rate_order=['left_front', 'left_rear', 'right_front', 'right_rear',
                              'left_wheel', 'right_wheel'],
            torque_inner_loop=torque_loop,
            torque_motor_assumptions=None,
            linkage_mapping='exact encoder geometry; tau=-J_axle.T F_body+J_support.T tau_wheel',
            actor_native_control_inputs=False,
            control_source_sha256=implementation_digest(__file__,
                Path(__file__).with_name('spin_wrench_control.py')))
        if reward_profile == 'pitch':
            self.reward_totals = {name: np.zeros(self.num_envs) for name in (
                'survival', 'terminal', 'pitch_upright', 'pitch_rate', 'roll_guard',
                'height_guard', 'effort', 'smooth')}
            self.provenance.update(
                task='exclusive_pitch_balance_compact_v2', reward_version='pitch_balance_v1',
                reward_profile='pitch', pitch_kernel_width_deg=10,
                pitch_reward_peak_per_s=6, pitch_rate_cost_max_per_s=.5,
                roll_guard_cost_max_per_s=.15, height_guard_cost_max_per_s=.3,
                remaining_failure_cost_per_s=0, terminal_failure_cost=10,
                training_reward_scale=1., reward_native_intervals=False,
                pitch_metrics_scope='RL ownership only',
                reward_source_sha256=implementation_digest(__file__))

    def _case(self, i):
        if self.reward_profile == 'pitch' and self.terrain_cases is None:
            return dict(location='lower_flat', handover='pid', takeover_time=0,
                        fade=.3, dropout=False)
        return super()._case(i)

    def _reward(self, *args):
        if self.reward_profile != 'pitch':
            return super()._reward(*args)
        roll, pitch = args[:2]
        pitch_rate = euler_rates(roll, pitch, self._imu()[1])[:, 1]
        reward, terms = pitch_balance_reward(pitch, pitch_rate, roll, args[9], args[7], args[8])
        active = self.rl_owner.astype(float)
        self.rl_pitch_sq += active*pitch**2*self.dt
        self.rl_pitch_peak[:] = np.maximum(self.rl_pitch_peak, active*np.abs(pitch))
        self.rl_pitch_seconds += active*self.dt
        # Base logging adds one survival unit per second unconditionally.
        self.reward_totals['survival'] -= (1-active)*self.dt
        return reward*active, {name: value*active for name, value in terms.items()}

    def _failure_reasons(self, finite, roll, pitch, drift):
        if self.reward_profile == 'pitch':
            return SpinRLBatchEnv._failure_reasons(self, finite, roll, pitch, drift)
        return super()._failure_reasons(finite, roll, pitch, drift)

    def _reset_context(self, ids):
        super()._reset_context(ids)
        for value in (self.direct_action, self.applied_torque, self.native_torque,
                      self.virtual_body_wrench, self.full_control_seconds, self.native_leak_peak,
                      self.rl_owner, self.owner_switches, self.native_update_seconds,
                      self.rl_pitch_sq, self.rl_pitch_peak, self.rl_pitch_seconds,
                      self.forward_target):
            value[ids] = 0
        self.jacobian_valid[ids] = True
        measured = self.actuator_force[ids]*self.model.actuator_gear[:, 0]
        self.native_previous_output[ids] = np.clip(measured, -TORQUE_LIMIT, TORQUE_LIMIT)

    def _prepare_control(self, elapsed, actions, mask):
        self.direct_action[:] = actions
        desired = elapsed >= self.takeover_time
        dropout = ((elapsed >= self.dropout_time)
                   & (elapsed < self.dropout_time+self.dropout_length))
        desired &= ~dropout
        exits = self.rl_owner & ~desired & mask
        if exits.any():
            # Re-enter native control from fresh estimator/integrator state,
            # rather than resuming integrators frozen before the RL interval.
            self.control.reset(exits)
            self.control.set_active(exits, True)
        self.owner_switches += mask & (desired != self.rl_owner)
        self.rl_owner[mask] = desired[mask]
        MixedTerrainSpinEnv._prepare_control(self, elapsed, np.zeros((self.num_envs, 5)), mask)
        self.authority[:] = self.rl_owner.astype(float)

    def _apply_residual(self, actions, mask):
        self.inputs[:, 0] = self.ext
        self.inputs[:, 2:] = 0
        native = mask & ~self.rl_owner
        if native.any():
            self.control.spin_command(self.inputs, native)

    def _actuator_control(self, mask):
        native_mask = mask & ~self.rl_owner
        native = np.zeros((self.num_envs, 6))
        if native_mask.any():
            native[native_mask] = self.control.step(native_mask, self.dt)[native_mask]
        self.native_update_seconds += native_mask*self.dt
        positions = self.qpos[:, self.qa[:4]]
        direct, torque, (wheel, force), valid = self.mapper.map(positions, self.direct_action*ACTION_SCALE)
        self.native_torque[:] = native*self.mapper.gain
        self.jacobian_valid[:] = valid
        self.virtual_body_wrench[:] = self.mapper.point_force_wrench(wheel, force, self.axle_lateral)
        full = self.rl_owner
        output = np.where(full[:, None], direct, native)
        self.full_control_seconds += full*self.dt
        self.native_leak_peak[full] = np.maximum(self.native_leak_peak[full],
            np.max(np.abs(output[full]-direct[full]), axis=1))
        requested = np.clip(output*self.mapper.gain, -TORQUE_LIMIT, TORQUE_LIMIT)
        self.applied_torque[:] = self.native_previous_output+np.clip(
            requested-self.native_previous_output,
            -self.output_rate_limit*self.dt, self.output_rate_limit*self.dt)
        self.native_previous_output[:] = self.applied_torque
        return self.applied_torque/self.mapper.gain

    def _kinematic_observation(self):
        raw = np.zeros((self.num_envs, 28), np.float32)
        raw[:, 9:11] = self.qvel[:, self.va[4:6]]
        raw[:, 26] = self.yaw_target
        # Mode is diagnostic only. The actor frame below never consumes it.
        raw[:, 27] = self.control.spin_observation()[:, 27]
        return self.geometry.observation(raw, self.qpos[:, self.qa[:4]], self.qvel[:, self.va[:4]])

    def _native_observation(self):
        return self._kinematic_observation()[0]

    def _frame_obs(self):
        _, gyro, _ = self._imu()
        q = aligned_control_attitude(self.sensordata[:, :4], self.imu_extrinsic)
        commands = np.column_stack((self.forward_target, self.yaw_target, LEG_MID-self.ext))
        return compact_frame(gyro, projected_up(q), commands,
                             self.qpos[:, self.qa[:4]], self.qvel[:, self.va], self.last_action)

    def _extra_metrics(self, i):
        metrics = super()._extra_metrics(i)
        if self.reward_profile == 'pitch':
            # Return-task accumulators are inactive in pitch pretraining.
            inactive = {'anchor_radius_m', 'final_drift_m', 'mean_drift_m',
                        'outside_seconds', 'inside_fraction', 'return_attempts',
                        'return_successes', 'unrecovered', 'max_return_seconds',
                        'mean_return_seconds', 'tail_max_drift_m', 'odometry_peak_error_m',
                        'bounded_spin_pass'}
            metrics = {key: value for key, value in metrics.items()
                       if key not in inactive and not key.startswith('standard_')}
            metrics['final_drift_m'] = float(np.linalg.norm(self.qpos[i, :2]-self.start_xy[i]))
        return {**metrics,
                'full_rl_control_seconds': float(self.full_control_seconds[i]),
                'native_torque_leak_peak_ctrl': float(self.native_leak_peak[i]),
                'rl_owns_all_motors': bool(self.rl_owner[i]),
                'owner_switches': int(self.owner_switches[i]),
                'native_update_seconds': float(self.native_update_seconds[i]),
                **({'rl_pitch_rms_deg': float(np.rad2deg(np.sqrt(
                        self.rl_pitch_sq[i]/max(self.rl_pitch_seconds[i], self.dt)))),
                    'rl_pitch_peak_deg': float(np.rad2deg(self.rl_pitch_peak[i]))}
                   if self.reward_profile == 'pitch' else {}),
                'final_applied_torque_nm': self.applied_torque[i].tolist(),
                'final_virtual_body_wrench': self.virtual_body_wrench[i].tolist()}


class ReturnWrenchSpinEnv(ReturnCurriculumMixin, WrenchSpinEnv):
    """Return training with exclusive full-actuator control and sensor-only actor."""

"""V7: privileged critic observations with an unchanged sensor actor boundary."""

from pathlib import Path

import gymnasium as gym
import numpy as np
from spin_anchor_env import AnchorSpinEnv, heading_rotate
from spin_asymmetric_policy import ACTOR_SIZE
from spin_imu import aligned_control_attitude, imu_attitude_extrinsic
from spin_leg_geometry import EncoderLegGeometry
from spin_rl_env import attitude
from spin_sensor_odometry import InnovationSpinOdometry
from spin_torque_loop import TorquePI, SimulatedTorqueMotors
from spin_wrench_control import WrenchTorqueMapper
from wheel_legged.provenance import implementation_digest

TASK_VERSION = 'slope_spin_asymmetric_v7'
PRIVILEGED_NAMES = (
    'true_position_heading_x/.30', 'true_position_heading_y/.30',
    'true_velocity_heading_x/.5', 'true_velocity_heading_y/.5', 'true_vertical_velocity/.5',
    'left_normal_load/weight', 'left_contact_slip/.5', 'left_contact_height_from_base/.2', 'left_loaded',
    'right_normal_load/weight', 'right_contact_slip/.5', 'right_contact_height_from_base/.2', 'right_loaded',
    'mapped_requested_length/.2', 'mapped_uphill_length/.2', 'mapped_downhill_length/.2',
    'distance_from_ramp_toe/2', 'distance_from_ramp_crest/2', 'ramp_grade/.2',
    'excursion_age/6', 'excursion_active', 'return_dwell/.5', 'remaining_episode_fraction',
)
PRIVILEGED_SIZE = len(PRIVILEGED_NAMES)


class AsymmetricSpinEnv(AnchorSpinEnv):
    def __init__(self, *args, directional_reference=False, friction_range=None,
                 residual_torque_loop='ideal', linkage_map='native', **kwargs):
        if linkage_map not in ('native', 'exact'):
            raise ValueError('Expected native or exact linkage map')
        if residual_torque_loop not in ('ideal', 'open', 'pi'):
            raise ValueError('Expected ideal, open or pi residual torque loop')
        if friction_range is not None:
            values = np.asarray(friction_range, dtype=float)
            if values.shape != (2,) or not np.isfinite(values).all() or not 0 < values[0] <= values[1]:
                raise ValueError('Expected positive finite friction bounds')
            friction_range = tuple(values)
        super().__init__(*args, **kwargs)
        self.residual_torque_loop = residual_torque_loop
        self.linkage_map = linkage_map
        self.force_mapper = WrenchTorqueMapper(self.model) if linkage_map == 'exact' else None
        self.motor_limits = np.array([60., 60., 10., 60., 60., 10.])
        self.motor_ctrl_gain = self.model.actuator_gainprm[:, 0]*self.model.actuator_gear[:, 0]
        self.residual_pi = TorquePI(self.num_envs, self.motor_limits)
        self.residual_motors = SimulatedTorqueMotors(self.num_envs, self.motor_limits)
        self.friction_range = friction_range
        self.friction_geoms = [self.model.geom(name).id for name in (
            'floor', 'spin_ramp', 'spin_upper_flat', 'L_Wheel_mesh_1', 'R_Wheel_mesh_1')]
        self.episode_friction = np.full(self.num_envs, np.nan)
        self.imu_extrinsic = imu_attitude_extrinsic(self.model)
        self.geometry = EncoderLegGeometry(self.model)
        self.odometry = InnovationSpinOdometry(self.num_envs, lateral=self.axle_lateral,
                                               directional_reference=directional_reference)
        self.warmup_velocity = np.zeros((self.num_envs, 3))
        self.wheel_rejected_seconds = np.zeros((self.num_envs, 2))
        self.sideways_rejected_seconds = np.zeros(self.num_envs)
        self.geometry_invalid_seconds = np.zeros((self.num_envs, 2))
        self.external_force = self.batch.bind('xfrc_applied')
        self.push_force = np.zeros((self.num_envs, 2))
        self.push_remaining = np.zeros(self.num_envs)
        self.base_body = self.model.body('base_link').id
        self.observation_space = gym.spaces.Box(
            -np.inf, np.inf, (ACTOR_SIZE+PRIVILEGED_SIZE,), np.float32)
        self.provenance.update(
            task=TASK_VERSION, algorithm='asymmetric PPO', actor_size=ACTOR_SIZE,
            critic_size=ACTOR_SIZE+PRIVILEGED_SIZE,
            privileged_features=list(PRIVILEGED_NAMES), actor_world_truth_inputs=False,
            odometry_quaternion_frame='calibrated IMU site, aligned with gyro and accelerometer',
            odometry_attitude_extrinsic=self.imu_extrinsic.tolist(),
            native_attitude_frame='legacy inertial axes; existing PID/LQR trim retained',
            odometry_axle_lateral_m=self.axle_lateral.tolist(),
            odometry_version=('encoder_support_innovation_v2_directional_reference'
                              if directional_reference else 'encoder_support_innovation_v1'),
            odometry_initial_velocity='calibrated IMU integration from stationary reset, through PID warmup',
            wheel_confidence='independent wheel/inertial innovation, not friction coefficient',
            disturbance='60 ms physical force pulse, IMU observable',
            rollout_friction_range=list(friction_range) if friction_range is not None else None,
            rollout_friction_scope='both wheels and all terrain pieces; sampled after clean PID preparation',
            residual_torque_loop=residual_torque_loop,
            linkage_map=linkage_map,
            linkage_map_scope='remap native virtual forces; retain existing servo feedback and wheel motor torques',
            residual_motor_assumptions=({'time_constant_s': .008, 'gain_range': [.8, 1.2],
                'pi_enabled': residual_torque_loop == 'pi',
                'kp': .4, 'ki_per_s': 60, 'anti_windup': 'conditional integration'}
                if residual_torque_loop != 'ideal' else None),
            asymmetric_source_sha256=implementation_digest(*(
                Path(__file__).with_name(name) for name in
                ('spin_asymmetric_env.py', 'spin_asymmetric_policy.py', 'spin_imu.py',
                 'spin_leg_geometry.py', 'spin_sensor_odometry.py', 'spin_wrench_control.py',
                 'spin_torque_loop.py'))))

    def _reset_ids(self, ids):
        self.warmup_velocity[ids] = 0
        super()._reset_ids(ids)

    def _controller_warmup(self, warm, mask, terrain):
        delta = np.zeros_like(self.warmup_velocity)
        self.control.warmup(self.model, self.state, warm, mask, self.sensordata,
                            self.actuator_force, terrain=terrain, imu_delta=delta)
        self.warmup_velocity += delta

    def _place_on_terrain(self, ids):
        _, _, before, _, _ = attitude(*self._imu()[:2])
        super()._place_on_terrain(ids)
        _, _, after, _, _ = attitude(*self._imu()[:2])
        self.warmup_velocity[ids, :2] = heading_rotate(
            self.warmup_velocity[:, :2], after-before)[ids]

    def _reset_context(self, ids):
        super()._reset_context(ids)
        self.odometry.initialize_velocity(self.warmup_velocity[ids, :2], self._imu()[1][ids], ids)
        self.wheel_rejected_seconds[ids] = self.sideways_rejected_seconds[ids] = 0
        self.geometry_invalid_seconds[ids] = 0
        self.push_force[ids] = self.push_remaining[ids] = 0
        self.episode_friction[ids] = self.floor_friction[ids, self.floor, 0]
        if self.friction_range is not None:
            self.episode_friction[ids] = [self.rng[i].uniform(*self.friction_range) for i in ids]
            # Equal-priority contact materials use the larger coefficient, so
            # randomizing only the floor would not produce low-friction trials.
            self.floor_friction[np.ix_(ids, self.friction_geoms, [0])] = self.episode_friction[ids, None, None]
        if self.residual_torque_loop != 'ideal':
            self.residual_pi.reset(ids)
            measured = self.actuator_force[ids]*self.model.actuator_gear[:, 0]
            gains = np.array([self.rng[i].uniform(.8, 1.2, 6) for i in ids])
            self.residual_motors.reset(ids, np.clip(measured, -self.motor_limits, self.motor_limits), gains)

    def _actuator_control(self, mask):
        ctrl = super()._actuator_control(mask)
        if self.linkage_map == 'exact':
            ctrl = ctrl.copy()
            source_j = self.control.leg_jacobians().swapaxes(-1, -2)
            _, exact_j, _, valid = self.force_mapper.jacobians(self.qpos[:, self.qa[:4]])
            leg_tau = ctrl[:, [0, 1, 3, 4]]*self.motor_ctrl_gain[[0, 1, 3, 4]]
            forces = np.einsum('nsij,nsj->nsi', np.linalg.pinv(source_j, rcond=1e-4),
                                leg_tau.reshape(self.num_envs, 2, 2))
            remapped = np.einsum('nscj,nsc->nsj', exact_j, forces)
            valid &= np.linalg.cond(source_j) < 1e4
            leg_tau = np.where(valid[:, :, None], remapped, leg_tau.reshape(self.num_envs, 2, 2))
            ctrl[:, [0, 1, 3, 4]] = np.clip(leg_tau.reshape(self.num_envs, 4), -60, 60)/self.motor_ctrl_gain[[0, 1, 3, 4]]
        if self.residual_torque_loop == 'ideal':
            return ctrl
        measured = self.actuator_force*self.model.actuator_gear[:, 0]
        target = ctrl*self.motor_ctrl_gain
        command = self.residual_pi.step(target, measured, self.dt) if self.residual_torque_loop == 'pi' else target
        return self.residual_motors.step(command, self.dt)/self.motor_ctrl_gain

    def _apply_push(self, push):
        self.push_force[push] = self.model.body_subtreemass[self.base_body]*self.push_velocity[push]/.060
        self.external_force[push, self.base_body, :2] += self.push_force[push]
        self.push_remaining[push] = .060

    def _kinematic_observation(self):
        frames = self.control.frames
        positions = np.column_stack([frames[k+'_pos'] for k in ('lf', 'lr', 'rf', 'rr')])
        velocities = np.column_stack([frames[k+'_vel'] for k in ('lf', 'lr', 'rf', 'rr')])
        return self.geometry.observation(self._native_observation(), positions, velocities)

    def _after_physics_step(self):
        _, gyro, acc = self._imu()
        q = aligned_control_attitude(self.sensordata[:, :4], self.imu_extrinsic)
        native, valid = self._kinematic_observation()
        self.odometry.step(q, gyro, acc, native, self.dt)
        self.wheel_rejected_seconds += (self.odometry.wheel_confidence < .2)*self.dt
        self.sideways_rejected_seconds += (self.odometry.sideways_confidence < .2)*self.dt
        self.geometry_invalid_seconds += (~valid)*self.dt
        active = self.push_remaining > 0
        self.push_remaining[:] = np.maximum(self.push_remaining-self.dt, 0)
        finished = active & (self.push_remaining < 1e-9)
        self.external_force[finished, self.base_body, :2] -= self.push_force[finished]
        self.push_force[finished] = 0
        self.push_remaining[finished] = 0

    def _extra_metrics(self, i):
        return {**super()._extra_metrics(i),
                'wheel_rejected_seconds': self.wheel_rejected_seconds[i].tolist(),
                'sideways_rejected_seconds': float(self.sideways_rejected_seconds[i]),
                'geometry_invalid_seconds': self.geometry_invalid_seconds[i].tolist(),
                'effective_rollout_friction': float(self.episode_friction[i])}

    def _privileged_observation(self):
        _, _, yaw, _, _ = attitude(*self._imu()[:2])
        delta = heading_rotate(self.qpos[:, :2]-self.start_xy, -yaw)/.30
        velocity = heading_rotate(self.qvel[:, :2], -yaw)/.5
        # Contact truth is queried only on the critic path at policy frequency.
        contact = self.contacts.query(self.state).copy()
        loaded = contact[:, :, 0] > 1
        contact[:, :, 0] /= self.robot_weight
        contact[:, :, 1] /= .5
        contact[:, :, 2] = np.where(loaded, (contact[:, :, 2]-self.qpos[:, None, 2])/.2, 0)
        contact[:, :, 3] = loaded
        return np.column_stack((
            delta, velocity, self.qvel[:, 2]/.5, contact.reshape(self.num_envs, -1),
            self.standard_requested/.2, self.standard_actual/.2,
            self.qpos[:, 0]/2, (self.qpos[:, 0]-self.ramp_length)/2,
            self.ramp_height/np.maximum(self.ramp_length, 1e-6)/.2,
            self.excursion_age/6, self.excursion_active, self.return_dwell/.5,
            1-self.steps/self.episode_steps)).astype(np.float32)

    def _observation(self):
        return np.column_stack((super()._observation(), self._privileged_observation()))

"""MuJoCo/mjbatch CPU adapter for the common physical-state/torque interface.

This is a low-level physics adapter, not a Gym task or native-controller wrapper.
It requires the v3 free-base robot and named IMU sensors. Domain randomization,
terrain generation, task termination and RL ownership belong to task code.
"""

from pathlib import Path

import mujoco
import numpy as np
from mjbatch import Batch

from .mujoco_model import JointBinding, imu_attitude_extrinsic
from ..core.frames import aligned_control_attitude
from ..core.robot import TORQUE_LIMIT_NM
from ..interfaces import SensorState, TruthState


class MujocoBackend:
    def __init__(self, model, num_envs=1, num_threads=1):
        if num_envs < 1 or num_threads < 1:
            raise ValueError('Expected positive environment and thread counts')
        self.model = model
        self.num_envs, self.dt = num_envs, float(model.opt.timestep)
        if not np.isfinite(self.dt) or self.dt <= 0:
            raise ValueError('Invalid physics timestep')
        self.binding = JointBinding(model)
        self.extrinsic = imu_attitude_extrinsic(model)
        base = model.body('base_link')
        joint = int(base.jntadr[0])
        if base.jntnum[0] != 1 or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError('Backend expects a free joint on base_link')
        self.base_qadr = int(model.jnt_qposadr[joint])
        self.base_vadr = int(model.jnt_dofadr[joint])
        self.sensor_slices = {}
        for name, size in (('body_quat', 4), ('body_gyro', 3), ('body_acc', 3)):
            sensor = model.sensor(name).id
            if model.sensor_dim[sensor] != size:
                raise ValueError(f'Unexpected sensor size for {name}')
            address = model.sensor_adr[sensor]
            self.sensor_slices[name] = slice(address, address+size)
        self.ball_qadr = []
        for body in range(1, model.nbody):
            if model.body(body).name.startswith(('test_ball_', 'large_ball_')):
                self.ball_qadr.append(int(model.jnt_qposadr[model.body_jntadr[body]]))
        self.batch = Batch(model, num_envs, num_threads=num_threads, forward=False)
        self.qpos, self.qvel, self.ctrl, self.sensors, self.actuator_force = (
            self.batch.bind(key) for key in ('qpos', 'qvel', 'ctrl', 'sensordata', 'actuator_force'))
        self._closed = False
        self.reset()

    @classmethod
    def from_xml(cls, path, **kwargs):
        return cls(mujoco.MjModel.from_xml_path(str(Path(path).resolve())), **kwargs)

    def _check_open(self):
        if self._closed:
            raise RuntimeError('Physics backend is closed')

    def read_sensors(self):
        self._check_open()
        raw = self.sensors[:, self.sensor_slices['body_quat']]
        quaternion = aligned_control_attitude(raw, self.extrinsic)
        # Gyro/acc are IMU site axes; legacy control frame is site * Rx(-90deg).
        gyro = self.sensors[:, self.sensor_slices['body_gyro']][:, [0, 2, 1]]*[1, -1, 1]
        acc = self.sensors[:, self.sensor_slices['body_acc']][:, [0, 2, 1]]*[1, -1, 1]
        return SensorState(self.qpos[:, self.binding.qadr].copy(),
                           self.qvel[:, self.binding.vadr].copy(), quaternion, gyro, acc)

    def read_truth(self):
        self._check_open()
        q, v, a = self.base_qadr, self.base_vadr, self.binding.actuator_ids
        return TruthState(self.qpos[:, q:q+3].copy(), self.qvel[:, v:v+3].copy(),
            (self.actuator_force[:, a]*self.model.actuator_gear[a, 0]).copy())

    def apply_joint_torque(self, torque_nm):
        self._check_open()
        torque_nm = np.asarray(torque_nm)
        if torque_nm.shape != (self.num_envs, 6) or not np.isfinite(torque_nm).all():
            raise ValueError('Expected finite joint Nm with shape (N,6)')
        if np.any(np.abs(torque_nm) > TORQUE_LIMIT_NM+1e-12):
            raise ValueError('Joint torque exceeds configured physical limits')
        ctrl = self.binding.control_from_torque(torque_nm)
        limited = self.model.actuator_ctrllimited.astype(bool)
        if (np.any(ctrl[:, limited] < self.model.actuator_ctrlrange[limited, 0])
                or np.any(ctrl[:, limited] > self.model.actuator_ctrlrange[limited, 1])):
            raise ValueError('Engine ctrl limits would clip requested physical torque')
        a = self.binding.actuator_ids
        joint_ids = self.model.actuator_trnid[a, 0]
        force = torque_nm/self.model.actuator_gear[a, 0]
        bounds = self.model.actuator_forcerange[a]
        mask = self.model.actuator_forcelimited[a].astype(bool)
        if (np.any(force[:, mask] < bounds[mask, 0])
                or np.any(force[:, mask] > bounds[mask, 1])):
            raise ValueError('Engine actuator force limits would clip requested torque')
        mask = self.model.jnt_actfrclimited[joint_ids].astype(bool)
        bounds = self.model.jnt_actfrcrange[joint_ids]
        if (np.any(torque_nm[:, mask] < bounds[mask, 0])
                or np.any(torque_nm[:, mask] > bounds[mask, 1])):
            raise ValueError('Engine joint force limits would clip requested torque')
        self.ctrl[:] = ctrl

    def step(self):
        self._check_open()
        self.batch.step()
        # Read-after-step contract includes current gyro/acc, not lagged sensors.
        self.batch.forward()

    def reset(self, ids=None):
        self._check_open()
        ids = np.arange(self.num_envs, dtype=np.int32) if ids is None else np.asarray(ids, dtype=np.int32)
        if ids.ndim != 1 or np.any(ids < 0) or np.any(ids >= self.num_envs):
            raise ValueError('Invalid environment reset indices')
        if not len(ids):
            return
        self.batch.reset(ids=ids, keyframe=0 if self.model.nkey else -1)
        self.ctrl[ids] = 0
        # The baseline fixture is flat ground; old scattered test balls are remote.
        for index, address in enumerate(self.ball_qadr):
            self.qpos[ids, address:address+3] = [100+index, 100, 0]
        self.batch.forward(ids=ids)

    def close(self):
        if not self._closed:
            self.qpos = self.qvel = self.ctrl = self.sensors = self.actuator_force = None
            self.batch = None
            self._closed = True

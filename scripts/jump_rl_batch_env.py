"""mjbatch/SB3 environment retaining the existing 34 observations and 5 actions."""

import os
import time

import gymnasium as gym
import mujoco
import numpy as np
from mjbatch import Batch
from stable_baselines3.common.vec_env import VecEnv

from jump_rl_env import (
    EPISODE_S,
    IMU_BAKE_Q,
    LEG_GAIN,
    LEG_V3_SW_NOM,
    NOM_HEIGHT,
    OBS_CUR_SCALE,
    RETRACT_DONE_L,
    TAU_DIFF_SCALE,
    TAU_L_SCALE,
    TAU_SW_SCALE,
    WARMUP_RANGE,
    XML,
    body_quat_from_control_euler,
    leg_v3_forces,
    leg_v3_forward,
    leg_v3_inverse,
)
from rl_batch_control import BatchControl


class JumpRLBatchEnv(VecEnv):
    render_mode = None

    def __init__(
        self, num_envs=16, num_threads=8, timestep=0.001, episode_steps=None, min_height=0.15
    ):
        if num_envs < 1 or num_threads < 1 or timestep <= 0:
            raise ValueError("Positive environment count, thread count and timestep required")
        self.model = mujoco.MjModel.from_xml_path(XML)
        self.model.opt.timestep = timestep
        self.dt = timestep
        self.episode_steps = int(EPISODE_S / timestep) if episode_steps is None else episode_steps
        if self.episode_steps < 1:
            raise ValueError("episode_steps must be positive")
        self.min_height = min_height
        self.curriculum = os.environ.get("RL_CURRICULUM", "full")
        self.batch = Batch(self.model, num_envs, num_threads=num_threads, forward=False)
        self.control = BatchControl(num_envs, num_threads)
        self.qpos = self.batch.bind("qpos")
        self.qvel = self.batch.bind("qvel")
        self.ctrl = self.batch.bind("ctrl")
        self.sensors = self.batch.bind("sensordata")
        self.forces = self.batch.bind("actuator_force")
        self.state = self.batch.bind("state")
        names = ["Left_front_joint", "Left_rear_joint", "Right_front_joint", "Right_rear_joint"]
        joints = [self.model.joint(name) for name in names]
        self.leg_qpos = [int(j.qposadr[0]) for j in joints]
        self.leg_dof = [int(j.dofadr[0]) for j in joints]
        lw, rw = self.model.joint("Left_Wheel_joint"), self.model.joint("Right_Wheel_joint")
        self.lw_qpos, self.rw_qpos = int(lw.qposadr[0]), int(rw.qposadr[0])
        self.lw_dof, self.rw_dof = int(lw.dofadr[0]), int(rw.dofadr[0])
        self.step_count = np.zeros(num_envs, dtype=np.int64)
        self.warm_steps = np.zeros(num_envs, dtype=np.int64)
        self.max_height = np.zeros(num_envs)
        self.reached_min = np.zeros(num_envs, dtype=bool)
        self.fallen = np.zeros(num_envs, dtype=bool)
        self.side_flip = np.zeros(num_envs, dtype=bool)
        self.expert_steps = np.zeros(num_envs, dtype=np.int64)
        self.act_filt = np.zeros((num_envs, 5))
        self.prev_airborne = np.zeros(num_envs, dtype=bool)
        self.landing = np.zeros(num_envs, dtype=bool)
        self.landing_t = np.zeros(num_envs)
        self.episode_returns = np.zeros(num_envs)
        self.episode_start = np.full(num_envs, time.monotonic())
        self.rng = [np.random.default_rng() for _ in range(num_envs)]
        self._actions = None
        self._closed = False
        super().__init__(
            num_envs,
            gym.spaces.Box(-np.inf, np.inf, shape=(34,), dtype=np.float32),
            gym.spaces.Box(-1.0, 1.0, shape=(5,), dtype=np.float32),
        )

    def _imu(self):
        b = self.sensors[:, :4]
        c = IMU_BAKE_Q[0]
        q = c * np.column_stack(
            (b[:, 0] + b[:, 1], b[:, 1] - b[:, 0], b[:, 2] - b[:, 3], b[:, 2] + b[:, 3])
        )
        gyro = self.sensors[:, [4, 6, 5]] * np.array([1.0, -1.0, 1.0])
        acc = self.sensors[:, [7, 9, 8]] * np.array([1.0, -1.0, 1.0])
        return q, gyro, acc

    def _get_obs(self):
        q, gyro, _ = self._imu()
        w, x, y, z = q.T
        pitch = np.arcsin(np.clip(2 * (w * y - x * z), -1, 1))
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sw, ll, lr = leg_v3_forward(*self.qpos[:, self.leg_qpos].T)
        obs = np.empty((self.num_envs, 34), dtype=np.float32)
        obs[:, 0], obs[:, 1] = pitch, roll
        obs[:, 2:4] = gyro[:, [1, 0]]
        obs[:, 4], obs[:, 5], obs[:, 6] = sw, ll, lr
        obs[:, 7:11] = self.qvel[:, self.leg_dof]
        obs[:, 11], obs[:, 12] = self.qvel[:, self.lw_dof], self.qvel[:, self.rw_dof]
        obs[:, 13] = self.qpos[:, 2]
        obs[:, 14:16] = np.abs(self.qvel[:, :2])
        obs[:, 16], obs[:, 17] = pitch * 57.3, roll * 57.3
        obs[:, 18] = self.qpos[:, 2] - NOM_HEIGHT
        obs[:, 19] = self.step_count * self.dt
        obs[:, 20:23] = self.qvel[:, :3]
        obs[:, 23], obs[:, 24] = self.qpos[:, self.lw_qpos], self.qpos[:, self.rw_qpos]
        obs[:, 25:27] = self.qpos[:, :2]
        obs[:, 27:33] = self.forces / OBS_CUR_SCALE
        obs[:, 33] = (self.qpos[:, 2] - NOM_HEIGHT) > 0.02
        return obs

    def _fill_frames(self, obs):
        f = self.control.frames
        q, g, a = self._imu()
        w, x, y, z = q.T
        f["pitch"], f["roll"] = obs[:, 0], obs[:, 1]
        f["pitch_rate"], f["roll_rate"], f["yaw_rate"] = g[:, 1], g[:, 0], g[:, 2]
        f["yaw"] = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        f["quat"], f["body_pos"], f["body_vel"] = q, self.qpos[:, :3], self.qvel[:, :3]
        f["acc_mag"] = np.hypot(a[:, 0], a[:, 2])
        f["acc_x"], f["acc_y"], f["acc_z"] = a.T
        f["left_wheel_vel"], f["right_wheel_vel"] = (
            self.qvel[:, self.lw_dof],
            self.qvel[:, self.rw_dof],
        )
        f["left_wheel_pos"], f["right_wheel_pos"] = (
            self.qpos[:, self.lw_qpos],
            self.qpos[:, self.rw_qpos],
        )
        f["left_wheel_tau"], f["right_wheel_tau"] = self.forces[:, 2] * 10, self.forces[:, 5] * 10
        for i, name in enumerate(("lf", "lr", "rf", "rr")):
            f[name + "_pos"] = self.qpos[:, self.leg_qpos[i]]
            f[name + "_vel"] = self.qvel[:, self.leg_dof[i]]
            f[name + "_tau"] = self.forces[:, (0, 1, 3, 4)[i]]
        f["sim_time"], f["step_count"] = self.warm_steps * self.dt, self.warm_steps

    def _setup_initial(self, ids):
        self.batch.reset(ids=ids)
        self.qpos[ids] = self.model.qpos0
        self.qpos[ids, 2] = -0.025
        self.qpos[ids, 3:7] = IMU_BAKE_Q
        self.qpos[np.ix_(ids, self.leg_qpos)] = [0.2, -0.2, -0.2, 0.2]
        self.qvel[ids] = 0
        self.ctrl[ids] = 0
        self.batch.forward(ids=ids)

    def _reset_ids(self, ids):
        ids = np.asarray(ids, dtype=np.int32)
        if len(ids) == 0:
            return
        selected = np.zeros(self.num_envs, dtype=bool)
        selected[ids] = True
        for i in ids:
            self.expert_steps[i] = 1000 if self.rng[i].uniform() < 0.5 else 0
        retry = selected.copy()
        for _ in range(4):
            current = np.flatnonzero(retry).astype(np.int32)
            if not len(current):
                break
            self.control.reset(retry)
            self._setup_initial(current)
            self.warm_steps[current] = 0
            targets = np.zeros(self.num_envs, dtype=np.int64)
            for i in current:
                targets[i] = int(self.rng[i].uniform(*WARMUP_RANGE) / self.dt)
            self.control.warmup(self.model, self.state, targets, retry, self.sensors, self.forces)
            self.warm_steps[current] = targets[current]
            obs = self._get_obs()
            valid = (
                (np.abs(obs[:, 16]) < 10)
                & (np.abs(obs[:, 17]) < 8)
                & (np.max(np.abs(obs[:, 11:13]), axis=1) < 20)
            )
            retry &= ~valid
        for i in ids:
            r = self.rng[i]
            self.qpos[i, 2] = r.uniform(0.07, 0.10)
            self.qpos[i, 3:7] = body_quat_from_control_euler(r.uniform(-0.06, 0.06, 3))
            self.qpos[i, 0], self.qpos[i, 1] = r.uniform(-0.5, 0.5, 2)
            for adr, (lo, hi) in zip(
                self.leg_qpos, [(-0.06, 0.08), (-0.06, 0.08), (-0.16, -0.04), (-0.06, 0.06)]
            ):
                self.qpos[i, adr] = r.uniform(lo, hi)
            self.qpos[i, self.lw_qpos], self.qpos[i, self.rw_qpos] = r.uniform(-10, 10, 2)
            self.qvel[i, :3] = r.uniform(-0.8, 0.8, 3)
            self.qvel[i, 3:6] = r.uniform(-1, 1, 3)
            self.qvel[i, self.lw_dof] += r.uniform(-4, 4)
            self.qvel[i, self.rw_dof] += r.uniform(-4, 4)
            self.ctrl[i] = 0
        self.batch.forward(ids=ids)
        self.qpos[np.ix_(ids, self.leg_qpos)] = leg_v3_inverse(
            LEG_V3_SW_NOM, RETRACT_DONE_L, RETRACT_DONE_L
        )
        self.batch.forward(ids=ids)
        self.control.set_active(selected, True)
        for values in (
            self.step_count,
            self.max_height,
            self.reached_min,
            self.fallen,
            self.side_flip,
            self.act_filt,
            self.prev_airborne,
            self.landing,
            self.landing_t,
            self.episode_returns,
        ):
            values[ids] = 0
        self.episode_start[ids] = time.monotonic()

    def reset(self):
        if self._closed:
            raise RuntimeError("Environment is closed")
        self._actions = None
        for i, seed in enumerate(self._seeds):
            if seed is not None:
                self.rng[i] = np.random.default_rng(seed)
        self._reset_ids(np.arange(self.num_envs, dtype=np.int32))
        self._reset_seeds()
        self._reset_options()
        return self._get_obs()

    def step_async(self, actions):
        if self._closed or self._actions is not None:
            raise RuntimeError("Environment closed or a step is already pending")
        actions = np.asarray(actions, dtype=np.float32)
        if actions.shape != (self.num_envs, 5) or not np.isfinite(actions).all():
            raise ValueError("Expected finite actions of shape (num_envs, 5)")
        self._actions = actions.copy()

    def _step_ctrl(self, actions):
        obs = self._get_obs()
        airborne = obs[:, 33] > 0.5
        ground = ~airborne
        landed = ground & self.prev_airborne
        self.landing[landed] = True
        self.landing_t[landed] = 0
        self.landing_t[ground & self.landing] += self.dt
        self.landing[ground & (self.landing_t > 0.3)] = False
        gain = np.where(self.landing, 4.0, 1.5)
        a = np.clip(actions, -1, 1)
        a[ground, 2] = np.clip(
            a[ground, 2]
            + (-gain[ground] * obs[ground, 17] - 0.3 * obs[ground, 3]) / TAU_DIFF_SCALE,
            -1,
            1,
        )
        self.prev_airborne[:] = airborne
        fl, fs, fd = a[:, 0] * TAU_L_SCALE, a[:, 1] * TAU_SW_SCALE, a[:, 2] * TAU_DIFF_SCALE
        legs = leg_v3_forces(fl + 0.5 * fd, fs, fl - 0.5 * fd, fs).T / LEG_GAIN
        self.ctrl[:, [0, 1, 3, 4]] = legs
        self.ctrl[:, [2, 5]] = 0
        if ground.any():
            self._fill_frames(obs)
            wheel = self.control.step(ground, self.dt)
            self.ctrl[np.ix_(ground, [2, 5])] = wheel[np.ix_(ground, [2, 5])]
        np.clip(self.ctrl, -10, 10, out=self.ctrl)
        self.batch.step()

    def _reward(self, obs):
        pitch, roll, h, air = obs[:, 16], obs[:, 17], obs[:, 18], obs[:, 33]
        cap = np.clip(h, 0, 0.18)
        mh_cap = np.clip(self.max_height, 0, 0.20)
        reward = 40 * cap - 0.5 * np.maximum(0, -h) - 10 * np.maximum(0, h - 0.18)
        reward += np.where(cap > mh_cap + 1e-4, 4 * (cap - mh_cap), 0)
        self.max_height = np.maximum(self.max_height, h)
        reward += np.where(
            (air > 0.5) & (self.step_count < 600),
            -0.8 * np.abs(obs[:, 2]) - 1.5 * np.abs(obs[:, 3]),
            0,
        )
        reward -= 8 * (1 - np.exp(-(roll**2) / (2 * 15.0**2)))
        land = np.where(
            (np.abs(roll) < 10) & (np.abs(pitch) < 25),
            8.0,
            np.where(np.abs(roll) < 25, -5.0, -12.0),
        )
        land -= 4 * np.abs(obs[:, 5] - obs[:, 6])
        reward += np.where((self.max_height > 0.02) & (h < 0.03), land, 0)
        reward -= 0.002 * np.abs(self.forces[:, [0, 1, 3, 4]]).sum(axis=1)
        side = np.abs(roll) > 30
        forward = ~side & (np.abs(pitch) > 50)
        self.side_flip[side] = True
        self.side_flip[forward] = False
        self.fallen |= side | forward
        reward += np.where(side, -50.0, np.where(forward, -10.0, 0))
        reward -= 0.03 * (np.abs(obs[:, 20]) + np.abs(obs[:, 21]))
        return reward

    def step_wait(self):
        if self._actions is None:
            raise RuntimeError("Call step_async before step_wait")
        a = np.clip(self._actions, -1, 1)
        self._actions = None
        expert = self.step_count < self.expert_steps
        a[expert] = 0
        launch = expert & (self.step_count * self.dt >= 0.05) & (self.step_count * self.dt < 0.45)
        a[launch] = [-0.6, 0.3, 0, 0.5, 0.5]
        self.act_filt = 0.5 * self.act_filt + 0.5 * a
        self._step_ctrl(self.act_filt)
        self.step_count += 1
        obs = self._get_obs()
        reward = self._reward(obs)
        reached = ~self.reached_min & (self.max_height >= self.min_height)
        self.reached_min |= reached
        reward += 30 * reached
        no_jump = ~self.fallen & (self.max_height < 0.02) & (self.step_count > 800)
        self.side_flip[no_jump] = False
        terminated = self.fallen | no_jump
        truncated = self.step_count >= self.episode_steps
        reward += np.where(
            terminated,
            np.where(self.side_flip | (self.max_height < 0.02), -100.0, -10.0),
            np.where(truncated, np.where(self.reached_min, 10.0, -50.0), 0),
        )
        done = terminated | truncated
        self.episode_returns += reward
        infos = []
        for i in range(self.num_envs):
            info = {
                "height": self.max_height[i],
                "reached_min": bool(self.reached_min[i]),
                "pitch_deg": obs[i, 16],
                "roll_deg": obs[i, 17],
                "TimeLimit.truncated": bool(truncated[i] and not terminated[i]),
            }
            if done[i]:
                info["terminal_observation"] = obs[i].copy()
                info["episode"] = {
                    "r": self.episode_returns[i],
                    "l": int(self.step_count[i]),
                    "t": time.monotonic() - self.episode_start[i],
                }
            infos.append(info)
        if done.any():
            self._reset_ids(np.flatnonzero(done))
            obs[done] = self._get_obs()[done]
        return obs, reward.astype(np.float32), done, infos

    def close(self):
        if not self._closed:
            self.control.close()
            self._closed = True
            self._actions = None
            self.qpos = self.qvel = self.ctrl = self.sensors = self.forces = self.state = None
            self.batch = None

    def get_attr(self, attr_name, indices=None):
        value = getattr(self, attr_name)
        ids = list(self._get_indices(indices))
        if isinstance(value, np.ndarray) and value.ndim and len(value) == self.num_envs:
            return [value[i] for i in ids]
        return [value for _ in ids]

    def set_attr(self, attr_name, value, indices=None):
        current = getattr(self, attr_name)
        ids = list(self._get_indices(indices))
        if isinstance(current, np.ndarray) and current.ndim and len(current) == self.num_envs:
            current[ids] = value
        elif ids == list(range(self.num_envs)):
            setattr(self, attr_name, value)
        else:
            raise ValueError(f"{attr_name} is shared; per-environment assignment is unsupported")

    def env_method(self, method_name, *args, indices=None, **kwargs):
        if method_name == "render":
            return [None for _ in self._get_indices(indices)]
        raise NotImplementedError(f"Per-environment method {method_name} is unsupported")

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False for _ in self._get_indices(indices)]

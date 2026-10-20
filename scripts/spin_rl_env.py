"""Slope-spin residual PPO task: sensor-only 41 x 8 history, five actions.

The deployed C controller runs at every physics step; the policy runs at 50 Hz.
Ground truth is used exclusively for reward, termination and diagnostics.
"""

import hashlib
import time
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from jump_rl_env import IMU_BAKE_Q, XML, quat_mul
from mjbatch import Batch
from rl_batch_control import BatchControl
from stable_baselines3.common.vec_env import VecEnv
from wheel_legged.core.frames import attitude, euler_rates

TASK_VERSION = "slope_spin_v2"
FRAME_SIZE = 41
HISTORY = 8
PHASES = ("hold", "accelerate", "spin", "decelerate", "recover")
FAILURES = ((1, "nonfinite"), (2, "roll"), (4, "pitch"), (8, "drift"), (16, "stall"))
ACTION_SCALE = np.array([0.01, 0.01, 0.005, 0.20, 0.15], dtype=np.float32)
LEG_MID = 0.117
EXT_LIMITS = (-0.093, 0.070)
# Fixed normalization is identical in training, evaluation and deployment.
OBS_SCALE = np.array(
    [1]*3 + [5]*3 + [20]*3 + [30]*2 + [0.2]*2 + [0.5]*2
    + [0.05]*2 + [0.5]*2 + [2] + [5]*2 + [0.04]*2
    + [0.2, 2, 3.5, 1] + [1]*5 + [5]*3 + [1]*5, dtype=np.float32,
)


def spin_reference(elapsed, duration, extension, direction, previous, dt, recovery_seconds=4):
    """Single source of the physical rad/s command for controller and reward."""
    turning = (elapsed >= 1) & (elapsed < duration-recovery_seconds)
    blend = np.clip(((LEG_MID-extension)-0.132)/0.013, 0, 1)
    goal = np.where(turning, 1.15*(2.4-1.2*blend)*direction, 0)
    target = previous+np.clip(goal-previous, -0.8*dt, 0.8*dt)
    target = np.where(np.abs(target) < 1e-8, 0, target)
    phase = np.where(turning, np.where(np.abs(target-goal) > 1e-6, 1, 2),
                     np.where(elapsed < 1, 0, np.where(np.abs(target) > 1e-6, 3, 4)))
    return target, phase


def spin_reward(roll, pitch, roll_rate, yaw_rate, yaw_target, speed, drift,
                actions, action_delta, length_error):
    """Reward density per second; finite terminal costs are applied separately."""
    # Bounded shaping prevents a failed pose from swamping every useful sample.
    terms = {
        "roll": -5*np.minimum((roll/np.deg2rad(3))**2, 4),
        "roll_rate": -1.5*np.minimum((roll_rate/np.deg2rad(30))**2, 4),
        "pitch": -0.25*np.minimum((pitch/np.deg2rad(8))**2, 4),
        "yaw": 2-0.5*np.where(
            np.abs(yaw_rate-yaw_target)/0.6 <= 1,
            0.5*((yaw_rate-yaw_target)/0.6)**2,
            np.minimum(np.abs(yaw_rate-yaw_target)/0.6, 6)-0.5),
        "drift": -0.2*(np.minimum((speed/0.3)**2, 9)
                        + np.minimum((drift/0.5)**2, 9)),
        "height": -0.1*np.minimum((length_error/0.02)**2, 9),
        "effort": -0.02*np.mean(actions**2, axis=1),
        "smooth": -0.05*np.mean(action_delta**2, axis=1),
    }
    return 1.0 + sum(terms.values()), terms


class SpinRLBatchEnv(VecEnv):
    render_mode = None
    failure_cost_per_second = 35
    training_reward_scale = 1.0

    def __init__(self, num_envs=16, num_threads=8, max_slope_deg=3.0,
                 episode_seconds=20.0, leg_ext_range=EXT_LIMITS,
                 fixed_slope_deg=None, fixed_heading=None, fixed_extension=None,
                 fixed_direction=None, residual_enabled=True, cases=None, pid_only=False,
                 controller_defines=None, reference_mode="spin", recovery_seconds=4,
                 model_path=None):
        if num_envs < 1 or num_threads < 1 or not 0 <= max_slope_deg <= 20:
            raise ValueError("Invalid batch dimensions or slope range")
        if not np.isfinite(episode_seconds) or episode_seconds <= 0:
            raise ValueError("episode_seconds must be positive")
        if not EXT_LIMITS[0] <= leg_ext_range[0] <= leg_ext_range[1] <= EXT_LIMITS[1]:
            raise ValueError("Leg extension range exceeds the full mechanical range")
        if fixed_slope_deg is not None and not -20 <= fixed_slope_deg <= 20:
            raise ValueError("Fixed slope must be in [-20,20] degrees")
        if fixed_extension is not None and not EXT_LIMITS[0] <= fixed_extension <= EXT_LIMITS[1]:
            raise ValueError("Fixed extension exceeds mechanical range")
        if fixed_direction is not None and fixed_direction not in (-1, 1):
            raise ValueError("Spin direction must be -1 or +1")
        if reference_mode not in ("spin", "hold"):
            raise ValueError("reference_mode must be spin or hold")
        if not np.isfinite(recovery_seconds) or recovery_seconds < 0:
            raise ValueError("recovery_seconds must be finite and nonnegative")
        self.reference_mode = reference_mode
        self.recovery_seconds = recovery_seconds
        model_path = Path(XML if model_path is None else model_path).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.model.opt.timestep = self.dt = 0.001
        self.policy_dt, self.substeps = 0.02, 20
        self.episode_steps = max(1, round(episode_seconds/self.policy_dt))
        self.max_slope_deg = max_slope_deg
        self.leg_ext_range = leg_ext_range
        self.fixed = (fixed_slope_deg, fixed_heading, fixed_extension, fixed_direction)
        self.cases = cases
        if cases is not None:
            values = np.asarray(cases, dtype=float)
            if values.shape != (num_envs, 4) or not np.isfinite(values).all():
                raise ValueError("cases must contain (slope_deg, heading_rad, extension, direction)")
            if (np.any(np.abs(values[:, 0]) > 20)
                    or np.any(values[:, 2] < EXT_LIMITS[0])
                    or np.any(values[:, 2] > EXT_LIMITS[1])
                    or not np.isin(values[:, 3], [-1, 1]).all()):
                raise ValueError("Evaluation case exceeds task limits")
        self.residual_enabled = residual_enabled
        self.floor = self.model.geom("floor").id
        self.model.geom_sameframe[self.floor] = mujoco.mjtSameFrame.mjSAMEFRAME_NONE
        self.model.geom_friction[self.floor, 0] = 1.0
        self.batch = Batch(self.model, num_envs, num_threads=num_threads, forward=False)
        self.control = BatchControl(num_envs, num_threads, pid_only=pid_only,
                                    controller_defines=controller_defines)
        self.provenance = {
            "controller_build": self.control.build_id,
            "pid_only": self.control.pid_only,
            "controller_defines": dict(controller_defines or {}),
            "reference_mode": reference_mode,
            "recovery_seconds": recovery_seconds,
            "mujoco_version": mujoco.__version__,
            "model_xml": str(model_path),
            "model_xml_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        }
        for name in ("qpos", "qvel", "ctrl", "sensordata", "actuator_force", "state"):
            setattr(self, name, self.batch.bind(name))
        self.floor_quat = self.batch.expand("geom_quat")
        self.floor_friction = self.batch.expand("geom_friction")
        self.geom_xmat = self.batch.bind("geom_xmat")
        names = ["Left_front_joint", "Left_rear_joint", "Right_front_joint", "Right_rear_joint",
                 "Left_Wheel_joint", "Right_Wheel_joint"]
        joints = [self.model.joint(name) for name in names]
        self.qa = [int(j.qposadr[0]) for j in joints]
        self.va = [int(j.dofadr[0]) for j in joints]
        self.balls = []
        for b in range(1, self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            if name.startswith(("test_ball_", "large_ball_")):
                self.balls.append(int(self.model.jnt_qposadr[self.model.body_jntadr[b]]))
        self.rng = [np.random.default_rng() for _ in range(num_envs)]
        self.history = np.zeros((num_envs, HISTORY, FRAME_SIZE), dtype=np.float32)
        self.last_action = np.zeros((num_envs, 5), dtype=np.float32)
        self.inputs = np.zeros((num_envs, 7), dtype=np.float32)
        self.ext = np.zeros(num_envs)
        self.direction = np.ones(num_envs)
        self.yaw_target = np.zeros(num_envs)
        self.slope_deg = np.zeros(num_envs)
        self.heading = np.zeros(num_envs)
        self.start_xy = np.zeros((num_envs, 2))
        self.steps = np.zeros(num_envs, dtype=np.int64)
        self.physics_steps = np.zeros(num_envs, dtype=np.int64)
        self.returns = np.zeros(num_envs)
        self.roll_sq = np.zeros(num_envs)
        self.roll_peak = np.zeros(num_envs)
        self.yaw_error_sq = np.zeros(num_envs)
        self.spin_seconds = np.zeros(num_envs)
        self.yaw_integral = np.zeros(num_envs)
        self.yaw_reference_integral = np.zeros(num_envs)
        self.drift_peak = np.zeros(num_envs)
        self.stall_seconds = np.zeros(num_envs)
        self.phase = np.zeros(num_envs, dtype=np.int32)
        self.failure_mask = np.zeros(num_envs, dtype=np.int32)
        self.failure_phase = np.full(num_envs, -1, dtype=np.int32)
        self.failure_time = np.full(num_envs, -1.0)
        self.phase_stats = np.zeros((num_envs, len(PHASES), 9))
        self.reward_totals = {name: np.zeros(num_envs) for name in (
            "survival", "roll", "roll_rate", "pitch", "yaw", "drift", "height", "effort", "smooth", "terminal")}
        self.start_wall = np.zeros(num_envs)
        self._actions, self._closed = None, False
        super().__init__(num_envs,
                         gym.spaces.Box(-np.inf, np.inf, (FRAME_SIZE*HISTORY,), np.float32),
                         gym.spaces.Box(-1, 1, (5,), np.float32))

    def _imu(self):
        b = self.sensordata[:, :4]
        q = IMU_BAKE_Q[0]*np.column_stack(
            (b[:, 0]+b[:, 1], b[:, 1]-b[:, 0], b[:, 2]-b[:, 3], b[:, 2]+b[:, 3]))
        gyro = self.sensordata[:, [4, 6, 5]]*np.array([1, -1, 1])
        acc = self.sensordata[:, [7, 9, 8]]*np.array([1, -1, 1])
        return q, gyro, acc

    def _fill_frames(self):
        f = self.control.frames
        q, gyro, acc = self._imu()
        roll, pitch, yaw, _, _ = attitude(q, gyro)
        f["pitch"], f["roll"], f["yaw"] = pitch, roll, yaw
        f["quat"] = q
        f["roll_rate"], f["pitch_rate"], f["yaw_rate"] = gyro.T
        f["acc_x"], f["acc_y"], f["acc_z"] = acc.T
        f["acc_mag"] = np.hypot(acc[:, 0], acc[:, 2])
        f["body_pos"], f["body_vel"] = self.qpos[:, :3], self.qvel[:, :3]
        for k, name in enumerate(("lf", "lr", "rf", "rr")):
            f[name+"_pos"], f[name+"_vel"] = self.qpos[:, self.qa[k]], self.qvel[:, self.va[k]]
            f[name+"_tau"] = self.actuator_force[:, (0, 1, 3, 4)[k]]
        for k, name in enumerate(("left_wheel", "right_wheel")):
            f[name+"_pos"] = self.qpos[:, self.qa[k+4]]
            f[name+"_vel"] = self.qvel[:, self.va[k+4]]
            f[name+"_tau"] = 10*self.actuator_force[:, (2, 5)[k]]
        f["sim_time"], f["step_count"] = self.physics_steps*self.dt, self.physics_steps

    def _frame_obs(self):
        q, gyro, _ = self._imu()
        roll, pitch, _, _, _ = attitude(q, gyro)
        obs = np.column_stack((self._native_observation(), self.last_action,
                               euler_rates(roll, pitch, gyro), self.control.spin_actions()))
        # Keep task intent observable even if the base controller gates its output.
        obs[:, 26] = self.yaw_target
        if not np.isfinite(obs).all():
            raise FloatingPointError("Nonfinite sensor observation")
        return np.clip(obs/OBS_SCALE, -20, 20).astype(np.float32)

    def _observation(self):
        return self.history.reshape(self.num_envs, -1).copy()

    def _native_observation(self):
        return self.control.spin_observation()

    def _reset_ids(self, ids):
        ids = np.asarray(ids, dtype=np.int32)
        if not len(ids):
            return
        mask = np.zeros(self.num_envs, dtype=bool)
        mask[ids] = True
        self.control.reset(mask)
        self.batch.reset(ids=ids, keyframe=0)
        self.floor_quat[ids, self.floor] = [1, 0, 0, 0]
        self.floor_friction[ids, self.floor] = self.model.geom_friction[self.floor]
        self.inputs[ids] = 0
        for i in ids:
            r = self.rng[i]
            slope, heading, ext, direction = self.fixed if self.cases is None else self.cases[i]
            self.slope_deg[i] = r.uniform(-self.max_slope_deg, self.max_slope_deg) if slope is None else slope
            self.heading[i] = r.uniform(-np.pi, np.pi) if heading is None else heading
            if ext is None:
                # Include the exact endpoints as well as continuous full-range samples.
                mode = r.random()
                ext = (self.leg_ext_range[0] if mode < 0.1 else self.leg_ext_range[1]
                       if mode < 0.2 else r.uniform(*self.leg_ext_range))
            self.ext[i] = ext
            self.direction[i] = r.choice([-1, 1]) if direction is None else direction
            for k, address in enumerate(self.balls):
                self.qpos[i, address:address+3] = [100+k, 100, 0]
            self.inputs[i, 0] = ext
        self.batch.forward(ids=ids)
        self.control.spin_command(self.inputs, mask)
        # Establish the requested height on flat ground, then introduce the slope.
        # No retries or filtering of failed preparations: failures are scored.
        warm = np.full(self.num_envs, 2500, dtype=np.int32)
        terrain = (self.floor_quat[:, self.floor], self.floor_friction[:, self.floor])
        self._controller_warmup(warm, mask, terrain)
        self._place_on_terrain(ids)
        self._warmup_scene(mask)
        self.physics_steps[ids] = 2520
        self.control.set_active(mask, True)
        self.batch.forward(ids=ids)
        self._assert_floor(ids)
        self.start_xy[ids] = self.qpos[ids, :2]
        for value in (self.steps, self.returns, self.roll_sq, self.roll_peak,
                      self.yaw_error_sq, self.spin_seconds, self.yaw_integral, self.yaw_reference_integral,
                      self.drift_peak, self.stall_seconds, self.last_action, self.yaw_target,
                      self.phase_stats, self.phase, self.failure_mask, *self.reward_totals.values()):
            value[ids] = 0
        self.failure_phase[ids] = -1
        self.failure_time[ids] = -1
        self.start_wall[ids] = time.monotonic()
        self._reset_context(ids)
        self.history[ids] = self._frame_obs()[ids, None, :]
        self.reset_infos = getattr(self, "reset_infos", [{} for _ in range(self.num_envs)])
        for i in ids:
            self.reset_infos[i] = {"slope_deg": float(self.slope_deg[i]),
                                   "extension": float(self.ext[i]),
                                   "preparation_speed": float(np.linalg.norm(self.qvel[i, :2]))}

    def _place_on_terrain(self, ids):
        for i in ids:
            a = np.deg2rad(self.slope_deg[i])/2
            slope_q = np.array([np.cos(a), 0, np.sin(a), 0])
            h = self.heading[i]/2
            rotation = quat_mul(slope_q, np.array([np.cos(h), 0, 0, np.sin(h)]))
            mat = np.empty(9)
            mujoco.mju_quat2Mat(mat, rotation)
            origin = self.model.geom_pos[self.floor]
            self.qpos[i, :3] = mat.reshape(3, 3)@(self.qpos[i, :3]-origin)+origin
            self.qpos[i, 3:7] = quat_mul(rotation, self.qpos[i, 3:7])
            self.qvel[i, :3] = mat.reshape(3, 3)@self.qvel[i, :3]
            self.floor_quat[i, self.floor] = slope_q
        self.batch.forward(ids=ids)

    def _warmup_scene(self, mask):
        # This warmup uses EACH environment's expanded terrain, not the template.
        warm = np.full(self.num_envs, 20, dtype=np.int32)
        terrain = (self.floor_quat[:, self.floor], self.floor_friction[:, self.floor])
        self._controller_warmup(warm, mask, terrain)

    def _controller_warmup(self, warm, mask, terrain):
        self.control.warmup(self.model, self.state, warm, mask, self.sensordata,
                            self.actuator_force, terrain=terrain)

    def _reset_context(self, ids):
        pass

    def _prepare_control(self, elapsed, actions, mask):
        self.inputs[:, 2:] = actions
        self.control.spin_command(self.inputs, mask)

    def _actuator_control(self, mask):
        return self.control.step(mask, self.dt)

    def _extra_metrics(self, i):
        return {}

    def _before_policy_step(self):
        pass

    def _after_physics_step(self):
        pass

    def _failure_reasons(self, finite, roll, pitch, drift):
        return ((~finite).astype(np.int32) | (2*(np.abs(roll) > np.deg2rad(35)))
                | (4*(np.abs(pitch) > np.deg2rad(45))) | (8*(drift > 3)))

    def _reward(self, *args):
        return spin_reward(*args)

    def _spin_reference(self, elapsed):
        return spin_reference(elapsed, self.episode_steps*self.policy_dt, self.ext,
                              self.direction, self.yaw_target, self.dt, self.recovery_seconds)

    def _assert_floor(self, ids):
        normals = self.geom_xmat[ids, self.floor].reshape(-1, 3, 3)[:, :, 2]
        a = np.deg2rad(self.slope_deg[ids])
        expected = np.column_stack((np.sin(a), np.zeros(len(ids)), np.cos(a)))
        if not np.allclose(normals, expected, atol=1e-8):
            raise RuntimeError("Per-environment slope normals do not match requested terrain")

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
        return self._observation()

    def step_async(self, actions):
        if self._closed or self._actions is not None:
            raise RuntimeError("Environment closed or a step is pending")
        a = np.asarray(actions, dtype=np.float32)
        if a.shape != (self.num_envs, *self.action_space.shape) or not np.isfinite(a).all():
            raise ValueError("Expected finite actions matching the environment action space")
        self._actions = np.clip(a, -1, 1)

    def step_wait(self):
        if self._actions is None:
            raise RuntimeError("Call step_async first")
        actions, self._actions = self._actions, None
        if not self.residual_enabled:
            actions = np.zeros_like(actions)
        self._before_policy_step()
        action_delta = actions-self.last_action
        mask = np.ones(self.num_envs, dtype=bool)
        failed = np.zeros(self.num_envs, dtype=bool)
        reward = np.zeros(self.num_envs)
        density = {}
        rows = np.arange(self.num_envs)
        for substep in range(self.substeps):
            elapsed = self.steps*self.policy_dt+substep*self.dt
            self.yaw_target[:], self.phase[:] = self._spin_reference(elapsed)
            if self.reference_mode == "hold":
                self.yaw_target[:] = 0
                self.phase[:] = 0
            turning = (self.phase == 1) | (self.phase == 2)
            self.inputs[:, 1] = self.yaw_target
            self._prepare_control(elapsed, actions, mask)
            self._fill_frames()
            self.ctrl[:] = self._actuator_control(mask)
            self.batch.step()
            self.physics_steps += 1
            self._after_physics_step()
            q, gyro, _ = self._imu()
            roll, pitch, _, roll_rate, yaw_rate = attitude(q, gyro)
            native_obs = self._native_observation()
            target = self.yaw_target
            speed = np.linalg.norm(self.qvel[:, :2], axis=1)
            drift = np.linalg.norm(self.qpos[:, :2]-self.start_xy, axis=1)
            length_error = np.mean(native_obs[:, 11:13], axis=1)-(LEG_MID-self.ext)
            r, density = self._reward(roll, pitch, roll_rate, yaw_rate, target, speed,
                                    drift, actions, action_delta, length_error)
            reward += r*self.dt
            self.reward_totals["survival"] += self.dt
            for name, value in density.items():
                self.reward_totals[name] += value*self.dt
            finite = np.isfinite(self.qpos).all(axis=1) & np.isfinite(self.qvel).all(axis=1)
            reasons = self._failure_reasons(finite, roll, pitch, drift)
            first = (self.failure_mask == 0) & (reasons != 0)
            self.failure_phase[first], self.failure_time[first] = self.phase[first], elapsed[first]
            self.failure_mask |= reasons
            failed |= reasons != 0
            self.roll_sq += roll**2*self.dt
            self.roll_peak = np.maximum(self.roll_peak, np.abs(roll))
            self.drift_peak = np.maximum(self.drift_peak, drift)
            self.yaw_integral += yaw_rate*self.dt
            self.yaw_reference_integral += target*self.dt
            tracking = turning
            self.spin_seconds += tracking*self.dt
            self.yaw_error_sq += tracking*(yaw_rate-target)**2*self.dt
            stalled = (tracking & (elapsed > 6) & (np.abs(target) > 0.6)
                       & (yaw_rate*target < 0.2*target**2))
            self.stall_seconds = np.where(stalled, self.stall_seconds+self.dt, 0)
            stats = self.phase_stats[rows, self.phase].copy()
            stats[:, 0] += self.dt
            stats[:, 1] += roll**2*self.dt
            stats[:, 2] += pitch**2*self.dt
            stats[:, 3] = np.maximum(stats[:, 3], np.abs(roll))
            stats[:, 4] = np.maximum(stats[:, 4], np.abs(pitch))
            stats[:, 5] += (yaw_rate-target)**2*self.dt
            stats[:, 6] += yaw_rate*self.dt
            stats[:, 7] = np.maximum(stats[:, 7], drift)
            stats[:, 8] = np.maximum(stats[:, 8], speed)
            self.phase_stats[rows, self.phase] = stats
        self.steps += 1
        stalled = self.stall_seconds > 2
        first = (self.failure_mask == 0) & stalled
        self.failure_phase[first], self.failure_time[first] = self.phase[first], self.steps[first]*self.policy_dt
        self.failure_mask |= 16*stalled
        terminated = failed | stalled
        truncated = self.steps >= self.episode_steps
        # Failure cost also charges the remaining survival opportunity: ending
        # early cannot avoid poor tracking/tilt penalties for a higher return.
        terminal = -terminated.astype(float)*(10+self.failure_cost_per_second
                    * np.maximum(0, self.episode_steps-self.steps)*self.policy_dt)
        reward += terminal
        self.reward_totals["terminal"] += terminal
        self.returns += reward
        self.last_action[:] = actions
        self.history[:, :-1] = self.history[:, 1:]
        self.history[:, -1] = self._frame_obs()
        obs = self._observation()
        done = terminated | truncated
        infos = []
        for i in range(self.num_envs):
            info = {
                "roll_deg": float(np.rad2deg(roll[i])),
                "pitch_deg": float(np.rad2deg(pitch[i])),
                "yaw_rate": float(yaw_rate[i]), "yaw_target": float(target[i]),
                "drift_m": float(drift[i]), "speed_mps": float(speed[i]),
                "phase": PHASES[self.phase[i]],
                "base_yaw_target": float(native_obs[i, 26]),
                "TimeLimit.truncated": bool(truncated[i] and not terminated[i]),
            }
            if done[i]:
                seconds = self.steps[i]*self.policy_dt
                info["terminal_observation"] = obs[i].copy()
                info["episode"] = {"r": float(self.returns[i]), "l": int(self.steps[i]),
                                   "t": time.monotonic()-self.start_wall[i]}
                info["spin_metrics"] = {
                    **self._extra_metrics(i),
                    "slope_deg": float(self.slope_deg[i]), "heading": float(self.heading[i]),
                    "extension": float(self.ext[i]), "direction": int(self.direction[i]),
                    "duration_s": float(seconds), "failed": bool(failed[i]),
                    "stalled": bool(stalled[i]),
                    "failure_reasons": [name for bit, name in FAILURES if self.failure_mask[i] & bit],
                    "failure_phase": PHASES[self.failure_phase[i]] if self.failure_phase[i] >= 0 else None,
                    "failure_time_s": float(self.failure_time[i]),
                    "roll_rms_deg": float(np.rad2deg(np.sqrt(self.roll_sq[i]/seconds))),
                    "roll_peak_deg": float(np.rad2deg(self.roll_peak[i])),
                    "yaw_rmse": float(np.sqrt(self.yaw_error_sq[i]/max(self.spin_seconds[i], 1e-9))),
                    "yaw_angle_rad": float(self.yaw_integral[i]),
                    "yaw_reference_angle_rad": float(self.yaw_reference_integral[i]),
                    "yaw_angle_ratio": float(self.yaw_integral[i]/self.yaw_reference_integral[i])
                    if abs(self.yaw_reference_integral[i]) > 1e-6 else 0.0,
                    "max_drift_m": float(self.drift_peak[i]),
                    "reward_totals": {key: float(value[i]) for key, value in self.reward_totals.items()},
                    "phases": self._phase_metrics(i),
                }
            info["reward_terms"] = {key: float(value[i]) for key, value in density.items()}
            infos.append(info)
        if done.any():
            self._reset_ids(np.flatnonzero(done))
            obs[done] = self._observation()[done]
        return obs, (reward*self.training_reward_scale).astype(np.float32), done, infos

    def _phase_metrics(self, i):
        result = {}
        for phase, s in zip(PHASES, self.phase_stats[i]):
            if s[0] > 0:
                result[phase] = {
                    "duration_s": float(s[0]),
                    "roll_rms_deg": float(np.rad2deg(np.sqrt(s[1]/s[0]))),
                    "pitch_rms_deg": float(np.rad2deg(np.sqrt(s[2]/s[0]))),
                    "roll_peak_deg": float(np.rad2deg(s[3])),
                    "pitch_peak_deg": float(np.rad2deg(s[4])),
                    "yaw_rmse": float(np.sqrt(s[5]/s[0])),
                    "yaw_angle_rad": float(s[6]), "max_drift_m": float(s[7]),
                    "max_speed_mps": float(s[8]),
                }
        return result

    def close(self):
        if not self._closed:
            self.control.close()
            self._closed = True
            self._actions = None
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
            raise ValueError("Cannot set shared configuration for a subset")

    def env_method(self, method_name, *args, indices=None, **kwargs):
        if method_name == "render":
            return [None for _ in self._get_indices(indices)]
        raise NotImplementedError(method_name)

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False for _ in self._get_indices(indices)]

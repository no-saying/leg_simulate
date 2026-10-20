"""Interactive deployment of the trained slope-spin policy and native controller."""

import argparse
import ctypes
import hashlib
import json
import time
from pathlib import Path

import glfw
import mujoco
import numpy as np
import torch
from spin_mixed_env import LOCATIONS
from spin_rl_env import EXT_LIMITS, LEG_MID, attitude
from spin_travel_env import TravelSpinEnv
from spin_wrench_env import WrenchSpinEnv
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "logs/rl_training/pitch_compact25_v2_131k_20261018.zip"


class LiveSpinEnv(TravelSpinEnv):
    """Deployment uses the trained actor/allocator without reward or terrain truth."""

    def __init__(self, slope=8, length=0.117, location="ramp", heading=0.7, seed=1000):
        terrain = {"location": location, "handover": "lqr", "offset": 0,
                   "takeover_time": 0, "fade": 0.3, "dropout": False}
        super().__init__(1, 1, cases=[(slope, heading, LEG_MID-length, 1)],
                         terrain_cases=[terrain])
        self.seed(seed)
        self.native = ctypes.CDLL(str(Path(self.control._directory.name)/"controller_0.so"))
        for name, args in (
            ("balance_set_manual", [ctypes.c_float, ctypes.c_float, ctypes.c_int]),
            ("balance_set_leg_extend_manual", [ctypes.c_float]),
            ("robot_control_set_spin_residual", [ctypes.POINTER(ctypes.c_float), ctypes.c_int]),
        ):
            function = getattr(self.native, name)
            function.argtypes, function.restype = args, None
        self._original_spin_command = self.control.spin_command
        self.control.spin_command = self._send_command
        self.driving = False
        self.drive_speed = self.turn_rate = 0.0
        self.spin_rate = self.age = 0.0
        self._rl_target, self._fade_start, self._fade_value = False, 0.0, 0.0
        self.spin_on = True
        self.rl_on = True
        self.lqr_on = True
        self.obs = self.reset()
        self.authority[:] = 0

    def _send_command(self, inputs, mask):
        if not self.driving:
            self._original_spin_command(inputs, mask)
            return
        # Sending an inactive spin reference before every manual command would
        # repeatedly release/re-enter manual mode, resetting its integrators.
        self.native.balance_set_leg_extend_manual(float(inputs[0, 0]))
        residual = np.ascontiguousarray(inputs[0, 2:], dtype=np.float32)
        self.native.robot_control_set_spin_residual(
            residual.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), 1)
        self.native.balance_set_manual(self.drive_speed, self.turn_rate, 1)

    def reset_live(self, *, slope=None, location=None):
        s, h, _, direction = self.cases[0]
        self.cases[0] = (s if slope is None else slope, h, float(self.ext[0]), direction)
        if location is not None:
            self.terrain_cases[0]["location"] = location
        self.driving = False
        self.spin_rate = self.age = 0.0
        self._rl_target, self._fade_start, self._fade_value = False, 0.0, 0.0
        self.obs = self.reset()
        self.authority[:] = 0
        self.last_action[:] = 0
        self.history[:] = self._frame_obs()[:, None, :]
        self.obs = self._observation()

    def advance(self, action, speed=0.0, turn=0.0):
        action = np.asarray(action, np.float32).reshape(1, 5)
        if not np.isfinite(action).all():
            raise FloatingPointError("Nonfinite policy output")
        action = np.clip(action, -1, 1)
        self.drive_speed, self.turn_rate = speed, turn
        self.driving = bool(speed or turn)
        if self.driving:
            self.spin_on = False
        desired = bool(self.spin_on and self.rl_on and not self.driving)
        if desired != self._rl_target:
            self._rl_target, self._fade_start = desired, self.age
            self._fade_value = float(self.authority[0])
        mask = np.ones(1, bool)
        # This policy was trained for stationary spin. Use the verified PID
        # velocity controller for translation; the current LQR holds against W.
        self.lqr_allowed[:] = self.lqr_on and not bool(speed)
        self.control.set_lqr_enabled(self.lqr_allowed)
        self.control.set_active(mask, self.spin_on or self.authority[0] > 0)
        for substep in range(self.substeps):
            blend = np.clip(((LEG_MID-self.ext[0])-0.132)/0.013, 0, 1)
            goal = 1.15*(2.4-1.2*blend)*self.direction[0] if self.spin_on else 0.0
            self.spin_rate += float(np.clip(goal-self.spin_rate, -0.8*self.dt, 0.8*self.dt))
            self.yaw_target[:] = self.turn_rate if self.driving else self.spin_rate
            fade = np.clip((self.age+substep*self.dt-self._fade_start)/0.3, 0, 1)
            self.authority[:] = self._fade_value+(float(desired)-self._fade_value)*fade
            self.inputs[:, 1] = self.spin_rate if not self.driving else 0
            self._apply_residual(action*self.authority[:, None], mask)
            self._fill_frames()
            self.ctrl[:] = self.control.step(mask, self.dt)
            self.batch.step()
            self.physics_steps += 1
        self.steps += 1
        self.age += self.policy_dt
        self.last_action[:] = action
        self.history[:, :-1] = self.history[:, 1:]
        self.history[:, -1] = self._frame_obs()
        self.obs = self._observation()
        if not np.isfinite(self.state).all():
            raise FloatingPointError("Nonfinite simulation state")
        return self.obs

    def status(self):
        q, gyro, _ = self._imu()
        roll, pitch, _, _, yaw_rate = attitude(q, gyro)
        native = self.control.spin_observation()
        return {"time": self.age, "spin": self.spin_on, "rl": self.rl_on,
                "authority": float(self.authority[0]), "lqr_mix": float(native[0, 27]),
                "roll_deg": float(np.rad2deg(roll[0])), "pitch_deg": float(np.rad2deg(pitch[0])),
                "yaw_rad_s": float(yaw_rate[0]), "yaw_target": float(self.yaw_target[0]),
                "lengths_mm": (native[0, 11:13]*1000).tolist(),
                "requested_mm": float((LEG_MID-self.ext[0])*1000),
                "position": self.qpos[0, :3].tolist(),
                "actions": self.last_action[0].tolist(),
                "wheel_ctrl": self.ctrl[0, [2, 5]].tolist()}


class LiveWrenchSpinEnv(WrenchSpinEnv):
    """Interactive exclusive RL/native ownership without training auto-resets."""

    def __init__(self, slope=8, length=.117, location='ramp', heading=.7, seed=1000):
        terrain = dict(location=location, handover='lqr', offset=0,
                       takeover_time=0, fade=.3, dropout=False)
        super().__init__(1, 1, cases=[(slope, heading, LEG_MID-length, 1)],
                         terrain_cases=[terrain], disturbances=False)
        self.seed(seed)
        self.native = ctypes.CDLL(str(Path(self.control._directory.name)/'controller_0.so'))
        for name, arguments in (
            ('balance_set_manual', [ctypes.c_float, ctypes.c_float, ctypes.c_int]),
            ('balance_set_leg_extend_manual', [ctypes.c_float]),
            ('robot_control_set_spin_residual', [ctypes.POINTER(ctypes.c_float), ctypes.c_int]),
        ):
            function = getattr(self.native, name)
            function.argtypes, function.restype = arguments, None
        self._original_spin_command = self.control.spin_command
        self.control.spin_command = self._send_command
        self.driving = False
        self.drive_speed = self.turn_rate = self.spin_rate = self.age = 0.
        self.spin_on = self.rl_on = self.lqr_on = True
        self.obs = self.reset()

    _send_command = LiveSpinEnv._send_command

    def reset_live(self, *, slope=None, location=None):
        s, h, _, direction = self.cases[0]
        self.cases[0] = (s if slope is None else slope, h, float(self.ext[0]), direction)
        if location is not None:
            self.terrain_cases[0]['location'] = location
        self.driving = False
        self.drive_speed = self.turn_rate = self.spin_rate = self.age = 0.
        self.obs = self.reset()

    def advance(self, action, speed=0., turn=0.):
        action = np.asarray(action, np.float32).reshape(1, 6)
        if not np.isfinite(action).all():
            raise FloatingPointError('Nonfinite policy output')
        action = np.clip(action, -1, 1)
        self.drive_speed, self.turn_rate = speed, turn
        self.forward_target[:] = speed
        self.driving = bool(speed or turn)
        if self.driving:
            self.spin_on = False
        desired = bool(self.rl_on and not self.driving)
        self.takeover_time[:] = 0 if desired else np.inf
        self.dropout_time[:] = np.inf
        self.handover[0] = 'lqr' if self.lqr_on and not speed else 'pid'
        mask = np.ones(1, bool)
        for substep in range(self.substeps):
            blend = np.clip(((LEG_MID-self.ext[0])-.132)/.013, 0, 1)
            goal = 1.15*(2.4-1.2*blend)*self.direction[0] if self.spin_on else 0.
            self.spin_rate += float(np.clip(goal-self.spin_rate, -.8*self.dt, .8*self.dt))
            self.yaw_target[:] = self.turn_rate if self.driving else self.spin_rate
            self.inputs[:, 1] = self.spin_rate if not self.driving else 0
            self._prepare_control(np.full(1, self.age+substep*self.dt), action, mask)
            self._fill_frames()
            self.ctrl[:] = self._actuator_control(mask)
            self.batch.step()
            self.physics_steps += 1
            self._after_physics_step()
        self.steps += 1
        self.age += self.policy_dt
        self.last_action[:] = action
        self.history[:, :-1] = self.history[:, 1:]
        self.history[:, -1] = self._frame_obs()
        self.obs = self._observation()
        if not np.isfinite(self.state).all():
            raise FloatingPointError('Nonfinite simulation state')
        return self.obs

    def status(self):
        state = LiveSpinEnv.status(self)
        state['lengths_mm'] = (self._native_observation()[0, 11:13]*1000).tolist()
        state['owner'] = 'RL' if self.rl_owner[0] else 'native'
        state['lqr_mix'] = 0. if self.rl_owner[0] else state['lqr_mix']
        state['native_torque_leak_peak_ctrl'] = float(self.native_leak_peak[0])
        return state


class SpinWindow:
    def __init__(self, env, policy, screenshot=None):
        self.env, self.policy, self.screenshot = env, policy, screenshot
        self.paused, self.camera_mode = False, 0
        self.previous_keys = set()
        self.last_cursor = None
        if not glfw.init():
            raise RuntimeError("GLFW cannot open the desktop display")
        self.window = glfw.create_window(1280, 900, "COD-2026 | Spin Control", None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Cannot create the spin-policy GUI window")
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)
        glfw.show_window(self.window)
        glfw.focus_window(self.window)
        self.data = mujoco.MjData(env.model)
        self.camera = mujoco.MjvCamera()
        self.camera.distance, self.camera.azimuth, self.camera.elevation = 2.2, 145, -25
        self.option = mujoco.MjvOption()
        self.scene = mujoco.MjvScene(env.model, maxgeom=5000)
        self.context = mujoco.MjrContext(env.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        glfw.set_scroll_callback(self.window, self.scroll)

    def scroll(self, window, dx, dy):
        self.camera.distance = float(np.clip(self.camera.distance*np.exp(-0.12*dy), 0.4, 10))

    def keys(self):
        codes = (glfw.KEY_Z, glfw.KEY_T, glfw.KEY_R, glfw.KEY_F, glfw.KEY_P, glfw.KEY_V,
                 glfw.KEY_SPACE, glfw.KEY_ESCAPE, glfw.KEY_LEFT_BRACKET, glfw.KEY_RIGHT_BRACKET,
                 *range(glfw.KEY_1, glfw.KEY_5+1), glfw.KEY_W, glfw.KEY_A, glfw.KEY_S,
                 glfw.KEY_D, glfw.KEY_Q, glfw.KEY_E)
        current = {k for k in codes if glfw.get_key(self.window, k) == glfw.PRESS}
        pressed = current-self.previous_keys
        self.previous_keys = current
        if glfw.KEY_ESCAPE in pressed:
            glfw.set_window_should_close(self.window, True)
        if pressed & {glfw.KEY_Z, glfw.KEY_T}:
            self.env.spin_on = not self.env.spin_on
        if glfw.KEY_F in pressed:
            self.env.rl_on = not self.env.rl_on
        if glfw.KEY_P in pressed:
            self.env.lqr_on = not self.env.lqr_on
        if glfw.KEY_SPACE in pressed:
            self.paused = not self.paused
        if glfw.KEY_V in pressed:
            self.camera_mode = (self.camera_mode+1) % 3
            self.camera.distance = (2.2, 4.0, 5.5)[self.camera_mode]
            self.camera.elevation = (-25, -80, -35)[self.camera_mode]
        if glfw.KEY_R in pressed:
            self.env.reset_live()
        for i, location in enumerate(LOCATIONS):
            if glfw.KEY_1+i in pressed:
                self.env.reset_live(location=location)
        slope = self.env.cases[0][0]
        if glfw.KEY_LEFT_BRACKET in pressed:
            self.env.reset_live(slope=max(0, slope-1))
        if glfw.KEY_RIGHT_BRACKET in pressed:
            self.env.reset_live(slope=min(15, slope+1))
        return current

    def render(self):
        env = self.env
        mujoco.mj_setState(env.model, self.data, env.state[0], mujoco.mjtState.mjSTATE_INTEGRATION)
        mujoco.mj_forward(env.model, self.data)
        self.camera.lookat[:] = env.qpos[0, :3]+[0, 0, 0.08]
        x, y = glfw.get_cursor_pos(self.window)
        if self.last_cursor is not None:
            dx, dy = x-self.last_cursor[0], y-self.last_cursor[1]
            if glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
                self.camera.azimuth -= dx*0.3
                self.camera.elevation = float(np.clip(self.camera.elevation-dy*0.3, -89, -3))
        self.last_cursor = x, y
        width, height = glfw.get_framebuffer_size(self.window)
        if not width or not height:
            return
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjv_updateScene(env.model, self.data, self.option, None, self.camera,
                             mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(viewport, self.scene, self.context)
        state = env.status()
        mode = "SPIN" if env.spin_on else "DRIVE" if env.driving else "HOLD"
        labels = "Policy\nMode\nRL authority\nLeg request\nLeg L / R\nRoll / Pitch\nYaw actual / ref\nSlope\nLQR mix"
        policy_name = 'Exclusive RL' if isinstance(env, LiveWrenchSpinEnv) else 'Legacy residual'
        values = (f"{policy_name} {'ON' if env.rl_on else 'OFF'}\n{mode}{' PAUSED' if self.paused else ''}\n"
                  f"{state['authority']:.2f}\n{state['requested_mm']:.0f} mm\n"
                  f"{state['lengths_mm'][0]:.0f} / {state['lengths_mm'][1]:.0f} mm\n"
                  f"{state['roll_deg']:+.1f} / {state['pitch_deg']:+.1f} deg\n"
                  f"{state['yaw_rad_s']:+.2f} / {state['yaw_target']:+.2f} rad/s\n"
                  f"{env.cases[0][0]:.0f} deg\n{state['lqr_mix']:.2f}")
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                           viewport, labels, values, self.context)
        if self.screenshot and env.age >= 2:
            from PIL import Image
            pixels = np.zeros((height, width, 3), np.uint8)
            mujoco.mjr_readPixels(pixels, None, viewport, self.context)
            self.screenshot.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(np.flipud(pixels)).save(self.screenshot)
            print("SCREENSHOT", self.screenshot, "pixel_std", float(pixels.std()), flush=True)
            self.screenshot = None
        glfw.swap_buffers(self.window)

    def run(self):
        previous, accumulator, log_at = time.monotonic(), 0.0, 0.0
        try:
            while not glfw.window_should_close(self.window):
                glfw.poll_events()
                keys = self.keys()
                now = time.monotonic()
                accumulator += min(now-previous, 0.1) if not self.paused else 0
                previous = now
                while accumulator >= self.env.policy_dt:
                    direction = int(glfw.KEY_Q in keys)-int(glfw.KEY_E in keys)
                    self.env.ext[:] = np.clip(self.env.ext+direction*0.035*self.env.policy_dt,
                                              *EXT_LIMITS)
                    speed = 1.2*(int(glfw.KEY_S in keys)-int(glfw.KEY_W in keys))
                    turn = 1.2*(int(glfw.KEY_A in keys)-int(glfw.KEY_D in keys))
                    action, _ = self.policy.predict(self.env.obs, deterministic=True)
                    self.env.advance(action, speed, turn)
                    accumulator -= self.env.policy_dt
                self.render()
                if now >= log_at:
                    print(json.dumps(self.env.status()), flush=True)
                    log_at = now+1
                time.sleep(0.001)
        finally:
            self.context.free()
            glfw.destroy_window(self.window)
            glfw.terminate()


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--legacy-residual', action='store_true',
                        help='Explicitly enable the archived five-output residual GUI')
    parser.add_argument("--slope", type=float, default=8)
    parser.add_argument("--length", type=float, default=0.117)
    parser.add_argument("--location", choices=LOCATIONS, default="ramp")
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--headless-seconds", type=float, default=0)
    args = parser.parse_args()
    if not 0 <= args.slope <= 15 or not 0.047 <= args.length <= 0.210:
        parser.error("Slope must be 0..15 degrees and length 0.047..0.210 meters")
    torch.set_num_threads(1)
    policy = PPO.load(args.model, device="cpu")
    exclusive = policy.action_space.shape == (6,)
    if not exclusive and not args.legacy_residual:
        parser.error('Five-output residual checkpoints are archived; select an exclusive six-output model')
    env_type = LiveWrenchSpinEnv if exclusive else LiveSpinEnv
    env = env_type(args.slope, args.length, args.location)
    try:
        if policy.observation_space.shape != env.observation_space.shape:
            raise ValueError("Policy observation contract differs from the live controller; "
                             "exclusive control now requires the compact 25-value frame checkpoint")
        contract = json.loads(args.model.with_suffix(".json").read_text())
        if contract["provenance"]["controller_build"] != env.control.build_id:
            raise ValueError("Native controller differs from the trained checkpoint")
        if exclusive and contract.get('arguments', {}).get('reference_mode') == 'hold':
            env.spin_on = False
        print("MODEL", args.model.resolve(), "SHA256",
              hashlib.sha256(args.model.read_bytes()).hexdigest(), flush=True)
        print(f"POLICY actor_input={getattr(policy.policy, 'actor_size', env.observation_space.shape[0])} "
              f"output={env.action_space.shape[0]} frequency=50Hz native_controller="+env.control.build_id, flush=True)
        print("Z/T spin; WASD drive; Q/E height; 1..5 terrain location; [/] slope; "
              "F RL; P PID/LQR; R reset; V camera; SPACE pause; ESC close", flush=True)
        if args.headless_seconds:
            for _ in range(round(args.headless_seconds/env.policy_dt)):
                action, _ = policy.predict(env.obs, deterministic=True)
                env.advance(action)
            print(json.dumps(env.status()), flush=True)
        else:
            SpinWindow(env, policy, args.screenshot).run()
    finally:
        env.close()


if __name__ == "__main__":
    main()

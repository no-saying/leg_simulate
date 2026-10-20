"""Live deployment retains the trained control timing and manual drive output."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_spin_rl_gui import LiveSpinEnv, LiveWrenchSpinEnv
from spin_wrench_env import WrenchSpinEnv, ACTOR_SIZE
from spin_travel_env import TravelSpinEnv


def test_live_policy_path_matches_training_observations_and_physics():
    live = LiveSpinEnv()
    reference = TravelSpinEnv(1, 1, cases=[(8, 0.7, 0, 1)], terrain_cases=[
        {"location": "ramp", "handover": "lqr", "offset": 0,
         "takeover_time": 1.0, "fade": 0.3, "dropout": False}])
    try:
        reference.seed(1000)
        reference.reset()
        np.testing.assert_allclose(live.obs, reference._observation(), atol=1e-7)
        for step in range(200):
            live.spin_on = step >= 50
            action = np.array([[0.1, 0.2, -0.1, 0.05, -0.05]], np.float32)
            live.advance(action)
            reference.step(action)
            np.testing.assert_allclose(live.state, reference.state, atol=1e-8, rtol=1e-8)
            np.testing.assert_allclose(live.obs, reference._observation(), atol=1e-6, rtol=1e-6)
    finally:
        live.close()
        reference.close()


@pytest.mark.parametrize("speed,turn", [(-0.6, 0), (0.6, 0), (0, 1.2), (0, -1.2)])
def test_manual_controls_produce_translation_or_rotation(speed, turn):
    env = LiveSpinEnv(slope=0, location="lower_flat")
    try:
        env.spin_on = False
        start = env.qpos[0, :2].copy()
        angle = 0.0
        for _ in range(250):
            env.advance(np.zeros((1, 5), np.float32), speed, turn)
            angle += env.status()["yaw_rad_s"]*env.policy_dt
        if speed:
            assert np.linalg.norm(env.qpos[0, :2]-start) > 0.15
        else:
            assert angle*np.sign(turn) > 0.3
        assert np.abs(env.status()["roll_deg"]) < 20
    finally:
        env.close()


def test_exclusive_live_path_matches_full_control_training_actor_and_physics():
    live = LiveWrenchSpinEnv(slope=0, location='lower_flat')
    reference = WrenchSpinEnv(1, 1, cases=[(0, .7, 0, 1)], terrain_cases=[dict(
        location='lower_flat', handover='lqr', offset=0, takeover_time=0, fade=.3, dropout=False)],
        reference_mode='hold', disturbances=False)
    try:
        reference.seed(1000)
        reference.reset()
        live.spin_on = False
        action = np.array([[0, .38, 0, .38, 0, 0]], np.float32)
        for _ in range(10):
            live.advance(action)
            reference.step(action)
            np.testing.assert_allclose(live.state, reference.state, atol=1e-8, rtol=1e-8)
            np.testing.assert_allclose(live.obs[:, :ACTOR_SIZE], reference._observation()[:, :ACTOR_SIZE],
                                       atol=1e-6, rtol=1e-6)
        assert live.status()['owner'] == 'RL'
        assert live.native_update_seconds[0] == 0
    finally:
        live.close()
        reference.close()


def test_exclusive_gui_toggle_never_steps_native_during_rl():
    env = LiveWrenchSpinEnv(slope=0, location='lower_flat')
    try:
        original = env.control.step
        called = []
        def step(mask, dt):
            assert not (mask & env.rl_owner).any()
            called.append(mask.copy())
            return original(mask, dt)
        env.control.step = step
        env.spin_on = False
        action = np.array([[0, .38, 0, .38, 0, 0]], np.float32)
        env.advance(action)
        assert env.status()['owner'] == 'RL' and not called
        env.rl_on = False
        env.advance(action)
        assert env.status()['owner'] == 'native' and len(called) == env.substeps
        env.rl_on = True
        env.advance(action)
        assert env.status()['owner'] == 'RL' and len(called) == env.substeps
        assert env.status()['native_torque_leak_peak_ctrl'] == 0
    finally:
        env.close()

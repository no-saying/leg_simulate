"""Physical parity, controller isolation and SB3 episode-contract checks."""

import ctypes
from pathlib import Path
import shutil
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jump_rl_env import JumpRLEnv
from jump_rl_batch_env import JumpRLBatchEnv
from rl_batch_control import BatchControl, build_bridges


@pytest.fixture
def scalar_factory(tmp_path):
    bridge, _ = build_bridges()
    count = 0

    def make():
        nonlocal count
        path = tmp_path / f"reference_{count}.so"
        count += 1
        shutil.copyfile(bridge, path)
        return JumpRLEnv(bridge_path=path)

    return make


def test_ground_rollout_matches_scalar(scalar_factory):
    env = JumpRLBatchEnv(2, 2)
    try:
        env.seed(123)
        obs = env.reset()
        references = [scalar_factory(), scalar_factory()]
        for i, ref in enumerate(references):
            expected, _ = ref.reset(seed=123 + i)
            np.testing.assert_allclose(obs[i], expected, atol=2e-5, rtol=2e-6)
            ref.expert_steps = 0
        env.expert_steps[:] = 0
        rng = np.random.default_rng(10)
        for _ in range(50):
            actions = rng.uniform(-0.2, 0.2, (2, 5)).astype(np.float32)
            obs, reward, done, _ = env.step(actions)
            for i, ref in enumerate(references):
                expected, r, terminated, truncated, _ = ref.step(actions[i])
                assert not done[i] and not terminated and not truncated
                np.testing.assert_allclose(obs[i], expected, atol=2e-4, rtol=2e-5)
                np.testing.assert_allclose(reward[i], r, atol=2e-4, rtol=2e-5)
    finally:
        env.close()


def test_controller_state_and_subset_reset_are_isolated(scalar_factory):
    batch = BatchControl(3, 2)
    references = [scalar_factory() for _ in range(3)]
    try:
        batch.reset([True] * 3)
        for ref in references:
            ref._bridge.ident_reset()
        for step in range(60):
            if step == 30:
                batch.reset([True, False, False])
                references[0]._bridge.ident_reset()
            for i, ref in enumerate(references):
                frame = ref._fill_frame()
                frame.pitch = (i - 1) * 0.03 + step * 0.0001
                frame.left_wheel_vel = i + step * 0.01
                frame.sim_time = step * 0.001
                frame.step_count = step
                batch._frames[i] = frame
            actual = batch.step([True] * 3, 0.001).copy()
            for i, ref in enumerate(references):
                expected = (ctypes.c_float * 6)()
                ref._bridge.ident_robot_step(ctypes.byref(batch._frames[i]), 0.001, expected)
                np.testing.assert_array_equal(actual[i], np.ctypeslib.as_array(expected))
    finally:
        batch.close()


def test_partial_autoreset_preserves_other_simulation():
    env, reference = JumpRLBatchEnv(2, 2), JumpRLBatchEnv(1, 1)
    try:
        env.seed(300)
        reference.seed(301)
        env.reset()
        reference.reset()
        env.expert_steps[:] = reference.expert_steps[:] = 0
        env.step_count[0] = env.episode_steps - 1
        env.max_height[0] = 0.03
        actions = np.zeros((2, 5), dtype=np.float32)
        expected, expected_r, expected_done, _ = reference.step(actions[1:])
        obs, r, done, infos = env.step(actions)
        assert done[0] and not done[1] and not expected_done[0]
        assert infos[0]["TimeLimit.truncated"]
        assert infos[0]["episode"]["l"] == env.episode_steps
        assert infos[0]["terminal_observation"][19] == pytest.approx(3.0)
        assert obs[0, 19] == 0
        np.testing.assert_array_equal(obs[1], expected[0])
        np.testing.assert_array_equal(r[1], expected_r[0])
        np.testing.assert_array_equal(env.state[1], reference.state[0])
        saved = env.state[1].copy()
        env._reset_ids(np.array([0], dtype=np.int32))
        np.testing.assert_array_equal(env.state[1], saved)
    finally:
        env.close()
        reference.close()


def test_reward_matches_scalar_across_phases(scalar_factory):
    env = JumpRLBatchEnv(4, 2)
    try:
        cases = [
            (0, 0, -0.01, 0, 1, 0),
            (5, 8, 0.04, 1, 400, 0.01),
            (10, 32, 0.01, 0, 700, 0.15),
            (52, 2, 0.03, 0, 1000, 0.10),
        ]
        obs = np.zeros((4, 34), dtype=np.float32)
        expected = []
        for i, (pitch, roll, height, air, step, maximum) in enumerate(cases):
            ref = scalar_factory()
            obs[i, [16, 17, 18, 33]] = [pitch, roll, height, air]
            obs[i, [2, 3, 5, 6, 20, 21]] = [0.2, -0.3, 0.10, 0.09, 0.4, -0.1]
            ref.max_height, ref.step_count = maximum, step
            ref.data.actuator_force[:] = [3, -4, 0, 5, -6, 0]
            env.max_height[i], env.step_count[i] = maximum, step
            env.forces[i] = ref.data.actuator_force
            expected.append(ref._rew(obs[i]))
        np.testing.assert_allclose(env._reward(obs), expected, rtol=1e-6, atol=1e-5)
        np.testing.assert_array_equal(env.fallen, [False, False, True, True])
        np.testing.assert_array_equal(env.side_flip, [False, False, True, False])
    finally:
        env.close()


def test_expert_and_airborne_paths_match_scalar(scalar_factory):
    env = JumpRLBatchEnv(1, 1)
    ref = scalar_factory()
    try:
        env.seed(18)
        env.reset()
        ref.reset(seed=18)
        env.expert_steps[:] = ref.expert_steps = 1000
        env.step_count[:] = ref.step_count = 75
        env.qpos[0, 2] = ref.data.qpos[2] = 0.14
        env.batch.forward()
        import mujoco

        mujoco.mj_forward(ref.model, ref.data)
        action = np.ones(5, dtype=np.float32)
        obs, r, done, _ = env.step(action[None])
        expected, expected_r, terminated, truncated, _ = ref.step(action)
        assert not done[0] and not terminated and not truncated
        np.testing.assert_allclose(obs[0], expected, atol=2e-4, rtol=2e-5)
        np.testing.assert_allclose(r[0], expected_r, atol=2e-4)
        np.testing.assert_array_equal(env.ctrl[0, [2, 5]], 0)
    finally:
        env.close()

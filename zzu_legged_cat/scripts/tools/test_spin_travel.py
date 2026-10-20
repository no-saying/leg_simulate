"""Directional travel, sensor-only actor, contact truth and V4 reward checks."""

import sys
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spin_mixed_env import MixedTerrainSpinEnv
from spin_travel_env import (
    COMMON_ACCEL_MAX,
    COMMON_OFFSET_MAX,
    COMMON_RATE_MAX,
    LENGTH_MAX,
    LENGTH_MIN,
    LENGTH_SPAN,
    TravelSpinEnv,
    differential_action,
    length_margins,
    travel_reward,
)
from train_spin_rl import TravelCurriculumCallback


def test_directional_margins_and_mirrored_differential():
    margins = length_margins([[LENGTH_MIN, LENGTH_MAX], [LENGTH_MAX, LENGTH_MIN]])
    np.testing.assert_allclose(margins[0], [0, LENGTH_SPAN, LENGTH_SPAN, 0, 0, 0, LENGTH_SPAN, 0])
    np.testing.assert_allclose(margins[1], [LENGTH_SPAN, 0, 0, LENGTH_SPAN, 0, 0, 0, LENGTH_SPAN])
    outside = length_margins([[LENGTH_MIN-0.002, LENGTH_MAX+0.003]])
    assert outside[0, 0] == pytest.approx(-0.002)
    assert outside[0, 3] == pytest.approx(-0.003)
    poses = np.array([[LENGTH_MIN, LENGTH_MAX], [LENGTH_MAX, LENGTH_MIN]])
    np.testing.assert_array_equal(differential_action(np.ones(2), poses), [1, 0])
    np.testing.assert_array_equal(differential_action(-np.ones(2), poses), [0, -1])


def reward(**kwargs):
    inputs = {
        "roll": np.zeros(1), "pitch": np.zeros(1), "roll_rate": np.zeros(1),
        "yaw_rate": np.array([2.]), "yaw_target": np.array([2.]),
        "speed": np.zeros(1), "drift": np.zeros(1), "actions": np.zeros((1, 5)),
        "action_delta": np.zeros((1, 5)), "length_error": np.zeros(1),
        "lengths": np.full((1, 2), 0.117),
        "contacts": np.array([[[20, 0, 0, 1], [20, 0, 0, 1]]], dtype=float),
        "common_rate": np.zeros(1), "robot_weight": 40,
    }
    for key, value in kwargs.items():
        inputs[key] = np.asarray(value) if key != "robot_weight" else value
    return travel_reward(**inputs)


def test_reward_roll_priority_travel_contact_and_progress():
    base, terms = reward()
    assert base[0]-reward(roll=[np.deg2rad(3)])[0][0] == pytest.approx(5)
    assert base[0]-reward(pitch=[np.deg2rad(8)])[0][0] == pytest.approx(0.25)
    assert terms["contact_slip"][0] == 0
    assert reward(drift=[0.3])[0][0] < base[0]-0.99
    assert reward(drift=[0.6])[0][0] < reward(drift=[0.3])[0][0]-1.99
    assert reward(drift=[2])[0][0] < reward(drift=[1])[0][0]-6
    assert reward(yaw_rate=[2])[0][0] > reward(yaw_rate=[1])[0][0] > reward(yaw_rate=[0])[0][0]
    assert reward(yaw_rate=[-1])[0][0] < reward(yaw_rate=[0])[0][0]
    assert reward(lengths=[[LENGTH_MIN, LENGTH_MIN]])[1]["leg_reserve"][0] < 0
    assert reward(lengths=[[LENGTH_MAX, LENGTH_MAX]])[1]["leg_reserve"][0] < 0
    # Moving inward on either end earns more remaining-travel reward.
    for endpoint, inward in ((LENGTH_MIN, LENGTH_MIN+0.020), (LENGTH_MAX, LENGTH_MAX-0.020)):
        assert reward(lengths=[[inward]*2])[0][0] > reward(lengths=[[endpoint]*2])[0][0]
    slip = [[[20, 1, 0, 1], [20, 1, 0, 1]]]
    assert reward(contacts=slip)[1]["contact_slip"][0] == pytest.approx(-2.4)
    # The measured support-height difference increases required reserve.
    different_heights = [[[20, 0, 0, 1], [20, 0, 0.04, 1]]]
    assert reward(lengths=[[0.07]*2], contacts=different_heights)[1]["leg_reserve"][0] < 0
    left_high = [[[20, 0, 0.04, 1], [20, 0, 0, 1]]]
    correct = reward(lengths=[[0.10, 0.14]], contacts=left_high)[1]["support_geometry"][0]
    wrong = reward(lengths=[[0.14, 0.10]], contacts=left_high)[1]["support_geometry"][0]
    assert correct == pytest.approx(0, abs=1e-12) and wrong < -0.79
    growth = reward(drift=[0.2], previous_peak=[0.1], dt=0.02)[1]["drift_peak"][0]
    assert growth*0.02 == pytest.approx(-4*0.1/0.3)
    assert reward(drift=[0.1], previous_peak=[0.2])[1]["drift_peak"][0] == 0


def test_curriculum_expands_real_lengths_without_changing_active_targets():
    env = SimpleNamespace(ext=np.array([0.070]))
    callback = TravelCurriculumCallback(env, 8)
    for steps, expected, slope in ((0, (0.090, 0.150), 3),
                                    (65536, (0.070, 0.180), 5),
                                    (131072, (0.047, 0.210), 8)):
        callback.apply(steps)
        np.testing.assert_allclose([0.117-env.leg_ext_range[1], 0.117-env.leg_ext_range[0]], expected)
        assert env.max_slope_deg == slope
        np.testing.assert_array_equal(env.ext, [0.070])


def test_common_reference_escape_direction_and_zero_margin_differential():
    env = TravelSpinEnv(2, 2, cases=[(0, 0, 0.070, 1), (0, 0, -0.093, 1)],
                        terrain_cases=[{"location": "lower_flat", "handover": "pid",
                                        "takeover_time": 0.01, "fade": 0.1}]*2)
    try:
        env.reset()
        actions = np.zeros((2, 5), np.float32)
        actions[:, 1] = [1, -1]
        previous_offset, previous_rate = env.common_offset.copy(), env.common_rate.copy()
        for _ in range(2000):
            env._apply_residual(actions, np.ones(2, bool))
            assert np.max(np.abs(env.common_offset-previous_offset)) <= COMMON_RATE_MAX*env.dt+1e-10
            assert np.max(np.abs(env.common_rate)) <= COMMON_RATE_MAX+1e-10
            if _ < 1000:
                assert np.max(np.abs(env.common_rate-previous_rate)) <= COMMON_ACCEL_MAX*env.dt+1e-10
            previous_offset, previous_rate = env.common_offset.copy(), env.common_rate.copy()
        assert 0.030 < env.common_offset[0] <= COMMON_OFFSET_MAX
        assert -COMMON_OFFSET_MAX <= env.common_offset[1] < -0.030
        assert env.inputs[0, 0] < 0.040  # Positive physical offset reduces native retraction.
        assert env.inputs[1, 0] > -0.063
        np.testing.assert_array_equal(env.inputs[:, 3], 0)  # No duplicate common residual.
    finally:
        env.close()


def test_actor_contract_no_contact_truth_and_zero_policy_preserves_v3_physics():
    kwargs = {"cases": [(5, 1.57, 0.070, 1)], "terrain_cases": [
        {"location": "toe", "handover": "pid_to_lqr", "offset": 0,
         "takeover_time": 0.8, "fade": 0.3, "switch_lead": 0.1, "dropout": False}]}
    env, reference = TravelSpinEnv(1, 1, **kwargs), MixedTerrainSpinEnv(1, 1, **kwargs)
    try:
        env.seed(66)
        reference.seed(66)
        obs, _ = env.reset(), reference.reset()
        assert obs.shape == (1, 416) and obs.dtype == np.float32
        np.testing.assert_array_equal(obs.reshape(1, 8, 52)[:, 0], obs.reshape(1, 8, 52)[:, -1])
        frame = env._frame_obs().copy()
        saved_contact = env.contact_values.copy()
        env.contact_values[:] = 100
        env.qpos[:, :3] += 100
        env.qvel[:, :3] += 20
        np.testing.assert_array_equal(env._frame_obs(), frame)
        # Restore integration state after the deliberate actor-leakage probe.
        env.qpos[:] = reference.qpos
        env.qvel[:] = reference.qvel
        env.contact_values[:] = saved_contact
        for _ in range(100):
            obs, rewards, done, _ = env.step(np.zeros((1, 5), np.float32))
            reference.step(np.zeros((1, 5), np.float32))
            assert not done.any() and np.isfinite(rewards).all()
        np.testing.assert_allclose(env.state, reference.state, atol=1e-10, rtol=1e-10)
        np.testing.assert_allclose(obs.reshape(1, 8, 52)[:, -1, 43:51],
                                   length_margins(env.control.spin_observation()[:, 11:13])/LENGTH_SPAN,
                                   atol=1e-6)
        before, native = env.state.copy(), env.control.spin_observation().copy()
        env.contacts.query(env.state)
        np.testing.assert_array_equal(env.state, before)
        np.testing.assert_array_equal(env.control.spin_observation(), native)
    finally:
        reference.close()
        env.close()


def test_contact_query_matches_native_force_and_contact_point_jacobian():
    env = TravelSpinEnv(1, 1, cases=[(0, 0, 0, 1)],
                        terrain_cases=[{"location": "lower_flat", "handover": "pid"}])
    try:
        env.reset()
        for _ in range(20):
            env.step(np.zeros((1, 5), np.float32))
        # Give both wheel rotation and body motion to the independent contact query.
        env.qvel[0, env.va[4:6]] = [-3, -2]
        env.qvel[0, :3] = [0.08, 0.04, 0]
        env.qvel[0, 3:6] = [0.1, 0.2, 0.3]
        result = env.contacts.query(env.state)[0].copy()
        model, data = env.model, mujoco.MjData(env.model)
        mujoco.mj_setState(model, data, env.state[0], mujoco.mjtState.mjSTATE_INTEGRATION)
        mujoco.mj_forward(model, data)
        expected = np.zeros((2, 4))
        jac = np.zeros((3, model.nv))
        for index, contact in enumerate(data.contact):
            for side, name in enumerate(("Left_Wheel_link", "Right_Wheel_link")):
                body = model.body(name).id
                if body not in model.geom_bodyid[contact.geom]:
                    continue
                if env.floor not in contact.geom:
                    continue
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, index, force)
                if force[0] <= 1e-6:
                    continue
                mujoco.mj_jac(model, data, jac, None, contact.pos, body)
                velocity = jac@data.qvel
                normal = contact.frame[:3]
                tangent = velocity-normal*(velocity@normal)
                expected[side] += [force[0], force[0]*np.linalg.norm(tangent),
                                   force[0]*contact.pos[2], 1]
        assert expected[:, 0].sum() > 1
        np.divide(expected[:, 1:3], expected[:, 0, None], out=expected[:, 1:3],
                   where=expected[:, 0, None] > 0)
        np.testing.assert_allclose(result, expected, atol=1e-8)
    finally:
        env.close()

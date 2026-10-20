"""Exercise actual finite terrain geometry and native controller handovers."""

import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rl_batch_control import BatchControl
from spin_mixed_env import HANDOVERS, LOCATIONS, MixedTerrainSpinEnv, validation_cases


def test_ramp_ray_heights_boundary_supports_and_reset_isolation():
    env = MixedTerrainSpinEnv(2, 2, cases=[(8, np.pi/2, 0, 1)]*2,
                              terrain_cases=[{"location": p, "handover": "lqr", "offset": 0}
                                             for p in ("toe", "crest")])
    try:
        env.seed(60)
        obs = env.reset()
        assert obs.shape == (2, 344)
        np.testing.assert_array_equal(obs.reshape(2, 8, 43)[:, 0],
                                      obs.reshape(2, 8, 43)[:, -1])
        # Ray only against the three support surfaces, using the real model/state.
        model = mujoco.MjModel.from_xml_path(env.provenance["model_xml"])
        model.geom_group[:] = 5
        for name in ("floor", "spin_ramp", "spin_upper_flat"):
            model.geom_group[model.geom(name).id] = 0
        data = mujoco.MjData(model)
        for i in range(2):
            mujoco.mj_setState(model, data, env.state[i], mujoco.mjtState.mjSTATE_INTEGRATION)
            mujoco.mj_forward(model, data)
            length = env.ramp_length[i]
            for x in (-0.3, -0.001, 0.001, length/2, length-0.001, length+0.001, length+1):
                point = np.array([x, 1.5, 3.0])
                distance = mujoco.mj_ray(model, data, point, np.array([0., 0., -1.]),
                                        np.array([1, 0, 0, 0, 0, 0], np.uint8), 1, -1,
                                        np.zeros(1, np.int32))
                assert 3-distance == pytest.approx(env.surface_height(i, x, 1.5), abs=1e-8)
            assert len(set(env.wheel_zones(i))) == 2
            assert abs(np.diff(env.initial_wheel_heights[i])[0]) > 0.005
        state, history = env.state[1].copy(), env.history[1].copy()
        native = env.control.spin_observation()[1].copy()
        env._reset_ids([0])
        np.testing.assert_array_equal(env.state[1], state)
        np.testing.assert_array_equal(env.history[1], history)
        np.testing.assert_array_equal(env.control.spin_observation()[1], native)
        original = env._frame_obs().copy()
        env.qpos[:, :3] += 100
        env.qvel[:, :3] += 20
        env.slope_deg[:] = 19
        env.initial_wheel_heights[:] = 50
        np.testing.assert_array_equal(env._frame_obs(), original)
    finally:
        env.close()


def test_native_warmup_uses_identical_mocap_terrain_to_batch():
    env = MixedTerrainSpinEnv(2, 2, cases=[(8, 1.5, 0, 1), (3, 0.5, 0.04, -1)],
                              terrain_cases=[{"location": p, "handover": "lqr"}
                                             for p in ("toe", "ramp")])
    reference = BatchControl(2, 2)
    try:
        env.reset()
        states, sensors, forces = env.state.copy(), env.sensordata.copy(), env.actuator_force.copy()
        reference.reset([1, 1])
        env.control.reset([1, 1])
        reference.spin_command(env.inputs, [1, 1])
        env.control.spin_command(env.inputs, [1, 1])
        reference.warmup(env.model, states, np.ones(2, np.int32), [1, 1], sensors, forces,
                         terrain=(env.floor_quat[:, env.floor], env.floor_friction[:, env.floor]))
        env.batch.forward()
        env._fill_frames()
        env.ctrl[:] = env.control.step([1, 1], env.dt)
        env.batch.step()
        np.testing.assert_allclose(env.state, states, atol=2e-6, rtol=2e-6)
        np.testing.assert_allclose(env.actuator_force, forces, atol=2e-5)
    finally:
        reference.close()
        env.close()


def test_real_pid_lqr_and_both_blends_at_policy_activation():
    env = MixedTerrainSpinEnv(4, 4, cases=[(5, 0, 0, 1)]*4,
                              terrain_cases=[{"location": "ramp", "handover": h,
                                              "takeover_time": 1.2, "fade": 0.3,
                                              "switch_lead": 0.1, "dropout": False}
                                             for h in HANDOVERS])
    try:
        env.seed(50)
        env.reset()
        actions = np.ones((4, 5), np.float32)*0.2
        for _ in range(60):
            _, _, done, _ = env.step(actions)
            assert not done.any()
        assert not env.authority.any()
        np.testing.assert_array_equal(env.control.spin_actions(), 0)
        observation, _, done, _ = env.step(actions)
        assert not done.any()
        np.testing.assert_allclose(env.actual_takeover_mix, [0, 1, 0.2525, 0.7475], atol=0.005)
        assert np.all((env.authority > 0) & (env.authority < 0.07))
        filtered = env.control.spin_actions()
        assert np.all(filtered > 0) and np.all(filtered < actions*env.authority[:, None])
        np.testing.assert_allclose(observation.reshape(4, 8, 43)[:, -1, 41], env.authority)
        # An outage is faded down and up; zero authority passes zero commands.
        env.dropout_time[:] = 2
        env.dropout_length[:] = 0.5
        for now, alpha in ((1.6, 1), (2.15, 0.5), (2.4, 0), (2.95, 0.5), (3.2, 1)):
            env._prepare_control(np.full(4, now), actions, np.ones(4, bool))
            np.testing.assert_allclose(env.authority, alpha, atol=1e-8)
            np.testing.assert_allclose(env.inputs[:, 2:], actions*alpha)
    finally:
        env.close()


def test_validation_grid_covers_modes_lengths_directions_and_seam_offsets():
    physics, terrain = validation_cases(slopes=(3, 8), headings=(0, np.pi/2),
                                         boundary_offsets=(-0.12, 0, 0.12))
    assert len(physics) == len(terrain) == 864
    assert {p[2] for p in physics} == {-0.093, 0, 0.070}
    assert {p[3] for p in physics} == {-1, 1}
    assert {t["location"] for t in terrain} == set(LOCATIONS)
    assert {t["handover"] for t in terrain} == set(HANDOVERS)
    for location in ("toe", "crest"):
        assert {t["offset"] for t in terrain if t["location"] == location} == {-0.12, 0, 0.12}

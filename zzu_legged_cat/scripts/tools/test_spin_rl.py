"""Check the sensor/action contract, terrain, actual outputs and task semantics."""

import ctypes
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rl_batch_control import BatchControl
from spin_rl_env import (
    OBS_SCALE,
    SpinRLBatchEnv,
    attitude,
    euler_rates,
    spin_reference,
    spin_reward,
)


def make_env(n=2, **kwargs):
    return SpinRLBatchEnv(n, min(n, 2), fixed_slope_deg=0, fixed_heading=0,
                          fixed_extension=0, fixed_direction=1, **kwargs)


def test_shapes_history_and_physics_clock():
    env = make_env()
    try:
        env.seed(10)
        obs = env.reset()
        assert obs.shape == (2, 328) and obs.dtype == np.float32
        history = obs.reshape(2, 8, 41)
        np.testing.assert_array_equal(history[:, 0], history[:, 7])
        before = env.physics_steps.copy()
        action = np.full((2, 5), 0.25, dtype=np.float32)
        after, _, done, _ = env.step(action)
        assert not done.any() and np.isfinite(after).all()
        np.testing.assert_array_equal(env.physics_steps, before+20)
        np.testing.assert_array_equal(after.reshape(2, 8, 41)[:, -1, 28:33], action)
        np.testing.assert_array_equal(after.reshape(2, 8, 41)[:, :-1], history[:, 1:])
        filtered = after.reshape(2, 8, 41)[:, -1, 36:41]
        np.testing.assert_allclose(filtered, env.control.spin_actions())
        assert np.all(filtered > 0) and np.all(filtered < action)
        with pytest.raises(ValueError):
            env.step_async(np.full((2, 5), np.nan))
    finally:
        env.close()


def test_hold_diagnostic_keeps_zero_reference_after_spin_start():
    env = make_env(1, reference_mode="hold", pid_only=True)
    try:
        env.reset()
        env.steps[:] = 400
        _, _, done, info = env.step(np.zeros((1, 5), dtype=np.float32))
        assert not done.any()
        assert env.yaw_target[0] == 0
        assert env.inputs[0, 1] == 0
        assert info[0]["phase"] == "hold"
        assert env.provenance["reference_mode"] == "hold"
    finally:
        env.close()


def test_roll_integral_stops_winding_into_unavailable_travel():
    env = make_env(1, pid_only=True)
    try:
        env.reset()
        env._fill_frames()
        env.control.reset([1])
        env.control.spin_command(env.inputs, [1])
        env.control.frames["roll"] = -0.02
        env.control.frames["roll_rate"] = 0
        library = ctypes.CDLL(str(Path(env.control._directory.name)/"controller_0.so"))
        limit = library.balance_set_roll_allocation_error
        limit.argtypes, limit.restype = [ctypes.c_float], None
        side = library.balance_get_leg_side_extend
        side.argtypes, side.restype = [], ctypes.c_float
        # Replay a constant supported pose, reporting a blocked positive output.
        for _ in range(2000):
            limit(0.01)
            env.control.step([1], env.dt)
        blocked = side()
        assert blocked == pytest.approx(0.30*0.02, abs=2e-5)
        for _ in range(1000):
            limit(0)
            env.control.step([1], env.dt)
        assert side()-blocked == pytest.approx(0.01*0.02, abs=2e-5)
    finally:
        env.close()


def test_terrain_normals_and_native_warmup_match_mjbatch():
    cases = [(5, 0, 0, 1), (-8, 1.2, 0.04, -1)]
    env = SpinRLBatchEnv(2, 2, cases=cases)
    try:
        env.reset()
        env._assert_floor(np.arange(2))
        # Identical integration state, terrain and next frame through native
        # warmup and mjbatch must produce the same physics/controller outputs.
        env._fill_frames()
        states, sensors, forces = env.state.copy(), env.sensordata.copy(), env.actuator_force.copy()
        # Use two fresh identical controllers to avoid advancing the same one twice.
        reference = BatchControl(2, 2)
        try:
            reference.reset([1, 1])
            env.control.reset([1, 1])
            reference.spin_command(env.inputs, [1, 1])
            env.control.spin_command(env.inputs, [1, 1])
            reference.warmup(env.model, states, np.ones(2, np.int32), [1, 1], sensors, forces,
                             terrain=(env.floor_quat[:, env.floor], env.floor_friction[:, env.floor]))
            # warmup begins with mj_forward; match that sensor timing explicitly.
            env.batch.forward()
            env._fill_frames()
            env.ctrl[:] = env.control.step([1, 1], env.dt)
            env.batch.step()
            np.testing.assert_allclose(env.state, states, atol=2e-6, rtol=2e-6)
            np.testing.assert_allclose(env.actuator_force, forces, atol=2e-5)
        finally:
            reference.close()
    finally:
        env.close()


def test_native_actions_reach_servos_and_wheels_with_correct_units():
    env = make_env(1)
    control = BatchControl(7, 2)
    try:
        env.reset()
        env._fill_frames()
        for i in range(7):
            control._frames[i] = env.control._frames[0]
        control.reset([1]*7)
        commands = np.zeros((7, 7), np.float32)
        commands[1:6, 2:] = np.eye(5)
        control.spin_command(commands, [1]*7)
        disabled = ctypes.CDLL(str(Path(control._directory.name)/"controller_6.so"))
        disabled.robot_control_set_spin_residual.argtypes = [ctypes.c_void_p, ctypes.c_int]
        disabled.robot_control_set_spin_residual(None, 0)
        for _ in range(400):
            outputs = control.step([1]*7, 0.001).copy()
        np.testing.assert_array_equal(outputs[0], outputs[6])
        # Replay identical sensor frames, so only residual paths can differ.
        delta = outputs-outputs[0]
        assert np.linalg.norm(delta[1, [0, 1]]) > 0.001
        assert np.linalg.norm(delta[2, [0, 1, 3, 4]]) > 0.001
        assert np.linalg.norm(delta[3, [0, 1, 3, 4]]) > 0.0001
        np.testing.assert_allclose(delta[4, [2, 5]], [0.020, 0.020], atol=2e-5)
        np.testing.assert_allclose(delta[5, [2, 5]], [-0.015, 0.015], atol=2e-5)
        # Check actual reference direction, before any actuator/kinematic sign.
        np.testing.assert_allclose(delta[1, [0, 1]], delta[2, [0, 1]], atol=2e-6)
        np.testing.assert_allclose(delta[1, [3, 4]], -delta[2, [3, 4]], atol=2e-6)
        path = Path(control._directory.name)/"controller_1.so"
        library = ctypes.CDLL(str(path))
        library.robot_control_get_spin_residual.argtypes = [ctypes.POINTER(ctypes.c_float)]
        applied = (ctypes.c_float*5)()
        library.robot_control_get_spin_residual(applied)
        assert applied[0] == pytest.approx(1, abs=0.001)
    finally:
        control.close()
        env.close()


def test_sensor_obs_do_not_depend_on_world_truth():
    env = make_env()
    try:
        env.reset()
        env._fill_frames()
        env.control.reset([1, 1])
        env.control.spin_command(env.inputs, [1, 1])
        env.control._frames[1] = env.control._frames[0]
        env.control.frames["body_pos"][1] = [100, -200, 30]
        env.control.frames["body_vel"][1] = [15, -20, 5]
        env.control.step([1, 1], 0.001)
        obs = env.control.spin_observation()
        np.testing.assert_array_equal(obs[0], obs[1])
        assert OBS_SCALE.shape == (41,)
    finally:
        env.close()


def test_explicit_full_range_turning_does_not_clear_extreme_leg_commands():
    env = SpinRLBatchEnv(2, 2, cases=[(0, 0, -0.093, 1), (0, 0, 0.070, 1)])
    try:
        env.reset()
        lengths = env.control.spin_observation()[:, 11:13].mean(axis=1)
        assert lengths[0] > 0.165 and lengths[1] < 0.066
        env._fill_frames()
        env.inputs[:, 1] = 2.4
        env.control.spin_command(env.inputs, [1, 1])
        for _ in range(200):
            env.control.step([1, 1], 0.001)
        assert np.all(env.control.spin_observation()[:, 26] > 0.05)
        for i in range(2):
            library = ctypes.CDLL(str(Path(env.control._directory.name)/f"controller_{i}.so"))
            library.balance_leg_drive_min.restype = ctypes.c_float
            library.balance_leg_drive_max.restype = ctypes.c_float
            assert library.balance_leg_drive_min() < 0.047
            assert library.balance_leg_drive_max() > 0.210
        # Disabling the residual interface restores the ordinary turn window.
        for i in range(2):
            library = ctypes.CDLL(str(Path(env.control._directory.name)/f"controller_{i}.so"))
            library.balance_leg_drive_min.restype = ctypes.c_float
            library.balance_leg_drive_max.restype = ctypes.c_float
            library.robot_control_set_spin_residual.argtypes = [ctypes.c_void_p, ctypes.c_int]
            library.robot_control_set_spin_residual(None, 0)
            assert library.balance_leg_drive_min() == pytest.approx(0.066)
            assert library.balance_leg_drive_max() == pytest.approx(0.165)
        env.control.step([1, 1], 0.001)
        assert np.all(env.control.spin_observation()[:, 26] == 0)
    finally:
        env.close()


def test_long_spin_release_retains_pitch_recovery_authority():
    env = SpinRLBatchEnv(1, 1, cases=[(0, 0, -0.093, 1)])
    try:
        env.reset()
        env._fill_frames()
        env.control.frames["pitch"] = 0.08
        env.control.frames["pitch_rate"] = 0
        env.control.frames["roll_rate"] = 0
        env.control.frames["yaw_rate"] = 0
        env.inputs[:, 1] = 1.38
        env.control.spin_command(env.inputs, [1])
        for _ in range(2000):
            env.control.step([1], env.dt)
        env.inputs[:, 1] = 0
        env.control.spin_command(env.inputs, [1])
        for _ in range(200):
            output = env.control.step([1], env.dt)
        library = ctypes.CDLL(str(Path(env.control._directory.name)/"controller_0.so"))
        assert library.balance_get_post_manual() == 1
        common_nm = 5*(output[0, 2]+output[0, 5])
        assert 1.1 < abs(common_nm) <= 4.00001
    finally:
        env.close()


def test_pid_only_stays_pid_when_pose_allows_lqr():
    env = make_env(1)
    pid = BatchControl(1, 1, pid_only=True)
    try:
        env.reset()
        env._fill_frames()
        pid._frames[0] = env.control._frames[0]
        assert pid.build_id != env.control.build_id
        controls = (env.control, pid)
        modes = []
        for control in controls:
            control.reset([1])
            control.spin_command(np.zeros((1, 7), np.float32), [1])
            library = ctypes.CDLL(str(Path(control._directory.name)/"controller_0.so"))
            library.controller_supervisor_lqr_mix.restype = ctypes.c_float
            modes.append(library.controller_supervisor_lqr_mix)
        # Replay the same upright frame past the handover duration.
        for _ in range(600):
            for control in controls:
                control.step([1], env.dt)
        assert modes[0]() == pytest.approx(1)
        assert modes[1]() == 0
        assert env.provenance["pid_only"] is False
        assert env.provenance["controller_build"] == env.control.build_id
    finally:
        pid.close()
        env.close()


def test_roll_priority_and_spin_tracking():
    zeros = np.zeros(1)
    actions = np.zeros((1, 5))
    def reward(roll=0, pitch=0, rate=2):
        return spin_reward(np.array([roll]), np.array([pitch]), zeros,
                           np.array([rate]), np.array([2]), zeros, zeros,
                           actions, actions, zeros)[0][0]
    assert reward()-reward(roll=np.deg2rad(3)) == pytest.approx(5)
    assert reward()-reward(pitch=np.deg2rad(8)) == pytest.approx(0.25)
    assert reward(rate=2) > reward(rate=0)+1.3
    assert reward(rate=-0.2) > reward(rate=-0.4)
    # Pure world-yaw rotation at a tilted pose must not appear as roll motion.
    r, p = 0.15, 0.1
    q = np.empty(4)
    mujoco.mju_euler2Quat(q, np.array([0.3, p, r]), "zyx")
    gyro = np.array([[-np.sin(p)*2, np.sin(r)*np.cos(p)*2, np.cos(r)*np.cos(p)*2]])
    _, _, _, roll_dot, yaw_dot = attitude(q[None], gyro)
    assert roll_dot[0] == pytest.approx(0, abs=1e-6)
    assert yaw_dot[0] == pytest.approx(2, abs=1e-6)
    np.testing.assert_allclose(euler_rates(np.array([r]), np.array([p]), gyro), [[0, 0, 2]], atol=1e-6)


def test_partial_reset_preserves_other_state_and_time_limit():
    env = make_env(2, episode_seconds=0.1)
    try:
        env.seed(20)
        env.reset()
        saved_state, saved_history = env.state[1].copy(), env.history[1].copy()
        env._reset_ids([0])
        np.testing.assert_array_equal(env.state[1], saved_state)
        np.testing.assert_array_equal(env.history[1], saved_history)
        env.steps[0] = env.episode_steps-1
        _, _, done, infos = env.step(np.zeros((2, 5), np.float32))
        assert done[0] and not done[1]
        assert infos[0]["TimeLimit.truncated"]
        assert infos[0]["terminal_observation"].shape == (328,)
        assert infos[0]["episode"]["l"] == 5
        assert env.steps[0] == 0 and env.steps[1] == 1
        stats = infos[0]["spin_metrics"]
        assert stats["failure_reasons"] == []
        assert sum(stats["reward_totals"].values()) == pytest.approx(infos[0]["episode"]["r"])
    finally:
        env.close()


def test_physical_reference_ramps_to_zero_before_recovery():
    target = np.zeros(2)
    ext = np.array([-0.093, 0.070])
    direction = np.array([1, -1])
    last = target.copy()
    for step in range(20_000):
        target, phase = spin_reference(np.full(2, step*0.001), 20, ext, direction, target, 0.001)
        assert np.max(np.abs(target-last)) <= 0.00080001
        if step == 15_999:
            np.testing.assert_allclose(target, [1.38, -2.76], atol=1e-6)
            np.testing.assert_array_equal(phase, [2, 2])
        last = target.copy()
    np.testing.assert_array_equal(target, [0, 0])
    np.testing.assert_array_equal(phase, [4, 4])
    # A longer diagnostic recovery must not extend the commanded spin.
    target, phase = spin_reference(np.array([16.0]), 24, np.array([0.070]),
                                   np.array([1]), np.array([2.76]), 0.001, 8)
    np.testing.assert_allclose(target, [2.7592], atol=1e-7)
    np.testing.assert_array_equal(phase, [3])


def test_base_and_reward_share_rate_during_deceleration_and_failure_is_diagnosed():
    env = make_env(1)
    try:
        env.reset()
        env.steps[:] = 800
        env.yaw_target[:] = 2.76
        _, _, done, info = env.step(np.zeros((1, 5), np.float32))
        assert not done[0]
        assert info[0]["phase"] == "decelerate"
        assert info[0]["yaw_target"] == pytest.approx(2.744, abs=1e-5)
        assert info[0]["base_yaw_target"] == pytest.approx(info[0]["yaw_target"], abs=1e-5)
        # A deterministic escape must be logged separately from attitude failure.
        env.qpos[0, 0] += 4
        _, _, done, info = env.step(np.zeros((1, 5), np.float32))
        assert done[0]
        metrics = info[0]["spin_metrics"]
        assert "drift" in metrics["failure_reasons"]
        assert metrics["failure_phase"] == "decelerate"
        assert "pitch_peak_deg" in metrics["phases"]["decelerate"]
        assert sum(metrics["reward_totals"].values()) == pytest.approx(info[0]["episode"]["r"])
    finally:
        env.close()


def test_task_target_survives_base_yaw_gate_and_stop_is_a_failure():
    env = make_env(1)
    try:
        env.reset()
        # Simulate a controller gate that reports no requested yaw or movement.
        # Task intent must remain nonzero and stall detection must terminate it.
        env.steps[:] = 350
        env.yaw_target[:] = 2.76
        env.stall_seconds[:] = 1.99
        env._imu = lambda: (np.array([[1., 0, 0, 0]]), np.zeros((1, 3)), np.zeros((1, 3)))
        _, _, done, infos = env.step(np.zeros((1, 5), np.float32))
        assert done[0] and infos[0]["spin_metrics"]["stalled"]
        assert not infos[0]["TimeLimit.truncated"]
        assert infos[0]["yaw_target"] == pytest.approx(2.76)
        assert infos[0]["spin_metrics"]["yaw_rmse"] > 2.7
        assert infos[0]["spin_metrics"]["yaw_reference_angle_rad"] == pytest.approx(2.76*0.02)
        assert infos[0]["spin_metrics"]["yaw_angle_ratio"] == 0
    finally:
        env.close()

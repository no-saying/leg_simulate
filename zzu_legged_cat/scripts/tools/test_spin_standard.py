"""Standard-pose length mapping, finite boundaries and V5 reward isolation."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from eval_spin_rl import summary
from spin_standard_env import (
    StandardSpinEnv,
    ramp_surface_height,
    standard_pose_lengths,
    standard_reward_weights,
)
from spin_travel_env import TravelSpinEnv


@pytest.mark.parametrize("slope", [0, 3, 8])
def test_map_same_base_height_across_headings_and_partial_ramp(slope):
    angle = np.deg2rad(slope)
    length, height, floor, track = 2*np.cos(angle), 2*np.sin(angle), -0.152, 0.272
    centers = [-0.6, -0.06, 0, 0.06, 0.8, length-0.06, length, length+0.06, length+0.6]
    headings = np.linspace(-np.pi, np.pi, 17)
    for center in centers:
        n = len(headings)
        lateral = np.array([-track/2, track/2])
        current_xy = np.stack((center-np.sin(headings)[:, None]*lateral,
                               np.cos(headings)[:, None]*lateral), axis=2)
        aligned_xy = np.tile([[center-track/2, 0], [center+track/2, 0]], (n, 1, 1))
        current, aligned = [ramp_surface_height(xy, np.full(n, length), np.full(n, height), floor)
                            for xy in (current_xy, aligned_xy)]
        base = floor+np.clip(center/length, 0, 1)*height+0.140
        lengths = base-current
        mapped = standard_pose_lengths(lengths, np.zeros_like(lengths), np.zeros(n),
                                       np.zeros(n), lateral, current, aligned)
        np.testing.assert_allclose(mapped[:, 0], base-aligned.max(axis=1), atol=1e-14)
        np.testing.assert_allclose(mapped[:, 1], base-aligned.min(axis=1), atol=1e-14)
        np.testing.assert_allclose(mapped, np.tile(mapped[0], (n, 1)), atol=1e-14)
        # At the aligned headings, the standard is the high-side short leg.
        for i in (4, 12):
            assert mapped[i, 0] == pytest.approx(lengths[i].min())
    if slope:
        # A wheel partly on the lower flat sees only the ramp part of the span.
        xy = np.array([[[-track/2, 0], [track/2, 0]]])
        h = ramp_surface_height(xy, [length], [height], floor)
        assert h[0, 1]-h[0, 0] == pytest.approx(track/2*np.tan(angle))


def test_mapping_uses_vertical_projection_and_retains_impossible_lengths():
    roll, pitch = np.array([0.12]), np.array([-0.09])
    lateral, swing = np.array([-0.12, 0.13]), np.array([[0.02, -0.01]])
    current, aligned = np.array([[0.02, 0.04]]), np.array([[0, 0.06]])
    base = 0.24
    cp, sp, cr, sr = np.cos(pitch[0]), np.sin(pitch[0]), np.cos(roll[0]), np.sin(roll[0])
    lengths = (base-current-sp*swing+cp*sr*lateral)/(cp*cr)
    result = standard_pose_lengths(lengths, swing, roll, pitch, lateral, current, aligned)
    np.testing.assert_allclose(result, [[0.18, 0.24]], atol=1e-14)
    assert result[0, 1] > 0.210
    # Swapping the uphill physical side does not change the measurement standard.
    swapped = standard_pose_lengths(lengths[:, ::-1], swing[:, ::-1], roll, pitch,
                                    lateral[::-1], current[:, ::-1], aligned[:, ::-1])
    np.testing.assert_allclose(swapped, result)


def test_finite_surface_includes_outside_ramp_width_and_platform_end():
    points = np.array([[[-0.1, 0], [0.5, 0], [1, 0], [3, 0], [5.01, 0], [0.5, 2.01]]])
    np.testing.assert_allclose(ramp_surface_height(points, [1], [0.2], -0.152),
                               [[-0.152, -0.052, 0.048, 0.048, -0.152, -0.152]])


def test_weight_schedule_is_direction_independent_bounded_and_smooth():
    lengths = np.linspace(0.02, 0.24, 10001)
    yaw, drift = standard_reward_weights(lengths)
    assert (np.diff(yaw) <= 1e-12).all() and (np.diff(drift) >= -1e-12).all()
    y, d = standard_reward_weights([0.047, 0.117, 0.210])
    np.testing.assert_array_equal(y, [2, 1, 1])
    np.testing.assert_array_equal(d, [1, 1, 1.5])
    for boundary in (0.047, 0.090, 0.165, 0.210):
        for weights in standard_reward_weights(np.array([boundary-1e-7, boundary, boundary+1e-7])):
            assert np.ptp(weights) < 1e-9


def test_v5_changes_reward_without_terrain_inputs_to_actor_or_commands():
    kwargs = {"cases": [(8, 1.57, 0, -1)], "terrain_cases": [
        {"location": "crest", "handover": "lqr_to_pid", "offset": 0.06,
         "takeover_time": 0.8, "fade": 0.3, "switch_lead": 0.1, "dropout": False}]}
    env, old = StandardSpinEnv(1, 1, **kwargs), TravelSpinEnv(1, 1, **kwargs)
    try:
        env.seed(126)
        old.seed(126)
        obs, old_obs = env.reset(), old.reset()
        np.testing.assert_array_equal(obs, old_obs)
        assert obs.shape == (1, 416)
        assert 0.2 < env.axle_track < 0.3
        frame, inputs = env._frame_obs().copy(), env.inputs.copy()
        env.standard_requested[:] = -1
        env.current_support[:] = 30
        env.aligned_support[:] = -50
        env.contact_values[:] = 100
        np.testing.assert_array_equal(env._frame_obs(), frame)
        np.testing.assert_array_equal(env.inputs, inputs)
        # Identical nonzero actions must retain identical native physics too.
        for _ in range(100):
            action = np.array([[0.1, 0.2, -0.1, 0.05, -0.05]], np.float32)
            env.step(action)
            old.step(action)
        np.testing.assert_allclose(env.state, old.state, atol=1e-10, rtol=1e-10)
        np.testing.assert_array_equal(env._frame_obs(), old._frame_obs())

        env._before_policy_step()
        args = [np.array([np.deg2rad(3)]), np.zeros(1), np.zeros(1),
                np.array([1.0]), np.array([2.0]), np.array([0.2]), np.array([0.3]),
                np.zeros((1, 5)), np.zeros((1, 5)), np.zeros(1)]
        terms = []
        for length in (0.047, 0.117, 0.210):
            env.standard_requested[:] = length
            terms.append(env._reward(*args)[1])
        assert [t["roll"][0] for t in terms] == pytest.approx([-5]*3)
        assert 2-terms[0]["yaw"][0] == pytest.approx(2*(2-terms[1]["yaw"][0]))
        assert terms[2]["drift"][0] == pytest.approx(1.5*terms[1]["drift"][0])
        assert terms[2]["yaw"][0] == pytest.approx(terms[1]["yaw"][0])
    finally:
        env.close()
        old.close()


def test_reports_use_standard_lengths_and_preserve_nominal_comparison():
    common = {"failed": False, "stalled": False, "return": 0, "roll_rms_deg": 0,
              "max_drift_m": 0, "duration_s": 20, "yaw_angle_ratio": 1,
              "standard_actual_mean_m": 0.08, "standard_infeasible_fraction": 0}
    rows = [{**common, "extension": 0, "standard_requested_initial_m": 0.08},
            {**common, "extension": -0.093, "standard_requested_initial_m": 0.19}]
    stats = summary(rows)
    assert stats["by_leg_band"]["short"]["cases"] == 1
    assert stats["by_nominal_common_leg_band"]["middle"]["cases"] == 1
    assert "middle" not in stats["by_leg_band"]

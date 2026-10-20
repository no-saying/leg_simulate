"""Physical torque units, actuator order and sustained settling measurements."""

from pathlib import Path
import json
import sys

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jump_rl_env import XML
from spin_joint_pd import (ACTUATOR_NAMES, JointPDGains, JointTargetPD,
                           TORQUE_RATE_NM_S, TORQUE_LIMIT_NM)
from tools.tune_rl_joint_pd import response_metrics, wheel_center_load


def test_pd_uses_physical_torque_before_actuator_gain():
    model = mujoco.MjModel.from_xml_path(str(XML))
    pd = JointTargetPD(model)
    gains = JointPDGains(20, .2, .4)
    ctrl, torque, requested = pd.control(np.zeros((1, 6)), np.zeros((1, 6)),
        np.full((1, 4), .01), np.full((1, 2), 2.), gains, np.zeros((1, 6)), 1.)
    np.testing.assert_allclose(torque, [[.2, .2, .8, .2, .2, .8]])
    np.testing.assert_allclose(ctrl, [[.004, .004, .08, .004, .004, .08]])
    np.testing.assert_allclose(torque, requested)
    np.testing.assert_allclose(ctrl*pd.motor_gain, torque)


def test_feedback_opposes_motion_and_respects_limits_and_slew():
    pd = JointTargetPD(mujoco.MjModel.from_xml_path(str(XML)))
    gain = JointPDGains(20, .2, .4)
    q = np.zeros((1, 6))
    ref = np.zeros((1, 4))
    wheel = np.zeros((1, 2))
    raw = pd.requested_torque(q, np.ones((1, 6)), ref, wheel, gain)
    assert (raw < 0).all()
    ctrl, torque, raw = pd.control(q, q, ref+100, wheel+100, gain, q, .001)
    assert (np.abs(torque) <= TORQUE_LIMIT_NM).all()
    np.testing.assert_allclose(torque, TORQUE_RATE_NM_S[None]*.001)
    assert (raw > torque).all()
    no_error = pd.requested_torque(q, q, ref, wheel, gain)
    np.testing.assert_array_equal(no_error, 0)


def test_settling_requires_remaining_inside_tolerance():
    time = np.linspace(.01, 1., 100)
    never_settled = np.ones(100)*.85
    never_settled[30:40] = 1.
    metrics = response_metrics(time, never_settled, 1., 0., 1., .2)
    assert metrics['settle_seconds'] is None
    settled = never_settled.copy()
    settled[50:] = 1.
    metrics = response_metrics(time, settled, 1., 0., 1., .2)
    assert metrics['settle_seconds'] == pytest.approx(.301)
    mirrored = response_metrics(time, -settled, -1., 0., -1., .2)
    assert mirrored == metrics


def test_bench_support_force_acts_at_wheel_axle():
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    wrenches = np.zeros((1, model.nbody, 6))
    wheel_center_load(model, wrenches, data.xpos[None], data.xipos[None], 12.)
    for name in ('Left_Wheel_link', 'Right_Wheel_link'):
        body = model.body(name).id
        wrench = wrenches[0, body]
        direct, shifted = np.zeros(model.nv), np.zeros(model.nv)
        mujoco.mj_applyFT(model, data, np.array([0., 0., 12.]), np.zeros(3),
                         data.xpos[body], body, direct)
        mujoco.mj_applyFT(model, data, wrench[:3], wrench[3:],
                         data.xipos[body], body, shifted)
        np.testing.assert_allclose(shifted, direct, atol=1e-12)


def test_saved_baseline_matches_physical_tracker_contract():
    path = Path(__file__).resolve().parents[2]/'config/rl_joint_pd.json'
    config = json.loads(path.read_text())
    gains = JointPDGains.load(path)
    assert gains == JointPDGains(120., 1.6, .5)
    assert config['actuator_order'] == list(ACTUATOR_NAMES)
    np.testing.assert_array_equal(config['torque_limits_nm'], TORQUE_LIMIT_NM)
    np.testing.assert_array_equal(config['torque_rate_limits_nm_s'], TORQUE_RATE_NM_S)
    assert config['servo_period_s'] == pytest.approx(.001)


@pytest.mark.parametrize('values', [(0, .2, .4), (20, -1, .4), (20, .2, float('nan'))])
def test_invalid_gains_are_rejected(values):
    with pytest.raises(ValueError):
        JointPDGains(*values)

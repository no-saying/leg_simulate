"""Closed-chain geometry, independent wheel rejection and warmup continuity."""

import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jump_rl_env import XML
from spin_anchor_env import SpinOdometry
from spin_leg_geometry import EncoderLegGeometry
from spin_sensor_odometry import InnovationSpinOdometry


@pytest.mark.parametrize('slipping', [(0,), (1,), (0, 1)])
def test_wheel_slip_is_rejected_per_side_during_spin(slipping):
    lateral = np.array([-.119375, .133])
    observer = InnovationSpinOdometry(1, lateral)
    baseline = SpinOdometry(1, lateral)
    gyro = np.array([[0., 0., 2.]])
    observer.initialize_velocity(np.zeros((1, 2)), gyro)
    native = np.zeros((1, 28))
    native[:, 11:13] = .117
    acc = np.array([[0., .020*4, 9.81]])
    for tick in range(3000):
        q = np.array([[np.cos(tick*.001), 0, 0, np.sin(tick*.001)]])
        native[:, 9:11] = 2*lateral/.043
        if 500 <= tick < 2000:
            native[:, np.array(slipping)+9] -= .8/.043
        observer.step(q, gyro, acc, native, .001)
        baseline.step(q, gyro, acc, native, .001)
        if tick == 1500:
            for side in slipping:
                assert observer.wheel_confidence[0, side] < .01
            for side in set((0, 1))-set(slipping):
                assert observer.wheel_confidence[0, side] > .99
    assert np.linalg.norm(observer.position) < .001
    assert np.linalg.norm(baseline.position) > .1
    assert (observer.wheel_confidence > .99).all()


def test_lateral_sliding_acceleration_is_not_erased_by_pseudo_measurement():
    observer = InnovationSpinOdometry(1, [-.119375, .133])
    observer.initialize_velocity(np.zeros((1, 2)), np.zeros((1, 3)))
    native = np.zeros((1, 28))
    native[:, 11:13] = .117
    dt = .001
    for tick in range(1500):
        acc = np.array([[0., 2. if tick < 100 else 0., 9.81]])
        observer.step(np.array([[1., 0., 0., 0.]]), np.zeros((1, 3)), acc, native, dt)
    assert observer.velocity[0, 1] == pytest.approx(.2, abs=.01)
    assert observer.position[0, 1] > .26
    assert observer.sideways_confidence[0] < .01


@pytest.mark.parametrize('slipping', [0, 1])
def test_good_wheel_remains_usable_with_attitude_error_and_other_wheel_slipping(slipping):
    observer = InnovationSpinOdometry(1, [-.119375, .133], directional_reference=True)
    baseline = InnovationSpinOdometry(1, [-.119375, .133], directional_reference=False)
    gyro = np.array([[0., 0., 2.]])
    observer.initialize_velocity(np.zeros((1, 2)), gyro)
    baseline.initialize_velocity(np.zeros((1, 2)), gyro)
    native = np.zeros((1, 28))
    native[:, 11:13] = .117
    acc = np.array([[0., .08, 9.81]])
    tilt = np.deg2rad(.05)/2
    c, s = np.cos(tilt), np.sin(tilt)
    peak_error = 0
    for tick in range(12000):
        angle = tick*.001
        w, z = np.cos(angle), np.sin(angle)
        # Fixed world tilt models a small residual error in the attitude estimate.
        q = np.array([[c*w, s*w, -s*z, c*z]])
        native[:, 9:11] = 2*observer.lateral/.043
        if 2000 <= tick < 10000:
            native[:, 9+slipping] -= .8/.043
        observer.step(q, gyro, acc, native, .001)
        baseline.step(q, gyro, acc, native, .001)
        peak_error = max(peak_error, np.linalg.norm(observer.position))
        if tick == 9999:
            assert observer.wheel_confidence[0, slipping] < .01
            assert observer.wheel_confidence[0, 1-slipping] > .9
    assert peak_error < .02
    assert (observer.wheel_confidence > .9).all()
    assert np.linalg.norm(baseline.position) > .15


def test_closed_chain_geometry_against_constrained_multibody_motion():
    model = mujoco.MjModel.from_xml_path(str(XML))
    model.opt.gravity[:] = 0
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    geometry = EncoderLegGeometry(model)
    data = mujoco.MjData(model)
    joint_names = ('Left_front_joint', 'Left_rear_joint', 'Right_front_joint', 'Right_rear_joint')
    qa = [model.joint(n).qposadr[0] for n in joint_names]
    va = [model.joint(n).dofadr[0] for n in joint_names]
    kp = np.linalg.inv([[.069055, -.070802], [.049910, .056241]])
    errors, spin_errors = [], []
    for length in (.055, .117, .200):
        mujoco.mj_resetData(model, data)
        for tick in range(1000):
            data.qpos[:7] = [0, 0, .5, 1, 0, 0, 0]
            data.qvel[:6] = 0
            target = kp@[length-.08656, .015*np.sin(tick*.004)]
            desired = np.r_[target, -target]
            data.ctrl[[0, 1, 3, 4]] = np.clip((20*(desired-data.qpos[qa])-.7*data.qvel[va])/50, -1.2, 1.2)
            mujoco.mj_step(model, data)
            if tick < 500 or tick % 20:
                continue
            data.qvel[:6] = 0
            mujoco.mj_forward(model, data)
            wheel, velocity, spin, valid = geometry.calculate(data.qpos[qa][None], data.qvel[va][None])
            assert valid.all()
            for side, name in enumerate(('Left_Wheel_joint', 'Right_Wheel_joint')):
                joint = model.joint(name).id
                body = model.jnt_bodyid[joint]
                parent = model.body_parentid[body]
                errors.append(wheel[0, side]-(data.xpos[body]-data.xpos[model.body('base_link').id])[:2])
                jac = np.zeros((3, model.nv))
                mujoco.mj_jacBody(model, data, None, jac, parent)
                actual_spin = data.xaxis[joint]@(jac@data.qvel)
                spin_errors.append(spin[0, side]-actual_spin)
    assert np.max(np.linalg.norm(errors, axis=1)) < .002
    assert np.sqrt(np.mean(np.array(spin_errors)**2)) < .15


def test_warmup_velocity_integral_uses_sensors_and_mask_only():
    from rl_batch_control import BatchControl
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for body in range(model.nbody):
        if model.body(body).name.startswith(('test_ball_', 'large_ball_')):
            adr = model.jnt_qposadr[model.body_jntadr[body]]
            data.qpos[adr:adr+3] = [100+body, 100, 0]
    mujoco.mj_forward(model, data)
    state = np.zeros((1, mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION)))
    mujoco.mj_getState(model, data, state[0], mujoco.mjtState.mjSTATE_INTEGRATION)
    control = BatchControl(1, 1)
    delta = np.full((1, 3), 99.)
    sensors, forces = np.zeros((1, model.nsensordata)), np.zeros((1, model.nu))
    try:
        control.reset([1])
        inputs = np.zeros((1, 7), np.float32)
        control.spin_command(inputs, [1])
        terrain = (np.array([[1., 0, 0, 0]]), model.geom_friction[model.geom('floor').id][None])
        control.warmup(model, state, np.array([1000]), np.array([True]), sensors, forces,
                       terrain=terrain, imu_delta=delta)
        mujoco.mj_setState(model, data, state[0], mujoco.mjtState.mjSTATE_INTEGRATION)
        np.testing.assert_allclose(delta[0], data.qvel[:3], atol=.02)
        saved = state.copy()
        delta[:] = 17
        control.warmup(model, state, np.array([100]), np.array([False]), sensors, forces,
                       terrain=terrain, imu_delta=delta)
        np.testing.assert_array_equal(delta, 17)
        np.testing.assert_array_equal(state, saved)
    finally:
        control.close()

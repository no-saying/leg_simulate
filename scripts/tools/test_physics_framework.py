"""Engine boundaries, offline calibration, physical units and legacy parity."""

import json
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from wheel_legged.backends.mujoco import MujocoBackend
from wheel_legged.backends.mujoco_model import JointBinding, extract_leg_calibration
from wheel_legged.core.actuation import JointPDGains
from wheel_legged.core.frames import projected_up
from wheel_legged.core.geometry import EncoderLegGeometry
from wheel_legged.core.observations import compact_frame, ObservationHistory, OBS_SCALE
from wheel_legged.core.robot import ACTUATOR_NAMES, LEG_CHANNELS, WHEEL_CHANNELS
from wheel_legged.runtime import JointServoRuntime

XML = ROOT/'model/v3/COD-2026RoboMaster-Balance-v3.xml'


def test_core_runtime_and_inference_do_not_import_simulator_or_trainer():
    script = '''
import importlib.abc, json, sys
from pathlib import Path
class BlockEngine(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'mujoco', 'mjbatch', 'stable_baselines3', 'gymnasium'}:
            raise AssertionError('Forbidden dependency: '+fullname)
sys.meta_path.insert(0, BlockEngine())
sys.path.insert(0, str(Path('scripts').resolve()))
from wheel_legged.core import actuation, geometry, frames, observations, rewards, wrench
from wheel_legged import interfaces, runtime
from wheel_legged.policies.networks import HistoryActor, SensorActor
from wheel_legged.interfaces import SensorState
import numpy as np
cal = json.loads(Path('config/v3_robot_calibration.json').read_text())
g = geometry.EncoderLegGeometry(cal['leg_sides'])
assert g.calculate([[.2,-.2,-.2,.2]], [[0,0,0,0]])[3].all()
class IndependentBackend:
    num_envs, dt = 1, .001
    def read_sensors(self):
        return SensorState(np.zeros((1,6)), np.zeros((1,6)),
            np.array([[1.,0,0,0]]), np.zeros((1,3)), np.zeros((1,3)))
    def apply_joint_torque(self, torque): self.torque = torque.copy()
    def step(self): self.advanced = True
b = IndependentBackend()
r = runtime.JointServoRuntime(b, actuation.JointPDGains(120,1.6,.5))
r.tick(np.full((1,4), .01), np.ones((1,2)))
assert b.advanced and b.torque.shape == (1,6)
assert np.max(b.torque) <= .3
'''
    result = subprocess.run([sys.executable, '-c', script], cwd=ROOT,
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout+result.stderr


def test_exported_geometry_matches_model_extraction():
    model = mujoco.MjModel.from_xml_path(str(XML))
    calibration = json.loads((ROOT/'config/v3_robot_calibration.json').read_text())
    direct = EncoderLegGeometry(extract_leg_calibration(model))
    offline = EncoderLegGeometry(calibration['leg_sides'])
    q = np.array([[.2, -.2, -.2, .2], [.6, -.6, -.6, .6]])
    dq = np.array([[.1, -.2, .3, -.1], [-.2, .4, -.1, .2]])
    for wanted, actual in zip(direct.calculate(q, dq), offline.calculate(q, dq)):
        np.testing.assert_array_equal(actual, wanted)


def test_compact_frame_is_exactly_legacy_order_and_scaling():
    rng = np.random.default_rng(33)
    groups = [rng.normal(size=(3, size)) for size in (3, 3, 3, 4, 6, 6)]
    expected = np.clip(np.column_stack(groups)/OBS_SCALE, -20, 20).astype(np.float32)
    np.testing.assert_array_equal(compact_frame(*groups), expected)
    np.testing.assert_array_equal(projected_up(np.array([[1., 0, 0, 0]])), [[0, 0, 1]])


def test_history_resets_only_selected_rows_and_counts_current_frame():
    history = ObservationHistory(2, frames=5)
    history.reset([0, 1], np.stack((np.zeros(25), np.ones(25))))
    for value in range(2, 7):
        history.append(np.full((2, 25), value))
    np.testing.assert_array_equal(history.values[0, :, 0], [2, 3, 4, 5, 6])
    before = history.values[1].copy()
    history.reset([0], np.full((1, 25), 10))
    np.testing.assert_array_equal(history.values[0], 10)
    np.testing.assert_array_equal(history.values[1], before)
    assert history.observation().shape == (2, 125)


def test_actuator_reordering_preserves_joint_torque():
    bodies = ''.join(f'<body><joint name="{name}"/><geom size=".01"/></body>'
                     for name in ACTUATOR_NAMES)
    actuators = ''.join(f'<motor joint="{name}" gear="{i+1}"/>'
                        for i, name in reversed(list(enumerate(ACTUATOR_NAMES))))
    model = mujoco.MjModel.from_xml_string(
        f'<mujoco><worldbody>{bodies}</worldbody><actuator>{actuators}</actuator></mujoco>')
    binding = JointBinding(model)
    data = mujoco.MjData(model)
    expected = np.array([.2, -.3, .4, -.5, .6, -.7])
    data.ctrl[:] = binding.control_from_torque(expected)
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.qfrc_actuator[binding.vadr], expected)
    with pytest.raises(ValueError, match='order'):
        JointBinding(model, require_actuator_order=True)


def test_runtime_matches_direct_mujoco_step_and_physical_pd():
    backend = MujocoBackend.from_xml(XML)
    try:
        data = mujoco.MjData(backend.model)
        data.qpos[:] = backend.qpos[0]
        data.qvel[:] = backend.qvel[0]
        mujoco.mj_forward(backend.model, data)
        gains = JointPDGains.load(ROOT/'config/rl_joint_pd.json')
        runtime = JointServoRuntime(backend, gains)
        s = backend.read_sensors()
        leg = s.joint_position[:, LEG_CHANNELS]+[.01, -.01, -.01, .01]
        wheel = np.array([[.4, -.4]])
        binding = backend.binding
        previous = np.zeros((1, 6))
        for _ in range(50):
            q, dq = data.qpos[binding.qadr], data.qvel[binding.vadr]
            requested = np.zeros(6)
            requested[LEG_CHANNELS] = 120*(leg[0]-q[LEG_CHANNELS])-1.6*dq[LEG_CHANNELS]
            requested[WHEEL_CHANNELS] = .5*(wheel[0]-dq[WHEEL_CHANNELS])
            limit, rate = np.array([60,60,10,60,60,10]), np.array([300,300,40,300,300,40])
            torque = previous+np.clip(np.clip(requested, -limit, limit)-previous,
                                     -rate*.001, rate*.001)
            actual = runtime.tick(leg, wheel)
            np.testing.assert_allclose(actual, torque, atol=1e-12)
            data.ctrl[:] = binding.control_from_torque(torque)[0]
            mujoco.mj_step(backend.model, data)
            mujoco.mj_forward(backend.model, data)
            previous[:] = torque
            np.testing.assert_allclose(backend.qpos[0], data.qpos, atol=1e-12)
            np.testing.assert_allclose(backend.qvel[0], data.qvel, atol=1e-12)
        sensors = backend.read_sensors()
        saved = sensors.joint_position.copy()
        backend.qpos[:, binding.qadr] += .1
        np.testing.assert_array_equal(sensors.joint_position, saved)
    finally:
        backend.close()


def test_backend_subset_reset_and_truth_are_separate_from_sensor_frame():
    backend = MujocoBackend.from_xml(XML, num_envs=2, num_threads=2)
    try:
        runtime = JointServoRuntime(backend, JointPDGains(120, 1.6, .5))
        sensors = backend.read_sensors()
        frame = compact_frame(sensors.body_angular_velocity, projected_up(sensors.body_quaternion),
            np.zeros((2, 3)), sensors.joint_position[:, LEG_CHANNELS],
            sensors.joint_velocity[:, np.r_[LEG_CHANNELS, WHEEL_CHANNELS]], np.zeros((2, 6)))
        backend.qpos[:, backend.base_qadr:backend.base_qadr+2] += [2, -3]
        backend.batch.forward()
        moved = backend.read_sensors()
        actual = compact_frame(moved.body_angular_velocity, projected_up(moved.body_quaternion),
            np.zeros((2, 3)), moved.joint_position[:, LEG_CHANNELS],
            moved.joint_velocity[:, np.r_[LEG_CHANNELS, WHEEL_CHANNELS]], np.zeros((2, 6)))
        np.testing.assert_array_equal(actual, frame)
        assert np.max(np.abs(backend.read_truth().world_position)) >= 2
        runtime.previous_torque[:] = .1
        saved = backend.qpos[1].copy()
        runtime.reset(np.array([0], np.int32))
        np.testing.assert_array_equal(backend.qpos[1], saved)
        np.testing.assert_array_equal(runtime.previous_torque[1], .1)
        np.testing.assert_array_equal(runtime.previous_torque[0], 0)
        with pytest.raises(ValueError, match='physical limits'):
            backend.apply_joint_torque(np.full((2, 6), 61))
        with pytest.raises(ValueError, match='finite'):
            backend.apply_joint_torque(np.full((2, 6), np.nan))
    finally:
        backend.close()
    backend.close()
    with pytest.raises(RuntimeError, match='closed'):
        backend.read_sensors()

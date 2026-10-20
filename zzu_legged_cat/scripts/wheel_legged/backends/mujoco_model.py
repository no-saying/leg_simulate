"""MuJoCo-specific name/address binding and installation calibration."""

import mujoco
import numpy as np

from ..core.geometry import cross
from ..core.robot import ACTUATOR_NAMES


class JointBinding:
    """Canonical motor order can differ from engine actuator order."""

    def __init__(self, model, require_actuator_order=False):
        joints = [model.joint(name) for name in ACTUATOR_NAMES]
        self.qadr = np.array([int(j.qposadr[0]) for j in joints])
        self.vadr = np.array([int(j.dofadr[0]) for j in joints])
        if model.nu != 6 or any(model.jnt_type[j.id] != mujoco.mjtJoint.mjJNT_HINGE for j in joints):
            raise ValueError('Expected six hinge motor joints')
        ids = []
        for joint in joints:
            matches = np.flatnonzero(model.actuator_trnid[:, 0] == joint.id)
            if len(matches) != 1:
                raise ValueError(f'Expected one actuator for {joint.name}')
            ids.append(matches[0])
        self.actuator_ids = np.array(ids)
        if require_actuator_order and not np.array_equal(self.actuator_ids, np.arange(6)):
            raise ValueError('Unexpected actuator order')
        a = self.actuator_ids
        if (np.any(model.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_JOINT)
                or np.any(model.actuator_gaintype[a] != mujoco.mjtGain.mjGAIN_FIXED)
                or np.any(model.actuator_biastype[a] != mujoco.mjtBias.mjBIAS_NONE)
                or np.any(model.actuator_dyntype[a] != mujoco.mjtDyn.mjDYN_NONE)):
            raise ValueError('Expected direct, fixed-gain joint torque actuators')
        self.motor_gain = model.actuator_gainprm[a, 0]*model.actuator_gear[a, 0]
        if not np.isfinite(self.motor_gain).all() or np.any(self.motor_gain == 0):
            raise ValueError('Invalid physical actuator gain')

    def control_from_torque(self, torque_nm):
        torque_nm = np.asarray(torque_nm)
        if torque_nm.shape[-1] != 6 or not np.isfinite(torque_nm).all():
            raise ValueError('Expected six finite physical joint torques')
        ctrl = np.empty_like(torque_nm, dtype=float)
        ctrl[..., self.actuator_ids] = torque_nm/self.motor_gain
        return ctrl


def extract_leg_calibration(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = model.body('base_link').id
    rotation = data.xmat[base].reshape(3, 3)

    def position(kind, name):
        values = data.xpos if kind == 'body' else data.site_xpos
        obj = model.body(name) if kind == 'body' else model.site(name)
        return (rotation.T@(values[obj.id]-data.xpos[base]))[:2]

    sides = []
    for names, sign in ((('Left', 'L_claf', 'L_Claf', '001', '002'), 1),
                         (('Right', 'R_Claf', 'R_Claf', '003', '004'), -1)):
        full, lower, upper, closure1, closure2 = names
        front = position('body', full+'_front_link')
        rear = position('body', full+'_rear_link')
        a = position('body', lower+'_link_1')
        b = position('body', lower+'_link_2')
        ca = position('site', 'closure_'+closure1+'_a_site')
        c = position('site', 'closure_'+closure1+'_b_site')
        d = position('body', upper+'_link_3')
        e = position('body', upper+'_link_4')
        fa = position('site', 'closure_'+closure2+'_a_site')
        f = position('site', 'closure_'+closure2+'_b_site')
        wheel = position('body', full+'_Wheel_link')
        sides.append(dict(front=front, rear=rear, arm_a=a-front,
                          arm_c=c-rear, arm_f=f-rear, bc=ca-b, bd=d-b,
                          ef=fa-e, ew=wheel-e, ab=np.linalg.norm(b-a),
                          de=np.linalg.norm(e-d),
                          branch_b=np.sign(cross(c-a, b-a)),
                          branch_e=np.sign(cross(f-d, e-d)), sign=sign))
    return sides


def imu_attitude_extrinsic(model):
    from ..core.frames import multiply
    sensor = model.sensor('body_quat').id
    site = model.site('imu').id
    kind, obj = model.sensor_objtype[sensor], model.sensor_objid[sensor]
    if kind == mujoco.mjtObj.mjOBJ_BODY and obj == model.site_bodyid[site]:
        inverse = model.body_iquat[obj]*np.array([1, -1, -1, -1])
        return multiply(inverse[None], model.site_quat[site])[0]
    if kind == mujoco.mjtObj.mjOBJ_SITE and obj == site:
        return np.array([1, 0, 0, 0])
    raise ValueError('Unsupported attitude/IMU frame relationship')

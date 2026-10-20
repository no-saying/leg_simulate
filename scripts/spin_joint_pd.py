"""Compatibility API for the MuJoCo PD bench; core outputs physical Nm."""

from wheel_legged.core.actuation import JointPDGains, JointTargetPD as PhysicalJointPD
from wheel_legged.core.robot import (ACTUATOR_NAMES, LEG_CHANNELS, WHEEL_CHANNELS,
                                     TORQUE_LIMIT_NM, TORQUE_RATE_NM_S)
from wheel_legged.backends.mujoco_model import JointBinding


class JointTargetPD(PhysicalJointPD):
    def __init__(self, model):
        binding = JointBinding(model, require_actuator_order=True)
        self.qadr, self.vadr = binding.qadr, binding.vadr
        self.motor_gain = binding.motor_gain

    def control(self, *args, **kwargs):
        torque, requested = super().control(*args, **kwargs)
        return torque/self.motor_gain, torque, requested

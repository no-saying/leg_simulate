"""Legacy MuJoCo ctrl interface for the physical virtual-work mapper."""

from wheel_legged.core.wrench import (
    ACTION_NAMES, ACTION_SCALE, ACTUATOR_ORDER, TORQUE_LIMIT,
    WrenchTorqueMapper as PhysicalWrenchMapper,
)
from wheel_legged.backends.mujoco_model import JointBinding
from spin_leg_geometry import EncoderLegGeometry


class WrenchTorqueMapper(PhysicalWrenchMapper):
    def __init__(self, model):
        super().__init__(EncoderLegGeometry(model))
        self.gain = JointBinding(model, require_actuator_order=True).motor_gain

    def map(self, positions, commands):
        torque, points, valid = super().map(positions, commands)
        return torque/self.gain, torque, points, valid

"""Legacy MuJoCo constructor for the engine-independent encoder geometry."""

from wheel_legged.core.geometry import (
    EncoderLegGeometry as CalibratedLegGeometry, cross, perpendicular, rotate,
    intersection, intersection_velocity,
)
from wheel_legged.backends.mujoco_model import extract_leg_calibration


class EncoderLegGeometry(CalibratedLegGeometry):
    def __init__(self, model):
        super().__init__(extract_leg_calibration(model))

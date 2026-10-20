"""Calibrate the legacy framequat(body) output into the physical IMU frame.

MuJoCo's body frame sensor uses the inertial principal axes. Gyro and accel
use the site axes. This fixed extrinsic transform is model/installation
calibration, not a runtime world-state observation.
"""

from wheel_legged.core.frames import multiply, aligned_control_attitude
from wheel_legged.backends.mujoco_model import imu_attitude_extrinsic

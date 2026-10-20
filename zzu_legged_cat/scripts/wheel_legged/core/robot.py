"""Canonical joint order and physical actuator protection settings."""

import numpy as np

ACTUATOR_NAMES = ('Left_front_joint', 'Left_rear_joint', 'Left_Wheel_joint',
                  'Right_front_joint', 'Right_rear_joint', 'Right_Wheel_joint')
LEG_CHANNELS = np.array([0, 1, 3, 4])
WHEEL_CHANNELS = np.array([2, 5])
TORQUE_LIMIT_NM = np.array([60., 60., 10., 60., 60., 10.])
TORQUE_RATE_NM_S = np.array([300., 300., 40., 300., 300., 40.])

# Legacy installation calibration, wxyz. Do not change old policy conventions.
IMU_BAKE_Q = np.array([.70710678, .70710678, 0., 0.])

"""Export fixed mechanism/sensor data for use without importing MuJoCo."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from wheel_legged.backends.mujoco_model import (
    JointBinding, extract_leg_calibration, imu_attitude_extrinsic,
)
from wheel_legged.core.geometry import EncoderLegGeometry
from wheel_legged.core.robot import ACTUATOR_NAMES, IMU_BAKE_Q


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--model', type=Path,
        default=root/'model/v3/COD-2026RoboMaster-Balance-v3.xml')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    geometry = EncoderLegGeometry(extract_leg_calibration(model))
    binding = JointBinding(model)
    calibration = dict(schema_version=1, joint_names=list(ACTUATOR_NAMES),
        units='meters, radians, seconds, physical joint Nm',
        leg_joint_order=['left_front', 'left_rear', 'right_front', 'right_rear'],
        leg_plane='forward/up in the calibrated control frame',
        leg_sides=geometry.calibration(),
        legacy_imu_mount_wxyz=IMU_BAKE_Q.tolist(),
        mujoco_attitude_to_site_wxyz=imu_attitude_extrinsic(model).tolist(),
        mujoco_ctrl_to_joint_nm=binding.motor_gain.tolist(),
        source_xml_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
        backend_note='MuJoCo gains/extrinsics are adapter metadata, not Isaac drive parameters')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(calibration, indent=2)+'\n')
    print(args.out)


if __name__ == '__main__':
    main()

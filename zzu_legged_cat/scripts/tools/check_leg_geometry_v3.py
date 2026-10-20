#!/usr/bin/env python3
"""Check v3 leg kinematics against constrained MuJoCo wheel axle positions.

This is a geometry check, not an LQR identification run. The base is fixed in
the air while joint PD moves both closed-chain legs through their usable range.
"""

from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import brentq


ROOT = Path(__file__).resolve().parents[2]
XML = ROOT / "model/v3/COD-2026RoboMaster-Balance-v3.xml"
SW_NOM, L_NOM = -0.01596, 0.08656
J = np.array([[0.069055, -0.070802], [0.049910, 0.056241]])
K = np.linalg.inv(J)


def features(q):
    a, b = q[:, 0], q[:, 1]
    return np.column_stack((np.ones(len(q)), a, b, a*a, a*b, b*b,
                            a*a*a, a*a*b, a*b*b, b*b*b))


def main():
    model = mujoco.MjModel.from_xml_path(str(XML))
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    joints = ["Left_front_joint", "Left_rear_joint",
              "Right_front_joint", "Right_rear_joint"]
    qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in joints]
    vadr = [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in joints]
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    wheels = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
              for name in ("Left_Wheel_link", "Right_Wheel_link")]
    mujoco.mj_resetData(model, data)
    base_pose = np.array([0, 0, 0.4, 0.7071067811865476,
                          0.7071067811865476, 0, 0])

    records = [[], []]
    for length in np.linspace(0.045, 0.210, 18):
        for swing in np.linspace(-0.050, 0.050, 11):
            target_l = K @ [length - L_NOM, swing - SW_NOM]
            target = np.r_[target_l, -target_l]
            for _ in range(350):
                data.qpos[:7] = base_pose
                data.qvel[:6] = 0.0
                for act, qa, va in zip((0, 1, 3, 4), qadr, vadr):
                    torque = 20.0 * (target[act - (1 if act > 2 else 0)] - data.qpos[qa]) \
                             - 0.7 * data.qvel[va]
                    data.ctrl[act] = np.clip(torque / 50.0, -1.2, 1.2)
                mujoco.mj_step(model, data)
            data.qpos[:7] = base_pose
            data.qvel[:6] = 0.0
            mujoco.mj_forward(model, data)
            rotation = data.xmat[base].reshape(3, 3)
            for side in range(2):
                axle_local = rotation.T @ (data.xpos[wheels[side]] - data.xpos[base])
                q = np.array([data.qpos[qadr[2*side]], data.qpos[qadr[2*side+1]]])
                if side:
                    q = -q
                com_local = rotation.T @ (data.subtree_com[base] - data.xpos[base])
                records[side].append((q, np.array([-axle_local[1], axle_local[0]]),
                                      q - target_l, com_local[0] - axle_local[0]))

    for side, name in enumerate(("left", "right")):
        q = np.array([entry[0] for entry in records[side]])
        actual = np.array([entry[1] for entry in records[side]])
        target_error = np.array([entry[2] for entry in records[side]])
        linear = np.array([L_NOM, SW_NOM]) + q @ J.T
        residual = linear - actual
        rng = np.random.default_rng(17).permutation(len(q))
        train, test = rng[::5], rng[1::5]
        coef = np.linalg.lstsq(features(q[test]), actual[test], rcond=None)[0]
        fitted = features(q[train]) @ coef
        print(f"{name}: range L={actual[:, 0].min():.4f}..{actual[:, 0].max():.4f} m, "
              f"sw={actual[:, 1].min():+.4f}..{actual[:, 1].max():+.4f} m")
        print(f"  joint target max error={np.max(np.abs(target_error), axis=0)} rad")
        print(f"  linear RMSE L/sw={np.sqrt(np.mean(residual**2, axis=0))}, "
              f"max={np.max(np.abs(residual), axis=0)}")
        print(f"  cubic holdout RMSE L/sw={np.sqrt(np.mean((fitted-actual[train])**2, axis=0))}, "
              f"max={np.max(np.abs(fitted-actual[train]), axis=0)}")
        print("  coefficients [L, sw] for 1,a,b,a2,ab,b2,a3,a2b,ab2,b3:")
        print(np.array2string(coef, precision=9, suppress_small=True))
        x = actual
        geom = np.column_stack((np.ones(len(x)), x[:, 0], x[:, 1],
                                x[:, 0]**2, x[:, 0]*x[:, 1], x[:, 1]**2))
        com_delta = np.array([entry[3] for entry in records[side]])
        trim_fit = np.linalg.lstsq(geom, com_delta, rcond=None)[0]
        print(f"  COM/axle fit max error={np.max(np.abs(geom @ trim_fit - com_delta)):.6f} m")
        for L in (0.055, 0.07, 0.09, 0.11, 0.117, 0.13, 0.15, 0.17, 0.19):
            def offset(sw):
                return np.array([1, L, sw, L*L, L*sw, sw*sw]) @ trim_fit
            if offset(-0.08) * offset(0.07) < 0:
                print(f"  L={L:.3f} m COM-aligned sw={brentq(offset, -0.08, 0.07):+.5f} m")


if __name__ == "__main__":
    main()

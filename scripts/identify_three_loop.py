#!/usr/bin/env python3
"""
三环基线 + 腿长全行程辨识
===============================================================
用线上真实三环控制器(通过 ident_bridge.so 调用 balance_update_frame)
作为稳定基线, 腿长作为输入控制量, 从最低到最高慢速扫描, 辨识
12 状态 x 4 输入的分段离散模型, 供 LQR+能量整形使用。

与 identify_discrete.py 的区别:
  - 基线不是简化 PD, 而是线上真实三环(俯仰PD+速度规划+偏航+roll稳定)。
  - 腿长激励从最低到最高全行程(慢三角波), 覆盖 leg_len 整个工作区。
  - 同时叠加小幅 PRBS 保证 wheel/leg_swing/leg_len_diff 通道可辨识。

用法:
  python3 scripts/identify_three_loop.py
输出:
  logs/three_loop_model_latest.json / .npz
"""

import ctypes
import json
import math
import os
import time

import mujoco
import numpy as np

os.environ["MUJOCO_GL"] = "osmesa"

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML = os.path.join(PROJECT_ROOT, "model", "COD-2026RoboMaster-Balance.xml")
BRIDGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ident_bridge.so")

SW_NOM = 0.23
LEG_KP = 160.0
LEG_KD = 2.5

IDX = {
    "pitch": 0, "pitch_rate": 1, "roll": 2, "roll_rate": 3,
    "leg_swing": 4, "leg_swing_rate": 5,
    "leg_len": 6, "leg_len_rate": 7,
    "leg_len_diff": 8, "leg_len_diff_rate": 9,
    "wheel_v": 10, "wheel_diff_v": 11,
    "wheel_v_comp": 12, "leg_kin_v": 13, "pitch_kin_v": 14,
    "imu_acc_x": 15, "true_forward_v": 16,
}

# 腿长全行程: 受四杆闭链约束, leg_len_ref 与实测 leg_len 有 ~0.066 偏移。
# ref=-0.15 -> 实测 -0.20(车最高); ref=+0.15 -> 实测 +0.05(车最低)。
LEG_LEN_MIN = float(os.environ.get("IDENT_LEG_MIN", "-0.15"))
LEG_LEN_MAX = float(os.environ.get("IDENT_LEG_MAX", "0.15"))
SWEEP_TIME = float(os.environ.get("IDENT_SWEEP_TIME", "30.0"))   # 单程扫描时长(s)
SWEEP_CYCLES = int(os.environ.get("IDENT_SWEEP_CYCLES", "3"))    # 来回次数
PRBS_WHEEL_AMP = float(os.environ.get("IDENT_PRBS_WHEEL", "0.15"))
PRBS_SWING_AMP = float(os.environ.get("IDENT_PRBS_SWING", "0.003"))
PRBS_DIFF_AMP = float(os.environ.get("IDENT_PRBS_DIFF", "0.010"))
PRBS_WHEELDIFF_AMP = float(os.environ.get("IDENT_PRBS_WHEELDIFF", "0.10"))
PRBS_HOLD_STEPS = 35
SETTLE_TIME = 5.0
Z_MIN, Z_MAX = 0.04, 0.35
PITCH_BIASES_DEG = [float(x) for x in
                    os.environ.get("IDENT_PITCH_BIASES_DEG",
                                   "0,2.5,-2.5,5.0").split(",") if x.strip()]
DECIM = 50   # 降采样加大: dt_eff=0.05s, 状态动态更明显, 减少 A 自回归吞掉输入


def pitch_from_quat(q):
    s = 2.0 * (q[0] * q[2] - q[1] * q[3])
    return math.asin(max(-1.0, min(1.0, s)))


def roll_from_quat(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class IdentFrame(ctypes.Structure):
    _fields_ = [
        ("pitch", ctypes.c_float), ("pitch_rate", ctypes.c_float),
        ("roll", ctypes.c_float), ("roll_rate", ctypes.c_float),
        ("left_wheel_vel", ctypes.c_float), ("right_wheel_vel", ctypes.c_float),
        ("acc_mag", ctypes.c_float), ("acc_x", ctypes.c_float), ("acc_z", ctypes.c_float),
        ("yaw", ctypes.c_float), ("yaw_rate", ctypes.c_float),
        ("left_wheel_pos", ctypes.c_float), ("right_wheel_pos", ctypes.c_float),
        ("lf_pos", ctypes.c_float), ("lr_pos", ctypes.c_float),
        ("rf_pos", ctypes.c_float), ("rr_pos", ctypes.c_float),
        ("lf_tau", ctypes.c_float), ("lr_tau", ctypes.c_float),
        ("rf_tau", ctypes.c_float), ("rr_tau", ctypes.c_float),
        ("left_wheel_tau", ctypes.c_float), ("right_wheel_tau", ctypes.c_float),
        ("lf_vel", ctypes.c_float), ("lr_vel", ctypes.c_float),
        ("rf_vel", ctypes.c_float), ("rr_vel", ctypes.c_float),
    ]


class IdentOut(ctypes.Structure):
    _fields_ = [
        ("wheel_tau", ctypes.c_float), ("yaw_tau", ctypes.c_float),
        ("leg_swing", ctypes.c_float), ("leg_extend", ctypes.c_float),
        ("leg_side_extend", ctypes.c_float),
        ("speed_ref", ctypes.c_float), ("position", ctypes.c_float),
    ]


def main():
    lib = ctypes.CDLL(BRIDGE)
    lib.ident_init()
    lib.ident_set_leg_len.argtypes = [ctypes.c_float]
    lib.ident_step.argtypes = [ctypes.POINTER(IdentFrame), ctypes.c_double,
                               ctypes.POINTER(IdentOut)]
    lib.ident_balance_active.restype = ctypes.c_int

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    home = model.key("home")

    leg_names = ["Left_front_joint", "Left_rear_joint", "Right_front_joint", "Right_rear_joint"]
    leg_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in leg_names]
    leg_qpos = [model.jnt_qposadr[j] for j in leg_ids]
    leg_dof = [model.jnt_dofadr[j] for j in leg_ids]
    leg_act = [0, 1, 3, 4]
    lw_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Left_Wheel_joint")
    rw_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Right_Wheel_joint")
    lw_dof = model.jnt_dofadr[lw_id]
    rw_dof = model.jnt_dofadr[rw_id]

    def leg_targets(swing_delta, length_delta, length_diff):
        left_len = length_delta + 0.5 * length_diff
        right_len = length_delta - 0.5 * length_diff
        sw = SW_NOM + swing_delta
        return np.array([
            sw - left_len,
            0.4 - sw - left_len,
            -0.1 - sw + right_len,
            -0.4 + sw + right_len,
        ])

    def setup(pitch_bias_deg):
        data.qpos[:] = home.qpos
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qpos[2] = 0.08
        pb = math.radians(pitch_bias_deg)
        data.qpos[3] = math.cos(pb / 2.0)
        data.qpos[4] = 0.0
        data.qpos[5] = math.sin(pb / 2.0)
        data.qpos[6] = 0.0
        for adr, val in zip(leg_qpos, leg_targets(0.0, 0.0, 0.0)):
            data.qpos[adr] = val
        mujoco.mj_forward(model, data)

    def read_meas():
        quat = data.sensordata[0:4]
        pitch = pitch_from_quat(quat)
        roll = roll_from_quat(quat)
        pitch_rate = float(data.sensordata[5])
        roll_rate = float(data.sensordata[4])

        lf_q, lr_q, rf_q, rr_q = [float(data.qpos[q]) for q in leg_qpos]
        lf_v, lr_v, rf_v, rr_v = [float(data.qvel[d]) for d in leg_dof]

        left_swing = 0.5 * (lf_q - lr_q + 0.4)
        right_swing = 0.5 * (rr_q - rf_q + 0.3)
        leg_swing = 0.5 * (left_swing + right_swing) - SW_NOM
        left_len = 0.5 * (0.4 - lf_q - lr_q)
        right_len = 0.5 * (rf_q + rr_q + 0.5)
        leg_len = 0.5 * (left_len + right_len)
        leg_len_diff = left_len - right_len

        left_swing_dot = 0.5 * (lf_v - lr_v)
        right_swing_dot = 0.5 * (rr_v - rf_v)
        leg_swing_rate = 0.5 * (left_swing_dot + right_swing_dot)
        left_ext_dot = -0.5 * (lf_v + lr_v)
        right_ext_dot = 0.5 * (rf_v + rr_v)
        leg_len_rate = 0.5 * (left_ext_dot + right_ext_dot)
        leg_len_diff_rate = left_ext_dot - right_ext_dot

        LEG_AXLE_DX_DSW = -0.15842
        LEG_AXLE_DX_DEL = -0.07492
        LEG_AXLE_DX_DER = -0.08349
        LEG_AXLE_Z_NOM = -0.20842
        leg_kin_v = (LEG_AXLE_DX_DSW * leg_swing_rate
                     + LEG_AXLE_DX_DEL * left_ext_dot
                     + LEG_AXLE_DX_DER * right_ext_dot)
        pitch_kin_v = LEG_AXLE_Z_NOM * pitch_rate

        left_wheel_v = data.qvel[lw_dof] * 0.058
        right_wheel_v = -data.qvel[rw_dof] * 0.058
        wheel_v = 0.5 * (left_wheel_v + right_wheel_v)
        wheel_diff_v = 0.5 * (left_wheel_v - right_wheel_v)
        wheel_v_comp = wheel_v - leg_kin_v - pitch_kin_v

        cp, sp = math.cos(pitch), math.sin(pitch)
        imu_acc_x = cp * float(data.sensordata[7]) + sp * float(data.sensordata[9]) - 9.81 * sp

        yaw = yaw_from_quat(quat)
        true_forward_v = float(data.qvel[0]) * math.cos(yaw) + float(data.qvel[1]) * math.sin(yaw)

        return np.array([
            pitch, pitch_rate, roll, roll_rate,
            leg_swing, leg_swing_rate,
            leg_len, leg_len_rate,
            leg_len_diff, leg_len_diff_rate,
            wheel_v, wheel_diff_v,
            wheel_v_comp, leg_kin_v, pitch_kin_v,
            imu_acc_x, true_forward_v,
        ])

    def fill_frame(meas):
        quat = data.sensordata[0:4]
        return IdentFrame(
            pitch=meas[IDX["pitch"]], pitch_rate=meas[IDX["pitch_rate"]],
            roll=meas[IDX["roll"]], roll_rate=meas[IDX["roll_rate"]],
            left_wheel_vel=data.qvel[lw_dof], right_wheel_vel=data.qvel[rw_dof],
            acc_mag=float(np.sqrt(data.sensordata[7]**2 + data.sensordata[9]**2)),
            acc_x=float(data.sensordata[7]), acc_z=float(data.sensordata[9]),
            yaw=yaw_from_quat(quat), yaw_rate=float(data.sensordata[6]),
            left_wheel_pos=float(data.qpos[lw_dof * 0 + model.jnt_qposadr[lw_id]]),
            right_wheel_pos=float(data.qpos[model.jnt_qposadr[rw_id]]),
            lf_pos=float(data.qpos[leg_qpos[0]]), lr_pos=float(data.qpos[leg_qpos[1]]),
            rf_pos=float(data.qpos[leg_qpos[2]]), rr_pos=float(data.qpos[leg_qpos[3]]),
            lf_tau=float(data.actuator_force[0]), lr_tau=float(data.actuator_force[1]),
            rf_tau=float(data.actuator_force[3]), rr_tau=float(data.actuator_force[4]),
            left_wheel_tau=float(data.actuator_force[2] * 10.0),
            right_wheel_tau=float(data.actuator_force[5] * 10.0),
            lf_vel=float(data.qvel[leg_dof[0]]), lr_vel=float(data.qvel[leg_dof[1]]),
            rf_vel=float(data.qvel[leg_dof[2]]), rr_vel=float(data.qvel[leg_dof[3]]),
        )

    rng = np.random.default_rng(42)
    sigs, us, zs, xpos = [], [], [], []
    n_sweep = int(SWEEP_TIME / dt)
    t0 = time.time()
    print(f"三环基线辨识: 腿长 [{LEG_LEN_MIN},{LEG_LEN_MAX}] rad 扫描 {SWEEP_CYCLES} 次, "
          f"pitch偏置={PITCH_BIASES_DEG}°, wheel PRBS={PRBS_WHEEL_AMP}Nm")

    for bias_deg in PITCH_BIASES_DEG:
        setup(bias_deg)
        # 先让三环稳定到当前腿长基准
        lib.ident_set_leg_len(ctypes.c_float(0.0))
        for _ in range(int(SETTLE_TIME / dt)):
            meas = read_meas()
            frame = fill_frame(meas)
            out = IdentOut()
            lib.ident_step(ctypes.byref(frame), dt, ctypes.byref(out))
            # 腿伺服执行三环腿摆参考 + 三环内部 leg_extend 基线
            targets = leg_targets(out.leg_swing - SW_NOM, out.leg_extend, 0.0)
            for i, (qadr, dadr) in enumerate(zip(leg_qpos, leg_dof)):
                tau = LEG_KP * (targets[i] - data.qpos[qadr]) - LEG_KD * data.qvel[dadr]
                data.ctrl[leg_act[i]] = np.clip(tau / 50.0, -10.0, 10.0)
            data.ctrl[2] = np.clip(out.wheel_tau / 10.0, -10.0, 10.0)
            data.ctrl[5] = np.clip(-out.wheel_tau / 10.0, -10.0, 10.0)
            mujoco.mj_step(model, data)

        prbs_wheel = 0.0
        prbs_swing = 0.0
        prbs_diff = 0.0
        prbs_wheeldiff = 0.0
        prbs_leg = 0.0
        for cyc in range(SWEEP_CYCLES):
            for step in range(n_sweep):
                # 前 80%: 全行程三角波; 后 20%: 停在正腿长段(+0.05..+0.15)慢扫,
                # 补足"车最低"方向的稀疏样本。
                frac = step / n_sweep
                if frac < 0.8:
                    tri = 2.0 * abs(2.0 * ((cyc + step / n_sweep) % 1.0) - 1.0) - 1.0
                    leg_len_ref = 0.5 * (LEG_LEN_MAX + LEG_LEN_MIN) + \
                                  0.5 * tri * (LEG_LEN_MAX - LEG_LEN_MIN)
                else:
                    pos_frac = (frac - 0.8) / 0.2
                    tri2 = 2.0 * abs(2.0 * pos_frac - 1.0) - 1.0
                    lo_pos = 0.05
                    leg_len_ref = 0.5 * (LEG_LEN_MAX + lo_pos) + \
                                  0.5 * tri2 * (LEG_LEN_MAX - lo_pos)

                if step % PRBS_HOLD_STEPS == 0:
                    # 正腿长段(车最低)激励减半, 避免叠加失稳
                    scale = 0.5 if leg_len_ref > 0.05 else 1.0
                    prbs_wheel = scale * (PRBS_WHEEL_AMP if rng.random() > 0.5 else -PRBS_WHEEL_AMP)
                    prbs_swing = scale * (PRBS_SWING_AMP if rng.random() > 0.5 else -PRBS_SWING_AMP)
                    prbs_diff = scale * (PRBS_DIFF_AMP if rng.random() > 0.5 else -PRBS_DIFF_AMP)
                    prbs_wheeldiff = scale * (PRBS_WHEELDIFF_AMP if rng.random() > 0.5 else -PRBS_WHEELDIFF_AMP)
                    # 腿长高频抖动: 在慢三角波上叠加 PRBS, 让 leg_len_rate
                    # 真正被激励(纯慢扫时 leg_len≈ref, 输入被自回归吞掉)。
                    prbs_leg = scale * (0.03 if rng.random() > 0.5 else -0.03)

                meas = read_meas()
                frame = fill_frame(meas)
                out = IdentOut()
                # 防侧翻: |roll|>0.5rad(28°) 时腿长回中, 让车先回正再继续
                if abs(meas[IDX["roll"]]) > 0.5:
                    leg_len_inj = 0.0
                else:
                    leg_len_inj = float(np.clip(leg_len_ref + prbs_leg,
                                                LEG_LEN_MIN, LEG_LEN_MAX))
                lib.ident_set_leg_len(ctypes.c_float(leg_len_inj))
                lib.ident_step(ctypes.byref(frame), dt, ctypes.byref(out))

                # 三环输出 leg_swing(绝对) + 腿长扫描 + PRBS diff
                # 腿长参考 = 三环内部 leg_extend 基线 + 辨识注入 leg_len_ref,
                # 这样三环保持平衡, 注入的腿长变化才是真正的辨识输入。
                targets = leg_targets(out.leg_swing - SW_NOM,
                                      out.leg_extend + leg_len_inj,
                                      out.leg_side_extend + prbs_diff)
                for i, (qadr, dadr) in enumerate(zip(leg_qpos, leg_dof)):
                    tau = LEG_KP * (targets[i] - data.qpos[qadr]) - LEG_KD * data.qvel[dadr]
                    data.ctrl[leg_act[i]] = np.clip(tau / 50.0, -10.0, 10.0)
                # 轮扭矩混控与真实控制器一致:
                #   ctrl[2] = (pitch_tau - yaw_tau)/10
                #   ctrl[5] = -(pitch_tau + yaw_tau)/10
                # 其中 pitch_tau = 三环共模 + PRBS 共模激励,
                #      yaw_tau   = 三环偏航 + PRBS 差速激励。
                pitch_tau = out.wheel_tau + prbs_wheel
                yaw_tau_cmd = out.yaw_tau + prbs_wheeldiff
                data.ctrl[2] = np.clip((pitch_tau - yaw_tau_cmd) / 10.0, -10.0, 10.0)
                data.ctrl[5] = np.clip(-(pitch_tau + yaw_tau_cmd) / 10.0, -10.0, 10.0)
                mujoco.mj_step(model, data)

                if step % DECIM == 0:
                    meas = read_meas()
                    z = float(data.qpos[2])
                    # 稳定性门控: 只保留姿态/高度在有效范围的样本
                    ok = (abs(meas[IDX["pitch"]]) < math.radians(6.0) and
                          abs(meas[IDX["roll"]]) < math.radians(8.0) and
                          Z_MIN < z < Z_MAX)
                    sigs.append(meas)
                    # 输入 = [共模PRBS, 差速PRBS, 腿摆PRBS, 腿长参考, 腿长差PRBS]
                    us.append([prbs_wheel, prbs_wheeldiff, prbs_swing, leg_len_inj, prbs_diff])
                    zs.append(z)
                    xpos.append(float(data.qpos[0]))
                    if not ok:
                        # 记录但不用于拟合(后续 fit 时按 z/pitch/roll 过滤)
                        zs[-1] = np.nan
            print(f"  扫描完成: bias={bias_deg:+.1f}° cycle={cyc+1}/{SWEEP_CYCLES} "
                  f"leg_len∈[{np.min([s[IDX['leg_len']] for s in sigs[-n_sweep//DECIM:]]):+.3f},"
                  f"{np.max([s[IDX['leg_len']] for s in sigs[-n_sweep//DECIM:]]):+.3f}]")

    SENS = np.asarray(sigs)
    U = np.asarray(us)
    Z = np.asarray(zs)
    XPOS = np.asarray(xpos)
    print(f"\n采集完成: {len(SENS)} 样本, 用时 {time.time()-t0:.1f}s")
    print(f"leg_len 覆盖: [{SENS[:, IDX['leg_len']].min():+.4f}, "
          f"{SENS[:, IDX['leg_len']].max():+.4f}] rad")
    print(f"wheel_v 覆盖: [{SENS[:, IDX['wheel_v']].min():+.2f}, "
          f"{SENS[:, IDX['wheel_v']].max():+.2f}] m/s")
    print(f"z 覆盖: [{Z.min():.3f}, {Z.max():.3f}] m")

    # ---- 拟合: 12 状态 x 4 输入 compact 模型 ----
    dt_eff = dt * DECIM
    compact_cols = [
        IDX["pitch"], IDX["pitch_rate"],
        IDX["roll"], IDX["roll_rate"],
        IDX["leg_swing"], IDX["leg_swing_rate"],
        IDX["leg_len"], IDX["leg_len_rate"],
        IDX["leg_len_diff"], IDX["leg_len_diff_rate"],
        IDX["wheel_v"], IDX["wheel_diff_v"],
    ]
    compact_names = [
        "pitch", "pitch_rate", "roll", "roll_rate",
        "leg_swing", "leg_swing_rate", "leg_len", "leg_len_rate",
        "leg_len_diff", "leg_len_diff_rate", "wheel_v", "wheel_diff_v",
    ]
    input_names = ["wheel_common_tau", "wheel_diff_tau", "leg_swing_ref",
               "leg_len_ref", "leg_len_diff_ref"]

    valid = np.isfinite(Z) & (np.abs(SENS[:, IDX["pitch"]]) < math.radians(6.0)) & \
            (np.abs(SENS[:, IDX["roll"]]) < math.radians(8.0))
    X = SENS[:, compact_cols]
    n, m = len(compact_cols), U.shape[1]
    idx = np.where(valid[:-1] & valid[1:])[0]
    print(f"\n拟合样本: {len(idx)} 有效转换 (共 {len(SENS)})")
    if len(idx) < 200:
        print("!!! 有效样本太少, 模型不可靠")

    Xk, Xn, Uk = X[idx], X[idx + 1], U[idx]
    Phi = np.hstack([Xk, Uk, np.ones((len(idx), 1))])
    theta, _, _, _ = np.linalg.lstsq(Phi, Xn, rcond=None)
    A, B, c = theta[:n, :].T, theta[n:n + m, :].T, theta[n + m, :]
    pred = Phi @ theta
    ss_res = np.sum((Xn - pred) ** 2, axis=0)
    ss_tot = np.sum((Xn - np.mean(Xn, axis=0)) ** 2, axis=0)
    r2 = 1.0 - ss_res / np.maximum(ss_tot, 1e-12)
    poles = np.linalg.eigvals(A)
    print("A=\n", np.array2string(A, precision=5, suppress_small=True))
    print("B=\n", np.array2string(B, precision=5, suppress_small=True))
    print("R2=", np.array2string(r2, precision=3))
    print("leg_len_ref 列 B[6:8,2]=", np.array2string(B[6:8, 2], precision=4))

    # LQR 权重: 腿长代价降低, 让 LQR 真正使用腿长通道
    q_diag = np.array([1200.0, 16.0, 600.0, 12.0, 120.0, 2.0, 80.0, 1.5, 160.0, 2.0, 2.0, 1.0])
    r_diag = np.array([0.20, 1.0, 50.0, 12.0, 6.0])
    Q = np.diag(q_diag)
    R = np.diag(r_diag)
    P = Q.copy()
    for _ in range(5000):
        Pn = Q + A.T @ P @ A - A.T @ P @ B @ np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        if np.linalg.norm(Pn - P, "fro") < 1e-10:
            P = Pn
            break
        P = Pn
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    print("LQR K(4x12)=", np.array2string(K, precision=3, suppress_small=True))

    # 按腿长分段拟合(每 0.05 rad 一段)
    leg_len_bins = np.arange(-0.30, 0.11, 0.04)
    piecewise = {}
    leg_len_all = SENS[:, IDX["leg_len"]]
    for bi in range(len(leg_len_bins) - 1):
        lo, hi = leg_len_bins[bi], leg_len_bins[bi + 1]
        key = f"leglen_{lo:+.2f}_{hi:+.2f}"
        seg = valid & (leg_len_all >= lo) & (leg_len_all < hi)
        sidx = np.where(seg[:-1] & seg[1:])[0]
        if len(sidx) < 60:
            piecewise[key] = {"model": None, "range": [lo, hi], "samples": int(seg.sum())}
            continue
        Xks, Xns, Uks = X[sidx], X[sidx + 1], U[sidx]
        Phis = np.hstack([Xks, Uks, np.ones((len(sidx), 1))])
        th, _, _, _ = np.linalg.lstsq(Phis, Xns, rcond=None)
        As, Bs, cs = th[:n, :].T, th[n:n + m, :].T, th[n + m, :]
        Ps = Q.copy()
        for _ in range(5000):
            Pns = Q + As.T @ Ps @ As - As.T @ Ps @ Bs @ np.linalg.solve(R + Bs.T @ Ps @ Bs, Bs.T @ Ps @ As)
            if np.linalg.norm(Pns - Ps, "fro") < 1e-10:
                Ps = Pns
                break
            Ps = Pns
        Ks = np.linalg.solve(R + Bs.T @ Ps @ Bs, Bs.T @ Ps @ As)
        trim = np.mean(X[seg], axis=0)
        piecewise[key] = {
            "model": {
                "A": As.tolist(), "B": Bs.tolist(), "c": cs.tolist(),
                "K_lqr": Ks.tolist(), "x_trim": trim.tolist(),
            },
            "range": [lo, hi], "samples": int(seg.sum()),
        }
        print(f"  {key}: samples={int(seg.sum())} B[6,2]={Bs[6,2]:+.4f} B[7,2]={Bs[7,2]:+.4f}")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(PROJECT_ROOT, "logs")
    os.makedirs(out_dir, exist_ok=True)
    payload = {
        "generated_at": stamp,
        "xml": XML,
        "dt_physics": dt,
        "dt_model": dt_eff,
        "decimation": DECIM,
        "leg_len_range": [LEG_LEN_MIN, LEG_LEN_MAX],
        "state_names": compact_names,
        "input_names": input_names,
        "state_definition": "x_dev = x_measured - x_trim; u_extra is perturbation around three-loop baseline",
        "A": A.tolist(), "B": B.tolist(), "c": c.tolist(),
        "K_lqr": K.tolist(), "r2": r2.tolist(),
        "poles_discrete_real": np.real(poles).tolist(),
        "poles_discrete_imag": np.imag(poles).tolist(),
        "x_trim": np.mean(X[valid], axis=0).tolist(),
        "u_extra_trim": [0.0, 0.0, 0.0, 0.0],
        "samples": len(SENS), "valid_samples": int(valid.sum()),
        "piecewise": piecewise,
    }
    with open(os.path.join(out_dir, "three_loop_model_latest.json"), "w") as f:
        json.dump(payload, f, indent=2)
    np.savez(os.path.join(out_dir, "three_loop_model_latest.npz"),
             sens=SENS, u=U, z=Z, xpos=XPOS,
             A=A, B=B, c=c, K=K,
             state_names=compact_names, input_names=input_names)
    print(f"\n模型 -> logs/three_loop_model_latest.json / .npz")


if __name__ == "__main__":
    main()

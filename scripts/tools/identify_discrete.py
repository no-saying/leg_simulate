#!/usr/bin/env python3
"""
轮腿平衡车的小角度闭环辨识脚本。

用途:
  1. 先用一个只依赖现实传感器的 PD 控制器把车带到 +/-5deg 附近。
  2. 在稳定工作点附近注入小幅 PRBS。
  3. 拟合给 LQR/MPC 使用的局部离散模型。

注意:
  这个模型只服务小角度精调，不负责大角度恢复。大角度仍由 PD/能量整形保底。

辨识状态只使用现实运行可得到的信号:
  IMU quat/gyro/accel、轮编码器、腿关节编码器/速度。

世界位置/高度/速度只用于剔除无效样本和报告质量，不作为模型状态或实时控制输入。
"""

import math
import json
import os
import time

import mujoco
import numpy as np

os.environ["MUJOCO_GL"] = "osmesa"

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML = os.path.join(PROJECT_ROOT, "model", "COD-2026RoboMaster-Balance.xml")

# 必须和 control/balance.h 的 BALANCE_LEG_SW_NOM 保持一致，否则辨识 trim 会偏离当前控制器。
SW_NOM = 0.23
LEG_TARGETS = np.array([SW_NOM, 0.4 - SW_NOM, -0.1 - SW_NOM, -0.4 + SW_NOM])
WHEEL_RADIUS = 0.058
# 当前控制器的 simple 路径约定: 正 wheel_v 对应正 body-x 前进。
# 这里必须与控制器一致，否则辨识会把速度阻尼和真实运动方向学反。
WHEEL_ODOM_SIGN = 1.0

# 与控制器诊断一致的腿部运动学线性系数。
# 系数来自模型标定，但实时计算只用关节编码器/速度、轮编码器和 IMU。
LEG_AXLE_DX_DSW = -0.15842
LEG_AXLE_DX_DEL = -0.07492
LEG_AXLE_DX_DER = -0.08349
LEG_AXLE_Z_NOM = -0.20842

IDX = {
    "pitch": 0,
    "pitch_rate": 1,
    "roll": 2,
    "roll_rate": 3,
    "leg_swing": 4,
    "leg_swing_rate": 5,
    "leg_len": 6,
    "leg_len_rate": 7,
    "leg_len_diff": 8,
    "leg_len_diff_rate": 9,
    "wheel_v": 10,
    "wheel_diff_v": 11,
    "wheel_v_comp": 12,
    "leg_kin_v": 13,
    "pitch_kin_v": 14,
    "imu_acc_x": 15,
    "true_forward_v": 16,
}

# 只用于采集局部数据的稳定 PD，不是最终控制器。
PD_KP = 325.0
PD_KD = 58.0
PD_KV = float(os.environ.get("IDENT_PD_KV", "0.0"))
PD_TAU_LIMIT = 145.0

# 辨识采集用的零速/原点保持基线。
# 它只用轮编码器积分出来的局部 odom 和轮速，不使用世界位置。
IDENT_SPEED_PLAN_ENABLE = int(os.environ.get("IDENT_SPEED_PLAN_ENABLE", "0"))
IDENT_SPEED_PLAN_POS_KP = float(os.environ.get("IDENT_SPEED_PLAN_POS_KP", "0.08"))
IDENT_SPEED_PLAN_PITCH_KP = float(os.environ.get("IDENT_SPEED_PLAN_PITCH_KP", "0.030"))
IDENT_SPEED_PLAN_PITCH_MAX = float(os.environ.get("IDENT_SPEED_PLAN_PITCH_MAX", "0.060"))

LEG_KP = 160.0
LEG_KD = 2.5
PITCH_VALID_DEG = 5.0
ROLL_VALID_DEG = 10.0
WHEEL_VALID_V_MAX = float(os.environ.get("IDENT_GATE_WHEEL_V_MAX", "6.0"))      # m/s
IMU_VALID_ACC_MAX = float(os.environ.get("IDENT_GATE_IMU_ACC_MAX", "45.0"))     # m/s^2

PRBS_WHEEL_AMP = float(os.environ.get("IDENT_PRBS_WHEEL", "0.25"))      # Nm
PRBS_SWING_AMP = float(os.environ.get("IDENT_PRBS_SWING", "0.002"))     # rad
PRBS_LEN_AMP = float(os.environ.get("IDENT_PRBS_LEN", "0.001"))         # rad
PRBS_LEN_DIFF_AMP = float(os.environ.get("IDENT_PRBS_DIFF", "0.001"))   # rad
PRBS_HOLD_STEPS = 35
SETTLE_TIME = 4.0
EXCITE_TIME = 12.0
DECIM = 10
PITCH_BIASES_DEG = [
    float(x.strip()) for x in os.environ.get("IDENT_PITCH_BIASES_DEG", "0,2.5,-2.5,4.2,-4.2").split(",")
    if x.strip()
]
# 为后续腿长变换准备的虚拟腿长基线（rad，正值 = 腿更短/车更低，见 leg_targets_from_virtual）。
# 每个基线都会配全套 pitch 偏置采集，并按实测 leg_len 分桶拟合各自的分段模型。
LEG_LEN_BIASES = [
    float(x.strip()) for x in os.environ.get("IDENT_LEG_LEN_BIASES", "-0.06,-0.03,0.0,0.03,0.06").split(",")
    if x.strip()
]


def jnt(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)


def pitch_from_quat(q):
    s = 2.0 * (q[0] * q[2] - q[1] * q[3])
    return math.asin(max(-1.0, min(1.0, s)))


def roll_from_quat(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def solve_dare_iter(A, B, Q, R, max_iter=5000, tol=1e-10):
    P = Q.copy()
    for _ in range(max_iter):
        P_new = Q + A.T @ P @ A - A.T @ P @ B @ np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        if np.linalg.norm(P_new - P, "fro") < tol:
            P = P_new
            break
        P = P_new
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    return K


def main():
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    home = model.key("home")

    leg_names = ["Left_front_joint", "Left_rear_joint", "Right_front_joint", "Right_rear_joint"]
    leg_ids = [jnt(model, n) for n in leg_names]
    leg_qpos = [model.jnt_qposadr[j] for j in leg_ids]
    leg_dof = [model.jnt_dofadr[j] for j in leg_ids]
    leg_act = [0, 1, 3, 4]

    lw_id = jnt(model, "Left_Wheel_joint")
    rw_id = jnt(model, "Right_Wheel_joint")
    lw_dof = model.jnt_dofadr[lw_id]
    rw_dof = model.jnt_dofadr[rw_id]

    def setup(initial_pitch=0.0, leg_len_bias=0.0):
        data.qpos[:] = home.qpos
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qpos[2] = 0.08
        data.qpos[3] = math.cos(initial_pitch / 2.0)
        data.qpos[4] = 0.0
        data.qpos[5] = math.sin(initial_pitch / 2.0)
        data.qpos[6] = 0.0
        for adr, val in zip(leg_qpos, leg_targets_from_virtual(0.0, leg_len_bias, 0.0)):
            data.qpos[adr] = val
        mujoco.mj_forward(model, data)

    def leg_targets_from_virtual(swing_delta=0.0, length_delta=0.0, length_diff=0.0):
        left_len = length_delta + 0.5 * length_diff
        right_len = length_delta - 0.5 * length_diff
        sw = SW_NOM + swing_delta
        return np.array([
            sw - left_len,
            0.4 - sw - left_len,
            -0.1 - sw + right_len,
            -0.4 + sw + right_len,
        ])

    def leg_control(targets=LEG_TARGETS):
        for i, (qadr, dadr) in enumerate(zip(leg_qpos, leg_dof)):
            tau = LEG_KP * (targets[i] - data.qpos[qadr]) - LEG_KD * data.qvel[dadr]
            data.ctrl[leg_act[i]] = np.clip(tau / 50.0, -10.0, 10.0)

    def wheel_control(common_tau):
        tau = np.clip(common_tau, -PD_TAU_LIMIT, PD_TAU_LIMIT)
        data.ctrl[2] = tau / 10.0
        data.ctrl[5] = -tau / 10.0
        return tau

    def baseline_pitch_ref(pitch_bias, wheel_v, odom_s):
        if not IDENT_SPEED_PLAN_ENABLE:
            return pitch_bias
        v_ref = -IDENT_SPEED_PLAN_POS_KP * odom_s
        v_err = wheel_v - v_ref
        neutral = pitch_bias - IDENT_SPEED_PLAN_PITCH_KP * v_err
        neutral = max(pitch_bias - IDENT_SPEED_PLAN_PITCH_MAX,
                      min(pitch_bias + IDENT_SPEED_PLAN_PITCH_MAX, neutral))
        return neutral

    def baseline_tau_from_state(x, pitch_bias, odom_s):
        pitch_ref = baseline_pitch_ref(pitch_bias, x[2], odom_s)
        return PD_KP * (x[0] - pitch_ref) + PD_KD * x[1] - PD_KV * x[2], pitch_ref

    def measured_signals():
        quat = data.sensordata[0:4]
        pitch = pitch_from_quat(data.sensordata[0:4])
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

        leg_kin_v = (
            LEG_AXLE_DX_DSW * leg_swing_rate
            + LEG_AXLE_DX_DEL * left_ext_dot
            + LEG_AXLE_DX_DER * right_ext_dot
        )
        pitch_kin_v = LEG_AXLE_Z_NOM * pitch_rate
        left_wheel_v = WHEEL_ODOM_SIGN * data.qvel[lw_dof] * WHEEL_RADIUS
        right_wheel_v = -WHEEL_ODOM_SIGN * data.qvel[rw_dof] * WHEEL_RADIUS
        wheel_v = 0.5 * (left_wheel_v + right_wheel_v)
        wheel_diff_v = 0.5 * (left_wheel_v - right_wheel_v)
        wheel_v_comp = wheel_v - leg_kin_v - pitch_kin_v

        cp, sp = math.cos(pitch), math.sin(pitch)
        imu_acc_x = cp * float(data.sensordata[7]) + sp * float(data.sensordata[9]) - 9.81 * sp

        # Validation labels only: not a runtime sensor and never fed to control.
        yaw = yaw_from_quat(data.sensordata[0:4])
        true_forward_v = float(data.qvel[0]) * math.cos(yaw) + float(data.qvel[1]) * math.sin(yaw)

        return {
            "vec": np.array(
                [
                    pitch,
                    pitch_rate,
                    roll,
                    roll_rate,
                    leg_swing,
                    leg_swing_rate,
                    leg_len,
                    leg_len_rate,
                    leg_len_diff,
                    leg_len_diff_rate,
                    wheel_v,
                    wheel_diff_v,
                    wheel_v_comp,
                    leg_kin_v,
                    pitch_kin_v,
                    imu_acc_x,
                    true_forward_v,
                ],
                dtype=float,
            ),
            "z": float(data.qpos[2]),
            "x": float(data.qpos[0]),
            "y": float(data.qpos[1]),
        }

    def measured_state():
        v = measured_signals()["vec"]
        return np.array([v[IDX["pitch"]], v[IDX["pitch_rate"]], v[IDX["wheel_v"]]], dtype=float)

    rng = np.random.default_rng(42)
    n_steps = int(EXCITE_TIME / dt)
    sigs = []
    us = []
    leg_segs = []
    leg_trim_per_sample = []
    zs = []
    xpos = []
    valid_small = []
    segment_trims = []
    t0 = time.time()
    print(
        f"Running small-signal closed-loop PRBS: {EXCITE_TIME:.1f}s per (leg_len_bias x pitch_bias) segment, "
        f"wheel=+/-{PRBS_WHEEL_AMP:.1f}Nm swing=+/-{PRBS_SWING_AMP:.4f}rad "
        f"len=+/-{PRBS_LEN_AMP:.4f}rad diff=+/-{PRBS_LEN_DIFF_AMP:.4f}rad, "
        f"pitch_biases={PITCH_BIASES_DEG}, leg_len_biases={LEG_LEN_BIASES}, "
        f"speed_plan={IDENT_SPEED_PLAN_ENABLE}, "
        f"record every {DECIM} steps"
    )
    for leg_seg_i, leg_len_bias in enumerate(LEG_LEN_BIASES):
        leg_baseline_targets = leg_targets_from_virtual(0.0, leg_len_bias, 0.0)
        for bias_deg in PITCH_BIASES_DEG:
            pitch_bias = math.radians(bias_deg)
            setup(pitch_bias, leg_len_bias)
            print(f"Settling closed loop for {SETTLE_TIME:.1f}s at leg_len_bias={leg_len_bias:+.3f} pitch_bias={bias_deg:+.2f}deg...")
            max_settle_pitch = 0.0
            min_z = 1e9
            settle_odom_s = 0.0
            for _ in range(int(SETTLE_TIME / dt)):
                x = measured_state()
                max_settle_pitch = max(max_settle_pitch, abs(x[0]))
                min_z = min(min_z, float(data.qpos[2]))
                settle_odom_s += x[2] * dt
                leg_control(leg_baseline_targets)
                base_tau, _pitch_ref = baseline_tau_from_state(x, pitch_bias, settle_odom_s)
                wheel_control(base_tau)
                mujoco.mj_step(model, data)

            sig0 = measured_signals()
            x0 = sig0["vec"]
            seg_trim = x0.copy()
            segment_trims.append(seg_trim)
            print(
                "Settled: pitch={:+.2f}deg roll={:+.2f}deg pitch_rate={:+.3f} roll_rate={:+.3f} "
                "leg_len={:+.4f} wheel_v={:+.3f}m/s z={:.3f} x={:+.3f}".format(
                    math.degrees(x0[IDX["pitch"]]), math.degrees(x0[IDX["roll"]]),
                    x0[IDX["pitch_rate"]], x0[IDX["roll_rate"]], x0[IDX["leg_len"]],
                    x0[IDX["wheel_v"]], sig0["z"], sig0["x"]
                )
            )
            print(f"Settle quality: max|pitch|={math.degrees(max_settle_pitch):.2f}deg min_z={min_z:.3f}")
            if min_z < 0.04 or abs(x0[IDX["pitch"]]) > math.radians(PITCH_VALID_DEG + 1.0):
                print("  settle outside preferred small-signal gate; keeping only valid samples from this segment")

            prbs_wheel = 0.0
            prbs_swing = 0.0
            prbs_len = 0.0
            prbs_len_diff = 0.0
            odom_s = 0.0
            for step in range(n_steps):
                if step % PRBS_HOLD_STEPS == 0:
                    prbs_wheel = PRBS_WHEEL_AMP if rng.random() > 0.5 else -PRBS_WHEEL_AMP
                    prbs_swing = PRBS_SWING_AMP if rng.random() > 0.5 else -PRBS_SWING_AMP
                    prbs_len = PRBS_LEN_AMP if rng.random() > 0.5 else -PRBS_LEN_AMP
                    prbs_len_diff = PRBS_LEN_DIFF_AMP if rng.random() > 0.5 else -PRBS_LEN_DIFF_AMP

                x = measured_state()
                odom_s += x[2] * dt
                leg_ref = leg_targets_from_virtual(prbs_swing, leg_len_bias + prbs_len, prbs_len_diff)
                leg_control(leg_ref)
                base_tau, _pitch_ref = baseline_tau_from_state(x, pitch_bias, odom_s)
                actual_tau = wheel_control(base_tau + prbs_wheel)
                mujoco.mj_step(model, data)

                if step % DECIM == 0:
                    meas = measured_signals()
                    xr = meas["vec"]
                    sigs.append(xr)
                    us.append([actual_tau - base_tau, prbs_swing, prbs_len, prbs_len_diff])
                    leg_segs.append(leg_seg_i)
                    leg_trim_per_sample.append(seg_trim[IDX["leg_len"]])
                    zs.append(meas["z"])
                    xpos.append(meas["x"])
                    ok_small = (
                        abs(xr[IDX["pitch"]]) < math.radians(PITCH_VALID_DEG)
                        and abs(xr[IDX["roll"]]) < math.radians(ROLL_VALID_DEG)
                        and abs(xr[IDX["pitch_rate"]]) < 3.0
                        and abs(xr[IDX["roll_rate"]]) < 6.0
                        and abs(xr[IDX["wheel_v"]]) < WHEEL_VALID_V_MAX
                        and abs(xr[IDX["wheel_v_comp"]]) < WHEEL_VALID_V_MAX
                        and abs(xr[IDX["imu_acc_x"]]) < IMU_VALID_ACC_MAX
                        and abs(xr[IDX["leg_swing"]] - seg_trim[IDX["leg_swing"]]) < 0.040
                        and abs(xr[IDX["leg_len"]] - seg_trim[IDX["leg_len"]]) < 0.030
                        and abs(xr[IDX["leg_len_diff"]] - seg_trim[IDX["leg_len_diff"]]) < 0.160
                        and 0.04 < meas["z"] < 0.35
                    )
                    valid_small.append(ok_small)
    print(f"Done in {time.time() - t0:.2f}s")

    SENS = np.asarray(sigs)
    U = np.asarray(us)
    LEG_SEGS = np.asarray(leg_segs, dtype=int)
    LEG_TRIM_PER_SAMPLE = np.asarray(leg_trim_per_sample)
    Z = np.asarray(zs)
    XPOS = np.asarray(xpos)
    valid_small = np.asarray(valid_small, dtype=bool)
    if int(valid_small.sum()) > 0:
        trim = np.mean(SENS[valid_small], axis=0)
    else:
        trim = np.mean(np.asarray(segment_trims), axis=0)
    DEV = SENS - trim
    pitch_acc = np.zeros(len(SENS))
    roll_acc = np.zeros(len(SENS))
    if len(SENS) > 1:
        pitch_acc[:-1] = np.diff(SENS[:, IDX["pitch_rate"]]) / (dt * DECIM)
        roll_acc[:-1] = np.diff(SENS[:, IDX["roll_rate"]]) / (dt * DECIM)
        pitch_acc[-1] = pitch_acc[-2]
        roll_acc[-1] = roll_acc[-2]
    print(
        "Data quality: samples={} small_valid={} max|pitch|={:.2f}deg max|roll|={:.2f}deg "
        "wheel_v=[{:+.2f},{:+.2f}] comp_v=[{:+.2f},{:+.2f}] true_v=[{:+.2f},{:+.2f}] "
        "z=[{:.3f},{:.3f}] x=[{:+.3f},{:+.3f}]".format(
            len(SENS), int(valid_small.sum()),
            math.degrees(np.max(np.abs(SENS[:, IDX["pitch"]]))),
            math.degrees(np.max(np.abs(SENS[:, IDX["roll"]]))),
            float(np.min(SENS[:, IDX["wheel_v"]])), float(np.max(SENS[:, IDX["wheel_v"]])),
            float(np.min(SENS[:, IDX["wheel_v_comp"]])), float(np.max(SENS[:, IDX["wheel_v_comp"]])),
            float(np.min(SENS[:, IDX["true_forward_v"]])), float(np.max(SENS[:, IDX["true_forward_v"]])),
            float(np.min(Z)), float(np.max(Z)), float(np.min(XPOS)), float(np.max(XPOS))
        )
    )
    gates = {
        "pitch": np.abs(SENS[:, IDX["pitch"]]) < math.radians(PITCH_VALID_DEG),
        "roll": np.abs(SENS[:, IDX["roll"]]) < math.radians(ROLL_VALID_DEG),
        "pitch_rate": np.abs(SENS[:, IDX["pitch_rate"]]) < 3.0,
        "roll_rate": np.abs(SENS[:, IDX["roll_rate"]]) < 6.0,
        "wheel_v": np.abs(SENS[:, IDX["wheel_v"]]) < WHEEL_VALID_V_MAX,
        "wheel_v_comp": np.abs(SENS[:, IDX["wheel_v_comp"]]) < WHEEL_VALID_V_MAX,
        "imu_acc_x": np.abs(SENS[:, IDX["imu_acc_x"]]) < IMU_VALID_ACC_MAX,
        "leg_swing": np.abs(SENS[:, IDX["leg_swing"]] - trim[IDX["leg_swing"]]) < 0.040,
        "leg_len": np.abs(SENS[:, IDX["leg_len"]] - LEG_TRIM_PER_SAMPLE) < 0.030,
        "leg_len_diff": np.abs(SENS[:, IDX["leg_len_diff"]] - trim[IDX["leg_len_diff"]]) < 0.160,
        "height": (Z > 0.04) & (Z < 0.35),
    }
    print("Small-signal gate pass counts:")
    for name, mask in gates.items():
        print(f"  {name:13s}: {int(mask.sum())}/{len(mask)}")
    print("Trim state around which the local model is identified:")
    for name in ["pitch", "pitch_rate", "roll", "roll_rate", "leg_swing", "leg_len", "leg_len_diff", "wheel_v"]:
        val = trim[IDX[name]]
        if name in ("pitch", "roll"):
            print(f"  {name:13s}: {math.degrees(val):+.4f} deg")
        else:
            print(f"  {name:13s}: {val:+.6f}")
    print(
        "Controlled-output accel ranges: pitch_acc=[{:+.1f},{:+.1f}] rad/s^2 "
        "roll_acc=[{:+.1f},{:+.1f}] rad/s^2".format(
            float(np.min(pitch_acc)), float(np.max(pitch_acc)),
            float(np.min(roll_acc)), float(np.max(roll_acc)),
        )
    )

    def report_velocity_observability():
        mask = valid_small & (np.abs(SENS[:, IDX["true_forward_v"]]) < 5.0)
        print("\nVelocity observability check (world velocity is validation label only):")
        if int(mask.sum()) < 40:
            print("  not enough valid samples")
            return
        y = SENS[mask, IDX["true_forward_v"]]
        names = [
            ("wheel_v", SENS[mask, IDX["wheel_v"]]),
            ("wheel_v_comp", SENS[mask, IDX["wheel_v_comp"]]),
            ("leg_swing", SENS[mask, IDX["leg_swing"]]),
            ("leg_swing_rate", SENS[mask, IDX["leg_swing_rate"]]),
            ("leg_kin_v", SENS[mask, IDX["leg_kin_v"]]),
            ("pitch_kin_v", SENS[mask, IDX["pitch_kin_v"]]),
            ("imu_acc_x", SENS[mask, IDX["imu_acc_x"]]),
        ]
        for name, x in names:
            xm, ym = float(np.mean(x)), float(np.mean(y))
            xv = float(np.sum((x - xm) ** 2))
            yv = float(np.sum((y - ym) ** 2))
            corr = float(np.sum((x - xm) * (y - ym)) / math.sqrt(max(1e-12, xv * yv)))
            gain = float(np.sum((x - xm) * (y - ym)) / max(1e-12, xv))
            bias = ym - gain * xm
            rmse = float(np.sqrt(np.mean((gain * x + bias - y) ** 2)))
            print(
                f"  {name:12s}: corr={corr:+.3f} gain={gain:+.4f} bias={bias:+.3f} "
                f"rmse={rmse:.3f} range=[{float(np.min(x)):+.2f},{float(np.max(x)):+.2f}]"
            )

        Phi = np.column_stack([
            SENS[mask, IDX["wheel_v"]],
            SENS[mask, IDX["wheel_v_comp"]],
            SENS[mask, IDX["leg_swing"]],
            SENS[mask, IDX["leg_swing_rate"]],
            SENS[mask, IDX["leg_kin_v"]],
            SENS[mask, IDX["pitch_kin_v"]],
            SENS[mask, IDX["imu_acc_x"]],
            np.ones(int(mask.sum())),
        ])
        theta, _, _, _ = np.linalg.lstsq(Phi, y, rcond=None)
        pred = Phi @ theta
        ss_res = float(np.sum((y - pred) ** 2))
        ss_tot = float(np.sum((y - np.mean(y)) ** 2))
        r2 = 1.0 - ss_res / max(ss_tot, 1e-12)
        print("  multi-feature fit true_v ~= [wheel, comp, swing, swing_rate, leg, pitch, imu_acc, 1]")
        print("  coef=", np.array2string(theta, precision=4, suppress_small=True), f"R2={r2:.4f}")

    report_velocity_observability()

    dt_eff = dt * DECIM

    def fit_state_model(name, X_all, valid_mask, q_diag=None, r_diag=None):
        # Keep consecutive valid transitions only.
        idx = np.where(valid_mask[:-1] & valid_mask[1:])[0]
        if len(idx) < 80:
            print(f"\n{name}: not enough valid transitions ({len(idx)}), skip")
            return None

        Xk = X_all[idx]
        Xn = X_all[idx + 1]
        Uk = U[idx]
        Phi = np.hstack([Xk, Uk, np.ones((len(idx), 1))])
        theta, _, _, _ = np.linalg.lstsq(Phi, Xn, rcond=None)
        n = X_all.shape[1]
        m = U.shape[1]
        A = theta[:n, :].T
        B = theta[n:n + m, :].T
        c = theta[n + m, :]

        pred = Phi @ theta
        ss_res = np.sum((Xn - pred) ** 2, axis=0)
        ss_tot = np.sum((Xn - np.mean(Xn, axis=0)) ** 2, axis=0)
        r2 = 1.0 - ss_res / np.maximum(ss_tot, 1e-12)
        poles_d = np.linalg.eigvals(A)
        poles_c = np.log(poles_d.astype(complex)) / dt_eff

        print(f"\n{name} local model @ dt={dt_eff:.3f}s, transitions={len(idx)}")
        print("A=\n", np.array2string(A, precision=6, suppress_small=True))
        print("B=\n", np.array2string(B, precision=6, suppress_small=True))
        print("c=", np.array2string(c, precision=6, suppress_small=True))
        print("R2=", np.array2string(r2, precision=4))
        print("poles discrete=", np.array2string(poles_d, precision=5))
        print("poles continuous=", np.array2string(poles_c, precision=2, suppress_small=True))

        K = None
        if q_diag is not None and r_diag is not None:
            Q = np.diag(q_diag)
            R = np.diag(r_diag)
            K = solve_dare_iter(A, B, Q, R)
            print("Local LQR around stabilizing PD / virtual-leg trim, u_extra = -K*x")
            print("K=", np.array2string(K, precision=4, suppress_small=True))
        return A, B, c, K, r2

    pitch_cols = [
        IDX["pitch"], IDX["pitch_rate"],
        IDX["leg_swing"], IDX["leg_swing_rate"],
        IDX["leg_len"], IDX["leg_len_rate"],
        IDX["wheel_v"],
    ]
    pitch_model = fit_state_model(
        "Pitch trim x=[pitch,pitch_rate,leg_swing,leg_swing_rate,leg_len,leg_len_rate,wheel_v]",
        DEV[:, pitch_cols],
        valid_small,
        q_diag=[900.0, 14.0, 120.0, 2.0, 60.0, 1.5, 2.0],
        r_diag=[0.30, 50.0, 80.0, 80.0],
    )

    roll_cols = [
        IDX["roll"], IDX["roll_rate"],
        IDX["leg_len_diff"], IDX["leg_len_diff_rate"],
        IDX["wheel_diff_v"],
    ]
    roll_model = fit_state_model(
        "Roll/length-diff trim x=[roll,roll_rate,leg_len_diff,leg_len_diff_rate,wheel_diff_v]",
        DEV[:, roll_cols],
        valid_small,
        q_diag=[500.0, 10.0, 180.0, 3.0, 1.0],
        r_diag=[2.0, 100.0, 60.0, 20.0],
    )

    compact_cols = [
        IDX["pitch"], IDX["pitch_rate"],
        IDX["roll"], IDX["roll_rate"],
        IDX["leg_swing"], IDX["leg_swing_rate"],
        IDX["leg_len"], IDX["leg_len_rate"],
        IDX["leg_len_diff"], IDX["leg_len_diff_rate"],
        IDX["wheel_v"], IDX["wheel_diff_v"],
    ]
    compact_model = fit_state_model(
        "Compact LQR/MPC trim model x=[pitch,pitch_rate,roll,roll_rate,swing,swing_rate,len,len_rate,len_diff,len_diff_rate,wheel_v,wheel_diff_v]",
        DEV[:, compact_cols],
        valid_small,
        q_diag=[900.0, 12.0, 500.0, 10.0, 100.0, 2.0, 60.0, 1.0, 140.0, 2.0, 2.0, 1.0],
        r_diag=[0.30, 70.0, 80.0, 40.0],
    )

    input_names = ["wheel_common_tau", "leg_swing_ref", "leg_len_ref", "leg_len_diff_ref"]
    pitch_names = ["pitch", "pitch_rate", "leg_swing", "leg_swing_rate", "leg_len", "leg_len_rate", "wheel_v"]
    roll_names = ["roll", "roll_rate", "leg_len_diff", "leg_len_diff_rate", "wheel_diff_v"]
    compact_names = [
        "pitch", "pitch_rate", "roll", "roll_rate",
        "leg_swing", "leg_swing_rate", "leg_len", "leg_len_rate",
        "leg_len_diff", "leg_len_diff_rate", "wheel_v", "wheel_diff_v",
    ]

    compact_piecewise = {}
    abs_pitch = np.abs(SENS[:, IDX["pitch"]])
    piecewise_specs = [
        ("compact_abs_0_2p5", "abs", -math.radians(2.5), math.radians(2.5), None),
        ("compact_pos_2p5_5", "pos", math.radians(2.5), math.radians(5.0), math.radians(2.5)),
        ("compact_neg_2p5_5", "neg", -math.radians(5.0), -math.radians(2.5), -math.radians(2.5)),
    ]
    print("\nPiecewise compact trim models by pitch linearization range:")
    for key, mode, lo, hi, expansion_pitch in piecewise_specs:
        if mode == "abs":
            band = abs_pitch <= hi
            label = "|pitch| in [0,2.5] deg, expansion around local center"
            trim_band = band
            pitch_range = [-math.degrees(hi), math.degrees(hi)]
            expansion_pitch_deg = 0.0
        else:
            band = (SENS[:, IDX["pitch"]] >= lo) & (SENS[:, IDX["pitch"]] <= hi)
            if mode == "pos":
                label = "pitch in [2.5,5] deg, expansion around +2.5 deg boundary"
                trim_band = np.abs(SENS[:, IDX["pitch"]] - expansion_pitch) <= math.radians(0.45)
            else:
                label = "pitch in [-5,-2.5] deg, expansion around -2.5 deg boundary"
                trim_band = np.abs(SENS[:, IDX["pitch"]] - expansion_pitch) <= math.radians(0.45)
            pitch_range = [math.degrees(lo), math.degrees(hi)]
            expansion_pitch_deg = math.degrees(expansion_pitch)
        mask = valid_small & band
        trim_mask = valid_small & trim_band
        count = int(mask.sum())
        trans = int(np.sum(mask[:-1] & mask[1:]))
        trim_count = int(trim_mask.sum())
        print(f"  {key}: samples={count} transitions={trans} trim_samples={trim_count} ({label})")
        if count < 80 or trans < 80:
            print(f"  {key}: not enough data, skip")
            compact_piecewise[key] = {
                "model": None,
                "cols": compact_cols,
                "names": compact_names,
                "trim": None,
                "valid_mask": mask,
                "pitch_range_deg": pitch_range,
                "expansion_pitch_deg": expansion_pitch_deg,
            }
            continue
        if trim_count < 20:
            print(f"  {key}: boundary trim samples too few; using band mean as local expansion point")
            trim_mask = mask
        local_trim = np.mean(SENS[trim_mask][:, compact_cols], axis=0)
        local_dev = SENS[:, compact_cols] - local_trim
        model = fit_state_model(
            f"{key} compact model {label}",
            local_dev,
            mask,
            q_diag=[900.0, 12.0, 500.0, 10.0, 100.0, 2.0, 60.0, 1.0, 140.0, 2.0, 2.0, 1.0],
            r_diag=[0.30, 70.0, 80.0, 40.0],
        )
        compact_piecewise[key] = {
            "model": model,
            "cols": compact_cols,
            "names": compact_names,
            "trim": local_trim,
            "valid_mask": mask,
            "pitch_range_deg": pitch_range,
            "expansion_pitch_deg": expansion_pitch_deg,
        }

    # ---- 腿长分段模型（为后续腿长变换准备） ----
    # 每个腿长基线采集时，腿长伺服全程保持该基线目标，所以样本按实测 leg_len
    # 自然聚在各基线附近。这里按基线分桶拟合独立的 compact 模型，并给出运行时可用的
    # leg_len_range（相邻段中心取中点为边界，首尾段外推半间距）。
    leglen_piecewise = {}
    leg_len_centers = []
    for leg_seg_i, leg_len_bias in enumerate(LEG_LEN_BIASES):
        seg_mask = valid_small & (LEG_SEGS == leg_seg_i)
        if int(seg_mask.sum()) >= 20:
            leg_len_centers.append(float(np.mean(SENS[seg_mask, IDX["leg_len"]])))
        else:
            leg_len_centers.append(float(np.mean(SENS[LEG_SEGS == leg_seg_i, IDX["leg_len"]])))
    print("\nPiecewise compact trim models by leg-length range:")
    for leg_seg_i, leg_len_bias in enumerate(LEG_LEN_BIASES):
        key = f"compact_leglen_{leg_seg_i}"
        mask = valid_small & (LEG_SEGS == leg_seg_i)
        count = int(mask.sum())
        trans = int(np.sum(mask[:-1] & mask[1:]))
        center = leg_len_centers[leg_seg_i]
        n_segs = len(leg_len_centers)
        if n_segs == 1:
            lo, hi = center - 0.020, center + 0.020
        elif leg_seg_i == 0:
            lo = center - 0.5 * (leg_len_centers[1] - center)
            hi = 0.5 * (center + leg_len_centers[1])
        elif leg_seg_i == n_segs - 1:
            lo = 0.5 * (center + leg_len_centers[-2])
            hi = center + 0.5 * (center - leg_len_centers[-2])
        else:
            lo = 0.5 * (center + leg_len_centers[leg_seg_i - 1])
            hi = 0.5 * (center + leg_len_centers[leg_seg_i + 1])
        print(
            f"  {key}: bias={leg_len_bias:+.3f} center={center:+.4f} "
            f"range=[{lo:+.4f},{hi:+.4f}] samples={count} transitions={trans}"
        )
        if count < 80 or trans < 80:
            print(f"  {key}: not enough data, skip")
            leglen_piecewise[key] = {
                "model": None,
                "cols": compact_cols,
                "names": compact_names,
                "trim": None,
                "valid_mask": mask,
                "leg_len_range": [lo, hi],
                "expansion_leg_len": center,
                "leg_len_bias": leg_len_bias,
            }
            continue
        local_trim = np.mean(SENS[mask][:, compact_cols], axis=0)
        local_dev = SENS[:, compact_cols] - local_trim
        model = fit_state_model(
            f"{key} compact model leg_len in [{lo:+.4f},{hi:+.4f}] rad",
            local_dev,
            mask,
            q_diag=[900.0, 12.0, 500.0, 10.0, 100.0, 2.0, 60.0, 1.0, 140.0, 2.0, 2.0, 1.0],
            r_diag=[0.30, 70.0, 80.0, 40.0],
        )
        leglen_piecewise[key] = {
            "model": model,
            "cols": compact_cols,
            "names": compact_names,
            "trim": local_trim,
            "valid_mask": mask,
            "leg_len_range": [lo, hi],
            "expansion_leg_len": center,
            "leg_len_bias": leg_len_bias,
        }

    if pitch_model is not None:
        print("\nOutput residuals for controlled accelerations (diagnostic, not core state):")
        idx = np.where(valid_small[:-1] & valid_small[1:])[0]
        if len(idx) >= 80:
            Xp = DEV[:, pitch_cols]
            A, B, c = pitch_model[0], pitch_model[1], pitch_model[2]
            pred_next = Xp[idx] @ A.T + U[idx] @ B.T + c
            pred_pitch_acc = (pred_next[:, 1] - Xp[idx, 1]) / dt_eff
            true_pitch_acc = pitch_acc[idx]
            pr = 1.0 - np.sum((true_pitch_acc - pred_pitch_acc) ** 2) / max(
                np.sum((true_pitch_acc - np.mean(true_pitch_acc)) ** 2), 1e-12
            )
            print(f"  pitch_acc R2={pr:.4f} rms_err={np.sqrt(np.mean((true_pitch_acc-pred_pitch_acc)**2)):.3f} rad/s^2")
    if roll_model is not None:
        idx = np.where(valid_small[:-1] & valid_small[1:])[0]
        if len(idx) >= 80:
            Xr = DEV[:, roll_cols]
            A, B, c = roll_model[0], roll_model[1], roll_model[2]
            pred_next = Xr[idx] @ A.T + U[idx] @ B.T + c
            pred_roll_acc = (pred_next[:, 1] - Xr[idx, 1]) / dt_eff
            true_roll_acc = roll_acc[idx]
            rr = 1.0 - np.sum((true_roll_acc - pred_roll_acc) ** 2) / max(
                np.sum((true_roll_acc - np.mean(true_roll_acc)) ** 2), 1e-12
            )
            print(f"  roll_acc  R2={rr:.4f} rms_err={np.sqrt(np.mean((true_roll_acc-pred_roll_acc)**2)):.3f} rad/s^2")

    def model_payload(name, model, cols, state_names, x_trim_vec, valid_mask):
        if model is None:
            return None
        A, B, c, K, r2 = model
        poles = np.linalg.eigvals(A)
        valid_transitions = int(np.sum(valid_mask[:-1] & valid_mask[1:]))
        return {
            "name": name,
            "dt": float(dt_eff),
            "state_names": state_names,
            "input_names": input_names,
            "state_definition": "x_dev = x_measured - x_trim; u_extra is perturbation around simple-PD/nominal-leg trim",
            "A": A.tolist(),
            "B": B.tolist(),
            "c": c.tolist(),
            "K_lqr": None if K is None else K.tolist(),
            "r2": r2.tolist(),
            "poles_discrete_real": np.real(poles).tolist(),
            "poles_discrete_imag": np.imag(poles).tolist(),
            "x_trim": x_trim_vec.tolist(),
            "u_extra_trim": [0.0, 0.0, 0.0, 0.0],
            "valid_transitions": valid_transitions,
        }

    piecewise_payloads = {}
    for key, item in compact_piecewise.items():
        payload = model_payload(
            key,
            item["model"],
            item["cols"],
            item["names"],
            np.zeros(len(item["cols"])) if item["trim"] is None else item["trim"],
            item["valid_mask"],
        )
        if payload is not None:
            payload["pitch_range_deg"] = item["pitch_range_deg"]
            payload["expansion_pitch_deg"] = item["expansion_pitch_deg"]
        piecewise_payloads[key] = payload

    leglen_payloads = {}
    for key, item in leglen_piecewise.items():
        payload = model_payload(
            key,
            item["model"],
            item["cols"],
            item["names"],
            np.zeros(len(item["cols"])) if item["trim"] is None else item["trim"],
            item["valid_mask"],
        )
        if payload is not None:
            payload["leg_len_range"] = item["leg_len_range"]
            payload["expansion_leg_len"] = item["expansion_leg_len"]
            payload["leg_len_bias"] = item["leg_len_bias"]
        leglen_payloads[key] = payload

    artifacts = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "xml": XML,
        "dt_physics": float(dt),
        "dt_model": float(dt_eff),
        "decimation": int(DECIM),
        "valid_samples": int(valid_small.sum()),
        "samples": int(len(SENS)),
        "valid_fraction": float(valid_small.sum() / max(1, len(SENS))),
        "pd_baseline": {
            "kp": PD_KP,
            "kd": PD_KD,
            "kv": PD_KV,
            "tau_limit": PD_TAU_LIMIT,
            "wheel_base_tau_trim": float(PD_KP * trim[IDX["pitch"]] + PD_KD * trim[IDX["pitch_rate"]] - PD_KV * trim[IDX["wheel_v"]]),
        },
        "prbs": {
            "wheel_tau_amp": PRBS_WHEEL_AMP,
            "leg_swing_amp": PRBS_SWING_AMP,
            "leg_len_amp": PRBS_LEN_AMP,
            "leg_len_diff_amp": PRBS_LEN_DIFF_AMP,
            "hold_steps": int(PRBS_HOLD_STEPS),
        },
        "valid_gates": {
            "pitch_deg": PITCH_VALID_DEG,
            "roll_deg": ROLL_VALID_DEG,
            "wheel_v_abs_max": WHEEL_VALID_V_MAX,
            "imu_acc_x_abs_max": IMU_VALID_ACC_MAX,
        },
        "full_trim_state_names": list(IDX.keys()),
        "full_trim_state": trim.tolist(),
        "leg_len_biases": LEG_LEN_BIASES,
        "leg_len_segments": [
            {
                "key": f"compact_leglen_{i}",
                "bias": LEG_LEN_BIASES[i],
                "center": leg_len_centers[i],
                "range": leglen_piecewise[f"compact_leglen_{i}"]["leg_len_range"],
                "samples": int((valid_small & (LEG_SEGS == i)).sum()),
                "transitions": int(np.sum((valid_small & (LEG_SEGS == i))[:-1] & (valid_small & (LEG_SEGS == i))[1:])),
            }
            for i in range(len(LEG_LEN_BIASES))
        ],
        "models": {
            "pitch": model_payload("pitch", pitch_model, pitch_cols, pitch_names, trim[pitch_cols], valid_small),
            "roll": model_payload("roll", roll_model, roll_cols, roll_names, trim[roll_cols], valid_small),
            "compact": model_payload("compact", compact_model, compact_cols, compact_names, trim[compact_cols], valid_small),
            **piecewise_payloads,
            **leglen_payloads,
        },
        "notes": [
            "Runtime model states are derived only from IMU, wheel encoders, and leg joint encoders/velocities.",
            "World pose/velocity is used only for data-quality validation, not as a model state or control input.",
            "Use u = u_baseline + u_extra. LQR gain K_lqr maps u_extra = -K_lqr * x_dev.",
        ],
    }

    required_piecewise = ["compact_abs_0_2p5", "compact_pos_2p5_5", "compact_neg_2p5_5"]
    required_piecewise += [f"compact_leglen_{i}" for i in range(len(LEG_LEN_BIASES))]
    publish_latest = (
        pitch_model is not None
        and roll_model is not None
        and compact_model is not None
        and all(piecewise_payloads.get(key) is not None for key in ["compact_abs_0_2p5", "compact_pos_2p5_5", "compact_neg_2p5_5"])
        and all(leglen_payloads.get(key) is not None for key in [f"compact_leglen_{i}" for i in range(len(LEG_LEN_BIASES))])
    )

    logs_dir = os.path.join(PROJECT_ROOT, "logs")
    os.makedirs(logs_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    json_path = os.path.join(logs_dir, f"trim_lqr_model_{stamp}.json")
    latest_json_path = os.path.join(logs_dir, "trim_lqr_model_latest.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(artifacts, f, indent=2)
        f.write("\n")
    if publish_latest:
        with open(latest_json_path, "w", encoding="utf-8") as f:
            json.dump(artifacts, f, indent=2)
            f.write("\n")

    npz_path = os.path.join(logs_dir, f"trim_lqr_model_{stamp}.npz")
    latest_npz_path = os.path.join(logs_dir, "trim_lqr_model_latest.npz")
    save_items = {
        "full_trim_state": trim,
        "u_extra_trim": np.zeros(4),
        "input_names": np.array(input_names),
        "full_trim_state_names": np.array(list(IDX.keys())),
    }
    for key, model, cols, names in [
        ("pitch", pitch_model, pitch_cols, pitch_names),
        ("roll", roll_model, roll_cols, roll_names),
        ("compact", compact_model, compact_cols, compact_names),
    ]:
        if model is None:
            continue
        A, B, c, K, r2 = model
        save_items[f"{key}_A"] = A
        save_items[f"{key}_B"] = B
        save_items[f"{key}_c"] = c
        save_items[f"{key}_K"] = np.array([]) if K is None else K
        save_items[f"{key}_r2"] = r2
        save_items[f"{key}_x_trim"] = trim[cols]
        save_items[f"{key}_state_names"] = np.array(names)
    for key, item in compact_piecewise.items():
        model = item["model"]
        if model is None:
            continue
        A, B, c, K, r2 = model
        save_items[f"{key}_A"] = A
        save_items[f"{key}_B"] = B
        save_items[f"{key}_c"] = c
        save_items[f"{key}_K"] = np.array([]) if K is None else K
        save_items[f"{key}_r2"] = r2
        save_items[f"{key}_x_trim"] = item["trim"]
        save_items[f"{key}_state_names"] = np.array(item["names"])
        save_items[f"{key}_pitch_range_deg"] = np.array(item["pitch_range_deg"])
        save_items[f"{key}_expansion_pitch_deg"] = np.array([item["expansion_pitch_deg"]])
    for key, item in leglen_piecewise.items():
        model = item["model"]
        if model is None:
            continue
        A, B, c, K, r2 = model
        save_items[f"{key}_A"] = A
        save_items[f"{key}_B"] = B
        save_items[f"{key}_c"] = c
        save_items[f"{key}_K"] = np.array([]) if K is None else K
        save_items[f"{key}_r2"] = r2
        save_items[f"{key}_x_trim"] = item["trim"]
        save_items[f"{key}_state_names"] = np.array(item["names"])
        save_items[f"{key}_leg_len_range"] = np.array(item["leg_len_range"])
        save_items[f"{key}_expansion_leg_len"] = np.array([item["expansion_leg_len"]])
        save_items[f"{key}_leg_len_bias"] = np.array([item["leg_len_bias"]])
    np.savez(npz_path, **save_items)
    if publish_latest:
        np.savez(latest_npz_path, **save_items)
    print("\nExported trim model artifacts:")
    print(f"  JSON: {json_path}")
    print(f"  NPZ : {npz_path}")
    if publish_latest:
        print(f"  latest JSON/NPZ also updated in {logs_dir}")
    else:
        missing = [key for key in required_piecewise if piecewise_payloads.get(key) is None and leglen_payloads.get(key) is None]
        print(f"  latest JSON/NPZ kept unchanged; incomplete model set, missing={missing}")

    print("\nConclusion: this script now identifies the +/-5deg trim model for LQR/MPC.")
    print("Large-angle recovery should stay in the energy-shaping / simple-PD layer before switching to this model.")


if __name__ == "__main__":
    main()

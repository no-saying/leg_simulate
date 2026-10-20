#!/usr/bin/env python3
"""
v3 模型 LQR 分段模型辨识
===============================================================
identify_three_loop.py 的 v3 版。差别集中在"机型相关"的部分:

  * 模型: model/v3/COD-2026RoboMaster-Balance-v3.xml, 立姿 = 绕 x +90°,
    轮轴沿世界 -y; 传感器要走 sensors.c 的安装矩阵补偿(q_ctrl = q_body⊗q_bake⁻¹,
    gyro_ctrl = (x,-z,y))才能得到控制系 pitch/roll。
  * 腿: 4 个驱动关节 -> (腿摆 sw, 腿长 L) 用 leg_kin_v3 闭链标定映射; 腿伺服是
    虚拟坐标伺服(腿长力/腿摆力经 J^T 到关节), 与 robot_control.c 部署一致。
  * 轮: 左右轮轴同向, ctrl[2]=(pitch−yaw)/10, ctrl[5]=(pitch+yaw)/10;
    状态里 wheel_v = 0.5(lw+rw)R, wheel_diff_v = 0.5(lw−rw)R(对齐 lqr_controller.c)。
  * 腿长输入: 走 balance_set_leg_extend_manual(ext), 部署里 L_ref = L_MID - ext。

输出:
  logs/three_loop_model_v3.json / .npz          12 状态 x 5 输入分段模型
  control/controllers/lqr/lqr_pitch_models.h    pitch 子系统(7状态3输入)
  control/controllers/lqr/lqr_roll_models.h     roll  子系统(5状态2输入)

用法: python3 scripts/tools/identify_lqr_v3.py
"""

import ctypes
import json
import math
import os
import shutil
import subprocess
import sys
import time

import mujoco
import numpy as np
from scipy.signal import butter, filtfilt

os.environ.setdefault("MUJOCO_GL", "osmesa")

# 本脚本在 scripts/tools/ 下, 上溯三层才是工程根目录
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
XML = os.path.join(PROJECT_ROOT, "model", "v3", "COD-2026RoboMaster-Balance-v3.xml")
BRIDGE = os.environ.get("V3_BRIDGE", os.path.join(PROJECT_ROOT, "logs", "ident_bridge_pid.so"))

# ---- v3 机型常量(必须与 control/ 下同名宏一致) ----
SW_NOM = -0.01596
L_NOM = 0.08656
L_MID = 0.117
K_LA, K_LB, K_LC, K_KD = 7.582279, 9.545355, -6.728685, 9.309838
LEG_CAL = np.array([
    [0.086715294, -0.016222440], [0.068312374, 0.051321734],
    [-0.071332646, 0.057643961], [-0.002672041, 0.036104740],
    [-0.042785317, -0.002465143], [-0.007430294, -0.032528860],
    [-0.014152201, -0.007137062], [0.013224687, -0.004273003],
    [-0.008973839, -0.019570855], [0.018311806, -0.005660699],
])
WHEEL_R = 0.043
WHEEL_MIX_PITCH = float(os.environ.get("V3_WHEEL_MIX_PITCH", "1.0"))
SERVO_KP = float(os.environ.get("V3_SERVO_KP", "1200.0"))
SERVO_KD = float(os.environ.get("V3_SERVO_KD", "60.0"))
SW_KP = float(os.environ.get("V3_SW_KP", "300.0"))
SW_KD = float(os.environ.get("V3_SW_KD", "10.0"))
SW_DEAD = float(os.environ.get("V3_SW_DEAD", "0.005"))
SHORT_SW_KP = float(os.environ.get("V3_SHORT_SW_KP", "450.0"))
SHORT_SW_KD = float(os.environ.get("V3_SHORT_SW_KD", "18.0"))
SHORT_SW_DEAD = float(os.environ.get("V3_SHORT_SW_DEAD", "0.001"))
LEG_GAIN = 50.0
BAL_SW_NOM = -0.01596             # balance 输出 *sw 的零点

IDX = {
    "pitch": 0, "pitch_rate": 1, "roll": 2, "roll_rate": 3,
    "leg_swing": 4, "leg_swing_rate": 5,
    "leg_len": 6, "leg_len_rate": 7,
    "leg_len_diff": 8, "leg_len_diff_rate": 9,
    "wheel_v": 10, "wheel_diff_v": 11,
}

# L = L_MID - ext.  Sweep the full observed operating envelope (about
# 0.05..0.21 m), including margin at both ends.  The ext interval is wider
# than the old +/-30 mm middle-only identification.
EXT_MIN = float(os.environ.get("V3_EXT_MIN", "-0.040"))
EXT_MAX = float(os.environ.get("V3_EXT_MAX", "0.040"))
SWEEP_TIME = float(os.environ.get("V3_SWEEP_TIME", "10.0"))
SWEEP_CYCLES = int(os.environ.get("V3_SWEEP_CYCLES", "3"))   # 每点多跑几轮 -> 样本量
N_SETPOINTS = int(os.environ.get("V3_SETPOINTS", "18"))  # 阶梯腿长点数(用户: 分区分小一点)
STAIR_RATE = float(os.environ.get("V3_STAIR_RATE", "0.02"))  # 阶梯过渡斜率 m/s
FAIL_ROLL = float(os.environ.get("V3_FAIL_ROLL", "0.35"))   # 超此侧倾判为摔车, 本点作废
DWELL_TIME = float(os.environ.get("V3_DWELL", "3.0"))    # 每点驻留时长 s
SETTLE_TIME = float(os.environ.get("V3_SETTLE_TIME", "3.0"))
POINT_SETTLE_TIME = float(os.environ.get("V3_POINT_SETTLE", "4.0"))
STATIC_V_MEAN_MAX = float(os.environ.get("V3_STATIC_V_MEAN_MAX", "0.05"))
STATIC_V_RMS_MAX = float(os.environ.get("V3_STATIC_V_RMS_MAX", "0.10"))
PRBS_WHEEL = float(os.environ.get("V3_PRBS_WHEEL", "0.35"))
PRBS_WHEELDIFF = float(os.environ.get("V3_PRBS_WHEELDIFF", "0.15"))
PRBS_SWING = float(os.environ.get("V3_PRBS_SWING", "0.0005"))
PRBS_DIFF = float(os.environ.get("V3_PRBS_DIFF", "0.001"))
PRBS_EXT = float(os.environ.get("V3_PRBS_EXT", "0.0015"))
PRBS_SPLAY = float(os.environ.get("V3_PRBS_SPLAY", "0.001"))  # 叉腿差动 PRBS (m)
PITCH_PULSE = float(os.environ.get("V3_PITCH_PULSE", "0.90"))  # 每窗口俯仰力矩脉冲 (Nm)
PULSE_T = float(os.environ.get("V3_PULSE_T", "0.30"))          # 脉冲时长 s
LEG_DITHER = float(os.environ.get("V3_LEG_DITHER", "0.001"))   # 窗口内腿长抖动 ±m
PRBS_RF = float(os.environ.get("V3_PRBS_RF", "0.0"))   # 可选侧向力 PRBS (N)
PRBS_RT = float(os.environ.get("V3_PRBS_RT", "0.0"))  # 可选侧倾力矩 PRBS (Nm)
PRBS_HOLD = 100   # 100ms = 4x采样周期, 否则速率状态被自回归吞掉
FILT_FC = float(os.environ.get("V3_FILT_FC", "0.0"))
RIDGE = float(os.environ.get("V3_RIDGE", "0.05"))
PITCH3_RIDGE = float(os.environ.get("V3_PITCH3_RIDGE", "0.5"))
DECIM = int(os.environ.get("V3_DECIM", "5"))
if DECIM < 1:
    raise ValueError("V3_DECIM 必须 >= 1")

# 腿长分段(实测腿长 m，v3 工作区约 0.08~0.16)
# Identification envelope follows the PID controlled travel.  The controller
# gate still uses measured leg length only to select a linearization segment.
IDENT_LEG_LEN_MIN = float(os.environ.get("V3_LEG_LEN_MIN", "0.065"))
IDENT_LEG_LEN_MAX = float(os.environ.get("V3_LEG_LEN_MAX", "0.160"))
LEG_LEN_BINS = np.arange(IDENT_LEG_LEN_MIN, IDENT_LEG_LEN_MAX + 0.005, 0.01)
STATIC_ONLY = os.environ.get("V3_STATIC_ONLY") == "1"
PID_PITCH_PULSE = float(os.environ.get("V3_PID_PITCH_PULSE", "0.0"))


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


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_to_pitch(q):
    return math.asin(max(-1.0, min(1.0, 2.0 * (q[0] * q[2] - q[1] * q[3]))))


def quat_to_roll(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def quat_to_yaw(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def leg_kin_side(front, rear, right=False):
    sign = -1.0 if right else 1.0
    a, b = sign * front, sign * rear
    terms = np.array([1, a, b, a*a, a*b, b*b, a*a*a, a*a*b, a*b*b, b*b*b])
    da = np.array([0, 1, 0, 2*a, b, 0, 3*a*a, 2*a*b, b*b, 0])
    db = np.array([0, 0, 1, 0, a, 2*b, 0, a*a, 2*a*b, 3*b*b])
    length, swing = terms @ LEG_CAL
    jac = sign * np.stack([da @ LEG_CAL, db @ LEG_CAL], axis=1)
    return length, swing, jac


def leg_kin_forward(lf, lr, rf, rr):
    L_l, sw_l, _ = leg_kin_side(lf, lr)
    L_r, sw_r, _ = leg_kin_side(rf, rr, True)
    return 0.5 * (sw_l + sw_r), L_l, L_r


def leg_kin_vel(lf, lr, rf, rr, lfv, lrv, rfv, rrv):
    _, _, jl = leg_kin_side(lf, lr)
    _, _, jr = leg_kin_side(rf, rr, True)
    l_l_d, sw_l_d = jl @ [lfv, lrv]
    l_r_d, sw_r_d = jr @ [rfv, rrv]
    return 0.5 * (sw_l_d + sw_r_d), l_l_d, l_r_d


def leg_kin_inverse(sw_l, L_l, sw_r, L_r):
    """(左右腿摆, 左右腿长) -> 4 个驱动关节角(与 leg_kin_v3_inverse 同式,
    区别只是允许左右摆角不同: 叉腿/差动时 sw_l != sw_r)。"""
    ext_l, ext_r = L_l - L_NOM, L_r - L_NOM
    swd_l, swd_r = sw_l - SW_NOM, sw_r - SW_NOM
    lf = K_LA * ext_l + K_LB * swd_l
    lr = K_LC * ext_l + K_KD * swd_l
    rf = -K_LA * ext_r - K_LB * swd_r
    rr = -K_LC * ext_r - K_KD * swd_r
    return lf, lr, rf, rr


def velocity_is_static(samples):
    values = np.asarray(samples)
    tail = values[-max(1, len(values) // 2):]
    return (abs(values.mean()) <= STATIC_V_MEAN_MAX and
            np.sqrt(np.mean(values * values)) <= STATIC_V_RMS_MAX and
            abs(tail.mean()) <= STATIC_V_MEAN_MAX)


def leg_forces(lf, lr, rf, rr, F_l, F_sw_l, F_r, F_sw_r):
    _, _, jl = leg_kin_side(lf, lr)
    _, _, jr = leg_kin_side(rf, rr, True)
    return (* (jl.T @ [F_l, F_sw_l]), * (jr.T @ [F_r, F_sw_r]))



# 前五对是位置/速率。wheel_v 与 wheel_diff_v 都是速度，不能互当导数。
POS2RATE = {0: 1, 2: 3, 4: 5, 6: 7, 8: 9}


def fit_structured(SENS_, U_, idx, dt_eff, n=12, mm=None):
    if mm is None:
        mm = U_.shape[1]
    """围绕 PID 工作点拟合完整状态增量，约束位置积分关系。"""
    Xk = SENS_[idx]
    Uk = U_[idx]
    Xn = SENS_[idx + 1]
    z = np.hstack([Xk, Uk])
    center = z.mean(axis=0)
    scale = np.maximum(z.std(axis=0), 1e-5)
    phi = (z - center) / scale
    delta = Xn - Xk
    gram = phi.T @ phi + RIDGE * len(idx) * np.eye(n + mm)
    coef = np.linalg.solve(gram, phi.T @ (delta - delta.mean(axis=0)))
    d = (coef / scale[:, None]).T
    A = np.eye(n) + d[:, :n]
    B = d[:, n:]
    c = delta.mean(axis=0) - d @ center
    for p, r in POS2RATE.items():
        A[p, :] = np.eye(n)[p] + 0.5 * dt_eff * (np.eye(n)[r] + A[r, :])
        B[p, :] = 0.5 * dt_eff * B[r, :]
        c[p] = 0.5 * dt_eff * c[r]
    return A, B, c


def fit_pitch3_structured(SENS_, U_, idx, dt_eff):
    """Fit the measured pitch plant without leg-loop states.

    The leg references are closed-loop actuator commands. Including them in
    the pitch regression makes wheel-torque gain change sign when the leg
    servo absorbs the same motion. The three measured states are sufficient
    for the pitch LQR and keep that nuisance correlation out of K.
    """
    cols = [IDX["pitch"], IDX["pitch_rate"], IDX["wheel_v"]]
    Xk, Uk, Xn = SENS_[idx][:, cols], U_[idx][:, [0]], SENS_[idx + 1][:, cols]
    z = np.hstack([Xk, Uk])
    center = z.mean(axis=0)
    scale = np.maximum(z.std(axis=0), 1e-5)
    phi = (z - center) / scale
    delta = Xn - Xk
    gram = phi.T @ phi + PITCH3_RIDGE * len(idx) * np.eye(4)
    coef = np.linalg.solve(gram, phi.T @ (delta - delta.mean(axis=0)))
    d = (coef / scale[:, None]).T
    A = np.eye(3) + d[:, :3]
    B = d[:, 3:]
    c = delta.mean(axis=0) - d @ center
    # wheel_v is the measured velocity state; pitch is its integral partner.
    A[0, :] = np.eye(3)[0] + 0.5 * dt_eff * (np.eye(3)[1] + A[1, :])
    B[0, :] = 0.5 * dt_eff * B[1, :]
    c[0] = 0.5 * dt_eff * c[1]
    return A, B, c

def solve_dare(A, B, Q, R, max_iter=5000, tol=1e-10):
    P = Q.copy()
    for _ in range(max_iter):
        Pn = Q + A.T @ P @ A - A.T @ P @ B @ np.linalg.solve(
            R + B.T @ P @ B, B.T @ P @ A)
        if np.linalg.norm(Pn - P, "fro") < tol:
            P = Pn
            break
        P = Pn
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


def build_pid_bridge():
    if os.environ.get("V3_BRIDGE"):
        return
    sources = [
        "scripts/ident_bridge.c",
        "control/balance/balance.c", "control/balance/balance_api.c",
        "control/balance/balance_util.c", "control/balance/balance_core.c",
        "control/balance/leg_kin_v3.c", "control/balance/vel_kf.c",
        "control/controllers/lqr/lqr_controller.c",
        "control/controllers/energy/energy_controller.c",
        "control/controllers/supervisor/controller_supervisor.c",
        "control/controllers/rl_jump/rl_jump_net.c",
        "control/controllers/rl_jump/rl_jump_controller.c",
        "control/runtime/robot_control.c", "control/lowlevel/pid.c",
        "control/lowlevel/wheel_t.c", "control/lowlevel/leg_t.c",
    ]
    os.makedirs(os.path.dirname(BRIDGE), exist_ok=True)
    cmd = ["gcc", "-O2", "-fPIC", "-shared", "-o", BRIDGE]
    cmd += [os.path.join(PROJECT_ROOT, source) for source in sources]
    cmd += ["-I" + os.path.join(PROJECT_ROOT, "control", path)
            for path in ("include", "", "balance", "lowlevel", "runtime", "controllers")]
    cmd += ["-lm"]
    subprocess.run(cmd, check=True)
    print("PID bridge rebuilt from current source:", BRIDGE, flush=True)


def main():
    build_pid_bridge()
    lib = ctypes.CDLL(BRIDGE)
    lib.ident_init()
    lib.ident_set_leg_len_direct.argtypes = [ctypes.c_float]
    lib.ident_step.argtypes = [ctypes.POINTER(IdentFrame), ctypes.c_double,
                               ctypes.POINTER(IdentOut)]
    if os.environ.get("V3_CONTACT_DIAG") == "1":
        for name in ("balance_get_wheel_body_v", "balance_get_airborne",
                     "balance_get_slip", "balance_get_neutral"):
            getattr(lib, name).restype = ctypes.c_float

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    dt = model.opt.timestep

    leg_names = ["Left_front_joint", "Left_rear_joint",
                 "Right_front_joint", "Right_rear_joint"]
    leg_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
               for n in leg_names]
    leg_qpos = [model.jnt_qposadr[j] for j in leg_ids]
    leg_dof = [model.jnt_dofadr[j] for j in leg_ids]
    leg_act = [0, 1, 3, 4]
    lw_j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Left_Wheel_joint")
    rw_j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Right_Wheel_joint")
    lw_dof, rw_dof = model.jnt_dofadr[lw_j], model.jnt_dofadr[rw_j]

    rng = np.random.default_rng(7)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    wheel_bids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
                  for n in ("Left_Wheel_link", "Right_Wheel_link")]
    wheel_geoms = [
        {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
         for n in (f"{side}_Wheel_mesh_1", f"{side}_Wheel_collision")
         if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) >= 0}
        for side in ("L", "R")]
    floor_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")

    def contact_diagnostic(m, out):
        normal = np.zeros(2)
        for i in range(data.ncon):
            c = data.contact[i]
            for side, geoms in enumerate(wheel_geoms):
                if floor_geom in (c.geom1, c.geom2) and \
                        (c.geom1 in geoms or c.geom2 in geoms):
                    force = np.zeros(6)
                    mujoco.mj_contactForce(model, data, i, force)
                    normal[side] += force[0]
        return (normal[0], normal[1], -m[IDX["wheel_v"]], m[14],
                lib.balance_get_wheel_body_v(), lib.balance_get_airborne(),
                lib.balance_get_slip(), m[IDX["pitch"]], out.wheel_tau,
                lib.balance_get_neutral())

    def setup():
        """v3 立姿(sensors_set_initial_pose 的等价实现)。"""
        mujoco.mj_resetData(model, data)
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qpos[2] = -0.025
        data.qpos[3:7] = [0.70710678, 0.70710678, 0.0, 0.0]
        for adr, val in zip(leg_qpos, [0.2, -0.2, -0.2, 0.2]):
            data.qpos[adr] = val
        mujoco.mj_forward(model, data)

    def sensors():
        q_body = data.sensordata[0:4]
        gyro_body = data.sensordata[4:7]
        acc_body = data.sensordata[7:10]
        q_mount = np.array([0.70710678, -0.70710678, 0.0, 0.0])
        q_ctrl = quat_mul(q_body, q_mount)
        gyro = np.array([gyro_body[0], -gyro_body[2], gyro_body[1]])
        acc = np.array([acc_body[0], -acc_body[2], acc_body[1]])
        return q_ctrl, gyro, acc

    def measure():
        q_ctrl, gyro, acc = sensors()
        lf, lr, rf, rr = (data.qpos[a] for a in leg_qpos)
        lfv, lrv, rfv, rrv = (data.qvel[a] for a in leg_dof)
        sw, L_l, L_r = leg_kin_forward(lf, lr, rf, rr)
        sw_d, l_d, r_d = leg_kin_vel(lf, lr, rf, rr, lfv, lrv, rfv, rrv)
        lw_v, rw_v = data.qvel[lw_dof], data.qvel[rw_dof]
        m = np.zeros(18)
        m[IDX["pitch"]] = quat_to_pitch(q_ctrl)
        m[IDX["pitch_rate"]] = gyro[1]  # Pure PID collection has no yaw command.
        m[IDX["roll"]] = quat_to_roll(q_ctrl)
        m[IDX["roll_rate"]] = gyro[0]
        m[IDX["leg_swing"]] = sw - SW_NOM
        m[IDX["leg_swing_rate"]] = sw_d
        m[IDX["leg_len"]] = 0.5 * (L_l + L_r)
        m[IDX["leg_len_rate"]] = 0.5 * (l_d + r_d)
        m[IDX["leg_len_diff"]] = L_l - L_r
        m[IDX["leg_len_diff_rate"]] = l_d - r_d
        # LQR 部署端 build_pitch_state 直接读取轮编码器速度。
        m[IDX["wheel_v"]] = 0.5 * (lw_v + rw_v) * WHEEL_R
        m[IDX["wheel_diff_v"]] = 0.5 * (lw_v - rw_v) * WHEEL_R
        m[12] = acc[0]
        m[13] = acc[2]
        m[14] = data.qvel[0]  # 离线诊断: 世界 x 速度，不进入辨识状态或控制
        m[15] = data.qpos[0]
        axle = 0.5 * (data.xpos[wheel_bids[0]] + data.xpos[wheel_bids[1]])
        axle_local = data.xmat[bid].reshape(3, 3).T @ (axle - data.xpos[bid])
        m[16] = axle_local[0]
        m[17] = -axle_local[1]
        return m, (lf, lr, rf, rr, lfv, lrv, rfv, rrv, lw_v, rw_v, lf, lr, rf, rr), \
               (sw, L_l, L_r)

    def fill_frame(m, raw):
        lf, lr, rf, rr, lfv, lrv, rfv, rrv, lw_v, rw_v = raw[:10]
        q_ctrl, gyro, acc = sensors()
        f = IdentFrame()
        f.pitch = m[IDX["pitch"]]
        # The bridge frame carries raw body gyro; each control path converts it.
        f.pitch_rate = gyro[1]
        f.roll = m[IDX["roll"]]
        f.roll_rate = m[IDX["roll_rate"]]
        f.left_wheel_vel = lw_v
        f.right_wheel_vel = rw_v
        f.acc_mag = math.hypot(acc[0], acc[2])
        f.acc_x = acc[0]
        f.acc_z = acc[2]
        f.yaw = quat_to_yaw(q_ctrl)
        f.yaw_rate = gyro[2]
        f.left_wheel_pos = data.qpos[model.jnt_qposadr[lw_j]]
        f.right_wheel_pos = data.qpos[model.jnt_qposadr[rw_j]]
        f.lf_pos, f.lr_pos, f.rf_pos, f.rr_pos = lf, lr, rf, rr
        f.lf_vel, f.lr_vel, f.rf_vel, f.rr_vel = lfv, lrv, rfv, rrv
        f.lf_tau = data.actuator_force[0]
        f.lr_tau = data.actuator_force[1]
        f.rf_tau = data.actuator_force[3]
        f.rr_tau = data.actuator_force[4]
        f.left_wheel_tau = data.actuator_force[2] * 10.0
        f.right_wheel_tau = data.actuator_force[5] * 10.0
        return f

    def leg_servo(sw_cmd, L_cmd, diff_sw, diff_L):
        """与 robot_control.c 的 v3 腿伺服等价(虚拟坐标力 -> J^T -> 关节)。"""
        sw_l = sw_cmd + diff_sw
        sw_r = sw_cmd - diff_sw
        L_l = L_cmd + diff_L
        L_r = L_cmd - diff_L
        lf, lr, rf, rr = (data.qpos[a] for a in leg_qpos)
        lfv, lrv, rfv, rrv = (data.qvel[a] for a in leg_dof)
        _, L_l_m, L_r_m = leg_kin_forward(lf, lr, rf, rr)
        _, l_d, r_d = leg_kin_vel(lf, lr, rf, rr, lfv, lrv, rfv, rrv)
        _, sw_l_m, jl = leg_kin_side(lf, lr)
        _, sw_r_m, jr = leg_kin_side(rf, rr, True)
        sw_l_d = (jl @ [lfv, lrv])[1]
        sw_r_d = (jr @ [rfv, rrv])[1]
        short_sw_blend = np.clip((0.054 - 0.5 * (L_l_m + L_r_m)) / 0.004, 0.0, 1.0)
        sw_dead = SW_DEAD + short_sw_blend * (SHORT_SW_DEAD - SW_DEAD)
        F_L_l = -(SERVO_KP * (L_l_m - L_l) + SERVO_KD * l_d)
        F_L_r = -(SERVO_KP * (L_r_m - L_r) + SERVO_KD * r_d)
        e_l = sw_l_m - sw_l
        e_r = sw_r_m - sw_r
        if abs(e_l) > sw_dead:
            e_l -= sw_dead if e_l > 0 else -sw_dead
        else:
            e_l = 0.0
        if abs(e_r) > sw_dead:
            e_r -= sw_dead if e_r > 0 else -sw_dead
        else:
            e_r = 0.0
        sw_d_avg = 0.5 * (sw_l_d + sw_r_d)
        long_leg_blend = np.clip((0.5 * (L_l_m + L_r_m) - 0.136) / 0.014, 0.0, 1.0)
        deep_long_blend = np.clip((0.5 * (L_l_m + L_r_m) - 0.160) / 0.015, 0.0, 1.0)
        sw_kp = SW_KP - 150.0 * long_leg_blend - 50.0 * deep_long_blend
        sw_kd = SW_KD - 5.0 * long_leg_blend - 2.0 * deep_long_blend
        sw_kp += short_sw_blend * (SHORT_SW_KP - SW_KP)
        sw_kd += short_sw_blend * (SHORT_SW_KD - SW_KD)
        F_sw_l = -(sw_kp * e_l + sw_kd * sw_d_avg)
        F_sw_r = -(sw_kp * e_r + sw_kd * sw_d_avg)
        tl = leg_forces(lf, lr, rf, rr, F_L_l, F_sw_l, F_L_r, F_sw_r)
        for i, (act, tau) in enumerate(zip(leg_act, tl)):
            data.ctrl[act] = np.clip(tau / LEG_GAIN, -10.0, 10.0)

    def wheel_ctrl(pitch_tau, yaw_tau_cmd):
        pitch_mix = WHEEL_MIX_PITCH * pitch_tau
        yaw_mix = 0.4 * yaw_tau_cmd
        data.ctrl[2] = np.clip((pitch_mix - yaw_mix) / 10.0, -10.0, 10.0)
        data.ctrl[5] = np.clip((pitch_mix + yaw_mix) / 10.0, -10.0, 10.0)

    n_sweep = int(SWEEP_TIME / dt)
    SENS, U, SEGID, RUNID, SAMPLE_STEP, CYCLE = [], [], [], [], [], []
    STATIC_X, STATIC_U = {}, {}
    run_id = 0

    print(f"v3 辨识: 腿长 ext∈[{EXT_MIN},{EXT_MAX}] 扫 {SWEEP_CYCLES} 次, "
          f"PRBS wheel={PRBS_WHEEL}Nm diff={PRBS_WHEELDIFF}Nm")
    prbs = dict(wheel=0.0, wdiff=0.0, swing=0.0, diff=0.0, ext=0.0, splay=0.0,
                rforce=0.0, rtorque=0.0)
    stair = 0.0
    hold_next = 0
    setpoints = np.linspace(EXT_MAX, EXT_MIN, N_SETPOINTS)
    n_dwell = (int(DWELL_TIME / dt) // DECIM) * DECIM
    if n_dwell < DECIM:
        raise ValueError("V3_DWELL 至少需要一个完整采样周期")
    print("阶梯腿长点 ext:", np.array2string(setpoints, precision=3))
    for cyc in range(SWEEP_CYCLES):
        # 从中位向两端推进；每个点先到目标并稳定，再开始计入辨识样本。
        order = sorted(range(N_SETPOINTS), key=lambda i: abs(setpoints[i]))
        for ip in order:
            ext_ref = float(setpoints[ip])
            setup()
            lib.ident_reset()
            stair = 0.0
            prbs = dict.fromkeys(prbs, 0.0)
            hold_next = 0
            lib.ident_set_leg_len_direct(ctypes.c_float(0.0))
            for _ in range(int(SETTLE_TIME / dt)):
                m, raw, _ = measure()
                out = IdentOut()
                lib.ident_step(ctypes.byref(fill_frame(m, raw)), dt, ctypes.byref(out))
                leg_servo(out.leg_swing, L_MID - out.leg_extend,
                          0.0, -out.leg_side_extend)
                wheel_ctrl(out.wheel_tau, out.yaw_tau)
                mujoco.mj_step(model, data)
            travel_steps = int(abs(ext_ref - stair) / (STAIR_RATE * dt)) + 1
            transition_failed = False
            point_steps = travel_steps + int(POINT_SETTLE_TIME / dt)
            steady_v, steady_body_v = [], []
            contact_tail = []
            static_x_win, static_u_win = [], []
            for transition_step in range(point_steps):
                stair += float(np.clip(ext_ref - stair, -STAIR_RATE * dt, STAIR_RATE * dt))
                m, raw, _ = measure()
                if transition_step >= point_steps - int(1.0 / dt):
                    steady_v.append(m[IDX["wheel_v"]])
                    steady_body_v.append(m[14])
                if abs(m[IDX["pitch"]]) > 0.6 or abs(m[IDX["roll"]]) > 0.6:
                    transition_failed = True
                    break
                lib.ident_set_leg_len_direct(ctypes.c_float(stair))
                out = IdentOut()
                lib.ident_step(ctypes.byref(fill_frame(m, raw)), dt, ctypes.byref(out))
                leg_servo(out.leg_swing, L_MID - out.leg_extend,
                          0.0, -out.leg_side_extend)
                wheel_ctrl(out.wheel_tau, out.yaw_tau)
                if os.environ.get("V3_CONTACT_DIAG") == "1" and \
                        transition_step >= point_steps - int(1.0 / dt) and \
                        transition_step % 10 == 0:
                    contact_tail.append(contact_diagnostic(m, out))
                if (transition_step >= point_steps - int(1.0 / dt) and
                        transition_step % DECIM == 0):
                    static_x_win.append(m[:12])
                    static_u_win.append([5.0 * (data.ctrl[2] + data.ctrl[5]),
                                         5.0 * (data.ctrl[5] - data.ctrl[2]),
                                         out.leg_swing, out.leg_extend,
                                         out.leg_side_extend, 0.0, 0.0, 0.0])
                mujoco.mj_step(model, data)
            not_static = not transition_failed and not (
                velocity_is_static(steady_v) and velocity_is_static(steady_body_v))
            if contact_tail:
                diag = np.asarray(contact_tail)
                print(f"    接地率 L/R={(diag[:, 0] > 0).mean():.2f}/"
                      f"{(diag[:, 1] > 0).mean():.2f}, "
                      f"法向力均值={diag[:, :2].mean(axis=0).round(2)}, "
                      f"前向轮/实/估速度={diag[:, 2:5].mean(axis=0).round(3)}, "
                      f"air/slip={diag[:, 5:7].mean(axis=0).round(2)}, "
                      f"pitch/tau/neutral={diag[:, 7:].mean(axis=0).round(3)}")
            if not transition_failed and not_static:
                print(f"  工作点{ip} ext={ext_ref:+.3f}: PID 未静止 "
                      f"(尾段轮速 {np.mean(steady_v):+.2f}m/s, "
                      f"机体速度 {m[14]:+.2f}m/s, x={m[15]:+.2f}m, "
                      f"L虚/实={m[IDX['leg_len']]:.3f}/{m[17]:.3f}m, "
                      f"sw={SW_NOM + m[IDX['leg_swing']]:+.3f}/"
                      f"{out.leg_swing:+.3f}/实{m[16]:+.3f}m)，不用于辨识")
                if os.environ.get("V3_CONTACT_DIAG") == "1":
                    contacts = sorted({tuple(sorted((
                        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
                        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2))))
                        for c in data.contact[:data.ncon]})
                    print("    接触:", contacts)
                    for c in data.contact[:data.ncon]:
                        names = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
                                 mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2))
                        if "floor" in names and "Wheel" in str(names):
                            wheel = wheel_bids[0] if "L_Wheel" in str(names) else wheel_bids[1]
                            print(f"    {names}: axle_z={data.xpos[wheel][2]:+.4f} "
                                  f"contact_z={c.pos[2]:+.4f} "
                                  f"effective_r={data.xpos[wheel][2]-c.pos[2]:.4f}")
                transition_failed = True
            if transition_failed:
                if not not_static:
                    print(f"  工作点{ip} ext={ext_ref:+.3f}: PID 过渡中失稳，跳过")
                continue
            STATIC_X.setdefault(ip, []).extend(static_x_win)
            STATIC_U.setdefault(ip, []).extend(static_u_win)
            run_id += 1
            win_start = len(SENS)          # 本窗口样本起点
            win_fallen = False
            dwell_v, dwell_body_v, dwell_pitch = [], [], []
            block_u = np.zeros(8)
            block_sample = None
            for step in range(n_dwell):
                if STATIC_ONLY:
                    prbs = dict.fromkeys(prbs, 0.0)
                elif step >= hold_next:
                    # 多样化: 保持时间随机(60~220ms => 4.5~15Hz 激励频段),
                    # 幅值也随机(0.35~1.0 倍), 避免单一频点/单一幅值导致的
                    # 系数方差(实测固定幅值+固定周期时 B 在段间乱跳变号)。
                    hold_next = step + int(rng.integers(150, 500))
                    # 逐窗口自适应: 两端加大激励(信噪比低)
                    _c = 0.5 * (N_SETPOINTS - 1)
                    end_boost = 1.0 + 0.9 * abs(ip - _c) / max(_c, 1.0)
                    amp = (0.35 + 0.65 * rng.random()) * end_boost
                    prbs["wheel"] = (PRBS_WHEEL * amp) * (1 if rng.random() > 0.5 else -1)
                    prbs["wdiff"] = (PRBS_WHEELDIFF * amp) * (1 if rng.random() > 0.5 else -1)
                    prbs["swing"] = (PRBS_SWING * amp) * (1 if rng.random() > 0.5 else -1)
                    prbs["diff"] = (PRBS_DIFF * amp) * (1 if rng.random() > 0.5 else -1)
                    prbs["ext"] = (PRBS_EXT * amp) * (1 if rng.random() > 0.5 else -1)
                    prbs["splay"] = (PRBS_SPLAY * amp) * (1 if rng.random() > 0.5 else -1)
                    prbs["rforce"] = (PRBS_RF * amp) * (1 if rng.random() > 0.5 else -1)
                    prbs["rtorque"] = (PRBS_RT * amp) * (1 if rng.random() > 0.5 else -1)

                # 阶梯过渡用斜率限制(瞬间跳 13mm 会把腿伺服顶翻)
                stair += float(np.clip(ext_ref - stair, -STAIR_RATE * dt, STAIR_RATE * dt))
                m, raw, _ = measure()
                if STATIC_ONLY:
                    dwell_pitch.append(m[IDX["pitch"]])
                if STATIC_ONLY and step >= n_dwell - int(1.0 / dt):
                    dwell_v.append(m[IDX["wheel_v"]])
                    dwell_body_v.append(m[14])
                # 摔倒检测: 本窗口内姿态越界就标记整窗作废(激励加大后更容易
                # 在极端腿长顶翻, 摔倒瞬态会让拟合发散, 必须整段剔除再重采)
                if abs(m[IDX["roll"]]) > 0.8 or abs(m[IDX["pitch"]]) > 0.8:
                    win_fallen = True
                if abs(m[IDX["roll"]]) > 0.5 or abs(m[IDX["pitch"]]) > 0.5:
                    inj = stair           # 姿态危险: 撤掉腿长激励, 先回正
                else:
                    inj = float(np.clip(stair + prbs["ext"], EXT_MIN, EXT_MAX))
                # 窗口内小步腿长抖动(打破回归量共线, 只 ±2mm)
                if not STATIC_ONLY:
                    inj += LEG_DITHER * math.sin(2.0 * math.pi * 0.8 * step * dt)
                lib.ident_set_leg_len_direct(ctypes.c_float(inj))
                out = IdentOut()
                lib.ident_step(ctypes.byref(fill_frame(m, raw)), dt, ctypes.byref(out))

                swing_cmd = out.leg_swing + prbs["swing"]
                # ident_set_leg_len 已进入 balance 的腿长参考，不能再次叠加 inj。
                ext_cmd = out.leg_extend
                diff_ext = out.leg_side_extend + prbs["diff"]
                leg_servo(swing_cmd, L_MID - ext_cmd, prbs["splay"], -diff_ext)
                pitch_tau = out.wheel_tau + prbs["wheel"]
                yaw_tau_cmd = out.yaw_tau + prbs["wdiff"]
                wheel_ctrl(pitch_tau, yaw_tau_cmd)
                if bid >= 0:
                    data.xfrc_applied[bid, 1] = prbs["rforce"]
                    data.xfrc_applied[bid, 3] = prbs["rtorque"]
                    if STATIC_ONLY:
                        pulse_on = 0.5 <= step * dt < 0.6
                        data.xfrc_applied[bid, 4] = PID_PITCH_PULSE if pulse_on else 0.0
                applied_u = np.array([5.0 * (data.ctrl[2] + data.ctrl[5]),
                                      5.0 * (data.ctrl[5] - data.ctrl[2]),
                                      swing_cmd, ext_cmd, diff_ext, prbs["splay"],
                                      prbs["rforce"], prbs["rtorque"]])
                if step % DECIM == 0:
                    block_u[:] = 0.0
                    block_sample = None
                    if (abs(m[IDX["pitch"]]) < math.radians(6.0) and
                            abs(m[IDX["roll"]]) < math.radians(8.0)):
                        if IDENT_LEG_LEN_MIN <= m[IDX["leg_len"]] <= IDENT_LEG_LEN_MAX:
                            SENS.append(m[:12])
                            U.append(None)
                            SEGID.append(ip)
                            RUNID.append(run_id)
                            SAMPLE_STEP.append(step)
                            CYCLE.append(cyc)
                            block_sample = len(U) - 1
                block_u += applied_u
                if step % DECIM == DECIM - 1 and block_sample is not None:
                    U[block_sample] = (block_u / DECIM).tolist()
                mujoco.mj_step(model, data)
            if STATIC_ONLY:
                if dwell_v and not (velocity_is_static(dwell_v) and
                                    velocity_is_static(dwell_body_v)):
                    win_fallen = True
                    print(f"  工作点{ip} 驻留时继续漂移 "
                          f"(尾段轮速 {np.mean(dwell_v):+.2f}m/s, "
                          f"机体速度 {np.mean(dwell_body_v):+.2f}m/s)，整窗剔除")
                valid_count = len(SENS) - win_start
                if valid_count < 0.95 * (n_dwell // DECIM):
                    win_fallen = True
                    print(f"  工作点{ip} 有效采样不足 "
                          f"({valid_count}/{n_dwell // DECIM})，整窗剔除")
                if not win_fallen and PID_PITCH_PULSE and dwell_pitch:
                    print(f"  工作点{ip} 俯仰扰动恢复: "
                          f"峰值 {np.degrees(np.max(np.abs(dwell_pitch))):.2f}deg, "
                          f"尾段轮速 {np.mean(dwell_v):+.3f}m/s, "
                          f"机体速度 {np.mean(dwell_body_v):+.3f}m/s")
            if win_fallen:
                del SENS[win_start:]
                del U[win_start:]
                del SEGID[win_start:]
                del RUNID[win_start:]
                del SAMPLE_STEP[win_start:]
                del CYCLE[win_start:]
                if not STATIC_ONLY:
                    print(f"  工作点{ip} 扰动后失稳，整窗剔除")

    if STATIC_ONLY:
        for ip, ext in enumerate(setpoints):
            selected = [i for i, sid in enumerate(SEGID) if sid == ip]
            rows = [SENS[i] for i in selected]
            if not rows:
                print(f"  静态 ext={ext:+.3f}: 无稳定样本")
                continue
            rows = np.array(rows)
            inputs = np.array([U[i] for i in selected])
            tail = rows[-min(20, len(rows)):]
            print(f"  静态 ext={ext:+.3f}: n={len(rows)} "
                  f"L={rows[:, IDX['leg_len']].mean():.3f}m "
                  f"|pitch|max={math.degrees(abs(rows[:, IDX['pitch']]).max()):.2f}deg "
                  f"|roll|max={math.degrees(abs(rows[:, IDX['roll']]).max()):.2f}deg "
                  f"v_end={tail[:, IDX['wheel_v']].mean():+.2f}m/s "
                  f"sw={SW_NOM + rows[:, IDX['leg_swing']].mean():+.3f}/"
                  f"{inputs[:, 2].mean():+.3f}m "
                  f"wheel_tau={inputs[:, 0].mean():+.2f}Nm")
        return

    SENS = np.array(SENS)
    U = np.array(U)
    SEGID = np.array(SEGID)
    RUNID = np.array(RUNID)
    SAMPLE_STEP = np.array(SAMPLE_STEP)
    CYCLE = np.array(CYCLE)
    print(f"样本 {len(SENS)} 条 (dt={dt*DECIM:.3f}s), "
          f"腿长范围 {SENS[:, IDX['leg_len']].min():.3f}~"
          f"{SENS[:, IDX['leg_len']].max():.3f}m")
    if len(SENS) < 500:
        raise SystemExit("样本太少, 辨识不可靠")

    # 只在同一连续驻留窗口内滤波；被姿态门限丢掉的样本形成断点。
    adjacent = ((RUNID[1:] == RUNID[:-1]) &
                (SAMPLE_STEP[1:] - SAMPLE_STEP[:-1] == DECIM))
    if FILT_FC > 0.0:
        fs_eff = 1.0 / (dt * DECIM)
        b, a = butter(2, FILT_FC / (fs_eff / 2.0), btype="low")
        cuts = np.r_[0, np.where(~adjacent)[0] + 1, len(SENS)]
        for start, end in zip(cuts[:-1], cuts[1:]):
            if end - start > 3 * max(len(a), len(b)):
                SENS[start:end] = filtfilt(b, a, SENS[start:end], axis=0)
                U[start:end] = filtfilt(b, a, U[start:end], axis=0)
        print(f"预滤波: {FILT_FC:.1f}Hz, 分 {len(cuts)-1} 个连续窗口")

    compact = np.arange(12)
    n, mm = 12, U.shape[1]   # mm 现在=6(含叉腿)
    valid_idx = np.where(adjacent)[0]
    Xk, Xn, Uk = SENS[valid_idx][:, compact], SENS[valid_idx + 1][:, compact], U[valid_idx]
    Phi = np.hstack([Xk, Uk, np.ones((len(Xk), 1))])
    theta, _, _, _ = np.linalg.lstsq(Phi, Xn, rcond=None)
    A, B, c = theta[:n, :].T, theta[n:n + mm, :].T, theta[n + mm, :]
    pred = Phi @ theta
    r2 = 1.0 - np.sum((Xn - pred) ** 2, axis=0) / np.maximum(
        np.sum((Xn - np.mean(Xn, axis=0)) ** 2, axis=0), 1e-12)
    print("全局 R2 =", np.array2string(r2, precision=3))

    # ---- 分段 + pitch/roll 子系统 LQR ----
    # 权重设计原则(用户: "能算的别拟合"):
    #   pitch/pitch_rate/wheel_v: 需要 LQR 调节 -> 给权重;
    #   leg_swing/leg_len 及其速率: 它们的动力学已由【空中辨识】给出
    #     F_sw = 0.208*swdd + 36.37*swd + 700.4*sw   (ident_leg_air.py)
    #     F_L  = -0.001*Ldd + 25.47*Ld + 1359.3*L
    #   是"可计算的已知环节", 由腿伺服闭环执行 + u_ss 前馈配平,
    #   不参与 K 的状态反馈(地面闭环里它们不可辨识, 5步R2为负)。
    Q_PITCH = np.diag([900.0, 12.0, 0.0, 0.0, 0.0, 0.0, 2.0])
    # 腿摆/腿长通道只做前馈(它们的地面动力学不可辨识, 由空中辨识参数
    # F_sw=0.208swdd+36.37swd+700.4sw / F_L=-0.001Ldd+25.47Ld+1359.3L 描述,
    # 由伺服闭环执行 + u_ss 配平)。用极大 R 把 K 的这两行压到 0。
    R_PITCH = np.diag([0.60, 1.0e6, 1.0e6])
    Q_ROLL = np.diag([500.0, 10.0, 160.0, 2.0, 1.0])
    # roll: 轮差速留给偏航环(它才是偏航执行器), LQR 只用差动腿长(几何找平)。
    R_ROLL = np.diag([1.0e6, 25.0])
    P_COLS = [IDX["pitch"], IDX["pitch_rate"], IDX["leg_swing"],
              IDX["leg_swing_rate"], IDX["leg_len"], IDX["leg_len_rate"],
              IDX["wheel_v"]]
    P_IN = [0, 2, 3]
    R_COLS = [IDX["roll"], IDX["roll_rate"], IDX["leg_len_diff"],
              IDX["leg_len_diff_rate"], IDX["wheel_diff_v"]]
    R_IN = [1, 4]

    def fit_seg(mask, point_id):
        idx = np.where(mask[:-1] & mask[1:] & adjacent)[0]
        if SWEEP_CYCLES > 1:
            idx = idx[CYCLE[idx] < SWEEP_CYCLES - 1]
        if len(idx) < 60:
            return None
        As, Bs, cs = fit_structured(SENS, U, idx, dt * DECIM)
        A3, B3, c3 = fit_pitch3_structured(SENS, U, idx, dt * DECIM)
        trim = np.mean(STATIC_X[point_id], axis=0)
        return As, Bs, cs, A3, B3, c3, trim, len(idx)

    segs = []
    # 按"阶梯工作点"分段(不是按实测腿长分箱): 每个驻留点样本均匀, 边界取
    # 相邻工作点实测腿长的中点。控制器用实测 leg_len 选段, 所以边界用实测值。
    means = {}
    for ip in sorted(set(SEGID.tolist())):
        msk = SEGID == ip
        if msk.sum() < 60:
            continue
        means[ip] = float(np.mean(np.array(STATIC_X[ip])[:, IDX["leg_len"]]))
    ordered = sorted(means.items(), key=lambda kv: kv[1])
    coverage_ok = (len(ordered) == N_SETPOINTS and
                   ordered[0][1] <= IDENT_LEG_LEN_MIN + 0.010 and
                   ordered[-1][1] >= IDENT_LEG_LEN_MAX - 0.010)
    if not coverage_ok:
        print(f"!! 稳定工作点只覆盖 {ordered[0][1]:.3f}~{ordered[-1][1]:.3f}m "
              f"({len(ordered)}/{N_SETPOINTS} 点)，不足以发布全腿长模型")
    for k, (ip, Lm) in enumerate(ordered):
        # Keep the identified table bounded by the physical sweep envelope.
        # The previous 0/1 m sentinels made the header look complete while the
        # first and last bins had no measurements at all.
        lo = IDENT_LEG_LEN_MIN if k == 0 else 0.5 * (ordered[k - 1][1] + Lm)
        hi = IDENT_LEG_LEN_MAX if k == len(ordered) - 1 else 0.5 * (Lm + ordered[k + 1][1])
        lo = max(IDENT_LEG_LEN_MIN, min(IDENT_LEG_LEN_MAX, lo))
        hi = max(IDENT_LEG_LEN_MIN, min(IDENT_LEG_LEN_MAX, hi))
        if hi <= lo:
            print(f"  工作点{ip} L={Lm:.3f} 区间退化, 跳过")
            continue
        mask = SEGID == ip
        r = fit_seg(mask, ip)
        if r is None:
            print(f"  工作点{ip} L={Lm:.3f} 样本不足, 跳过")
            continue
        As, Bs, cs, A3, B3, c3, trim, cnt = r
        # 配平来自激励前的纯 PID 静止段，不能从受扰动数据平均或病态 B 反解。
        x_ss = trim.copy()
        u_ss = np.mean(STATIC_U[ip], axis=0)
        Ap, Bp = As[np.ix_(P_COLS, P_COLS)], Bs[np.ix_(P_COLS, P_IN)]
        Ar, Br = As[np.ix_(R_COLS, R_COLS)], Bs[np.ix_(R_COLS, R_IN)]
        Kp, Kr = solve_dare(Ap, Bp, Q_PITCH, R_PITCH), solve_dare(Ar, Br, Q_ROLL, R_ROLL)
        # Replace only the pitch wheel row with the independent three-state
        # solution. The leg reference rows stay in their own closed-loop
        # subsystem and cannot alter wheel damping through cross-correlation.
        K3 = solve_dare(A3, B3, np.diag([900.0, 12.0, 2.0]), np.array([[0.60]]))
        Kp[0, :] = 0.0
        Kp[0, [0, 1, 6]] = K3[0]
        segs.append(dict(lo=lo, hi=hi, cnt=cnt, point_id=ip,
                         Ap=Ap.tolist(), Bp=Bp.tolist(), Kp=Kp.tolist(),
                         tp=trim[P_COLS].tolist(),
                         Ar=Ar.tolist(), Br=Br.tolist(), Kr=Kr.tolist(),
                         tr=trim[R_COLS].tolist(),
                         x_ss=x_ss.tolist(), u_ss=u_ss.tolist(),
                         up=np.array(P_IN and [u_ss[i] for i in P_IN]).tolist(),
                         ur=[u_ss[i] for i in R_IN],
                         xp_ss=[x_ss[i] for i in P_COLS],
                         xr_ss=[x_ss[i] for i in R_COLS],
                         A=As.tolist(), B=Bs.tolist(), c=cs.tolist(),
                         pitch3_A=A3.tolist(), pitch3_B=B3.tolist(),
                         pitch3_K=K3.tolist(),
                         pitch3_c=c3.tolist(),
                         trim=trim.tolist()))
        print(f"  工作点{ip} L={Lm:.3f} 段[{lo:.3f},{hi:.3f}] n={cnt} "
              f"Kp[0]={Kp[0,0]:+7.2f} Kr[0]={Kr[0,0]:+7.2f} | "
              f"u_ss=[wheel {u_ss[0]:+.3f} yaw {u_ss[1]:+.3f} "
              f"sw {u_ss[2]:+.3f} ext {u_ss[3]:+.3f} diff {u_ss[4]:+.3f}]")
    if not segs:
        raise SystemExit("没有可用分段")

    # ---- 模型健全性验收 ----
    import numpy.linalg as la
    ok = coverage_ok and FILT_FC == 0.0
    if FILT_FC != 0.0:
        print("!! 离线双向滤波与实时状态定义不一致，仅允许诊断，不能发布")
    ks = [np.array(sg["Kp"])[0][0] for sg in segs]
    for i, sg in enumerate(segs):
        # Pitch stability must be checked against the same independent
        # three-state plant used to compute the wheel row of Kp.
        ap, bp, kp = (np.array(sg[k]) for k in ("pitch3_A", "pitch3_B", "pitch3_K"))
        ar, br, kr = (np.array(sg[k]) for k in ("Ar", "Br", "Kr"))
        rho_p = max(abs(la.eigvals(ap - bp @ kp)))
        rho_r = max(abs(la.eigvals(ar - br @ kr)))
        uss = sg["u_ss"]
        bad = []
        if not np.isfinite(rho_p) or rho_p >= 1.0: bad.append(f"pitch闭环谱半径{rho_p:.3f}")
        if not np.isfinite(rho_r) or rho_r >= 1.0: bad.append(f"roll闭环谱半径{rho_r:.3f}")
        if abs(uss[2]) > 0.05: bad.append(f"腿摆配平{uss[2]:+.3f}")
        if abs(uss[0]) > 4.0: bad.append(f"轮配平{uss[0]:+.2f}")
        if abs(uss[3]) > 0.08: bad.append(f"腿长配平{uss[3]:+.3f}")
        if i > 0 and np.sign(ks[i]) != np.sign(ks[i - 1]) and min(abs(ks[i]), abs(ks[i-1])) > 0.5:
            bad.append(f"Kp符号翻转({ks[i-1]:+.1f}->{ks[i]:+.1f})")
        if bad:
            ok = False
            print(f"  !! 段{i+1} 不合格: " + ", ".join(bad))
    # 无论模型采样周期如何变化，都验证相同的 125 ms 物理时间。
    # R2 在几乎不动的工作点分母太小，仅作诊断；发布门槛使用绝对误差。
    validation_horizon = 0.125
    validation_steps = max(1, round(validation_horizon / (dt * DECIM)))
    def validate_seg(seg, h):
        mask = SEGID == seg["point_id"]
        candidate = np.arange(len(SENS) - h)
        valid = mask[candidate + h].copy()
        for j in range(h):
            valid &= mask[candidate + j] & adjacent[candidate + j]
        if SWEEP_CYCLES > 1:
            valid &= CYCLE[candidate] == SWEEP_CYCLES - 1
        idx = candidate[valid]
        if len(idx) < 50:
            return None
        A_, B_, c_ = np.array(seg["A"]), np.array(seg["B"]), np.array(seg["c"])
        P = SENS[idx][:, :].copy()
        for j in range(h):
            P = P @ A_.T + U[idx + j] @ B_.T + c_
        y = SENS[idx + h]
        rmse = np.sqrt(np.mean((y - P) ** 2, axis=0))
        persistence = np.sqrt(np.mean((y - SENS[idx]) ** 2, axis=0))
        r2 = 1 - np.sum((y - P) ** 2, axis=0) / np.maximum(
            np.sum((y - y.mean(0)) ** 2, axis=0), 1e-12)
        return rmse, persistence, r2
    validation_all = []
    for i, sg in enumerate(segs):
        result = validate_seg(sg, validation_steps)
        if result is None:
            ok = False
            print(f"  !! 段{i+1} 没有足够的独立验证窗口")
            continue
        rmse, persistence, rr = result
        sg["validation_rmse"] = rmse.tolist()
        sg["validation_r2"] = rr.tolist()
        validation_all.append(rmse)
        # 俯仰误差 0.5°、侧倾 0.3°、腿长 1.5 mm、轮速 0.12 m/s。
        # 这些是 125 ms 开环预测误差，不因窗口方差大小而改变。
        limits = {IDX["pitch"]: math.radians(0.5),
                  IDX["roll"]: math.radians(0.3),
                  IDX["leg_len"]: 0.0015,
                  IDX["wheel_v"]: 0.20}
        failed = [name for name, col in (("pitch", 0), ("roll", 2),
                                         ("leg_len", 6), ("wheel_v", 10))
                  if rmse[col] > limits[col] or
                     (col == IDX["wheel_v"] and
                      rmse[col] > max(0.12, 0.70 * persistence[col]))]
        print(f"  段{i+1} L≈{0.5*(sg['lo']+sg['hi']):.3f} "
              f"{validation_steps}步/{validation_steps*dt*DECIM*1000:.0f}ms "
              f"RMSE[pitch {math.degrees(rmse[0]):.3f}° "
              f"roll {math.degrees(rmse[2]):.3f}° "
              f"L {rmse[6]*1000:.2f}mm v {rmse[10]:.3f}m/s] "
              f"基线[pitch {math.degrees(persistence[0]):.3f}° "
              f"v {persistence[10]:.3f}m/s] "
              f"未达标={','.join(failed) if failed else '无'}")
        if failed:
            ok = False
    if not validation_all:
        ok = False
        print("!! 没有连续的独立验证数据")
    else:
        worst = np.max(np.asarray(validation_all)[:, [0, 2, 6, 10]], axis=0)
        print("最差 125ms RMSE [pitch°,roll°,L mm,v m/s] =",
              [round(math.degrees(worst[0]), 3), round(math.degrees(worst[1]), 3),
               round(worst[2]*1000, 3), round(worst[3], 3)])
    print("模型验收:", "通过" if ok else "不通过(需继续加数据/改约束)")

    # 段间平滑 K(相邻工作点动力学连续, 增益不该跳变或变号)
    if len(segs) >= 3:
        for key, nrow, ncol in (("Kp", 3, 7), ("Kr", 2, 5)):
            Ks = [np.array(sg[key]) for sg in segs]
            sm = []
            for i in range(len(Ks)):
                if i == 0:
                    sm.append(0.7 * Ks[0] + 0.3 * Ks[1])
                elif i == len(Ks) - 1:
                    sm.append(0.7 * Ks[-1] + 0.3 * Ks[-2])
                else:
                    sm.append(0.6 * Ks[i] + 0.2 * Ks[i - 1] + 0.2 * Ks[i + 1])
            for sg, v in zip(segs, sm):
                sg[key] = v.tolist()
        print("已对 Kp/Kr 做段间平滑")

    # 保留短腿和长腿两端的辨识段。异常工况可通过显式
    # V3_LEG_WINDOW_MAX 截断，但默认必须覆盖整个物理扫描包络。
    window_max_env = os.environ.get("V3_LEG_WINDOW_MAX")
    if window_max_env:
        window_max = float(window_max_env)
        keep = [sg for sg in segs
                if 0.5 * (sg["lo"] + min(sg["hi"], IDENT_LEG_LEN_MAX)) <= window_max]
        if keep:
            keep[-1]["hi"] = IDENT_LEG_LEN_MAX
            print(f"显式腿长窗口 {window_max:.3f}m: {len(segs)} -> {len(keep)} 段")
            segs = keep

    # A fit over the GUI/PID operating interval is useful diagnostics, but it
    # cannot authorize a controller that is allowed to enter at any leg length.
    full_range_requested = (EXT_MIN <= -0.040 and EXT_MAX >= 0.040 and
                            IDENT_LEG_LEN_MIN <= 0.065 and
                            IDENT_LEG_LEN_MAX >= 0.160 and
                            window_max_env is None)
    deploy_ok = ok and full_range_requested
    if ok and not full_range_requested:
        print("!! 当前仅辨识部分腿长，不能发布任意腿长接管的 LQR 模型")

    out_dir = os.path.join(PROJECT_ROOT, "logs")
    report_base = os.path.join(out_dir, "three_loop_model_v3_" +
                               time.strftime("%Y%m%d_%H%M%S"))
    report_json = report_base + ".json"
    report_npz = report_base + ".npz"
    with open(report_json, "w") as f:
        json.dump({"generated_at": time.strftime("%Y%m%d_%H%M%S"),
                   "xml": XML, "dt_physics": dt, "dt_model": dt * DECIM,
                   "decimation": DECIM, "n_samples": len(SENS),
                   "validation_horizon_sec": validation_steps * dt * DECIM,
                   "pid_only": True, "model_fit_passed": bool(ok),
                   "accepted": bool(deploy_ok),
                   "stable_leg_centers": [v for _, v in ordered],
                   "state_names": list(IDX.keys()),
                   "input_names": ["wheel_common_tau", "wheel_diff_tau",
                                   "leg_swing_ref", "leg_len_ref",
                                   "leg_len_diff_ref", "leg_splay_ref",
                                   "lateral_force", "roll_torque"],
                   "A": A.tolist(), "B": B.tolist(), "c": c.tolist(),
                   "r2": r2.tolist(), "segs": segs}, f, indent=2)
    np.savez(report_npz,
             sens=SENS, u=U, segment_id=SEGID, run_id=RUNID,
             sample_step=SAMPLE_STEP, cycle=CYCLE, A=A, B=B, c=c)
    print("诊断数据:", report_json, report_npz)

    if not deploy_ok:
        raise SystemExit("PID 辨识未通过全腿长发布验收；诊断数据已保存，部署 LQR 头文件未覆盖")
    shutil.copyfile(report_json, os.path.join(out_dir, "three_loop_model_v3.json"))
    shutil.copyfile(report_npz, os.path.join(out_dir, "three_loop_model_v3.npz"))

    # ---- 生成 C 头 ----
    def emit_mat(name, M):
        return "    { " + name + ",\n" + "\n".join(
            "      {" + ",".join(f"{v:.8e}f" for v in row) + "}," for row in M
        ) + "\n    },\n"

    def emit(path, guard, sname, tname, arr, nstate, ninput, keyA, keyB, keyK, keyT):
        """输出与 lqr_controller.c 对接的头文件:
        数组名 lqr_pitch_segments/lqr_roll_segments、类型 LqrPitchSegment/
        LqrRollSegment、共用 LQR_N_SEG(pitch/roll 必须同分段数、同区间,
        因为控制器用一个 g_seg 同时索引两边)。"""
        L = ["/* %s - 由 scripts/tools/identify_lqr_v3.py 生成, 勿手改 */" % os.path.basename(path),
             "/* 仅在完整覆盖、闭环极点和独立轮次预测通过后生成。 */",
             f"#ifndef {guard}", f"#define {guard}", ""]
        if sname == "LQR_PITCH":
            L += ["#define LQR_IDENT_ACCEPTED 1",
                  "#define LQR_IDENT_FILTER_ENABLED 0"]
        L += [f"#define LQR_N_SEG {len(segs)}",
             f"#define {sname}_N_STATE {nstate}",
             f"#define {sname}_N_INPUT {ninput}",
             f"#define {sname}_SAMPLE_DT {dt*DECIM:.6f}f", "",
             "typedef struct {",
             "    float lo, hi;",
             f"    float A[{nstate}][{nstate}];",
             f"    float B[{nstate}][{ninput}];",
             f"    float K[{ninput}][{nstate}];",
             f"    float x_trim[{nstate}];",
             f"    float x_ss[{nstate}];   /* 独占模式目标稳态 */",
             f"    float u_ss[{ninput}];   /* 独占模式配平输入 */",
             f"}} {tname};", "",
             f"static const {tname} {arr}[{len(segs)}] = {{"]
        for s in segs:
            L.append("  {")
            L.append(f"    {s['lo']:.6f}f, {s['hi']:.6f}f,")
            L.append("    { /* A */")
            for row in s[keyA]:
                L.append("      {" + ",".join(f"{v:.8e}f" for v in row) + "},")
            L.append("    },")
            L.append("    { /* B */")
            for row in s[keyB]:
                L.append("      {" + ",".join(f"{v:.8e}f" for v in row) + "},")
            L.append("    },")
            L.append("    { /* K */")
            for row in s[keyK]:
                L.append("      {" + ",".join(f"{v:.8e}f" for v in row) + "},")
            L.append("    },")
            L.append("    { " + ",".join(f"{v:.6e}f" for v in s[keyT]) + " },")
            xss = s["xp_ss"] if keyT == "tp" else s["xr_ss"]
            uss = s["up"] if keyT == "tp" else s["ur"]
            L.append("    { " + ",".join(f"{v:.6e}f" for v in xss) + " },")
            L.append("    { " + ",".join(f"{v:.6e}f" for v in uss) + " },")
            L.append("  },")
        L += ["};", "", f"#endif /* {guard} */", ""]
        with open(path, "w") as f:
            f.write("\n".join(L))
        print("wrote", path)

    lqr_dir = os.path.join(PROJECT_ROOT, "control", "controllers", "lqr")
    emit(os.path.join(lqr_dir, "lqr_pitch_models.h"), "LQR_PITCH_MODELS_H",
         "LQR_PITCH", "LqrPitchSegment", "lqr_pitch_segments",
         7, 3, "Ap", "Bp", "Kp", "tp")
    emit(os.path.join(lqr_dir, "lqr_roll_models.h"), "LQR_ROLL_MODELS_H",
         "LQR_ROLL", "LqrRollSegment", "lqr_roll_segments",
         5, 2, "Ar", "Br", "Kr", "tr")

    # ---- 3 状态俯仰子系统 + 按通道 valid 标志 ----
    # 验收指标从 R2 换成 NRMSE: R2 在"安静段"(状态几乎不动的窗口)分母接近 0
    # 会炸出 -1e5 的假象(实测段6), 不是模型坏。NRMSE = RMSE/max(std, 物理下限)。
    P3 = [IDX["pitch"], IDX["pitch_rate"], IDX["wheel_v"]]
    IN3 = [0]
    FLOOR3 = [0.005, 0.05, 0.02]      # 各状态方差的物理下限(rad, rad/s, m/s)
    LIMIT3 = float(os.environ.get("V3_NRMSE_LIMIT", "0.7"))
    p3segs = []
    for sg in segs:
        A = np.array(sg["pitch3_A"])
        B = np.array(sg["pitch3_B"])
        c = np.array(sg["pitch3_c"])
        mask = SEGID == sg["point_id"]
        h = validation_steps
        candidate = np.arange(len(SENS) - h)
        valid = mask[candidate + h].copy()
        for j in range(h):
            valid &= mask[candidate + j] & adjacent[candidate + j]
        if SWEEP_CYCLES > 1:
            valid &= CYCLE[candidate] == SWEEP_CYCLES - 1
        idx = candidate[valid]
        # 验收用【绝对误差】(控制相关量纲): 125ms 预测的 RMSE。
        # 阈值: pitch < 0.5°, wheel_v < 0.12 m/s —— 直接对应"这点误差会不会
        # 让 LQR 的阻尼项做错事"。归一化比值(R2/NRMSE)会随窗口安静程度漂移,
        # 中部安静就被误判成"差", 两端抖就被误判成"好"。
        nrmse = [9.9, 9.9, 9.9]
        if len(idx) > 50:
            Y = SENS[idx + h][:, P3]
            P_ = SENS[idx][:, P3].copy()
            for j in range(h):
                P_ = P_ @ A.T + U[idx + j][:, IN3] @ B.T + c
            for k in range(3):
                nrmse[k] = np.sqrt(np.mean((Y[:, k] - P_[:, k]) ** 2))
        vp = 1 if nrmse[0] < math.radians(0.5) else 0
        vv = 1 if nrmse[2] < 0.12 else 0
        # DARE
        Q = np.diag([900.0, 12.0, 2.0]); R = np.array([[0.6]])
        Pk = Q.copy()
        for _ in range(20000):
            Pn = Q + A.T @ Pk @ A - A.T @ Pk @ B @ np.linalg.solve(R + B.T @ Pk @ B, B.T @ Pk @ A)
            if np.linalg.norm(Pn - Pk, "fro") < 1e-11:
                Pk = Pn; break
            Pk = Pn
        K = np.linalg.solve(R + B.T @ Pk @ B, B.T @ Pk @ A)
        uss = float(sg["u_ss"][0])
        p3segs.append(dict(lo=sg["lo"], hi=sg["hi"], A=A.tolist(), B=B.tolist(),
                           K=K.tolist(), uss=uss, vp=vp, vv=vv, nrmse=nrmse))
    print("\n3状态俯仰子系统 125ms 绝对误差(RMSE) 与 valid 标志:")
    for i, g in enumerate(p3segs):
        print(f"  段{i+1} L≈{0.5*(g['lo']+min(g['hi'],0.2)):.3f} "
              f"RMSE=[pitch {math.degrees(g['nrmse'][0]):5.2f}deg "
              f"prate {g['nrmse'][1]:6.3f} v {g['nrmse'][2]:6.3f}m/s] "
              f"valid_pitch={g['vp']} valid_v={g['vv']}")
    H = ["/* lqr_pitch3_models.h - 由 identify_lqr_v3.py 生成, 勿手改",
         " * 3 状态俯仰子系统 [pitch, pitch_rate, wheel_v] <- [共模轮扭矩]",
         " * valid_pitch/valid_v: 该窗口是否通过 NRMSE 验收(按通道), ",
         " * 控制器据此逐个窗口决定是否接管(姿态 / 运动各自独立)。 */",
         "#ifndef LQR_PITCH3_MODELS_H", "#define LQR_PITCH3_MODELS_H", "",
         f"#define LQR_P3_N_SEG {len(p3segs)}", "#define LQR_P3_N_STATE 3",
         "#define LQR_P3_N_INPUT 1", "",
         "typedef struct {", "    float lo, hi;", "    float A[3][3];",
         "    float B[3][1];", "    float K[1][3];", "    float u_ss[1];",
         "    int valid_pitch;", "    int valid_v;", "} LqrPitch3Segment;", "",
         f"static const LqrPitch3Segment lqr_pitch3_segments[{len(p3segs)}] = {{"]
    for g in p3segs:
        H.append("  {")
        H.append(f"    {g['lo']:.6f}f, {g['hi']:.6f}f,")
        H.append("    { " + "\n".join("      {" + ",".join(f"{v:.8e}f" for v in r) + "}," for r in g["A"]) + "\n    },")
        H.append("    { " + "\n".join("      {" + ",".join(f"{v:.8e}f" for v in r) + "}," for r in g["B"]) + "\n    },")
        H.append("    { " + "\n".join("      {" + ",".join(f"{v:.8e}f" for v in r) + "}," for r in g["K"]) + "\n    },")
        H.append(f"    {{ {g['uss']:.6e}f }},")
        H.append(f"    {g['vp']}, {g['vv']},")
        H.append("  },")
    H += ["};", "", "#endif /* LQR_PITCH3_MODELS_H */", ""]
    with open(os.path.join(PROJECT_ROOT, "control", "controllers", "lqr",
                           "lqr_pitch3_models.h"), "w") as f:
        f.write("\n".join(H))
    print("wrote lqr_pitch3_models.h (含 valid 标志)")
    subprocess.run([sys.executable, os.path.join(PROJECT_ROOT, "scripts", "tools",
                    "synthesize_lqr_pitch_damping.py"), report_json], check=True)

    print("\n完成: logs/three_loop_model_v3.json + lqr_pitch_models.h + lqr_roll_models.h")


if __name__ == "__main__":
    main()

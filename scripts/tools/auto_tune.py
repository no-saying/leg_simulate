#!/usr/bin/env python3
"""
系统辨识 + LQR 增益计算 (MuJoCo 3.x, 纯 numpy)
===============================================

完整模型
-------
控制量 u = [τ_pitch,  τ_yaw,  Δsw]ᵀ  (3维)
  τ_pitch = (τ_L + τ_R)/2    轮毂共模扭矩 → 前后平衡  [Nm]
  τ_yaw   = (τ_R - τ_L)/2    轮毂差动扭矩 → 偏航控制  [Nm]
  Δsw     = sw - 0.23        腿摆动偏移 → COM移动    [rad]

状态量 x = [θ_pitch, θ̇_pitch, φ̇_avg, θ_yaw, θ̇_yaw]ᵀ  (5阶)
  θ_pitch  : pitch 角度 (IMU 四元数)
  θ̇_pitch : pitch 角速度 (陀螺仪 gyro[1])
  φ̇_avg   : 两轮平均角速度
  θ_yaw    : yaw 角度 (IMU 四元数)
  θ̇_yaw   : yaw 角速度 (陀螺仪 gyro[2])

输出量 y = [θ_pitch,  θ_yaw,  θ̇_yaw]ᵀ

腿关节角度映射 (controller.c):
  LF=sw,  LR=0.4-sw,  RF=-0.1-sw,  RR=-0.4+sw
  sw增大 → COM前移 → 产生后仰力矩 → 对抗前倾
"""

import mujoco
import numpy as np
import os, sys

os.environ['MUJOCO_GL'] = 'osmesa'
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML = os.path.join(PROJECT_ROOT, 'model', 'COD-2026RoboMaster-Balance.xml')

model = mujoco.MjModel.from_xml_path(XML)
data = mujoco.MjData(model)
home = model.key('home')
data.qpos[:] = home.qpos
data.qvel[:] = 0
mujoco.mj_forward(model, data)

base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base')
lw_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'Left_Wheel_link')
rw_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'Right_Wheel_link')

g = 9.81
dt = model.opt.timestep
r_wheel = 0.06

# ============================================================
# 辅助函数
# ============================================================
def subtree_mass(bid):
    total = model.body_mass[bid]
    for j in range(model.nbody):
        if model.body_parentid[j] == bid:
            total += subtree_mass(j)
    return total

def is_descendant(body_id, ancestor_id):
    p = body_id
    while p != 0:
        if p == ancestor_id: return True
        p = model.body_parentid[p]
    return False

def expm_pade(A, order=6):
    n = A.shape[0]
    I = np.eye(n)
    norm = np.linalg.norm(A, ord=1)
    k = max(0, int(np.ceil(np.log2(norm)))) if norm > 1 else 0
    A_s = A / (2 ** k)
    c = np.array([1.0, 1/2, 5/44, 1/66, 1/792, 1/15840, 1/665280])
    N = np.zeros((n, n)); D = np.zeros((n, n))
    Apow = I.copy()
    for j in range(7):
        N += c[j] * Apow
        D += c[j] * (-1)**j * Apow
        Apow = Apow @ A_s
    R = np.linalg.solve(D, N)
    for _ in range(k): R = R @ R
    return R

def solve_dare_iter(A, B, Q, R, max_iter=5000, tol=1e-10):
    n = A.shape[0]
    P = Q.copy()
    for _ in range(max_iter):
        P_new = Q + A.T @ P @ A - A.T @ P @ B @ np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        if np.linalg.norm(P_new - P, 'fro') < tol: break
        P = P_new
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    return P, K

def discretize(A_ct, B_ct, dt):
    n, m = A_ct.shape[0], B_ct.shape[1]
    M_aug = np.zeros((n + m, n + m))
    M_aug[:n, :n] = A_ct * dt
    M_aug[:n, n:] = B_ct * dt
    M_exp = expm_pade(M_aug)
    return M_exp[:n, :n], M_exp[:n, n:]

# ============================================================
# 1. 提取物理参数
# ============================================================
M = subtree_mass(base_id)

# 设置腿到 nominal 位置并计算参数
def set_legs(sw):
    lf = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'Left_front_joint')
    lr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'Left_rear_joint')
    rf = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'Right_front_joint')
    rr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'Right_rear_joint')
    data.qpos[:] = home.qpos
    data.qpos[model.jnt_qposadr[lf]] = sw
    data.qpos[model.jnt_qposadr[lr]] = 0.4 - sw
    data.qpos[model.jnt_qposadr[rf]] = -0.1 - sw
    data.qpos[model.jnt_qposadr[rr]] = -0.4 + sw
    mujoco.mj_forward(model, data)

def compute_params():
    """计算 COM, 惯量 (依赖当前 data 状态)"""
    com = data.subtree_com[base_id].copy()
    lw = data.xpos[lw_id].copy()
    rw = data.xpos[rw_id].copy()
    axle = 0.5 * (lw + rw)
    dx = com[0] - axle[0]
    dz = com[2] - axle[2]
    track = abs(rw[1] - lw[1])

    I_p = 0.0
    for i in range(model.nbody):
        if i == 0 or not is_descendant(i, base_id): continue
        mi = model.body_mass[i]
        Iyy = model.body_inertia[i][1]
        bc = data.subtree_com[i] if i == base_id else data.xipos[i]
        d2 = (bc[0]-axle[0])**2 + (bc[2]-axle[2])**2
        I_p += Iyy + mi*d2

    I_y = 0.0
    for i in range(model.nbody):
        if i == 0 or not is_descendant(i, base_id): continue
        mi = model.body_mass[i]
        Izz = model.body_inertia[i][2]
        bc = data.subtree_com[i] if i == base_id else data.xipos[i]
        d2 = (bc[0]-com[0])**2 + (bc[1]-com[1])**2
        I_y += Izz + mi*d2

    return dx, dz, track, I_p, I_y

# 与 control/balance.h 的 BALANCE_LEG_SW_NOM 对齐，保证离线质心/腿摆估计不偏工作点。
sw_nom = 0.23
set_legs(sw_nom)
dx0, dz0, track, I_p0, I_y0 = compute_params()

# 轮子惯量
inert_lw = model.body_inertia[lw_id]
inert_rw = model.body_inertia[rw_id]
I_wheel = 0.5 * (inert_lw[1] + inert_rw[1])

# 灵敏度 (数值微分)
eps = 0.01
set_legs(sw_nom + eps)
dx_p, dz_p, _, I_p_p, _ = compute_params()
set_legs(sw_nom - eps)
dx_m, dz_m, _, I_p_m, _ = compute_params()

ddx_dsw = (dx_p - dx_m) / (2*eps)
ddz_dsw = (dz_p - dz_m) / (2*eps)
dI_dsw  = (I_p_p - I_p_m) / (2*eps)
set_legs(sw_nom)  # restore

print(f"{'='*60}")
print(f"物理参数 (sw={sw_nom})")
print(f"{'='*60}")
print(f"  M          = {M:.3f} kg")
print(f"  COM vs axle: dx={dx0:.4f} dz={dz0:.4f} m")
print(f"  track      = {track:.4f} m")
print(f"  r_wheel    = {r_wheel:.3f} m")
print(f"  I_pitch    = {I_p0:.6f} kg·m²")
print(f"  I_yaw      = {I_y0:.6f} kg·m²")
print(f"  I_wheel    = {I_wheel:.6f} kg·m²")
print(f"\n  灵敏度 (∂/∂sw):")
print(f"  ddx/dsw    = {ddx_dsw:.4f}  (COM前移)")
print(f"  ddz/dsw    = {ddz_dsw:.4f}  (COM升高)")
print(f"  dI/dsw     = {dI_dsw:.4f}   (惯量变化)")
print(f"  ∂τ_g/∂sw   = {M*g*ddx_dsw:.2f} Nm/unit (重力矩灵敏度)")

# ============================================================
# 2. 耦合质量矩阵 & Pitch 基础参数
# ============================================================
M11 = I_p0
M12 = M * dz0 * r_wheel
M22 = M * r_wheel**2 + 2 * I_wheel
detM = M11 * M22 - M12**2
inv_M11 = M22 / detM
inv_M12 = -M12 / detM
inv_M22 = M11 / detM

a21 = inv_M11 * M * g * dz0     # ∂θ̈/∂θ
a31 = inv_M12 * M * g * dz0     # ∂φ̈/∂θ
b2  = inv_M11 + inv_M12          # ∂θ̈/∂τ_pitch
b3  = inv_M12 + inv_M22          # ∂φ̈/∂τ_pitch

# 腿摆动对 θ̈ 的直接影响
b_sw = M * g * ddx_dsw / I_p0   # ∂θ̈/∂sw

print(f"\n  Pitch 基础: a21={a21:.3f} b2={b2:.3f} b_sw={b_sw:.3f}")

# ============================================================
# 3. 增广状态空间 (5状态, 3控制)
# ============================================================
# x = [θ_pitch, θ̇_pitch, φ̇_avg, θ_yaw, θ̇_yaw]
# u = [τ_pitch, τ_yaw, Δsw]

A_ct = np.array([
    #  dθ      dθ̇        dφ̇       dψ       dψ̇
    [  0.0,    1.0,       0.0,      0.0,     0.0  ],  # d(θ)/dt
    [  a21,    0.0,       0.0,      0.0,     0.0  ],  # d(θ̇)/dt
    [  a31,    0.0,       0.0,      0.0,     0.0  ],  # d(φ̇)/dt
    [  0.0,    0.0,       0.0,      0.0,     1.0  ],  # d(ψ)/dt
    [  0.0,    0.0,       0.0,      0.0,     0.0  ],  # d(ψ̇)/dt
])

b_yaw = track / (r_wheel * I_y0)

B_ct = np.array([
    # τ_pitch   τ_yaw    Δsw
    [  0.0,      0.0,     0.0   ],  # d(θ)/dt
    [  b2,       0.0,     b_sw  ],  # d(θ̇)/dt
    [  b3,       0.0,     0.0   ],  # d(φ̇)/dt
    [  0.0,      0.0,     0.0   ],  # d(ψ)/dt
    [  0.0,      b_yaw,   0.0   ],  # d(ψ̇)/dt
])

print(f"\n{'='*60}")
print(f"增广模型: 5状态 x 3控制")
print(f"{'='*60}")
print(f"  x = [θ_pitch, θ̇_pitch, φ̇_avg, θ_yaw, θ̇_yaw]ᵀ")
print(f"  u = [τ_pitch, τ_yaw, Δsw]ᵀ")
print(f"  b_yaw = {b_yaw:.3f}  b_sw = {b_sw:.3f}")
print(f"\nA_ct =\n{A_ct}")
print(f"B_ct =\n{B_ct}")

# 开环极点
ev = np.linalg.eigvals(A_ct)
print(f"\n开环极点: {np.array2string(ev, precision=2)}")

# ============================================================
# 4. LQR 求解
# ============================================================
# 权重: [θ_pitch, θ̇_pitch, φ̇_avg, θ_yaw, θ̇_yaw]
Q_theta_p = 500.0   # pitch 角度
Q_theta_y = 100.0   # yaw 角度
R_tau_p   = 0.001   # 轮扭矩代价
R_tau_y   = 0.01    # yaw 扭矩代价
R_sw      = 50.0    # 腿摆动代价 (大 = 少用腿)

for i, a in enumerate(sys.argv):
    if a == '--qp' and i+1 < len(sys.argv): Q_theta_p = float(sys.argv[i+1])
    if a == '--qy' and i+1 < len(sys.argv): Q_theta_y = float(sys.argv[i+1])
    if a == '--rp' and i+1 < len(sys.argv): R_tau_p = float(sys.argv[i+1])
    if a == '--ry' and i+1 < len(sys.argv): R_tau_y = float(sys.argv[i+1])
    if a == '--rsw' and i+1 < len(sys.argv): R_sw = float(sys.argv[i+1])

Q = np.diag([Q_theta_p, 1.0, 5.0, Q_theta_y, 1.0])
R = np.diag([R_tau_p, R_tau_y, R_sw])

A_d, B_d = discretize(A_ct, B_ct, dt)
P, K_raw = solve_dare_iter(A_d, B_d, Q, R)

# 翻转符号: LQR u=-Kx → balance.c tau/sw = +Kx
K = -K_raw

print(f"\n{'='*60}")
print(f"LQR 增益 (Qθ_p={Q_theta_p} Rτ_p={R_tau_p} Qθ_y={Q_theta_y} Rτ_y={R_tau_y} R_sw={R_sw})")
print(f"{'='*60}")

# K shape: (3, 5)
# Row 0: τ_pitch = K[0,0]*θ_p + K[0,1]*θ̇_p + K[0,2]*φ̇ + K[0,3]*θ_y + K[0,4]*θ̇_y
# Row 1: τ_yaw   = K[1,0]*θ_p + K[1,1]*θ̇_p + K[1,2]*φ̇ + K[1,3]*θ_y + K[1,4]*θ̇_y
# Row 2: Δsw     = K[2,0]*θ_p + K[2,1]*θ̇_p + K[2,2]*φ̇ + K[2,3]*θ_y + K[2,4]*θ̇_y

print(f"\n  τ_pitch = {K[0,0]:.1f}·θ_p + {K[0,1]:.1f}·θ̇_p + {K[0,2]:.1f}·φ̇ + {K[0,3]:.1f}·θ_y + {K[0,4]:.1f}·θ̇_y")
print(f"  τ_yaw   = {K[1,0]:.1f}·θ_p + {K[1,1]:.1f}·θ̇_p + {K[1,2]:.1f}·φ̇ + {K[1,3]:.1f}·θ_y + {K[1,4]:.1f}·θ̇_y")
print(f"  Δsw     = {K[2,0]:.1f}·θ_p + {K[2,1]:.1f}·θ̇_p + {K[2,2]:.1f}·φ̇ + {K[2,3]:.1f}·θ_y + {K[2,4]:.1f}·θ̇_y")

# 闭环极点
A_cl = A_d - B_d @ K_raw
poles_d = np.linalg.eigvals(A_cl)
poles_c = np.log(poles_d) / dt
print(f"\n闭环极点 (连续): {np.array2string(poles_c, precision=1, suppress_small=True)}")

# ============================================================
# 5. 参数扫描
# ============================================================
print(f"\n{'='*80}")
print(f"Pitch 增益 vs R_sw (Qθ_p={Q_theta_p} Rτ_p={R_tau_p})")
print(f"{'R_sw':>8s} {'Kp':>8s} {'Kd':>7s} {'Kv':>7s} {'Ksw_p':>8s} {'Ksw_d':>8s} | {'连续极点':>50s}")
print("-" * 95)
for rsw in [0.1, 1.0, 10.0, 50.0, 100.0, 500.0, 1000.0]:
    Rt = np.diag([R_tau_p, R_tau_y, rsw])
    _, Kt_raw = solve_dare_iter(A_d, B_d, Q, Rt)
    Kt = -Kt_raw
    A_cl_t = A_d - B_d @ Kt_raw
    p_c = np.log(np.linalg.eigvals(A_cl_t)) / dt
    print(f"{rsw:8.1f} {Kt[0,0]:8.1f} {Kt[0,1]:7.1f} {Kt[0,2]:7.1f} {Kt[2,0]:8.1f} {Kt[2,1]:8.1f} | {np.array2string(p_c, precision=1, suppress_small=True)}")

# ============================================================
# 6. 输出 C 代码
# ============================================================
print(f"\n{'='*60}")
print(f"填入 balance.c:")
print(f"{'='*60}")
print(f"// M={M:.1f}kg dz0={dz0:.3f}m I_p={I_p0:.4f} I_y={I_y0:.4f}")
print(f"// track={track:.4f} r={r_wheel:.2f} sw_nom={sw_nom}")
print(f"// Qθ_p={Q_theta_p} Rτ_p={R_tau_p} Qθ_y={Q_theta_y} Rτ_y={R_tau_y} R_sw={R_sw}")
print(f"")
print(f"// ---- Pitch (轮扭矩) ----")
print(f"float Kp_pitch = {K[0,0]:.1f}f;   // θ_pitch → τ_pitch")
print(f"float Kd_pitch = {K[0,1]:.1f}f;   // θ̇_pitch → τ_pitch")
print(f"// ---- Yaw (轮差动) ----")
print(f"float Kp_yaw   = {K[1,3]:.1f}f;   // θ_yaw → τ_yaw")
print(f"float Kd_yaw   = {K[1,4]:.1f}f;   // θ̇_yaw → τ_yaw")
print(f"// ---- Leg swing (COM移动) ----")
print(f"float Ksw_p    = {K[2,0]:.1f}f;   // θ_pitch → Δsw")
print(f"float Ksw_d    = {K[2,1]:.1f}f;   // θ̇_pitch → Δsw")
print(f"float sw_nom   = {sw_nom:.2f}f;    // 名义腿位置")

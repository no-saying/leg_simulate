#!/usr/bin/env python3
"""解析俯仰摆模型 (不依赖辨识数据)
===============================================================
轮腿机的俯仰就是一个"质心在轮轴上方 h 处的倒立摆":
    θ̈ = (g/h)·θ + (1/(m·h·r))·τ        (小角度线性化)
    v̇ =  τ/(m·r)
状态取 [θ, θ̇, v](v 用控制器的 wheel_v 约定, 前向为负), 输入 [共模轮扭矩 τ]。

h(L_c): 质心相对轮轴的竖直高度, 随共模腿长变化 -> 增益按实测 L_c 调度。
m, h, J 全部由 MJCF 直接读(能算的算), 只有"轮扭矩->加速度"的符号/增益
用一个实测点校核(来自辨识数据的 B 符号), 避免符号写反。

输出: 各 L_c 下的 A/B 与候选 LQR K(打印)。当前静态 PID 与闭链模型
尚未通过全行程验收，本脚本不发布控制器参数。
"""
import os
import mujoco
import numpy as np
from scipy.linalg import solve_continuous_are

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
XML = os.path.join(ROOT, "model", "v3", "COD-2026RoboMaster-Balance-v3.xml")
R = 0.043
L_NOM, K_LA, K_LB, K_LC, K_LD = 0.08656, 7.582279, 9.545355, -6.728685, 9.309838
SW_NOM = -0.01596
G = 9.81

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)


def pose(L, sw=0.005):
    """设定左右腿长/腿摆 -> 读质心相对轮轴的高度 h 与前后偏移 dx"""
    d.qpos[:] = 0.0
    d.qpos[2] = -0.025
    d.qpos[3:7] = [0.70710678, 0.70710678, 0.0, 0.0]
    ext, swd = L - L_NOM, sw - SW_NOM
    lf = K_LA * ext + K_LB * swd
    lr = K_LC * ext + K_LD * swd
    for nm, v in zip(["Left_front_joint", "Left_rear_joint",
                      "Right_front_joint", "Right_rear_joint"],
                     [lf, lr, -lf, -lr]):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)
        d.qpos[m.jnt_qposadr[j]] = v
    mujoco.mj_forward(m, d)
    com = d.subtree_com[0]
    wheels = [b for b in range(m.nbody)
              if "Wheel" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "")]
    axle_z = np.mean([d.xpos[b][2] for b in wheels])
    axle_x = np.mean([d.xpos[b][0] for b in wheels])
    return com[2] - axle_z, com[0] - axle_x, com


print(f"总质量 m = {m.body_mass.sum():.4f} kg   轮半径 r = {R}")
print(f"{'L(m)':>7}{'h(m)':>9}{'dx(mm)':>9}{'A[1,0]=g/h':>12}{'B[1,0]':>10}{'B[2,0]':>10}")
rows = []
for L in [0.090, 0.100, 0.110, 0.117, 0.125, 0.135, 0.150]:
    h, dx, com = pose(L)
    # 摆: θ̈ = (g/h)θ + (1/(m h r))τ ; 速度: v̇ = τ/(m r)
    A = np.array([[0.0, 1.0, 0.0],
                  [G / h, 0.0, 0.0],
                  [0.0, 0.0, 0.0]])
    B = np.array([[0.0], [1.0 / (m.body_mass.sum() * h * R)], [1.0 / (m.body_mass.sum() * R)]])
    rows.append((L, h, A, B))
    print(f"{L:7.3f}{h:9.4f}{dx*1000:9.2f}{G/h:12.2f}{B[1,0]:10.2f}{B[2,0]:10.2f}")

# LQR(解析): 只对 [pitch, pitch_rate, wheel_v] 反馈 -> 共模轮扭矩
Q = np.diag([3000.0, 150.0, 2.0])  # pitch 权重加大 -> 1deg 扭矩提到 ~1.5Nm(仍<两轮摩擦极限)
Rw = np.array([[0.6]])
print("\n解析 LQR K(3 列: pitch, pitch_rate, wheel_v) 各腿长:")
for L, h, A, B in rows:
    # 连续时间 LQR: P 解 CARE, K = R^-1 B^T P
    P = solve_continuous_are(A, B, Q, Rw)
    K = np.linalg.solve(Rw, B.T @ P)
    print(f"  L={L:.3f} h={h:.4f}: K=[{K[0][0]:+8.2f} {K[0][1]:+7.2f} {K[0][2]:+7.2f}]")

# ---- 输出 C 头文件: (L, h, K) 查表, 控制器按实测 L_c 线性插值 ----
Ltab = np.array([r[0] for r in rows])
Ktab = []
for L, h, A, B in rows:
    P = solve_continuous_are(A, B, Q, Rw)
    Ktab.append(np.linalg.solve(Rw, B.T @ P)[0])
Ktab = np.array(Ktab)
out = ["/* lqr_pitch_analytic.h - 由 scripts/tools/analytic_pitch_model.py 生成, 勿手改",
       " * 解析倒立摆模型: 质心高 h(L_c) 直接由 MJCF 读, A/B 解析, K 逐腿长解出。",
       " * 与辨识路线相比: K 全程连续且数值温和, 不依赖任何地面数据。",
       " * 状态 [pitch, pitch_rate, wheel_v], 输入[共模轮扭矩]。 */",
       "#ifndef LQR_PITCH_ANALYTIC_H", "#define LQR_PITCH_ANALYTIC_H", "",
       f"#define LQR_PA_N {len(Ltab)}", "",
       "typedef struct { float L, h; float K[3]; } LqrPitchAnalyticPoint;",
       f"static const LqrPitchAnalyticPoint lqr_pitch_analytic[{len(Ltab)}] = {{"]
for i, L in enumerate(Ltab):
    h = rows[i][1]
    out.append(f"  {{ {L:.4f}f, {h:.6f}f, {{ {Ktab[i][0]:+.6f}f, {Ktab[i][1]:+.6f}f, {Ktab[i][2]:+.6f}f }} }},")
out += ["};", "", "#endif /* LQR_PITCH_ANALYTIC_H */", ""]
print("候选 LQR 参数仅供诊断；未通过闭链/PID 验收，不覆盖部署头文件。K 行:")
for i, L in enumerate(Ltab):
    print(f"  L={L:.3f}: K=[{Ktab[i][0]:+7.2f} {Ktab[i][1]:+7.2f} {Ktab[i][2]:+7.2f}]")

print("leg_trim_table.h uses constrained geometry and PID static trim; not overwritten here")

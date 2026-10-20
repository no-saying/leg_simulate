#!/usr/bin/env python3
"""空中腿动力学辨识
====================
车体固定在半空中(freejoint 锁定), 对 4 个腿驱动关节注入正弦力矩,
测量关节角/速度/加速度和实际执行器力矩, 拟合每关节等效惯量/阻尼:
    τ = M*q̈ + B*q̇ + K*q   (空中线性模型, 无重力项因车体锁定?)
同时用运动学 J 映射到虚拟坐标(腿长 L 径向, 腿摆 sw 转动), 得到:
    腿长方向: F_L = M_L*L̈ + B_L*L̇ + K_L*L
    腿摆方向: F_sw = M_sw*sẅ + B_sw*sẇ + K_sw*sw
输出用于设计腿长/腿摆三环 PID。

用法: python3 scripts/tools/ident_leg_air.py
"""
import mujoco, numpy as np, math, os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
XML = os.path.join(ROOT, 'model', 'v3', 'COD-2026RoboMaster-Balance-v3.xml')

J_L0, J_L1 = 0.069055, -0.070802   # dL/d(claf0, thigh)
J_S0, J_S1 = 0.049910, 0.056241    # dsw/d(claf0, thigh)
L_NOM, SW_NOM = 0.08656, -0.01596

def main():
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    def jadr(name):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        return m.jnt_qposadr[j], m.jnt_dofadr[j]
    names = ['Left_front_joint','Left_rear_joint','Right_front_joint','Right_rear_joint']
    lj = {n: jadr(n) for n in names}
    lw = jadr('Left_Wheel_joint'); rw = jadr('Right_Wheel_joint')
    dt = m.opt.timestep

    # 空中锁定: freejoint 固定
    def lock():
        d.qpos[0:7] = [0.0, 0.0, 0.0, 0.70710678, 0.70710678, 0.0, 0.0]
        d.qvel[0:6] = 0.0

    # 扫频激励: 每 2s 一个频率, 0.5/1/2/4 Hz, 幅度 1.0 ctrl (=50Nm 腿关节, 力限60Nm内)
    freqs = [0.5, 1.0, 2.0, 4.0]
    amp = 0.8
    dur_per = 2.0
    N = int(sum(dur_per for _ in freqs) / dt)
    data = {n: {'q': [], 'qd': [], 'tau': []} for n in names}
    t_global = 0.0
    for fi, f in enumerate(freqs):
        n_steps = int(dur_per/dt)
        for i in range(n_steps):
            t = i*dt
            lock()
            for a, n in zip([0,1,3,4], names):
                # 相位错开, 左右一致, claf0/thigh 同频
                phase = 0.0 if a in (0,3) else math.pi/2
                ctrl = amp*math.sin(2*math.pi*f*t + phase)
                d.ctrl[a] = np.clip(ctrl, -1.0, 1.0)
            d.ctrl[2]=d.ctrl[5]=0.0
            mujoco.mj_step(m, d)
            for a, n in zip([0,1,3,4], names):
                q0 = lj[n]
                data[n]['q'].append(d.qpos[q0[0]])
                data[n]['qd'].append(d.qvel[q0[1]])
                data[n]['tau'].append(d.actuator_force[a])
            t_global += dt
    print("激励完成, 样本数:", len(data['Left_front_joint']['q']))

    # 拟合每关节: tau = M*qdd + B*qd + K*q
    print("\n=== 关节级辨识 (tau = M*qdd + B*qd + K*q) ===")
    for n in names:
        q = np.array(data[n]['q']); qd = np.array(data[n]['qd']); tau = np.array(data[n]['tau'])
        qdd = np.gradient(qd, dt)
        A = np.stack([qdd, qd, q, np.ones_like(q)], axis=1)
        sol, res, *_ = np.linalg.lstsq(A, tau, rcond=None)
        M, B, K, C = sol
        print(f"  {n:20s} M={M:8.4f} kg·m²  B={B:8.4f} Nm·s/rad  K={K:8.4f} Nm/rad  C={C:8.4f}")

    # 映射虚拟坐标: L, sw 及其一阶/二阶, 虚拟力 F = J^{-T} tau
    J = np.array([[J_L0, J_L1],[J_S0, J_S1]])
    JinvT = np.linalg.inv(J).T
    print("\n=== 虚拟坐标辨识 (腿长 L 径向 / 腿摆 sw 转动) ===")
    # 用左腿(claf0=0, thigh=1)
    q0 = np.array(data['Left_front_joint']['q'])
    q1 = np.array(data['Left_rear_joint']['q'])
    qd0 = np.array(data['Left_front_joint']['qd'])
    qd1 = np.array(data['Left_rear_joint']['qd'])
    tau0 = np.array(data['Left_front_joint']['tau'])
    tau1 = np.array(data['Left_rear_joint']['tau'])
    L = L_NOM + J_L0*q0 + J_L1*q1
    sw = SW_NOM + J_S0*q0 + J_S1*q1
    Ld = J_L0*qd0 + J_L1*qd1
    swd = J_S0*qd0 + J_S1*qd1
    Ldd = np.gradient(Ld, dt); swdd = np.gradient(swd, dt)
    # 虚拟力: [F_L, F_sw] = JinvT @ [tau0, tau1]
    F = np.stack([tau0, tau1], axis=1) @ JinvT.T   # 每行 = (F_L, F_sw)
    FL, Fsw = F[:,0], F[:,1]
    # 拟合 FL = ML*Ldd + BL*Ld + KL*L + CL
    A_L = np.stack([Ldd, Ld, L, np.ones_like(L)], axis=1)
    solL, *_ = np.linalg.lstsq(A_L, FL, rcond=None)
    A_S = np.stack([swdd, swd, sw, np.ones_like(sw)], axis=1)
    solS, *_ = np.linalg.lstsq(A_S, Fsw, rcond=None)
    print(f"  腿长L: F_L = {solL[0]:.3f}*Ldd + {solL[1]:.3f}*Ld + {solL[2]:.3f}*L + {solL[3]:.3f}")
    print(f"        (等效质量 ML={solL[0]:.3f}kg, 阻尼 BL={solL[1]:.3f}N·s/m, 刚度 KL={solL[2]:.3f}N/m)")
    print(f"  腿摆sw: F_sw = {solS[0]:.3f}*swdd + {solS[1]:.3f}*swd + {solS[2]:.3f}*sw + {solS[3]:.3f}")
    print(f"        (等效惯量 Msw={solS[0]:.3f}kg·m? , 阻尼 Bsw={solS[1]:.3f}, 刚度 Ksw={solS[2]:.3f})")

    # 建议 PID (临界阻尼二阶系统)
    print("\n=== 建议腿长/腿摆三环 PID (按辨识参数, 临界阻尼 ωn 选 8 rad/s) ===")
    wn = 8.0
    ML, BL = solL[0], solL[1]
    Msw, Bsw = solS[0], solS[1]
    kpL = max(ML, 0.05)*wn*wn
    kdL = 2*max(ML, 0.05)*wn
    print(f"  腿长环: Kp={kpL:.2f} N/m  Kd={kdL:.2f} N·s/m (ML={ML:.3f}kg, BL={BL:.2f}N·s/m)")
    print(f"  腿摆环: Kp={Msw*wn*wn:.2f}  Kd={2*Msw*wn:.2f}")

if __name__ == '__main__':
    main()

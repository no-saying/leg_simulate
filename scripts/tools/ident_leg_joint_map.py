#!/usr/bin/env python3
"""4 腿关节电机 <-> 腿长 L / 腿摆 sw 完整映射
============================================
关系表:
  A. 每侧雅可比 J (精确闭链标定, 与 leg_kin_v3.h 一致)
  B. 4 关节电机速度 -> 左右 L/sw 速度 (4x4 速度雅可比)
  C. 4 关节电机力矩 -> 左右 L/sw 虚拟力 (J^T)
  D. 空中辨识: 每个关节电机 tau = M*qdd + B*qd + K*q + C
  E. 关节动力学经 J 转换 -> 腿长/腿摆方向等效
"""
import mujoco, numpy as np, math, os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
XML = os.path.join(ROOT, 'model', 'v3', 'COD-2026RoboMaster-Balance-v3.xml')

def main():
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    def jadr(name):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        return m.jnt_qposadr[j], m.jnt_dofadr[j]
    JN = ['Left_front_joint','Left_rear_joint','Right_front_joint','Right_rear_joint']
    lj = {n: jadr(n) for n in JN}

    print("=== A. 每侧雅可比 J = d(L,sw)/d(claf0,thigh) (leg_kin_v3 标定) ===")
    J_L0, J_L1 = 0.069055, -0.070802   # dL/d(claf0), dL/d(thigh)
    J_S0, J_S1 = 0.049910, 0.056241    # dsw/d(claf0), dsw/d(thigh)
    print("  左腿: dL  = +%.4f*dq_claf0 + %+.4f*dq_thigh" % (J_L0, J_L1))
    print("        dsw = +%.4f*dq_claf0 + %+.4f*dq_thigh" % (J_S0, J_S1))
    print("  右腿: dL  = -%.4f*dq_claf0 - %+.4f*dq_thigh (镜像)" % (J_L0, J_L1))
    print("        dsw = -%.4f*dq_claf0 - %+.4f*dq_thigh" % (J_S0, J_S1))
    Jl = np.array([[J_L0,J_L1],[J_S0,J_S1]])
    print("  J^-1 =", np.round(np.linalg.inv(Jl), 4).tolist())

    print("\n=== B. 4 关节电机速度 -> 左右腿长/腿摆速度 ===")
    print("  [ L_L̇ ]   [ %+.4f %+.4f   0       0    ] [ q̇_lf(claf0L) ]" % (J_L0,J_L1))
    print("  [ sw_L̇] = [ %+.4f %+.4f   0       0    ] [ q̇_lr(thighL) ]" % (J_S0,J_S1))
    print("  [ L_Ṙ ]   [   0      0   %+.4f %+.4f ] [ q̇_rf(claf0R) ]" % (-J_L0,-J_L1))
    print("  [ sw_Ṙ]   [   0      0   %+.4f %+.4f ] [ q̇_rr(thighR) ]" % (-J_S0,-J_S1))

    print("\n=== C. 4 关节电机力矩 -> 腿长/腿摆虚拟力 (J^T) ===")
    print("  左腿: τ_lf(claf0) = %+.4f*F_L + %+.4f*F_sw" % (J_L0, J_S0))
    print("        τ_lr(thigh) = %+.4f*F_L + %+.4f*F_sw" % (J_L1, J_S1))
    print("  右腿: τ_rf(claf0) = -%+.4f*F_L - %+.4f*F_sw" % (J_L0, J_S0))
    print("        τ_rr(thigh) = -%+.4f*F_L - %+.4f*F_sw" % (J_L1, J_S1))
    JinvT = np.linalg.inv(Jl).T
    print("  反解(力分配): F_L = %+.4f*τ_lf + %+.4f*τ_lr" % (JinvT[0,0], JinvT[1,0]))
    print("                 F_sw = %+.4f*τ_lf + %+.4f*τ_lr" % (JinvT[0,1], JinvT[1,1]))

    print("\n=== D. 空中辨识: 每个关节电机 tau = M*qdd + B*qd + K*q + C ===")
    dt = m.opt.timestep
    def lock():
        d.qpos[0:7] = [0,0,0, 0.70710678,0.70710678,0,0]
        d.qvel[0:6] = 0.0
    freqs = [0.5, 1.0, 2.0, 3.0]
    data = {n: {'q':[], 'qd':[], 'tau':[]} for n in JN}
    for f in freqs:
        for i in range(int(2.0/dt)):
            t = i*dt
            lock()
            for a, n in zip([0,1,3,4], JN):
                phase = 0.0 if a in (0,3) else math.pi/3
                d.ctrl[a] = np.clip(0.8*math.sin(2*math.pi*f*t + phase), -1.2, 1.2)
            d.ctrl[2]=d.ctrl[5]=0.0
            mujoco.mj_step(m, d)
            for a, n in zip([0,1,3,4], JN):
                q0 = lj[n]
                data[n]['q'].append(d.qpos[q0[0]])
                data[n]['qd'].append(d.qvel[q0[1]])
                data[n]['tau'].append(d.actuator_force[a])
    print("  关节          M(kg·m²)   B(Nm·s/rad)  K(Nm/rad)   C(Nm)   | 驱动:")
    params = {}
    for a, n in zip([0,1,3,4], JN):
        q = np.array(data[n]['q']); qd = np.array(data[n]['qd']); tau = np.array(data[n]['tau'])
        qdd = np.gradient(qd, dt)
        A = np.stack([qdd, qd, q, np.ones_like(q)], axis=1)
        sol, *_ = np.linalg.lstsq(A, tau, rcond=None)
        params[n] = sol
        role = 'claf0(腿长+摆)' if a in (0,3) else 'thigh(腿长+摆)'
        print(f"  {n:20s} {sol[0]:8.4f}  {sol[1]:8.4f}  {sol[2]:8.4f}  {sol[3]:8.4f}  {role}")

    print("\n=== E. 关节动力学 -> 腿长/腿摆等效(经 J 精确转换) ===")
    Ml = np.linalg.inv(Jl).T @ np.array([params['Left_front_joint'][0], params['Left_rear_joint'][0]])
    Bl = np.linalg.inv(Jl).T @ np.array([params['Left_front_joint'][1], params['Left_rear_joint'][1]])
    Kl = np.linalg.inv(Jl).T @ np.array([params['Left_front_joint'][2], params['Left_rear_joint'][2]])
    print(f"  腿长 L 方向: F_L ≈ {Ml[0]:.4f}*Ldd + {Bl[0]:.2f}*Ld + {Kl[0]:.1f}*L")
    print(f"  腿摆 sw方向: F_sw ≈ {Ml[1]:.4f}*swdd + {Bl[1]:.2f}*swd + {Kl[1]:.1f}*sw")

if __name__ == '__main__':
    main()

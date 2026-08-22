#!/usr/bin/env python3
"""v3 空中腿伺服自动整定
========================
车体锁定在空中(freejoint 固定), 对 4 个腿关节注入阶跃目标,
扫描直接位置伺服 ctrl = KP*(ref-pos) - KD*vel 的 (KP, KD),
按 稳态误差/超调/调节时间/振荡 评分, 输出推荐增益并写入 config_lowlevel.h。

用法: python3 scripts/tools/tune_leg_servo_air.py [--write]
"""
import argparse
import math
import numpy as np
import mujoco
import re
import sys
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
XML = os.path.join(ROOT, 'model', 'v3', 'COD-2026RoboMaster-Balance-v3.xml')
CFG = os.path.join(ROOT, 'control', 'include', 'config_lowlevel.h')

STEP = 0.30        # 阶跃目标 rad
SETTLE = 1.2       # 观察时长 s
KP_CANDS = [20.0, 40.0, 80.0, 160.0]
KD_CANDS = [0.5, 1.0, 2.0, 4.0]


def jadr(m, name):
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    return m.jnt_qposadr[j], m.jnt_dofadr[j]


def run_case(KP, KD, joint='Left_front_joint', seed_joint='Right_front_joint'):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    jq, jd = jadr(m, joint)
    sq, sd = jadr(m, seed_joint)
    dt = m.opt.timestep
    N = int(SETTLE / dt)
    q0 = 0.0
    prev_q, prev_t = q0, 0.0
    peak = 0.0
    t_peak = -1.0
    settle_t = -1.0
    final_q = 0.0
    undershoot = 0.0
    osc = 0.0
    for i in range(N):
        # 锁基座(车体固定空中)
        d.qpos[0:7] = [0.0, 0.0, -0.022, 1.0, 0.0, 0.0, 0.0]
        d.qvel[0:6] = 0.0
        q = d.qpos[jq]
        v = d.qvel[jd]
        ref = STEP
        ctrl = KP * (ref - q) - KD * v
        ctrl = max(-10.0, min(10.0, ctrl))
        d.ctrl[:] = 0.0
        d.ctrl[0] = ctrl   # Left_front 执行器
        mujoco.mj_step(m, d)
        t = (i + 1) * dt
        if abs(q) > abs(peak):
            peak = q
            t_peak = t
        if q < -1e-3:
            undershoot = max(undershoot, -q)
        # 振荡检测: 过零次数
        if prev_q > 0.0 and q <= 0.0:
            osc += 1.0
        prev_q = q
        # 调节时间: |误差|<5% 且保持
        if abs(q - STEP) < 0.05 * STEP:
            if settle_t < 0.0:
                settle_t = t
        final_q = q
    err_ss = abs(final_q - STEP)
    overshoot = max(0.0, (peak - STEP) / STEP)
    score = (err_ss * 50.0            # 稳态误差 -> 重罚
             + overshoot * 20.0       # 超调
             + (0.0 if settle_t < 0 else settle_t) * 3.0  # 调节时间
             + osc * 2.0
             + undershoot * 10.0)
    return dict(KP=KP, KD=KD, err_ss=err_ss, overshoot=overshoot,
                settle_t=settle_t, osc=osc, undershoot=undershoot,
                final_q=final_q, score=score)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--write', action='store_true', help='写入 config_lowlevel.h')
    ap.add_argument('--joint', default='Left_front_joint')
    args = ap.parse_args()

    results = []
    for KP in KP_CANDS:
        for KD in KD_CANDS:
            r = run_case(KP, KD, args.joint)
            results.append(r)
            print(f"KP={KP:6.1f} KD={KD:4.1f} | err={r['err_ss']*100:5.1f}cm "
                  f"ovs={r['overshoot']*100:4.0f}% settle={r['settle_t']:.2f}s "
                  f"osc={r['osc']:.0f} score={r['score']:7.2f}")
    results.sort(key=lambda r: r['score'])
    best = results[0]
    print(f"\n最优: KP={best['KP']:.0f} KD={best['KD']:.1f} score={best['score']:.2f} "
          f"err={best['err_ss']*100:.1f}cm ovs={best['overshoot']*100:.0f}%")

    if args.write:
        s = open(CFG).read()
        s = re.sub(r'#define LEG_V3_SERVO_KP [0-9.]+f',
                   f'#define LEG_V3_SERVO_KP {best["KP"]:.0f}f', s)
        s = re.sub(r'#define LEG_V3_SERVO_KD [0-9.]+f',
                   f'#define LEG_V3_SERVO_KD {best["KD"]:.1f}f', s)
        open(CFG, 'w').write(s)
        print(f"已写入 {CFG}: LEG_V3_SERVO_KP={best['KP']:.0f} LEG_V3_SERVO_KD={best['KD']:.1f}")


if __name__ == '__main__':
    main()

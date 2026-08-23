/*
 * leg_kin_v3.h - v3 串联腿闭链模型: 驱动关节 <-> 虚拟坐标 线性映射
 * ================================================================
 * v3 每条腿只有 2 个驱动关节:
 *   lf = Left_front_joint (claf_link_0, 大腿根部闭链驱动杆)
 *   lr = Left_rear_joint  (Left_Thigh)
 *   rf = Right_front_joint
 *   rr = Right_rear_joint
 * 虚拟坐标(单位 m, 世界系):
 *   sw = 轮轴相对基座的前向 x 偏移(前后摆)
 *   L  = 轮轴相对基座的竖直下探距离(-z, 腿长)
 * 名义位(全部驱动关节=0, CAD 装配位)标定值:
 *   SW_NOM = -0.01596 m, L_NOM = 0.08656 m
 * 线性化雅可比(名义位, 由 MuJoCo 闭链牛顿求解标定;
 * 与 scripts/tools/ident_leg_joint_map.py 空中辨识输出一致, 2026-08-23 复核):
 *   左腿: [dL/dlf, dL/dlr] = [+0.069055, -0.070802]
 *         [dsw/dlf, dsw/dlr] = [+0.049910, +0.056241]
 *   右腿: 与左腿互为镜像(J_right = -J_left)。
 * 反解:
 *   左腿: lf = 7.5823*(L-L_NOM) + 9.5454*(sw-SW_NOM)
 *         lr = -6.7287*(L-L_NOM) + 9.3098*(sw-SW_NOM)
 *   右腿: rf = -7.5833*(L-L_NOM) - 9.5452*(sw-SW_NOM)
 *         rr = 6.7278*(L-L_NOM) - 9.3099*(sw-SW_NOM)
 */
#ifndef LEG_KIN_V3_H
#define LEG_KIN_V3_H

#ifdef __cplusplus
extern "C" {
#endif

/* 名义虚拟坐标(单位 m) */
#define LEG_V3_SW_NOM   (-0.01596f)
#define LEG_V3_L_NOM    ( 0.08656f)

/* 左腿雅可比: d(L,sw)/d(lf,lr) */
#define LEG_V3_J_L0  0.069055f
#define LEG_V3_J_L1 -0.070802f
#define LEG_V3_J_S0  0.049910f
#define LEG_V3_J_S1  0.056241f

/* 左腿反解 */
#define LEG_V3_K_LA  7.582279f
#define LEG_V3_K_LB  9.545355f
#define LEG_V3_K_LC -6.728685f
#define LEG_V3_K_LD  9.309838f

/* 正解: 4 驱动关节角 -> (sw, leg_l, leg_r) */
void leg_kin_v3_forward(float lf, float lr, float rf, float rr,
                        float *sw, float *leg_l, float *leg_r);

/* 反解: (sw 偏差, 左腿长偏差, 右腿长偏差) -> 4 关节目标 */
void leg_kin_v3_inverse(float sw_dev, float ext_l, float ext_r,
                        float *lf, float *lr, float *rf, float *rr);

/* 关节速度 -> 虚拟速度 */
void leg_kin_v3_velocities(float lfv, float lrv, float rfv, float rrv,
                           float *sw_dot, float *l_dot, float *r_dot);

#ifdef __cplusplus
}
#endif

#endif /* LEG_KIN_V3_H */

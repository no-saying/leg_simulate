/*
 * leg_kin_v3.c - v3 腿运动学线性映射实现
 * 见 leg_kin_v3.h 说明。
 */
#include "leg_kin_v3.h"

void leg_kin_v3_forward(float lf, float lr, float rf, float rr,
                        float *sw, float *leg_l, float *leg_r)
{
    /* 左腿: 名义 + J·关节偏差 */
    float sw_l = LEG_V3_SW_NOM + LEG_V3_J_S0 * lf + LEG_V3_J_S1 * lr;
    float L_l = LEG_V3_L_NOM + LEG_V3_J_L0 * lf + LEG_V3_J_L1 * lr;
    /* 右腿: 镜像(雅可比取负) */
    float sw_r = LEG_V3_SW_NOM - LEG_V3_J_S0 * rf - LEG_V3_J_S1 * rr;
    float L_r = LEG_V3_L_NOM - LEG_V3_J_L0 * rf - LEG_V3_J_L1 * rr;
    *sw = 0.5f * (sw_l + sw_r);
    *leg_l = L_l;
    *leg_r = L_r;
}

void leg_kin_v3_inverse(float sw_dev, float ext_l, float ext_r,
                        float *lf, float *lr, float *rf, float *rr)
{
    /* sw_dev = sw - SW_NOM; ext = L - L_NOM */
    *lf = LEG_V3_K_LA * ext_l + LEG_V3_K_LB * sw_dev;
    *lr = LEG_V3_K_LC * ext_l + LEG_V3_K_LD * sw_dev;
    *rf = -LEG_V3_K_LA * ext_r - LEG_V3_K_LB * sw_dev;
    *rr = -LEG_V3_K_LC * ext_r - LEG_V3_K_LD * sw_dev;
}

void leg_kin_v3_velocities(float lfv, float lrv, float rfv, float rrv,
                           float *sw_dot, float *l_dot, float *r_dot)
{
    float sw_l_dot = LEG_V3_J_S0 * lfv + LEG_V3_J_S1 * lrv;
    float sw_r_dot = -LEG_V3_J_S0 * rfv - LEG_V3_J_S1 * rrv;
    float l_l_dot = LEG_V3_J_L0 * lfv + LEG_V3_J_L1 * lrv;
    float l_r_dot = -LEG_V3_J_L0 * rfv - LEG_V3_J_L1 * rrv;
    *sw_dot = 0.5f * (sw_l_dot + sw_r_dot);
    *l_dot = 0.5f * (l_l_dot + l_r_dot);
    *r_dot = l_r_dot;
}

void leg_kin_v3_forces(float F_l, float F_sw_l, float F_r, float F_sw_r,
                       float *tau_lf, float *tau_lr,
                       float *tau_rf, float *tau_rr)
{
    /* 左腿: J^T [F_L, F_sw] */
    *tau_lf = LEG_V3_J_L0 * F_l + LEG_V3_J_S0 * F_sw_l;
    *tau_lr = LEG_V3_J_L1 * F_l + LEG_V3_J_S1 * F_sw_l;
    /* 右腿镜像: 取负 */
    *tau_rf = -LEG_V3_J_L0 * F_r - LEG_V3_J_S0 * F_sw_r;
    *tau_rr = -LEG_V3_J_L1 * F_r - LEG_V3_J_S1 * F_sw_r;
}

void leg_kin_v3_force_split(float tau_lf, float tau_lr,
                            float tau_rf, float tau_rr,
                            float *F_l, float *F_sw_l,
                            float *F_r, float *F_sw_r)
{
    /* 左腿: [F_L, F_sw] = J^-T [τ_lf, τ_lr]
     * J^-1 = [[K_LA, K_LB],[K_LC, K_LD]] = [[7.5823, 9.5453],[-6.7287, 9.3098]]
     * J^-T = [[K_LA, K_LC],[K_LB, K_LD]] = [[7.5823, -6.7287],[9.5453, 9.3098]]
     * F_L = 7.5823*τ_lf - 6.7287*τ_lr ; F_sw = 9.5453*τ_lf + 9.3098*τ_lr */
    *F_l    =  LEG_V3_K_LA * tau_lf + LEG_V3_K_LC * tau_lr;
    *F_sw_l =  LEG_V3_K_LB * tau_lf + LEG_V3_K_LD * tau_lr;
    /* 右腿: 关节力矩已镜像(τ_rf=-..., τ_rr=-...), 力也镜像 */
    *F_r    = -LEG_V3_K_LA * tau_rf - LEG_V3_K_LC * tau_rr;
    *F_sw_r = -LEG_V3_K_LB * tau_rf - LEG_V3_K_LD * tau_rr;
}

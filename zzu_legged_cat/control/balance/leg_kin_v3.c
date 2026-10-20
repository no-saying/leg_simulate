/*
 * leg_kin_v3.c - v3 腿运动学线性映射实现
 * 见 leg_kin_v3.h 说明。
 */
#include "leg_kin_v3.h"

/* Fit from constrained MuJoCo axle positions in base_link coordinates.
 * Terms: 1,a,b,a^2,ab,b^2,a^3,a^2b,ab^2,b^3; a/b are mirrored on the right.
 * Original fit holds <2 mm on L=0.056..0.195 m samples.  The deeper folded
 * poses near 0.048 m are outside that validation range. */
static const float cal_l[10] = {
    0.086715294f, 0.068312374f, -0.071332646f, -0.002672041f,
    -0.042785317f, -0.007430294f, -0.014152201f, 0.013224687f,
    -0.008973839f, 0.018311806f
};
static const float cal_sw[10] = {
    -0.016222440f, 0.051321734f, 0.057643961f, 0.036104740f,
    -0.002465143f, -0.032528860f, -0.007137062f, -0.004273003f,
    -0.019570855f, -0.005660699f
};

static void poly_eval(const float c[10], float a, float b,
                      float *value, float *da, float *db)
{
    float aa = a*a, bb = b*b;
    *value = c[0] + c[1]*a + c[2]*b + c[3]*aa + c[4]*a*b + c[5]*bb
           + c[6]*aa*a + c[7]*aa*b + c[8]*a*bb + c[9]*bb*b;
    if (da) *da = c[1] + 2.0f*c[3]*a + c[4]*b + 3.0f*c[6]*aa
                + 2.0f*c[7]*a*b + c[8]*bb;
    if (db) *db = c[2] + c[4]*a + 2.0f*c[5]*b + c[7]*aa
                + 2.0f*c[8]*a*b + 3.0f*c[9]*bb;
}

void leg_kin_v3_calibrated_side(float front, float rear, int right,
                                float *length, float *swing, float J[2][2])
{
    float sign = right ? -1.0f : 1.0f;
    float a = sign*front, b = sign*rear;
    float la, lb, sa, sb;
    poly_eval(cal_l, a, b, length, J ? &la : 0, J ? &lb : 0);
    poly_eval(cal_sw, a, b, swing, J ? &sa : 0, J ? &sb : 0);
    if (J) {
        J[0][0] = sign*la; J[0][1] = sign*lb;
        J[1][0] = sign*sa; J[1][1] = sign*sb;
    }
}

void leg_kin_v3_forward_calibrated(float lf, float lr, float rf, float rr,
                                   float *sw, float *leg_l, float *leg_r)
{
    float sw_l, sw_r;
    leg_kin_v3_calibrated_side(lf, lr, 0, leg_l, &sw_l, 0);
    leg_kin_v3_calibrated_side(rf, rr, 1, leg_r, &sw_r, 0);
    *sw = 0.5f*(sw_l + sw_r);
}

void leg_kin_v3_velocities_calibrated(float lf, float lr, float rf, float rr,
                                      float lfv, float lrv, float rfv, float rrv,
                                      float *sw_dot, float *l_dot, float *r_dot)
{
    float L, sw, jl[2][2], jr[2][2];
    leg_kin_v3_calibrated_side(lf, lr, 0, &L, &sw, jl);
    leg_kin_v3_calibrated_side(rf, rr, 1, &L, &sw, jr);
    *l_dot = jl[0][0]*lfv + jl[0][1]*lrv;
    *r_dot = jr[0][0]*rfv + jr[0][1]*rrv;
    *sw_dot = 0.5f*(jl[1][0]*lfv + jl[1][1]*lrv
                    + jr[1][0]*rfv + jr[1][1]*rrv);
}

void leg_kin_v3_forces_calibrated(float lf, float lr, float rf, float rr,
                                  float F_l, float F_sw_l, float F_r, float F_sw_r,
                                  float *tau_lf, float *tau_lr,
                                  float *tau_rf, float *tau_rr)
{
    float L, sw, jl[2][2], jr[2][2];
    leg_kin_v3_calibrated_side(lf, lr, 0, &L, &sw, jl);
    leg_kin_v3_calibrated_side(rf, rr, 1, &L, &sw, jr);
    *tau_lf = jl[0][0]*F_l + jl[1][0]*F_sw_l;
    *tau_lr = jl[0][1]*F_l + jl[1][1]*F_sw_l;
    *tau_rf = jr[0][0]*F_r + jr[1][0]*F_sw_r;
    *tau_rr = jr[0][1]*F_r + jr[1][1]*F_sw_r;
}

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
    /* 输出左右两侧各自的腿长速度。旧实现把左侧写成左右平均、右侧
     * 写成右侧全量，导致两边阻尼不对称，腿摆/腿长耦合时会持续偏向一侧。 */
    *l_dot = l_l_dot;
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

#include "vel_kf.h"

/* 噪声参数(与辨识脚本同名常量保持一致) */
#define VEL_KF_Q_V    0.30f     /* 过程噪声谱密度: 速度 (m/s)^2/s */
#define VEL_KF_Q_BA   1.0e-4f   /* 过程噪声谱密度: 偏置 (m/s^2)^2/s */
#define VEL_KF_R0     0.010f    /* 轮速量测噪声 (m/s)^2 (trust=1 时) */
#define VEL_KF_P0_V   1.0f
#define VEL_KF_P0_BA  1.0e-2f
#define VEL_KF_V_MAX  5.0f

static float s_v, s_ba, s_p00, s_p01, s_p11;
static int   s_init;

void vel_kf_reset(void)
{
    s_v = 0.0f;
    s_ba = 0.0f;
    s_p00 = VEL_KF_P0_V;
    s_p01 = 0.0f;
    s_p11 = VEL_KF_P0_BA;
    s_init = 1;
}

float vel_kf_bias(void) { return s_ba; }
void vel_kf_set_velocity(float velocity) { s_v = velocity; }

float vel_kf_step(float a_imu, float v_wheel, float trust, int airborne, float dt)
{
    if (!s_init) vel_kf_reset();
    if (dt <= 0.0f) dt = 0.001f;

    /* ---- 预测: A=[[1,-dt],[0,1]], Q=diag(q_v·dt, q_ba·dt) ---- */
    float v = s_v + (a_imu - s_ba) * dt;
    float p00 = s_p00 - 2.0f * dt * s_p01 + dt * dt * s_p11 + VEL_KF_Q_V * dt;
    float p01 = s_p01 - dt * s_p11;
    float p11 = s_p11 + VEL_KF_Q_BA * dt;

    /* ---- 量测更新: H=[1,0], R=R0/trust ---- */
    int do_update = (!airborne) && (trust > 0.001f);
    if (do_update) {
        float r = VEL_KF_R0 / trust;
        float s = p00 + r;
        float k0 = p00 / s;
        float k1 = p01 / s;
        float innov = v_wheel - v;
        v   += k0 * innov;
        s_ba += k1 * innov;
        float np00 = (1.0f - k0) * p00;
        float np01 = (1.0f - k0) * p01;
        float np11 = p11 - k1 * p01;
        p00 = np00; p01 = np01; p11 = np11;
    }
    if (p11 < 1e-8f) p11 = 1e-8f;
    if (v > VEL_KF_V_MAX) v = VEL_KF_V_MAX;
    if (v < -VEL_KF_V_MAX) v = -VEL_KF_V_MAX;

    s_v = v; s_p00 = p00; s_p01 = p01; s_p11 = p11;
    return s_v;
}

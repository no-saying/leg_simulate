/*
 * lqr_controller.c - 分段 LQR 控制器实现
 * ================================================================
 * 从 SensorState 提取 pitch/roll 两个子系统的状态, 按实测 leg_len
 * 选择分段模型, 分别计算 u_extra = -K*(x-x_trim) 输出。
 *
 * 注意:
 *   - 输出是"扰动"量, 由 supervisor 叠加到 PID 基线上。
 *   - pitch 与 roll 子系统独立选段/独立 LQR, 互不污染。
 *   - 状态全部来自真实传感器(IMU/编码器/关节角), 不依赖世界坐标。
 */
#include "lqr_controller.h"

#include <math.h>
#include <stdio.h>

#include "control_config.h"
#include "config_controllers.h"
#include "lqr_pitch_models.h"
#include "lqr_roll_models.h"
#include "leg_kin_v3.h"

/* 输出限幅: 防止启动暂态/分段切换时 LQR 扰动过大推翻车体 */

/* pitch 子系统 7 状态 3 输入, roll 子系统 5 状态 2 输入 */
#define LQR_P_STATE LQR_PITCH_N_STATE
#define LQR_P_INPUT LQR_PITCH_N_INPUT
#define LQR_R_STATE LQR_ROLL_N_STATE
#define LQR_R_INPUT LQR_ROLL_N_INPUT

/* pitch/roll 子系统分段模型结构由各自头文件定义 */

static int g_seg = -1;
static int g_ready = 0;
static float g_pitch_out = 0.0f;
static float g_roll_out = 0.0f;

static float clampf_lqr(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void lqr_controller_init(void)
{
    g_seg = -1;
    g_ready = 1;
}

int lqr_controller_segment(void)
{
    return g_seg;
}

float lqr_controller_pitch_out(void) { return g_pitch_out; }
float lqr_controller_roll_out(void)  { return g_roll_out; }

/* pitch 子系统状态(7 维): pitch/pitch_rate/leg_swing/leg_swing_rate/
 * leg_len/leg_len_rate/wheel_v, 严格对齐 split_models.py 的 PITCH_COLS。 */
static void build_pitch_state(const SensorState *s, float x[LQR_P_STATE])
{
    float lf = (float)s->lf_pos, lr = (float)s->lr_pos;
    float rf = (float)s->rf_pos, rr = (float)s->rr_pos;
    float lfv = (float)s->lf_vel, lrv = (float)s->lr_vel;
    float rfv = (float)s->rf_vel, rrv = (float)s->rr_vel;

#if MODEL_V3
    const float R = WHEEL_RADIUS;
    float sw, leg_l, leg_r;
    leg_kin_v3_forward(lf, lr, rf, rr, &sw, &leg_l, &leg_r);
    float sw_dot, l_dot, r_dot;
    leg_kin_v3_velocities(lfv, lrv, rfv, rrv, &sw_dot, &l_dot, &r_dot);
    float left_wheel_v = (float)s->lw_vel * R;
    float right_wheel_v = (float)s->rw_vel * R;

    x[0] = (float)s->pitch;
    x[1] = (float)s->gyro[1];
    x[2] = sw - BALANCE_LEG_SW_NOM;
    x[3] = sw_dot;
    x[4] = 0.5f * (leg_l + leg_r);
    x[5] = 0.5f * (l_dot + r_dot);
    x[6] = 0.5f * (left_wheel_v + right_wheel_v);
#else
    const float SW_NOM = 0.23f;   /* 必须与 balance.h BALANCE_LEG_SW_NOM 一致 */
    const float R = 0.058f;       /* 轮半径 m */
    float left_swing = 0.5f * (lf - lr + 0.4f);
    float right_swing = 0.5f * (rr - rf + 0.3f);
    float left_len = 0.5f * (0.4f - lf - lr);
    float right_len = 0.5f * (rf + rr + 0.5f);
    float left_swing_dot = 0.5f * (lfv - lrv);
    float right_swing_dot = 0.5f * (rrv - rfv);
    float left_ext_dot = -0.5f * (lfv + lrv);
    float right_ext_dot = 0.5f * (rfv + rrv);
    float left_wheel_v = (float)s->lw_vel * R;
    float right_wheel_v = -(float)s->rw_vel * R;

    x[0] = (float)s->pitch;
    x[1] = (float)s->gyro[1];
    x[2] = 0.5f * (left_swing + right_swing) - SW_NOM;
    x[3] = 0.5f * (left_swing_dot + right_swing_dot);
    x[4] = 0.5f * (left_len + right_len);
    x[5] = 0.5f * (left_ext_dot + right_ext_dot);
    x[6] = 0.5f * (left_wheel_v + right_wheel_v);
#endif
}

/* roll 子系统状态(5 维): roll/roll_rate/leg_len_diff/leg_len_diff_rate/
 * wheel_diff_v, 严格对齐 split_models.py 的 ROLL_COLS。 */
static void build_roll_state(const SensorState *s, float x[LQR_R_STATE])
{
#if MODEL_V3
    const float R = WHEEL_RADIUS;
    float lf = (float)s->lf_pos, lr = (float)s->lr_pos;
    float rf = (float)s->rf_pos, rr = (float)s->rr_pos;
    float lfv = (float)s->lf_vel, lrv = (float)s->lr_vel;
    float rfv = (float)s->rf_vel, rrv = (float)s->rr_vel;
    float sw, leg_l, leg_r;
    leg_kin_v3_forward(lf, lr, rf, rr, &sw, &leg_l, &leg_r);
    float sw_dot, l_dot, r_dot;
    leg_kin_v3_velocities(lfv, lrv, rfv, rrv, &sw_dot, &l_dot, &r_dot);
    float left_wheel_v = (float)s->lw_vel * R;
    float right_wheel_v = (float)s->rw_vel * R;
    x[0] = (float)s->roll;
    x[1] = (float)s->gyro[0];
    x[2] = leg_l - leg_r;
    x[3] = l_dot - r_dot;
    x[4] = 0.5f * (left_wheel_v - right_wheel_v);
#else
    const float R = 0.058f;
    float lf = (float)s->lf_pos, lr = (float)s->lr_pos;
    float rf = (float)s->rf_pos, rr = (float)s->rr_pos;
    float lfv = (float)s->lf_vel, lrv = (float)s->lr_vel;
    float rfv = (float)s->rf_vel, rrv = (float)s->rr_vel;
    float left_len = 0.5f * (0.4f - lf - lr);
    float right_len = 0.5f * (rf + rr + 0.5f);
    float left_ext_dot = -0.5f * (lfv + lrv);
    float right_ext_dot = 0.5f * (rfv + rrv);
    float left_wheel_v = (float)s->lw_vel * R;
    float right_wheel_v = -(float)s->rw_vel * R;

    x[0] = (float)s->roll;
    x[1] = (float)s->gyro[0];
    x[2] = left_len - right_len;
    x[3] = left_ext_dot - right_ext_dot;
    x[4] = 0.5f * (left_wheel_v - right_wheel_v);
#endif
}

/* 按腿长选段 */
static int select_segment(float leg_len)
{
    for (int i = 0; i < LQR_N_SEG; i++) {
        if (leg_len >= lqr_pitch_segments[i].lo &&
            leg_len < lqr_pitch_segments[i].hi)
            return i;
    }
    if (leg_len < lqr_pitch_segments[0].lo) return 0;
    return LQR_N_SEG - 1;
}

void lqr_controller_update(const SensorState *s, double dt, ControlCmd *out)
{
    (void)dt;
    if (!g_ready) lqr_controller_init();

    float xp[LQR_P_STATE];
    build_pitch_state(s, xp);
    float xr[LQR_R_STATE];
    build_roll_state(s, xr);
    g_seg = select_segment((float)s->leg_len);

    /* ---- pitch 子系统: u_p = -K_p * (xp - x_trim) ---- */
    const LqrPitchSegment *segp = &lqr_pitch_segments[g_seg];
    float xdev_p[LQR_P_STATE];
    for (int i = 0; i < LQR_P_STATE; i++)
        xdev_p[i] = xp[i] - segp->x_trim[i];
    float up[LQR_P_INPUT] = {0};
    for (int j = 0; j < LQR_P_INPUT; j++)
        for (int i = 0; i < LQR_P_STATE; i++)
            up[j] -= segp->K[j][i] * xdev_p[i];
    g_pitch_out = up[0];

    /* ---- roll 子系统: u_r = -K_r * (xr - x_trim) ---- */
    const LqrRollSegment *segr = &lqr_roll_segments[g_seg];
    float xdev_r[LQR_R_STATE];
    for (int i = 0; i < LQR_R_STATE; i++)
        xdev_r[i] = xr[i] - segr->x_trim[i];
    float ur[LQR_R_INPUT] = {0};
    for (int j = 0; j < LQR_R_INPUT; j++)
        for (int i = 0; i < LQR_R_STATE; i++)
            ur[j] -= segr->K[j][i] * xdev_r[i];
    g_roll_out = ur[0];

    /* 限幅后再输出 */
    out->wheel_tau    = clampf_lqr(up[0], -LQR_WHEEL_TAU_MAX, LQR_WHEEL_TAU_MAX);
    out->yaw_tau      = clampf_lqr(ur[0], -LQR_YAW_TAU_MAX, LQR_YAW_TAU_MAX);
    out->leg_swing    = clampf_lqr(up[1], -LQR_LEG_SWING_MAX, LQR_LEG_SWING_MAX);
    out->leg_len      = clampf_lqr(up[2], -LQR_LEG_LEN_MAX, LQR_LEG_LEN_MAX);
    out->leg_len_diff = clampf_lqr(ur[1], -LQR_LEG_DIFF_MAX, LQR_LEG_DIFF_MAX);
    out->active       = 1;
}

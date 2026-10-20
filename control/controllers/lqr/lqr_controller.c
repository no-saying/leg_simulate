/*
 * lqr_controller.c - 分段 LQR 控制器实现
 * ================================================================
 * 从 SensorState 提取 pitch/roll 两个子系统的状态, 按实测 leg_len
 * 选择分段模型, 分别计算 u_extra = -K*(x-x_trim) 输出。
 *
 * 注意:
 *   - update 输出扰动量；update_full 输出绝对伺服参考和轮扭矩。
 *   - pitch 与 roll 子系统独立选段/独立 LQR, 互不污染。
 *   - 状态全部来自真实传感器(IMU/编码器/关节角), 不依赖世界坐标。
 */
#include "lqr_controller.h"

#include <math.h>
#include <stdio.h>

#include "control_config.h"
#include "config_controllers.h"
#include "attitude_rates.h"
#include "lqr_pitch_models.h"
#include "lqr_pitch_damped_models.h"
#include "lqr_roll_models.h"
#include "lqr_pitch3_models.h"
#include "lqr_pitch_analytic.h"
#include "leg_kin_v3.h"

#ifndef LQR_IDENT_ACCEPTED
#define LQR_IDENT_ACCEPTED 0
#endif
#ifndef LQR_IDENT_FILTER_ENABLED
#define LQR_IDENT_FILTER_ENABLED 1
#endif

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

/* ---- 状态预滤波 + 模型采样率对齐 ----
 * 1) 辨识时对状态/输入做了 2 阶 4Hz 零相位低通(scripts/tools/identify_lqr_v3.py
 *    的 V3_FILT_FC)。部署侧必须施加同一个滤波器, 否则模型对不上——尤其是
 *    pitch_rate/leg_len_rate 这类差分通道, 不滤的话噪声会被 K 直接放大。
 *    这里用因果 2 阶 Butterworth(4Hz), 为了不额外引入相位滞后, 只对
 *    "速率类"状态滤波(位置类本来就很干净), 且极点与辨识滤波一致。
 * 2) 辨识模型的离散周期由 LQR_*_SAMPLE_DT 给出(当前为 5ms), 而控制环 1kHz。
 *    这里按模型周期更新 K*(x-x_trim) 并在其间保持(零阶保持), 与辨识时的
 *    输入速率一致。
 */
#if LQR_IDENT_FILTER_ENABLED
static float g_pitch_rate_lp = 0.0f, g_pitch_rate_lp2 = 0.0f;
static float g_roll_rate_lp = 0.0f, g_roll_rate_lp2 = 0.0f;
static float g_leg_sw_rate_lp = 0.0f, g_leg_sw_rate_lp2 = 0.0f;
static float g_leg_len_rate_lp = 0.0f, g_leg_len_rate_lp2 = 0.0f;
static float g_lendiff_rate_lp = 0.0f, g_lendiff_rate_lp2 = 0.0f;
#endif
static float g_lqr_accum = 0.0f;
/* 运动接管: wheel_v 的状态参考(机体系加速度计坐标系下, 与 x[2] 同号)。
 * 置 valid=0 时退回模型配平(x_ss), 即只稳姿态、不管运动。 */
static float g_v_ref[1] = {0.0f};
static float g_ref_pitch, g_ref_sw, g_ref_ext, g_ref_diff;
static float g_ref_yaw;
void lqr_controller_set_yaw_ref(float yaw_rate) { g_ref_yaw = yaw_rate; }

void lqr_controller_set_pose_ref(float pitch, float swing, float extension, float differential)
{
    g_ref_pitch = pitch;
    g_ref_sw = swing;
    g_ref_ext = extension;
    g_ref_diff = differential;
}
static int   g_v_ref_valid[1] = {0};
static float g_out_wheel = 0.0f, g_out_yaw = 0.0f, g_out_sw = 0.0f;
static float g_out_len = 0.0f, g_out_diff = 0.0f;
/* 输出高通(泄放直流): 辨识用的 x_trim 是激励数据的每段均值, 与当前稳态
 * 不一致时 -K(x-x_trim) 会在静止时输出一个恒定轮扭矩把车推走(实测
 * 静止 20s 漂移 0.48m -> 4.1m)。这里对 LQR 输出做一阶高通(截止约 0.03Hz),
 * 稳态自动泄放到 0, 只保留真正的动态修正。 */
static float g_hp_wheel = 0.0f, g_hp_sw = 0.0f, g_hp_len = 0.0f, g_hp_diff = 0.0f;
#define LQR_HP_TAU 5.0f
static float hp_step(float y, float *dc, float dt)
{
    float a = dt / (LQR_HP_TAU + dt);
    *dc += a * (y - *dc);
    return y - *dc;
}

/* 2 阶 Butterworth 4Hz 在 1kHz 下的差分系数(scipy 预计算, 见脚本注释) */
#if LQR_IDENT_FILTER_ENABLED
#define LQR_LP_B0 0.00015515f
#define LQR_LP_B1 0.00031030f
#define LQR_LP_B2 0.00015515f
#define LQR_LP_A1 (-1.96446058f)
#define LQR_LP_A2 0.96508117f

static float lp2_step(float x, float *z1, float *z2)
{
    float y = LQR_LP_B0 * x + *z1;
    *z1 = LQR_LP_B1 * x - LQR_LP_A1 * y + *z2;
    *z2 = LQR_LP_B2 * x - LQR_LP_A2 * y;
    return y;
}
#endif

static float clampf_lqr(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void lqr_controller_init(void)
{
    g_seg = -1;
    g_ready = 1;
    /* Do not carry held commands or filter history across a reset. */
    g_pitch_out = 0.0f;
    g_roll_out = 0.0f;
    g_lqr_accum = 0.0f;
    g_v_ref[0] = 0.0f;
    g_ref_pitch = 0.0f;
    g_ref_sw = BALANCE_LEG_SW_NOM;
    g_ref_ext = g_ref_diff = 0.0f;
    g_ref_yaw = 0.0f;
    g_v_ref_valid[0] = 0;
    g_out_wheel = 0.0f;
    g_out_yaw = 0.0f;
    g_out_sw = 0.0f;
    g_out_len = 0.0f;
    g_out_diff = 0.0f;
    g_hp_wheel = 0.0f;
    g_hp_sw = 0.0f;
    g_hp_len = 0.0f;
    g_hp_diff = 0.0f;
#if LQR_IDENT_FILTER_ENABLED
    g_pitch_rate_lp = g_pitch_rate_lp2 = 0.0f;
    g_roll_rate_lp = g_roll_rate_lp2 = 0.0f;
    g_leg_sw_rate_lp = g_leg_sw_rate_lp2 = 0.0f;
    g_leg_len_rate_lp = g_leg_len_rate_lp2 = 0.0f;
    g_lendiff_rate_lp = g_lendiff_rate_lp2 = 0.0f;
#endif
}

int lqr_controller_model_valid(void) { return LQR_IDENT_ACCEPTED; }

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
    leg_kin_v3_forward_calibrated(lf, lr, rf, rr, &sw, &leg_l, &leg_r);
    float sw_dot, l_dot, r_dot;
    leg_kin_v3_velocities_calibrated(lf, lr, rf, rr, lfv, lrv, rfv, rrv,
                                     &sw_dot, &l_dot, &r_dot);
    float left_wheel_v = (float)s->lw_vel * R;
    float right_wheel_v = (float)s->rw_vel * R;

    x[0] = (float)s->pitch;
    x[1] = sensor_pitch_rate(s);
    x[2] = sw - BALANCE_LEG_SW_NOM;
    x[3] = sw_dot;
    x[4] = 0.5f * (leg_l + leg_r);
    x[5] = 0.5f * (l_dot + r_dot);
    x[6] = 0.5f * (left_wheel_v + right_wheel_v);
    float slip_mix = balance_get_slip_blend();
    float inertial_wheel = -balance_get_slip_inertial_v() - (float)s->leg_kin_v
                                                        - (float)s->pitch_kin_v;
    x[6] += slip_mix * (inertial_wheel - x[6]);
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
    x[1] = sensor_pitch_rate(s);
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
    leg_kin_v3_forward_calibrated(lf, lr, rf, rr, &sw, &leg_l, &leg_r);
    float sw_dot, l_dot, r_dot;
    leg_kin_v3_velocities_calibrated(lf, lr, rf, rr, lfv, lrv, rfv, rrv,
                                     &sw_dot, &l_dot, &r_dot);
    float left_wheel_v = (float)s->lw_vel * R;
    float right_wheel_v = (float)s->rw_vel * R;
    x[0] = (float)s->roll;
    x[1] = attitude_roll_rate(s->roll, s->pitch, s->gyro[0], s->gyro[1], s->gyro[2]);
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

/* 线性化段按腿长排序。相邻段在中心点间交叉渐变，避免腿长越过
 * bin 边界时 K/u_ss 硬切换造成控制输出阶跃；超出标定范围时钳在端段。 */
static void select_segment_blend(float leg_len, int *i0, int *i1, float *blend)
{
    float c0 = 0.5f * (lqr_pitch_segments[0].lo + lqr_pitch_segments[0].hi);
    if (leg_len <= c0) {
        *i0 = *i1 = 0;
        *blend = 0.0f;
        return;
    }
    for (int i = 0; i < LQR_N_SEG - 1; i++) {
        float ca = 0.5f * (lqr_pitch_segments[i].lo + lqr_pitch_segments[i].hi);
        float cb = 0.5f * (lqr_pitch_segments[i + 1].lo + lqr_pitch_segments[i + 1].hi);
        if (leg_len <= cb) {
            *i0 = i;
            *i1 = i + 1;
            *blend = clampf_lqr((leg_len - ca) / (cb - ca), 0.0f, 1.0f);
            return;
        }
    }
    *i0 = *i1 = LQR_N_SEG - 1;
    *blend = 0.0f;
}

float lqr_controller_trim_swing(float leg_len)
{
    int a, b;
    float blend;
    select_segment_blend(leg_len, &a, &b, &blend);
    return lqr_pitch_segments[a].u_ss[1] + blend *
        (lqr_pitch_segments[b].u_ss[1] - lqr_pitch_segments[a].u_ss[1]);
}

static void evaluate_segment(const LqrPitchSegment *p, const LqrRollSegment *r,
                             const float xp[LQR_P_STATE], const float xr[LQR_R_STATE],
                             int full, float v_ref_state, int v_ref_valid,
                             float up[LQR_P_INPUT], float ur[LQR_R_INPUT])
{
    float xp_ref[LQR_P_STATE];
    float xr_ref[LQR_R_STATE];
    const float *pxref = full ? p->x_ss : p->x_trim;
    const float *rxref = full ? r->x_ss : r->x_trim;
    for (int i = 0; i < LQR_P_STATE; i++) xp_ref[i] = pxref[i];
    for (int i = 0; i < LQR_R_STATE; i++) xr_ref[i] = rxref[i];
    /* Reference units and signs match the states collected under pure PID. */
    if (full && v_ref_valid)
        xp_ref[6] = v_ref_state;
    if (full) {
        xp_ref[0] = p->x_ss[0] + g_ref_pitch;
        xp_ref[1] = 0.0f;
        xp_ref[2] = g_ref_sw - BALANCE_LEG_SW_NOM;
        xp_ref[4] = LEG_V3_L_MID - g_ref_ext;
        xr_ref[2] = -2.0f * g_ref_diff;
    }
    for (int j = 0; j < LQR_P_INPUT; j++) {
        up[j] = full ? p->u_ss[j] : 0.0f;
        if (full && j == 1) up[j] = g_ref_sw;
        if (full && j == 2) up[j] = g_ref_ext;
        for (int i = 0; i < LQR_P_STATE; i++)
            up[j] -= p->K[j][i] * (xp[i] - xp_ref[i]);
        if (full && j == 0) {
            int segment = (int)(p - lqr_pitch_segments);
            const float *k = lqr_pitch_damped_K[segment];
            up[j] = p->u_ss[j] - k[0] * (xp[0] - xp_ref[0])
                              - k[1] * (xp[1] - xp_ref[1])
                              - k[2] * (xp[6] - xp_ref[6]);
        }
    }
    for (int j = 0; j < LQR_R_INPUT; j++) {
        ur[j] = full ? r->u_ss[j] : 0.0f;
        if (full && j == 1) ur[j] = g_ref_diff;
        for (int i = 0; i < LQR_R_STATE; i++)
            ur[j] -= r->K[j][i] * (xr[i] - xr_ref[i]);
    }
}

void lqr_controller_update(const SensorState *s, double dt, ControlCmd *out)
{
    if (!g_ready) lqr_controller_init();
    if (dt <= 0.0) dt = 0.001;

    float xp[LQR_P_STATE];
    build_pitch_state(s, xp);
    float xr[LQR_R_STATE];
    build_roll_state(s, xr);

    /* 状态预滤波(与辨识端 4Hz 零相位滤波对应; 这里只滤速率类通道) */
#if LQR_IDENT_FILTER_ENABLED
    xp[1] = lp2_step(xp[1], &g_pitch_rate_lp, &g_pitch_rate_lp2);
    xp[3] = lp2_step(xp[3], &g_leg_sw_rate_lp, &g_leg_sw_rate_lp2);
    xp[5] = lp2_step(xp[5], &g_leg_len_rate_lp, &g_leg_len_rate_lp2);
    xr[1] = lp2_step(xr[1], &g_roll_rate_lp, &g_roll_rate_lp2);
    xr[3] = lp2_step(xr[3], &g_lendiff_rate_lp, &g_lendiff_rate_lp2);
#endif
    /* 模型采样率对齐: 使用头文件中的离散模型周期, 控制环为 1kHz。
     * 按模型周期更新 K*(x-trim) 并在其间零阶保持, 避免在 1kHz 重复施加同一增益。 */
    g_lqr_accum += (float)dt;
    float model_dt = LQR_PITCH_SAMPLE_DT;
    if (g_lqr_accum < model_dt) {
        out->wheel_tau    = g_out_wheel;
        out->yaw_tau      = g_out_yaw;
        out->leg_swing    = g_out_sw;
        out->leg_len      = g_out_len;
        out->leg_len_diff = g_out_diff;
        out->active       = 1;
        return;
    }
    g_lqr_accum -= model_dt;

    int seg0, seg1;
    float seg_blend;
    select_segment_blend((float)s->leg_len, &seg0, &seg1, &seg_blend);
    g_seg = (seg_blend < 0.5f) ? seg0 : seg1;
    float up0[LQR_P_INPUT], ur0[LQR_R_INPUT];
    float up1[LQR_P_INPUT], ur1[LQR_R_INPUT];
    evaluate_segment(&lqr_pitch_segments[seg0], &lqr_roll_segments[seg0],
                     xp, xr, 0, 0.0f, 0, up0, ur0);
    evaluate_segment(&lqr_pitch_segments[seg1], &lqr_roll_segments[seg1],
                     xp, xr, 0, 0.0f, 0, up1, ur1);
    float up[LQR_P_INPUT], ur[LQR_R_INPUT];
    for (int j = 0; j < LQR_P_INPUT; j++)
        up[j] = up0[j] + seg_blend * (up1[j] - up0[j]);
    for (int j = 0; j < LQR_R_INPUT; j++)
        ur[j] = ur0[j] + seg_blend * (ur1[j] - ur0[j]);

    g_pitch_out = up[0];
    g_roll_out = ur[0];

    /* 限幅后再输出 */
    /* The identified 3-state speed controller contains the required DC
     * drive term.  High-pass filtering it removes the very command needed to
     * hold nonzero W/S speed, so bypass that filter only during speed mode. */
    float wheel_out = g_v_ref_valid[0]
                    ? up[0]
                    : hp_step(up[0], &g_hp_wheel, model_dt);
    out->wheel_tau = clampf_lqr(wheel_out,
                                 -LQR_WHEEL_TAU_MAX, LQR_WHEEL_TAU_MAX);
    out->yaw_tau      = clampf_lqr(ur[0], -LQR_YAW_TAU_MAX, LQR_YAW_TAU_MAX);
    out->leg_swing    = clampf_lqr(hp_step(up[1], &g_hp_sw, model_dt),
                                   -LQR_LEG_SWING_MAX, LQR_LEG_SWING_MAX);
    out->leg_len      = clampf_lqr(hp_step(up[2], &g_hp_len, model_dt),
                                   -LQR_LEG_LEN_MAX, LQR_LEG_LEN_MAX);
    out->leg_len_diff = clampf_lqr(hp_step(ur[1], &g_hp_diff, model_dt),
                                   -LQR_LEG_DIFF_MAX, LQR_LEG_DIFF_MAX);
    out->active       = 1;
    g_out_wheel = out->wheel_tau;
    g_out_yaw   = out->yaw_tau;
    g_out_sw    = out->leg_swing;
    g_out_len   = out->leg_len;
    g_out_diff  = out->leg_len_diff;
}

/*
 * lqr_controller_update_full - "独占模式": u = u_ss - K(x - x_ss)
 * ----------------------------------------------------------------
 * 与 update 的区别:
 *   - update():      u_extra = -K(x - x_trim)  作为扰动叠加在三环 PID 上;
 *   - update_full(): u       = u_ss - K(x - x_ss), u_ss/x_ss 是辨识模型反解
 *     出的"站住所需的稳态输入/状态"(见 identify_lqr_v3.py), 因此它自带配平,
 *     可以**完全替代**三环的对应通道(PID 让位)。
 * 状态滤波与采样率对齐与 update 完全一致(同一套静态滤波器/累加器)。
 */
void lqr_controller_update_full(const SensorState *s, double dt, ControlCmd *out)
{
    if (!g_ready) lqr_controller_init();
    if (dt <= 0.0) dt = 0.001;

    float xp[LQR_P_STATE], xr[LQR_R_STATE];
    build_pitch_state(s, xp);
    build_roll_state(s, xr);
#if LQR_IDENT_FILTER_ENABLED
    xp[1] = lp2_step(xp[1], &g_pitch_rate_lp, &g_pitch_rate_lp2);
    xp[3] = lp2_step(xp[3], &g_leg_sw_rate_lp, &g_leg_sw_rate_lp2);
    xp[5] = lp2_step(xp[5], &g_leg_len_rate_lp, &g_leg_len_rate_lp2);
    xr[1] = lp2_step(xr[1], &g_roll_rate_lp, &g_roll_rate_lp2);
    xr[3] = lp2_step(xr[3], &g_lendiff_rate_lp, &g_lendiff_rate_lp2);
#endif

    g_lqr_accum += (float)dt;
    float model_dt = LQR_PITCH_SAMPLE_DT;
    if (g_lqr_accum < model_dt) {
        out->wheel_tau = g_out_wheel; out->yaw_tau = g_out_yaw;
        out->leg_swing = g_out_sw; out->leg_len = g_out_len;
        out->leg_len_diff = g_out_diff; out->active = 1;
        return;
    }
    g_lqr_accum -= model_dt;

    int seg0, seg1;
    float seg_blend;
    select_segment_blend((float)s->leg_len, &seg0, &seg1, &seg_blend);
    g_seg = (seg_blend < 0.5f) ? seg0 : seg1;
    float up0[LQR_P_INPUT], ur0[LQR_R_INPUT];
    float up1[LQR_P_INPUT], ur1[LQR_R_INPUT];
    evaluate_segment(&lqr_pitch_segments[seg0], &lqr_roll_segments[seg0],
                     xp, xr, 1, g_v_ref[0], g_v_ref_valid[0], up0, ur0);
    evaluate_segment(&lqr_pitch_segments[seg1], &lqr_roll_segments[seg1],
                     xp, xr, 1, g_v_ref[0], g_v_ref_valid[0], up1, ur1);
    float up[LQR_P_INPUT], ur[LQR_R_INPUT];
    for (int j = 0; j < LQR_P_INPUT; j++)
        up[j] = up0[j] + seg_blend * (up1[j] - up0[j]);
    for (int j = 0; j < LQR_R_INPUT; j++)
        ur[j] = ur0[j] + seg_blend * (ur1[j] - ur0[j]);

    g_pitch_out = up[0];
    g_roll_out = ur[0];
    out->wheel_tau    = clampf_lqr(up[0], -CTRL_LQR_FULL_WHEEL_MAX, CTRL_LQR_FULL_WHEEL_MAX);
    /* Scalar yaw-rate LQR for omega_dot=b*tau, b>0: CARE gives
     * K=sqrt(Q/R)=0.55 for Q=0.3025, R=1. The roll fit contains no
     * yaw-rate state. Convert wheel differential torque to mixer units. */
    /* At the requested wheel speed, damping needs a nonzero steady torque. */
    out->yaw_tau = clampf_lqr(WHEEL_YAW_DRAG_GAIN * g_ref_yaw +
                              0.55f * (g_ref_yaw - (float)s->yaw_rate), -0.8f, 0.8f)
                   / WHEEL_MIX_YAW_WEIGHT;
    /* The full output is an absolute servo position, not a +/-20 mm correction. */
    out->leg_swing = clampf_lqr(up[1], -0.065f, 0.065f);
    out->leg_len      = clampf_lqr(up[2], CTRL_LQR_FULL_EXT_MIN, CTRL_LQR_FULL_EXT_MAX);
    out->leg_len_diff = clampf_lqr(ur[1], -LQR_LEG_DIFF_MAX, LQR_LEG_DIFF_MAX);
    out->active = 1;
    g_out_wheel = out->wheel_tau; g_out_yaw = out->yaw_tau;
    g_out_sw = out->leg_swing; g_out_len = out->leg_len; g_out_diff = out->leg_len_diff;
}

/*
 * lqr_controller_update_pitch3 - 模态基 3 状态俯仰接管
 * ----------------------------------------------------------------
 * 状态 [pitch, pitch_rate(KF/低通后), wheel_v], 输入 [共模轮扭矩]。
 * 与 update_full 的区别: 状态集只含"实测可信"的三个量, K 只有 3 列,
 * 因此不会像满阵 K 那样把 roll/腿摆(不可辨识)的偏差经耦合列灌进轮扭矩
 * (实测满阵只接管轮扭矩会漂 6.65m)。
 * u = u_ss - K(x - x_ss), u_ss/x_ss 由分段模型反解得到(自带配平)。
 */
void lqr_controller_set_v_ref(float v_ref_state, int valid)
{
    g_v_ref[0] = v_ref_state;
    g_v_ref_valid[0] = valid ? 1 : 0;
}

void lqr_controller_update_pitch3(const SensorState *s, double dt, float *wheel_tau_out)
{
    if (!g_ready) lqr_controller_init();
    if (dt <= 0.0) dt = 0.001;
    float xp[LQR_P_STATE], xr[LQR_R_STATE];
    build_pitch_state(s, xp);
    build_roll_state(s, xr);
#if LQR_IDENT_FILTER_ENABLED
    xp[1] = lp2_step(xp[1], &g_pitch_rate_lp, &g_pitch_rate_lp2);
#endif
    float x[3] = { xp[0], xp[1], xp[6] };      /* pitch, pitch_rate, wheel_v */
    float leg_len = (float)s->leg_len;
    /* 段间按腿长线性插值 K / x_ss / u_ss(替代硬切换)。
     * 硬分段在段边界会让 K 从 -5.5 跳到 +13.5, 实测 |pitch| 因此到 2.2°;
     * 插值后增益连续, 相当于"按摆长在线调度"的离散近似。 */
    int seg = 0;
    for (int i = 0; i < LQR_P3_N_SEG; i++)
        if (leg_len >= lqr_pitch3_segments[i].lo &&
            leg_len <  lqr_pitch3_segments[i].hi) { seg = i; break; }
    float w = 0.0f;
    const LqrPitch3Segment *g0 = &lqr_pitch3_segments[seg];
    const LqrPitch3Segment *g1 = g0;
    if (seg + 1 < LQR_P3_N_SEG) {
        float c0 = 0.5f * (g0->lo + (g0->hi > 0.2f ? 0.2f : g0->hi));
        float c1 = 0.5f * (lqr_pitch3_segments[seg+1].lo +
                           (lqr_pitch3_segments[seg+1].hi > 0.2f ? 0.2f
                            : lqr_pitch3_segments[seg+1].hi));
        if (c1 > c0 + 1e-6f) {
            w = (leg_len - c0) / (c1 - c0);
            if (w < 0.0f) w = 0.0f;
            if (w > 1.0f) w = 1.0f;
        }
        g1 = &lqr_pitch3_segments[seg + 1];
    }
    /* ---- 配平用【解析值】, 不用辨识数据反解 ----
     * 平衡态的物理定义很明确: 竖直(pitch=0)、无运动(wheel_v=0), 此时
     * "质心落在轮轴正上方"由共模腿摆前馈保证(三环/前馈负责), 轮扭矩配平
     * 应为 0 —— 需要多少轮扭矩由 K 反馈自己给。
     * 之前用 pinv(B) 反解 u_ss/x_ss, 结果随段跳变(B 的轮扭矩列符号乱跳),
     * 造成 1.7° 的稳态 pitch 偏置。改成解析常量后, 配平天然连续且物理正确
     * (这就是用户说的"能算的算, 不要辨识")。 */
    /* 状态配平用解析值(竖直/无运动), 这是几何定义、必然正确;
     * 轮扭矩配平 u_ss 仍取该段数据均值 —— "质心略偏轮轴"需要一个小的
     * 常值轮扭矩来平衡重力, 这个标量只能由数据给(纯几何给不出 Offset),
     * 但它随段平滑, 不会造成 1.7° 的偏置。 */
    const float x_ss_a[3] = {0.0f, 0.0f, 0.0f};
    float u = (1.0f - w) * g0->u_ss[0] + w * g1->u_ss[0];
    for (int i = 0; i < LQR_P3_N_STATE; i++) {
        float kk = (1.0f - w) * g0->K[0][i] + w * g1->K[0][i];
        float xr_ = x[i];
        /* 运动接管: 反馈实测轮速与目标轮速之差。 */
        if (i == 2 && g_v_ref_valid[0]) xr_ -= g_v_ref[0];
        u -= kk * (xr_ - x_ss_a[i]);
    }
    g_pitch_out = u;
    *wheel_tau_out = clampf_lqr(u, -CTRL_LQR_FULL_WHEEL_MAX, CTRL_LQR_FULL_WHEEL_MAX);
}

/*
 * lqr_controller_update_analytic - 解析俯仰 LQR(不依赖辨识数据)
 * ----------------------------------------------------------------
 * 模型: 倒立摆 θ̈ = (g/h)·θ + τ/(m·h·r), v̇ = τ/(m·r), h 由 MJCF 读、
 * 随共模腿长 L_c 变化(44~56mm)。K 由 scripts/tools/analytic_pitch_model.py
 * 用连续 CARE 解出并查表, 运行时按实测腿长线性插值。
 * 状态 [pitch, pitch_rate, wheel_v], 输出[共模轮扭矩]。
 * 配平: 解析零点(pitch=0, wheel_v=v_ref), 不需要任何数据反解。
 */
void lqr_controller_update_analytic(const SensorState *s, double dt, float *wheel_tau_out)
{
    if (!g_ready) lqr_controller_init();
    if (dt <= 0.0) dt = 0.001;
    float xp[LQR_P_STATE], xr[LQR_R_STATE];
    build_pitch_state(s, xp);
    build_roll_state(s, xr);
#if LQR_IDENT_FILTER_ENABLED
    xp[1] = lp2_step(xp[1], &g_pitch_rate_lp, &g_pitch_rate_lp2);
#endif
    float x[3] = { xp[0], xp[1], xp[6] };

    /* 按腿长插值 K */
    float L = (float)s->leg_len;
    int i = 0;
    while (i < LQR_PA_N - 2 && L > lqr_pitch_analytic[i + 1].L) i++;
    const LqrPitchAnalyticPoint *a0 = &lqr_pitch_analytic[i];
    const LqrPitchAnalyticPoint *a1 = &lqr_pitch_analytic[i + 1 < LQR_PA_N ? i + 1 : i];
    float w = 0.0f;
    if (a1->L > a0->L + 1e-6f) {
        w = (L - a0->L) / (a1->L - a0->L);
        if (w < 0.0f) w = 0.0f;
        if (w > 1.0f) w = 1.0f;
    }
    float K[3];
    for (int k = 0; k < 3; k++)
        K[k] = (1.0f - w) * a0->K[k] + w * a1->K[k];

    float x_err[3] = { x[0], x[1], x[2] };
    if (g_v_ref_valid[0]) x_err[2] = x[2] - g_v_ref[0];
    /* 符号对齐: 解析模型用的是教科书约定(正共模轮扭矩 -> 正 θ̈), 而部署侧
     * 的轮扭矩/俯仰环约定与之相反(实测接管后静止漂 3.2m、pitch 5.1° 的典型
     * 正反馈特征)。这里整体取正号对齐部署约定 —— 一个常数就能修。 */
    /* 符号对齐(逐列, 不是整体):
     *  - pitch / pitch_rate: 与部署俯仰环同向 -> 取正;
     *  - wheel_v: 部署侧状态 wheel_v = 0.5(lw+rw)R, 前进时为【负】, 而解析
     *    模型的 v 是前进为正 -> 该列要再翻一次(整体翻符号会修好 pitch 却让
     *    速度项变正反馈: 实测静止漂 3.05m)。 */
    float u = (K[0] * x_err[0] + K[1] * x_err[1] - K[2] * x_err[2]);
    g_pitch_out = u;
    *wheel_tau_out = clampf_lqr(u, -CTRL_LQR_FULL_WHEEL_MAX, CTRL_LQR_FULL_WHEEL_MAX);
}

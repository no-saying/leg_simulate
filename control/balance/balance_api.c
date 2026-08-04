/*
 * balance_api.c - 平衡控制器公开接口(状态查询/指令注入)
 * ================================================================
 *
 * 从原 balance_core.c 拆出的所有对外接口:
 *   1. 平衡开关、手动驾驶指令、腿长指令、跳跃/自救触发。
 *   2. 各类运行状态只读查询(getter)，供 GUI/日志/行车记录仪使用。
 *   3. BalanceSensorFrame 结构化输入到历史 balance_update() 的适配。
 *
 * 这里不放控制算法；所有状态读写都通过 balance_internal.h 的 g_balance。
 */
#include "balance_internal.h"

#include <math.h>

void balance_init(void)  { g_balance.bal_mode = 0; }
void balance_toggle(void) { g_balance.bal_mode = !g_balance.bal_mode; }
int  balance_active(void) { return g_balance.bal_mode; }
int  balance_get_manual_active(void) { return g_balance.manual_active; }
int  balance_get_post_manual(void) { return g_balance.post_manual; }
float balance_get_manual_v(void) { return g_balance.manual_v; }
float balance_get_manual_yaw(void) { return g_balance.manual_yaw; }
float balance_get_hold_pos(void) { return g_balance.hold_pos; }

void balance_set_manual(float speed_ref, float yaw_rate_ref, int active)
{
    int want_active = (active && g_balance.bal_mode) ? 1 : 0;
    if (want_active && !g_balance.manual_active) {
        /* 刚进入手动: 保持点从当前位置开始，并把当前航向锁为新零点，
         * 这样 A/D 转向从当前朝向累积，松开后保持当前航向而不是转回启动朝向。 */
        g_balance.hold_pos = g_balance.pos;
        g_balance.yaw_zero = g_balance.yaw_raw_prev;
    }
    if (!want_active && g_balance.manual_active) {
        /* 松手边沿(只执行一次): 保持点吸附到当前实际位置，车停在这里。
         * 注意不能每帧都吸附，否则位置环永远无误差、车会自由漂移。 */
        g_balance.hold_pos = g_balance.pos;
        g_balance.yaw_zero = g_balance.yaw_raw_prev;
        g_balance.manual_neutral = 0.0f;
        /* 只有前后行驶后进停车刹车; 纯转向松手直接由 yaw 环保持航向。 */
        g_balance.post_manual = (fabsf(g_balance.manual_v) > 0.05f) ? 1 : 0;
        g_balance.post_manual_t = 0.0f;
    }
    g_balance.manual_v_target = speed_ref;
    g_balance.manual_yaw_target = yaw_rate_ref;
    g_balance.manual_active = want_active;
    if (!g_balance.manual_active) {
        g_balance.manual_v = 0.0f;
        g_balance.manual_yaw = 0.0f;
    }
}

float balance_get_neutral(void) { return g_balance.neutral; }
int  balance_get_fallen(void) { return g_balance.fallen; }
float balance_get_wheel_trust(void) { return g_balance.wheel_trust_dbg; }
float balance_get_pos_trust(void) { return g_balance.pos_trust_dbg; }
float balance_get_airborne(void) { return g_balance.airborne_dbg; }
float balance_get_impact(void) { return g_balance.impact_dbg; }
float balance_get_slip(void) { return g_balance.slip_dbg; }
float balance_get_raw_airborne(void) { return g_balance.raw_airborne_dbg; }
float balance_get_raw_impact(void) { return g_balance.raw_impact_dbg; }
float balance_get_raw_slip(void) { return g_balance.raw_slip_dbg; }
float balance_get_raw_zupt(void) { return g_balance.raw_zupt_dbg; }
float balance_get_support(void) { return g_balance.support_dbg; }
float balance_get_leg_load(void) { return g_balance.leg_load_dbg; }
float balance_get_leg_swing(void) { return g_balance.leg_swing; }
float balance_get_leg_extend(void) { return g_balance.leg_extend; }
float balance_get_leg_extend_manual(void) { return g_balance.leg_extend_manual; }
float balance_get_leg_len(void) { return g_balance.leg_len_dbg; }
void balance_set_leg_extend_manual(float v)
{
    g_balance.leg_extend_manual = clampf_local(v, -MANUAL_LEG_EXTEND_MAX, MANUAL_LEG_EXTEND_MAX);
}

/* 空格键: 触发一次跳跃时序(下蹲->蹬伸->空中->落地)。 */
void balance_trigger_jump(void)
{
#if MANUAL_JUMP_ENABLE
    /* 跳跃进行中忽略重复触发，避免中途打断时序导致落地失控。 */
    if (!g_balance.fallen && g_balance.airborne_dbg < 0.5f && !g_balance.jump_active &&
        fabsf(g_balance.last_pitch) < JUMP_TRIG_PITCH_MAX &&
        fabsf(g_balance.last_roll) < JUMP_TRIG_ROLL_MAX) {
        g_balance.jump_active = 1;
        g_balance.jump_t = 0.0f;
    }
#endif
}

/* 当前跳跃阶段: 0=无, 1=下蹲, 2=蹬伸, 3=空中, 4=落地缓冲。 */
int balance_get_jump_phase(void)
{
#if MANUAL_JUMP_ENABLE
    if (!g_balance.jump_active) return 0;
    float jt = g_balance.jump_t;
    if (jt < JUMP_CROUCH_T) return 1;
    if (jt < JUMP_CROUCH_T + JUMP_LAUNCH_T) return 2;
    if (jt < JUMP_CROUCH_T + JUMP_LAUNCH_T + JUMP_AIR_T) return 3;
    if (jt < JUMP_CROUCH_T + JUMP_LAUNCH_T + JUMP_AIR_T + JUMP_LAND_T) return 4;
    return 0;
#else
    return 0;
#endif
}
int balance_get_self_right_active(void) { return g_balance.self_right_active; }
/* H 键: 手动触发侧翻自救。 */
void balance_trigger_self_right(void)
{
#if SELF_RIGHT_ENABLE
    if (!g_balance.self_right_active) {
        g_balance.self_right_active = 1;
        g_balance.self_right_t = 0.0f;
    }
#endif
}
/* 自救脉冲: 推 0.35s / 松 0.25s 循环，避免连续推过冲。 */
int balance_get_self_right_push(void)
{
    return (fmodf(g_balance.self_right_t, 0.60f) < 0.35f) ? 1 : 0;
}
float balance_get_leg_side_extend(void) { return g_balance.leg_side_extend; }
void balance_set_leg_side_extend(float v)
{
    g_balance.leg_side_extend = clampf_local(v, -0.26f, 0.26f);
}
float balance_get_obs_phase(void) { return g_balance.obs_phase_dbg; }
float balance_get_return_score(void) { return g_balance.return_score_dbg; }
float balance_get_capture_score(void) { return g_balance.capture_score_dbg; }
float balance_get_odom_forward(void) { return g_balance.odom_forward_dbg; }
float balance_get_contact_mode(void) { return g_balance.contact_mode_dbg; }
float balance_get_side_balance(void) { return g_balance.side_support_balance; }
float balance_get_ekf_s(void) { return g_balance.ekf_s; }
float balance_get_odom_x(void) { return g_balance.odom_x; }
float balance_get_odom_y(void) { return g_balance.odom_y; }
float balance_get_wheel_v_filt(void) { return g_balance.wheel_v_filt; }
float balance_get_wheel_fir(void) { return g_balance.wheel_fir; }
float balance_get_wheel_thrash(void) { return g_balance.wheel_thrash; }
float balance_get_imu_thrash(void) { return g_balance.imu_thrash; }
float balance_get_wheel_acc(void) { return g_balance.wheel_acc_dbg; }
float balance_get_imu_for_slip(void) { return g_balance.imu_for_slip_dbg; }
float balance_get_wheel_imu_res(void) { return g_balance.wheel_imu_res_dbg; }
float balance_get_leg_kin_v(void) { return g_balance.leg_kin_v_dbg; }
float balance_get_pitch_kin_v(void) { return g_balance.pitch_kin_v_dbg; }
float balance_get_wheel_body_v(void) { return g_balance.wheel_body_v_dbg; }
float balance_get_yaw_error(void) { return g_balance.yaw_err_dbg; }

typedef enum {
    SENSOR_PITCH = 0,
    SENSOR_PITCH_RATE,
    SENSOR_ROLL,
    SENSOR_ROLL_RATE,
    SENSOR_LEFT_WHEEL_VEL,
    SENSOR_RIGHT_WHEEL_VEL,
    SENSOR_ACC_MAG,
    SENSOR_ACC_X,
    SENSOR_ACC_Z,
    SENSOR_YAW,
    SENSOR_YAW_RATE,
    SENSOR_LEFT_WHEEL_POS,
    SENSOR_RIGHT_WHEEL_POS,
    SENSOR_LF_POS,
    SENSOR_LR_POS,
    SENSOR_RF_POS,
    SENSOR_RR_POS,
    SENSOR_LF_TAU,
    SENSOR_LR_TAU,
    SENSOR_RF_TAU,
    SENSOR_RR_TAU,
    SENSOR_LEFT_WHEEL_TAU,
    SENSOR_RIGHT_WHEEL_TAU,
    SENSOR_LF_VEL,
    SENSOR_LR_VEL,
    SENSOR_RF_VEL,
    SENSOR_RR_VEL,
    SENSOR_LEGACY_COUNT
} BalanceLegacySensorIndex;

void balance_update_frame(const BalanceSensorFrame *frame, double dt, BalanceOutput *out)
{
    /*
     * 旧核心还使用 double 数组，索引含义集中在上面的枚举里。
     * 外部新代码只应该使用 BalanceSensorFrame，避免魔法下标继续扩散。
     */
    double legacy[SENSOR_LEGACY_COUNT] = {
        [SENSOR_PITCH] = frame->pitch,
        [SENSOR_PITCH_RATE] = frame->pitch_rate,
        [SENSOR_ROLL] = frame->roll,
        [SENSOR_ROLL_RATE] = frame->roll_rate,
        [SENSOR_LEFT_WHEEL_VEL] = frame->left_wheel_vel,
        [SENSOR_RIGHT_WHEEL_VEL] = frame->right_wheel_vel,
        [SENSOR_ACC_MAG] = frame->acc_mag,
        [SENSOR_ACC_X] = frame->acc_x,
        [SENSOR_ACC_Z] = frame->acc_z,
        [SENSOR_YAW] = frame->yaw,
        [SENSOR_YAW_RATE] = frame->yaw_rate,
        [SENSOR_LEFT_WHEEL_POS] = frame->left_wheel_pos,
        [SENSOR_RIGHT_WHEEL_POS] = frame->right_wheel_pos,
        [SENSOR_LF_POS] = frame->lf_pos,
        [SENSOR_LR_POS] = frame->lr_pos,
        [SENSOR_RF_POS] = frame->rf_pos,
        [SENSOR_RR_POS] = frame->rr_pos,
        [SENSOR_LF_TAU] = frame->lf_tau,
        [SENSOR_LR_TAU] = frame->lr_tau,
        [SENSOR_RF_TAU] = frame->rf_tau,
        [SENSOR_RR_TAU] = frame->rr_tau,
        [SENSOR_LEFT_WHEEL_TAU] = frame->left_wheel_tau,
        [SENSOR_RIGHT_WHEEL_TAU] = frame->right_wheel_tau,
        [SENSOR_LF_VEL] = frame->lf_vel,
        [SENSOR_LR_VEL] = frame->lr_vel,
        [SENSOR_RF_VEL] = frame->rf_vel,
        [SENSOR_RR_VEL] = frame->rr_vel,
    };

    float wheel_tau = 0.0f;
    float yaw_tau = 0.0f;
    float leg_swing = BALANCE_LEG_SW_NOM;
    float speed_ref = 0.0f;
    float position = 0.0f;
    balance_update(legacy, dt, &wheel_tau, &yaw_tau, &leg_swing, &speed_ref, &position);

    out->wheel_tau = wheel_tau;
    out->yaw_tau = yaw_tau;
    out->leg_swing = leg_swing;
    out->speed_ref = speed_ref;
    out->position = position;
}

void balance_tune_neutral(void) {}
void balance_tune_off(void) {}
void balance_adj_neutral(float d) { (void)d; }
int  balance_tuning(void) { return 0; }
int  balance_need_reset(void) { return 0; }

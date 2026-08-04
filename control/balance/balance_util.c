/*
 * balance_util.c - 平衡控制核心的通用工具函数
 * ================================================================
 * 从原 balance_core.c 拆分出来的纯工具:
 *   - 限幅 / 死区 / 衰减 / 角度回绕
 *   - 匹配 FIR 低通滤波(轮速/惯性加速度/驱动加速度共用)
 *   - 原点判定窗口(位置环能否积分/回正/释放)
 *   - 里程计/接触/调试状态的整体复位
 *
 * 这些函数不依赖具体控制环，可被俯仰环、速度环、偏航环共用。
 */
#include "balance_internal.h"

#include <math.h>

/* 匹配 FIR 滤波系数: 一型 Hamming 窗 sinc 低通，N=11，截止 2.5 Hz。
 * 线性相位、群延迟 5 拍(50ms)，轮速/惯性/驱动用同一组保证残差相位对齐。 */
static const float FIR_LP[FIR_N] = {
    0.01337904f, 0.02916792f, 0.07119788f, 0.12463806f, 0.16873978f,
    0.18575465f,
    0.16873978f, 0.12463806f, 0.07119788f, 0.02916792f, 0.01337904f
};

float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float decay_to_zero_local(float x, float rate, float dt)
{
    /* 按连续时间近似把状态拉向 0，rate 单位为 1/s。 */
    float k = clampf_local(rate * dt, 0.0f, 1.0f);
    x *= 1.0f - k;
    if (fabsf(x) < 1e-5f) x = 0.0f;
    return x;
}

float deadband_position_error(float pos, float deadband)
{
    /* 只返回死区外的误差，避免近原点时位置环不断追噪声。 */
    if (pos > deadband) return pos - deadband;
    if (pos < -deadband) return pos + deadband;
    return 0.0f;
}

float wrap_pi_local(float x)
{
    while (x > (float)M_PI) x -= 2.0f * (float)M_PI;
    while (x < -(float)M_PI) x += 2.0f * (float)M_PI;
    return x;
}

float fir_push(float *buf, float x)
{
    /* 环形缓冲卷积，最新样本位于 g_balance.fir_idx。 */
    buf[g_balance.fir_idx] = x;
    float y = 0.0f;
    int j = g_balance.fir_idx;
    for (int k = 0; k < FIR_N; k++) {
        y += FIR_LP[k] * buf[j];
        j--;
        if (j < 0) j = FIR_N - 1;
    }
    return y;
}

float fir_delayed_raw(const float *buf)
{
    /* 取与 FIR 输出同相位的原始延迟样本，用于计算高频残差。 */
    int j = g_balance.fir_idx - FIR_M;
    while (j < 0) j += FIR_N;
    return buf[j];
}

SimpleOriginWindow make_simple_origin_window(float pos, float body_v, float pitch,
                                             float pitch_rate, float imu_acc,
                                             float wheel_trust, int stable_odom,
                                             int slip, int zupt)
{
    SimpleOriginWindow window;
    window.pos_err = deadband_position_error(pos, SIMPLE_ORIGIN_DEADBAND);
    window.release_ok = (fabsf(pos) < SIMPLE_ORIGIN_RELEASE_POS &&
                         fabsf(body_v) < SIMPLE_ORIGIN_RELEASE_V);

#if SIMPLE_ORIGIN_SHORT_WINDOW_ENABLE
    /*
     * 短窗口策略:
     *   - 姿态安静、轮速可信、惯性加速度不大时才积分前向位置。
     *   - 如果短窗口里已经留下了位置记忆，高速退出窗口后仍允许用这段有界记忆刹回原点。
     *   - 进入原点释放窗口后不再继续用位置误差“拉住”车，让位置记忆自然回零。
     */
    window.track_ok = (stable_odom &&
                       !slip &&
                       !zupt &&
                       wheel_trust >= SIMPLE_ORIGIN_TRUST_WHEEL_MIN &&
                       fabsf(pitch) < SIMPLE_ORIGIN_TRUST_PITCH_RAD &&
                       fabsf(pitch_rate) < SIMPLE_ORIGIN_TRUST_RATE_RADPS &&
                       fabsf(body_v) < SIMPLE_ORIGIN_TRUST_BODY_V_MAX &&
                       fabsf(imu_acc) < SIMPLE_ORIGIN_TRUST_ACC_MAX);
    int memory_return_ok = (stable_odom && !slip);
    window.return_ok = ((window.track_ok || memory_return_ok) &&
                        !window.release_ok &&
                        fabsf(window.pos_err) > 1e-5f);
    window.trust_ref = window.return_ok ? 1.0f : (window.track_ok ? 0.45f : 0.0f);
#else
    window.track_ok = stable_odom;
    window.return_ok = (stable_odom &&
                        !window.release_ok &&
                        fabsf(window.pos_err) > 1e-5f);
    window.trust_ref = window.return_ok ? 1.0f : (window.track_ok ? 0.45f : 0.0f);
#endif
    return window;
}

void reset_odom_filter(void)
{
    /* 关闭平衡或重新启动时清空所有里程计/接触/调试状态，避免旧漂移带入新实验。 */
    g_balance.pos = 0.0f;
    g_balance.ekf_s = 0.0f;
    g_balance.odom_x = 0.0f;
    g_balance.odom_y = 0.0f;
    g_balance.vel = 0.0f;
    g_balance.vel_ff = 0.0f;
    g_balance.origin_creep = 0.0f;
    g_balance.manual_v = 0.0f;
    g_balance.manual_v_target = 0.0f;
    g_balance.manual_yaw = 0.0f;
    g_balance.manual_yaw_target = 0.0f;
    g_balance.manual_active = 0;
    g_balance.hold_pos = 0.0f;
    g_balance.post_manual = 0;
    g_balance.post_manual_rest = 0;
    g_balance.runaway_t = 0.0f;
    g_balance.pitch_i = 0.0f;
    g_balance.post_manual_t = 0.0f;
    g_balance.manual_neutral = 0.0f;
    g_balance.manual_v_i = 0.0f;
    g_balance.odom_rest_lock = 0;
    g_balance.fall_score = 0;
    g_balance.fallen = 0;
    g_balance.soft_s = 0.0f;
    g_balance.soft_s_prev = 0.0f;
    g_balance.capture_t = 0.0f;
    g_balance.settle_s = 0.0f;
    g_balance.away_s = 0.0f;
    g_balance.coast_t = 0.0f;
    g_balance.coast_s = 0.0f;
    g_balance.brake_latched = 0;
    g_balance.brake_dir = 0.0f;
    g_balance.origin_trust = 0.0f;
    g_balance.hold_v = 0.0f;
    g_balance.hold_v_age = 0.0f;
    g_balance.wheel_v_filt = 0.0f;
    g_balance.body_v_filt = 0.0f;
    g_balance.simple_imu_v = 0.0f;
    g_balance.simple_wheel_v_limited = 0.0f;
    g_balance.simple_wheel_v_ready = 0;
    g_balance.imu_acc_filt = 0.0f;
    g_balance.drive_acc_filt = 0.0f;
    g_balance.filter_ready = 0;
    g_balance.wheel_s_prev = 0.0f;
    g_balance.wheel_s_ready = 0;
    g_balance.acc_bias = 0.0f;
    g_balance.pos_i = 0.0f;
    g_balance.leg_swing = 0.0f;
    g_balance.leg_extend = 0.0f;
    g_balance.leg_extend_manual = 0.0f;
    g_balance.leg_side_extend = 0.0f;
    g_balance.roll_rate_filt = 0.0f;
    g_balance.roll_i = 0.0f;
    g_balance.leg_len_dbg = 0.0f;
    g_balance.idle_calm_t = 0.0f;
    g_balance.jump_active = 0;
    g_balance.jump_t = 0.0f;
    g_balance.last_pitch = 0.0f;
    g_balance.last_roll = 0.0f;
    g_balance.self_right_active = 0;
    g_balance.self_right_t = 0.0f;
    g_balance.drive_tau_bias = 0.0f;
    g_balance.obs_phase_dbg = 0.0f;
    g_balance.return_score_dbg = 0.0f;
    g_balance.capture_score_dbg = 0.0f;
    g_balance.odom_forward_dbg = 0.0f;
    g_balance.airborne_score = 0;
    g_balance.impact_score = 0;
    g_balance.slip_score = 0;
    g_balance.zupt_score = 0;
    g_balance.grip_score = 0;
    g_balance.airborne_dbg = 0.0f;
    g_balance.impact_dbg = 0.0f;
    g_balance.slip_dbg = 0.0f;
    g_balance.raw_airborne_dbg = 0.0f;
    g_balance.raw_impact_dbg = 0.0f;
    g_balance.raw_slip_dbg = 0.0f;
    g_balance.raw_zupt_dbg = 0.0f;
    g_balance.support_dbg = 1.0f;
    g_balance.leg_load_dbg = 0.0f;
    g_balance.contact_mode_dbg = (float)CONTACT_STICK;
    g_balance.side_support_balance = 1.0f;
    g_balance.wheel_acc_dbg = 0.0f;
    g_balance.imu_for_slip_dbg = 0.0f;
    g_balance.wheel_imu_res_dbg = 0.0f;
    g_balance.run_time = 0.0f;
    g_balance.tau_cmd = 0.0f;
    g_balance.pitch_rate_filt = 0.0f;
    g_balance.wheel_capture_dir = 0.0f;
    g_balance.wheel_capture_t = 0.0f;
    g_balance.pitch_rate_ready = 0;
    g_balance.yaw_elapsed = 0.0f;
    g_balance.yaw_held = 0.0f;
    g_balance.yaw_raw_prev = 0.0f;
    g_balance.yaw_zero = 0.0f;
    g_balance.yaw_cont = 0.0f;
    g_balance.yaw_err_dbg = 0.0f;
    g_balance.yaw_i = 0.0f;
    g_balance.yaw_ready = 0;
    g_balance.wheel_thrash = 0.0f;
    g_balance.wheel_fir = 0.0f;
    g_balance.imu_thrash = 0.0f;
    g_balance.P[0][0] = 0.04f; g_balance.P[0][1] = 0.0f;  g_balance.P[0][2] = 0.0f;
    g_balance.P[1][0] = 0.0f;  g_balance.P[1][1] = 0.25f; g_balance.P[1][2] = 0.0f;
    g_balance.P[2][0] = 0.0f;  g_balance.P[2][1] = 0.0f;  g_balance.P[2][2] = 0.09f;
}

/* ==========================================================================
 * balance_core.c - 平衡控制主入口
 * ==========================================================================
 *
 * 原 3200 行的单一文件按功能拆分为:
 *   - balance_internal.h : 共享状态结构/段落接口
 *   - balance_util.c     : 通用工具(滤波/限幅/窗口/复位)
 *   - balance_api.c      : 对外接口(getter/setter/跳跃/自救触发)
 *   - balance_step_*.c   : 各控制段落(传感器/原点/腿/俯仰偏航)
 *   - balance_core.c     : balance_update() 入口与段落编排
 *
 * 设计约束(保持不变):
 *   1. 实时控制只用真实机器人能拿到的量: 惯性测量、轮编码器、腿编码器、执行器力矩。
 *   2. 世界坐标只允许用于日志、图形界面和离线验证，不能进控制闭环。
 *   3. 三个主环并联工作: 俯仰共模扭矩、速度规划目标俯仰角、偏航差动扭矩。
 *
 * 当前分频:
 *   - 俯仰平衡环: 每个物理步，模型步长为 0.001 秒时等效 1 kHz。
 *   - 速度/里程计环: 100 Hz。
 *   - 偏航环: 50 Hz。
 */
#include "balance_internal.h"
#include "leg_kin_v3.h"
#include "vel_kf.h"
#include "attitude_rates.h"
#include "../controllers/lqr/leg_trim_table.h"
#include "../controllers/lqr/lqr_pitch_analytic.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* 全局平衡状态对象: 唯一实例，各段落文件通过 balance_internal.h 的 extern 访问。 */
BalanceRuntime g_balance = {
    .P = {{0.04f, 0.0f, 0.0f}, {0.0f, 0.25f, 0.0f}, {0.0f, 0.0f, 0.09f}},
    .wheel_trust_dbg = 1.0f,
    .pos_trust_dbg = 1.0f,
    .support_dbg = 1.0f,
    .contact_mode_dbg = (float)CONTACT_STICK,
    .side_support_balance = 1.0f,
};

/* 腿摆名义角(rad): 等效在调质心/静态平衡点，供各段落文件读取。 */
float balance_sw_nom(void) { return BALANCE_LEG_SW_NOM; }

/* 摆的等效摆长 h(L): 质心相对轮轴的竖直高度, 由解析模型从 MJCF 读出
 * (h=56.4mm@L=0.09 → 44.3mm@L=0.15)。倾角->加速度映射 a = g·θ/h,
 * 所以同样的速度误差要产生同样的加速度, 倾角指令必须按 h(L) 缩放。 */
float balance_h_of_L(float leg_len)
{
    int i = 0;
    while (i < LQR_PA_N - 2 && leg_len > lqr_pitch_analytic[i + 1].L) i++;
    const LqrPitchAnalyticPoint *a = &lqr_pitch_analytic[i];
    const LqrPitchAnalyticPoint *b = &lqr_pitch_analytic[i + 1 < LQR_PA_N ? i + 1 : i];
    float w = 0.0f;
    if (b->L > a->L + 1e-6f) {
        w = (leg_len - a->L) / (b->L - a->L);
        if (w < 0.0f) w = 0.0f;
        if (w > 1.0f) w = 1.0f;
    }
    return (1.0f - w) * a->h + w * b->h;
}

/* 腿摆配平参考，见 leg_trim_table.h 的纯 PID 标定区间。 */
float leg_trim_dev(float leg_len)
{
    /* 标定用: 可用环境变量给配平表加一个常数偏置(标定/试验用) */
    static int off_ready = 0;
    static float off = 0.0f;
    static float scale = 1.0f;
    if (!off_ready) {
        const char *e = getenv("LEG_TRIM_OFFSET");
        off = e ? (float)atof(e) : 0.0f;
        e = getenv("LEG_TRIM_SCALE");
        if (e) scale = (float)atof(e);
        off_ready = 1;
    }
    int i = 0;
    while (i < LEG_TRIM_N - 2 && leg_len > leg_trim_table[i + 1].L) i++;
    const LegTrimPoint *a = &leg_trim_table[i];
    const LegTrimPoint *b = &leg_trim_table[i + 1 < LEG_TRIM_N ? i + 1 : i];
    float w = 0.0f;
    if (b->L > a->L + 1e-6f) {
        w = (leg_len - a->L) / (b->L - a->L);
        if (w < 0.0f) w = 0.0f;
        if (w > 1.0f) w = 1.0f;
    }
    return scale * ((1.0f - w) * a->sw_dev + w * b->sw_dev) + off;
}

/* 手动前后驱动的腿摆权限窗口(m)。
 * ------------------------------------------------
 * 历史: 曾经用 [0.085, 0.140] 判定"腿摆没有权限", 窗口外把腿摆清零、
 * 改由目标俯仰角(倾角)驱动轮扭矩。2026-09-26 全腿长扫描(A/B 实验)
 * 证明这个判据是错的, 而且降级路径本身是坏的:
 *   1) 腿摆在 0.072~0.157m 全程都能驱动车体(摆角小但杠杆有效), 窗口
 *      内外的 W/S/A/D/Z 全腿长实测都能正常动;
 *   2) 降级路径(倾角驱动)在任何腿长下都把车往【反方向】推:
 *      L=0.110m(GUI 可控段)强制打开降级路径后, 按 W 同样反向加速到
 *      4.3m/s 然后翻车 —— 说明不是"短腿无权限", 而是这条路径的
 *      速度误差->倾角符号与物理相反, 且没有刹车能力;
 *   3) 腿摆被清零的同时 `-K·v` 轮速阻尼保留(见 pitch_yaw 段), 在这个
 *      位形上它也是正反馈, 于是"按 W 反向疯狂加速、压根没刹车"。
 * 现在改为: 腿摆速度环是唯一的前后执行器, 窗口只作为"腿长读数是否
 * 合理"的兜底(超出机械行程就是传感器/机构异常, 不驱动)。
 * 默认范围取实际机械行程(最短 0.0716m / 最长 0.157m)再各留 5mm 余量。 */
static float g_leg_drive_min = -1.0f;
static float g_leg_drive_max = -1.0f;

float balance_leg_drive_min(void)
{
#if MODEL_V3
    if (g_balance.turn_full_range)
        return LEG_V3_L_MID - CTRL_LQR_FULL_EXT_MAX - 0.008f;
#endif
    if (g_leg_drive_min < 0.0f) {
        const char *e = getenv("LEGDRIVE_MIN");
        g_leg_drive_min = e ? (float)atof(e) : 0.066f;
    }
    return g_leg_drive_min;
}

float balance_leg_drive_max(void)
{
#if MODEL_V3
    if (g_balance.turn_full_range)
        return LEG_V3_L_MID - CTRL_LQR_FULL_EXT_MIN + 0.008f;
#endif
    if (g_leg_drive_max < 0.0f) {
        const char *e = getenv("LEGDRIVE_MAX");
        g_leg_drive_max = e ? (float)atof(e) : 0.165f;
    }
    return g_leg_drive_max;
}

void balance_update_with_imu(const double *s, double dt,
                    float *tau, float *yaw, float *sw,
                    float *spd, float *ex, float acc_y, float gyro_y)
{
    /* 输入数组由 controller.c 组包。这里保持显式拆包，避免传感器顺序变更时静默出错。 */
    float pitch      = (float)s[0];
    float pitch_rate = (float)s[1];
    float roll       = (float)s[2];
    float roll_rate  = (float)s[3];
    g_balance.last_pitch = pitch;
    g_balance.last_roll = roll;
#if SELF_RIGHT_ENABLE
    /* 侧翻自救: 由按键(H)手动触发进入，回正后自动退出。 */
    if (g_balance.self_right_active &&
               fabsf(roll) < SELF_RIGHT_ROLL_OFF && fabsf(pitch) < SELF_RIGHT_PITCH_OFF) {
        g_balance.self_right_active = 0;
        g_balance.self_right_t = 0.0f;
    }
    if (g_balance.self_right_active) g_balance.self_right_t += dt;
#endif
    float lw_vel     = (float)s[4];
    float rw_vel     = (float)s[5];
    float acc_mag    = (float)s[6];
    float acc_x      = (float)s[7];
    float acc_z      = (float)s[8];
    float yaw_ang    = (float)s[9];
    float yaw_rate   = (float)s[10];
    float lw_pos     = (float)s[11];
    float rw_pos     = (float)s[12];
    float lf_pos     = (float)s[13];
    float lr_pos     = (float)s[14];
    float rf_pos     = (float)s[15];
    float rr_pos     = (float)s[16];
    float lf_tau     = (float)s[17];
    float lr_tau     = (float)s[18];
    float rf_tau     = (float)s[19];
    float rr_tau     = (float)s[20];
    float lw_tau     = (float)s[21];
    float rw_tau     = (float)s[22];
    float lf_vel_j   = (float)s[23];
    float lr_vel_j   = (float)s[24];
    float rf_vel_j   = (float)s[25];
    float rr_vel_j   = (float)s[26];
    float dt_f       = (dt > 0.0) ? (float)dt : 0.001f;
    (void)lw_pos;
    (void)rw_pos;
    (void)lw_tau;
    (void)rw_tau;

    /* 侧翻检测: |roll|>0.55rad(约31.5°)持续0.2s判定摔倒，停机直到复位。 */
    if (!g_balance.fallen) {
        if (fabsf(roll) > 0.55f) {
            g_balance.fall_score++;
            if (g_balance.fall_score > 200) {
                g_balance.fallen = 1;
                g_balance.manual_active = 0;
                g_balance.manual_v = 0.0f;
                g_balance.manual_v_target = 0.0f;
                g_balance.manual_yaw = 0.0f;
                g_balance.manual_yaw_target = 0.0f;
                printf("[Fall] roll=%.1f deg, wheel torque stopped\n",
                       roll * 57.29578);
            }
        } else if (g_balance.fall_score > 0) {
            g_balance.fall_score -= 4;
        }
    }

    *spd = g_balance.vel;
    *ex  = g_balance.pos;

    /* v3: 左右轮轴同向(都绕世界 -y)，轮编码器符号与运动方向相反:
     * 实测车体 +x 前进时 lw/rw 均为负(轮向后滚), 所以取 -(lw+rw)。
     * 旧模型左右轮轴相反用 lw-rw，见 git 历史。 */
#if MODEL_V3
    float fwd_spd = -(lw_vel + rw_vel) * 0.5f;
#else
    float fwd_spd = (lw_vel - rw_vel) * 0.5f;
#endif
    float wheel_v = fwd_spd * WHEEL_RADIUS;

    if(!g_balance.bal_mode){
        /* 平衡关闭时不保留任何控制积分和里程计历史。 */
        g_balance.fall_score = 0;
        g_balance.fallen = 0;
        *tau = 0; *yaw = 0;
        reset_odom_filter();
        g_balance.neutral = 0;
        g_balance.wheel_trust_dbg = 1.0f;
        g_balance.pos_trust_dbg = 1.0f;
        g_balance.airborne_dbg = 0.0f;
        g_balance.impact_dbg = 0.0f;
        g_balance.slip_dbg = 0.0f;
        g_balance.raw_airborne_dbg = 0.0f;
        g_balance.raw_impact_dbg = 0.0f;
        g_balance.raw_slip_dbg = 0.0f;
        g_balance.raw_zupt_dbg = 0.0f;
        g_balance.support_dbg = 1.0f;
        g_balance.leg_load_dbg = 0.0f;
        g_balance.leg_extend = 0.0f;
        g_balance.leg_extend_manual = 0.0f;
        g_balance.leg_side_extend = 0.0f;
        g_balance.obstacle_roll_hold = 0.0f;
        g_balance.obstacle_retract = 0.0f;
        for (int i = 0; i < 2; ++i) {
            g_balance.obstacle_wheel_hold[i] = 0.0f;
            g_balance.obstacle_leg_retract[i] = 0.0f;
        }
        g_balance.leg_swing_diff = 0.0f;
        g_balance.leg_trim_ref = 0.0f;
        g_balance.leg_trim_ready = 0;
        g_balance.roll_rate_filt = 0.0f;
        g_balance.roll_i = 0.0f;
        g_balance.idle_calm_t = 0.0f;
        g_balance.jump_active = 0;
        g_balance.jump_t = 0.0f;
        g_balance.last_pitch = 0.0f;
        g_balance.last_roll = 0.0f;
        g_balance.self_right_active = 0;
        g_balance.self_right_t = 0.0f;
        g_balance.obs_phase_dbg = 0.0f;
        g_balance.return_score_dbg = 0.0f;
        g_balance.capture_score_dbg = 0.0f;
        g_balance.odom_forward_dbg = 0.0f;
        g_balance.contact_mode_dbg = (float)CONTACT_STICK;
        g_balance.wheel_acc_dbg = 0.0f;
        g_balance.imu_for_slip_dbg = 0.0f;
        g_balance.wheel_imu_res_dbg = 0.0f;
        g_balance.leg_kin_v_dbg = 0.0f;
        g_balance.pitch_kin_v_dbg = 0.0f;
        g_balance.wheel_body_v_dbg = 0.0f;
        g_balance.tau_cmd = 0.0f;
        g_balance.speed_elapsed = 0.0f;
        g_balance.yaw_elapsed = 0.0f;
        g_balance.yaw_held = 0.0f;
        g_balance.yaw_out = 0.0f;
        g_balance.yaw_raw_prev = 0.0f;
        g_balance.yaw_zero = 0.0f;
        g_balance.yaw_cont = 0.0f;
        g_balance.yaw_err_dbg = 0.0f;
        g_balance.yaw_i = 0.0f;
        g_balance.yaw_ready = 0;
        g_balance.drive_tau_bias = 0.0f;
        g_balance.run_time = 0.0f;
        *sw = balance_sw_nom();
        return;
    }

#if USE_SIMPLE_THREE_LOOP
    /* ===== 简化三环路径主体(默认): 三段 include ===== */
#include "balance_step_sensor.inc"
#include "balance_step_origin.inc"
#include "balance_step_pitch_yaw.inc"
    }
    *yaw = g_balance.fallen ? 0.0f : g_balance.yaw_held;
    return;
#else
    /* ===== 旧速度环路径(USE_SIMPLE_THREE_LOOP=0 备用): 四段 include ===== */
#include "balance_step_legacy1.inc"
#include "balance_step_legacy2.inc"
#include "balance_step_legacy3.inc"
#include "balance_step_legacy4.inc"
#endif
}

void balance_update(const double *s, double dt, float *tau, float *yaw,
                    float *sw, float *spd, float *ex)
{
    /* Keep the 27-channel legacy ABI. Full turning observations use the
     * structured frame, which supplies the measured lateral acceleration. */
    float lateral_gravity = 9.81f * cosf((float)s[0]) * sinf((float)s[2]);
    balance_update_with_imu(s, dt, tau, yaw, sw, spd, ex,
                            lateral_gravity, (float)s[1]);
}

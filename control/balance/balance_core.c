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
#include <math.h>
#include <stdio.h>

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

void balance_update(const double *s, double dt,
                    float *tau, float *yaw, float *sw,
                    float *spd, float *ex)
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

    /* 左右轮轴向相反，所以共模前进速度用 lw-rw。半径必须和 MJCF 轮半径一致。 */
    float fwd_spd = (lw_vel - rw_vel) * 0.5f;
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

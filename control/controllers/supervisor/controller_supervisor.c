/*
 * controller_supervisor.c - 控制器模式状态机
 * ================================================================
 * 用有限状态机统一管理"正常平衡 / 前翻自救 / 后翻自救":
 *
 *   ST_IDLE       正常平衡: PID/LQR/MIXED 由姿态决定, 无自救动作。
 *   ST_PREP       检测到翻倒: 记录方向, 把腿摆到撑地侧, 等待就位。
 *   ST_PUSH       撑地推回: 腿保持撑地 + 轮扭矩推车回竖直。
 *   ST_SETTLE     回正确认: 接近竖直后保持, 确认动量吸收完才回 IDLE。
 *   ST_ABORT      超时/滚转过大: 放弃自救, 回到 IDLE 交 PID(防无限打腿)。
 *
 * 相比原来的"模式+滞回"写法, 状态机显式跟踪自救进度, 每个状态有
 * 进入/退出条件与超时, 避免能量整形和 PID 在阈值附近反复争夺、
 * 车轮疯狂空转把车越推越翻。
 */
#include "controller_supervisor.h"

#include <math.h>
#include <stdio.h>

#include "control_config.h"
#include "config_controllers.h"
#include "attitude_rates.h"
#include "../../balance/balance.h"
#include "../lqr/lqr_controller.h"
#include "../energy/energy_controller.h"
#include "../rl_jump/rl_jump_controller.h"

/* 状态机阶段(内部) */
enum {
    ST_IDLE = 0,
    ST_PREP,
    ST_PUSH,
    ST_SETTLE,
    ST_ABORT,
};

/* ---- 状态机运行状态 ---- */
static int  g_st = ST_IDLE;
static int  g_dir = 0;            /* +1=前翻(车头朝下), -1=后翻(车尾朝下) */
static float g_st_t = 0.0f;       /* 当前状态累计时间 s */
static float g_settle_t = 0.0f;   /* 回正保持时间 s */
static float g_swing_cmd = 0.0f;  /* 腿摆平滑输出 */
static float g_len_cmd = 0.0f;    /* 腿长平滑输出(正=收腿/降低, 负=伸长/升高) */
static float g_tau_cmd = 0.0f;    /* 轮扭矩平滑输出 */
static int  g_ready = 0;
static int  g_rl_busy = 0;        /* RL 接管标志: 跳过翻倒检测自救 */
static int  g_full_on = 0;
static float g_full_mix = 0.0f;
static float g_full_swing_ref = 0.0f;
static float g_full_park_trim = 0.0f;
static int g_lqr_enabled = 1;

static float clampf_sv(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void controller_supervisor_init(void)
{
    g_st = ST_IDLE;
    g_dir = 0;
    g_st_t = 0.0f;
    g_settle_t = 0.0f;
    g_swing_cmd = 0.0f;
    g_len_cmd = 0.0f;
    g_tau_cmd = 0.0f;
    g_rl_busy = 0;
    g_full_on = 0;
    g_full_mix = 0.0f;
    g_full_swing_ref = 0.0f;
    g_full_park_trim = 0.0f;
    g_lqr_enabled = 1;
    g_ready = 1;
    lqr_controller_init();
    energy_controller_init();
}

void controller_supervisor_set_rl_busy(int on)
{
    g_rl_busy = on ? 1 : 0;
}

void controller_supervisor_set_lqr_enabled(int enabled)
{
    if (!g_ready) controller_supervisor_init();
    g_lqr_enabled = enabled != 0;
}

CtrlMode controller_supervisor_mode(void)
{
    if (g_st == ST_IDLE) {
        if (g_full_mix >= 1.0f) return CTRL_MODE_LQR;
        if (g_full_mix > 0.0f) return CTRL_MODE_MIXED;
        return CTRL_MODE_PID;
    }
    return CTRL_MODE_ENERGY;
}

int controller_supervisor_phase(void)
{
    return g_st;
}

float controller_supervisor_lqr_mix(void) { return g_full_mix; }

void controller_supervisor_update(const SensorState *s, double dt, ControlCmd *out)
{
    if (!g_ready) controller_supervisor_init();

    float pitch = (float)s->pitch;
    float pitch_deg = (float)(fabs(pitch) * 57.29578f);
    float roll_deg = (float)(fabs(s->roll) * 57.29578f);
    float pitch_rate = sensor_pitch_rate(s);
    int lqr_ready = g_lqr_enabled && (s->sim_time > CTRL_LQR_STARTUP_DELAY);
    float dtf = (float)dt;

    g_st_t += dtf;

    /* 跳跃/自救期间状态机不介入: 跳跃会大幅改变 pitch 和腿长,
     * 正常行驶的加速前倾也可能超过进入阈值, 不能误判为翻倒。
     * balance_get_jump_phase() 1=下蹲 2=蹬伸 3=空中 4=落地。 */
    int jump_busy = g_rl_busy
                    || (balance_get_jump_phase() != 0) || balance_get_self_right_active()
                    || rl_jump_controller_active();
    (void)jump_busy;

    /* ================= 状态转移 ================= */
    switch (g_st) {
    case ST_IDLE: {
        /* 自动前后翻自救暂时禁用(用户要求): 偶发误触发会在训练预热/
         * 收腿阶段把车折腾翻。三环/LQR 轮子稳定不受影响。
         * 如需恢复, 取消下面注释即可。 */
        /*
        if (!jump_busy &&
            pitch_deg >= CTRL_ENERGY_ENTER_DEG && roll_deg < 25.0f) {
            g_dir = (pitch > 0.0f) ? 1 : -1;
            g_st = ST_PREP;
            g_st_t = 0.0f;
            g_swing_cmd = 0.0f;
            g_tau_cmd = 0.0f;
            printf("[SV] PREP dir=%+d pitch=%+.1f°\n", g_dir, pitch_deg);
        }
        */
        break;
    }
    case ST_PREP: {
        /* 摆腿到撑地侧: 前翻 0.23(腿前撑), 后翻 -1.0(腿后撑) */
        float target = (g_dir > 0) ? CTRL_SWING_FWD : CTRL_SWING_BACK;
        float max_d = CTRL_SWING_RATE * dtf;
        float d = target - g_swing_cmd;
        d = clampf_sv(d, -max_d, max_d);
        g_swing_cmd += d;
        /* 就位(误差<0.05)且姿态仍大 -> 进入撑地推回 */
        if (fabsf(target - g_swing_cmd) < 0.05f && pitch_deg > 8.0f) {
            g_st = ST_PUSH;
            g_st_t = 0.0f;
            printf("[SV] PUSH dir=%+d\n", g_dir);
        }
        /* 摆腿过程中自己回正了 -> 保持一段时间确认才回 IDLE(防振荡反复) */
        if (pitch_deg < 6.0f && fabsf(pitch_rate) < 2.0f) {
            g_settle_t += dtf;
            if (g_settle_t > 0.2f) {
                g_st = ST_IDLE;
                g_st_t = 0.0f;
                g_dir = 0;
                g_settle_t = 0.0f;
            }
        } else {
            g_settle_t = 0.0f;
        }
        /* 超时保护 */
        if (g_st_t > CTRL_PREP_TIMEOUT) {
            g_st = ST_ABORT;
            g_st_t = 0.0f;
            printf("[SV] ABORT(prep timeout)\n");
        }
        break;
    }
    case ST_PUSH: {
        /* 撑地自然回正: 腿摆到撑地侧 + 收腿到最短。
         * 决定性实验(纯腿伺服, 无轮扭矩):
         *   前翻60° sw=+0.23+收腿0.2 -> -2.1° 回正
         *   后翻-60° sw=-1.00+收腿0.2 -> +16.1° 接近回正
         * 所以 PUSH 只需保持撑地腿位+收腿, 车靠腿撑+重力自然翻回,
         * 不需要主动轮扭矩(只留微小方向辅助帮助最后几度)。 */
        float target = (g_dir > 0) ? CTRL_SWING_FWD : CTRL_SWING_BACK;
        float max_d = CTRL_SWING_RATE * dtf;
        float d = target - g_swing_cmd;
        d = clampf_sv(d, -max_d, max_d);
        g_swing_cmd += d;

        /* 收腿到最短: leg_len 参考 = 最大收腿量 */
        float len_ref = CTRL_PUSH_LEN;
        float max_ld = CTRL_LEN_RATE * dtf;
        float dld = len_ref - g_len_cmd;
        dld = clampf_sv(dld, -max_ld, max_ld);
        g_len_cmd += dld;

        /* 完全靠腿撑地+收腿自然撬回, 不给轮扭矩(实验证明扭矩会干扰)。
         * 实验: 后翻撑地-1.0+收腿0.2, 无轮扭矩, 2s 撬到+14°, 摆腿回
         * 0.23+放腿后稳定-2.1°。 */
        float tau_ref = 0.0f;
        /* 轮扭矩平滑 */
        float tau_max_d = CTRL_TAU_RATE * dtf;
        float td = tau_ref - g_tau_cmd;
        td = clampf_sv(td, -tau_max_d, tau_max_d);
        g_tau_cmd += td;

        /* 接近竖直 且 撑地腿摆已到位 -> 进 SETTLE。
         * 关键: 必须在腿真正撑住后才切, 否则撬到一半就撤腿又倒回。 */
        float swing_target = (g_dir > 0) ? CTRL_SWING_FWD : CTRL_SWING_BACK;
        if (pitch_deg < CTRL_SETTLE_ENTER_DEG &&
            fabsf(swing_target - g_swing_cmd) < 0.05f) {
            g_st = ST_SETTLE;
            g_st_t = 0.0f;
            g_settle_t = 0.0f;
            printf("[SV] SETTLE pitch=%+.1f°\n", pitch_deg);
        }
        /* 超时保护 */
        if (g_st_t > CTRL_PUSH_TIMEOUT) {
            g_st = ST_ABORT;
            g_st_t = 0.0f;
            g_tau_cmd = 0.0f;
            printf("[SV] ABORT(push timeout)\n");
        }
        break;
    }
    case ST_SETTLE: {
        /* 回正确认: 撑地撬到位后立即"摆动腿回名义 + 放腿",
         * 让车体自然落回平衡(用户机制: 收腿到最短, 腿一摆动就恢复)。
         * 实验: 后翻撑地到+14°后摆腿回0.23+放腿 -> 稳定-2.1°。 */
        /* SETTLE: 轮子交还三环 PID(平衡力由 PID 提供), 只控制腿摆回位。
         * 摆腿回位瞬间车失去撑地支点, 必须有 PID 轮扭矩稳住车体。 */
        g_tau_cmd = 0.0f;   /* 覆盖标志: 输出段用三环 wheel_tau */

        /* 立即恢复腿摆到名义 + 放腿(摆动腿动作) */
        float swing_home = CTRL_SWING_FWD;
        float max_sw = CTRL_SWING_RATE * dtf;
        float dsw = swing_home - g_swing_cmd;
        dsw = clampf_sv(dsw, -max_sw, max_sw);
        g_swing_cmd += dsw;
        float max_ld = CTRL_LEN_RATE * dtf;
        float dld = 0.0f - g_len_cmd;
        dld = clampf_sv(dld, -max_ld, max_ld);
        g_len_cmd += dld;

        /* 腿摆回名义且姿态稳定 -> 交 PID */
        if (fabsf(swing_home - g_swing_cmd) < 0.05f &&
            pitch_deg < CTRL_SETTLE_EXIT_DEG &&
            fabsf(pitch_rate) < CTRL_SETTLE_RATE &&
            roll_deg < 10.0f) {
            g_settle_t += dtf;
            if (g_settle_t > CTRL_SETTLE_HOLD) {
                g_st = ST_IDLE;
                g_st_t = 0.0f;
                g_dir = 0;
                g_swing_cmd = 0.0f;
                g_tau_cmd = 0.0f;
                printf("[SV] RECOVERED pitch=%+.1f°\n", pitch_deg);
            }
        } else {
            g_settle_t = 0.0f;
        }
        /* 又翻回去 -> 回 PUSH 继续(方向自适应) */
        if (pitch_deg > CTRL_SETTLE_BACK_DEG) {
            g_st = ST_PUSH;
            g_st_t = 0.0f;
            printf("[SV] back to PUSH\n");
        }
        if (g_st_t > CTRL_SETTLE_TIMEOUT) {
            g_st = ST_ABORT;
            g_st_t = 0.0f;
            g_tau_cmd = 0.0f;
            printf("[SV] ABORT(settle timeout)\n");
        }
        break;
    }
    default: /* ST_ABORT: 撤力回 IDLE, 交 PID 兜底 */
        g_tau_cmd *= 0.90f;
        g_swing_cmd *= 0.90f;
        if (g_st_t > 0.5f) {
            g_st = ST_IDLE;
            g_st_t = 0.0f;
            g_dir = 0;
            g_swing_cmd = 0.0f;
            g_len_cmd = 0.0f;
            g_tau_cmd = 0.0f;
        }
        break;
    }

    /* ================= 输出 ================= */
    if (g_st == ST_IDLE) {
        /* 正常平衡: PID 基线, 可叠加 LQR */
        /* 腿长不再是 LQR 启用门控。进入姿态窗口后，所有腿长都使用
         * 对应的线性化段；短腿/长腿只影响参数选择和段间插值。 */
        if (lqr_ready && lqr_controller_model_valid() &&
            pitch_deg < CTRL_LQR_MAX_DEG && roll_deg < 5.0f) {
#if !CTRL_LQR_FULL_ENABLE
            ControlCmd l = {0};
            lqr_controller_update(s, dt, &l);
            /* 叠加可信通道:
             *   wheel_tau/leg_swing : pitch 子系统(7×3), R² 0.92~1.0
             *   leg_len_diff        : roll 子系统差动腿长(roll R²=1.0,
             *                         leg_len_diff R²≈1.0), 与三环 roll
             *                         稳定同向, 增强找平, 限幅 ±0.02
             * 不叠加:
             *   yaw_tau : 依赖 wheel_diff_v(R²≈0.5), 叠加会破坏三环 yaw 环
             *   leg_len : 腿长扰动会经轮腿耦合间接扰动 yaw/速度
             */
            out->wheel_tau    += CTRL_LQR_MIX_GAIN * l.wheel_tau;
            out->leg_swing    += CTRL_LQR_MIX_GAIN * l.leg_swing;
            out->leg_len_diff += CTRL_LQR_MIX_GAIN * l.leg_len_diff;
#endif
        }
        out->active = 1;

#if CTRL_LQR_FULL_ENABLE
        /* ---- LQR 独占门控 + 无扰切换 ----
         * 窗口: 相对目标俯仰和侧倾均进入小角度区间；腿长只用于选段。
         *
         * 切换逻辑(用户强调):
         *  1) 用 full_mix 做 0->1 / 1->0 的斜率过渡(全接管 0.4s),
         *     切换瞬间输出是"三环↔LQR"的加权混合, 不会阶跃(否则轮扭矩
         *     和腿摆跳变会把车顶翻);
         *  2) 进窗前提是已经初始化(LQR 状态滤波器在窗口内才被喂数,
         *     出窗后不再更新, 避免用陈旧状态乱输出);
         *  3) 出窗立即把 full_mix 拉回 0(速率同上), 三环 PID 持续在算,
         *     随时可无扰接管。 */
        {
            float enter = CTRL_LQR_FULL_PITCH_DEG;
            float exit = 1.5f * enter;
            /* 门控用"相对目标俯仰的偏差"而不是绝对 pitch:
             * 行驶/转向时 neutral 本身就是 3° 级的前倾驱动角, 用绝对 pitch
             * 会把正常工况误判成超限, LQR 永远进不来(实测按 W 巡航
             * |pitch|=3.0° > 2° 门限 -> FULL=1 与基线完全一致)。 */
            float pitch_err_deg = fabsf((float)(s->pitch - balance_get_neutral()))
                                  * 57.29578f;
            /* 姿态达到小角度窗口时，任何腿长都允许 LQR 接管；腿长
             * 仅用于在分段模型间插值参数，不作为接管门槛。 */
            int model_ready = (g_lqr_enabled && (CTRL_LQR_P3_ENABLE || lqr_controller_model_valid()) &&
                               s->sim_time > 1.5);
            if (g_full_on) {
                if (!model_ready || pitch_err_deg > exit || roll_deg > 4.0f)
                    g_full_on = 0;
            } else if (model_ready && pitch_err_deg < enter && roll_deg < 3.0f) {
                g_full_on = 1;
            }
            /* 接管失败保护: 全接管后一旦姿态跑到危险区, 硬切回 PID
             * (不走斜率), 并把 LQR 输出清零, 避免带着错误模型摔车。 */
            if (g_full_mix > 0.0f && (pitch_err_deg > 5.0f || roll_deg > 6.0f)) {
                g_full_on = 0;
                g_full_mix = 0.0f;
            }
            /* full_mix 斜率: 2.5/s => 0.4s 完成接管/退出 */
            float dmix = 2.5f * (float)dt;
            if (g_full_on) g_full_mix = (g_full_mix + dmix > 1.0f) ? 1.0f : g_full_mix + dmix;
            else           g_full_mix = (g_full_mix - dmix < 0.0f) ? 0.0f : g_full_mix - dmix;
            if (g_full_mix <= 0.0f) lqr_controller_set_v_ref(0.0f, 0);
            if (g_full_mix > 0.0f) {
#if CTRL_LQR_P3_ENABLE
                /* 模态基 3 状态接管: 只接管共模轮扭矩, K 只乘 pitch/pitch_rate/wheel_v */
                float lqr_wheel = 0.0f;
                /* 运动接管: 把 LQR 的 wheel_v 参考设成手动速度指令, LQR 同时
                 * 管姿态与速度(窗口内三环的手动腿摆速度环让位)。
                 * 符号: 状态 wheel_v = 0.5(lw+rw)R, 前进(+x)时 lw/rw 为负,
                 * 故 v_ref_state = -manual_v(反了会变成"按 W 后退")。 */
                lqr_controller_set_v_ref(-(float)balance_get_manual_v_target() *
                                         (1.0f - 0.65f * balance_get_slip_blend()),
                                         fabsf(balance_get_manual_v_target()) >= 0.05f ? 1 : 0);
                lqr_controller_update_analytic(s, dt, &lqr_wheel);
                /* 3-state analytic controller owns longitudinal balance and speed.
                 * Keep PID yaw for now: the accepted LQR roll model has no identified
                 * yaw authority, so claiming A/D ownership would silently do nothing. */
                out->wheel_tau = (1.0f - g_full_mix) * out->wheel_tau + g_full_mix * lqr_wheel;
                out->active = 1;
#else
                ControlCmd lf = {0};
                /* Keep a zero speed reference after release: the identified
                 * wheel row alone has too little velocity authority to park. */
                float manual_v_ref = balance_get_manual_v() *
                                      (1.0f - 0.65f * balance_get_slip_blend());
                /* The identified state is raw wheel speed. A body-speed
                 * request must include axle motion from the legs and pitch. */
                lqr_controller_set_v_ref(-manual_v_ref - (float)s->leg_kin_v
                                                       - (float)s->pitch_kin_v, 1);
                float swing_ref = out->leg_swing;
                float freeze = clampf_sv(
                    (SHORT_LEG_SW_FREEZE_RELEASE - (float)s->leg_len) /
                    (SHORT_LEG_SW_FREEZE_RELEASE - SHORT_LEG_SW_FREEZE_FULL),
                    0.0f, 1.0f);
                if (fabsf(manual_v_ref) < 0.05f &&
                    !balance_get_manual_spin() &&
                    fabsf((float)s->est_v) < 0.4f &&
                    !s->airborne && !s->impact && !s->slip) {
                    g_full_park_trim += CTRL_LQR_PARK_SWING_KI *
                        (float)s->est_v * dtf * g_full_mix;
                    g_full_park_trim = clampf_sv(g_full_park_trim,
                        -CTRL_LQR_PARK_TRIM_MAX, CTRL_LQR_PARK_TRIM_MAX);
                }
                if (!balance_get_manual_active()) {
                    float correction = CTRL_LQR_PARK_SWING_KP *
                        clampf_sv((float)s->est_v, -SIMPLE_MANUAL_SWING_V_ERR_MAX,
                                                   SIMPLE_MANUAL_SWING_V_ERR_MAX);
                    float park_ref = lqr_controller_trim_swing((float)s->leg_len) + correction;
                    swing_ref += (1.0f - freeze) * (park_ref - swing_ref);
                }
                swing_ref += (1.0f - freeze) * g_full_park_trim;
                if (g_full_mix <= dmix)
                    g_full_swing_ref = out->leg_swing;
                float swing_step = SIMPLE_MANUAL_SWING_RATE * dtf / 0.001f;
                if (!balance_get_manual_active())
                    swing_step = CTRL_LQR_PARK_SWING_RATE * dtf;
                g_full_swing_ref += clampf_sv(swing_ref - g_full_swing_ref,
                                              -swing_step, swing_step);
                lqr_controller_set_pose_ref(balance_get_neutral(), g_full_swing_ref,
                                            (float)s->leg_extend, (float)s->leg_side_extend);
                lqr_controller_set_yaw_ref(balance_get_manual_yaw());
                lqr_controller_update_full(s, dt, &lf);
                /* K 是满矩阵: 必须 pitch 三通道(轮扭矩/腿摆/腿长)+
                 * roll 两通道(偏航/腿差)整体接管。只接管子集时, 未接管
                 * 通道的稳态偏差会经 K 的耦合列污染已接管输出(实测只接管
                 * 轮扭矩+腿摆时被腿长 x_ss 偏差带跑 8.5m)。 */
                float a = g_full_mix;
                /* FULL 模式下五个通道统一由同一组分段 LQR 输出，避免
                 * 轮子由 LQR、腿摆/腿长却仍由 PID 互相打架。 */
                out->wheel_tau = (1.0f - a) * out->wheel_tau + a * lf.wheel_tau;
                out->yaw_tau = (1.0f - a) * out->yaw_tau + a * lf.yaw_tau;
                out->leg_swing = (1.0f - a) * out->leg_swing + a * lf.leg_swing;
                /* 运行层把这两个字段叠加到 PID 腿长参考，因此先换算为
                 * 增量；a=1 时最终伺服参考恰好等于 LQR 的绝对参考。 */
                out->leg_len = (1.0f - a) * out->leg_len
                             + a * (lf.leg_len - (float)s->leg_extend);
                out->leg_len_diff = (1.0f - a) * out->leg_len_diff
                                  + a * (lf.leg_len_diff - (float)s->leg_side_extend);
                out->leg_range_full = 1;
                out->active = 1;
#endif
            }
        }
#endif
    } else if (g_st == ST_ABORT) {
        /* 自救放弃: 只保留 PID 基线(不覆盖 out), 轮扭矩由 PID 控制 */
        out->active = 1;
    } else {
        /* PREP/PUSH/SETTLE: 自救接管。
         * - wheel_tau: 清 0(不干扰撑地撬回; SETTLE 内可用小阻尼)
         * - leg_swing/leg_len: 状态机给出(撑地/收腿/摆回)
         * - leg_len_diff: 保留三环 roll 稳定输出(不覆盖, 防止侧翻)
         */
        /* SETTLE 阶段轮子交还三环 PID(不覆盖 out->wheel_tau) */
        if (g_st != ST_SETTLE)
            out->wheel_tau = g_tau_cmd;
        out->yaw_tau      = 0.0f;
        out->leg_swing    = g_swing_cmd;
        out->leg_len      = g_len_cmd;
        /* leg_len_diff 不覆盖: 三环 roll 稳定继续工作 */
        out->active       = 1;
    }

    /* 调试输出 */
#if CTRL_DEBUG
    {
        static double last_t = -1.0;
        if (last_t < 0.0 || s->sim_time - last_t >= 0.5) {
            last_t = s->sim_time;
            const char *st_names[] = {"IDLE", "PREP", "PUSH", "SETTLE", "ABORT"};
            printf("[SV] t=%.2f st=%s dir=%+d pitch=%+.1f° roll=%+.1f° "
                   "sw=%+.2f tau=%+.1f\n",
                   s->sim_time, st_names[g_st], g_dir, pitch_deg, roll_deg,
                   g_swing_cmd, g_tau_cmd);
        }
    }
#endif
}

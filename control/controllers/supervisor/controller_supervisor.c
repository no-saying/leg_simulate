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
    g_ready = 1;
    lqr_controller_init();
    energy_controller_init();
}

void controller_supervisor_set_rl_busy(int on)
{
    g_rl_busy = on ? 1 : 0;
}

CtrlMode controller_supervisor_mode(void)
{
    if (g_st == ST_IDLE) {
        return CTRL_MODE_PID;   /* 正常平衡阶段由 update 内细分 */
    }
    return CTRL_MODE_ENERGY;
}

int controller_supervisor_phase(void)
{
    return g_st;
}

void controller_supervisor_update(const SensorState *s, double dt, ControlCmd *out)
{
    if (!g_ready) controller_supervisor_init();

    float pitch = (float)s->pitch;
    float pitch_deg = (float)(fabs(pitch) * 57.29578f);
    float roll_deg = (float)(fabs(s->roll) * 57.29578f);
    float pitch_rate = (float)s->gyro[1];
    float leg_len = (float)s->leg_len;
    int lqr_ready = (s->sim_time > CTRL_LQR_STARTUP_DELAY);
    float dtf = (float)dt;

    g_st_t += dtf;

    /* 跳跃/自救期间状态机不介入: 跳跃会大幅改变 pitch 和腿长,
     * 正常行驶的加速前倾也可能超过进入阈值, 不能误判为翻倒。
     * balance_get_jump_phase() 1=下蹲 2=蹬伸 3=空中 4=落地。 */
    int jump_busy = g_rl_busy
                    || (balance_get_jump_phase() != 0) || balance_get_self_right_active()
                    || rl_jump_controller_active();

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
        if (lqr_ready &&
            pitch_deg < CTRL_LQR_MAX_DEG && roll_deg < 5.0f &&
            leg_len >= CTRL_LQR_LEG_LEN_MIN && leg_len <= CTRL_LQR_LEG_LEN_MAX) {
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
        }
        out->active = 1;
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

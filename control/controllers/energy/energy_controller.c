/*
 * energy_controller.c - 能量整形控制器实现
 * ================================================================
 * 实现见头文件注释。参数(质量/惯量/重心高)来自模型标定, 可在
 * control/include/config_controllers.h 中调整。
 */
#include "energy_controller.h"

#include <math.h>

#include "control_config.h"
#include "config_controllers.h"

/* 模型标定常数(与 scripts/auto_tune.py 一致) */

static float g_dE = 0.0f;
static float g_tau = 0.0f;
static float g_swing_cmd = 0.0f;  /* 腿摆平滑输出, 避免退出时跳变 */
static int g_pitch_dir = 0;       /* 0=无, +1=前翻, -1=后翻(滞回记忆) */
static int g_ready = 0;

void energy_controller_init(void)
{
    g_dE = 0.0f;
    g_tau = 0.0f;
    g_swing_cmd = 0.0f;
    g_pitch_dir = 0;
    g_ready = 1;
}

float energy_controller_delta(void)
{
    return g_dE;
}

void energy_controller_update(const SensorState *s, double dt, ControlCmd *out)
{
    if (!g_ready) energy_controller_init();

    float pitch = (float)s->pitch;
    float pitch_rate = (float)s->gyro[1];

    /* ---- 前翻/后翻腿摆自救 ----
     * 用户实测: 前翻(pitch 大正, 车头着地)时腿保持名义摆角(sw≈0.23)撑地,
     * 车体靠腿支撑自然回正; 后翻(pitch 大负, 车尾着地)时腿摆需转到大负角
     * (sw≈-0.6~-1.0)让腿在后方撑地, 配合轮扭矩把车推回竖直。
     * 这里只输出 leg_swing 目标, 轮扭矩仍由能量整形给出。 */
    /* 前翻/后翻腿摆自救(带滞回): 一旦进入大角度姿态, 腿摆保持撑地
     * 直到回到平衡区, 避免在阈值边界抖动导致腿摆来回甩。 */
    if (pitch > 0.35f)      g_pitch_dir =  1;   /* 前翻: 腿前撑地 */
    else if (pitch < -0.20f) g_pitch_dir = -1;  /* 后翻: 阈值放宽, 早介入 */
    else if (fabsf(pitch) < 0.14f) g_pitch_dir = 0;  /* 接近竖直(±8°)释放腿 */

    float swing_ref;
    if (g_pitch_dir > 0)      swing_ref = 0.23f;   /* 前翻: 名义腿摆 */
    else if (g_pitch_dir < 0) swing_ref = -1.0f;   /* 后翻: 极限 */
    else                      swing_ref = 0.0f;
    /* 腿摆变化率限制: 撑地/收回时都平滑过渡, 避免瞬态扰动 */
    float max_swing_delta = (float)(2.0 * dt);
    float swing_delta = swing_ref - g_swing_cmd;
    if (swing_delta >  max_swing_delta) swing_delta =  max_swing_delta;
    if (swing_delta < -max_swing_delta) swing_delta = -max_swing_delta;
    g_swing_cmd += swing_delta;
    out->leg_swing = g_swing_cmd;

    /* 机械能量: 转动 + 重力势能(相对竖直平衡点) */
    float E_kin = 0.5f * ENERGY_PITCH_INERTIA * pitch_rate * pitch_rate;
    float E_pot = ENERGY_MASS * ENERGY_G * ENERGY_COM_H * (cosf(pitch) - 1.0f);
    float E = E_kin + E_pot;
    float E0 = 0.0f;   /* 竖直静止平衡点能量 */
    g_dE = E - E0;

    /* 能量控制律: 能量不足(下摆)注入, 能量过高(上摆)耗散。
     * 前翻/后翻大角度时额外叠加固定方向的强扭矩(腿已撑地,
     * 轮子滚动把车体翻回竖直), 实测最后 10° 单靠能量差扭矩太弱。 */
    float sgn = (pitch_rate * cosf(pitch) >= 0.0f) ? 1.0f : -1.0f;
    float tau_ref = ENERGY_KP * g_dE * sgn;
    /* 只有后翻叠加固定推回扭矩(前翻纯能量已验证稳定):
     * 后翻时腿摆-1.0 撑地, 轮子向后滚(负 tau)把车尾抬回竖直。 */
    if (g_pitch_dir < 0 && pitch < -0.10f) {
        /* 后翻固定推回扭矩: 保持到 |pitch|<0.10(约5.7°) 才衰减,
         * 让轮子把车真正推过竖直, 否则腿撑地只能到 -10° 附近。 */
        float back_scale = (fabsf(pitch) - 0.10f) / (0.45f - 0.10f);
        back_scale = (back_scale > 1.0f) ? 1.0f : ((back_scale < 0.0f) ? 0.0f : back_scale);
        tau_ref -= 28.0f * back_scale;
    }
    if (tau_ref >  ENERGY_TAU_MAX) tau_ref =  ENERGY_TAU_MAX;
    if (tau_ref < -ENERGY_TAU_MAX) tau_ref = -ENERGY_TAU_MAX;

    /* 变化率限幅 */
    float max_delta = (float)(ENERGY_RATE_MAX * dt);
    float delta = tau_ref - g_tau;
    if (delta >  max_delta) delta =  max_delta;
    if (delta < -max_delta) delta = -max_delta;
    g_tau += delta;

    out->wheel_tau    = g_tau;
    out->yaw_tau      = 0.0f;
    out->leg_len      = 0.0f;
    out->leg_len_diff = 0.0f;
    out->active       = 1;
}

/*
 * app_keys.c - 应用层: 键盘全局状态与 GLFW 按键回调
 * ================================================================
 * 从原 controller.c 拆出:
 *   - 全局按键状态(g_key_*) 在此定义, 供 GUI 手动指令和行车记录仪读取。
 *   - keyboard_cb 是 GLFW 按键回调: B=平衡开关, L=锁位, V=切换视角,
 *     R=复位, SPACE=跳跃, H=自救, WASD=手动驾驶, E/Q=手动腿长。
 *   - manual_keys_state 返回当前按住的按键串, 供 dashcam 记录。
 */
#include "app_internal.h"

#include <stdio.h>
#include <string.h>

#include "balance.h"
#include "dashcam.h"

/* ---- 全局按键状态定义 ---- */
int g_key_balance = 0;
int g_key_lock = 0;
int g_key_view = 0;
int g_key_reset = 0;
int g_key_w = 0, g_key_a = 0, g_key_s = 0, g_key_d = 0;
int g_key_extend = 0, g_key_shorten = 0;   /* E=伸长腿/升高, Q=缩短腿/降低 */
int g_manual_prev_active = 0;
float g_manual_leg_target = 0.0f;          /* 手动腿长保持目标，松开不回弹 */

double g_dash_sim_t = 0.0;

const char *manual_keys_state(void)
{
    static char buf[16];
    buf[0] = '\0';
    if (g_key_w) strcat(buf, "W");
    if (g_key_a) strcat(buf, "A");
    if (g_key_s) strcat(buf, "S");
    if (g_key_d) strcat(buf, "D");
    if (g_key_extend) strcat(buf, "E");
    if (g_key_shorten) strcat(buf, "Q");
    return buf[0] ? buf : "-";
}

void keyboard_cb(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    (void)scancode;
    (void)mods;

    int pressed = (action == GLFW_PRESS);
    int released = (action == GLFW_RELEASE);
    double wall_t = glfwGetTime();

    if (pressed) {
        if (key == GLFW_KEY_B) g_key_balance = 1;
        if (key == GLFW_KEY_L) g_key_lock = 1;
        if (key == GLFW_KEY_V) g_key_view = 1;
        if (key == GLFW_KEY_R) g_key_reset = 1;
        if (key == GLFW_KEY_SPACE) balance_trigger_jump();
        if (key == GLFW_KEY_H) balance_trigger_self_right();
        if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window, GLFW_TRUE);
        if (key == GLFW_KEY_B) dashcam_log_event(g_dash_sim_t, "B", "PRESS", wall_t);
        if (key == GLFW_KEY_L) dashcam_log_event(g_dash_sim_t, "L", "PRESS", wall_t);
        if (key == GLFW_KEY_V) dashcam_log_event(g_dash_sim_t, "V", "PRESS", wall_t);
        if (key == GLFW_KEY_R) dashcam_log_event(g_dash_sim_t, "R", "PRESS", wall_t);
        if (key == GLFW_KEY_SPACE) dashcam_log_event(g_dash_sim_t, "J", "PRESS", wall_t);
        if (key == GLFW_KEY_H) dashcam_log_event(g_dash_sim_t, "H", "PRESS", wall_t);
        {
            static int dbg_keys = 0;
            if (dbg_keys < 10) {
                dbg_keys++;
                printf("[Key] code=%d action=%d\n", key, action);
            }
        }
    }

    /* WASD 手动驾驶 + E/Q 手动腿长: 同时记录按下/松开，支持组合按键。 */
    if (pressed || released) {
        if (key == GLFW_KEY_W) g_key_w = pressed;
        if (key == GLFW_KEY_A) g_key_a = pressed;
        if (key == GLFW_KEY_S) g_key_s = pressed;
        if (key == GLFW_KEY_D) g_key_d = pressed;
        if (key == GLFW_KEY_E) g_key_extend = pressed;
        if (key == GLFW_KEY_Q) g_key_shorten = pressed;
        if (key == GLFW_KEY_W) dashcam_log_event(g_dash_sim_t, "W", pressed ? "PRESS" : "RELEASE", wall_t);
        if (key == GLFW_KEY_A) dashcam_log_event(g_dash_sim_t, "A", pressed ? "PRESS" : "RELEASE", wall_t);
        if (key == GLFW_KEY_S) dashcam_log_event(g_dash_sim_t, "S", pressed ? "PRESS" : "RELEASE", wall_t);
        if (key == GLFW_KEY_D) dashcam_log_event(g_dash_sim_t, "D", pressed ? "PRESS" : "RELEASE", wall_t);
        if (key == GLFW_KEY_E) dashcam_log_event(g_dash_sim_t, "E", pressed ? "PRESS" : "RELEASE", wall_t);
        if (key == GLFW_KEY_Q) dashcam_log_event(g_dash_sim_t, "Q", pressed ? "PRESS" : "RELEASE", wall_t);
    }
}

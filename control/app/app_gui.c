/*
 * app_gui.c - 应用层: 图形界面(GUI)
 * ================================================================
 * 从原 controller.c 拆出，职责:
 *   1. GLFW 窗口创建/销毁, MuJoCo 渲染上下文。
 *   2. 相机预设: 跟随/俯视/车后/侧面/对肩/车后向前。
 *   3. 每帧处理按键、生成手动驾驶指令、渲染场景与 HUD 覆盖文本。
 *
 * 键盘状态与回调在 app_keys.c, 仿真步进在 app_sim.c。
 */
#include "app_internal.h"

#include <stdio.h>
#include <string.h>

#include "balance.h"
#include "control_config.h"
#include "dashcam.h"
#include "telemetry.h"

void keyboard_cb(GLFWwindow *window, int key, int scancode, int action, int mods);

static void apply_camera_preset(GuiRuntime *gui);

static int gui_init(GuiRuntime *gui, const mjModel *model)
{
    memset(gui, 0, sizeof(*gui));

    fprintf(stderr, "glfwInit...\n");
    if (!glfwInit()) return 0;
    fprintf(stderr, "glfwInit OK\n");

    fprintf(stderr, "glfwCreateWindow...\n");
    gui->window = glfwCreateWindow(1200, 900, "COD-2026", NULL, NULL);
    if (!gui->window) {
        fprintf(stderr, "Window failed\n");
        glfwTerminate();
        return 0;
    }
    fprintf(stderr, "Window OK\n");

    glfwMakeContextCurrent(gui->window);
    glfwSetKeyCallback(gui->window, keyboard_cb);
    glfwSwapInterval(0);

    mjv_defaultCamera(&gui->camera);
    gui->camera.type = mjCAMERA_FREE;
    gui->camera.distance = 1.5;
    gui->camera.azimuth = 160;
    gui->camera.elevation = -15;
    gui->camera.lookat[0] = 0.0;
    gui->camera.lookat[1] = 0.0;
    gui->camera.lookat[2] = 0.15;
    gui->camera_preset = 0;
    {
        const char *cp = getenv("CAM_PRESET");
        if (cp) {
            int p = atoi(cp);
            if (p >= 0 && p < 6) gui->camera_preset = p;
        }
    }
    gui->track_body_id = mj_name2id(model, mjOBJ_BODY, "base");
    if (gui->track_body_id < 0) gui->track_body_id = 0;
    apply_camera_preset(gui);

    mjv_defaultOption(&gui->option);
    mjv_defaultScene(&gui->scene);
    mjv_makeScene(model, &gui->scene, 5000);
    mjr_defaultContext(&gui->context);
    mjr_makeContext(model, &gui->context, 200);

    trail_reset(&gui->trail);
    gui->sim_start_wall_time = glfwGetTime();
    gui->last_render_wall_time = gui->sim_start_wall_time;
    gui->last_log_sim_time = 0.0;
    return 1;
}

static void gui_destroy(GuiRuntime *gui)
{
    mjr_freeContext(&gui->context);
    mjv_freeScene(&gui->scene);
    if (gui->window) glfwDestroyWindow(gui->window);
    glfwTerminate();
    memset(gui, 0, sizeof(*gui));
}

static void handle_gui_keys(GuiRuntime *gui, Simulation *sim)
{
    if (g_key_reset) {
        g_key_reset = 0;
        simulation_reset(sim);
    }
    if (g_key_balance) {
        g_key_balance = 0;
        balance_toggle();
        printf("[Balance] %s\n", balance_active() ? "ON" : "OFF");
    }
    if (g_key_lock) {
        g_key_lock = 0;
        gui->body_lock = !gui->body_lock;
        printf("[Lock] %s\n", gui->body_lock ? "ON" : "OFF");
    }
    if (g_key_view) {
        g_key_view = 0;
        gui->camera_preset = (gui->camera_preset + 1) % 6;
        const char *names[] = {"FOLLOW", "TOP", "CHASE", "SIDE", "SHOULDER", "BEHIND-FWD"};
        printf("[View] %s\n", names[gui->camera_preset]);
        apply_camera_preset(gui);
    }
}

static void apply_camera_preset(GuiRuntime *gui)
{
    mjvCamera *cam = &gui->camera;
    cam->trackbodyid = gui->track_body_id;
    switch (gui->camera_preset) {
    case 0: /* 跟随第三人称 */
        cam->type = mjCAMERA_TRACKING;
        cam->distance = 1.5f;
        cam->azimuth = 160.0f;
        cam->elevation = -15.0f;
        break;
    case 1: /* 俯视 */
        cam->type = mjCAMERA_FREE;
        cam->trackbodyid = -1;
        cam->distance = 4.0f;
        cam->azimuth = 0.0f;
        cam->elevation = -80.0f;
        break;
    case 2: /* 车后跟随 */
        cam->type = mjCAMERA_TRACKING;
        cam->distance = 1.6f;
        cam->azimuth = 0.0f;
        cam->elevation = -10.0f;
        break;
    case 3: /* 侧面近景 */
        cam->type = mjCAMERA_TRACKING;
        cam->distance = 1.1f;
        cam->azimuth = 90.0f;
        cam->elevation = -8.0f;
        break;
    case 4: /* 对肩视角: 车后偏侧、略高，跟随车头朝向 */
        cam->type = mjCAMERA_TRACKING;
        cam->distance = 1.3f;
        cam->azimuth = 150.0f;
        cam->elevation = -12.0f;
        break;
    default: /* 车后向前看: 相机在车尾，视线指向车前方 */
        cam->type = mjCAMERA_FREE;
        cam->trackbodyid = -1;
        cam->distance = 1.6f;
        cam->azimuth = 0.0f;
        cam->elevation = -8.0f;
        break;
    }
}

static void update_manual_command(GLFWwindow *window)
{
    /* 回调 + 状态轮询双保险: 部分窗口管理器/远程显示下回调收不到按键，
     * 轮询 glfwGetKey 只要窗口有输入焦点就能读到键状态。 */
    if (window) {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) g_key_w = 1;
        else if (glfwGetKey(window, GLFW_KEY_W) == GLFW_RELEASE) g_key_w = 0;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) g_key_a = 1;
        else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_RELEASE) g_key_a = 0;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) g_key_s = 1;
        else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_RELEASE) g_key_s = 0;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) g_key_d = 1;
        else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_RELEASE) g_key_d = 0;
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) g_key_extend = 1;
        else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_RELEASE) g_key_extend = 0;
        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) g_key_shorten = 1;
        else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_RELEASE) g_key_shorten = 0;
        if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) g_key_spin = 1;
        else if (glfwGetKey(window, GLFW_KEY_T) == GLFW_RELEASE) g_key_spin = 0;
        {
            static int g_space_prev = 0;
            int space_now = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS);
            if (space_now && !g_space_prev) balance_trigger_jump();
            g_space_prev = space_now;
            static int g_h_prev = 0;
            int h_now = (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS);
            if (h_now && !g_h_prev) balance_trigger_self_right();
            g_h_prev = h_now;
        }
    }
    float speed = 0.0f;
    float yaw_rate = 0.0f;
    if (g_key_spin) {
        /* 原地小陀螺: 不前进, 恒定偏航角速度自转。
         * 方向沿用 A/D 约定: 正=左转, 负=右转。 */
        yaw_rate = SIMPLE_MANUAL_SPIN_YAW_RATE;
    } else {
        if (g_key_w) speed += SIMPLE_MANUAL_V_MAX;
        if (g_key_s) speed -= SIMPLE_MANUAL_V_MAX;
        /* 平衡核心内 manual_yaw<0 -> 右转(实测头less 'r'=-0.5 转 yaw 负)，
         * 所以 A 左转给正、D 右转给负。 */
        if (g_key_a) yaw_rate += SIMPLE_MANUAL_YAW_RATE;
        if (g_key_d) yaw_rate -= SIMPLE_MANUAL_YAW_RATE;
    }

    int active = (g_key_w || g_key_a || g_key_s || g_key_d || g_key_spin);
    if (active != g_manual_prev_active) {
        g_manual_prev_active = active;
        printf("[Manual] %s speed=%+.2f yaw=%+.2f\n",
               active ? "ON" : "OFF", speed, yaw_rate);
    }
    balance_set_manual(speed, yaw_rate, active);

#if MANUAL_LEG_EXTEND_ENABLE
    /* E=伸长腿(车体升高) -> leg_extend 负; Q=缩短腿(车体降低) -> leg_extend 正。
     * 按住持续调节(带变化率限幅)，松开保持当前高度，不回弹。 */
    static double g_last_ext_t = -1.0;
    double now_ext = glfwGetTime();
    double dt_ext = (g_last_ext_t > 0.0) ? (now_ext - g_last_ext_t) : 0.016;
    g_last_ext_t = now_ext;
    float ext_step = MANUAL_LEG_EXTEND_RATE * (float)dt_ext;
    if (g_key_extend) g_manual_leg_target -= ext_step;
    if (g_key_shorten) g_manual_leg_target += ext_step;
    if (g_manual_leg_target >  MANUAL_LEG_EXTEND_MAX) g_manual_leg_target =  MANUAL_LEG_EXTEND_MAX;
    if (g_manual_leg_target < -MANUAL_LEG_EXTEND_MAX) g_manual_leg_target = -MANUAL_LEG_EXTEND_MAX;
    balance_set_leg_extend_manual(g_manual_leg_target);
#endif
}

static void gui_log_if_due(GuiRuntime *gui, const SensorState *s)
{
    if (s->sim_time - gui->last_log_sim_time < 0.05)
        return;

    gui->last_log_sim_time = s->sim_time;
    trail_push(&gui->trail, s->sim_time,
               (float)s->odom_x, (float)s->odom_y,
               (float)s->body_pos[0], (float)s->body_pos[1]);

    double now = glfwGetTime();
    double fps = 1.0 / (now - gui->last_render_wall_time + 0.0001);
    telemetry_print_frame(s, fps);
    dashcam_log_row(s, now, manual_keys_state(),
                    balance_get_manual_active(), balance_get_hold_pos());
}

static int manual_active(void)
{
    return (g_key_w || g_key_a || g_key_s || g_key_d || g_key_spin);
}

static void gui_render_if_due(GuiRuntime *gui, const Simulation *sim)
{
    const double frame_interval = 1.0 / 60.0;
    double now = glfwGetTime();
    if (now - gui->last_render_wall_time < frame_interval)
        return;

    gui->last_render_wall_time = now;
    if (gui->camera_preset == 2 || gui->camera_preset == 4) {
        /* 车后/对肩跟随: 相机方位角每帧跟随机体偏航，保持镜头在车尾方向；
         * 对肩再加 30° 偏侧形成“过肩”构图。 */
        float side = (gui->camera_preset == 4) ? 30.0f : 0.0f;
        gui->camera.azimuth = 180.0f + side + (float)(sim->state.yaw * 57.29578);
    }
    if (gui->camera_preset == 5) {
        /* 车后向前看: 用四元数算车头朝向(不依赖 yaw 符号约定)，
         * 相机放在车尾(-forward)上方，lookat 指向车前方 1.5m。 */
        const double *q = sim->state.quat;
        float fx = (float)(1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        float fy = (float)(2.0 * (q[1] * q[2] + q[0] * q[3]));
        gui->camera.type = mjCAMERA_FREE;
        gui->camera.trackbodyid = -1;
        /* MuJoCo 相机位置 = lookat - dist*(cos az, sin az)，
         * 所以 az = atan2(fy, fx) 才把相机放在车尾。 */
        gui->camera.azimuth = (float)(atan2(fy, fx) * 57.29578);
        gui->camera.distance = 2.0f;
        gui->camera.elevation = -6.0f;
        gui->camera.lookat[0] = sim->state.body_pos[0] + fx * 0.5;
        gui->camera.lookat[1] = sim->state.body_pos[1] + fy * 0.5;
        gui->camera.lookat[2] = 0.15;
    }
    mjv_updateScene(sim->model, sim->data, &gui->option, NULL, &gui->camera,
                    mjCAT_ALL, &gui->scene);
    trail_add_scene(&gui->trail, &gui->scene);

    int vw, vh;
    glfwGetFramebufferSize(gui->window, &vw, &vh);
    mjrRect viewport = {0, 0, vw, vh};

    if (gui->camera_preset != 5) {
        gui->camera.lookat[0] = sim->state.body_pos[0];
        gui->camera.lookat[1] = sim->state.body_pos[1];
        gui->camera.lookat[2] = 0.10;
    }

    mjr_render(viewport, &gui->scene, &gui->context);

    char left_overlay[320];
    const char *view_names[] = {"FOLLOW", "TOP", "CHASE", "SIDE", "SHOULDER", "BEHIND-FWD"};
    snprintf(left_overlay, sizeof(left_overlay),
             "pit=%.1f  roll=%.1f  yaw=%.1f  ye=%.1f  %s %s %s\n"
             "ODOM  ox=%+.3f  oy=%+.3f  |path|=%.3f  v=%.2f\n"
             "TRUE  bx=%+.3f  by=%+.3f\n"
             "map: cyan=odom  blue=odom-now  magenta=true  red=true-now",
             sim->state.pitch * 57.3, sim->state.roll * 57.3,
             sim->state.yaw * 57.3,
             sim->state.yaw_error * 57.3,
             balance_active() ? "[BAL]" : "[--]",
             manual_active() ? "[MANUAL]" : "",
             balance_get_fallen() ? "[FALL] press R" : "",
             sim->state.odom_x, sim->state.odom_y,
             fabs(sim->state.est_x), sim->state.est_v,
             sim->state.body_pos[0], sim->state.body_pos[1]);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, left_overlay, NULL, &gui->context);

    char right_overlay[192];
    snprintf(right_overlay, sizeof(right_overlay),
             "2D ODOM\nox %+.3f m\noy %+.3f m\nxh %+.3f m\ntrail %d\nview %s",
             sim->state.odom_x, sim->state.odom_y,
             sim->state.est_x, trail_count(&gui->trail),
             view_names[gui->camera_preset]);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, right_overlay, NULL, &gui->context);

    glfwSwapBuffers(gui->window);
}

int run_gui(Simulation *sim)
{
    GuiRuntime gui;
    if (!gui_init(&gui, sim->model))
        return 1;
    dashcam_open();

    printf("COD-2026  |  dt=%.4f  nsensordata=%d\n",
           sim->model->opt.timestep, (int)sim->model->nsensordata);
    printf("B=balance  L=lock  V=view  R=reset  SPACE=jump  W/S=drive  A/D=turn  ESC=quit\n");

    while (!glfwWindowShouldClose(gui.window)) {
        glfwPollEvents();
        handle_gui_keys(&gui, sim);
        update_manual_command(gui.window);
        {
            static int last_focus = -1;
            int focused = glfwGetWindowAttrib(gui.window, GLFW_FOCUSED);
            if (focused != last_focus) {
                last_focus = focused;
                printf("[Focus] window %s\n", focused ? "focused" : "NOT focused");
            }
        }
        maybe_auto_enable_balance(sim);

        int phys_steps = 0;
        double elapsed = glfwGetTime() - gui.sim_start_wall_time;
        while (sim->state.sim_time < elapsed &&
               phys_steps < MAX_PHYS_STEPS_PER_FRAME) {
            if (!simulation_step(sim, gui.body_lock)) {
                gui_destroy(&gui);
                return 1;
            }
            g_dash_sim_t = sim->state.sim_time;
            dashcam_log_row(&sim->state, glfwGetTime(),
                            manual_keys_state(), balance_get_manual_active(),
                            balance_get_hold_pos());
            phys_steps++;
        }

        gui_log_if_due(&gui, &sim->state);
        gui_render_if_due(&gui, sim);
    }

    dashcam_close();
    gui_destroy(&gui);
    return 0;
}

/*
 * app_internal.h - 应用层(GUI/无界面)内部共享头文件
 * ================================================================
 * 由 app/controller.c 拆出的多个文件共享:
 *   - Simulation / GuiRuntime 结构体
 *   - 全局按键状态(跨文件可见, 供键盘回调和手动指令生成使用)
 *   - 各模块对外函数声明
 *
 * 职责边界:
 *   - app_sim.c     : 加载/销毁/复位 MuJoCo 模型与数据, 单步推进
 *   - app_gui.c     : GLFW 窗口、相机、渲染、按键处理
 *   - app_headless.c: 无界面测试钩子(MANUAL_TEST/JUMP_AT/ROLL_* 等)
 *   - controller.c  : main 入口, 按参数选择 GUI 或无界面模式
 */
#ifndef APP_INTERNAL_H
#define APP_INTERNAL_H

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "sensors.h"
#include "trail.h"

/* 应用层枚举: 执行器数量/自动平衡/每帧最大物理步数 */
enum {
    ACTUATOR_COUNT = 6,
    AUTO_BALANCE_STEP = 0,      /* 平衡从第 0 步开启(与训练环境一致, 防启动瞬态倒车) */
    MAX_PHYS_STEPS_PER_FRAME = 20,
};

/* 仿真对象: 模型 + 数据 + 当前传感器状态 */
typedef struct {
    mjModel *model;
    mjData *data;
    SensorState state;
    int auto_balance_done;
} Simulation;

/* GUI 运行状态: 相机/渲染/窗口/按键 */
typedef struct {
    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;
    OdomTrail trail;
    GLFWwindow *window;
    int body_lock;
    int camera_preset;      /* 0=跟随 1=俯视 2=车后 3=侧面 4=对肩 5=车后向前 */
    int track_body_id;      /* 跟踪/跟随视角的目标 body id */
    double sim_start_wall_time;
    double last_render_wall_time;
    double last_log_sim_time;
} GuiRuntime;

/* ---- 全局按键状态(controller.c 定义, 其余文件 extern) ---- */
extern int g_key_balance, g_key_lock, g_key_view, g_key_reset;
extern int g_key_w, g_key_a, g_key_s, g_key_d;
extern int g_key_extend, g_key_shorten;
extern int g_key_spin;            /* T 键: 原地小陀螺 */
extern int g_manual_prev_active;
extern float g_manual_leg_target;
extern double g_dash_sim_t;

/* ---- app_sim.c ---- */
int simulation_load(Simulation *sim, const char *xml_path);
void simulation_destroy(Simulation *sim);
void simulation_reset(Simulation *sim);
int simulation_step(Simulation *sim, int body_lock);
void maybe_auto_enable_balance(Simulation *sim);
void apply_open_loop_override(SensorState *s);

/* ---- app_gui.c ---- */
int run_gui(Simulation *sim);

/* ---- app_headless.c ---- */
int run_headless(Simulation *sim, double duration);

/* 当前按住的按键串(供行车记录仪/诊断显示使用) */
const char *manual_keys_state(void);

#endif /* APP_INTERNAL_H */

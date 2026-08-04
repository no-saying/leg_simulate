/*
 * app_sim.c - 应用层: 仿真模型加载/复位/单步推进
 * ================================================================
 * 从原 controller.c 拆出，职责:
 *   1. 加载 MJCF 模型并创建 mjData。
 *   2. 复位到初始位姿、清理平衡核心状态。
 *   3. 每个物理步: mj_step -> sensors_read -> robot_control_step ->
 *      写回执行器控制量。
 *   4. 开环轮测试钩子(OPEN_LOOP_WHEEL_TEST)与自动启用平衡。
 */
#include "app_internal.h"

#include <stdio.h>
#include <string.h>

#include "balance.h"
#include "robot_control.h"

int simulation_load(Simulation *sim, const char *xml_path)
{
    char err[1000] = "";
    memset(sim, 0, sizeof(*sim));

    sim->model = mj_loadXML(xml_path, NULL, err, sizeof(err));
    if (!sim->model) {
        fprintf(stderr, "ERROR: %s\n", err);
        return 0;
    }
    fprintf(stderr, "XML loaded\n");

    sim->data = mj_makeData(sim->model);
    if (!sim->data) {
        fprintf(stderr, "ERROR: mj_makeData failed\n");
        mj_deleteModel(sim->model);
        sim->model = NULL;
        return 0;
    }
    fprintf(stderr, "Data created\n");

    sensors_set_initial_pose(sim->model, sim->data);
    mj_forward(sim->model, sim->data);
    fprintf(stderr, "Init done\n");
    return 1;
}

void simulation_destroy(Simulation *sim)
{
    if (sim->data) mj_deleteData(sim->data);
    if (sim->model) mj_deleteModel(sim->model);
    memset(sim, 0, sizeof(*sim));
}

void maybe_auto_enable_balance(Simulation *sim)
{
#if !OPEN_LOOP_WHEEL_TEST
    if (!sim->auto_balance_done &&
        sim->state.step_count >= AUTO_BALANCE_STEP &&
        !balance_active()) {
        balance_toggle();
        sim->auto_balance_done = 1;
        printf("[Auto] Balance ON @ t=0.02s\n");
    }
#else
    (void)sim;
#endif
}

void simulation_reset(Simulation *sim)
{
    mjData *d = sim->data;
    d->qpos[0] = 0.0;
    d->qpos[1] = 0.0;
    d->qpos[2] = 0.5;
    d->qpos[3] = 1.0;
    d->qpos[4] = 0.0;
    d->qpos[5] = 0.0;
    d->qpos[6] = 0.0;
    for (int i = 0; i < 6; i++) d->qvel[i] = 0.0;
    mj_forward(sim->model, sim->data);

    sim->state.step_count = 0;
    sim->state.sim_time = 0.0;
    sim->auto_balance_done = 0;
    memset(&sim->state.ctrl, 0, sizeof(sim->state.ctrl));

    /* 平衡核心状态复位: 关/开一次，清空里程计、接触滞回和侧翻标志。 */
    if (balance_active()) balance_toggle();
    balance_toggle();
    g_key_w = g_key_a = g_key_s = g_key_d = 0;
    g_key_extend = g_key_shorten = 0;
    g_manual_leg_target = 0.0f;
    balance_set_leg_extend_manual(0.0f);
    balance_set_manual(0.0f, 0.0f, 0);
    printf("[Reset] pose restored, balance re-armed\n");
}

static void apply_body_lock_if_requested(Simulation *sim, int body_lock)
{
    if (!body_lock) return;

    mjData *d = sim->data;
    d->qpos[0] = 0.0;
    d->qpos[1] = 0.0;
    d->qpos[2] = 0.5;
    d->qpos[3] = 1.0;
    d->qpos[4] = 0.0;
    d->qpos[5] = 0.0;
    d->qpos[6] = 0.0;
    for (int i = 0; i < 6; i++) d->qvel[i] = 0.0;
}

void apply_open_loop_override(SensorState *s)
{
#if OPEN_LOOP_WHEEL_TEST
    s->ctrl[2] = OPEN_LOOP_WHEEL_CTRL_L;
    s->ctrl[5] = OPEN_LOOP_WHEEL_CTRL_R;
#else
    (void)s;
#endif
}

int simulation_step(Simulation *sim, int body_lock)
{
    mj_step(sim->model, sim->data);
    apply_body_lock_if_requested(sim, body_lock);

    if (!sensors_read(sim->model, sim->data, &sim->state))
        return 0;

    robot_control_step(&sim->state, sim->model->opt.timestep);
    apply_open_loop_override(&sim->state);

    for (int i = 0; i < ACTUATOR_COUNT; i++)
        sim->data->ctrl[i] = sim->state.ctrl[i];

    sim->state.step_count++;
    sim->state.sim_time = sim->state.step_count * sim->model->opt.timestep;
    return 1;
}

/*
 * leg_t - 腿关节实现 (仿真适配版)
 * 位置伺服: ctrl输出 = PID计算的目标角度
 * 三环串级PID: 角度 → 速度 → 力矩
 */

#include "leg_t.h"
#include <string.h>
#include <math.h>

void LegJointInit(LegJoint *lj, LegJoint_Init_s *cfg, float dt) {
    memset(lj, 0, sizeof(LegJoint));
    lj->settings  = cfg->control_settings;
    lj->dt        = dt;
    lj->stop_flag = LEG_ENABLED;

    PIDInit(&lj->controller.angle_PID,  &cfg->angle_PID_cfg,  dt);
    PIDInit(&lj->controller.speed_PID,  &cfg->speed_PID_cfg,  dt);
    PIDInit(&lj->controller.torque_PID, &cfg->torque_PID_cfg, dt);

    lj->controller.other_angle_fb  = cfg->other_angle_fb;
    lj->controller.other_speed_fb  = cfg->other_speed_fb;
    lj->controller.other_torque_fb = cfg->other_torque_fb;

    lj->controller.angle_div  = (cfg->angle_div  > 0) ? cfg->angle_div  : 5;
    lj->controller.speed_div  = (cfg->speed_div  > 0) ? cfg->speed_div  : 2;
    lj->controller.torque_div = (cfg->torque_div > 0) ? cfg->torque_div : 1;
}

void LegJointSetRef(LegJoint *lj, float ref) {
    lj->controller.pid_ref = ref;
}

void LegJointStop(LegJoint *lj)    { lj->stop_flag = LEG_STOP; }
void LegJointEnable(LegJoint *lj)  { lj->stop_flag = LEG_ENABLED; }

void LegJointOuterLoop(LegJoint *lj, LegCloseloop_Type_e loop) {
    lj->settings.outer_loop_type = loop;
}

float LegJointUpdate(LegJoint *lj, float angle_rad, float speed_radps,
                     float torque_fb) {
    LegControlSetting_s *s = &lj->settings;
    LegController_s *c = &lj->controller;
    LegMeasure_s *m = &lj->measure;

    m->angle_rad  = angle_rad;
    m->speed_radps = speed_radps;
    m->torque_nm  = torque_fb;

    float pid_ref = c->pid_ref;
    if (s->motor_dir == LEG_DIR_REVERSE)
        pid_ref *= -1.0f;

    int   tick    = ++c->tick;

    // ---- 角度环 (最外层, e.g. 200Hz) ----
    if ((s->close_loop_type & LEG_ANGLE_LOOP) &&
        s->outer_loop_type == LEG_ANGLE_LOOP) {
        if (tick % c->angle_div == 0) {
            float measure = (s->angle_fb_src == LEG_FEED_OTHER)
                            ? *c->other_angle_fb : m->angle_rad;
            c->angle_out_held = PIDCalculate(&c->angle_PID, measure, pid_ref);
        }
        pid_ref = c->angle_out_held;
    }

    // ---- 速度环 (中速, e.g. 500Hz) ----
    if ((s->close_loop_type & LEG_SPEED_LOOP) &&
        (s->outer_loop_type == LEG_ANGLE_LOOP ||
         s->outer_loop_type == LEG_SPEED_LOOP)) {
        if (lj->stop_flag == LEG_STOP) pid_ref = 0.0f;
        if (tick % c->speed_div == 0) {
            float measure = (s->speed_fb_src == LEG_FEED_OTHER)
                            ? *c->other_speed_fb : m->speed_radps;
            c->speed_out_held = PIDCalculate(&c->speed_PID, measure, pid_ref);
        }
        pid_ref = c->speed_out_held;
    }

    // ---- 力矩环 (最快, e.g. 1000Hz) ----
    if (s->close_loop_type & LEG_TORQUE_LOOP) {
        if (tick % c->torque_div == 0) {
            float measure = (s->torque_fb_src == LEG_FEED_OTHER)
                            ? *c->other_torque_fb : m->torque_nm;
            pid_ref = PIDCalculate(&c->torque_PID, measure, pid_ref);
        }
    }

    if (s->feedback_dir == LEG_DIR_REVERSE)
        pid_ref *= -1.0f;

    if (lj->stop_flag == LEG_STOP)
        pid_ref = 0.0f;

    return pid_ref;  // 返回目标关节角 [rad]
}

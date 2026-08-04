/*
 * wheel_t - 轮毂电机实现 (参考 DJI motor 架构)
 * 方向控制: motor_reverse_flag(反转参考), feedback_reverse_flag(反转反馈)
 */

#include "wheel_t.h"
#include <string.h>
#include <math.h>

void WheelMotorInit(WheelMotor *wm, WheelMotor_Init_s *cfg, float dt) {
    memset(wm, 0, sizeof(WheelMotor));
    wm->settings  = cfg->control_settings;
    wm->dt        = dt;
    wm->stop_flag = MOTOR_ENABLED;

    PIDInit(&wm->controller.angle_PID,  &cfg->angle_PID_cfg,  dt);
    PIDInit(&wm->controller.speed_PID,  &cfg->speed_PID_cfg,  dt);
    PIDInit(&wm->controller.current_PID,&cfg->current_PID_cfg,dt);

    wm->controller.other_angle_fb = cfg->other_angle_fb;
    wm->controller.other_speed_fb = cfg->other_speed_fb;

    wm->controller.angle_div   = (cfg->angle_div   > 0) ? cfg->angle_div   : 5;
    wm->controller.speed_div   = (cfg->speed_div   > 0) ? cfg->speed_div   : 2;
    wm->controller.current_div = (cfg->current_div > 0) ? cfg->current_div : 1;
}

void WheelMotorSetRef(WheelMotor *wm, float ref) {
    wm->controller.pid_ref = ref;
}

void WheelMotorStop(WheelMotor *wm)   { wm->stop_flag = MOTOR_STOP; }
void WheelMotorEnable(WheelMotor *wm) { wm->stop_flag = MOTOR_ENABLED; }

void WheelMotorOuterLoop(WheelMotor *wm, Closeloop_Type_e loop) {
    wm->settings.outer_loop_type = loop;
}

float WheelMotorUpdate(WheelMotor *wm, float angle_rad, float speed_radps,
                       float torque_fb) {
    Motor_Control_Setting_s *s = &wm->settings;
    Motor_Controller_s *c = &wm->controller;
    Motor_Measure_s *m = &wm->measure;

    // 更新测量
    float diff = angle_rad - m->last_angle;
    if (diff >  M_PI) m->total_round--;
    if (diff < -M_PI) m->total_round++;
    m->last_angle  = angle_rad;
    m->total_angle = (float)m->total_round * 2.0f * M_PI + angle_rad;
    m->speed_radps = speed_radps;
    m->torque_nm   = torque_fb;

    float pid_ref = c->pid_ref;

    // 电机反转: 参考取反 (DJI: motor_reverse_flag)
    if (s->motor_reverse_flag == MOTOR_DIR_REVERSE)
        pid_ref *= -1.0f;

    int tick = ++c->tick;

    // ---- 角度环 ----
    if ((s->close_loop_type & ANGLE_LOOP) && s->outer_loop_type == ANGLE_LOOP) {
        if (tick % c->angle_div == 0) {
            float fb = (s->angle_fb_src == OTHER_FEED)
                       ? *c->other_angle_fb : m->total_angle;
            c->angle_out_held = PIDCalculate(&c->angle_PID, fb, pid_ref);
        }
        pid_ref = c->angle_out_held;
    }

    // ---- 速度环 ----
    if ((s->close_loop_type & SPEED_LOOP) &&
        (s->outer_loop_type == ANGLE_LOOP || s->outer_loop_type == SPEED_LOOP)) {
        if (wm->stop_flag == MOTOR_STOP) pid_ref = 0.0f;
        if (tick % c->speed_div == 0) {
            float fb = (s->speed_fb_src == OTHER_FEED)
                       ? *c->other_speed_fb : m->speed_radps;
            // 反馈反向 (DJI: feedback_reverse_flag)
            if (s->feedback_reverse_flag == FEEDBACK_DIR_REVERSE) fb *= -1.0f;
            c->speed_out_held = PIDCalculate(&c->speed_PID, fb, pid_ref);
        }
        pid_ref = c->speed_out_held;
    }

    // ---- 电流环 ----
    if (s->close_loop_type & CURRENT_LOOP) {
        if (tick % c->current_div == 0) {
            float fb = torque_fb;
            if (s->feedback_reverse_flag == FEEDBACK_DIR_REVERSE) fb *= -1.0f;
            pid_ref = PIDCalculate(&c->current_PID, fb, pid_ref);
        }
    }

    if (wm->stop_flag == MOTOR_STOP) pid_ref = 0.0f;

    return pid_ref;
}

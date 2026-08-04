/*
 * wheel_t - 轮毂电机模块
 * 参考 DJI motor 架构: motor_reverse_flag + feedback_reverse_flag 处理方向
 */

#ifndef WHEEL_T_H
#define WHEEL_T_H
#include "pid.h"

/* ---- 闭环类型 ---- */
typedef enum {
    OPEN_LOOP   = 0x00,
    CURRENT_LOOP= 0x01,
    SPEED_LOOP  = 0x02,
    ANGLE_LOOP  = 0x04,
} Closeloop_Type_e;

typedef enum { MOTOR_FEED=0, OTHER_FEED=1 } Feedback_Source_e;
typedef enum { MOTOR_DIR_NORMAL=0, MOTOR_DIR_REVERSE=1 } Motor_Reverse_e;
typedef enum { FEEDBACK_DIR_NORMAL=0, FEEDBACK_DIR_REVERSE=1 } Feedback_Reverse_e;
typedef enum { MOTOR_STOP=0, MOTOR_ENABLED=1 } Motor_WorkState_e;

/* ---- 电机控制设置 (对应 DJI Motor_Control_Setting_s) ---- */
typedef struct {
    Closeloop_Type_e   outer_loop_type;
    Closeloop_Type_e   close_loop_type;
    Motor_Reverse_e    motor_reverse_flag;
    Feedback_Reverse_e feedback_reverse_flag;
    Feedback_Source_e  angle_fb_src;
    Feedback_Source_e  speed_fb_src;
} Motor_Control_Setting_s;

/* ---- 电机控制器 ---- */
typedef struct {
    float *other_angle_fb, *other_speed_fb;
    PIDInstance angle_PID, speed_PID, current_PID;
    float pid_ref;
    int   angle_div, speed_div, current_div, tick;
    float angle_out_held, speed_out_held;
} Motor_Controller_s;

/* ---- 电机测量值 ---- */
typedef struct {
    float total_angle, speed_radps, torque_nm;
    float last_angle; int total_round;
} Motor_Measure_s;

/* ---- 电机实例 ---- */
typedef struct {
    Motor_Measure_s       measure;
    Motor_Control_Setting_s settings;
    Motor_Controller_s    controller;
    Motor_WorkState_e     stop_flag;
    float                 dt;
} WheelMotor;

/* ---- 初始化配置 ---- */
typedef struct {
    Motor_Control_Setting_s control_settings;
    PID_Init_Config_s       angle_PID_cfg, speed_PID_cfg, current_PID_cfg;
    float *other_angle_fb, *other_speed_fb;
    int   angle_div, speed_div, current_div;
} WheelMotor_Init_s;

void  WheelMotorInit(WheelMotor *wm, WheelMotor_Init_s *cfg, float dt);
void  WheelMotorSetRef(WheelMotor *wm, float ref);
void  WheelMotorStop(WheelMotor *wm);
void  WheelMotorEnable(WheelMotor *wm);
void  WheelMotorOuterLoop(WheelMotor *wm, Closeloop_Type_e loop);
float WheelMotorUpdate(WheelMotor *wm, float angle_rad, float speed_radps,
                       float torque_fb);

#endif

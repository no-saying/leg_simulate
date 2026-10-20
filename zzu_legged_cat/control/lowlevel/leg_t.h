/*
 * leg_t - 腿关节模块 (仿真适配版)
 * 位置伺服: 输出力 = gainprm * (ctrl - q)，gainprm=50
 * 支持角度/速度/力矩三环串级PID
 */

#ifndef LEG_T_H
#define LEG_T_H
#include "pid.h"

/* ---- 闭环类型 ---- */
typedef enum {
    LEG_OPEN_LOOP    = 0x00,
    LEG_TORQUE_LOOP  = 0x01,  // 力矩环 (最内层)
    LEG_SPEED_LOOP   = 0x02,  // 速度环
    LEG_ANGLE_LOOP   = 0x04,  // 角度环 (最外层)
} LegCloseloop_Type_e;

typedef enum {
    LEG_FEED_MOTOR  = 0,      // 用关节自带传感器
    LEG_FEED_OTHER  = 1,      // 用外部反馈
} LegFeedback_Source_e;

typedef enum {
    LEG_DIR_NORMAL  = 0,
    LEG_DIR_REVERSE = 1,
} LegDir_e;

typedef enum {
    LEG_STOP    = 0,
    LEG_ENABLED = 1,
} LegWorkState_e;

/* ---- 关节测量值 ---- */
typedef struct {
    float angle_rad;       // 当前位置 [rad]
    float speed_radps;     // 当前速度 [rad/s]
    float torque_nm;       // 当前力矩 [Nm]
} LegMeasure_s;

/* ---- 关节控制器 ---- */
typedef struct {
    float *other_angle_fb;
    float *other_speed_fb;
    float *other_torque_fb;
    PIDInstance angle_PID;
    PIDInstance speed_PID;
    PIDInstance torque_PID;
    float pid_ref;

    // 分频
    int   angle_div, speed_div, torque_div;
    int   tick;
    float angle_out_held, speed_out_held;
} LegController_s;

/* ---- 关节控制设置 ---- */
typedef struct {
    LegCloseloop_Type_e outer_loop_type;
    LegCloseloop_Type_e close_loop_type;
    LegDir_e            motor_dir;
    LegDir_e            feedback_dir;
    LegFeedback_Source_e angle_fb_src;
    LegFeedback_Source_e speed_fb_src;
    LegFeedback_Source_e torque_fb_src;
} LegControlSetting_s;

/* ---- 关节实例 ---- */
typedef struct {
    LegMeasure_s       measure;
    LegControlSetting_s settings;
    LegController_s    controller;
    LegWorkState_e     stop_flag;
    float              dt;
} LegJoint;

/* ---- 初始化配置 ---- */
typedef struct {
    LegControlSetting_s control_settings;
    PID_Init_Config_s   angle_PID_cfg;
    PID_Init_Config_s   speed_PID_cfg;
    PID_Init_Config_s   torque_PID_cfg;
    float *other_angle_fb;
    float *other_speed_fb;
    float *other_torque_fb;
    int   angle_div;      // 默认5
    int   speed_div;      // 默认2
    int   torque_div;     // 默认1
} LegJoint_Init_s;

/* ---- 对外接口 ---- */
void LegJointInit(LegJoint *lj, LegJoint_Init_s *cfg, float dt);
void LegJointSetRef(LegJoint *lj, float ref);
void LegJointStop(LegJoint *lj);
void LegJointEnable(LegJoint *lj);
void LegJointOuterLoop(LegJoint *lj, LegCloseloop_Type_e loop);
float LegJointUpdate(LegJoint *lj, float angle_rad, float speed_radps,
                     float torque_fb);

#endif

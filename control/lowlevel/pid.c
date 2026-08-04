/*
 * PID 控制器实现 — 适配自 RM_hero 项目
 * 原版: wanghongxi / neozng, HNU YueLu EC
 */

#include "pid.h"
#include <string.h>

/* ---- 内部优化环节 ---- */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid) {
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2.0f) * pid->dt;
}

// 变速积分
static void f_Changing_Integration_Rate(PIDInstance *pid) {
    if (pid->Err * pid->Iout > 0) {
        if (fabsf(pid->Err) <= pid->CoefB) return;
        if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
        else
            pid->ITerm = 0;
    }
}

// 积分限幅
static void f_Integral_Limit(PIDInstance *pid) {
    float temp_Iout = pid->Iout + pid->ITerm;
    float temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (fabsf(temp_Output) > pid->MaxOut) {
        if (pid->Err * pid->Iout > 0) pid->ITerm = 0;
    }
    if (temp_Iout > pid->IntegralLimit)  { pid->ITerm = 0; pid->Iout =  pid->IntegralLimit; }
    if (temp_Iout < -pid->IntegralLimit) { pid->ITerm = 0; pid->Iout = -pid->IntegralLimit; }
}

// 微分先行 (用测量值而非误差的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid) {
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波
static void f_Derivative_Filter(PIDInstance *pid) {
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt)
              + pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid) {
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt)
                + pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid) {
    if (pid->Output > pid->MaxOut)  pid->Output = pid->MaxOut;
    if (pid->Output < pid->MaxOut_) pid->Output = pid->MaxOut_;
}

/* ---- 外部API ---- */

void PIDInit(PIDInstance *pid, PID_Init_Config_s *config, float dt) {
    memset(pid, 0, sizeof(PIDInstance));
    memcpy(pid, config, sizeof(PID_Init_Config_s));
    pid->dt = dt;
    pid->first = 1;
}

void PIDClear(PIDInstance *pid) {
    pid->Iout = 0;
    pid->ITerm = 0;
    pid->Output = 0;
    pid->Last_Err = 0;
    pid->Last_Measure = 0;
    pid->Last_Dout = 0;
    pid->Last_Output = 0;
    pid->first = 1;
}

float PIDCalculate(PIDInstance *pid, float measure, float ref) {
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 死区
    if (fabsf(pid->Err) <= pid->DeadBand) {
        pid->Output = 0;
        pid->ITerm = 0;
        pid->Last_Measure = measure;
        pid->Last_Err = pid->Err;
        pid->Last_Output = pid->Output;
        pid->Last_Dout = pid->Dout;
        pid->Last_ITerm = pid->ITerm;
        pid->first = 0;
        return 0;
    }

    // 基本PID
    pid->Pout = pid->Kp * pid->Err;
    pid->ITerm = pid->Ki * pid->Err * pid->dt;
    pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

    // 梯形积分
    if (pid->Improve & PID_Trapezoid_Intergral)
        f_Trapezoid_Intergral(pid);
    // 变速积分
    if (pid->Improve & PID_ChangingIntegrationRate)
        f_Changing_Integration_Rate(pid);
    // 微分先行
    if (pid->Improve & PID_Derivative_On_Measurement)
        f_Derivative_On_Measurement(pid);
    // 微分滤波
    if (pid->Improve & PID_DerivativeFilter)
        f_Derivative_Filter(pid);
    // 积分限幅
    if (pid->Improve & PID_Integral_Limit)
        f_Integral_Limit(pid);

    pid->Iout += pid->ITerm;
    pid->Output = pid->Pout + pid->Iout + pid->Dout;

    // 输出滤波
    if (pid->Improve & PID_OutputFilter)
        f_Output_Filter(pid);
    // 输出限幅
    f_Output_Limit(pid);

    // 保存状态
    pid->Last_Measure = measure;
    pid->Last_Err = pid->Err;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_ITerm = pid->ITerm;
    pid->first = 0;

    return pid->Output;
}

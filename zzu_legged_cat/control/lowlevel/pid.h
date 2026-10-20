/*
 * PID 控制器 — 适配自 RM_hero 项目
 * 已剥离嵌入式硬件依赖 (DWT, CMSIS-DSP)
 */

#ifndef PID_H
#define PID_H

#include <math.h>

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#endif

/* PID 优化环节使能标志位 */
typedef enum {
    PID_IMPROVE_NONE                = 0x00,
    PID_Integral_Limit              = 0x01,
    PID_Derivative_On_Measurement   = 0x02,
    PID_Trapezoid_Intergral         = 0x04,
    PID_Proportional_On_Measurement = 0x08,
    PID_OutputFilter                = 0x10,
    PID_ChangingIntegrationRate     = 0x20,
    PID_DerivativeFilter            = 0x40,
    PID_ErrorHandle                 = 0x80,
} PID_Improvement_e;

/* PID 结构体 */
typedef struct {
    float Kp, Ki, Kd;
    float MaxOut;               // 正向输出限幅
    float MaxOut_;              // 反向输出限幅 (负值)
    float DeadBand;             // 死区
    PID_Improvement_e Improve;
    float IntegralLimit;
    float CoefA, CoefB;         // 变速积分参数
    float Output_LPF_RC;        // 输出滤波系数
    float Derivative_LPF_RC;    // 微分滤波系数

    // 内部变量
    float Measure, Last_Measure;
    float Err, Last_Err, Last_ITerm;
    float Pout, Iout, Dout, ITerm;
    float Output, Last_Output, Last_Dout;
    float Ref;
    float dt;
    int   first;                 // 首次调用标志 (替代DWT)
} PIDInstance;

/* PID 初始化配置 (仅需填的字段) */
typedef struct {
    float Kp, Ki, Kd;
    float MaxOut;
    float MaxOut_;
    float DeadBand;
    PID_Improvement_e Improve;
    float IntegralLimit;
    float CoefA, CoefB;
    float Output_LPF_RC;
    float Derivative_LPF_RC;
} PID_Init_Config_s;

// ---- 供外部调用的接口 ----
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config, float dt);
void PIDClear(PIDInstance *pid);
float PIDCalculate(PIDInstance *pid, float measure, float ref);

#endif

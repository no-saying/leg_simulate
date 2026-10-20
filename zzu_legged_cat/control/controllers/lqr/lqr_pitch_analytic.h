/* lqr_pitch_analytic.h - 由 scripts/tools/analytic_pitch_model.py 生成, 勿手改
 * 解析倒立摆模型: 质心高 h(L_c) 直接由 MJCF 读, A/B 解析, K 逐腿长解出。
 * 与辨识路线相比: K 全程连续且数值温和, 不依赖任何地面数据。
 * 状态 [pitch, pitch_rate, wheel_v], 输入[共模轮扭矩]。 */
#ifndef LQR_PITCH_ANALYTIC_H
#define LQR_PITCH_ANALYTIC_H

#define LQR_PA_N 7

typedef struct { float L, h; float K[3]; } LqrPitchAnalyticPoint;
static const LqrPitchAnalyticPoint lqr_pitch_analytic[7] = {
  { 0.0900f, 0.056362f, { +75.670075f, +15.942739f, -1.825742f } },
  { 0.1000f, 0.055295f, { +75.669477f, +15.940246f, -1.825742f } },
  { 0.1100f, 0.053839f, { +75.668661f, +15.936845f, -1.825742f } },
  { 0.1170f, 0.052593f, { +75.667963f, +15.933935f, -1.825742f } },
  { 0.1250f, 0.050946f, { +75.667039f, +15.930088f, -1.825742f } },
  { 0.1350f, 0.048560f, { +75.665703f, +15.924517f, -1.825742f } },
  { 0.1500f, 0.044329f, { +75.663333f, +15.914642f, -1.825742f } },
};

#endif /* LQR_PITCH_ANALYTIC_H */

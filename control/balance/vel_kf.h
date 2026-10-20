/*
 * vel_kf.h - 前向速度卡尔曼融合(轮速 + IMU 加速度)
 * ================================================================
 * 替代原来的互补滤波(g_balance.simple_imu_v 的手工分段加权)。
 *
 * 状态: x = [v, b_a]^T   v=机体系前向速度, b_a=加速度计前向偏置
 * 预测: v += (a_imu - b_a)·dt ; b_a 随机游走
 * 量测: z = v_wheel(已做腿运动学补偿的轮速)
 * 量测噪声 R = R0 / trust, trust 由打滑/离地/冲击标志给出:
 *      粘着 1.0 / 打滑 0.05 / 离地 0(跳过量测, 纯惯性递推)
 * 显式估计偏置 => 静止不漂(配合 ZUPT), 打滑时不被污染轮速带偏。
 *
 * 参数与 scripts/tools/identify_lqr_v3.py 里的 KF 实现严格一致
 * (辨识端与部署端必须用同一套方程/参数, 否则状态定义不一致)。
 */
#ifndef VEL_KF_H
#define VEL_KF_H

#ifdef __cplusplus
extern "C" {
#endif

void  vel_kf_reset(void);
float vel_kf_step(float a_imu, float v_wheel, float trust, int airborne, float dt);
float vel_kf_bias(void);
void  vel_kf_set_velocity(float velocity);

#ifdef __cplusplus
}
#endif
#endif /* VEL_KF_H */

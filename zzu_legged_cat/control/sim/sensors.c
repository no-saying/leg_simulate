#include "sensors.h"

#include "control_config.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* ==========================================================================
 * 传感器安装矩阵补偿(v3 串联腿闭链模型)
 * ==========================================================================
 * v3 模型绕世界 x 轴旋转 +90° 立起(机器人 CAD 坐标系 z 轴水平朝左,
 * 立起后轮轴沿世界 -y)。因此 base_link 在静止站立时世界姿态为
 * R_bake = R_x(+90°), 控制器约定(与旧模型一致)要求:
 *   - 站立时 pitch/roll/yaw = 0 (控制系 = 世界系);
 *   - 安装补偿后的 gyro 仍是机体系读数；只有零姿态时轴与世界系重合。
 *     俯仰角速度由 attitude_rates.h 换算，避免侧倾时混入偏航运动。
 * 补偿:
 *   q_ctrl = q_body ⊗ q_bake^{-1}, q_bake^{-1} = conj(R_x(+90°))
 *   ω_ctrl = R_bake·ω_body   -> (x, -z, y)
 *   a_ctrl = R_bake·a_body   -> (x, -z, y)
 */
#define IMU_BAKE_Q_W 0.70710678f
#define IMU_BAKE_Q_X 0.70710678f
#define IMU_BAKE_Q_Y 0.0f
#define IMU_BAKE_Q_Z 0.0f

typedef struct {
    int ready;
    int lw_joint, rw_joint;
    int lw_qpos, rw_qpos;
    int lw_dof, rw_dof;
} SensorAddressCache;

static SensorAddressCache g_addr;

static double clamp_unit(double x)
{
    if (x > 1.0) return 1.0;
    if (x < -1.0) return -1.0;
    return x;
}

static double quat_to_pitch(const double q[4])
{
    return asin(clamp_unit(2.0 * (q[0] * q[2] - q[1] * q[3])));
}

static double quat_to_roll(const double q[4])
{
    double w = q[0], x = q[1], y = q[2], z = q[3];
    return atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
}

static double quat_to_yaw(const double q[4])
{
    double w = q[0], x = q[1], y = q[2], z = q[3];
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

/* 四元数乘法(w,x,y,z), 表示 R(q1*q2) = R(q1)·R(q2) */
static void quat_mul(const double a[4], const double b[4], double out[4])
{
    double aw = a[0], ax = a[1], ay = a[2], az = a[3];
    double bw = b[0], bx = b[1], by = b[2], bz = b[3];
    out[0] = aw * bw - ax * bx - ay * by - az * bz;
    out[1] = aw * bx + ax * bw + ay * bz - az * by;
    out[2] = aw * by - ax * bz + ay * bw + az * bx;
    out[3] = aw * bz + ax * by - ay * bx + az * bw;
}

/* 安装矩阵补偿: 把 base_link 传感器系转到控制器约定系 */
static void imu_mount_compensate(const double q_body[4], const double gyro_body[3],
                                 const double acc_body[3],
                                 double q_ctrl[4], double gyro_ctrl[3],
                                 double acc_ctrl[3])
{
    const double q_mount[4] = {IMU_BAKE_Q_W, -IMU_BAKE_Q_X,
                               -IMU_BAKE_Q_Y, -IMU_BAKE_Q_Z};
    quat_mul(q_body, q_mount, q_ctrl);
    /* R_bake = R_x(+90°): (x,y,z) -> (x, -z, y) */
    gyro_ctrl[0] = gyro_body[0];
    gyro_ctrl[1] = -gyro_body[2];
    gyro_ctrl[2] = gyro_body[1];
    acc_ctrl[0] = acc_body[0];
    acc_ctrl[1] = -acc_body[2];
    acc_ctrl[2] = acc_body[1];
}

static int cache_sensor_addresses(const mjModel *m)
{
    if (g_addr.ready) return 1;

    g_addr.lw_joint = mj_name2id(m, mjOBJ_JOINT, "Left_Wheel_joint");
    g_addr.rw_joint = mj_name2id(m, mjOBJ_JOINT, "Right_Wheel_joint");
    if (g_addr.lw_joint < 0 || g_addr.rw_joint < 0) {
        fprintf(stderr, "FATAL: wheel joint name not found in MJCF\n");
        return 0;
    }

    g_addr.lw_qpos = m->jnt_qposadr[g_addr.lw_joint];
    g_addr.rw_qpos = m->jnt_qposadr[g_addr.rw_joint];
    g_addr.lw_dof = m->jnt_dofadr[g_addr.lw_joint];
    g_addr.rw_dof = m->jnt_dofadr[g_addr.rw_joint];
    g_addr.ready = 1;
    return 1;
}

void sensors_set_initial_pose(const mjModel *m, mjData *d)
{
    /* v3 模型: 轮底触地(轮心 z=-0.107, 半径 0.043, 地面 -0.152),
     * 初始姿态 = 绕 x +90° 立起(与 IMU_BAKE_Q_* 一致)。 */
    d->qpos[0] = 0.0;
    d->qpos[1] = 0.0;
    d->qpos[2] = -0.025;   /* 中位车高轮子着地 */
    d->qpos[3] = IMU_BAKE_Q_W;
    d->qpos[4] = IMU_BAKE_Q_X;
    d->qpos[5] = IMU_BAKE_Q_Y;
    d->qpos[6] = IMU_BAKE_Q_Z;
    /* 驱动关节初始设中位(重心在轮轴正上方): claf0=+0.2, thigh=-0.2。
     * 注意: v3 右腿是左腿的镜像(J_right=-J_left), 同样腿长/腿摆下
     * 右腿关节角必须取反(claf0=-0.2, thigh=+0.2)。旧代码四腿同符号
     * 导致右腿初始 L=0.0586、左腿 0.1145, 腿长伺服一启动就输出
     * 116N 巨力拉右腿, 表现为"右腿摆动方向反了、车原地转圈"。 */
    const char *mid[4] = {"Left_front_joint", "Left_rear_joint",
                          "Right_front_joint", "Right_rear_joint"};
    const float mid_v[4] = {0.2f, -0.2f, -0.2f, 0.2f};
    for (int i = 0; i < 4; i++) {
        int j = mj_name2id(m, mjOBJ_JOINT, mid[i]);
        if (j >= 0) d->qpos[m->jnt_qposadr[j]] = mid_v[i];
    }
    for (int i = 7; i < m->nq; i++) {
        int is_mid = 0;
        for (int k = 0; k < 4; k++) {
            int j = mj_name2id(m, mjOBJ_JOINT, mid[k]);
            if (j >= 0 && i == m->jnt_qposadr[j]) { is_mid = 1; break; }
        }
        /* Preserve the initial positions and unit quaternions of scene objects. */
        if (!is_mid) d->qpos[i] = m->qpos0[i];
    }
    for (int i = 0; i < m->nv; i++) d->qvel[i] = 0.0;
}

int sensors_read(const mjModel *m, mjData *d, SensorState *s)
{
    if (m->nsensordata < 18) {
        fprintf(stderr, "FATAL: nsensordata=%d\n", (int)m->nsensordata);
        return 0;
    }
    if (!cache_sensor_addresses(m)) return 0;

    /* 轮编码器通道。速度来自 qvel，不使用世界系机体速度。 */
    s->lw_pos = d->qpos[g_addr.lw_qpos];
    s->rw_pos = d->qpos[g_addr.rw_qpos];
    s->lw_vel = d->qvel[g_addr.lw_dof];
    s->rw_vel = d->qvel[g_addr.rw_dof];

    /* 原始传感器帧(base_link) -> 控制器约定帧 */
    double q_ctrl[4], gyro_ctrl[3], acc_ctrl[3];
    const double q_body[4] = {d->sensordata[0], d->sensordata[1],
                              d->sensordata[2], d->sensordata[3]};
    const double gyro_body[3] = {d->sensordata[4], d->sensordata[5],
                                 d->sensordata[6]};
    const double acc_body[3] = {d->sensordata[7], d->sensordata[8],
                                d->sensordata[9]};
    imu_mount_compensate(q_body, gyro_body, acc_body,
                         q_ctrl, gyro_ctrl, acc_ctrl);

    for (int i = 0; i < 4; i++) s->quat[i] = (float)q_ctrl[i];
    for (int i = 0; i < 3; i++) s->gyro[i] = (float)gyro_ctrl[i];
    for (int i = 0; i < 3; i++) s->acc[i] = (float)acc_ctrl[i];
    s->acc_mag = sqrt(s->acc[0] * s->acc[0] + s->acc[2] * s->acc[2]);

    /* 调试/鲁棒性验证: 注入 IMU 安装角误差(度)。模拟装配倾斜 —— 控制器
     * 看到的 pitch 与实际姿态差一个固定偏置。 */
    {
        static int tilt_ready = 0;
        static double tilt_off = 0.0;
        if (!tilt_ready) {
            const char *e = getenv("IMU_TILT_DEG");
            tilt_off = e ? atof(e) * 3.14159265358979 / 180.0 : 0.0;
            tilt_ready = 1;
        }
        if (tilt_off != 0.0) s->pitch = quat_to_pitch(q_ctrl) + tilt_off;
        else s->pitch = quat_to_pitch(q_ctrl);
    }
    s->roll = quat_to_roll(q_ctrl);
    s->yaw = quat_to_yaw(q_ctrl);
    s->yaw_rate = s->gyro[2];

    /* 世界系位置明确只用于显示。 */
    for (int i = 0; i < 3; i++) s->body_pos[i] = d->qpos[i];
    /* 世界系线速度: RL 跳跃观测需要 |vx|, |vy|。 */
    for (int i = 0; i < 3; i++) s->body_vel[i] = d->qvel[i];

    s->lf_pos = d->sensordata[10]; s->lf_vel = d->sensordata[11];
    s->lr_pos = d->sensordata[12]; s->lr_vel = d->sensordata[13];
    s->rf_pos = d->sensordata[14]; s->rf_vel = d->sensordata[15];
    s->rr_pos = d->sensordata[16]; s->rr_vel = d->sensordata[17];

    s->lf_torque = d->actuator_force[0];
    s->lr_torque = d->actuator_force[1];
    /* actuator_force 是电机侧扭矩，乘减速比换算成轮上扭矩，保持日志单位=Nm@轮。 */
    s->lw_torque = d->actuator_force[2] * WHEEL_GEAR_RATIO;
    s->rf_torque = d->actuator_force[3];
    s->rr_torque = d->actuator_force[4];
    s->rw_torque = d->actuator_force[5] * WHEEL_GEAR_RATIO;
    return 1;
}

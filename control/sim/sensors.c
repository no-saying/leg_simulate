#include "sensors.h"

#include "control_config.h"

#include <math.h>
#include <stdio.h>

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
    (void)m;
    d->qpos[0] = 0.0;
    d->qpos[1] = 0.0;
    d->qpos[2] = 0.08;
    d->qpos[3] = 1.0;
    d->qpos[4] = 0.0;
    d->qpos[5] = 0.0;
    d->qpos[6] = 0.0;
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

    for (int i = 0; i < 4; i++) s->quat[i] = d->sensordata[i];
    for (int i = 0; i < 3; i++) s->gyro[i] = d->sensordata[4 + i];
    for (int i = 0; i < 3; i++) s->acc[i] = d->sensordata[7 + i];
    s->acc_mag = sqrt(s->acc[0] * s->acc[0] + s->acc[2] * s->acc[2]);

    s->pitch = quat_to_pitch(s->quat);
    s->roll = quat_to_roll(s->quat);
    s->yaw = quat_to_yaw(s->quat);
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

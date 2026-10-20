#ifndef ATTITUDE_RATES_H
#define ATTITUDE_RATES_H

#include <math.h>
#include "sim_state.h"
#include "../balance/balance.h"

/* ZYX pitch derivative: a tilted body's Y gyro includes yaw motion. */
static inline float attitude_pitch_rate(double roll, double gyro_y, double gyro_z)
{
    return (float)(cos(roll) * gyro_y - sin(roll) * gyro_z);
}

/* ZYX roll derivative; raw body X gyro also contains tilted yaw motion. */
static inline float attitude_roll_rate(double roll, double pitch,
                                      double gyro_x, double gyro_y, double gyro_z)
{
    double cp = cos(pitch);
    if (fabs(cp) < 0.2) return (float)gyro_x;
    return (float)(gyro_x + sin(pitch) / cp *
                  (sin(roll) * gyro_y + cos(roll) * gyro_z));
}

/* The deployed fit uses body Y rate without a commanded yaw motion. */
static inline float attitude_control_pitch_rate(double roll, double gyro_y, double gyro_z, int turning)
{
    if (!turning) return (float)gyro_y;
    double blend = fabs(gyro_z) / 0.4;
    if (blend > 1.0) blend = 1.0;
    return (float)(gyro_y + blend * (attitude_pitch_rate(roll, gyro_y, gyro_z) - gyro_y));
}

static inline float sensor_pitch_rate(const SensorState *s)
{
    int turning = balance_get_manual_spin() || fabsf(balance_get_manual_yaw()) >= 0.05f ||
                  (balance_get_post_manual() && fabs(s->gyro[2]) >= 0.05);
    return attitude_control_pitch_rate(s->roll, s->gyro[1], s->gyro[2], turning);
}

#endif

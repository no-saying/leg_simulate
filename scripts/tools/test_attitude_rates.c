#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "attitude_rates.h"

static double pitch(const double q[4])
{
    return asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
}

static double roll_angle(const double q[4])
{
    return atan2(2.0*(q[0]*q[1]+q[2]*q[3]), 1.0-2.0*(q[1]*q[1]+q[2]*q[2]));
}

int main(void)
{
    double worst = 0.0;
    int cases = 0;
    for (int ir = -4; ir <= 4; ir++) {
        for (int ip = -4; ip <= 4; ip++) {
            for (int iw = -3; iw <= 3; iw++) {
                double roll = ir * 0.2, tilt = ip * 0.2, yaw = 0.7;
                double cr = cos(roll / 2), sr = sin(roll / 2);
                double cp = cos(tilt / 2), sp = sin(tilt / 2);
                double cy = cos(yaw / 2), sy = sin(yaw / 2);
                double q[4] = { cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                                cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy };
                double wx = 0.3, wy = iw * 0.4, wz = 2.4;
                double dq[4] = { -q[1]*wx - q[2]*wy - q[3]*wz,
                                 q[0]*wx + q[2]*wz - q[3]*wy,
                                 q[0]*wy + q[3]*wx - q[1]*wz,
                                 q[0]*wz + q[1]*wy - q[2]*wx };
                double next[4], norm = 0.0, dt = 1e-6;
                for (int i = 0; i < 4; i++) {
                    next[i] = q[i] + 0.5 * dq[i] * dt;
                    norm += next[i] * next[i];
                }
                for (int i = 0; i < 4; i++) next[i] /= sqrt(norm);
                /* Independent oracle: integrate a body gyro quaternion step,
                 * then measure the change of the resulting Euler pitch. */
                double actual = (pitch(next) - pitch(q)) / dt;
                double error = fabs(actual - attitude_pitch_rate(roll, wy, wz));
                assert(error < 1e-5);
                double actual_roll = (roll_angle(next) - roll_angle(q)) / dt;
                double roll_error = fabs(actual_roll - attitude_roll_rate(roll,tilt,wx,wy,wz));
                assert(roll_error < 1e-5);
                if (roll_error > worst) worst = roll_error;
                if (error > worst) worst = error;
                cases++;
            }
        }
    }
    double roll = 0.3, rate = 2.4;
    assert(fabs(attitude_pitch_rate(roll, rate * sin(roll), rate * cos(roll))) < 1e-6);
    assert(attitude_control_pitch_rate(roll, 0.7, 0.05, 0) == (float)0.7);
    assert(attitude_control_pitch_rate(roll, 0.7, rate, 1) == attitude_pitch_rate(roll, 0.7, rate));
    double tilt = 0.4;
    assert(fabs(attitude_roll_rate(roll,tilt,-rate*sin(tilt),
                rate*sin(roll)*cos(tilt),rate*cos(roll)*cos(tilt))) < 1e-6);
    printf("%d quaternion integration cases passed; max error %.9f rad/s; tilted yaw has zero pitch/roll derivative\n",
           cases, worst);
    return 0;
}

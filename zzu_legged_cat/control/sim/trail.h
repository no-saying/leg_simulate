#ifndef SIM_TRAIL_H
#define SIM_TRAIL_H

#include <mujoco/mujoco.h>

enum { ODOM_TRAIL_MAX = 1200 };

typedef struct {
    float odom_x[ODOM_TRAIL_MAX];
    float odom_y[ODOM_TRAIL_MAX];
    float body_x[ODOM_TRAIL_MAX];
    float body_y[ODOM_TRAIL_MAX];
    int count;
    int next;
    double last_t;
} OdomTrail;

void trail_reset(OdomTrail *trail);
void trail_push(OdomTrail *trail, double t, float ox, float oy, float bx, float by);
void trail_add_scene(const OdomTrail *trail, mjvScene *scn);
int trail_count(const OdomTrail *trail);

#endif

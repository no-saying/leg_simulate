#ifndef SIM_SENSORS_H
#define SIM_SENSORS_H

#include <mujoco/mujoco.h>
#include "sim_state.h"

void sensors_set_initial_pose(const mjModel *m, mjData *d);
int sensors_read(const mjModel *m, mjData *d, SensorState *s);

#endif

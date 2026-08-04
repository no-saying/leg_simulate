#ifndef RUNTIME_ROBOT_CONTROL_H
#define RUNTIME_ROBOT_CONTROL_H

#include "sim_state.h"

void robot_control_step(SensorState *s, double dt);

#endif

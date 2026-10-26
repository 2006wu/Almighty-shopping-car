/* ROS-independent simulation model. All deadlines are monotonic seconds. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "contract.h"

typedef struct {
  double heartbeat_time, command_time, last_step, drop_commands_until;
  uint32_t counter;
  unsigned recovery;
  bool invalid;
  double linear, angular, position[4], velocity[4];
  const char *status;
} Model;
void model_init(Model *m);
void model_heartbeat(Model *m, uint32_t counter, double now);
void model_command(Model *m, const double command[6], double now);
void model_step(Model *m, double now);

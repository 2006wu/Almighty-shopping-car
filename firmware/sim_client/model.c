#include "model.h"
#include <math.h>
#include <string.h>

static double clamp(double x, double low, double high) {
  return fmax(low, fmin(high, x));
}
static bool linked(Model *m, double now) {
  if (now < m->heartbeat_time || now-m->heartbeat_time >= MCU_HEARTBEAT_TIMEOUT) {
    m->recovery = 0;
    m->command_time = -INFINITY;
  }
  return m->recovery >= MCU_RECOVERY_COUNT;
}
void model_init(Model *m) {
  memset(m, 0, sizeof(*m));
  m->heartbeat_time = m->command_time = m->last_step = -INFINITY;
  m->status = MCU_STATUS_LINK_LOST;
}
void model_heartbeat(Model *m, uint32_t counter, double now) {
  bool ready = linked(m, now);
  uint32_t delta = counter-m->counter;
  if (!m->recovery || (delta > 0 && delta < UINT32_C(0x80000000))) {
    if (m->recovery < MCU_RECOVERY_COUNT) m->recovery++;
    m->counter = counter;
    m->heartbeat_time = now;
  }
  if (!ready) m->command_time = -INFINITY;
}
void model_command(Model *m, const double c[6], double now) {
  if (now < m->drop_commands_until) return;
  m->invalid = false;
  for (int i = 0; i < 6; ++i) {
    if (!isfinite(c[i]) || ((i != 0 && i != 5) && c[i] != 0)) m->invalid = true;
  }
  if (m->invalid) {
    m->command_time = -INFINITY;
    memset(m->velocity, 0, sizeof(m->velocity));
    return;
  }
  m->linear = clamp(c[0], 0, MCU_MAX_LINEAR);
  m->angular = clamp(c[5], -MCU_MAX_ANGULAR, MCU_MAX_ANGULAR);
  m->command_time = linked(m, now) ? now : -INFINITY;
}
void model_step(Model *m, double now) {
  double dt = isfinite(m->last_step) ? clamp(now-m->last_step, 0, 0.1) : 0;
  m->last_step = now;
  bool link = linked(m, now);
  bool fresh = now >= m->command_time && now-m->command_time < MCU_COMMAND_TIMEOUT;
  m->status = !link ? MCU_STATUS_LINK_LOST : m->invalid ? MCU_STATUS_INVALID_COMMAND :
    !fresh ? MCU_STATUS_CMD_TIMEOUT : MCU_STATUS_READY;
  if (strcmp(m->status, MCU_STATUS_READY)) {
    memset(m->velocity, 0, sizeof(m->velocity));
    return;
  }
  double alpha = 1-exp(-dt/MCU_RESPONSE_TIME);
  double limit = MCU_WHEEL_ACCELERATION*dt;
  for (int i = 0; i < 4; ++i) {
    double target = (m->linear + (i < 2 ? -1 : 1)*m->angular*MCU_TRACK/2)/MCU_WHEEL_RADIUS;
    m->velocity[i] += clamp((target-m->velocity[i])*alpha, -limit, limit);
    m->position[i] += m->velocity[i]*dt;
  }
}

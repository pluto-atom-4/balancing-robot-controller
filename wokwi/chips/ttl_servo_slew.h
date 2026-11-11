/* Sim-only XL330 position-mode slew. Profile Velocity unit and model UNVERIFIED against the e-Manual; not hardware-validated. */
#ifndef TTL_SERVO_SLEW_H
#define TTL_SERVO_SLEW_H

#include <stdint.h>

#define TTL_SERVO_POS_MAX 4095u

static inline uint32_t ttl_servo_clamp_goal(uint32_t goal) {
  return goal > TTL_SERVO_POS_MAX ? TTL_SERVO_POS_MAX : goal;
}

/* profile_vel == 0 means no limit (jump to goal). *frac is the signed sub-tick remainder; it is dropped on direction reversal. No wrap-around. */
static inline uint32_t ttl_servo_slew_step(uint32_t pos, uint32_t goal, uint32_t profile_vel,
                                           double ticks_per_s_per_unit, double delta_t, double *frac) {
  goal = ttl_servo_clamp_goal(goal);
  if (pos == goal) {
    *frac = 0.0;
    return pos;
  }
  int32_t rem = (int32_t)goal - (int32_t)pos;
  if (profile_vel == 0) {
    *frac = 0.0;
    return goal;
  }
  double step = (double)profile_vel * ticks_per_s_per_unit * delta_t;
  double absrem = rem > 0 ? (double)rem : -(double)rem;
  if (step >= absrem) {
    *frac = 0.0;
    return goal;
  }
  if ((*frac > 0.0 && rem < 0) || (*frac < 0.0 && rem > 0)) {
    *frac = 0.0;
  }
  *frac += (rem > 0 ? step : -step);
  int32_t whole = (int32_t)*frac;
  *frac -= (double)whole;
  int32_t newpos = (int32_t)pos + whole;
  if ((rem > 0 && newpos > (int32_t)goal) || (rem < 0 && newpos < (int32_t)goal)) {
    newpos = (int32_t)goal;
    *frac = 0.0;
  }
  if (newpos < 0) {
    newpos = 0;
  }
  if (newpos > (int32_t)TTL_SERVO_POS_MAX) {
    newpos = (int32_t)TTL_SERVO_POS_MAX;
  }
  return (uint32_t)newpos;
}

#endif

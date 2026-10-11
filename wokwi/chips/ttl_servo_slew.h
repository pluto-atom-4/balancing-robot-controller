/*
 * ttl_servo_slew.h
 *
 * Sim-only model of XL330 position-mode motion (goal position slewed at a
 * Profile Velocity limit). The Profile Velocity unit and the whole model are
 * UNVERIFIED against the XL330 e-Manual. Simulation-only, not hardware-validated.
 *
 * Pure C99 (also valid C++). No libm, no heap, no stdio.
 */
#ifndef TTL_SERVO_SLEW_H
#define TTL_SERVO_SLEW_H

#include <stdint.h>

#define TTL_SERVO_POS_MAX 4095u

static inline uint32_t ttl_servo_clamp_goal(uint32_t goal) {
  return goal > TTL_SERVO_POS_MAX ? TTL_SERVO_POS_MAX : goal;
}

/* One timer tick of position-mode motion toward goal.
   profile_vel == 0 means no velocity limit: jump to the goal.
   Otherwise step = profile_vel * ticks_per_s_per_unit * delta_t (ticks, may be fractional).
   *frac carries the sub-tick remainder between calls (signed). Never overshoots the goal.
   The goal is clamped with ttl_servo_clamp_goal first. Raw difference, no wrap-around. */
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
  *frac += (rem > 0 ? step : -step);
  int32_t whole = (int32_t)*frac; /* truncates toward zero */
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

#endif /* TTL_SERVO_SLEW_H */

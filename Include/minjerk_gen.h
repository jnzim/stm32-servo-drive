// minjerk_gen.h — single-axis minimum-jerk move generator
//
// Rest-to-rest quintic. With D = target - start and tau = t/T in [0, 1]:
//
//     s(tau)    =  10 tau^3 -  15 tau^4 +   6 tau^5
//     s'(tau)   =  30 tau^2 -  60 tau^3 +  30 tau^4
//     s''(tau)  =  60 tau   - 180 tau^2 + 120 tau^3
//     s'''(tau) =  60       - 360 tau   + 360 tau^2
//
//     x(t) = start + D * s(tau)
//     v(t) = D/T   * s'(tau)        <- derivatives are w.r.t. tau, so each
//     a(t) = D/T^2 * s''(tau)          time derivative carries one more 1/T
//     j(t) = D/T^3 * s'''(tau)
//
// Velocity and acceleration are zero at both ends; jerk is not (60*D/T^3),
// so the profile is smooth in acceleration but not in jerk at start and end.
//
// Same interface as trap_gen.h, including the vel_max/accel_max limits, so the
// Pi's MoveCmd is unchanged. The duration T is the shortest that respects both:
//
//     peak |v| = 1.875 |D| / T      at tau = 1/2
//     peak |a| = 5.7735 |D| / T^2   at tau = (3 - sqrt 3)/6, i.e. 10/sqrt(3)
//
//     T = max( 1.875 |D| / vel_max,  sqrt(5.7735 |D| / accel_max) )
//
// Units are the caller's -- foc_trajectory.c uses encoder counts, counts/s.

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool     active;
    int32_t  start_pos;   // counts
    int32_t  target_pos;  // counts
    float    dist;        // counts, D = target - start (signed)
    float    duration;    // s, T
    uint32_t tick;        // steps taken since start; t = tick*dt, no float drift
    float    pos;         // counts, current commanded position
    float    vel;         // counts/s, current commanded velocity (signed)
} MinJerkGen;

// Start a move from start_pos to target_pos. vel_max/accel_max are
// magnitudes (must be > 0).
void minjerk_gen_start(MinJerkGen *g, int32_t start_pos, int32_t target_pos,
                       int32_t vel_max, int32_t accel_max);

// Advance one tick of dt seconds; writes this tick's commanded position and
// velocity. dt must be the same on every call -- time is tick*dt. Safe to
// keep calling after the move ends (holds at target, zero velocity).
void minjerk_gen_step(MinJerkGen *g, float dt, int32_t *pos_cmd, float *vel_cmd);

// True from minjerk_gen_start() until tau reaches 1.
bool minjerk_gen_active(const MinJerkGen *g);

// trap_gen.h — single-axis trapezoidal move generator
//
// Given a target position and vel/accel limits, produces a commanded
// position + velocity each tick that ramps up to vel_max (or a lower peak
// if the move is too short to reach it), holds, then ramps back down to
// stop exactly at the target. All units are the caller's choice (this file
// only does the math) — foc_trajectory.c uses encoder counts / counts-per-
// second, matching the wire units the Pi will eventually send.
//
// Usage: trap_gen_start() once, then trap_gen_step() every tick until
// trap_gen_active() returns false.

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TRAP_GEN_IDLE = 0,
    TRAP_GEN_ACCEL,
    TRAP_GEN_CRUISE,
    TRAP_GEN_DECEL,
    TRAP_GEN_DONE,
} TrapGenState;

typedef struct
{
    TrapGenState state;
    int32_t      target_pos;  // counts
    float        vel_max;     // counts/s, magnitude
    float        accel_max;   // counts/s^2, magnitude
    float        pos;         // counts, current generator position (float for sub-count precision)
    float        vel;         // counts/s, current generator velocity (signed)
} TrapGen;

// Start a new move from start_pos to target_pos. vel_max/accel_max are
// magnitudes (must be > 0) — direction is inferred from target vs. start.
void trap_gen_start(TrapGen *g, int32_t start_pos, int32_t target_pos,
                     int32_t vel_max, int32_t accel_max);

// Advance by dt seconds; writes the commanded position/velocity for this
// tick. Safe to keep calling after the move is DONE (holds at target,
// zero velocity).
void trap_gen_step(TrapGen *g, float dt, int32_t *pos_cmd, float *vel_cmd);

// True from trap_gen_start() until the move reaches DONE.
bool trap_gen_active(const TrapGen *g);

// trap_gen.h — real-time trapezoidal motion profile generator
//
// Computes a trapezoidal (or triangular, when the target is too close to
// reach vel_max) position/velocity profile analytically each call, from a
// target + velocity/acceleration limits -- not a precomputed sample vector.
// Design ported from trajectory-streamer's origin/main branch
// (motion/TrapGenerator.hpp/cpp, states IDLE/ACCEL/CRUISE/DECEL/DONE); this
// is a from-scratch C reimplementation of that state machine, not a
// line-for-line port of the C++.
//
// Units: counts (encoder counts) for position, counts/s for velocity,
// counts/s^2 for acceleration -- caller converts to/from radians.

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    int32_t start_pos;   // counts, captured at trap_gen_start()
    int32_t target_pos;  // counts
    float   direction;   // +1.0f or -1.0f
    float   distance;    // counts, |target_pos - start_pos|, as float

    float   t_accel;     // s -- duration of accel phase (== decel phase)
    float   t_cruise;    // s -- duration of cruise phase (0 if triangular)
    float   v_peak;      // counts/s -- peak velocity actually reached
                          // (== vel_max if trapezoidal, < vel_max if triangular)
    float   accel_max_f; // counts/s^2, as float (avoids repeated int->float casts)

    float   elapsed;     // s, since trap_gen_start()
    bool    active;      // false once the move is complete
} TrapGen;

// Start a new move. vel_max/accel_max are magnitudes -- must both be > 0
// or the move completes immediately (trap_gen_active() returns false right
// away, pos_cmd stays at start_pos). target_pos == start_pos also completes
// immediately (zero-distance move).
void trap_gen_start(TrapGen *g, int32_t start_pos, int32_t target_pos,
                     int32_t vel_max, int32_t accel_max);

// Advance by dt seconds, write the current commanded position/velocity.
// Returns true while the move is still in progress; once it returns false,
// pos_cmd_counts is exactly target_pos and vel_cmd_counts_s is 0 (a plain
// hold at the final commanded value, not a random mid-profile point --
// safe to keep calling this after completion, it just keeps returning the
// same final values).
bool trap_gen_step(TrapGen *g, float dt, int32_t *pos_cmd_counts, float *vel_cmd_counts_s);

bool trap_gen_active(const TrapGen *g);

// trap_gen.c — real-time trapezoidal motion profile generator
//
// See trap_gen.h for the design note (ported from trajectory-streamer's
// origin/main branch motion/TrapGenerator.hpp/cpp, from-scratch C
// reimplementation, not a line-for-line port).
//
// Standard trapezoidal-velocity-profile derivation:
//   distance covered accelerating 0 -> vel_max at accel_max:
//     d_accel = 0.5 * vel_max^2 / accel_max
//   if 2*d_accel >= total distance, vel_max is never reached -- triangular
//   profile instead, peak velocity v_peak = sqrt(distance * accel_max)
//   (from d/2 = 0.5*v_peak^2/accel_max), t_accel = v_peak/accel_max, no
//   cruise phase, t_total = 2*t_accel.
//   Otherwise (trapezoidal): t_accel = vel_max/accel_max,
//   d_cruise = distance - 2*d_accel, t_cruise = d_cruise/vel_max,
//   t_total = 2*t_accel + t_cruise.
//
// Position at elapsed time t (0 <= t <= t_total), relative to start,
// magnitude only (direction/start_pos applied by the caller-facing API):
//   accel phase   (t < t_accel):              0.5*accel_max*t^2
//   cruise phase  (t_accel <= t < t_accel+t_cruise):
//                                              d_accel + v_peak*(t-t_accel)
//   decel phase   (t_accel+t_cruise <= t < t_total): let td = t_total-t,
//                                              distance - 0.5*accel_max*td^2

#include "trap_gen.h"
#include <math.h>

void trap_gen_start(TrapGen *g, int32_t start_pos, int32_t target_pos,
                     int32_t vel_max, int32_t accel_max)
{
    g->start_pos  = start_pos;
    g->target_pos = target_pos;
    g->elapsed    = 0.0f;

    float distance = (float)target_pos - (float)start_pos;

    if (vel_max <= 0 || accel_max <= 0 || distance == 0.0f)
    {
        // Nothing to do -- immediately complete. When limits are invalid
        // (not just zero-distance), don't move at all: force target_pos
        // back to start_pos so the inactive-path report below (which always
        // reports target_pos) holds at start, not silently commands an
        // instant jump to wherever an invalid command asked for.
        g->target_pos  = start_pos;
        g->direction   = 1.0f;
        g->distance    = 0.0f;
        g->t_accel     = 0.0f;
        g->t_cruise    = 0.0f;
        g->v_peak      = 0.0f;
        g->accel_max_f = 0.0f;
        g->active      = false;
        return;
    }

    g->direction   = (distance > 0.0f) ? 1.0f : -1.0f;
    g->distance    = fabsf(distance);
    g->accel_max_f = (float)accel_max;

    float d_accel_full = 0.5f * (float)vel_max * (float)vel_max / (float)accel_max;

    if (2.0f * d_accel_full >= g->distance)
    {
        // Triangular -- vel_max is never reached.
        g->v_peak   = sqrtf(g->distance * (float)accel_max);
        g->t_accel  = g->v_peak / (float)accel_max;
        g->t_cruise = 0.0f;
    }
    else
    {
        // Trapezoidal.
        g->v_peak         = (float)vel_max;
        g->t_accel        = (float)vel_max / (float)accel_max;
        float d_cruise    = g->distance - 2.0f * d_accel_full;
        g->t_cruise       = d_cruise / (float)vel_max;
    }

    g->active = true;
}

bool trap_gen_step(TrapGen *g, float dt, int32_t *pos_cmd_counts, float *vel_cmd_counts_s)
{
    if (!g->active)
    {
        *pos_cmd_counts   = g->target_pos;
        *vel_cmd_counts_s = 0.0f;
        return false;
    }

    g->elapsed += dt;

    float t_total = 2.0f * g->t_accel + g->t_cruise;
    float pos_rel;   // magnitude, counts, relative to start_pos
    float vel_mag;   // magnitude, counts/s

    if (g->elapsed >= t_total)
    {
        // Done -- land exactly on target, not wherever the last partial
        // tick's math would put it (avoids float-accumulation drift).
        *pos_cmd_counts   = g->target_pos;
        *vel_cmd_counts_s = 0.0f;
        g->active         = false;
        return false;
    }

    if (g->elapsed < g->t_accel)
    {
        // Accel phase.
        pos_rel = 0.5f * g->accel_max_f * g->elapsed * g->elapsed;
        vel_mag = g->accel_max_f * g->elapsed;
    }
    else if (g->elapsed < g->t_accel + g->t_cruise)
    {
        // Cruise phase.
        float d_accel = 0.5f * g->accel_max_f * g->t_accel * g->t_accel;
        pos_rel = d_accel + g->v_peak * (g->elapsed - g->t_accel);
        vel_mag = g->v_peak;
    }
    else
    {
        // Decel phase.
        float td = t_total - g->elapsed;
        pos_rel = g->distance - 0.5f * g->accel_max_f * td * td;
        vel_mag = g->accel_max_f * td;
    }

    *pos_cmd_counts   = g->start_pos + (int32_t)(g->direction * pos_rel);
    *vel_cmd_counts_s = g->direction * vel_mag;

    return true;
}

bool trap_gen_active(const TrapGen *g)
{
    return g->active;
}

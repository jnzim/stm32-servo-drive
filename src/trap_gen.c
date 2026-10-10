// trap_gen.c — see trap_gen.h for usage.
//
// Braking-distance method: each tick, ask "if I decelerated at accel_max
// starting now, would I stop exactly at (or past) the target?" If yes,
// decelerate. If no and below vel_max, accelerate. Otherwise hold at
// vel_max (cruise). This needs no precomputed phase durations and
// automatically produces a triangular (no-cruise) profile for moves too
// short to reach vel_max — the same decision, made fresh every tick.

#include "trap_gen.h"

#include <math.h>

void trap_gen_start(TrapGen *g, int32_t start_pos, int32_t target_pos,
                     int32_t vel_max, int32_t accel_max)
{
    g->target_pos = target_pos;
    g->vel_max    = (float)vel_max;
    g->accel_max  = (float)accel_max;
    g->pos        = (float)start_pos;
    g->vel        = 0.0f;
    g->state      = (start_pos == target_pos) ? TRAP_GEN_DONE : TRAP_GEN_ACCEL;
}

void trap_gen_step(TrapGen *g, float dt, int32_t *pos_cmd, float *vel_cmd)
{
    if (g->state == TRAP_GEN_DONE || g->state == TRAP_GEN_IDLE)
    {
        *pos_cmd = g->target_pos;
        *vel_cmd = 0.0f;
        return;
    }

    float direction = (g->target_pos >= (int32_t)g->pos) ? 1.0f : -1.0f;
    float remaining = direction * ((float)g->target_pos - g->pos);   // > 0 while short of target
    float speed     = direction * g->vel;                             // signed vel, positive = toward target

    float decel_dist = (speed > 0.0f) ? (speed * speed) / (2.0f * g->accel_max) : 0.0f;

    if (remaining <= decel_dist)
    {
        g->state = TRAP_GEN_DECEL;
        speed -= g->accel_max * dt;
        if (speed < 0.0f) speed = 0.0f;
    }
    else if (speed < g->vel_max)
    {
        g->state = TRAP_GEN_ACCEL;
        speed += g->accel_max * dt;
        if (speed > g->vel_max) speed = g->vel_max;
    }
    else
    {
        g->state = TRAP_GEN_CRUISE;
        speed = g->vel_max;
    }

    g->vel  = direction * speed;
    g->pos += g->vel * dt;

    // Snap to target once the remaining distance is smaller than this
    // tick's own travel and speed has decayed near zero -- avoids
    // overshoot/settling oscillation from stepping past the target in a
    // single tick right at the end of DECEL.
    float new_remaining = direction * ((float)g->target_pos - g->pos);
    if (new_remaining <= 0.0f || (speed < 1.0f && new_remaining < fabsf(g->vel * dt)))
    {
        g->pos   = (float)g->target_pos;
        g->vel   = 0.0f;
        g->state = TRAP_GEN_DONE;
    }

    *pos_cmd = (int32_t)lroundf(g->pos);
    *vel_cmd = g->vel;
}

bool trap_gen_active(const TrapGen *g)
{
    return g->state != TRAP_GEN_IDLE && g->state != TRAP_GEN_DONE;
}

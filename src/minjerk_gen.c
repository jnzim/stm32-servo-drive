// minjerk_gen.c — see minjerk_gen.h for the profile and its derivation.

#include "minjerk_gen.h"

#include <math.h>

#define MINJERK_PEAK_VEL_COEF    1.875f      // max s'(tau),  at tau = 1/2
#define MINJERK_PEAK_ACCEL_COEF  5.773503f   // max s''(tau), 10/sqrt(3)

void minjerk_gen_start(MinJerkGen *g, int32_t start_pos, int32_t target_pos,
                       int32_t vel_max, int32_t accel_max)
{
    g->start_pos  = start_pos;
    g->target_pos = target_pos;
    g->dist       = (float)(target_pos - start_pos);
    g->tick       = 0u;
    g->pos        = (float)start_pos;
    g->vel        = 0.0f;

    float d       = fabsf(g->dist);
    float t_vel   = MINJERK_PEAK_VEL_COEF * d / (float)vel_max;
    float t_accel = sqrtf(MINJERK_PEAK_ACCEL_COEF * d / (float)accel_max);
    g->duration   = (t_vel > t_accel) ? t_vel : t_accel;

    g->active     = (start_pos != target_pos);
}

void minjerk_gen_step(MinJerkGen *g, float dt, int32_t *pos_cmd, float *vel_cmd)
{
    if (!g->active)
    {
        *pos_cmd = g->target_pos;
        *vel_cmd = 0.0f;
        return;
    }

    g->tick++;

    // Double, not float, for tau and the polynomial. Near the end of a move
    // the position advances by a tiny fraction of a count per tick, and float
    // rounding at a few hundred thousand counts is larger than that -- so the
    // rounded command could step back one count. In double the error (~1e-11
    // counts) stays far below the smallest per-tick step. The F411 FPU is
    // single-precision, so this is software double, once per 1 kHz tick.
    double tau = ((double)g->tick * (double)dt) / (double)g->duration;

    if (tau >= 1.0)
    {
        // End exactly on target: s(1) = 1 and s'(1) = 0 analytically, but
        // evaluating the polynomial would leave a fraction of a count.
        g->pos    = (float)g->target_pos;
        g->vel    = 0.0f;
        g->active = false;
        *pos_cmd  = g->target_pos;
        *vel_cmd  = 0.0f;
        return;
    }

    double tau2  = tau * tau;
    double tau3  = tau2 * tau;
    double s     = tau3 * (10.0 + tau * (-15.0 + 6.0 * tau));   // 10t^3 - 15t^4 + 6t^5
    double s_dot = tau2 * (30.0 + tau * (-60.0 + 30.0 * tau));  // 30t^2 - 60t^3 + 30t^4
    double pos   = (double)g->start_pos + (double)g->dist * s;

    g->pos = (float)pos;
    g->vel = (float)((double)g->dist * s_dot / (double)g->duration);  // d/dt = (1/T) d/dtau

    *pos_cmd = (int32_t)lround(pos);
    *vel_cmd = g->vel;
}

bool minjerk_gen_active(const MinJerkGen *g)
{
    return g->active;
}

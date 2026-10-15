#include "observer.h"
#include "config.h"
#include "encoder.h"

#include <math.h>
#include <stdbool.h>

static float    theta_hat;      // [rad]     relative to theta_zero
static float    omega_hat;      // [rad/s]
static float    disturbance_hat;       // [rad/s^2]

static int32_t  counts_zero;    // encoder count at init

static float    gain_pos, gain_vel, gain_dist;
static bool     primed;

/* TIM1_UP is a hardware timer, so the control tick is exactly 50us with no
 * jitter worth modelling. What is NOT guaranteed is that observer_update()
 * runs on every one of them: it sits in the sysid telemetry section, and each
 * run_* function returns early on !current_feedback_sample_valid() before
 * reaching it. A skipped call with a hardcoded dt makes the model
 * under-integrate, and on an integrator that accumulates. So take dt from the
 * free-running tick counter instead of assuming consecutive calls. */
extern volatile uint32_t g_control_tick;   /* main.c, 20kHz */

static const float TICK_SECONDS = DT_CURRENT;    /* 50us per tick */
static uint32_t    last_tick;

void observer_init(void)
{
    const float wo_rad_s = 2.0f * M_PI * OBS_WO_HZ;

    /* Triple pole at wo_rad_s: the characteristic polynomial of the 3-state
     * estimator becomes (s + wo_rad_s)^3 = s^3 + 3*wo_rad_s*s^2 + 3*wo_rad_s^2*s + wo_rad_s^3, so the
     * gains are its coefficients. Equal poles is the default choice when
     * there is no reason to prefer one state's convergence over another's. */
    gain_pos = 3.0f * wo_rad_s;
    gain_vel = 3.0f * wo_rad_s * wo_rad_s;
    gain_dist = wo_rad_s * wo_rad_s * wo_rad_s;

    last_tick   = 0u;
    theta_hat   = 0.0f;
    omega_hat   = 0.0f;
    disturbance_hat    = 0.0f;
    counts_zero = 0;
    primed      = false;
}

void observer_update(int32_t counts, float iq_meas)
{
    /* Position is kept relative to the first sample. Absolute encoder counts
     * grow without bound, and a float carrying a large integer part loses the
     * fractional resolution that makes the estimate worth having. Re-basing
     * costs nothing because only differences matter downstream. This zero is
     * the observer's own, independent of the servo's absolute zero. */
    if (!primed)
    {
        counts_zero = counts;
        last_tick   = g_control_tick;
        primed      = true;
        return;
    }

    const uint32_t now     = g_control_tick;
    const uint32_t elapsed = now - last_tick;   /* unsigned: wrap-safe */
    last_tick              = now;

    if (elapsed == 0u) { return; }              /* called twice in one tick */

    const float theta_meas = (float)(counts - counts_zero) / COUNTS_PER_RAD;

    /* A long gap is a stage change or a stall, not a sample interval. Don't
     * integrate across it, and don't carry the stale position into the next
     * update either -- theta_hat would be far from the encoder and the correction
     * would slam. Resync position to the measurement and keep the
     * velocity and disturbance estimates, which are still the best available. */
    if (elapsed > 20u)
    {
        theta_hat = theta_meas;
        return;
    }

    const float dt        = (float)elapsed * TICK_SECONDS;
    const float pos_error = theta_meas - theta_hat;

    /* Model acceleration: commanded torque less the viscous term the friction
     * sweep already measured, less whatever disturbance_hat has learned. */
    const float model_accel = OBS_K_ACC * (iq_meas - OBS_B_VISC * omega_hat) - disturbance_hat;

    /* Euler integration of each state's derivative. The gain*pos_error terms are rate
     * corrections, so they sit inside the brackets alongside the model terms
     * and carry that state's derivative units. Order matters only to the
     * extent that every update must use the SAME pos_error, so it is
     * computed once above and all three states advance from it. */
    const float theta_rate       =  omega_hat   + gain_pos  * pos_error;
    const float omega_rate       =  model_accel + gain_vel  * pos_error;
    const float disturbance_rate =              - gain_dist * pos_error;

    theta_hat       += dt * theta_rate;
    omega_hat       += dt * omega_rate;
    disturbance_hat += dt * disturbance_rate;
}

float observer_get_velocity(void)    { return omega_hat; }

float velocity_feedback_rad(int32_t boxcar_counts)
{
#if VEL_LOOP_USES_OBSERVER
    (void)boxcar_counts;
    return omega_hat;
#else
    return (float)boxcar_counts / COUNTS_PER_RAD;
#endif
}

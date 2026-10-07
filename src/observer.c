#include "observer.h"
#include "config.h"
#include "encoder.h"

#include <math.h>
#include <stdbool.h>

static float    theta_hat;      // [rad]     relative to theta_zero
static float    omega_hat;      // [rad/s]
static float    dist_hat;       // [rad/s^2]

static int32_t  counts_zero;    // encoder count at init

static float    l1, l2, l3;
static bool     primed;

static const float TS = DT_CURRENT;   // 50us, the 20kHz control tick

void observer_init(void)
{
    const float wo = 2.0f * M_PI * OBS_WO_HZ;

    /* Triple pole at wo: the characteristic polynomial of the 3-state
     * estimator becomes (s + wo)^3 = s^3 + 3*wo*s^2 + 3*wo^2*s + wo^3, so the
     * gains are its coefficients. Equal poles is the default choice when
     * there is no reason to prefer one state's convergence over another's. */
    l1 = 3.0f * wo;
    l2 = 3.0f * wo * wo;
    l3 = wo * wo * wo;

    theta_hat   = 0.0f;
    omega_hat   = 0.0f;
    dist_hat    = 0.0f;
    counts_zero = 0;
    primed      = false;
}

void observer_update(int32_t counts, float iq_meas)
{
    /* Position is kept relative to the first sample. Absolute encoder counts
     * grow without bound, and a float carrying a large integer part loses the
     * fractional resolution that makes the estimate worth having. Re-basing
     * costs nothing because only differences matter downstream. */
    if (!primed)
    {
        counts_zero = counts;
        primed      = true;
        return;
    }

    const float theta_meas = (float)(counts - counts_zero) / COUNTS_PER_RAD;
    const float e          = theta_meas - theta_hat;

    /* Model acceleration: commanded torque less the viscous term the friction
     * sweep already measured, less whatever dist_hat has learned. */
    const float accel = OBS_K_ACC * (iq_meas - OBS_B_VISC * omega_hat) - dist_hat;

    /* Euler integration of each state's derivative. The l*e terms are rate
     * corrections, so they sit inside the brackets alongside the model terms
     * and carry that state's derivative units. Order matters only to the
     * extent that every update must use the SAME e, so e is computed once
     * above and the states are advanced from it. */
    const float d_theta = omega_hat + l1 * e;
    const float d_omega = accel     + l2 * e;
    const float d_dist  =           - l3 * e;

    theta_hat += TS * d_theta;
    omega_hat += TS * d_omega;
    dist_hat  += TS * d_dist;
}

float observer_get_velocity(void)    { return omega_hat; }
float observer_get_position(void)    { return theta_hat; }
float observer_get_disturbance(void) { return dist_hat;  }

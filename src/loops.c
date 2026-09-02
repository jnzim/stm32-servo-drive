#include "loops.h"
#include "control.h"
#include "spi.h"
#include "pwm.h"
#include <math.h>
#include "config.h"


// Zero-cancellation design from the fitted mechanical plant, re-identified
// with the stage attached (K=1623.529 rad/s per A, tau=123.67ms -- a much
// heavier/more damped plant than the bare-motor fit this replaces) at a
// 50 Hz BW target -- see velocity_bode_plot.py. Nominal PM = 90 deg from
// the pole-zero-cancelled open loop, but measured phase in the 40-90Hz
// crossover-adjacent band sits at -70 to -75 deg against a fitted -85 to
// -88 deg there (tight, repeatable, high coherence -- not noise), so the
// first-order model isn't a complete description right where 50Hz sits.
// Unverified against closed-loop measurement -- run SYSID_TEST_CL_VEL_CHIRP
// and check the real PM before trusting this like the position loop's note
// above.
#define VEL_KP              0.0239f    // A / (rad/s)
#define VEL_KI              0.1935f    // A / rad
#define VEL_IQ_LIMIT        0.5f


// ── Reset after each move ─────────────────────────────────────────────────────
volatile uint32_t samples_consumed = 0;

// ── Controller state ──────────────────────────────────────────────────────────
PIState current_loop;
PIState d_current_loop;
PIState velocity_loop;
PState  position_loop;

// ── Inter-loop setpoints ──────────────────────────────────────────────────────
float          vel_cmd_rad_sec  = 0.0f;
float          iq_cmd           = 0.0f;
volatile float v_q_cmd          = 0.0f; 

// ── Rate dividers ─────────────────────────────────────────────────────────────
uint32_t vel_div = 0;

// ── Sequencing ────────────────────────────────────────────────────────────────
volatile uint8_t first_sample_ready = 0;

// ─────────────────────────────────────────────────────────────────────────────
// loops_reset — called from drive.c on SERVO_ON entry
// ─────────────────────────────────────────────────────────────────────────────
void loops_reset(void)
{


    // Output clamp was +-10V, but volts_to_duty() physically can't produce
    // more than +-V_BUS/2 per phase (6V @ V_BUS=12V) -- the PI's own
    // anti-windup never saw real saturation until 10V, while pwm.c was
    // silently clipping at 6V the whole time. Clamp now matches the actual
    // achievable limit so the integrator stops winding up past what the
    // hardware can really deliver.
    //
    // See config.h (CURRENT_LOOP_KP/KI) for the gain derivation.
    pi_init(&current_loop, CURRENT_LOOP_KP, CURRENT_LOOP_KI, -(V_BUS / 2.0f), (V_BUS / 2.0f));
    // Same gains as the q-axis current loop -- Ld ~= Lq for this machine, so
    // the d-axis plant (id/vd) is the same R/L electrical dynamics. Cancels
    // the id = we*Lq*iq/R cross-coupling that shows up when vd is forced to
    // zero (see foc-sysid RIPPLE_DEBUG id_mean investigation).
    pi_init(&d_current_loop, CURRENT_LOOP_KP, CURRENT_LOOP_KI, -(V_BUS / 2.0f), (V_BUS / 2.0f));
    pi_init(&velocity_loop, VEL_KP, VEL_KI, -VEL_IQ_LIMIT, VEL_IQ_LIMIT);


    p_init(&position_loop, POSITION_LOOP_KP);

    vel_cmd_rad_sec    = 0.0f;
    iq_cmd             = 0.0f;
    v_q_cmd            = 0.0f;
    vel_div            = 0;
    samples_consumed   = 0;
    first_sample_ready = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// open_loop_step — called from TIM1 ISR when STATE_OPEN_LOOP active
//
// Advances electrical angle by d_theta per call, applies rotating voltage
// vector at magnitude v_mag with v_d=0. No closed-loop feedback.
// ─────────────────────────────────────────────────────────────────────────────
void open_loop_step(float *theta, float v_mag, float d_theta)
{
    *theta += d_theta;
    if (*theta >= 2.0f * M_PI) *theta -= 2.0f * M_PI;
    pwm_apply_dq(0.0f, v_mag, *theta);
}
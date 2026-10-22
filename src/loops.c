#include "loops.h"
#include "control.h"
#include "config.h"
#include <math.h>

// ── Reset after each move ─────────────────────────────────────────────────────
volatile uint32_t samples_consumed = 0;

// ── Controller state ──────────────────────────────────────────────────────────
PIState current_loop;
PIState d_current_loop;
PIState velocity_loop;
PState  position_loop;

// ── Inter-loop setpoints ──────────────────────────────────────────────────────
volatile float vel_cmd_rad_sec  = 0.0f;
float          iq_cmd           = 0.0f;

// ── Rate dividers ─────────────────────────────────────────────────────────────
uint32_t loop_div = 0;

// ── Sequencing ────────────────────────────────────────────────────────────────
volatile uint8_t first_sample_ready = 0;

// ─────────────────────────────────────────────────────────────────────────────
// loops_reset — load the gains and zero every loop state and setpoint.
// Called on SERVO_ON entry (drive.c) and at the start of trajectory and
// sysid runs.
// ─────────────────────────────────────────────────────────────────────────────
void loops_reset(void)
{
    // Current PI clamp is +-V_BUS/2 (+-12V at 24V), the most volts_to_duty()
    // can put on a phase, so the integrator cannot wind up past what the
    // bridge delivers. Gains: config.h (CURRENT_LOOP_KP/KI, VEL_KP/KI).
    pi_init(&current_loop, CURRENT_LOOP_KP, CURRENT_LOOP_KI, -(V_BUS / 2.0f), (V_BUS / 2.0f));
    pi_init(&d_current_loop, CURRENT_LOOP_KP, CURRENT_LOOP_KI, -(V_BUS / 2.0f), (V_BUS / 2.0f));
    pi_init(&velocity_loop, VEL_KP, VEL_KI, -VEL_IQ_LIMIT, VEL_IQ_LIMIT);
    p_init(&position_loop, POSITION_LOOP_KP);

    vel_cmd_rad_sec    = 0.0f;
    iq_cmd             = 0.0f;
    loop_div           = 0;
    samples_consumed   = 0;
    first_sample_ready = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// velocity_loop_step — velocity PI plus friction feedforward. Used by both
// the trajectory path and the sysid tests.
//
// Feedforward = Coulomb * tanh(vel_cmd / smooth) + viscous * vel_cmd, from
// SYSID_TEST_FRICTION_SWEEP (config.h FRICTION_FF_*). It is driven by the
// command, not the measurement. tanh instead of sign() so the Coulomb term
// does not chatter at zero velocity.
// ─────────────────────────────────────────────────────────────────────────────
float velocity_loop_step(float vel_cmd_rad_sec, float vel_meas_rad, float dt)
{
    float iq = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, dt);

#if FRICTION_FF_ENABLE
    float ff_sign = tanhf(vel_cmd_rad_sec / FRICTION_FF_SMOOTH_RAD_S);
    iq += (FRICTION_FF_COULOMB_MA * 0.001f) * ff_sign +
          (FRICTION_FF_VISCOUS_MA_PER_RADS * 0.001f) * vel_cmd_rad_sec;
#endif

    return iq;
}

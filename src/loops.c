#include "loops.h"
#include "control.h"
#include "spi.h"
#include "pwm.h"
#include <math.h>
#include "config.h"


// Zero-cancellation design against the plant backed out of the P-only
// SYSID_TEST_CL_VEL_CHIRP run at CL_VEL_CHIRP_AMPLITUDE=15 rad/s (K=190.345
// rad/s/A, tau=12.44ms, bare motor, archived
// sysid_log_CL_Ponly_15rads_bare_20260904_1057.csv) -- the cleanest
// measurement of this plant so far (coherence ~1.0 out to ~150Hz) and the
// closest match to the original archived bare-motor OL fit
// (K=242.4, tau=12.92ms), within ~4% on tau/fc. 15rad/s specifically
// because it's the largest CL_VEL_CHIRP_AMPLITUDE that keeps peak
// commanded iq (~Kp_Ponly*amplitude) under VEL_IQ_LIMIT=0.5A with the
// P-only Kp=0.0239 used for that identification run -- 30rad/s clamped
// (peak ~0.72A > 0.5A) and gave a visibly worse, inconsistent fit.
// BW target 50Hz, same convention as the rest of this project
// (current-loop BW / 10).
// Closed-loop verified (SYSID_TEST_CL_VEL_CHIRP, bare motor,
// CL_VEL_CHIRP_AMPLITUDE=15 rad/s): DC gain -0.01dB (true unity, integrator
// working), measured -3dB BW=80.4Hz (above the 50Hz target). Loop's own
// backed-out margin (L=H/(1-H), valid for any C(s)) is gc=44.6Hz,
// PM=69.2 deg, GM=15.1dB -- short of the idealized 90deg a perfect
// pole-zero cancellation assumes, real extra phase lag not yet isolated
// (leading candidates: VEL_FILTER_N group delay, the current loop's own
// ~300Hz dynamics, single-pole plant model being incomplete). PM<90deg is
// exactly why measured BW (80.4Hz) exceeds gc (44.6Hz) -- mild peaking
// near crossover, not an error. Healthy margins either way.
// 2026-09-19, loaded machine, measured with VEL_FILTER_N=20:
//   Kp 0.02053 -> 0.0450 (2.2x), crossover 40Hz -> ~93Hz, PM 97.6deg -> ~70deg.
//   The old gain left the velocity loop crossing over at only 1.2x the position
//   loop's 32.5Hz -- the usual guideline is 3-5x, and that phase shortfall is
//   what capped the position loop at 47deg PM. Ki scales with Kp to keep the PI
//   zero on the measured 11.2Hz plant pole: Ki = Kp * 2*pi*11.2.
#define VEL_KP              0.04500f   // A / (rad/s)
#define VEL_KI              3.1700f    // A / rad -- PI zero on the LOADED plant pole
                                       // (2026-09-19): Ki = Kp * 2*pi*11.2Hz. The old 1.6505 put
                                       // the zero at 12.8Hz against a pole measured at 11.2Hz, and
                                       // the leftover pole-zero pair showed as a ~1dB shelf from
                                       // 10Hz out to crossover in the closed-loop magnitude.
// 1.5A, raised from the 0.5A bring-up placeholder (2026-09-19). With the load
// attached, 0.5A clamped the velocity PI for ~20% of a position chirp (iq peak
// 0.709A -- the friction feedforward is added after this clamp, so it is not
// the ceiling it looks like), which saturated the measurement: crossover read
// 25.9Hz / PM 48.9deg against a 33.8Hz / 60deg design. Motor is rated 2.91A.
#define VEL_IQ_LIMIT        1.5f


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
// velocity_loop_step — velocity PI + friction feedforward, single source of
// truth so every caller (sysid tests, real trajectory mode) gets the same
// friction compensation instead of duplicating it inline. Fc/Fv are from
// SYSID_TEST_FRICTION_SWEEP (stage attached, averaged across 3 repeat runs);
// tanh() replaces a hard sign(vel_cmd) so this doesn't chatter full Coulomb
// torque back and forth right at v=0, which every real position move
// crosses through on its final settle. Feedforward is based on vel_cmd (the
// reference), not vel_meas -- it supplies the expected disturbance-
// canceling torque for where the loop is trying to go; the PI term already
// reacts to actual measured error.
// ─────────────────────────────────────────────────────────────────────────────
float velocity_loop_step(float vel_cmd_rad_sec, float vel_meas_rad, float dt)
{
    float iq = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, dt);

    float ff_sign = tanhf(vel_cmd_rad_sec / FRICTION_FF_SMOOTH_RAD_S);
    iq += (FRICTION_FF_COULOMB_MA * 0.001f) * ff_sign +
          (FRICTION_FF_VISCOUS_MA_PER_RADS * 0.001f) * vel_cmd_rad_sec;

    return iq;
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
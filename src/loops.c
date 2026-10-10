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
// 2026-10-06, velocity loop closed on the observer instead of the encoder
// boxcar: Kp 0.0450 -> 0.0900, Ki 3.17 -> 6.34 (2x both, so the PI zero stays
// on the 11.2Hz plant pole).
//
// The observer handed back +31 deg of phase margin at crossover -- the whole
// group delay of the 20-tap boxcar (0.36 deg/Hz, 31 deg at 93Hz). Measured
// through the signal each loop actually regulates:
//
//   loop on boxcar    gc  93.4Hz   PM 68.5 deg   -3dB 168.8Hz
//   loop on observer  gc 104.1Hz   PM 99.5 deg   -3dB  86.1Hz
//
// Note the bandwidth DROPPED. With the gains unchanged, recovered phase does
// not make the loop faster, it makes it overdamped -- at 99.5 deg PM the
// closed-loop response is sluggish relative to its own crossover, which is why
// -3dB fell below gc. Measured through the same boxcar sensor in both runs,
// physical tracking went 168.8 -> 83.6Hz.
//
// So the margin is the deliverable and it was sitting unspent against a 60 deg
// design target. 2x converts it: crossover scales with Kp, phase lag grows with
// frequency, and PM should land back in the 60s with gc near 200Hz.
//
// Revert pair if this is too hot: 0.04500 / 3.1700, which is the configuration
// measured above at PM 99.5 deg.
#define VEL_KP              0.09000f   // A / (rad/s)
#define VEL_KI              6.3400f    // A / rad -- PI zero on the LOADED plant pole
                                       // (2026-09-19): Ki = Kp * 2*pi*11.2Hz. The old 1.6505 put
                                       // the zero at 12.8Hz against a pole measured at 11.2Hz, and
                                       // the leftover pole-zero pair showed as a ~1dB shelf from
                                       // 10Hz out to crossover in the closed-loop magnitude.
// 1.5A, raised from the 0.5A bring-up placeholder (2026-09-19). With the load
// attached, 0.5A clamped the velocity PI for ~20% of a position chirp (iq peak
// 0.709A -- the friction feedforward is added after this clamp, so it is not
// the ceiling it looks like), which saturated the measurement: crossover read
// 25.9Hz / PM 48.9deg against a 33.8Hz / 60deg design. Motor is rated 2.91A.
// 2.0 -> 2.5 -> 3.5A (2026-10-06). 2.5 still clipped.
//
// 2.91A is the CONTINUOUS rating, which constrains RMS -- a thermal limit --
// not transient peaks. Measured on the 0.15 rad CL_POS_CHIRP that was clipping:
//
//     iq RMS 0.734 A   peak 3.025 A   only 1.17% of samples above 2.4 A
//
// So RMS sits 4x under the continuous rating. 3.5A is 1.2x continuous for
// peaks, against a servo motor's typical ~3x short-burst peak capability, and
// is still inside the current-sense range (+-2048 counts at 2.88 mA/count
// saturates near 5.9A, so the measurement stays honest).
//
// Check the bench supply allows it: 3.5A peaks at 24V.
//
// 2.0 was itself a bring-up value, not a thermal or hardware limit. It became
// the binding constraint once POSITION_VEL_LIMIT went 50 -> 80: a 0.15 rad
// CL_POS_CHIRP then hit vel_cmd's clamp 2.98% of samples AND iq's 2.71%, and
// the measured bandwidth barely moved (83.4 -> 86.6Hz) while phase margin fell
// to 58.6deg. Raising one clamp had just handed the bottleneck to the next one
// down the chain.
//
// vq peaks 11711mV against its own 12000mV clamp with 0% of samples near it, so
// the 24V bus is not the limit here.
#define VEL_IQ_LIMIT        3.5f


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
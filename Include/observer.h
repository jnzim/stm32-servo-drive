#ifndef OBSERVER_H
#define OBSERVER_H

#include <stdint.h>

#include "config.h"   /* FRICTION_FF_VISCOUS_MA_PER_RADS */

// =============================================================================
// Velocity observer -- a replacement for differentiating the encoder.
// =============================================================================
//
// The problem it solves: encoder_get_velocity() is a VEL_FILTER_N=20 boxcar
// over count differences. A moving average has group delay (N-1)/2 regardless
// of anything else -- 0.36 deg/Hz here, which is 14 deg at a 40Hz velocity
// crossover and 31 deg at the 95Hz crossover actually measured. That is more
// than five times what the current loop costs at the same frequency, so it,
// not the inner loop, is what limits velocity and position bandwidth.
//
// Shortening the window does not help: the window length IS the resolution.
// At N=20 (1ms) one count is 0.767 rad/s; at N=1 (50us) it is 15.3 rad/s,
// which is the entire amplitude of a typical chirp. Window length trades
// resolution against delay and there is no setting that wins.
//
// So stop differentiating. This runs a model of the mechanics forward using
// the torque already being commanded, and uses the encoder only to correct
// the model's drift. The high-frequency content of the velocity estimate then
// comes from a signal that is known exactly and has no delay, rather than from
// differencing a quantized position. That is why an observer beats a filter
// instead of merely being a better one.
//
// States, all in SI, position relative to wherever observer_init() was called:
//
//     theta_hat [rad]      estimated position
//     omega_hat [rad/s]    estimated velocity   <- the useful output
//     disturbance_hat  [rad/s^2]  estimated disturbance acceleration
//
// Update, with pos_error = theta_meas - theta_hat:
//
//     theta_hat += Ts*( omega_hat                            + gain_pos*pos_error )
//     omega_hat += Ts*( K_ACC*(iq - B_VISC*omega_hat) - disturbance_hat + gain_vel*pos_error )
//     disturbance_hat  += Ts*(                                      - gain_dist*pos_error )
//
// Every term inside a bracket has the units of that state's derivative; that
// is the check that the gains are dimensionally right. The l*pos_error terms are
// corrections injected as RATES, not writes to the state -- this is a control
// loop whose plant is the model and whose setpoint is the sensor.
//
// disturbance_hat is the reason a wrong inertia is survivable. It is an integrator,
// so it converges onto whatever constant acceleration the model is missing --
// Coulomb friction, load torque, or an error in K_ACC itself -- and cancels
// it. Get K_ACC wrong and the estimate degrades toward what a plain tracking
// observer gives; it does not diverge.

// Runs the observer and logs it. Independent of whether the loop consumes it
// (see VEL_LOOP_USES_OBSERVER below) -- with this on, the boxcar and the
// observer are both in every frame, so one capture compares them whichever is
// driving the loop.
#define OBSERVER_ENABLE 1

// Kt/J [rad/s^2 per A] -- acceleration per amp of q-axis current.
//
// Not measured directly. For a first-order mechanical plant w/i = K/(tau*s+1)
// you have K = Kt/B and tau = J/B, so K/tau = Kt/J -- and that RATIO is far
// better conditioned than either parameter. Two P-only identifications a month
// apart:
//
//     2026-09-19 : K=177.2, tau=14.19ms  ->  12488
//     2026-10-06 : K= 79.25, tau= 6.66ms ->  11901
//
// K and tau differ by more than 2x between them; K/tau agrees to 5%.
//
// A properly identified value does NOT work here. 11900 comes from the only
// valid back-out (DEPLOYED=0, where C(s) really is the constant the identity
// assumes); 19800 came from a DEPLOYED=1 run, which config.h explicitly warns
// is meaningless because C(s) is a PI plus a feedforward path. So the number in
// use has an invalid derivation and the correct one is unusable: at 11900 the
// velocity loop went unstable, swinging +-52 rad/s against a +-15 rad/s command
// and driving the stage into its hard stops (2026-10-06).
//
// The mechanism is not understood. Naively a smaller model term should lean
// the estimate harder on the encoder correction and behave more like the old
// filter, not less stably.
//
// What it does establish: K_ACC is not just a model constant. The observer sits
// inside the velocity feedback path, so this is a LOOP parameter and changing
// it changes stability. Validate any change at small amplitude and short
// duration with the stage centred, before a full sweep.
#define OBS_K_ACC   19800.0f

// Viscous damping as a current, derived from the friction sweep's own constant
// rather than written out again -- it is the same physical quantity, and two
// copies in different units is one edit away from disagreeing.
#define OBS_B_VISC  (FRICTION_FF_VISCOUS_MA_PER_RADS * 0.001f)   // A per rad/s

// Observer bandwidth [Hz] -- the single tuning knob, placing all three poles
// together. High: tracks the encoder hard and inherits its quantization noise.
// Low: trusts the model, smooth but slow onto real disturbances.
//
// 300 -> 800 (2026-10-06). The velocity loop caught up with the estimator. At
// 2x velocity gains its crossover reached 170Hz, which is only 1.8x the 300Hz
// observer -- the usual requirement is 3-5x, or the estimator's own dynamics
// sit inside the loop's bandwidth and omega_hat stops tracking the real motion
// where it matters. That showed up as a loop which looked excellent measured
// through the observer (gc 170Hz, PM 103deg) while real motion, measured
// through the encoder, got slightly WORSE than the boxcar baseline
// (168.8 -> 148.3Hz). Regulating an estimate only works while the estimate is
// right. 800Hz restores ~5x separation.
//
// The cost is inheriting more encoder quantization, which shows up as
// roughness at low speed -- check POSITION_STEP for buzz at rest if this gets
// raised further.
#define OBS_WO_HZ   800.0f

// Selects what the VELOCITY LOOP consumes. 0 = the encoder boxcar, i.e. the
// long-standing behaviour. 1 = the observer.
//
// Telemetry is deliberately unaffected: vel_meas_counts keeps carrying the
// boxcar, so a capture still logs both estimates side by side regardless of
// which one is in the loop. That is what makes the A/B honest -- flipping this
// changes the control path and nothing else.
#define VEL_LOOP_USES_OBSERVER 1

void  observer_init(void);

// Velocity feedback [rad/s] for the loop, selected by VEL_LOOP_USES_OBSERVER.
// Takes the boxcar's counts/s so the caller does not need to know which source
// is live.
float velocity_feedback_rad(int32_t boxcar_counts);

// Call once per control tick from TIM1_UP, after encoder_update().
//   counts  -- raw encoder position, same units as encoder_get_position()
//   iq_meas -- measured q-axis current [A]
void  observer_update(int32_t counts, float iq_meas);

float observer_get_velocity(void);      // [rad/s]
float observer_get_position(void);      // [rad], relative to init
float observer_get_disturbance(void);   // [rad/s^2]

#endif /* OBSERVER_H */

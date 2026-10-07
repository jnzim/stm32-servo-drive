#ifndef OBSERVER_H
#define OBSERVER_H

#include <stdint.h>

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
//     dist_hat  [rad/s^2]  estimated disturbance acceleration
//
// Update, with e = theta_meas - theta_hat:
//
//     theta_hat += Ts*( omega_hat                            + l1*e )
//     omega_hat += Ts*( K_ACC*(iq - B_VISC*omega_hat) - dist_hat + l2*e )
//     dist_hat  += Ts*(                                      - l3*e )
//
// Every term inside a bracket has the units of that state's derivative; that
// is the check that the gains are dimensionally right. The l*e terms are
// corrections injected as RATES, not writes to the state -- this is a control
// loop whose plant is the model and whose setpoint is the sensor.
//
// dist_hat is the reason a wrong inertia is survivable. It is an integrator,
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
// you have K = Kt/B and tau = J/B, so K/tau = Kt/J, and that RATIO is far
// better conditioned than either parameter:
//
//     backed out of CL_VEL_CHIRP 2026-10-06 : K=684.6, tau=34.5ms -> 19850
//     loops.c loaded fit                    : K=242.4, tau=12.9ms -> 18760
//     open-loop mechanical chirp            : K=58.0,  tau=4.1ms  -> 14140
//
// K and tau disagree across those by up to 1000%; K/tau agrees to 6% between
// the two loaded fits. 19800 is the starting value. A dedicated
// accelerate-then-coast test would refine it (the sum of the accelerating and
// coasting slopes cancels Coulomb friction exactly), but dist_hat absorbs the
// error well enough that it is not a prerequisite.
#define OBS_K_ACC   19800.0f

// Viscous damping as a current, straight from FRICTION_FF_VISCOUS_MA_PER_RADS
// (1.9 mA per rad/s). Folding a known term into the model leaves dist_hat with
// only Coulomb friction and genuine load disturbance to absorb, which makes it
// a better estimate of both.
#define OBS_B_VISC  0.0019f    // A per rad/s

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

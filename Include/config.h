#pragma once

// =============================================================================
// Run mode
// =============================================================================

#define RUN_MODE_SYSID        0
#define RUN_MODE_CLOSED_LOOP  1
#define RUN_MODE              RUN_MODE_SYSID

// =============================================================================
// Sysid test selector
// =============================================================================

#define SYSID_TEST_CURRENT_LOOP_CHIRP       0
#define SYSID_TEST_CURRENT_LOOP_STEP        1
#define SYSID_TEST_VEL_CHIRP                2
#define SYSID_TEST_CL_CURRENT_CHIRP         3
#define SYSID_TEST_CL_VEL_STEP              4
#define RIPPLE_DEBUG                        5
#define SYSID_TEST_CL_VEL_CHIRP             6
#define SYSID_TEST_POSITION_STEP            7
#define SYSID_TEST_CL_POS_CHIRP             8
#define SYSID_TEST_CINE_SWEEP               9
#define SYSID_TEST_PHASE_CHECK              10
#define SYSID_TEST_SHUNT_NOISE              11
#define SYSID_TEST_FRICTION_SWEEP           12
#define SYSID_TEST_BREAKAWAY                13

#define SYSID_TEST SYSID_TEST_CL_CURRENT_CHIRP

// SHUNT_NOISE_PWM_ENABLE -- 1: bridge switches normally during the noise
// window (real operating condition, includes any switching-induced ripple).
// 0: pwm_disable() clears MOE so the bridge outputs go static, but TIM1
// keeps counting and still fires the same ADC injected-conversion trigger
// (TIM1 CH4 -> TRGO) at the same cadence -- isolates whether the measured
// noise is switching-induced or an intrinsic shunt/amp/ADC floor, since the
// sample timing is otherwise identical between the two runs.
#define SHUNT_NOISE_PWM_ENABLE 1




// =============================================================================
// SPI telemetry
// =============================================================================
//
// CL_VEL_CHIRP instability events used to end with the whole system frozen.
// Root cause: EXTI15_10_IRQHandler (CS rising edge, NVIC priority 0 --
// highest in the system) was passively waiting for DMA1_Stream4's TX
// enable bit to self-clear, which only happens if the host's SPI
// transaction completed all 32 bytes on its own. A short/glitched host
// transaction leaves the stream permanently mid-transfer with no more
// clock edges ever coming, so it never self-clears -- confirmed live via
// debugger (stall_guard counting all the way down). At priority 0, burning
// the full ~6-8ms bound there blocks the main control loop, ADC, and
// everything else for the duration -- that's what was actually causing the
// "instability" all session; disabling SPI entirely (3 clean passes, no
// freeze) proved it wasn't the velocity loop. Fixed properly in
// EXTI15_10_IRQHandler (spi.c) -- it now force-disables the stream instead
// of waiting on natural completion, so re-enabling telemetry here.
#define SPI_TELEM_ENABLED  1



// =============================================================================
// Math
// =============================================================================

#ifndef M_PI
#define M_PI        3.14159265358979323846f
#endif
#define FOC_TWO_PI  6.28318530718f

// =============================================================================
// Motor — AKM11E
// =============================================================================

#define MOTOR_POLE_PAIRS    3u

// =============================================================================
// Encoder
// =============================================================================

#define ENCODER_CPR      8192u
#define COUNTS_PER_REV   ((float)ENCODER_CPR)
#define COUNTS_PER_RAD   (COUNTS_PER_REV / (2.0f * M_PI))

// =============================================================================
// Bus voltage
// =============================================================================

// 12V is the only bus voltage actually validated tonight. 24V was
// discussed and drafted in this file at one point but the PSU was never
// actually changed to it -- a "76.7% torn SPI frames" result earlier that
// got misattributed to 24V was really at 12V, running
// SYSID_TEST_POSITION_STEP (a big, fast, saturating move). So the real,
// confirmed picture:
//
//  - 12V, big/fast step move, SPI at 4MHz: 76.7% torn frames. Slowing the
//    Pi's SPI clock to 1MHz (spi_open_configure() speed_hz in foc-sysid's
//    main.cpp -- a Pi-side runtime arg, no firmware change) dropped that to
//    12% -- real improvement, not a full fix. Scope on MISO (STM32 output)
//    vs. SCK (from the Pi) showed MISO riding on visible noise, including a
//    slow DC-level shift mid-burst while SCK kept clocking normally --
//    direct evidence of noise coupling onto the line itself (likely PWM/
//    motor switching related), not a firmware or protocol bug. Wiring is
//    already the shortest jumpers available, so length/routing isn't a
//    lever here.
//  - 16V (the one real voltage change tried): the Pi's capture tool gets no
//    usable SPI data at all -- gives up before ALIGN even finishes -- while
//    the STM32 control loop keeps running correctly regardless (motion
//    confirmed physically). Worse than 12V's partial corruption, which
//    argues against a simple "more voltage = more noise" relationship.
//  - 24V: not actually tried. Don't trust any comment/number attributed to
//    24V from earlier tonight.
//
// Net: SPI corruption is real even at the "known good" 12V under heavy
// switching, and gets worse (not better, not proportionally) at 16V. This
// is a bench hardware question (scope the SPI2 signal path and whatever
// feeds its logic levels) -- not something more config changes or clock
// tweaks alone are going to fully resolve. Staying at 12V, since it's the
// only setting where the SPI link is usable at all, even if imperfectly.
//
// Doesn't touch the current loop's design regardless of value: P(s)=1/(Ls+R)
// and CURRENT_LOOP_KP/KI depend only on motor R/L, not V_BUS -- this only
// moves the +-(V_BUS/2) saturation clamp (loops.c pi_init calls) and the PWM
// normalization range (pwm.c), both already written generically off this
// constant.
#define V_BUS   12.0f

// =============================================================================
// Current sensing
// =============================================================================

#define SHUNT_R          0.007f
#define SHUNT_GAIN       40.0f
#define VREF             3.3f
#define ADC_COUNTS       4096.0f
#define ADC_ZERO         2048.0f
#define AMPS_PER_COUNT   (VREF / (ADC_COUNTS * SHUNT_R * SHUNT_GAIN))
#define ADC_SMP_84_CYCLES  4u
#define ADC_SMP_15_CYCLES  1u

// =============================================================================
// Control loop timing
// =============================================================================

#define DT_CURRENT   (1.0f / 20000.0f)
#define DT_VELOCITY  (1.0f /  5000.0f)
#define DT_POSITION  (1.0f /  1000.0f)

// Telemetry-only scale factor for vel_meas_counts before it's narrowed to
// int16_t for the SPI frame. encoder_get_velocity()/vel_meas_counts are
// full-width (int32_t/float) everywhere the control loop actually uses
// them -- this only matters for the telemetry cast, which was overflowing
// unscaled above 32767 counts/s (=25.15 rad/s) in several SYSID_TEST
// blocks, producing a garbled *logged* velocity while the real velocity
// loop feedback stayed correct the whole time. /8 gives headroom to
// ~201 rad/s before this cast wraps again.
#define VEL_TELEM_DIV  8.0f

// =============================================================================
// Current loop — zero-cancellation design, actually ~255Hz, not the 500Hz
// this section used to claim.
//
// Fresh SYSID_TEST_CURRENT_LOOP_CHIRP run (rotor locked, 1.0A amplitude,
// bode_plot.py curve fit, coherence ~1.0 out to ~150-200Hz) gives the
// plant as R=1.45 ohm / L=1.29mH / fc=179Hz -- superseding the previously-
// cited "R=1.67/L=1.41mH" figure, which was a stale citation never
// re-verified in this session. Against THIS fit, Kp=2.07/Ki=2450 (below)
// has a zero/pole ratio of 1.053 -- essentially perfect cancellation --
// and predicts BW=255Hz by construction. That was independently
// corroborated by SYSID_TEST_CL_CURRENT_CHIRP (closed-loop iq_cmd->iq_meas
// chirp, rotor locked): raw -3dB crossing 197-300Hz across two runs,
// single-pole curve fit of the full coherent band 361Hz, backed-out real
// margin gc=252-313Hz/PM=83-114 deg -- all consistent with a real loop
// performing at or a bit above a ~255Hz idealized prediction, not with
// 500Hz.
//
// A genuine 500Hz zero-cancellation design against this plant would need
// Kp=4.05/Ki=4555 -- close to the "1000Hz bump (Kp=4.43/Ki=5184)" already
// tried and reverted earlier tonight for continuous idle buzz (vq
// +-2-3V, iq_meas +-400-800mA at rest, SYSID_TEST_POSITION_STEP). Not
// re-trying that blind. Kp=2.07/Ki=2450 is the proven-quiet value and is
// being kept -- the fix here is the label, not the gain. Re-run
// bode_plot.py + SYSID_TEST_CL_CURRENT_CHIRP again if R/L is ever
// re-measured, rather than trusting this citation indefinitely either.
// Same gains used for both current_loop (q-axis) and d_current_loop --
// Ld ~= Lq for this machine (see loops.c).
// =============================================================================

#define CURRENT_LOOP_KP  2.07f      // V/A -- ~255Hz zero-cancellation vs R=1.45/L=1.29mH, proven quiet
#define CURRENT_LOOP_KI  2450.0f    // V/(A*s)

// =============================================================================
// FOC bring-up voltages
// =============================================================================

#define V_ALIGN  1.0f
#define V_RUN    1.0f
#define ENC_DIR  (+1.0f)
#define FF_GAIN  0.95f


// =============================================================================
// Alignment timing — ALIGN_TICKS at 20 kHz = 100 ms
// =============================================================================

#define ALIGN_TICKS  100000u

// =============================================================================
// Current loop ID Sysid chirp parameters
// =============================================================================

#define SYSID_AMPLITUDE  1.0f
#define SYSID_F_START    1.0f
#define SYSID_F_END      2000.0f
#define SYSID_DURATION   20.0f
#define SYSID_DT         DT_CURRENT

// =============================================================================
// Velocity loop ID Sysid chirp parameters
// =============================================================================


#define VEL_CHIRP_F_START    0.5f
// Bumped 100Hz -> 300Hz: identification is only trustworthy up to roughly
// 1/3 of the actuating current loop's bandwidth (above that, the current
// loop's own dynamics start corrupting the "pure" plant measurement instead
// of injecting a clean iq). That ceiling was ~165Hz when this was set at
// the old 500Hz current-loop target; now that the current loop is
// re-tuned to ~1000Hz (measured gc=1002.6Hz), it's ~330Hz. Also what
// falsified the 65-90Hz "resonance" theory from the closed velocity loop
// chirp: extended to 300Hz, this open-loop plant sweep showed a clean
// single-pole rolloff with no bump at all through that band -- the real
// cause turned out to be encoder_update()'s velocity filter group delay
// (encoder.c, VEL_FILTER_N), not a mechanical mode. See the position loop
// section below for the full story.
#define VEL_CHIRP_F_END     300.0f
#define VEL_CHIRP_DURATION  100.0f
// Bumped 0.1f -> 0.3f: stage now attached (was bare motor when 0.1f/0.3f/0.5f/
// 0.7f were last characterized) and 0.1A (0.064 Nm @ KT=0.64 Nm/A) produced
// real motion up to ~10Hz then a hard freeze above that -- not a gradual
// 1/f^2 inertial rolloff into the noise floor (which would shrink smoothly),
// a sharp cutoff. Now explained directly: SYSID_TEST_FRICTION_SWEEP/
// BREAKAWAY measured this stage's breakaway current at ~100mA -- 0.1A sits
// right at/below that, so above some chirp frequency each half-cycle is too
// short for current to ramp up to breakaway before reversing, and the stage
// just sits there. 0.3A is 3x breakaway, comfortable headroom, still well
// under VEL_CHIRP_IQ_LIMIT=1.0A's clamp.
#define VEL_CHIRP_AMPLITUDE  0.3f


// iq_cmd here is open-loop and unclamped by construction -- nothing bounds
// it like VEL_IQ_LIMIT bounds the closed velocity loop's PI output. At
// AMPLITUDE=0.3A this ran away once real motion (real back-EMF) appeared,
// current railed at the ADC's representable limit for ~3.5s. Safety
// ceiling, not a normal operating limit -- should never actually bind at
// the intended amplitude.
//
// Tightened from 1.0A -- the clamp only bounds the *commanded* iq_cmd, not
// how hard the current loop's PI can react to a real disturbance. A real
// stick-slip breakaway snap (confirmed via ADC counter staying healthy
// through the event -- not a stale-sensor artifact) still let measured
// current ring out to ~10.5A despite iq_cmd never exceeding 1.0A. Tighter
// clamp limits how much corrective authority the loop has to ring with.
#define VEL_CHIRP_IQ_LIMIT  1.0f    // A -- 0.3A was reactive (well below the ~10.5A ring-out), not
                                    // hardware-derived. Motor rated 2.9A continuous; still well under
                                    // that with real margin for AMPLITUDE=0.2A to sit unclamped.

// Settle-onto-zero before the chirp starts -- covers current decaying to
// zero (tau ~= 0.79ms electrical). 20 ticks @ this test's raw 20kHz rate =
// 1ms, comfortably past tau. Kept as
// its own constant (not reused from CL_VEL_CHIRP_SETTLE_TICKS below) since
// that one runs at the 5kHz-decimated velocity-loop rate -- same tick count
// would mean a different real time on each.
#define VEL_CHIRP_SETTLE_TICKS  20u

// =============================================================================
// Closed velocity loop chirp — sweeps vel_cmd (not iq) with the vel PI
// closed, to identify the CLOSED-loop vel_cmd -> vel_meas response for
// designing the outer position P controller.

// =============================================================================

#define CL_VEL_CHIRP_F_START     0.5f
#define CL_VEL_CHIRP_F_END     200.0f
#define CL_VEL_CHIRP_DURATION   60.0f
#define CL_VEL_CHIRP_AMPLITUDE  15.0f     // rad/s -- re-testing the 8 vs 15 comparison with
                                          // friction feedforward now active (FRICTION_FF_*
                                          // below); this run is the amplitude=15 half

// P-only velocity loop for this identification run only (Ki=0) -- makes the
// controller a known memoryless gain C(s)=Kp, so the plant can be backed out
// of the measured closed-loop response with a single division:
//   H = CP/(1+CP)  =>  L = H/(1-H)  =>  P = L / Kp
// Same value used for the bare-motor identification (loops.c:9-31,
// Kp=0.0239, coherence ~1.0 to ~150Hz there). Re-used as the starting point
// for the stage-loaded re-identification -- peak iq_cmd = Kp*AMPLITUDE =
// 0.0239*8 = 0.19A, well under VEL_IQ_LIMIT=0.5A (loops.c), so headroom
// exists to raise CL_VEL_CHIRP_AMPLITUDE later if the stage's response is
// too small to fit well.
#define CL_VEL_CHIRP_PONLY_KP    0.0239f

// Friction feedforward -- Fc/Fv from the repeated SYSID_TEST_FRICTION_SWEEP
// runs (stage attached, averaged across 3 clean runs: Fc~94mA, Fv~1.9mA per
// rad/s; breakaway came out ~same order as kinetic within measurement
// noise, so no separate static term). Added to CL_VEL_CHIRP's P-only iq_cmd
// here specifically to re-test whether the amplitude-dependence seen
// between CL_VEL_CHIRP_AMPLITUDE=8 vs 15 rad/s (K/tau backed out ~2x
// different) was actually the P-only loop fighting friction rather than a
// real plant nonlinearity -- if it was, feeding friction forward instead of
// making the loop react to it should make the two amplitudes agree.
// FF_SMOOTH_RAD_S replaces a hard sign(v) with tanh(v/FF_SMOOTH_RAD_S) so
// the feedforward doesn't flip-flop full Coulomb torque back and forth
// chattering right at v=0 -- matters for any real position move (always
// ends by crossing v=0 to settle), not for this chirp test itself.
#define FRICTION_FF_COULOMB_MA           94.0f   // mA
#define FRICTION_FF_VISCOUS_MA_PER_RADS   1.9f   // mA per rad/s
#define FRICTION_FF_SMOOTH_RAD_S          2.0f   // rad/s

// =============================================================================
// Friction sweep — closed velocity loop (production PI, real integrator),
// series of constant-velocity holds from V_MIN to V_MAX rad/s, one full
// ascending sweep per direction. A real integrator drives steady-state
// velocity error to zero, so steady-state i_q at each hold is a direct
// measurement of the torque needed to overcome friction (+ viscous drag) at
// that speed -- the actual friction curve, not inferred indirectly from how
// closed-loop Bode gain shifts with chirp amplitude (which is what
// motivated this test: CL_VEL_CHIRP amplitude 8 vs 15 rad/s gave a nearly
// 2x different backed-out plant gain/tau on this stage-loaded system, and
// the archived bare-motor CL_VEL_CHIRP amplitude sweep -- 5 rad/s: DC=-6.2dB
// BW=1.7Hz vs 20/25 rad/s: DC=-1.3dB BW~61-62Hz -- showed the same cliff,
// pre-dating the stage entirely).
//
// V_MAX capped at 23, not 25 -- the first run of this test showed vel_meas
// (encoder_get_velocity()) intermittently flipping sign at 24-25 rad/s
// (real position trace stayed smooth/monotonic through those points --
// confirmed via enc_hi_raw/enc_lo_raw, so the stage itself was fine, this
// is a measurement glitch, likely an overflow in the velocity calc at high
// speed) while feeding that same corrupted value back into the velocity
// PI's own feedback, producing erratic iq at exactly those two steps. Not
// yet root-caused -- until it is, 24-25 rad/s data from this test isn't
// trustworthy, so stop the sweep at 23 instead of spending time/travel on
// two points that have to be thrown out anyway.
//
// One direction at a time, with a zero-velocity settle between them,
// rather than alternating sign every step -- keeps any breakaway/hysteresis
// asymmetry between directions as a difference between the two sweep
// halves instead of scrambling it step to step.
// =============================================================================
#define FRICTION_SWEEP_V_MIN         1.0f    // rad/s
#define FRICTION_SWEEP_V_MAX        23.0f    // rad/s
#define FRICTION_SWEEP_V_STEP        1.0f    // rad/s

// Stage has 150mm total travel, 2mm lead/rev, ENCODER_CPR=8192 counts/rev.
// A constant-velocity hold displaces the stage by v*lead/(2*pi) per second
// regardless of how "settled" the loop already is, so hold time directly
// sets one-way sweep distance -- originally 5000 ticks (1.0s/step) summed
// to ~103mm one-way, nearly the entire travel. Cut to 1000 ticks (0.2s) for
// the first run, which measured ~20.7mm one-way (enc_hi_raw/enc_lo_raw,
// confirmed real motion, no hard-stop). Told directly that real available
// travel is 2x what that run used (~41mm), not just an estimate -- sized
// against that number now instead of the original 150mm/2 guess: 1500
// ticks (0.3s/step) sums to ~26.3mm one-way (sum(1..23)*0.318mm/s*0.3s),
// safely under that 41mm with real margin, and buys a longer steady-state
// averaging window per step than the first run had (that run's fit
// residual std was already only ~5.3mA, consistent with pure measurement
// noise -- this should tighten it further, not chase a real nonlinearity).
#define FRICTION_SWEEP_HOLD_TICKS   1500u    // @ 5kHz decimated = 0.3s/step
#define FRICTION_SWEEP_SETTLE_TICKS 1000u    // @ 5kHz decimated = 0.2s

#define STAGE_LEAD_MM_PER_REV         2.0f

// Hard backstop, independent of the hold-time arithmetic above -- aborts
// the current sweep direction early if actual measured displacement from
// the test's start position exceeds this, rather than trusting the
// commanded-velocity x hold-time estimate alone not to be wrong (stale
// constant, wrong direction sign, anything). Tightened from 50mm to 35mm
// now that the real safe budget is known to be ~41mm (2x the ~20.7mm the
// first run actually used) -- 50mm exceeded that; 35mm sits under it with
// real margin instead of just under the old 150mm/2 guess.
#define FRICTION_SWEEP_POS_LIMIT_MM  35.0f

// =============================================================================
// Breakaway / static friction -- slow open current-loop ramp from rest,
// watching for real motion (encoder velocity crossing a debounced
// threshold), in both directions. Different risk profile than the kinetic
// sweep above: this deliberately drives toward the exact failure mode that
// already produced a real ~10.5A current ring-out on this stage (see
// VEL_CHIRP_IQ_LIMIT section above) -- a stick-slip breakaway snap
// releasing stored energy faster than an open loop can react. Mitigated by
// (1) ramping slowly so the breakaway current is resolved finely instead of
// jumped past, (2) requiring the detection threshold to be sustained for
// multiple ticks, not a single noisy sample -- idle jitter measured up to
// ~0.77 rad/s in the friction sweep's zero-holds, and (3) handing off to
// the CLOSED velocity loop the instant breakaway is detected instead of
// continuing the open-loop ramp -- the closed loop settles toward the
// already-validated ~94mA kinetic friction current instead of continuing
// to push whatever torque the ramp had built up.
// =============================================================================
#define BREAKAWAY_IQ_CEILING           0.5f   // A -- matches VEL_IQ_LIMIT (loops.c),
                                               // already validated safe elsewhere
#define BREAKAWAY_RAMP_DURATION        5.0f   // s, 0 -> ceiling (0.1A/s)
#define BREAKAWAY_DETECT_THRESH_RAD_S  1.0f   // rad/s -- above the ~0.77 rad/s
                                               // idle jitter ceiling observed
#define BREAKAWAY_DETECT_TICKS          25u   // @ 5kHz decimated = 5ms sustained
#define BREAKAWAY_HANDOFF_VEL          1.0f   // rad/s, closed-loop target once free
#define BREAKAWAY_HANDOFF_TICKS       1500u   // @ 5kHz decimated = 0.3s

// Expected excursion here is <1mm (no motion at all until breakaway, then a
// brief 1 rad/s hold) -- tight backstop sized for this test's much smaller
// envelope, not reused from the friction sweep's 35mm.
#define BREAKAWAY_POS_LIMIT_MM        10.0f   // mm

// =============================================================================
// Closed current loop chirp — sweeps iq_cmd (not Vq) with the current PI
// closed, to directly measure H(s) = iq_meas/iq_cmd and back out the real
// closed-loop margin (L = H/(1-H)) instead of assuming C(s)=Kp+Ki/s and
// combining it with the separately-measured open-loop electrical plant
// (bode_plot.py's method -- shown on the velocity loop to be off by 37% on
// crossover, most likely from saturation invalidating that assumption).
// =============================================================================

#define CL_CURRENT_CHIRP_F_START        1.0f
#define CL_CURRENT_CHIRP_F_END        900.0f   /* capped below the 1kHz Nyquist of the 2kHz telemetry frame rate */
#define CL_CURRENT_CHIRP_DURATION      40.0f
#define CL_CURRENT_CHIRP_AMPLITUDE     0.2f    // A
#define CL_CURRENT_CHIRP_SETTLE_TICKS  20u

// =============================================================================
// Position loop — pure P, back to the phase-margin-targeted design off the
// closed-velocity-loop chirp (crossover picked from measured phase, Kp
// sized from the ACTUAL measured gain there, not assumed unity).
//
// The previous note here blamed a 65-90Hz peak on a two-inertia mechanical
// resonance (motor/load coupling compliance) and backed this gain off to
// gc~15Hz to stay clear of it. That theory is dropped: extending the
// open-loop plant sweep to 300Hz (VEL_CHIRP_F_END) showed a clean,
// monotonic single-pole rolloff with good coherence right through that
// band -- no bump at all. A real mechanical mode is a property of P(s)
// itself and would have to show up in a direct P(s) measurement; it
// didn't. The actual cause was encoder_update()'s velocity filter group
// delay (encoder.c, VEL_FILTER_N 80->20) -- fixing that took the closed
// velocity loop's measured phase crossover from 82.7Hz to 165.4Hz and gain
// margin from 3.7dB to 8.8dB, with no physical change needed.
//
// Was 954.5 (from closed_vel_bode_plot.py's phase-margin-targeted
// auto-design: predicted gc=71.92Hz/PM=60deg/GM=8.8dB; measured whole-
// system result was actually better than predicted -- gc=32.17Hz,
// PM=60.7deg, pc=88.48Hz, GM=16.6dB, 32.6Hz BW, SYSID_TEST_CL_POS_CHIRP).
// Margins were excellent, but that chirp-based verification only checks
// small-signal stability, not idle noise -- SYSID_TEST_POSITION_STEP then
// showed continuous chatter at rest (vq buzzing ~+-2-3V, iq_meas
// ~+-400-800mA, vel_meas dense noise throughout, not just at step edges).
// Reverting CURRENT_LOOP_KP/KI to the 500Hz-target gains didn't fix it,
// which points at this gain: at 954.5, ordinary vel_meas noise (from
// VEL_FILTER_N=20's shorter/noisier window) gets amplified into real
// velocity-command jitter that the inner loops then chase continuously.
// Was reverted to 200 (the last value with a clean, quiet step response)
// on the STAGE-ATTACHED config -- that history stands, but doesn't
// directly apply here since VEL_KP/KI were just re-designed against a
// fresh bare-motor plant (see loops.c) and closed-loop verified
// (gc=44.6Hz, PM=69.2deg, GM=15.1dB, BW=80.4Hz).
//
// Re-derived as a frequency-ratio design instead: pick outer crossover at
// 1/3-1/5 of the velocity loop's measured -3dB BW (80.4Hz), read H(jf)
// AT that frequency from the same verification run's raw data (not
// assumed unity), size Kp_pos so |Kp_pos*H(jf)/jf|=1 there:
//   1/3  (26.80Hz): H=0.26dB   Kp_pos=163.37  PM=57.7deg
//   1/4  (20.10Hz): H=0.13dB   Kp_pos=124.45  PM=68.0deg  <- chosen, middle of range
//   1/5  (16.08Hz): H=0.24dB   Kp_pos=98.31   PM=73.1deg
//
// Closed-loop verified directly (SYSID_TEST_CL_POS_CHIRP, bare motor,
// whole-system H_pos(s)=pos_meas/pos_cmd, margin backed out via
// L=H_pos/(1-H_pos)): gain crossover landed at 21.11Hz, within 5% of the
// 20.10Hz this Kp was sized for -- reading real H(jf) instead of assuming
// unity gain got the crossover right. PM=73.0deg (beat the 68.0deg
// prediction), GM=16.6dB. Measured -3dB bandwidth is 42.64Hz, roughly 2x
// the crossover -- NOT the same thing as gc, don't confuse the two: BW
// exceeding gc is the standard consequence of PM<90deg (mild peaking near
// crossover extends the -3dB point outward), same mechanism as the
// velocity loop's 80.4Hz BW vs its 44.6Hz gc. The number actually designed
// for (crossover) landed almost exactly on target.
// =============================================================================

// Superseded by the stage-attached + friction-feedforward design below --
// 124.45 was sized against the bare-motor velocity loop (no stage, no
// feedforward) and is kept here only as prior-art history, not the active
// value.
//
// Re-derived after: (1) attaching the stage, (2) characterizing its
// friction (SYSID_TEST_FRICTION_SWEEP, 3 repeat runs: Fc~94mA, Fv~1.9mA per
// rad/s), and (3) folding that in as feedforward (velocity_loop_step(),
// loops.c) instead of leaving the velocity PI to fight it -- confirmed by
// re-running the CL_VEL_CHIRP amplitude=8-vs-15 comparison, which had
// previously shown a ~2x different backed-out plant gain/tau depending on
// amplitude (friction dominating a P-only loop's response, not a real
// plant nonlinearity): with feedforward active, DC gain converged to
// -0.01/-0.09dB at both amplitudes (was -8.67/-4.02dB), confirming the
// feedforward-compensated system behaves consistently enough to design
// against.
//
// Sized phase-margin-targeted (not decade-below-BW) off that feedforward-
// compensated closed velocity loop (H(s)=vel_meas/vel_cmd, amplitude=15
// rad/s run): DC gain -0.01dB, -3dB BW=107.0Hz. Crossover picked where H's
// phase lag gives ~60deg position-loop PM, Kp_pos sized off the ACTUAL
// measured gain there (0.17dB), not assumed unity:
//   gain crossover 33.35Hz, PM=60.0deg, GM=10.7dB (open-loop check,
//   L(s)=Kp_pos*H(s)/s)
// Not yet closed-loop verified with a real SYSID_TEST_CL_POS_CHIRP run on
// the stage-attached + feedforward system (bare-motor precedent above found
// the predicted crossover landed within 5% once closed-loop verified --
// worth repeating that check here before trusting this number blindly).
#define POSITION_LOOP_KP        100.0f   // (rad/s) per rad of position error -- backed off from 205.5
                                          // (measured 39deg PM, short of 60deg target) to restore margin.
                                          // Closed-loop verified: gc=18.91Hz, PM=70.6deg, GM=22.0dB,
                                          // whole-system BW=31.6Hz (SYSID_TEST_CL_POS_CHIRP)
#define POSITION_VEL_LIMIT      50.0f    // rad/s -- clamp P output, no windup state to clamp

// =============================================================================
// Position step test parameters
// =============================================================================

// NOTE: cl_step_tick in run_position_step() increments inside the
// position-loop-decimated block (1 kHz, POS_LOOP_DECIMATE=20), NOT the raw
// 20kHz ISR tick -- so these counts are in 1kHz ticks, not 20kHz ticks.
#define POSITION_STEP_AMPLITUDE_RAD  1.0f      // ~11.5 deg -- gentle first test
#define POSITION_STEP_SETTLE_TICKS   1000u     // 1s @ 1kHz
#define POSITION_STEP_HOLD_TICKS     2000u     // 2s @ 1kHz

// =============================================================================
// Closed position loop chirp — sweeps pos_cmd with the FULL cascade closed
// (position P -> velocity PI -> current PI), to directly MEASURE the actual
// closed-loop response of the whole system (pos_meas/pos_cmd), rather than
// relying on the H(s)/s construction used to design POSITION_LOOP_KP.
//
// F_END was originally 30 Hz, sized for the OLD conservative Kp_pos=51.81
// (~7-8 Hz expected BW). With the faster phase-margin-targeted Kp_pos=157.7,
// a 30 Hz sweep top wasn't high enough to see the closed-loop -3dB point OR
// the open-loop phase reach -180 deg (mag was still only -1.3dB and phase
// only -108 deg at 30 Hz) -- widened to 150 Hz for headroom above both.
// =============================================================================

#define CL_POS_CHIRP_F_START     0.1f     // Hz
#define CL_POS_CHIRP_F_END     150.0f     // Hz
#define CL_POS_CHIRP_DURATION   60.0f     // s
#define CL_POS_CHIRP_AMPLITUDE   0.3f   // rad

// =============================================================================
// Cine sweep — same closed-loop position chirp, but sized for FILMING. Now
// deliberately swept PAST both gains' real tracking limit -- the point isn't
// just to look pretty, it's to visibly show the system run out of bandwidth
// (or hit its velocity clamp) as frequency climbs: measured amplitude
// shrinking below commanded, growing phase lag, eventually degenerating into
// a blurred/reduced-amplitude wobble instead of clean tracking. The overlay
// graph (cine_overlay_render.py) makes the breakdown legible even once the
// physical motion itself is too fast to resolve by eye.
//
// AMPLITUDE was cut from 0.4 to 0.25 rad so the failure you see is genuinely
// BANDWIDTH-driven for as much of the sweep as possible, not an artifact of
// hitting POSITION_VEL_LIMIT early: a perfectly-tracking loop needs peak
// velocity ~2*pi*f*AMPLITUDE, which only reaches the 20 rad/s clamp around
// 12.7 Hz at this amplitude (vs. ~8 Hz at the old 0.4 rad) -- past that point
// the clamp becomes part of the limitation too, which is a real, legitimate
// system limit, not a bug in the test.
//
// Run it once per gain you want to compare on camera (rebuild/reflash between
// runs): POSITION_LOOP_KP=51.81 for the "before" take, 157.7 for "after".
// Expect the old gain to visibly give up well before the new one.
//
// F_START raised from 0.1 to 3Hz -- the slow end was dead time, nothing
// visibly interesting happens until well into the sweep. F_END raised from
// 40 to 80Hz -- 40 wasn't high enough to kill the visible motion entirely,
// it was still clearly vibrating at the end. DURATION cut from 40 to 20s to
// keep the sweep rate (time per decade) roughly the same over the smaller,
// higher-frequency range instead of dragging the interesting part out.
// =============================================================================

#define CINE_SWEEP_F_START      3.0f    // Hz
#define CINE_SWEEP_F_END       80.0f    // Hz
#define CINE_SWEEP_DURATION    20.0f    // s
#define CINE_SWEEP_AMPLITUDE    0.25f   // rad

// =============================================================================
// Phase check — per-phase DC excitation, isolates a bad phase connection
// (connector/ferrule, not energizing) from a bad current-sense channel
// (energized fine, that ADC channel just doesn't show it). Reuses
// run_voltage_test's (0.5, -0.25, -0.25) magnitude, already proven safe held
// for 30s on this hardware.
// =============================================================================

#define PHASE_CHECK_V_MAIN      1.5f     // V -- driven phase
#define PHASE_CHECK_V_RETURN   -0.75f    // V -- other two phases, balanced return
#define PHASE_CHECK_HOLD_TICKS  40000u   // 2s @ 20kHz -- long enough to read a settled DC value
#define PHASE_CHECK_ZERO_TICKS   2000u   // 100ms @ 20kHz -- decay margin between phases (same as ALIGN's discharge)
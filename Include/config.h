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
#define SYSID_TEST_CL_VEL_STEP              4
#define RIPPLE_DEBUG                        5
#define SYSID_TEST_CL_VEL_CHIRP             6
#define SYSID_TEST_POSITION_STEP            7
#define SYSID_TEST_CL_POS_CHIRP             8
#define SYSID_TEST_CINE_SWEEP               9

#define SYSID_TEST SYSID_TEST_POSITION_STEP




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

// =============================================================================
// Current loop — zero-cancellation design, back to a 500Hz BW target
//
// Reverted from the 1000Hz bump (Kp=4.43/Ki=5184) made earlier tonight.
// That comment predicted the exact tradeoff in advance: "doubled Kp/Ki
// means more control effort per unit current error, watch for the ~40mA
// RMS ADC noise floor showing up as buzz." SYSID_TEST_POSITION_STEP at
// V_BUS=12V/POSITION_VEL_LIMIT=50 then showed continuous vq buzz (~+-2-3V,
// spiking to the rail) and iq_meas buzz (~+-400-800mA) even at rest, not
// just at step edges -- real evidence of that predicted cost, on top of
// (and likely compounding with) the outer loop's own noise from
// VEL_FILTER_N=20 and POSITION_LOOP_KP=954.5. Reverting the current loop
// gain is the direct lever for the current-loop's own contribution to
// that buzz, independent of the outer-loop retuning question.
//
// 500Hz-target gains, designed the same zero-cancellation way against an
// earlier plant fit (R=1.56-1.67/L=1.32-1.38mH, halved to per-phase):
// measured landing at gc=469-499Hz (run to run) with PM=85-88 deg,
// heavily overdamped -- real margin, but with less control effort per
// unit current error than the 1000Hz design. Re-run bode_plot.py after
// any change here to confirm gc against the real (now 1.67/1.41mH)
// plant fit, not just by construction. Same gains used for both
// current_loop (q-axis) and d_current_loop -- Ld ~= Lq for this machine
// (see loops.c).
// =============================================================================

#define CURRENT_LOOP_KP  2.07f      // V/A
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
// zero motion -- plausibly below the stage's static/breakaway friction, not
// a loop problem. 0.3A (0.192 Nm) was already a known, characterized point
// on the bare motor (reactive but understood); going incremental from the
// clamp ceiling (VEL_CHIRP_IQ_LIMIT=1.0A, 0.64 Nm) down, not guessing a
// big number blind on a stage whose breakaway torque loaded is unknown.
#define VEL_CHIRP_AMPLITUDE  0.9f


// iq_cmd here is open-loop and unclamped by construction -- nothing bounds
// it like VEL_IQ_LIMIT bounds the closed velocity loop's PI output. At
// AMPLITUDE=0.3A this ran away once real motion (real back-EMF) appeared,
// current railed at the ADC's representable limit for ~3.5s, sagged the
// bench PS hard enough (shared return/ground) to brown-out reset the MCU
// -- confirmed via RCC->CSR (BORRSTF/PORRSTF). Safety ceiling, not a normal
// operating limit -- should never actually bind at the intended amplitude.
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

// Was a DC-biased chirp (kept the stage moving one direction to avoid
// stick-slip breakaway every half cycle) with a soft travel-limit reversal
// -- both removed. The travel-limit reversal introduced real discontinuities
// into the chirp response that corrupted the linear-system-ID fit; bias
// went with it since the reversal logic was its only consumer.

// Settle-onto-zero before the chirp starts -- with the bias removed, this
// only needs to cover current decaying to zero (tau ~= 0.79ms electrical),
// not letting the stage reach a steady-state kinetic speed anymore. 20
// ticks @ this test's raw 20kHz rate = 1ms, comfortably past tau. Kept as
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
#define CL_VEL_CHIRP_AMPLITUDE  10.0f    // rad/s

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
// Back to 200 (the last value with a clean, quiet step response) to
// check that theory -- gc~15-20Hz, well under the phase-margin-targeted
// design but with no demonstrated chatter cost. Re-verify with another
// SYSID_TEST_POSITION_STEP capture before trusting either number again;
// a stability margin isn't the same thing as a quiet loop.
// =============================================================================

#define POSITION_LOOP_KP        200.0f   // (rad/s) per rad of position error -- reverted pending buzz check, see comment above
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
#define CL_POS_CHIRP_AMPLITUDE   0.15f   // rad

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
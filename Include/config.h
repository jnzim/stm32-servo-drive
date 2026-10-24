#pragma once

// =============================================================================
// Run mode
// =============================================================================

// Compile-time switch: RUN_MODE_SYSID runs one identification test
// (foc_sysid.c, SysIdSample telemetry); RUN_MODE_CLOSED_LOOP runs commanded
// moves from the Pi (foc_trajectory.c, TelemetryFrame telemetry). Changing it
// means a rebuild and reflash.
#define RUN_MODE_SYSID        0
#define RUN_MODE_CLOSED_LOOP  1
#define RUN_MODE              RUN_MODE_CLOSED_LOOP

// Move profile for RUN_MODE_CLOSED_LOOP. Both generators take the same
// MoveCmd (target, vel_max, accel_max), so the Pi side does not change.
//   TRAJ_PROFILE_TRAP     trapezoidal velocity (trap_gen.c): accel steps
//                         between +-accel_max, infinite jerk at each corner
//   TRAJ_PROFILE_MINJERK  rest-to-rest quintic (minjerk_gen.c): accel is
//                         continuous; duration chosen so neither vel_max nor
//                         accel_max is exceeded
#define TRAJ_PROFILE_TRAP     0
#define TRAJ_PROFILE_MINJERK  1
#define TRAJ_PROFILE          TRAJ_PROFILE_TRAP

// =============================================================================
// Sysid test selector
// =============================================================================

// IDs are sent in every telemetry frame and the Pi dispatches on them, so the
// numbers are fixed; gaps are tests that were removed.
#define SYSID_TEST_CURRENT_LOOP_CHIRP       0
#define SYSID_TEST_CL_CURRENT_CHIRP         3
#define SYSID_TEST_CL_VEL_CHIRP             6
#define SYSID_TEST_CL_POS_CHIRP             8
#define SYSID_TEST_FRICTION_SWEEP           12
#define SYSID_TEST_CL_VEL_PLANT_CHIRP       14

#define SYSID_TEST SYSID_TEST_CL_POS_CHIRP

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

// Sets the current PI clamp (+-V_BUS/2 = +-12V, loops.c) and the PWM
// normalization (pwm.c). The supply must actually be at this voltage, or the
// firmware commands more than the bridge delivers. 24V rather than 12V because
// at 12V the current loop's vq hit its 6V clamp near 1.25kHz during a 0.8A
// chirp; the loop gains do not depend on V_BUS.
#define V_BUS   24.0f

// =============================================================================
// Current sensing
// =============================================================================

#define SHUNT_R          0.007f
#define SHUNT_GAIN       40.0f
#define VREF             3.3f
#define ADC_COUNTS       4096.0f
#define ADC_ZERO         2048.0f
#define AMPS_PER_COUNT   (VREF / (ADC_COUNTS * SHUNT_R * SHUNT_GAIN))
#define ADC_SMP_15_CYCLES  1u

// =============================================================================
// Control loop timing
// =============================================================================

#define DT_CURRENT   (1.0f / 20000.0f)

// Velocity loop runs every VEL_LOOP_DECIMATE-th tick (5 kHz); DT_VELOCITY is
// derived from it so the two cannot disagree. Running it at 20 kHz was tried
// (2026-10-06) and changed nothing measurable (gc 114.7 vs 107.7Hz, PM 68.2
// vs 70.9deg) at 4x the CPU.
#define VEL_LOOP_DECIMATE  4u
#define DT_VELOCITY  (DT_CURRENT * (float)VEL_LOOP_DECIMATE)

#define DT_POSITION  (1.0f /  1000.0f)

// Telemetry-only scaling so velocities fit the int16 frame slots. The control
// loops use full-width values.
#define VEL_TELEM_DIV      8.0f   // vel_meas counts/s / 8: range +-201 rad/s
#define VEL_CMD_TELEM_DIV  4.0f   // vel_cmd mrad/s / 4: range +-131 rad/s (position chirp)

// =============================================================================
// Current loop
// =============================================================================

// Zero-cancellation PI, same gains on d and q (Ld ~= Lq). Plant from the
// closed-loop current chirp (0.8A): R = 1.73 ohm, L = 1.05 mH, pole 262Hz.
//     Kp = 2*pi*800*L = 5.28 V/A
//     Ki = 2*pi*800*R = 8696 V/(A*s)      Ki/Kp = R/L, so L(s) -> wc/s
// Measured (2026-10-23, locked rotor, chirp to 2kHz): crossover ~830Hz,
// PM ~63deg, closed-loop -3dB ~1.8kHz. Was a 500Hz design (Kp 3.30,
// Ki 5435): crossover 504Hz, PM 72.7deg, -3dB 767Hz.
//
// Fit the plant from the closed-loop chirp, not the open-loop one: the
// open-loop chirp's current falls off at high frequency, where dead-time
// error reads L high (1.20 vs 1.05 mH).
#define CURRENT_LOOP_KP  5.28f      // V/A
#define CURRENT_LOOP_KI  8696.0f    // V/(A*s)

// =============================================================================
// Velocity loop
// =============================================================================

// PI on the observer velocity, loaded stage. Plant from the P-only chirp
// (SYSID_TEST_CL_VEL_PLANT_CHIRP, 15 rad/s, 2026-10-22): K = 82.07 rad/s/A,
// tau = 6.85 ms, pole at 23.2Hz; three earlier runs at 15 rad/s gave pole
// 23-26Hz. Ki = Kp * 2*pi*23.2 puts the PI zero on that pole. Ki was 6.34
// (zero at 11.2Hz, from the 2026-09-19 fit), which left a 0.3-0.6dB sag in
// the closed loop from 10 to 80Hz; at 13.13 it is flat within 0.2dB.
// Kp sets the closed-loop bandwidth to ~250Hz, about a third of the 750Hz
// current loop it was designed against. With the current loop at 800Hz the
// same Kp 0.090 measured 214Hz (less lag, less lift near crossover), so Kp
// went to 0.107 to hold it.
// Measured (2026-10-23, current loop 800Hz, OBS_K_ACC 11976): through the
// observer crossover 272.9Hz, PM 74.9deg; through the encoder -3dB 262.6Hz.
// Position loop on top: 63.5Hz, PM 64.5deg, GM 11.7dB. 10mm moves: 1.22um
// peak, 0.27um rms.
#define VEL_KP        0.10700f   // A / (rad/s)
#define VEL_KI        15.600f    // A / rad      Kp * 2*pi*23.2

// Velocity PI output clamp. The motor's 2.91A rating is continuous (RMS,
// thermal), not a peak limit: on a 0.15 rad position chirp iq ran 0.73A RMS
// with 3.0A peaks. 3.5A stays inside the current-sense range (saturates near
// 5.9A). Friction feedforward is added after this clamp. The bench supply
// must allow 3.5A peaks at 24V.
#define VEL_IQ_LIMIT  3.5f       // A

// =============================================================================
// Alignment (d-axis hold that pulls the rotor to electrical angle 0)
// =============================================================================

#define V_ALIGN      1.0f        // V on the d axis
#define ENC_DIR      (+1.0f)
#define ALIGN_TICKS  100000u     // trajectory mode: 5 s at 20 kHz

// =============================================================================
// Open-loop current chirp (test 0)
// =============================================================================

#define SYSID_AMPLITUDE  2.0f      // V on q, theta = 0
#define SYSID_F_START    1.0f
#define SYSID_F_END      1000.0f   // 5 samples/cycle at 5 kHz telemetry
#define SYSID_DURATION   20.0f
#define SYSID_DT         DT_CURRENT

// =============================================================================
// Closed velocity loop chirp (tests 6 and 14)
// =============================================================================

#define CL_VEL_CHIRP_F_START     0.5f
#define CL_VEL_CHIRP_F_END     400.0f
#define CL_VEL_CHIRP_DURATION   60.0f
#define CL_VEL_CHIRP_AMPLITUDE  15.0f     // rad/s

// Test 14 (plant identification) closes the loop with a P-only gain, so
// C(s) = Kp exactly and the plant backs out of the measured response:
//   H = CP/(1+CP)  =>  L = H/(1-H)  =>  P = L / Kp
// Peak iq_cmd = Kp*AMPLITUDE = 0.0239*15 = 0.36A, well under VEL_IQ_LIMIT.
// Test 6 (verification) runs the deployed VEL_KP/VEL_KI; read H only.
#define CL_VEL_CHIRP_PONLY_KP    0.0239f

// Test 14 only: 1 adds the friction feedforward on top of the P-only loop,
// for the FF on/off DC-gain comparison. C(s) is then no longer Kp, so leave
// it 0 for plant identification.
#define CL_VEL_PLANT_FF          0

// Test 6 only: 1 adds friction feedforward on top of the deployed PI. 0 is
// the loop the position chirp's identification mode sits on.
#define CL_VEL_CHIRP_FF          0

// =============================================================================
// Friction feedforward (velocity_loop_step, loops.c)
// =============================================================================

// iq_ff = Fc * tanh(vel_cmd / smooth) + Fv * vel_cmd. From the friction sweep
// (test 12), stage attached, averaged over 3 runs. Re-measured 2026-10-06:
// Fc 90.6mA, Fv 2.13 mA per rad/s, inside the sweep's +-5mA repeatability, so
// the constants were left alone. tanh instead of sign() so the Coulomb term
// does not chatter at zero velocity.
// 0 turns the friction feedforward off, for before/after move captures.
#define FRICTION_FF_ENABLE                1
#define FRICTION_FF_COULOMB_MA           94.0f   // mA
#define FRICTION_FF_VISCOUS_MA_PER_RADS   1.9f   // mA per rad/s
#define FRICTION_FF_SMOOTH_RAD_S          2.0f   // rad/s

// =============================================================================
// Friction sweep (test 12)
// =============================================================================

// Deployed velocity PI (real integrator, so steady-state iq is the friction
// torque) holding constant velocities V_MIN..V_MAX, one direction at a time
// with a zero-velocity settle between. Repeatability is +-5mA (forward vs
// reverse halves at matched speeds); fit Fv over the full range, not a narrow
// one.
#define FRICTION_SWEEP_V_MIN         1.0f    // rad/s
#define FRICTION_SWEEP_V_MAX        23.0f    // rad/s
#define FRICTION_SWEEP_V_STEP        1.0f    // rad/s

// 0.3 s per step is about 26mm of travel one way, inside the ~41mm available.
#define FRICTION_SWEEP_HOLD_TICKS   1500u    // @ 5kHz = 0.3s/step
#define FRICTION_SWEEP_SETTLE_TICKS 1000u    // @ 5kHz = 0.2s

#define STAGE_LEAD_MM_PER_REV         2.0f

// Backstop: ends a sweep direction if measured travel from its start exceeds
// this, whatever the hold-time arithmetic says.
#define FRICTION_SWEEP_POS_LIMIT_MM  35.0f

// =============================================================================
// Closed current loop chirp (test 3)
// =============================================================================

// Chirps iq_cmd through the closed current PI; L = H/(1-H) gives the margin.
#define CL_CURRENT_CHIRP_F_START        1.0f
#define CL_CURRENT_CHIRP_F_END       1000.0f   // 5 samples/cycle at 5 kHz telemetry; raise only with the telemetry rate
#define CL_CURRENT_CHIRP_DURATION      40.0f
#define CL_CURRENT_CHIRP_AMPLITUDE     0.8f    // A
#define CL_CURRENT_CHIRP_SETTLE_TICKS  20u

// =============================================================================
// Position loop
// =============================================================================

// Pure P at 1 kHz, with velocity feedforward in trajectory mode. The position
// loop is an integrator plus the velocity loop's lag, so
// PM ~= 90deg - (velocity loop phase lag at gc). Reading the lag off the
// measured velocity loop: ~29deg at 60Hz, so gc 60Hz for a 60deg target,
// which is 260 * 60/41.2 = 380.
// Measured closed loop: crossover 61.2Hz, PM 61.2deg.
// Revert: 260.0f (gc 41.2Hz, PM 70.4deg, GM 14.9dB).
#define POSITION_LOOP_KP        380.0f   // (rad/s) per rad

// Clamp on the position loop's velocity command, about 25.5 mm/s at the
// stage. 80 rad/s is the fastest speed verified clean: the 30-40Hz cruise
// oscillation seen with the old boxcar velocity estimate is at noise level
// with the observer (+-8 mm/s -> 0.05-0.07 mm/s).
#define POSITION_VEL_LIMIT      80.0f    // rad/s

// Following-error trip (trajectory mode). If |pos_cmd - pos_fbk| exceeds this
// while a move is running, the drive faults: bridge off, STATE_FAULT latched
// until reset, FAULT_FOLLOWING_ERROR set in the telemetry fault_flags. A
// normal 10mm move peaks at 1.71um (7 counts); 1mm is 4096 counts, so this
// only fires on a runaway.
#define FOLLOWING_ERROR_LIMIT_MM      1.0f
#define FOLLOWING_ERROR_LIMIT_COUNTS  \
    ((int32_t)(FOLLOWING_ERROR_LIMIT_MM * (float)ENCODER_CPR / STAGE_LEAD_MM_PER_REV))

// =============================================================================
// Closed position loop chirp (test 8)
// =============================================================================

// Chirps pos_cmd with the full cascade closed and measures pos_meas/pos_cmd
// directly.
#define CL_POS_CHIRP_F_START     0.1f     // Hz
#define CL_POS_CHIRP_F_END     300.0f     // Hz
#define CL_POS_CHIRP_DURATION   60.0f     // s
#define CL_POS_CHIRP_SETTLE_TICKS  1000u  // 1s @ 1kHz, position held at 0 first

// Which velocity loop the position chirp runs on:
//   0 = identification: PI only, no friction feedforward. The Coulomb term
//       flips sign at every velocity zero crossing, so with it in the loop
//       no amplitude gives a linear Bode. Use this for crossover and margin.
//   1 = verification: velocity_loop_step(), exactly what ships. Read it in
//       the time domain, not as a Bode plot.
#define CL_POS_CHIRP_DEPLOYED    0

// Small enough that the loop stays off POSITION_VEL_LIMIT and VEL_IQ_LIMIT
// near crossover. A saturated loop measures as a describing function, not a
// transfer function, and its PM depends on amplitude.
#define CL_POS_CHIRP_AMPLITUDE  0.10f   // rad

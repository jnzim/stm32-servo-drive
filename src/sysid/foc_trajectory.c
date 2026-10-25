// foc_trajectory.c — closed-loop trajectory mode FOC
//
// RUN_MODE == RUN_MODE_CLOSED_LOOP only.
//
// Loop structure:
//   20 kHz  TIM1 ISR  → foc_trajectory_step()
//               align stage (d-axis lock, encoder offset capture)
//               move generator + position loop, every 20th tick → 1 kHz
//               velocity loop, every 4th tick → 5 kHz
//               current loop  (pi_step, real i_q_meas feedback)
//   1 kHz   SysTick   → telemetry only (foc_trajectory_telemetry_tick)
//
// Gains: config.h (CURRENT_LOOP_*, VEL_*, POSITION_LOOP_KP).

#include "foc_trajectory.h"
#include "observer.h"
#include "config.h"
#include "encoder.h"
#include "current_feedback.h"
#include "pwm.h"
#include "loops.h"
#include "trap_gen.h"
#include "minjerk_gen.h"
#include "drive.h" // drive_is_servo_on() / drive_get_state()
#include "spi.h"   // spi_update_telem(), spi_move_seq()
#include "stm32f4xx.h" // __DMB

#include <math.h>
#include <stdint.h>

// Defined in main.c, incremented by SysTick -- no shared header for this
// one global yet, declared here the same way it already is in main.c.
extern volatile uint32_t tick_ms;

// =============================================================================
// Alignment stage
// =============================================================================

typedef enum
{
    TRAJ_STAGE_ALIGN = 0,
    TRAJ_STAGE_RUN,
} TrajStage;

static TrajStage traj_stage      = TRAJ_STAGE_ALIGN;
static uint32_t  traj_align_tick = 0;
static int32_t   traj_enc_offset = 0;

// The move generator, and a latch for a move requested before RUN begins
// (e.g. boot scaffolding firing immediately, well before ALIGN finishes) --
// see foc_trajectory_start_move()/position_loop_tick() below.
//
// TRAJ_PROFILE (config.h) picks the generator. Both share one interface, so
// the rest of this file calls move_gen_* and does not know which it is.
#if TRAJ_PROFILE == TRAJ_PROFILE_MINJERK
typedef MinJerkGen MoveGen;
#define move_gen_start   minjerk_gen_start
#define move_gen_step    minjerk_gen_step
#define move_gen_active  minjerk_gen_active
#else
typedef TrapGen    MoveGen;
#define move_gen_start   trap_gen_start
#define move_gen_step    trap_gen_step
#define move_gen_active  trap_gen_active
#endif

static MoveGen  g_move;

// FAULT_* bits (protocol.h) latched by this file, reported in every
// telemetry frame. Cleared only by reset, like STATE_FAULT itself.
static uint8_t  traj_fault_flags = 0;
static volatile bool g_move_pending = false;
static int32_t  g_move_target    = 0;
static int32_t  g_move_vel_max   = 0;
static int32_t  g_move_accel_max = 0;

// Latest profile command and the encoder position the position loop used
// with it, written together by the position loop, read by telemetry, so
// pos_cmd - pos_fbk in telemetry is the loop's own following error.
static volatile int32_t traj_pos_cmd_counts   = 0;
static volatile float   traj_vel_cmd_counts_s = 0.0f;
static volatile int32_t traj_pos_meas_counts  = 0;
static volatile uint32_t traj_pair_seq        = 0;   // bumped after each write

// The outer loops share loop_div (loops.c), a 0..19 tick counter: velocity
// on every 4th tick from 0, position on tick POS_LOOP_TICK. Position never
// lands on a velocity tick, so one ISR call runs at most one outer loop.
#define LOOP_DIV_TICKS  20u   // 20 kHz / 20 = 1 kHz position loop
#define VEL_LOOP_TICKS   4u   // 20 kHz / 4  = 5 kHz velocity loop
#define POS_LOOP_TICK    2u

// =============================================================================
// Applied voltage and measured iq — written each 20 kHz tick, read by the
// telemetry (SysTick)
// =============================================================================

static float foc_vq_applied = 0.0f;
static float foc_vd_applied = 0.0f;
static float foc_iq_meas    = 0.0f;


static void position_loop_tick(void);

// =============================================================================
// foc_trajectory_step — called from TIM1_UP_TIM10_IRQHandler at 20 kHz
// ALIGN: hold the rotor at electrical angle 0 and capture the encoder offset.
// RUN: position loop (1 kHz), velocity loop (5 kHz) and current loop (20 kHz).
// =============================================================================

void foc_trajectory_step(void)
{
    switch (traj_stage)
    {
        // ---------------------------------------------------------------------
        case TRAJ_STAGE_ALIGN:
        {
            foc_vd_applied = V_ALIGN;
            foc_vq_applied = 0.0f;
            pwm_apply_dq(foc_vd_applied, foc_vq_applied, 0.0f);

            if (++traj_align_tick >= ALIGN_TICKS)
            {
                traj_enc_offset = encoder_get_position();
                traj_stage      = TRAJ_STAGE_RUN;
                loops_reset();  // zero integrators and setpoints after align
            }
        }
        break;

        // ---------------------------------------------------------------------
        case TRAJ_STAGE_RUN:
        {
            if (!drive_is_servo_on())
                return;

            loop_div++;
            if (loop_div >= LOOP_DIV_TICKS)
                loop_div = 0u;

            // ── Position loop @ 1 kHz ────────────────────────────────────────
            // Runs ahead of the gates below: it is what sets first_sample_ready.
            if (loop_div == POS_LOOP_TICK)
            {
                // position loop: next profile point, read encoder, following-error trip,
                // vel_cmd = vel FF + Kp*(pos error)
                position_loop_tick();
            }

            if (!first_sample_ready)
                return;

            if (!current_feedback_sample_valid())
                return;

            float theta = encoder_elec_angle(traj_enc_offset);

            // ── Current feedback ─────────────────────────────────────────────
            float i_d_meas;
            current_feedback_get_dq(theta, &i_d_meas, &foc_iq_meas);

            // ── Velocity observer @ 20 kHz ───────────────────────────────────
            // velocity_feedback_rad() below returns the observer's omega_hat
            // when VEL_LOOP_USES_OBSERVER=1, so the observer has to be
            // advanced here, every tick, exactly as foc_sysid_step() does.
            // Without this call omega_hat stays 0 and the velocity loop runs
            // with no feedback at all.
#if OBSERVER_ENABLE
            observer_update(encoder_get_position(), foc_iq_meas);
#endif

            // ── Velocity loop @ 5 kHz ────────────────────────────────────────
            if (loop_div % VEL_LOOP_TICKS == 0u)
            {
                float vel_fbk_rad = velocity_feedback_rad(encoder_get_velocity());
                iq_cmd = velocity_loop_step(vel_cmd_rad_sec, vel_fbk_rad, DT_VELOCITY);
            }

            // ── Current loop @ 20 kHz ────────────────────────────────────────
            // d axis regulated to zero, same PI and gains as q -- exactly as
            // foc_sysid.c does, where the current loop was tuned and verified.
            // Previously vd was hardwired to 0, leaving i_d uncontrolled.
            foc_vd_applied = pi_step(&d_current_loop,
                                     0.0f - i_d_meas,
                                     DT_CURRENT);
            foc_vq_applied = pi_step(&current_loop,
                                     iq_cmd - foc_iq_meas,
                                     DT_CURRENT);
            pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
        }
        break;
    }
}

// =============================================================================
// position_loop_tick — 1 kHz, from foc_trajectory_step() (TIM1 ISR) while
// SERVO_ON and past ALIGN
//
// Starts a pending MoveCmd, steps the move generator, runs the position loop
// and the following-error trip. ALIGN holds for ALIGN_TICKS (5 s) before
// traj_enc_offset is captured, so a move that arrives earlier waits here.
// =============================================================================

static void position_loop_tick(void)
{
    if (g_move_pending)
    {
        // g_move_target is a delta from wherever the stage actually is
        // right now, not an absolute position -- resolved here, at the
        // moment the move actually starts (traj_enc_offset is only
        // valid once ALIGN has finished).
        int32_t start_pos       = encoder_get_position() - traj_enc_offset;
        int32_t absolute_target = start_pos + g_move_target;
        move_gen_start(&g_move, start_pos, absolute_target, g_move_vel_max, g_move_accel_max);
        g_move_pending = false;
    }

    int32_t pos_cmd_counts;
    float   vel_cmd_counts_s;
    move_gen_step(&g_move, DT_POSITION, &pos_cmd_counts, &vel_cmd_counts_s);
    traj_pos_cmd_counts   = pos_cmd_counts;
    traj_vel_cmd_counts_s = vel_cmd_counts_s;

    // Command and measured position, in rad for the position P loop; the counts go to telemetry.
    float   pos_cmd_rad  = (float)pos_cmd_counts / COUNTS_PER_RAD;
    int32_t raw          = encoder_get_position() - traj_enc_offset;
    float   pos_meas_rad = (float)raw / COUNTS_PER_RAD;
    traj_pos_meas_counts = raw;
    traj_pair_seq++;   // new cmd/meas pair written: telemetry re-reads if this changes mid-read

    // Following-error trip -- see FOLLOWING_ERROR_LIMIT_MM (config.h).
    // drive_sm_run() takes the fault on the next 1ms tick: bridge off,
    // STATE_FAULT latched.
    int32_t following_err = pos_cmd_counts - raw;
    if (following_err >  FOLLOWING_ERROR_LIMIT_COUNTS ||
        following_err < -FOLLOWING_ERROR_LIMIT_COUNTS)
    {
        traj_fault_flags |= FAULT_FOLLOWING_ERROR;
        drive_request_fault();
    }

    // Velocity feedforward plus position P. Without the feedforward the
    // cruise following error is v/Kp. The clamp (p_step has none of its
    // own) applies to the total: at 10 mm/s the feedforward alone is
    // 31.4 of the 80 rad/s POSITION_VEL_LIMIT.
    float v = (vel_cmd_counts_s / COUNTS_PER_RAD) + p_step(&position_loop, pos_cmd_rad - pos_meas_rad);
    if (v >  POSITION_VEL_LIMIT) v =  POSITION_VEL_LIMIT;
    if (v < -POSITION_VEL_LIMIT) v = -POSITION_VEL_LIMIT;
    vel_cmd_rad_sec = v;

    samples_consumed++;
    first_sample_ready = 1;
}

// =============================================================================
// foc_trajectory_telemetry_tick — call from SysTick at 1 kHz, every tick
//
// Sends a TelemetryFrame every tick regardless of drive state, so the Pi
// sees the drive return to IDLE.
// =============================================================================

void foc_trajectory_telemetry_tick(void)
{
    int32_t pos_cmd_counts   = (int32_t)lroundf(g_move.pos);
    float   vel_cmd_counts_s = 0.0f;
    int32_t pos_fbk          = encoder_get_position() - traj_enc_offset;

    if (drive_is_servo_on() && traj_stage == TRAJ_STAGE_RUN)
    {
        // The position loop (TIM1 ISR) can interrupt these reads and write a
        // new set halfway through. Re-read until no write landed in between;
        // the next write is 1 ms away, so this repeats at most once.
        uint32_t seq;
        do
        {
            seq              = traj_pair_seq;
            pos_cmd_counts   = traj_pos_cmd_counts;
            vel_cmd_counts_s = traj_vel_cmd_counts_s;
            pos_fbk          = traj_pos_meas_counts;
        } while (seq != traj_pair_seq);
    }

    // Real telemetry, every tick, regardless of drive state -- named
    // fields, no relabeling. iq_cmd/foc_iq_meas are already in amps
    // (current-loop domain); TelemetryFrame reports both in mA.
    TelemetryFrame frame = {
        .pos_cmd          = pos_cmd_counts,
        .pos_fbk          = pos_fbk,
        .vel_cmd          = (int32_t)lroundf(vel_cmd_counts_s),
        // The velocity the loop actually consumes (observer or boxcar, per
        // VEL_LOOP_USES_OBSERVER), in the field's counts/s units.
        .vel_fbk          = (int32_t)lroundf(velocity_feedback_rad(encoder_get_velocity())
                                             * COUNTS_PER_RAD),
        .timestamp_ms     = tick_ms,
        .drive_state      = (uint8_t)drive_get_state(),
        .fault_flags      = traj_fault_flags,
        .samples_consumed = (uint16_t)samples_consumed,
        .move_seq         = spi_move_seq(),
        .iq_cmd           = (int16_t)lroundf(iq_cmd * 1000.0f),      // A -> mA
        .i_q_fbk          = (int16_t)lroundf(foc_iq_meas * 1000.0f), // A -> mA
    };
    spi_update_telem(&frame);
}

// =============================================================================
// foc_trajectory_start_move / foc_trajectory_move_active — see header.
// =============================================================================

void foc_trajectory_start_move(int32_t target_pos, int32_t vel_max, int32_t accel_max)
{
    g_move_target    = target_pos;
    g_move_vel_max   = vel_max;
    g_move_accel_max = accel_max;
    __DMB();   // the TIM1 ISR can preempt this: parameters land before the flag
    g_move_pending   = true;
}

bool foc_trajectory_move_active(void)
{
    return g_move_pending || move_gen_active(&g_move);
}

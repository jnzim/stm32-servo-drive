// foc_trajectory.c — closed-loop trajectory mode FOC
//
// RUN_MODE == RUN_MODE_CLOSED_LOOP only.
//
// Loop structure:
//   20 kHz  TIM1 ISR  → foc_trajectory_step()
//               align stage (d-axis lock, encoder offset capture)
//               current loop  (pi_step, real i_q_meas feedback)
//               velocity loop decimated /4 → 5 kHz
//   1 kHz   SysTick   → position loop + ring_pop() → writes vel_cmd_rad_sec
//
// Gains in loops_reset() (loops.c) are tuned from confirmed sysid params:
//   R = 3.55 Ω,  L = 2.47 mH,  fc_plant = 229 Hz
//   Current loop target fc = 1 kHz
//   Kp = 2π × 1000 × L = 15.52
//   Ki = Kp × R/L       = 22306

#include "foc_trajectory.h"
#include "config.h"
#include "encoder.h"
#include "current_feedback.h"
#include "pwm.h"
#include "loops.h"
#include "trap_gen.h"
#include "minjerk_gen.h"
#include "drive.h" // drive_is_servo_on() / drive_get_state()
#include "spi.h"   // spi_update_telem()

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
// see foc_trajectory_start_move()/foc_trajectory_position_tick() below.
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
static bool     g_move_pending   = false;
static int32_t  g_move_target    = 0;
static int32_t  g_move_vel_max   = 0;
static int32_t  g_move_accel_max = 0;

// =============================================================================
// Applied voltage — written each ISR, exposed for telemetry in SysTick
// =============================================================================

float foc_vq_applied = 0.0f;
float foc_vd_applied = 0.0f;
float foc_iq_meas     = 0.0f;  // real i_q feedback, exposed for telemetry in SysTick

// =============================================================================
// foc_theta_from_encoder — electrical angle relative to alignment offset
// =============================================================================

static float foc_theta_from_encoder(void)
{
    int32_t raw = encoder_get_position() - traj_enc_offset;

    float theta_mech = (float)raw * (FOC_TWO_PI / ENCODER_CPR);
    float theta_elec = ENC_DIR * theta_mech * (float)MOTOR_POLE_PAIRS;

    theta_elec = fmodf(theta_elec, FOC_TWO_PI);
    if (theta_elec < 0.0f)
        theta_elec += FOC_TWO_PI;

    return theta_elec;
}

// =============================================================================
// foc_trajectory_step — called from TIM1_UP_TIM10_IRQHandler at 20 kHz
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

            if (!first_sample_ready)
                return;

            if (!current_feedback_sample_valid())
                return;

            float theta = foc_theta_from_encoder();

            // ── Current feedback ─────────────────────────────────────────────
            float i_d_meas;
            current_feedback_get_dq(theta, &i_d_meas, &foc_iq_meas);

            // ── Velocity loop @ 5 kHz ────────────────────────────────────────
            if (++vel_div >= 4u)
            {
                vel_div = 0u;
                float vel_fbk_rad = (float)encoder_get_velocity() / COUNTS_PER_RAD;
                iq_cmd = velocity_loop_step(vel_cmd_rad_sec, vel_fbk_rad, DT_VELOCITY);
            }

            // ── Current loop @ 20 kHz ────────────────────────────────────────
            foc_vd_applied = 0.0f;
            foc_vq_applied = pi_step(&current_loop,
                                     iq_cmd - foc_iq_meas,
                                     DT_CURRENT);
            pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
        }
        break;
    }
}

// =============================================================================
// foc_trajectory_position_tick — call from SysTick at 1 kHz, SERVO_ON only
//
// M1 (trajectory-streaming plan): fake hardcoded ramp+hold trajectory, no
// ring buffer yet -- proves the position-loop-in-SysTick plumbing in
// isolation before the real transport (ring buffer, then SPI RX) is
// trusted. Target/rate deliberately small and slow -- comparable to
// SYSID_TEST_POSITION_STEP's own amplitude (POSITION_STEP_AMPLITUDE_RAD),
// not exploring new dynamic territory while also exercising brand-new code.
//
// Gated on traj_stage == TRAJ_STAGE_RUN, not just drive_is_servo_on(): boot
// scaffolding (main.c) requests SERVO_ON immediately at boot, but
// foc_trajectory_step()'s own ALIGN stage still holds for ALIGN_TICKS
// (100000 @ 20kHz = 5s) before traj_enc_offset is captured and the
// current/velocity loop actually goes live. Starting the generator before
// that would have the move complete (or sit mid-move) against a dead loop,
// so a pending move waits here for RUN before move_gen_start() runs.
// =============================================================================

void foc_trajectory_position_tick(void)
{
    int32_t pos_cmd_counts   = (int32_t)lroundf(g_move.pos);
    float   vel_cmd_counts_s = 0.0f;

    // Only actually run the generator/position loop while servo is on and
    // past ALIGN -- but telemetry below is reported every tick regardless,
    // so drive_state always reflects reality. Previously this whole
    // function (including the telemetry write) was skipped once
    // drive_is_servo_on() went false, which meant the tick that flips
    // drive_state to IDLE never got reported -- the Pi saw the last
    // SERVO_ON frame forever and never observed completion.
    if (drive_is_servo_on() && traj_stage == TRAJ_STAGE_RUN)
    {
        if (g_move_pending)
        {
            // g_move_target is a delta from wherever the stage actually is
            // right now, not an absolute position -- resolved here, at the
            // moment the move actually starts (traj_enc_offset is only
            // valid once ALIGN has finished, which is why this waits for
            // TRAJ_STAGE_RUN rather than resolving it back when the
            // command was first received).
            int32_t start_pos      = encoder_get_position() - traj_enc_offset;
            int32_t absolute_target = start_pos + g_move_target;
            move_gen_start(&g_move, start_pos, absolute_target, g_move_vel_max, g_move_accel_max);
            g_move_pending = false;
        }

        move_gen_step(&g_move, DT_POSITION, &pos_cmd_counts, &vel_cmd_counts_s);
        float pos_cmd_rad = (float)pos_cmd_counts / COUNTS_PER_RAD;

        int32_t raw          = encoder_get_position() - traj_enc_offset;
        float   pos_meas_rad = (float)raw * (FOC_TWO_PI / ENCODER_CPR);

        // p_step() has no built-in output clamp (unlike pi_step()'s
        // PIState) -- manual clamp, same idiom as foc_sysid.c's
        // run_position_step().
        //
        // Velocity feedforward: the generator already knows what velocity it
        // is asking for, so hand that to the velocity loop directly and leave
        // the position loop to correct only what is left over.
        //
        // Without it this is pure position feedback, and cruise following
        // error is then exactly v/Kp. Measured 2026-09-20, 10mm at 10mm/s:
        // 157.2 counts (38.4um) of lag, backing out Kv=260.6 1/s against
        // POSITION_LOOP_KP=260.0 -- 0.2% agreement, which is what a type-1
        // loop with no feedforward has to do.
        //
        // The clamp applies to the TOTAL command, not just the correction: at
        // 10mm/s the feedforward term alone is 31.4 rad/s of the 50 rad/s
        // POSITION_VEL_LIMIT, so the correction has ~19 rad/s of headroom
        // before the clamp starts eating it.
        float v = (vel_cmd_counts_s / COUNTS_PER_RAD)
                + p_step(&position_loop, pos_cmd_rad - pos_meas_rad);
        if (v >  POSITION_VEL_LIMIT) v =  POSITION_VEL_LIMIT;
        if (v < -POSITION_VEL_LIMIT) v = -POSITION_VEL_LIMIT;
        vel_cmd_rad_sec = v;

        samples_consumed++;
        first_sample_ready = 1;
    }

    // Real telemetry, every tick, regardless of drive state -- named
    // fields, no relabeling. iq_cmd/foc_iq_meas are already in amps
    // (current-loop domain); TelemetryFrame reports both in mA.
    int32_t pos_fbk = encoder_get_position() - traj_enc_offset;
    TelemetryFrame frame = {
        .pos_cmd          = pos_cmd_counts,
        .pos_fbk          = pos_fbk,
        .vel_cmd          = (int32_t)lroundf(vel_cmd_counts_s),
        .vel_fbk          = encoder_get_velocity(),
        .timestamp_ms     = tick_ms,
        .drive_state      = (uint8_t)drive_get_state(),
        .fault_flags      = 0,
        .samples_consumed = samples_consumed,
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
    g_move_pending   = true;
}

bool foc_trajectory_move_active(void)
{
    return g_move_pending || move_gen_active(&g_move);
}

// =============================================================================
// foc_trajectory_reset — call on fault recovery or re-enable
// =============================================================================

void foc_trajectory_reset(void)
{
    traj_stage      = TRAJ_STAGE_ALIGN;
    traj_align_tick = 0;
    traj_enc_offset = 0;

    foc_vd_applied = 0.0f;
    foc_vq_applied = 0.0f;

    loops_reset();
}
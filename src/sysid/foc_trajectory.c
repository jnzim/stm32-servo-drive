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
#include "spi.h"   // M1/M2 diagnostic telemetry only -- spi_sysid_update_latest()
#include "drive.h" // M1/M2 diagnostic telemetry only -- drive_is_servo_on()

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

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

// M2: the real trapezoidal move generator (replaces M1's fake ramp).
// g_move_pending latches a move requested before RUN begins (e.g. a
// MOVE_CMD arriving during ALIGN) so it isn't lost -- foc_trajectory_
// position_tick() starts the generator for real once traj_stage reaches
// RUN and a pending move exists.
static TrapGen g_move;
static bool    g_move_pending      = false;
static int32_t g_move_target       = 0;
static int32_t g_move_vel_max      = 0;
static int32_t g_move_accel_max    = 0;

// Last generator output -- written by foc_trajectory_position_tick()
// (SysTick, 1kHz), read by the diagnostic telemetry block in
// foc_trajectory_step() (TIM1, 20kHz) without re-advancing the generator.
static int32_t g_last_pos_cmd_counts   = 0;
static float   g_last_vel_cmd_counts_s = 0.0f;

// =============================================================================
// Applied voltage — written each ISR, exposed for telemetry in SysTick
// =============================================================================

float foc_vq_applied = 0.0f;
float foc_vd_applied = 0.0f;

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
            // M1 diagnostic: the real "test over" signal must be reachable
            // BEFORE the first_sample_ready gate below, not after -- drive.c's
            // SERVO_ON auto-exit clears first_sample_ready in the same
            // SysTick tick it flips the drive state, so a flags=2 report
            // placed after that gate can never actually run (confirmed: this
            // is exactly why the Pi's capture tool never saw the stop signal
            // and never auto-stopped last run, even though the underlying
            // motion was clean). Minimal frame, no current-loop data needed.
            if (!drive_is_servo_on())
            {
                spi_sysid_update_latest(0, 0, 0, 0, 0, 0, 0, 0, 0u, 0u, 0u, 2u, 99u);
                return;
            }

            if (!first_sample_ready)
                return;

            if (!current_feedback_sample_valid())
                return;

            float theta = foc_theta_from_encoder();

            // ── Current feedback ─────────────────────────────────────────────
            float i_d_meas, i_q_meas;
            current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

            // ── Velocity loop @ 5 kHz ────────────────────────────────────────
            if (++vel_div >= 4u)
            {
                vel_div = 0u;
                float vel_fbk_rad = (float)encoder_get_velocity() / COUNTS_PER_RAD;
                iq_cmd = pi_step(&velocity_loop,
                                 vel_cmd_rad_sec - vel_fbk_rad,
                                 DT_VELOCITY);
            }

            // ── Current loop @ 20 kHz ────────────────────────────────────────
            foc_vd_applied = 0.0f;
            foc_vq_applied = pi_step(&current_loop,
                                     iq_cmd - i_q_meas,
                                     DT_CURRENT);
            pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);

            // ── M1/M2 diagnostic telemetry -- TEMPORARY, remove once M2 is
            // fully verified. Reuses the proven free-running SysIdSample TX
            // path (works regardless of RUN_MODE) at full 20kHz resolution.
            // Units chosen so nothing clips int16 even during a large
            // transient: position/theta in mrad, velocity in centirad/s
            // (0.01 rad/s -- ±327 rad/s headroom), current in mA. The
            // "test over" (flags=2) signal is sent above, before
            // first_sample_ready gates this block -- diag_flags here is
            // always traj_stage (0/1) since we only reach this while
            // servo_on is true. pos_cmd/vel_cmd read the generator's last
            // output (g_last_*, written by foc_trajectory_position_tick())
            // rather than re-stepping it -- stepping here too would
            // silently double-advance elapsed time.
            {
                int32_t raw_diag          = encoder_get_position() - traj_enc_offset;
                float   pos_meas_rad_diag = (float)raw_diag * (FOC_TWO_PI / ENCODER_CPR);
                float   vel_fbk_rad_diag  = (float)encoder_get_velocity() / COUNTS_PER_RAD;
                float   pos_cmd_rad_diag  = (float)g_last_pos_cmd_counts / COUNTS_PER_RAD;
                float   gen_vel_rad_diag  = g_last_vel_cmd_counts_s / COUNTS_PER_RAD;
                uint16_t diag_flags       = (uint16_t)traj_stage;

                spi_sysid_update_latest(
                    (int16_t)(pos_cmd_rad_diag       * 1000.0f), /* ia_mA slot: pos_cmd [mrad] */
                    (int16_t)(pos_meas_rad_diag      * 1000.0f), /* ib_mA slot: pos_meas [mrad] */
                    (int16_t)(gen_vel_rad_diag       *  100.0f), /* ic_mA slot: generator vel [0.01 rad/s] */
                    (int16_t)(vel_fbk_rad_diag       *  100.0f), /* id_mA slot: vel_fbk [0.01 rad/s] */
                    (int16_t)(iq_cmd                 * 1000.0f), /* iq_mA slot: iq_cmd [mA] */
                    (int16_t)(i_q_meas               * 1000.0f), /* vd_mV slot: i_q_meas [mA] */
                    (int16_t)(foc_vq_applied         * 1000.0f), /* vq_mV slot: applied vq [mV] */
                    (int16_t)(theta                  * 1000.0f), /* theta_mrad slot */
                    0u, 0u, 0u,
                    diag_flags,
                    98u                     /* test_id: M2 diagnostic marker */
                );
            }
        }
        break;
    }
}

// =============================================================================
// foc_trajectory_start_move — request a move. Safe to call from any
// context/priority (SPI RX ISR is the intended caller once M2's opcode
// parsing is wired up) -- just captures the current position and latches
// the request; foc_trajectory_position_tick() (SysTick, strictly lower
// priority than the RX ISR) actually starts the generator, so there's no
// torn-read risk between the two even without extra locking (a
// higher-priority ISR always runs to completion before a lower one
// resumes on Cortex-M).
//
// target_pos/vel_max/accel_max are all in the same "counts relative to
// traj_enc_offset" frame the position loop already uses -- NOT raw
// encoder_get_position() counts.
// =============================================================================

void foc_trajectory_start_move(int32_t target_pos, int32_t vel_max, int32_t accel_max)
{
    g_move_target    = target_pos;
    g_move_vel_max   = vel_max;
    g_move_accel_max = accel_max;
    g_move_pending   = true;
}

// =============================================================================
// foc_trajectory_move_active — see header. g_move_pending covers the window
// between a MOVE_CMD arriving and foc_trajectory_position_tick() actually
// starting the generator (e.g. a command received during ALIGN); trap_gen_
// active(&g_move) covers the generator's own run.
// =============================================================================

bool foc_trajectory_move_active(void)
{
    return g_move_pending || trap_gen_active(&g_move);
}

// =============================================================================
// foc_trajectory_position_tick — call from SysTick at 1 kHz, SERVO_ON only
//
// M2: real trapezoidal generator (trap_gen.c), replacing M1's fake ramp
// (which proved this SysTick/position-loop plumbing in isolation first).
//
// Gated on traj_stage == TRAJ_STAGE_RUN, not just drive_is_servo_on(): the
// scaffolding trigger can request SERVO_ON before foc_trajectory_step()'s
// own ALIGN stage (ALIGN_TICKS @ 20kHz = 5s) has captured traj_enc_offset
// and gone live -- starting the generator against that dead period would
// have it complete (or sit mid-move) before real tracking even begins,
// same class of bug M1 already found and fixed for the fake ramp.
// =============================================================================

void foc_trajectory_position_tick(void)
{
    if (traj_stage != TRAJ_STAGE_RUN)
        return;   // still aligning -- don't start/advance a move against a dead loop

    if (g_move_pending)
    {
        int32_t start_pos = encoder_get_position() - traj_enc_offset;
        trap_gen_start(&g_move, start_pos, g_move_target, g_move_vel_max, g_move_accel_max);
        g_move_pending = false;
    }

    int32_t pos_cmd_counts;
    float   vel_cmd_counts_s;
    trap_gen_step(&g_move, DT_POSITION, &pos_cmd_counts, &vel_cmd_counts_s);
    g_last_pos_cmd_counts   = pos_cmd_counts;
    g_last_vel_cmd_counts_s = vel_cmd_counts_s;

    float pos_cmd_rad = (float)pos_cmd_counts / COUNTS_PER_RAD;

    int32_t raw          = encoder_get_position() - traj_enc_offset;
    float   pos_meas_rad = (float)raw * (FOC_TWO_PI / ENCODER_CPR);

    // p_step() has no built-in output clamp (unlike pi_step()'s PIState) --
    // manual clamp, same idiom as foc_sysid.c's run_position_step(). Pure
    // feedback for now, no velocity feedforward from the generator's own
    // vel_cmd_counts_s yet -- deferred to its own separate validation once
    // basic tracking is proven, per the plan.
    float v = p_step(&position_loop, pos_cmd_rad - pos_meas_rad);
    if (v >  POSITION_VEL_LIMIT) v =  POSITION_VEL_LIMIT;
    if (v < -POSITION_VEL_LIMIT) v = -POSITION_VEL_LIMIT;
    vel_cmd_rad_sec = v;

    samples_consumed++;
    first_sample_ready = 1;
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
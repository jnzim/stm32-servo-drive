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
#include "spi.h"   // M1 diagnostic telemetry only -- spi_sysid_update_latest()
#include "drive.h" // M1 diagnostic telemetry only -- drive_is_servo_on()

#include <math.h>
#include <stdint.h>

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

// M1 fake trajectory state -- declared here (not just above
// foc_trajectory_position_tick()) so the M1 diagnostic telemetry block in
// foc_trajectory_step() can read it too. Goes away in M2.
static float fake_traj_pos_cmd_rad = 0.0f;

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

            // ── M1 diagnostic telemetry -- TEMPORARY, remove once M1 is
            // fully verified. Confirmed clean (no runaway) on the last
            // capture; this is now just tracking/logging visibility.
            // Reuses the proven free-running SysIdSample TX path (works
            // regardless of RUN_MODE) at full 20kHz resolution. Units
            // chosen so nothing clips int16 even during a large transient:
            // position/theta in mrad, velocity in centirad/s (0.01 rad/s
            // -- ±327 rad/s headroom), current in mA. The "test over"
            // (flags=2) signal is sent above, before first_sample_ready
            // gates this block -- diag_flags here is always traj_stage
            // (0/1) since we only reach this while servo_on is true.
            {
                int32_t raw_diag          = encoder_get_position() - traj_enc_offset;
                float   pos_meas_rad_diag = (float)raw_diag * (FOC_TWO_PI / ENCODER_CPR);
                float   vel_fbk_rad_diag  = (float)encoder_get_velocity() / COUNTS_PER_RAD;
                uint16_t diag_flags       = (uint16_t)traj_stage;

                spi_sysid_update_latest(
                    (int16_t)(fake_traj_pos_cmd_rad * 1000.0f),  /* ia_mA slot: pos_cmd [mrad] */
                    (int16_t)(pos_meas_rad_diag      * 1000.0f), /* ib_mA slot: pos_meas [mrad] */
                    (int16_t)(vel_cmd_rad_sec        *  100.0f), /* ic_mA slot: vel_cmd [0.01 rad/s] */
                    (int16_t)(vel_fbk_rad_diag       *  100.0f), /* id_mA slot: vel_fbk [0.01 rad/s] */
                    (int16_t)(iq_cmd                 * 1000.0f), /* iq_mA slot: iq_cmd [mA] */
                    (int16_t)(i_q_meas               * 1000.0f), /* vd_mV slot: i_q_meas [mA] */
                    (int16_t)(foc_vq_applied         * 1000.0f), /* vq_mV slot: applied vq [mV] */
                    (int16_t)(theta                  * 1000.0f), /* theta_mrad slot */
                    0u, 0u, 0u,
                    diag_flags,
                    99u                     /* test_id: M1 diagnostic marker */
                );
            }
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
// Gated on traj_stage == TRAJ_STAGE_RUN, not just drive_is_servo_on(): the
// M1 scaffolding in main.c requests SERVO_ON immediately at boot, but
// foc_trajectory_step()'s own ALIGN stage still holds for ALIGN_TICKS
// (100000 @ 20kHz = 5s) before traj_enc_offset is captured and the
// current/velocity loop actually goes live. Without this gate the ramp
// would fully complete (200ms) and sit at its target for the rest of that
// 5s dead period, so real tracking would start with a full-scale step
// instead of a gentle ramp the instant RUN begins -- caught before ever
// flashing this.
//
// Replaced by a real ring_pop() in M2 -- this function's fake-trajectory
// state (fake_traj_pos_cmd_rad) goes away then, the p_step/clamp/
// vel_cmd_rad_sec idiom below stays (same pattern as foc_sysid.c's
// run_position_step(), just fed from a different source).
// =============================================================================

#define FAKE_TRAJ_TARGET_RAD     3.0f  // ~172 deg -- bumped from 1.0 (matched
                                        // POSITION_STEP_AMPLITUDE_RAD) so the
                                        // move is actually visible on the
                                        // bench; still under a full turn,
                                        // still well inside what's already
                                        // been run tonight (POSITION_STEP
                                        // amplitude has been as high as
                                        // 10.0f rad earlier this session)
#define FAKE_TRAJ_RATE_RAD_S     5.0f  // ramp rate -- well under POSITION_VEL_LIMIT
#define FAKE_TRAJ_RATE_PER_TICK  (FAKE_TRAJ_RATE_RAD_S / 1000.0f)  // per 1kHz tick

void foc_trajectory_position_tick(void)
{
    if (traj_stage != TRAJ_STAGE_RUN)
        return;   // still aligning -- don't advance the ramp against a dead loop

    if (fake_traj_pos_cmd_rad < FAKE_TRAJ_TARGET_RAD)
    {
        fake_traj_pos_cmd_rad += FAKE_TRAJ_RATE_PER_TICK;
        if (fake_traj_pos_cmd_rad > FAKE_TRAJ_TARGET_RAD)
            fake_traj_pos_cmd_rad = FAKE_TRAJ_TARGET_RAD;
    }

    int32_t raw          = encoder_get_position() - traj_enc_offset;
    float   pos_meas_rad = (float)raw * (FOC_TWO_PI / ENCODER_CPR);

    // p_step() has no built-in output clamp (unlike pi_step()'s PIState) --
    // manual clamp, same idiom as foc_sysid.c's run_position_step().
    float v = p_step(&position_loop, fake_traj_pos_cmd_rad - pos_meas_rad);
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
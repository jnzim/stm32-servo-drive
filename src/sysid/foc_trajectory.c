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
            if (!first_sample_ready)
                return;

            if (!current_feedback_sample_valid())
                return;

            float theta = foc_theta_from_encoder();

            // ── Current feedback ─────────────────────────────────────────────
            float i_d_meas, i_q_meas;
            current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

            // ── Velocity loop @ 5 kHz ────────────────────────────────────────
            // if (++vel_div >= 4u)
            // {
            //     vel_div = 0u;
            //     float vel_fbk_rad = (float)encoder_get_velocity() / COUNTS_PER_RAD;
            //     iq_cmd = pi_step(&velocity_loop,
            //                      vel_cmd_rad_sec - vel_fbk_rad,
            //                      DT_VELOCITY);
            // }

            // ── Current loop @ 20 kHz ────────────────────────────────────────
            foc_vd_applied = 0.0f;
            foc_vq_applied = pi_step(&current_loop,
                                     iq_cmd - i_q_meas,
                                     DT_CURRENT);
            if( foc_vq_applied > 0 )
            {
                volatile float testVar = foc_vq_applied;
            }
            pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
        }
        break;
    }
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
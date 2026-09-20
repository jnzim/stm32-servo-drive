// foc_sysid.c — system identification mode FOC
//
// Extracted from main.c (jz-f411). Logic unchanged from working sysid firmware.
// Added: FOC_SYSID_CL_STEP test mode for closed-loop current step response.
//        FOC_SYSID_CL_VEL_STEP test mode for closed-loop velocity step response.
//
// Internal test selected by SYSID_TEST in config.h:
//   SYSID_TEST_CHIRP        — OL Vd chirp sweep (proven, existing)
//   SYSID_TEST_CL_STEP      — CL iq step, current loop closed, logs iq_ref/iq_meas
//   SYSID_TEST_CL_VEL_STEP  — CL velocity step, velocity loop closed
//                             sysid_f_Hz slot  → vel_cmd  [mrad/s]  Python: val/1000.0
//                             iq_cmd_mA slot   → vel_meas [counts/s] Python: val*(2pi/8192)

#include "stm32f4xx.h"
#include "foc_sysid.h"
#include "config.h"
#include "encoder.h"
#include "current_feedback.h"
#include "pwm.h"
#include "loops.h"
#include "spi.h"

#include <math.h>
#include <stdint.h>
#include <stdbool.h>


// =============================================================================
// Alignment stage
// =============================================================================

typedef enum
{
    SYSID_STAGE_ALIGN = 0,
    SYSID_STAGE_RUN,
    SYSID_STAGE_IDLE
} SysidStage;

static SysidStage   sysid_stage             = SYSID_STAGE_ALIGN;
static uint32_t     sysid_align_tick        = 0;
static uint32_t     sysid_discharge_tick    = 0;
static int32_t      sysid_enc_offset        = 0;
static uint32_t     vel_chirp_settle_tick   = 0;

// =============================================================================
// Chirp state
// =============================================================================

static float sysid_t            = 0.0f;
static float chirp_angle_rad    = 0.0f;
static float sysid_f            = SYSID_F_START;

// =============================================================================
// CL step state
// =============================================================================

#define CL_STEP_AMPLITUDE       0.3f        // A
#define CL_STEP_VEL_AMPLITUDE   10.0       // rad/s
#define CL_STEP_HOLD_TICKS      5000u       
#define CL_STEP_SETTLE_TICKS    1000u     

// Was 20000 (4.0s @ the 5kHz decimated vel-loop rate) for no real reason --
// loops_reset() already zeroes every PI state at the ALIGN->RUN transition,
// and real current is already at genuine zero by then too (see the align
// discharge phase). The only actual constraint is encoder_get_velocity()'s
// 80-sample moving-average filter (VEL_FILTER_N in encoder.c) needing to
// fill before vel_meas_counts is trustworthy -- 80 samples @ 20kHz = 4ms =
// 20 ticks @ the 5kHz decimated rate this counter runs at. Any more than
// that just gives a persistent current bias more time to wind up the
// d-axis integrator before the real test signal starts, for no benefit.
//
// Named CL_-specific and kept separate from VEL_CHIRP_SETTLE_TICKS
// (config.h) on purpose -- run_cl_vel_chirp()'s settle counter runs at this
// 5kHz-decimated rate, but run_vel_chirp()'s (the open-loop test) runs on
// the raw undecimated 20kHz tick. Sharing one constant between them
// silently broke the open-loop test's settle time (4ms instead of ~1s)
// when this one got tuned down for the closed-loop context.
#define CL_VEL_CHIRP_SETTLE_TICKS  20u

typedef enum
{
    CL_STEP_SETTLE = 0,
    CL_STEP_HIGH,
    CL_STEP_LOW,
    CL_STEP_ZERO,
    CL_STEP_DONE
} ClStepPhase;

static ClStepPhase cl_step_phase = CL_STEP_SETTLE;
static uint32_t    cl_step_tick  = 0;

// =============================================================================
// Shared measured values (written each ISR, read by spi_sysid_update_latest)
// =============================================================================

static float ia_meas  = 0.0f;
static float ib_meas  = 0.0f;
static float ic_meas  = 0.0f;
static float i_d_meas = 0.0f;
static float i_q_meas = 0.0f;

static float foc_vd_applied = 0.0f;
static float foc_vq_applied = 0.0f;

// Velocity step telemetry
// vel_cmd_rad_sec is defined in loops.c, declared extern in loops.h
static float vel_meas_counts = 0.0f;   // measured velocity [counts/s] — raw from encoder


// =============================================================================
// foc_theta_from_encoder — electrical angle relative to alignment offset
// =============================================================================

static float foc_theta_from_encoder(void)
{
    int32_t raw = encoder_get_position() - sysid_enc_offset;

    float theta_mech = (float)raw * (FOC_TWO_PI / ENCODER_CPR);
    float theta_elec = ENC_DIR * theta_mech * (float)MOTOR_POLE_PAIRS;

    theta_elec = fmodf(theta_elec, FOC_TWO_PI);
    if (theta_elec < 0.0f)
        theta_elec += FOC_TWO_PI;

    return theta_elec;
}


// =============================================================================
// run_voltage_test
// =============================================================================

static void run_voltage_test(void)
{
    pwm_apply_phase_volts(0.5f, -0.25f, -0.25f);

    sysid_t += SYSID_DT;

    if (sysid_t >= 30.0f)
    {
        pwm_apply_phase_volts(0.0f, 0.0f, 0.0f);
        sysid_stage = SYSID_STAGE_IDLE;
    }
}


// =============================================================================
// run_current_test
// =============================================================================

static void run_current_test(void)
{
    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);

    pwm_apply_phase_volts(0.5f, -0.25f, -0.25f);

    sysid_t += SYSID_DT;

    if (sysid_t >= 1.0f)
    {
        pwm_apply_phase_volts(0.0f, 0.0f, 0.0f);
        sysid_stage = SYSID_STAGE_IDLE;
    }
}


// =============================================================================
// run_chirp — Open current loop chirp. Input is Vq, measure Iq for f analysis
// =============================================================================

static void run_current_lp_chirp(void)
{
    sysid_f = SYSID_F_START * powf(SYSID_F_END / SYSID_F_START, sysid_t / SYSID_DURATION);

    chirp_angle_rad += FOC_TWO_PI * sysid_f * SYSID_DT;
    if (chirp_angle_rad >= FOC_TWO_PI)
        chirp_angle_rad -= FOC_TWO_PI;

    foc_vq_applied = SYSID_AMPLITUDE * sinf(chirp_angle_rad);
    foc_vd_applied = 0.0f;

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, 0.0f);

    current_feedback_get_dq(0.0f, &i_d_meas, &i_q_meas);

    sysid_t += SYSID_DT;
    if (sysid_t >= SYSID_DURATION)
    {
        sysid_stage = SYSID_STAGE_IDLE;
    }
}


// =============================================================================
// run_cl_current_chirp — CLOSED current loop chirp: sweeps iq_cmd (not Vq)
// with the current PI closed around it, to directly measure the closed-loop
// response H(s) = iq_meas/iq_cmd and back out the loop's real open-loop
// margin via L = H/(1-H) -- exact algebra, no assumption that C(s) is the
// linear Kp+Ki/s it's designed to be. bode_plot.py's method (measure the
// bare electrical plant open-loop, then combine it with the *assumed*
// linear C(s)) was shown on the velocity loop to disagree with a real
// closed-loop measurement by 37% on crossover frequency, most likely from
// saturation -- this test exists to get the current loop's real margin the
// same trustworthy way, not repeat that assumption here too.
//
// Velocity loop plays no part in this test: iq_cmd is driven directly by
// the chirp, same as run_current_lp_chirp's open-loop version -- the only
// difference is that reference now goes through the closed current PI
// instead of driving Vq directly.
//
// Telemetry slot usage:
//   sysid_f_Hz slot -> iq_cmd * 1000  [mA]
//   iq_cmd_mA slot  -> iq_meas (i_q_meas, already logged natively) [mA]
// =============================================================================

static float cl_current_chirp_t         = 0.0f;
static float cl_current_chirp_angle_rad = 0.0f;

static void run_cl_current_chirp(void)
{
    float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    if (vel_chirp_settle_tick < CL_CURRENT_CHIRP_SETTLE_TICKS)
    {
        iq_cmd = 0.0f;
        ++vel_chirp_settle_tick;
    }
    else
    {
        sysid_f = CL_CURRENT_CHIRP_F_START *
                  powf(CL_CURRENT_CHIRP_F_END / CL_CURRENT_CHIRP_F_START,
                       cl_current_chirp_t / CL_CURRENT_CHIRP_DURATION);

        cl_current_chirp_angle_rad += FOC_TWO_PI * sysid_f * SYSID_DT;
        if (cl_current_chirp_angle_rad >= FOC_TWO_PI)
            cl_current_chirp_angle_rad -= FOC_TWO_PI;

        // sinf, not cosf -- continuous with the settle phase's iq_cmd=0.0f
        // hold, same reasoning as run_cl_vel_chirp.
        iq_cmd = CL_CURRENT_CHIRP_AMPLITUDE * sinf(cl_current_chirp_angle_rad);

        cl_current_chirp_t += SYSID_DT;

        if (cl_current_chirp_t >= CL_CURRENT_CHIRP_DURATION)
        {
            iq_cmd         = 0.0f;
            foc_vd_applied = 0.0f;
            foc_vq_applied = 0.0f;
            pwm_apply_dq(0.0f, 0.0f, theta);
            sysid_stage = SYSID_STAGE_IDLE;
            return;
        }
    }

    foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// =============================================================================
// run_vel_chirp — CL iq chirp for mechanical plant identification
// =============================================================================

static void run_vel_chirp(void)
{
    
    const float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    if (vel_chirp_settle_tick < VEL_CHIRP_SETTLE_TICKS)
    {
        if (vel_chirp_settle_tick == 0u)
        {
            // P-only current loop for this test -- Ki=5561's integrator
            // winding up then unwinding is exactly what produced the
            // ringing (instant spike, then ~15-tick decaying oscillation)
            // seen after a real stick-slip disturbance. Same Kp as
            // everywhere else (loops.c), Ki=0 so the measured response
            // reflects the raw electrical/mechanical plant directly instead
            // of the controller's own dynamics on top of it. Scoped to just
            // this test -- current_loop/d_current_loop are shared globals,
            // loops_reset() puts the real Ki back at the next ALIGN->RUN.
            pi_init(&current_loop,   3.86f, 0.0f, -(V_BUS / 2.0f), (V_BUS / 2.0f));
            pi_init(&d_current_loop, 3.86f, 0.0f, -(V_BUS / 2.0f), (V_BUS / 2.0f));
        }

        // Settle onto the bias current itself, not zero -- so the stage is
        // already moving at the bias's steady-state kinetic speed before the
        // chirp starts, instead of stepping onto the bias at t=0 of the sweep.
        float i_cmd = 0;

        foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
        foc_vq_applied = pi_step(&current_loop, i_cmd - i_q_meas, SYSID_DT);

        pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);

        ++vel_chirp_settle_tick;
        return;
    }


    sysid_f = VEL_CHIRP_F_START * powf(VEL_CHIRP_F_END / VEL_CHIRP_F_START, sysid_t / VEL_CHIRP_DURATION);

    chirp_angle_rad += FOC_TWO_PI * sysid_f * SYSID_DT;
    if (chirp_angle_rad >= FOC_TWO_PI)
    {
        chirp_angle_rad -= FOC_TWO_PI;
    }
    iq_cmd = VEL_CHIRP_AMPLITUDE * cosf(chirp_angle_rad);
    if (iq_cmd >  VEL_CHIRP_IQ_LIMIT) iq_cmd =  VEL_CHIRP_IQ_LIMIT;
    if (iq_cmd < -VEL_CHIRP_IQ_LIMIT) iq_cmd = -VEL_CHIRP_IQ_LIMIT;

    foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);

    sysid_t += SYSID_DT;

    if (sysid_t >= VEL_CHIRP_DURATION)
    {
        iq_cmd         = 0.0f;
        foc_vd_applied = 0.0f;
        foc_vq_applied = 0.0f;
        pwm_apply_dq(0.0f, 0.0f, theta);
        loops_reset();
        sysid_stage = SYSID_STAGE_IDLE;
    }
}


// =============================================================================
// run_cl_step — Closed current loop step input (set CL_STEP_HIGH )
// =============================================================================

static void run_cl_current_step(void)
{

    float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid())
        return;

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);
    vel_meas_counts = encoder_get_velocity();  // ADD HERE

    switch (cl_step_phase)
    {
        case CL_STEP_SETTLE:
            iq_cmd = 0.0f;
            if (++cl_step_tick >= CL_STEP_SETTLE_TICKS)
            {
                cl_step_tick  = 0;
                cl_step_phase = CL_STEP_HIGH;
            }
            break;

        case CL_STEP_HIGH:
            iq_cmd = CL_STEP_AMPLITUDE;
            if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
            {
                cl_step_tick  = 0;
                cl_step_phase = CL_STEP_LOW;
            }
            break;

        case CL_STEP_LOW:
            iq_cmd = -1.0f * CL_STEP_AMPLITUDE;
            if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
            {
                cl_step_tick  = 0;
                cl_step_phase = CL_STEP_ZERO;
            }
            break;

        case CL_STEP_ZERO:
            iq_cmd = 0.0f;
            if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
            {
                cl_step_tick  = 0;
                cl_step_phase = CL_STEP_DONE;
                sysid_stage   = SYSID_STAGE_IDLE;
            }
            break;

        case CL_STEP_DONE:
        default:
            iq_cmd         = 0.0f;
            foc_vd_applied = 0.0f;
            foc_vq_applied = 0.0f;
            pwm_apply_dq(0.0f, 0.0f, theta);
            return;
    }

    foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// =============================================================================
// run_cl_vel_step — CL velocity loop step response
//
// Architecture:
//   Outer loop (velocity): runs every 4th ISR tick = 5 kHz (DT_VELOCITY)
//     vel_cmd → [vel PI] → iq_cmd  [A]
//   Inner loop (current): runs every ISR tick = 20 kHz (SYSID_DT)
//     iq_cmd  → [cur PI] → vq_applied  [V]
//
// Telemetry slot usage:
//   sysid_f_Hz slot → vel_cmd_rad_sec * 1000  [mrad/s]
//                      Python: df["sysid_f"] / 1000.0  → rad/s
//   iq_cmd_mA slot  → vel_meas_counts          [counts/s, int16]
//                      Python: df["iq_cmd_mA"] * (2*pi / 8192)  → rad/s
// =============================================================================

#define VEL_LOOP_DECIMATE  4u   // 20 kHz / 4 = 5 kHz

static uint32_t vel_loop_tick = 0;
static void run_cl_vel_step(void)
{
    float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    vel_meas_counts = encoder_get_velocity();   // counts/s, stored for telemetry

    // ── Outer velocity loop — 5 kHz ──────────────────────────────────────────
    if (++vel_loop_tick >= VEL_LOOP_DECIMATE)
    {
        vel_loop_tick = 0;

        float vel_meas_rad = vel_meas_counts * (FOC_TWO_PI / ENCODER_CPR);

        switch (cl_step_phase)
        {
            case CL_STEP_SETTLE:
                vel_cmd_rad_sec = 0.0f;
                if (++cl_step_tick >= CL_STEP_SETTLE_TICKS)
                {
                    cl_step_tick             = 0;
                    cl_step_phase            = CL_STEP_HIGH;
                    velocity_loop.integrator = 0.0f;  // clear vel integrator only
                }
                break;

            case CL_STEP_HIGH:
                vel_cmd_rad_sec = CL_STEP_VEL_AMPLITUDE;
                if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
                {
                    cl_step_tick  = 0;
                    cl_step_phase = CL_STEP_LOW;
                }
                break;

            case CL_STEP_LOW:
                vel_cmd_rad_sec = -1.0f * CL_STEP_VEL_AMPLITUDE;
                if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
                {
                    cl_step_tick  = 0;
                    cl_step_phase = CL_STEP_ZERO;
                }
                break;

            case CL_STEP_ZERO:
                vel_cmd_rad_sec = 0.0f;
                if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
                {
                    cl_step_tick  = 0;
                    cl_step_phase = CL_STEP_DONE;
                    sysid_stage   = SYSID_STAGE_IDLE;
                }
                break;

            case CL_STEP_DONE:
            default:
                vel_cmd_rad_sec = 0.0f;
                iq_cmd          = 0.0f;
                foc_vd_applied  = 0.0f;
                foc_vq_applied  = 0.0f;
                pwm_apply_dq(0.0f, 0.0f, theta);
                return;
        }

        // Velocity PI → iq_cmd [A], clamped to ±0.5A
        iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
    }

    // ── Inner current loop — 20 kHz ──────────────────────────────────────────
    foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// =============================================================================
// run_friction_sweep — closed velocity loop (production PI), series of
// constant-velocity holds from FRICTION_SWEEP_V_MIN to FRICTION_SWEEP_V_MAX,
// one full ascending sweep per direction. See config.h for the full
// rationale. loops_reset() already put the real VEL_KP/VEL_KI into
// velocity_loop at ALIGN->RUN -- no P-only override here, unlike
// CL_VEL_CHIRP's plant-ID trick, because a real integrator is required for
// steady-state i_q to equal the friction torque instead of sitting at
// whatever partial-tracking error a P-only loop would leave.
//
// Telemetry: same convention as CL_VEL_STEP/CL_VEL_CHIRP --
//   sysid_f slot -> vel_cmd_rad_sec [mrad/s]
//   last slot    -> vel_meas_counts [counts/s]
// iq_mA (native slot) is the actual measurement of interest -- per-step
// steady-state averaging is a Pi-side post-processing step (grab the tail
// of each ~1s hold), same division of labor as every other sysid test here.
// =============================================================================

typedef enum
{
    FRICTION_SETTLE_START = 0,
    FRICTION_SWEEP_POS,
    FRICTION_SETTLE_MID,
    FRICTION_SWEEP_NEG,
    FRICTION_SETTLE_END,
    FRICTION_DONE
} FrictionSweepPhase;

static FrictionSweepPhase friction_phase          = FRICTION_SETTLE_START;
static uint32_t           friction_tick           = 0u;
static float              friction_step           = FRICTION_SWEEP_V_MIN;
static int32_t            friction_sweep_start_pos = 0;

static void run_friction_sweep(void)
{
    float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    vel_meas_counts = encoder_get_velocity();

    if (++vel_loop_tick >= VEL_LOOP_DECIMATE)
    {
        vel_loop_tick = 0;

        float vel_meas_rad = vel_meas_counts * (FOC_TWO_PI / ENCODER_CPR);

        // Hard position-limit backstop, independent of the hold-time
        // arithmetic sized in config.h -- see FRICTION_SWEEP_POS_LIMIT_MM.
        // Checked every tick, not just at step boundaries, so a runaway
        // shows up immediately rather than only being caught at the next
        // scheduled transition.
        float disp_mm = (float)(encoder_get_position() - friction_sweep_start_pos) *
                         (STAGE_LEAD_MM_PER_REV / (float)ENCODER_CPR);
        bool  limit_hit = (fabsf(disp_mm) > FRICTION_SWEEP_POS_LIMIT_MM);

        switch (friction_phase)
        {
            case FRICTION_SETTLE_START:
                vel_cmd_rad_sec = 0.0f;
                if (++friction_tick >= FRICTION_SWEEP_SETTLE_TICKS)
                {
                    friction_tick            = 0u;
                    friction_step            = FRICTION_SWEEP_V_MIN;
                    friction_sweep_start_pos = encoder_get_position();
                    friction_phase           = FRICTION_SWEEP_POS;
                }
                break;

            case FRICTION_SWEEP_POS:
                vel_cmd_rad_sec = friction_step;
                if (limit_hit)
                {
                    friction_tick  = 0u;
                    friction_phase = FRICTION_SETTLE_MID;
                }
                else if (++friction_tick >= FRICTION_SWEEP_HOLD_TICKS)
                {
                    friction_tick = 0u;
                    friction_step += FRICTION_SWEEP_V_STEP;
                    if (friction_step > FRICTION_SWEEP_V_MAX + 0.5f * FRICTION_SWEEP_V_STEP)
                    {
                        friction_phase = FRICTION_SETTLE_MID;
                    }
                }
                break;

            case FRICTION_SETTLE_MID:
                vel_cmd_rad_sec = 0.0f;
                if (++friction_tick >= FRICTION_SWEEP_SETTLE_TICKS)
                {
                    friction_tick            = 0u;
                    friction_step            = FRICTION_SWEEP_V_MIN;
                    friction_sweep_start_pos = encoder_get_position();
                    friction_phase           = FRICTION_SWEEP_NEG;
                }
                break;

            case FRICTION_SWEEP_NEG:
                vel_cmd_rad_sec = -friction_step;
                if (limit_hit)
                {
                    friction_tick  = 0u;
                    friction_phase = FRICTION_SETTLE_END;
                }
                else if (++friction_tick >= FRICTION_SWEEP_HOLD_TICKS)
                {
                    friction_tick = 0u;
                    friction_step += FRICTION_SWEEP_V_STEP;
                    if (friction_step > FRICTION_SWEEP_V_MAX + 0.5f * FRICTION_SWEEP_V_STEP)
                    {
                        friction_phase = FRICTION_SETTLE_END;
                    }
                }
                break;

            case FRICTION_SETTLE_END:
                vel_cmd_rad_sec = 0.0f;
                if (++friction_tick >= FRICTION_SWEEP_SETTLE_TICKS)
                {
                    friction_tick  = 0u;
                    friction_phase = FRICTION_DONE;
                    sysid_stage    = SYSID_STAGE_IDLE;
                }
                break;

            case FRICTION_DONE:
            default:
                vel_cmd_rad_sec = 0.0f;
                iq_cmd          = 0.0f;
                foc_vd_applied  = 0.0f;
                foc_vq_applied  = 0.0f;
                pwm_apply_dq(0.0f, 0.0f, theta);
                return;
        }

        iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
    }

    foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// =============================================================================
// run_breakaway — static friction / breakaway current, both directions.
// See config.h for the full safety rationale. Slow open current-loop ramp
// from rest; the instant real motion is confirmed (debounced encoder
// velocity threshold), immediately hands off to the closed velocity loop
// at a low target instead of continuing the ramp -- that handoff is what
// keeps this from repeating the earlier ~10.5A ring-out, since the closed
// loop settles toward the known ~94mA kinetic current instead of
// continuing to apply whatever torque the ramp had built up.
//
// Telemetry:
//   sysid_f slot -> commanded iq [mA] (ramp value, or the handoff PI's
//                   output once breakaway is detected)
//   last slot    -> measured velocity [counts/s] -- find the tick velocity
//                   first crosses the threshold, read sysid_f at that same
//                   row for the breakaway current
//   enc_hi slot  -> breakaway_phase (see BreakawayPhase below)
//   enc_lo slot  -> raw encoder position low word (unchanged default use)
// =============================================================================

typedef enum
{
    BREAKAWAY_SETTLE_START = 0,
    BREAKAWAY_RAMP_POS,
    BREAKAWAY_HANDOFF_POS,
    BREAKAWAY_SETTLE_MID,
    BREAKAWAY_RAMP_NEG,
    BREAKAWAY_HANDOFF_NEG,
    BREAKAWAY_SETTLE_END,
    BREAKAWAY_DONE
} BreakawayPhase;

static BreakawayPhase breakaway_phase         = BREAKAWAY_SETTLE_START;
static uint32_t        breakaway_tick         = 0u;
static float            breakaway_iq_ramp     = 0.0f;
static uint32_t        breakaway_detect_count = 0u;
static int32_t          breakaway_start_pos   = 0;

static void run_breakaway(void)
{
    float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    vel_meas_counts = encoder_get_velocity();

    if (++vel_loop_tick >= VEL_LOOP_DECIMATE)
    {
        vel_loop_tick = 0;

        float vel_meas_rad = vel_meas_counts * (FOC_TWO_PI / ENCODER_CPR);
        float disp_mm = (float)(encoder_get_position() - breakaway_start_pos) *
                         (STAGE_LEAD_MM_PER_REV / (float)ENCODER_CPR);
        bool  limit_hit = (fabsf(disp_mm) > BREAKAWAY_POS_LIMIT_MM);

        // 5000.0f = 5kHz decimated rate (1/DT_VELOCITY) -- ramp reaches
        // BREAKAWAY_IQ_CEILING after BREAKAWAY_RAMP_DURATION seconds if no
        // breakaway is detected first.
        float ramp_step = BREAKAWAY_IQ_CEILING / (BREAKAWAY_RAMP_DURATION * 5000.0f);

        switch (breakaway_phase)
        {
            case BREAKAWAY_SETTLE_START:
                vel_cmd_rad_sec = 0.0f;
                iq_cmd          = 0.0f;
                if (++breakaway_tick >= FRICTION_SWEEP_SETTLE_TICKS)
                {
                    breakaway_tick         = 0u;
                    breakaway_iq_ramp      = 0.0f;
                    breakaway_detect_count = 0u;
                    breakaway_start_pos    = encoder_get_position();
                    breakaway_phase        = BREAKAWAY_RAMP_POS;
                }
                break;

            case BREAKAWAY_RAMP_POS:
                breakaway_iq_ramp += ramp_step;
                if (breakaway_iq_ramp > BREAKAWAY_IQ_CEILING) breakaway_iq_ramp = BREAKAWAY_IQ_CEILING;
                iq_cmd = breakaway_iq_ramp;

                if (fabsf(vel_meas_rad) > BREAKAWAY_DETECT_THRESH_RAD_S)
                {
                    ++breakaway_detect_count;
                }
                else
                {
                    breakaway_detect_count = 0u;
                }

                if (limit_hit)
                {
                    breakaway_tick  = 0u;
                    breakaway_phase = BREAKAWAY_SETTLE_MID;
                }
                else if (breakaway_detect_count >= BREAKAWAY_DETECT_TICKS)
                {
                    // Breakaway confirmed -- hand off to the closed velocity
                    // loop immediately, don't keep ramping.
                    velocity_loop.integrator = 0.0f;
                    breakaway_tick  = 0u;
                    breakaway_phase = BREAKAWAY_HANDOFF_POS;
                }
                else if (breakaway_iq_ramp >= BREAKAWAY_IQ_CEILING)
                {
                    // Ceiling reached, no breakaway seen -- give up, don't
                    // hold max current indefinitely.
                    breakaway_tick  = 0u;
                    breakaway_phase = BREAKAWAY_SETTLE_MID;
                }
                break;

            case BREAKAWAY_HANDOFF_POS:
                vel_cmd_rad_sec = BREAKAWAY_HANDOFF_VEL;
                iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
                if (limit_hit || ++breakaway_tick >= BREAKAWAY_HANDOFF_TICKS)
                {
                    breakaway_tick  = 0u;
                    breakaway_phase = BREAKAWAY_SETTLE_MID;
                }
                break;

            case BREAKAWAY_SETTLE_MID:
                vel_cmd_rad_sec = 0.0f;
                iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
                if (++breakaway_tick >= FRICTION_SWEEP_SETTLE_TICKS)
                {
                    breakaway_tick         = 0u;
                    breakaway_iq_ramp      = 0.0f;
                    breakaway_detect_count = 0u;
                    breakaway_start_pos    = encoder_get_position();
                    breakaway_phase        = BREAKAWAY_RAMP_NEG;
                }
                break;

            case BREAKAWAY_RAMP_NEG:
                breakaway_iq_ramp += ramp_step;
                if (breakaway_iq_ramp > BREAKAWAY_IQ_CEILING) breakaway_iq_ramp = BREAKAWAY_IQ_CEILING;
                iq_cmd = -breakaway_iq_ramp;

                if (fabsf(vel_meas_rad) > BREAKAWAY_DETECT_THRESH_RAD_S)
                {
                    ++breakaway_detect_count;
                }
                else
                {
                    breakaway_detect_count = 0u;
                }

                if (limit_hit)
                {
                    breakaway_tick  = 0u;
                    breakaway_phase = BREAKAWAY_SETTLE_END;
                }
                else if (breakaway_detect_count >= BREAKAWAY_DETECT_TICKS)
                {
                    velocity_loop.integrator = 0.0f;
                    breakaway_tick  = 0u;
                    breakaway_phase = BREAKAWAY_HANDOFF_NEG;
                }
                else if (breakaway_iq_ramp >= BREAKAWAY_IQ_CEILING)
                {
                    breakaway_tick  = 0u;
                    breakaway_phase = BREAKAWAY_SETTLE_END;
                }
                break;

            case BREAKAWAY_HANDOFF_NEG:
                vel_cmd_rad_sec = -BREAKAWAY_HANDOFF_VEL;
                iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
                if (limit_hit || ++breakaway_tick >= BREAKAWAY_HANDOFF_TICKS)
                {
                    breakaway_tick  = 0u;
                    breakaway_phase = BREAKAWAY_SETTLE_END;
                }
                break;

            case BREAKAWAY_SETTLE_END:
                vel_cmd_rad_sec = 0.0f;
                iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
                if (++breakaway_tick >= FRICTION_SWEEP_SETTLE_TICKS)
                {
                    breakaway_tick  = 0u;
                    breakaway_phase = BREAKAWAY_DONE;
                    sysid_stage     = SYSID_STAGE_IDLE;
                }
                break;

            case BREAKAWAY_DONE:
            default:
                vel_cmd_rad_sec = 0.0f;
                iq_cmd          = 0.0f;
                foc_vd_applied  = 0.0f;
                foc_vq_applied  = 0.0f;
                pwm_apply_dq(0.0f, 0.0f, theta);
                return;
        }
    }

    foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// ============================================= ================================
// run_cl_vel_chirp — CLOSED velocity loop chirp: sweeps vel_cmd (not iq)
// with the velocity PI closed around it, to identify the closed-loop
// vel_cmd -> vel_meas response for designing the outer position P
// controller. Same cascade as run_cl_vel_step(), chirp instead of a step.
//
// Telemetry slot usage — same convention as CL_VEL_STEP:
//   sysid_f_Hz slot → vel_cmd_rad_sec * 1000  [mrad/s]
//   iq_cmd_mA slot  → vel_meas_counts          [counts/s, int16]
// =============================================================================

static float cl_vel_chirp_t         = 0.0f;
static float cl_vel_chirp_angle_rad = 0.0f;

static void run_cl_vel_chirp(void)
{
    float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    vel_meas_counts = encoder_get_velocity();

    if (++vel_loop_tick >= VEL_LOOP_DECIMATE)
    {
        vel_loop_tick = 0;

        float vel_meas_rad = vel_meas_counts * (FOC_TWO_PI / ENCODER_CPR);

        if (vel_chirp_settle_tick < CL_VEL_CHIRP_SETTLE_TICKS)
        {
            if (vel_chirp_settle_tick == 0u)
            {
                // P-only velocity loop for this identification run -- see
                // CL_VEL_CHIRP_PONLY_KP in config.h for why. velocity_loop is
                // a shared global (loops.c); loops_reset() puts the real
                // VEL_KP/VEL_KI back at the next ALIGN->RUN transition.
#if CL_VEL_CHIRP_DEPLOYED
                /* deployed VEL_KP/VEL_KI already installed by loops_reset() */
#else
                pi_init(&velocity_loop, CL_VEL_CHIRP_PONLY_KP, 0.0f, -0.5f, 0.5f);
#endif
            }

            vel_cmd_rad_sec = 0.0f;
            ++vel_chirp_settle_tick;
        }
        else
        {
            sysid_f = CL_VEL_CHIRP_F_START *
                      powf(CL_VEL_CHIRP_F_END / CL_VEL_CHIRP_F_START,
                           cl_vel_chirp_t / CL_VEL_CHIRP_DURATION);

            cl_vel_chirp_angle_rad += FOC_TWO_PI * sysid_f * DT_VELOCITY;
            if (cl_vel_chirp_angle_rad >= FOC_TWO_PI)
                cl_vel_chirp_angle_rad -= FOC_TWO_PI;

            // sinf, not cosf -- starts at 0 so this is continuous with the
            // settle phase's vel_cmd_rad_sec=0.0f hold. cosf(0)=1 would step
            // the full CL_VEL_CHIRP_AMPLITUDE instantly the moment settle
            // ends, with current already closed-loop and reacting live.
            vel_cmd_rad_sec = CL_VEL_CHIRP_AMPLITUDE * sinf(cl_vel_chirp_angle_rad);

            cl_vel_chirp_t += DT_VELOCITY;

            if (cl_vel_chirp_t >= CL_VEL_CHIRP_DURATION)
            {
                vel_cmd_rad_sec = 0.0f;
                iq_cmd          = 0.0f;
                foc_vd_applied  = 0.0f;
                foc_vq_applied  = 0.0f;
                pwm_apply_dq(0.0f, 0.0f, theta);
                loops_reset();
                sysid_stage = SYSID_STAGE_IDLE;
                return;
            }
        }

        // pi_step directly, NOT velocity_loop_step: the friction feedforward
        // in loops.c adds Coulomb + viscous terms driven by vel_cmd, which is
        // a feedforward path from the reference straight to the plant input.
        // That makes C(s) != VEL_KP_TEST and breaks this test's whole point --
        // the plant back-out P(s) = L(s)/VEL_KP_TEST assumes the controller is
        // that constant. Measured 2026-09-19 with the feedforward left in:
        // closed-loop DC gain +1.47dB (impossible for P-only, since H(0) =
        // KpK/(1+KpK) < 1), vel_meas/vel_cmd a flat 1.18 at every frequency,
        // and a backed-out K of -229 rad/s per A -- the sign flip is just
        // 1-H going negative. At 0.6Hz the measured iq_cmd peak was 0.105A
        // against a 0.094A Coulomb term, i.e. the feedforward was supplying
        // essentially all of the current and the P term almost none.
#if CL_VEL_CHIRP_DEPLOYED
  #if CL_VEL_CHIRP_FF
        iq_cmd = velocity_loop_step(vel_cmd_rad_sec, vel_meas_rad, DT_VELOCITY);
  #else
        /* deployed VEL_KP/VEL_KI, friction feedforward off -- what the
         * position loop actually runs on. See CL_VEL_CHIRP_FF. */
        iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
  #endif
#else
        iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
#endif
    }

    foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// =============================================================================
// run_position_step — position P loop step response, wrapped around the
// already-closed velocity + current loops (position_loop.kp = POSITION_LOOP_KP,
// designed from the closed-velocity-loop chirp -- see foc-sysid
// closed_vel_bode_plot.py).
//
// Cascade:
//   Position loop (1 kHz, DT_POSITION):  pos_cmd -> [P]      -> vel_cmd
//   Velocity loop (5 kHz, DT_VELOCITY):  vel_cmd -> [vel PI] -> iq_cmd
//   Current loop  (20 kHz, SYSID_DT):    iq_cmd  -> [cur PI] -> vq_applied
//
// Reuses cl_step_phase/cl_step_tick (same SETTLE/HIGH/LOW/ZERO/DONE sequence
// as the other step tests) with pos_cmd_rad in place of iq/vel command.
//
// Telemetry slot usage:
//   sysid_f_Hz slot -> pos_cmd_rad  * 1000  [mrad]
//   iq_cmd_mA slot  -> pos_meas_rad * 1000  [mrad]
// =============================================================================

#define POS_LOOP_DECIMATE  20u   // 20 kHz / 20 = 1 kHz = DT_POSITION

static uint32_t pos_loop_tick   = 0;
static float    pos_cmd_rad     = 0.0f;
static float    pos_meas_rad    = 0.0f;

static void run_position_step(void)
{
    float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    vel_meas_counts = encoder_get_velocity();

    // ── Position loop — 1 kHz ────────────────────────────────────────────────
    if (++pos_loop_tick >= POS_LOOP_DECIMATE)
    {
        pos_loop_tick = 0;

        int32_t raw = encoder_get_position() - sysid_enc_offset;
        pos_meas_rad = (float)raw * (FOC_TWO_PI / ENCODER_CPR);

        switch (cl_step_phase)
        {
            case CL_STEP_SETTLE:
                pos_cmd_rad = 0.0f;
                if (++cl_step_tick >= POSITION_STEP_SETTLE_TICKS)
                {
                    cl_step_tick  = 0;
                    cl_step_phase = CL_STEP_HIGH;
                }
                break;

            case CL_STEP_HIGH:
                pos_cmd_rad = POSITION_STEP_AMPLITUDE_RAD;
                if (++cl_step_tick >= POSITION_STEP_HOLD_TICKS)
                {
                    cl_step_tick  = 0;
                    cl_step_phase = CL_STEP_LOW;
                }
                break;

            case CL_STEP_LOW:
                pos_cmd_rad = -POSITION_STEP_AMPLITUDE_RAD;
                if (++cl_step_tick >= POSITION_STEP_HOLD_TICKS)
                {
                    cl_step_tick  = 0;
                    cl_step_phase = CL_STEP_ZERO;
                }
                break;

            case CL_STEP_ZERO:
                pos_cmd_rad = 0.0f;
                if (++cl_step_tick >= POSITION_STEP_HOLD_TICKS)
                {
                    cl_step_tick  = 0;
                    cl_step_phase = CL_STEP_DONE;
                    sysid_stage   = SYSID_STAGE_IDLE;
                }
                break;

            case CL_STEP_DONE:
            default:
                vel_cmd_rad_sec = 0.0f;
                iq_cmd          = 0.0f;
                foc_vd_applied  = 0.0f;
                foc_vq_applied  = 0.0f;
                pwm_apply_dq(0.0f, 0.0f, theta);
                return;
        }

        // P controller has no built-in output clamp (unlike PIState) -- clamp
        // the velocity demand manually so a step doesn't ask for more speed
        // than we've actually validated.
        vel_cmd_rad_sec = p_step(&position_loop, pos_cmd_rad - pos_meas_rad);
        if (vel_cmd_rad_sec >  POSITION_VEL_LIMIT) vel_cmd_rad_sec =  POSITION_VEL_LIMIT;
        if (vel_cmd_rad_sec < -POSITION_VEL_LIMIT) vel_cmd_rad_sec = -POSITION_VEL_LIMIT;
    }

    // ── Velocity loop — 5 kHz ────────────────────────────────────────────────
    if (++vel_loop_tick >= VEL_LOOP_DECIMATE)
    {
        vel_loop_tick = 0;
        float vel_meas_rad = vel_meas_counts * (FOC_TWO_PI / ENCODER_CPR);
        // velocity_loop_step(), not a raw pi_step() -- same reason as
        // run_cl_pos_chirp(): POSITION_LOOP_KP was sized against the
        // feedforward-compensated system.
        iq_cmd = velocity_loop_step(vel_cmd_rad_sec, vel_meas_rad, DT_VELOCITY);
    }

    // ── Current loop — 20 kHz ────────────────────────────────────────────────
    foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// =============================================================================
// run_cl_pos_chirp — CLOSED position loop chirp: sweeps pos_cmd with the
// full P -> PI -> PI cascade closed, to directly MEASURE the actual
// closed-loop response of the whole system (pos_meas/pos_cmd) instead of
// relying on the H(s)/s construction used to design POSITION_LOOP_KP.
//
// Cascade: identical to run_position_step() -- position P (1kHz) ->
// velocity PI (5kHz) -> current PI (20kHz). Reuses pos_loop_tick,
// pos_cmd_rad, pos_meas_rad from run_position_step() above (mutually
// exclusive via SYSID_TEST dispatch, so no conflict).
//
// Telemetry: shares the SYSID_TEST_POSITION_STEP slot convention
//   sysid_f_Hz slot -> pos_cmd_rad  * 1000  [mrad]
//   iq_cmd_mA slot  -> pos_meas_rad * 1000  [mrad]
// plus vel_cmd/vel_meas repurposed into the enc_hi/enc_lo slots -- see
// foc_sysid_step() telemetry section below.
// =============================================================================

static uint32_t pos_chirp_settle_tick  = 0;
static float    cl_pos_chirp_t         = 0.0f;
static float    cl_pos_chirp_angle_rad = 0.0f;

static void run_cl_pos_chirp(void)
{
    float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    vel_meas_counts = encoder_get_velocity();

    // ── Position loop — 1 kHz ────────────────────────────────────────────────
    if (++pos_loop_tick >= POS_LOOP_DECIMATE)
    {
        pos_loop_tick = 0;

        int32_t raw = encoder_get_position() - sysid_enc_offset;
        pos_meas_rad = (float)raw * (FOC_TWO_PI / ENCODER_CPR);

        if (pos_chirp_settle_tick < POSITION_STEP_SETTLE_TICKS)
        {
            pos_cmd_rad = 0.0f;
            ++pos_chirp_settle_tick;
        }
        else
        {
            sysid_f = CL_POS_CHIRP_F_START *
                      powf(CL_POS_CHIRP_F_END / CL_POS_CHIRP_F_START,
                           cl_pos_chirp_t / CL_POS_CHIRP_DURATION);

            cl_pos_chirp_angle_rad += FOC_TWO_PI * sysid_f * DT_POSITION;
            if (cl_pos_chirp_angle_rad >= FOC_TWO_PI)
                cl_pos_chirp_angle_rad -= FOC_TWO_PI;

            pos_cmd_rad = CL_POS_CHIRP_AMPLITUDE * cosf(cl_pos_chirp_angle_rad);

            cl_pos_chirp_t += DT_POSITION;

            if (cl_pos_chirp_t >= CL_POS_CHIRP_DURATION)
            {
                pos_cmd_rad     = 0.0f;
                vel_cmd_rad_sec = 0.0f;
                iq_cmd          = 0.0f;
                foc_vd_applied  = 0.0f;
                foc_vq_applied  = 0.0f;
                pwm_apply_dq(0.0f, 0.0f, theta);
                loops_reset();
                sysid_stage = SYSID_STAGE_IDLE;
                return;
            }
        }

        vel_cmd_rad_sec = p_step(&position_loop, pos_cmd_rad - pos_meas_rad);
        if (vel_cmd_rad_sec >  POSITION_VEL_LIMIT) vel_cmd_rad_sec =  POSITION_VEL_LIMIT;
        if (vel_cmd_rad_sec < -POSITION_VEL_LIMIT) vel_cmd_rad_sec = -POSITION_VEL_LIMIT;
    }

    // ── Velocity loop — 5 kHz ────────────────────────────────────────────────
    if (++vel_loop_tick >= VEL_LOOP_DECIMATE)
    {
        vel_loop_tick = 0;
        float vel_meas_rad = vel_meas_counts * (FOC_TWO_PI / ENCODER_CPR);
        // velocity_loop_step(), not a raw pi_step() -- POSITION_LOOP_KP was
        // sized against the feedforward-compensated H(s), so verifying it
        // needs the same feedforward active underneath, not the bare PI.
#if CL_POS_CHIRP_DEPLOYED
        iq_cmd = velocity_loop_step(vel_cmd_rad_sec, vel_meas_rad, DT_VELOCITY);
#else
        /* friction feedforward bypassed -- see CL_POS_CHIRP_DEPLOYED */
        iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
#endif
    }

    // ── Current loop — 20 kHz ────────────────────────────────────────────────
    foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// =============================================================================
// run_cine_sweep — same closed position loop chirp as run_cl_pos_chirp(),
// sized for FILMING instead of analysis (see CINE_SWEEP_* in config.h: a
// slow 0.1-3Hz sweep that's actually visible as motion, not a measurement
// sweep). Structurally identical cascade, own state/parameters so it can be
// captured independently of the real analysis chirp.
//
// Telemetry: shares the SYSID_TEST_POSITION_STEP slot convention, same as
// run_cl_pos_chirp() -- see foc_sysid_step() telemetry section below.
// =============================================================================

static uint32_t cine_settle_tick   = 0;
static float    cine_sweep_t       = 0.0f;
static float    cine_sweep_angle   = 0.0f;

static void run_cine_sweep(void)
{
    float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    vel_meas_counts = encoder_get_velocity();

    // ── Position loop — 1 kHz ────────────────────────────────────────────────
    if (++pos_loop_tick >= POS_LOOP_DECIMATE)
    {
        pos_loop_tick = 0;

        int32_t raw = encoder_get_position() - sysid_enc_offset;
        pos_meas_rad = (float)raw * (FOC_TWO_PI / ENCODER_CPR);

        if (cine_settle_tick < POSITION_STEP_SETTLE_TICKS)
        {
            pos_cmd_rad = pos_meas_rad;
            ++cine_settle_tick;
        }
        else
        {
            sysid_f = CINE_SWEEP_F_START *
                      powf(CINE_SWEEP_F_END / CINE_SWEEP_F_START,
                           cine_sweep_t / CINE_SWEEP_DURATION);

            cine_sweep_angle += FOC_TWO_PI * sysid_f * DT_POSITION;
            if (cine_sweep_angle >= FOC_TWO_PI)
                cine_sweep_angle -= FOC_TWO_PI;

            pos_cmd_rad = CINE_SWEEP_AMPLITUDE * cosf(cine_sweep_angle);

            cine_sweep_t += DT_POSITION;

            if (cine_sweep_t >= CINE_SWEEP_DURATION)
            {
                pos_cmd_rad     = 0.0f;
                vel_cmd_rad_sec = 0.0f;
                iq_cmd          = 0.0f;
                foc_vd_applied  = 0.0f;
                foc_vq_applied  = 0.0f;
                pwm_apply_dq(0.0f, 0.0f, theta);
                loops_reset();
                sysid_stage = SYSID_STAGE_IDLE;
                return;
            }
        }

        vel_cmd_rad_sec = p_step(&position_loop, pos_cmd_rad - pos_meas_rad);
        if (vel_cmd_rad_sec >  POSITION_VEL_LIMIT) vel_cmd_rad_sec =  POSITION_VEL_LIMIT;
        if (vel_cmd_rad_sec < -POSITION_VEL_LIMIT) vel_cmd_rad_sec = -POSITION_VEL_LIMIT;
    }

    // ── Velocity loop — 5 kHz ────────────────────────────────────────────────
    if (++vel_loop_tick >= VEL_LOOP_DECIMATE)
    {
        vel_loop_tick = 0;
        float vel_meas_rad = vel_meas_counts * (FOC_TWO_PI / ENCODER_CPR);
        iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
    }

    // ── Current loop — 20 kHz ────────────────────────────────────────────────
    foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// =============================================================================
// run_phase_check — per-phase DC excitation, isolates a bad phase connection
// (motor/driver not energizing that phase) from a bad current-sense channel
// (phase is energized fine, but that ADC channel doesn't show it). Drives
// each phase to +PHASE_CHECK_V_MAIN in turn (other two split the balanced
// return at PHASE_CHECK_V_RETURN each -- same magnitude run_voltage_test
// already proved safe held for 30s), holding long enough to read a settled
// DC value on all three sense channels every tick.
//
// Expected healthy result per step: the driven phase reads ~+I, the other
// two read ~-I/2 each, and ia+ib+ic ~= 0 throughout (KCL, same check
// SYSID_TEST_CURRENT_LOOP_STEP already logs). A phase that isn't actually
// energized (bad connector/ferrule) reads near-zero or noisy/non-settling
// current on ALL three channels during its own excitation step (broken
// loop, no current path at all). A dead/miswired sense channel instead
// shows the OTHER two channels responding correctly (proving current IS
// flowing) while that one channel stays flat/wrong-signed -- energization
// is fine, only the feedback path is bad.
// =============================================================================

typedef enum
{
    PHASE_CHECK_A = 0,
    PHASE_CHECK_ZERO_A,
    PHASE_CHECK_B,
    PHASE_CHECK_ZERO_B,
    PHASE_CHECK_C,
    PHASE_CHECK_ZERO_C,
    PHASE_CHECK_DONE
} PhaseCheckStage;

static PhaseCheckStage phase_check_stage = PHASE_CHECK_A;
static uint32_t        phase_check_tick  = 0u;

static void run_phase_check(void)
{
    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);

    switch (phase_check_stage)
    {
        case PHASE_CHECK_A:
            pwm_apply_phase_volts(PHASE_CHECK_V_MAIN, PHASE_CHECK_V_RETURN, PHASE_CHECK_V_RETURN);
            if (++phase_check_tick >= PHASE_CHECK_HOLD_TICKS)
            { phase_check_tick = 0; phase_check_stage = PHASE_CHECK_ZERO_A; }
            break;

        case PHASE_CHECK_ZERO_A:
            pwm_apply_phase_volts(0.0f, 0.0f, 0.0f);
            if (++phase_check_tick >= PHASE_CHECK_ZERO_TICKS)
            { phase_check_tick = 0; phase_check_stage = PHASE_CHECK_B; }
            break;

        case PHASE_CHECK_B:
            pwm_apply_phase_volts(PHASE_CHECK_V_RETURN, PHASE_CHECK_V_MAIN, PHASE_CHECK_V_RETURN);
            if (++phase_check_tick >= PHASE_CHECK_HOLD_TICKS)
            { phase_check_tick = 0; phase_check_stage = PHASE_CHECK_ZERO_B; }
            break;

        case PHASE_CHECK_ZERO_B:
            pwm_apply_phase_volts(0.0f, 0.0f, 0.0f);
            if (++phase_check_tick >= PHASE_CHECK_ZERO_TICKS)
            { phase_check_tick = 0; phase_check_stage = PHASE_CHECK_C; }
            break;

        case PHASE_CHECK_C:
            pwm_apply_phase_volts(PHASE_CHECK_V_RETURN, PHASE_CHECK_V_RETURN, PHASE_CHECK_V_MAIN);
            if (++phase_check_tick >= PHASE_CHECK_HOLD_TICKS)
            { phase_check_tick = 0; phase_check_stage = PHASE_CHECK_ZERO_C; }
            break;

        case PHASE_CHECK_ZERO_C:
            pwm_apply_phase_volts(0.0f, 0.0f, 0.0f);
            if (++phase_check_tick >= PHASE_CHECK_ZERO_TICKS)
            { phase_check_tick = 0; phase_check_stage = PHASE_CHECK_DONE; }
            break;

        case PHASE_CHECK_DONE:
        default:
            pwm_apply_phase_volts(0.0f, 0.0f, 0.0f);
            sysid_stage = SYSID_STAGE_IDLE;
            break;
    }
}

// =============================================================================
// run_shunt_noise — holds a static zero-vector command (PWM switching, zero
// commanded current, no rotor motion) for a long window so every phase
// current sample is pure noise floor: shunt/amp thermal+quantization noise
// plus whatever switching noise leaks through the ADC's sample timing. No
// current loop, no alignment kick beyond the normal ALIGN stage -- rotor
// alignment doesn't matter here since nothing is being commutated.
// =============================================================================
#define SHUNT_NOISE_TICKS   100000u   /* 5s @ 20kHz -- long enough for a clean std/PSD estimate */

static uint32_t shunt_noise_tick = 0u;

static void run_shunt_noise(void)
{
    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);

    if (shunt_noise_tick == 0u)
    {
        /* One-time transition, not per-tick -- see SHUNT_NOISE_PWM_ENABLE
         * in config.h. pwm_disable() only clears MOE (bridge outputs static);
         * TIM1 keeps running the same ADC injected-trigger cadence either
         * way, so this isolates switching-induced content from intrinsic
         * shunt/amp/ADC noise without changing the sample timing. */
    #if SHUNT_NOISE_PWM_ENABLE
        pwm_enable();
    #else
        pwm_disable();
    #endif
    }

    foc_vd_applied = 0.0f;
    foc_vq_applied = 0.0f;
    pwm_apply_dq(0.0f, 0.0f, 0.0f);

    if (++shunt_noise_tick >= SHUNT_NOISE_TICKS)
    {
        shunt_noise_tick = 0u;
        pwm_enable();          /* restore normal state before idling */
        sysid_stage       = SYSID_STAGE_IDLE;
    }
}


// =============================================================================
// foc_sysid_step — called from TIM1_UP_TIM10_IRQHandler at 20 kHz
// =============================================================================
#define SYSID_ALIGN_REPEATS   10u
#define SYSID_ALIGN_TICKS     40000u   /* same hold time you already use */
#define SYSID_PERTURB_TICKS   500u    /* kick duration between repeats */
#define SYSID_PERTURB_VQ      0.3f    /* torque-axis kick, not d-axis */

static uint32_t sysid_align_repeat = 0u;
static uint32_t sysid_perturb_tick = 0u;
static uint32_t sysid_idle_tick = 0u;
static int32_t  sysid_align_log[SYSID_ALIGN_REPEATS];   /* <-- set breakpoint, read this array */

void foc_sysid_step(void)
{
    switch (sysid_stage)
    {
        case SYSID_STAGE_ALIGN:
        {
            if (sysid_align_tick < 20000u)
            //if (1)
            {
                foc_vd_applied = V_ALIGN;
                foc_vq_applied = 0.0f;
                pwm_apply_dq(foc_vd_applied, foc_vq_applied, 0.0f);
                ++sysid_align_tick;
            }
            else
            {
                // Motor off -- let the real current that built up during the
                // align hold decay to actual zero through the winding's own
                // R/L before ever closing a loop around it, instead of
                // patching around a large real current the fresh controller
                // has never seen. tau = L/R ~= 1.25mH/1.59ohm ~= 0.79ms;
                // 2000 ticks (100ms) is well over 100 time constants.
                foc_vd_applied = 0.0f;
                foc_vq_applied = 0.0f;
                pwm_apply_dq(0.0f, 0.0f, 0.0f);

                if (++sysid_discharge_tick >= 2000u)
                {
                    sysid_enc_offset = encoder_get_position();
                    loops_reset();
                    sysid_stage      = SYSID_STAGE_RUN;
                }
            }
        }
        break;





        case SYSID_STAGE_RUN:
        {
            /* ADC stall watchdog -- current_feedback_sample_count() only
             * advances when ADC_IRQHandler actually services a JEOC event.
             * Telemetry showed this freezing for seconds at a time during
             * CL_VEL_CHIRP instability while the rest of this ISR (theta,
             * encoder, current loop) kept running fine. Trap right at the
             * point it goes stale so a live debugger halts with the fault
             * still present, instead of the freeze being over by the time
             * a human can react to it. Only meaningful with a debugger
             * attached -- an unhandled BKPT with no debugger will hard-fault.
             */
            #define ADC_STALL_WATCH_TICKS  100u   /* ~5ms @ 20kHz -- well past healthy ADC cadence */
            static uint32_t adc_stall_prev_count = 0xFFFFFFFFu;
            static uint32_t adc_stall_tick       = 0u;

            uint32_t adc_count_now = current_feedback_sample_count();
            if (adc_count_now == adc_stall_prev_count)
            {
                if (++adc_stall_tick >= ADC_STALL_WATCH_TICKS)
                {
                    //__BKPT(0);            /* <-- ADC_IRQn has stalled; halt here */
                    adc_stall_tick = 0u;  /* in case execution is resumed manually */
                }
            }
            else
            {
                adc_stall_tick = 0u;
            }
            adc_stall_prev_count = adc_count_now;

        #if   SYSID_TEST == SYSID_TEST_CURRENT_LOOP_CHIRP
            run_current_lp_chirp();
        #elif SYSID_TEST == SYSID_TEST_CURRENT_LOOP_STEP
            run_cl_current_step();
        #elif SYSID_TEST == SYSID_TEST_CL_CURRENT_CHIRP
            run_cl_current_chirp();
        #elif SYSID_TEST == SYSID_TEST_VEL_CHIRP
            run_vel_chirp();
        #elif SYSID_TEST == SYSID_TEST_CL_VEL_STEP
            run_cl_vel_step();
        #elif SYSID_TEST == RIPPLE_DEBUG
            run_cl_vel_step();
        #elif SYSID_TEST == SYSID_TEST_CL_VEL_CHIRP
            run_cl_vel_chirp();
        #elif SYSID_TEST == SYSID_TEST_POSITION_STEP
            run_position_step();
        #elif SYSID_TEST == SYSID_TEST_CL_POS_CHIRP
            run_cl_pos_chirp();
        #elif SYSID_TEST == SYSID_TEST_CINE_SWEEP
            run_cine_sweep();
        #elif SYSID_TEST == SYSID_TEST_PHASE_CHECK
            run_phase_check();
        #elif SYSID_TEST == SYSID_TEST_SHUNT_NOISE
            run_shunt_noise();
        #elif SYSID_TEST == SYSID_TEST_FRICTION_SWEEP
            run_friction_sweep();
        #elif SYSID_TEST == SYSID_TEST_BREAKAWAY
            run_breakaway();
        #endif
        }
        break;

        case SYSID_STAGE_IDLE:
        {
            foc_vd_applied = 0.0f;
            foc_vq_applied = 0.0f;
            pwm_apply_dq(0.0f, 0.0f, 0.0f);

            if (++sysid_idle_tick >= 80000)
            {
                sysid_align_tick = 0u;
                sysid_idle_tick = 0;

            }


        }
        break;
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    uint16_t flags = (uint16_t)sysid_stage;

#if SYSID_TEST == SYSID_TEST_CURRENT_LOOP_CHIRP
    float theta_telem = 0.0f;
#else
    float theta_telem = foc_theta_from_encoder();
#endif

    int32_t enc_pos = encoder_get_position();

#if SYSID_TEST == SYSID_TEST_CL_VEL_STEP

    /* Closed-loop velocity step:
     *   sysid_f slot -> velocity command [mrad/s]
     *   last slot    -> measured velocity [counts/s / VEL_TELEM_DIV]
     */
    int16_t sysid_f_slot = (int16_t)(vel_cmd_rad_sec * 1000.0f);
    int16_t last_slot    = (int16_t)(vel_meas_counts / VEL_TELEM_DIV);

#elif SYSID_TEST == SYSID_TEST_CL_VEL_CHIRP

    /* Closed-loop velocity chirp:
     *   sysid_f slot -> velocity command [mrad/s]
     *   last slot    -> measured velocity [counts/s / VEL_TELEM_DIV]
     *
     * sysid_f briefly carried raw ADC1->SR instead, while root-causing the
     * SPI DMA freeze (see spi.c EXTI15_10_IRQHandler) -- that's fixed now,
     * back to vel_cmd for real Bode/plant-ID captures.
     */
    int16_t sysid_f_slot = (int16_t)(vel_cmd_rad_sec * 1000.0f);
    int16_t last_slot    = (int16_t)(vel_meas_counts / VEL_TELEM_DIV);

#elif SYSID_TEST == SYSID_TEST_FRICTION_SWEEP

    /* Friction sweep -- same convention as CL_VEL_STEP/CL_VEL_CHIRP:
     *   sysid_f slot -> velocity command [mrad/s]
     *   last slot    -> measured velocity [counts/s / VEL_TELEM_DIV]
     * iq_mA (native slot) is the actual friction-torque measurement --
     * steady-state per-step average is done Pi-side from the tail of each
     * ~1s hold.
     */
    int16_t sysid_f_slot = (int16_t)(vel_cmd_rad_sec * 1000.0f);
    int16_t last_slot    = (int16_t)(vel_meas_counts / VEL_TELEM_DIV);

#elif SYSID_TEST == SYSID_TEST_BREAKAWAY

    /* Breakaway/static friction: sysid_f slot -> commanded iq [mA] (ramp
     * value, or the handoff PI's output once breakaway is detected). last
     * slot -> measured velocity [counts/s / VEL_TELEM_DIV], to find the
     * exact tick breakaway occurred and read sysid_f at that same row.
     */
    int16_t sysid_f_slot = (int16_t)(iq_cmd * 1000.0f);
    int16_t last_slot    = (int16_t)(vel_meas_counts / VEL_TELEM_DIV);

#elif SYSID_TEST == RIPPLE_DEBUG

    /* Constant-iq ripple test:
     *   sysid_f slot -> actual iq command [mA]
     *   last slot    -> same measured velocity used by velocity controller
     */
    int16_t sysid_f_slot = (int16_t)(iq_cmd * 1000.0f);
    int16_t last_slot = (int16_t)(vel_meas_counts / VEL_TELEM_DIV);

#elif SYSID_TEST == SYSID_TEST_POSITION_STEP || SYSID_TEST == SYSID_TEST_CL_POS_CHIRP || SYSID_TEST == SYSID_TEST_CINE_SWEEP

    /* Position step / closed position loop chirp / cine sweep:
     *   sysid_f slot -> position command [mrad]
     *   last slot    -> measured position [mrad]
     */
    int16_t sysid_f_slot = (int16_t)(pos_cmd_rad * 1000.0f);
    int16_t last_slot    = (int16_t)(pos_meas_rad * 1000.0f);

#elif SYSID_TEST == SYSID_TEST_CL_CURRENT_CHIRP

    /* Closed-loop current chirp:
     *   sysid_f slot -> current command [mA]
     *   last slot    -> measured current [mA] (mirrors i_q_meas, already
     *                   logged natively in iq_mA -- kept here too so the
     *                   Pi-side script can use the same H=last/sysid_f
     *                   pattern every other closed-loop chirp test uses)
     */
    int16_t sysid_f_slot = (int16_t)(iq_cmd * 1000.0f);
    int16_t last_slot    = (int16_t)(i_q_meas * 1000.0f);

#elif SYSID_TEST == SYSID_TEST_CURRENT_LOOP_STEP

    /* Current-loop step: sysid_f slot is otherwise unused by this test
     * (only the chirp tests touch it) -- repurposed to carry ic_mA so
     * all three phase currents (ia, ib, ic) are visible over time,
     * verifying ia+ib+ic ~= 0 continuously instead of at one instant. */
    int16_t sysid_f_slot = (int16_t)(ic_meas * 1000.0f);
    int16_t last_slot    = (int16_t)(iq_cmd * 1000.0f);

#elif SYSID_TEST == SYSID_TEST_PHASE_CHECK

    /* Per-phase DC excitation: sysid_f slot carries ic_meas [mA] (ia/ib are
     * already natively logged in the fixed protocol below), so all three
     * phase currents are visible every tick. last slot carries which stage
     * is active (0=A held, 1=zero after A, 2=B held, 3=zero after B, 4=C
     * held, 5=zero after C, 6=done) so the Pi-side script can segment the
     * three DC steps. */
    int16_t sysid_f_slot = (int16_t)(ic_meas * 1000.0f);
    int16_t last_slot    = (int16_t)phase_check_stage;

#elif SYSID_TEST == SYSID_TEST_SHUNT_NOISE

    /* Shunt noise characterization: static zero-vector command the whole
     * run, so every sample is pure noise floor. sysid_f slot carries
     * ic_meas [mA] (ia/ib already natively logged below) so all three
     * shunt channels are visible every tick. last slot carries the ADC IRQ
     * sample counter -- a stalled/slow ADC would otherwise look identical
     * to "very low noise" (flat repeated readings), so this makes a stall
     * distinguishable from a genuinely quiet channel. */
    int16_t sysid_f_slot = (int16_t)(ic_meas * 1000.0f);
    int16_t last_slot    = (int16_t)(current_feedback_sample_count() & 0xFFFFu);

#elif SYSID_TEST == SYSID_TEST_VEL_CHIRP

    /* Open-loop velocity/mechanical-plant chirp:
     *   sysid_f slot -> chirp frequency [Hz]
     *   last slot    -> commanded current [mA]
     *
     * last slot briefly carried the ADC sample counter instead (diagnostic
     * for a stall theory that turned out not to apply here -- the P-only
     * current loop + tighter VEL_CHIRP_IQ_LIMIT fixed the real instability).
     * Reverted -- bode_vel_plot.py reads this column as iq_cmd_mA and uses
     * it as the coherence reference signal against theta; a monotonic
     * counter there has ~zero coherence with anything once detrended,
     * which silently broke the whole plant-ID fit (0 bins >= 0.5 coherence)
     * until this was caught.
     */
    int16_t sysid_f_slot = (int16_t)sysid_f;
    int16_t last_slot    = (int16_t)(iq_cmd * 1000.0f);

#else

    int16_t sysid_f_slot = (int16_t)sysid_f;
    int16_t last_slot    = (int16_t)(iq_cmd * 1000.0f);

#endif

    /* ALIGN-stage override, independent of SYSID_TEST -- carries RCC->CSR's
     * reset-cause flags (top byte: PORRSTF/BORRSTF/PINRSTF/SFTRSTF/
     * IWDGRSTF/WWDGRSTF/LPWRRSTF) so a brownout/reset during a violent
     * instability event shows up in the CSV directly, without needing the
     * debugger's register view (unreliable this session). g_reset_cause is
     * captured once at boot in main() and never cleared, so it accumulates
     * every reset cause since the last real power-cycle. */
    if (sysid_stage == SYSID_STAGE_ALIGN)
    {
        extern volatile uint32_t g_reset_cause;
        sysid_f_slot = (int16_t)((g_reset_cause >> 16) & 0xFFFFu);
    }

    /* enc_hi/enc_lo normally carry the raw 32-bit encoder position. For the
     * position-loop tests that's fully redundant with pos_meas_rad above (same
     * encoder count, just rescaled), so those two slots are repurposed to
     * carry the velocity profile instead -- vel_cmd_rad_sec (the position
     * P controller's output) and the raw measured velocity, same units/scale
     * SYSID_TEST_CL_VEL_STEP/CHIRP already use. */
#if SYSID_TEST == SYSID_TEST_POSITION_STEP || SYSID_TEST == SYSID_TEST_CL_POS_CHIRP || SYSID_TEST == SYSID_TEST_CINE_SWEEP
    int16_t enc_hi_slot = (int16_t)(vel_cmd_rad_sec * 1000.0f);   /* vel_cmd  [mrad/s]   */
    int16_t enc_lo_slot = (int16_t)(vel_meas_counts / VEL_TELEM_DIV); /* vel_meas [counts/s / VEL_TELEM_DIV] */
#elif SYSID_TEST == SYSID_TEST_CL_VEL_CHIRP
    /* enc_hi repurposed to carry the velocity controller's own output
     * (iq_cmd, what it's actually asking the current loop for) -- not
     * previously visible in telemetry at all for this test, only the
     * current loop's resulting id/iq and vd/vq were. enc_lo repurposed to
     * carry the ADC injected-conversion sample counter (low 16 bits) --
     * ia/ib were seen to freeze bit-identical for ~13 consecutive ticks
     * during an instability event while theta (a different peripheral) kept
     * updating normally, suggesting ADC_IRQHandler stalls. This counter only
     * advances when that ISR actually fires, so a flat run of samples here
     * directly proves/disproves the stall instead of inferring it. */
    int16_t enc_hi_slot = (int16_t)(iq_cmd * 1000.0f);             /* iq_cmd   [mA]       */
    int16_t enc_lo_slot = (int16_t)(current_feedback_sample_count() & 0xFFFFu); /* ADC IRQ counter */
#elif SYSID_TEST == SYSID_TEST_BREAKAWAY
    /* enc_hi repurposed to carry breakaway_phase (BreakawayPhase enum) so
     * the Pi-side script can segment ramp/handoff/settle without guessing
     * from the iq_cmd shape. enc_lo left at its normal raw-encoder-position
     * low word -- cheap smoothness/no-hard-stop check, same as the friction
     * sweep's default handling. */
    int16_t enc_hi_slot = (int16_t)breakaway_phase;
    int16_t enc_lo_slot = (int16_t)(enc_pos & 0xFFFF);
#else
    int16_t enc_hi_slot = (int16_t)(enc_pos >> 16);
    int16_t enc_lo_slot = (int16_t)(enc_pos & 0xFFFF);
#endif

    /* ALIGN-stage override, independent of SYSID_TEST -- carries the DRV8353's
     * own fault registers (FAULT_STATUS_1, VGS_STATUS_2), captured once at
     * boot right after drv_enable_high() in main(), so a driver-detected
     * gate-drive/VDS fault on a specific half-bridge shows up in the CSV
     * directly without needing the debugger's register view (unreliable
     * this session, same reason g_reset_cause above is telemetry-routed
     * instead of breakpoint-inspected). */
    if (sysid_stage == SYSID_STAGE_ALIGN)
    {
        extern volatile uint16_t g_drv_fault_status_1;
        extern volatile uint16_t g_drv_vgs_status_2;
        enc_hi_slot = (int16_t)g_drv_fault_status_1;
        enc_lo_slot = (int16_t)g_drv_vgs_status_2;
    }

    spi_sysid_update_latest(
        enc_hi_slot,                             /* enc_hi  OR vel_cmd mrad/s      */
        enc_lo_slot,                             /* enc_lo  OR vel_meas counts/s   */
        sysid_f_slot,                           /* sysid_f_Hz  OR vel_cmd mrad/s  */
        (int16_t)(i_d_meas * 1000.0f),          /* id_mA          */
        (int16_t)(i_q_meas * 1000.0f),          /* iq_mA          */
        (int16_t)(foc_vd_applied * 1000.0f),    /* vd_mV          */
        (int16_t)(foc_vq_applied * 1000.0f),    /* vq_mV          */
        (int16_t)(theta_telem * 1000.0f),       /* theta_mrad     */
        (int16_t)(ia_meas * 1000.0f),           /* ia_mA          */
        (int16_t)(ib_meas * 1000.0f),           /* ib_mA          */
        last_slot,                               /* iq_cmd_mA  OR vel_meas counts/s */
        flags,
        (uint16_t)SYSID_TEST                     /* which test is compiled in, for the Pi's script dispatch */
    );
}


// =============================================================================
// foc_sysid_reset — call on fault recovery or re-enable
// =============================================================================

void foc_sysid_reset(void)
{
    sysid_stage      = SYSID_STAGE_ALIGN;
    sysid_align_tick     = 0;
    sysid_discharge_tick = 0;
    sysid_enc_offset = 0;

    sysid_t         = 0.0f;
    chirp_angle_rad = 0.0f;
    sysid_f         = SYSID_F_START;

    cl_step_phase = CL_STEP_SETTLE;
    cl_step_tick  = 0;

    iq_cmd          = 0.0f;
    foc_vd_applied  = 0.0f;
    foc_vq_applied  = 0.0f;
    vel_cmd_rad_sec = 0.0f;
    vel_meas_counts = 0.0f;
    vel_loop_tick   = 0u;

    ia_meas  = ib_meas = ic_meas = 0.0f;
    i_d_meas = i_q_meas = 0.0f;

    vel_chirp_settle_tick   = 0u;

    cl_current_chirp_t         = 0.0f;
    cl_current_chirp_angle_rad = 0.0f;

    pos_chirp_settle_tick  = 0u;
    cl_pos_chirp_t         = 0.0f;
    cl_pos_chirp_angle_rad = 0.0f;

    cine_settle_tick = 0u;
    cine_sweep_t     = 0.0f;
    cine_sweep_angle = 0.0f;

    phase_check_stage = PHASE_CHECK_A;
    phase_check_tick  = 0u;
}
// foc_sysid.c — system identification tests (RUN_MODE_SYSID)
//
// foc_sysid_step() runs from the TIM1 ISR at 20 kHz: align the rotor, then
// run the test selected by SYSID_TEST (config.h), then idle. Every tick
// sends one SysIdSample frame; the frame carries SYSID_TEST so the Pi knows
// how to read the two test-specific slots (sysid_f and last).
//
//   0  CURRENT_LOOP_CHIRP   open-loop Vq chirp, theta = 0     -> R, L
//   3  CL_CURRENT_CHIRP     closed current loop, iq_cmd chirp -> current-loop margin
//   6  CL_VEL_CHIRP         deployed velocity PI, vel_cmd chirp
//  14  CL_VEL_PLANT_CHIRP   P-only velocity loop, vel_cmd chirp -> velocity plant
//   8  CL_POS_CHIRP         full cascade, pos_cmd chirp       -> position-loop margin
//  12  FRICTION_SWEEP       constant-velocity holds           -> Coulomb + viscous
//
// Only the selected test's run_* function is compiled.

#include "stm32f4xx.h"
#include "foc_sysid.h"
#include "observer.h"
#include "config.h"
#include "encoder.h"
#include "current_feedback.h"
#include "pwm.h"
#include "loops.h"
#include "spi.h"

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

// State shared by several tests; unused by whichever one is not compiled in.
#define SHARED __attribute__((unused))


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
static SHARED uint32_t vel_chirp_settle_tick = 0;

// =============================================================================
// Chirp state
// =============================================================================

static SHARED float sysid_f = SYSID_F_START;   // current chirp frequency [Hz]

// Velocity-chirp settle, in 5 kHz velocity ticks (4 ms): long enough for the
// velocity estimate to fill before the chirp starts.
#define CL_VEL_CHIRP_SETTLE_TICKS  20u

// =============================================================================
// Shared measured values (written each tick, sent in telemetry)
// =============================================================================

static float ia_meas  = 0.0f;
static float ib_meas  = 0.0f;
static SHARED float ic_meas = 0.0f;
static float i_d_meas = 0.0f;
static float i_q_meas = 0.0f;

static float foc_vd_applied = 0.0f;
static float foc_vq_applied = 0.0f;

static SHARED float vel_meas_counts = 0.0f;   // measured velocity [counts/s]

static SHARED uint32_t vel_loop_tick = 0;   // 20 kHz -> 5 kHz velocity divider


#if SYSID_TEST == SYSID_TEST_CURRENT_LOOP_CHIRP
// =============================================================================
// run_current_lp_chirp — test 0. Open-loop Vq chirp at theta = 0 (rotor
// held by alignment); logs id/iq for the R/L fit.
// =============================================================================

static float sysid_t         = 0.0f;
static float chirp_angle_rad = 0.0f;

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


#endif


#if SYSID_TEST == SYSID_TEST_CL_CURRENT_CHIRP
// =============================================================================
// run_cl_current_chirp — test 3. Chirps iq_cmd through the closed current
// PI; H = iq_meas / iq_cmd, and L = H / (1 - H) gives the loop margin.
//
// Telemetry: sysid_f = iq_cmd [mA], last = iq_meas [mA].
// =============================================================================

static float cl_current_chirp_t         = 0.0f;
static float cl_current_chirp_angle_rad = 0.0f;

static void run_cl_current_chirp(void)
{
    float theta = encoder_elec_angle(sysid_enc_offset);

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

        // sinf so the command starts at 0, continuous with the settle hold.
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


#endif


#if SYSID_TEST == SYSID_TEST_FRICTION_SWEEP
// =============================================================================
// run_friction_sweep — test 12. Deployed velocity PI (with integrator, so
// steady-state iq equals the friction torque), constant-velocity holds
// from FRICTION_SWEEP_V_MIN to V_MAX, one sweep per direction. The Pi
// averages iq over the tail of each hold.
//
// Telemetry: sysid_f = vel_cmd [mrad/s], last = vel_meas [counts/s / VEL_TELEM_DIV].
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
    float theta = encoder_elec_angle(sysid_enc_offset);

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    vel_meas_counts = encoder_get_velocity();

    if (++vel_loop_tick >= VEL_LOOP_DECIMATE)
    {
        vel_loop_tick = 0;

        float vel_meas_rad = velocity_feedback_rad(vel_meas_counts);

        // Position-limit backstop (FRICTION_SWEEP_POS_LIMIT_MM), checked
        // every velocity tick so a runaway ends the direction at once.
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


#endif


#if SYSID_TEST == SYSID_TEST_CL_VEL_CHIRP || SYSID_TEST == SYSID_TEST_CL_VEL_PLANT_CHIRP
// =============================================================================
// run_cl_vel_chirp — tests 6 and 14. Chirps vel_cmd with the velocity loop
// closed around it: the deployed PI (6) or a P-only loop for plant
// identification (14).
//
// Telemetry: sysid_f = vel_cmd [mrad/s], last = vel_meas [counts/s / VEL_TELEM_DIV],
// enc_hi = iq_cmd [mA], enc_lo = observer velocity (same units as last).
// =============================================================================

static float cl_vel_chirp_t         = 0.0f;
static float cl_vel_chirp_angle_rad = 0.0f;

// Which controller closes the velocity loop. Picked by the test ID, which
// telemetry carries, so every log records which one was in the loop.
typedef enum
{
    CL_VEL_CTRL_PLANT_ID = 0,   // private P-only loop, no feedforward
    CL_VEL_CTRL_DEPLOYED        // shared velocity_loop, FF per CL_VEL_CHIRP_FF
} ClVelController;

// P-only loop for plant identification, private so the shared
// velocity_loop's gains are never touched.
static PIState vel_plant_id_loop;

static void run_cl_vel_chirp(ClVelController ctrl)
{
    float theta = encoder_elec_angle(sysid_enc_offset);

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    vel_meas_counts = encoder_get_velocity();

    if (++vel_loop_tick >= VEL_LOOP_DECIMATE)
    {
        vel_loop_tick = 0;

        float vel_meas_rad = velocity_feedback_rad(vel_meas_counts);

        if (vel_chirp_settle_tick < CL_VEL_CHIRP_SETTLE_TICKS)
        {
            if (vel_chirp_settle_tick == 0u && ctrl == CL_VEL_CTRL_PLANT_ID)
            {
                // P-only, Ki=0 (CL_VEL_CHIRP_PONLY_KP). The deployed path
                // already has VEL_KP/VEL_KI from loops_reset().
                pi_init(&vel_plant_id_loop, CL_VEL_CHIRP_PONLY_KP, 0.0f, -0.5f, 0.5f);
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

            // sinf so the command starts at 0, continuous with the settle hold.
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

        // Plant ID uses pi_step, not velocity_loop_step: the friction
        // feedforward would make C(s) != Kp and break the back-out
        // P = L / Kp.
        if (ctrl == CL_VEL_CTRL_PLANT_ID)
        {
            iq_cmd = pi_step(&vel_plant_id_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
        }
        else
        {
#if CL_VEL_CHIRP_FF
            iq_cmd = velocity_loop_step(vel_cmd_rad_sec, vel_meas_rad, DT_VELOCITY);
#else
            /* deployed PI, friction feedforward off (CL_VEL_CHIRP_FF) */
            iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
#endif
        }
    }

    foc_vd_applied = pi_step(&d_current_loop, 0.0f - i_d_meas, SYSID_DT);
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


#endif


#if SYSID_TEST == SYSID_TEST_CL_POS_CHIRP
// =============================================================================
// run_cl_pos_chirp — test 8. Chirps pos_cmd with the full cascade closed:
// position P (1 kHz) -> velocity PI (5 kHz) -> current PI (20 kHz).
//
// Telemetry: sysid_f = pos_cmd [mrad], last = pos_meas [mrad],
// enc_hi = vel_cmd [mrad/s / VEL_CMD_TELEM_DIV], enc_lo = vel_meas [counts/s / VEL_TELEM_DIV].
// =============================================================================

#define POS_LOOP_DECIMATE  20u   // 20 kHz / 20 = 1 kHz = DT_POSITION

static uint32_t pos_loop_tick          = 0;
static float    pos_cmd_rad            = 0.0f;
static float    pos_meas_rad           = 0.0f;
static uint32_t pos_chirp_settle_tick  = 0;
static float    cl_pos_chirp_t         = 0.0f;
static float    cl_pos_chirp_angle_rad = 0.0f;

static void run_cl_pos_chirp(void)
{
    float theta = encoder_elec_angle(sysid_enc_offset);

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

        if (pos_chirp_settle_tick < CL_POS_CHIRP_SETTLE_TICKS)
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
        float vel_meas_rad = velocity_feedback_rad(vel_meas_counts);
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


#endif


// =============================================================================
// foc_sysid_step — called from TIM1_UP_TIM10_IRQHandler at 20 kHz
// =============================================================================

void foc_sysid_step(void)
{
    switch (sysid_stage)
    {
        case SYSID_STAGE_ALIGN:
        {
            if (sysid_align_tick < 20000u)
            {
                // 1 s d-axis hold pulls the rotor to electrical angle 0.
                foc_vd_applied = V_ALIGN;
                foc_vq_applied = 0.0f;
                pwm_apply_dq(foc_vd_applied, foc_vq_applied, 0.0f);
                ++sysid_align_tick;
            }
            else
            {
                // Then 100 ms with zero volts so the align current decays
                // (L/R < 1 ms) before any loop closes.
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
        #if   SYSID_TEST == SYSID_TEST_CURRENT_LOOP_CHIRP
            run_current_lp_chirp();
        #elif SYSID_TEST == SYSID_TEST_CL_CURRENT_CHIRP
            run_cl_current_chirp();
        #elif SYSID_TEST == SYSID_TEST_CL_VEL_CHIRP
            run_cl_vel_chirp(CL_VEL_CTRL_DEPLOYED);
        #elif SYSID_TEST == SYSID_TEST_CL_VEL_PLANT_CHIRP
            run_cl_vel_chirp(CL_VEL_CTRL_PLANT_ID);
        #elif SYSID_TEST == SYSID_TEST_CL_POS_CHIRP
            run_cl_pos_chirp();
        #elif SYSID_TEST == SYSID_TEST_FRICTION_SWEEP
            run_friction_sweep();
        #else
            #error "SYSID_TEST: unknown test"
        #endif
        }
        break;

        case SYSID_STAGE_IDLE:
        {
            foc_vd_applied = 0.0f;
            foc_vq_applied = 0.0f;
            pwm_apply_dq(0.0f, 0.0f, 0.0f);
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
    float theta_telem = encoder_elec_angle(sysid_enc_offset);
#endif

    int32_t enc_pos = encoder_get_position();

#if OBSERVER_ENABLE
    observer_update(enc_pos, i_q_meas);
#endif

    // Test-specific slots; see each run_* header for units.
#if SYSID_TEST == SYSID_TEST_CL_VEL_CHIRP || SYSID_TEST == SYSID_TEST_CL_VEL_PLANT_CHIRP
    int16_t sysid_f_slot = (int16_t)(vel_cmd_rad_sec * 1000.0f);
    int16_t last_slot    = (int16_t)(vel_meas_counts / VEL_TELEM_DIV);
    int16_t enc_hi_slot  = (int16_t)(iq_cmd * 1000.0f);
    int16_t enc_lo_slot  = (int16_t)(observer_get_velocity() * COUNTS_PER_RAD / VEL_TELEM_DIV);
#elif SYSID_TEST == SYSID_TEST_FRICTION_SWEEP
    int16_t sysid_f_slot = (int16_t)(vel_cmd_rad_sec * 1000.0f);
    int16_t last_slot    = (int16_t)(vel_meas_counts / VEL_TELEM_DIV);
    int16_t enc_hi_slot  = (int16_t)(enc_pos >> 16);
    int16_t enc_lo_slot  = (int16_t)(enc_pos & 0xFFFF);
#elif SYSID_TEST == SYSID_TEST_CL_POS_CHIRP
    int16_t sysid_f_slot = (int16_t)(pos_cmd_rad * 1000.0f);
    int16_t last_slot    = (int16_t)(pos_meas_rad * 1000.0f);
    int16_t enc_hi_slot  = (int16_t)(vel_cmd_rad_sec * 1000.0f / VEL_CMD_TELEM_DIV);
    int16_t enc_lo_slot  = (int16_t)(vel_meas_counts / VEL_TELEM_DIV);
#elif SYSID_TEST == SYSID_TEST_CL_CURRENT_CHIRP
    int16_t sysid_f_slot = (int16_t)(iq_cmd * 1000.0f);
    int16_t last_slot    = (int16_t)(i_q_meas * 1000.0f);
    int16_t enc_hi_slot  = (int16_t)(enc_pos >> 16);
    int16_t enc_lo_slot  = (int16_t)(enc_pos & 0xFFFF);
#else   /* CURRENT_LOOP_CHIRP: sysid_f = chirp frequency [Hz] */
    int16_t sysid_f_slot = (int16_t)sysid_f;
    int16_t last_slot    = (int16_t)(iq_cmd * 1000.0f);
    int16_t enc_hi_slot  = (int16_t)(enc_pos >> 16);
    int16_t enc_lo_slot  = (int16_t)(enc_pos & 0xFFFF);
#endif

    // During ALIGN the slots carry boot diagnostics instead: reset cause
    // (RCC->CSR top half) and the DRV8353 fault registers (main.c).
    if (sysid_stage == SYSID_STAGE_ALIGN)
    {
        extern volatile uint32_t g_reset_cause;
        extern volatile uint16_t g_drv_fault_status_1;
        extern volatile uint16_t g_drv_vgs_status_2;
        sysid_f_slot = (int16_t)((g_reset_cause >> 16) & 0xFFFFu);
        enc_hi_slot  = (int16_t)g_drv_fault_status_1;
        enc_lo_slot  = (int16_t)g_drv_vgs_status_2;
    }

    spi_sysid_update_latest(
        enc_hi_slot,
        enc_lo_slot,
        sysid_f_slot,
        (int16_t)(i_d_meas * 1000.0f),          /* id_mA      */
        (int16_t)(i_q_meas * 1000.0f),          /* iq_mA      */
        (int16_t)(foc_vd_applied * 1000.0f),    /* vd_mV      */
        (int16_t)(foc_vq_applied * 1000.0f),    /* vq_mV      */
        (int16_t)(theta_telem * 1000.0f),       /* theta_mrad */
        (int16_t)(ia_meas * 1000.0f),           /* ia_mA      */
        (int16_t)(ib_meas * 1000.0f),           /* ib_mA      */
        last_slot,
        flags,
        (uint16_t)SYSID_TEST
    );
}

// drive.c — servo drive state machine
//
// Architecture:
//   drive_sm_run() runs in SysTick at 1 kHz — only place state transitions happen.
//   TIM1 ISR calls drive_get_state() to gate loop execution — reads only.
//   DMA ISR calls drive_request_*() — sets flags only.
//
// Initialization principle:
//   State init runs at the *transition*, in the source state's transition
//   path, BEFORE the new state is assigned. This matters because SysTick
//   pops the ring after drive_sm_run() returns; if init were deferred to
//   the entry tick of the destination state, it would wipe samples_consumed
//   and first_sample_ready *after* SysTick had already advanced them on
//   the transition tick, losing exactly one sample's count.
//   (Bug recovered Nov 2026; see jz-min-streamer branch history.)

#include "drive.h"
#include "spi.h"
#include "loops.h"
#include "plant.h"
#include "control.h"
#include "ringBuffer.h"
#include "pwm.h"
#include "config.h"
#include <math.h>
#include <stdint.h>

// ── State ─────────────────────────────────────────────────────────────────────
static DriveState state      = STATE_IDLE;
static DriveState state_prev = STATE_IDLE;

// ── Request flags — set by ISRs, cleared by drive_sm_run() ───────────────────
static volatile uint8_t servo_on_req  = 0;
static volatile uint8_t open_loop_req = 0;
static volatile uint8_t stop_req      = 0;
static volatile uint8_t fault_req     = 0;

// ── Open-loop parameters — written by DMA ISR, read by SysTick ───────────────
static volatile float ol_v_mag   = 0.0f;
static volatile float ol_d_theta = 0.0f;

// ── Open-loop angle accumulator ───────────────────────────────────────────────
static float ol_theta = 0.0f;

// ── entry_flag — set on state transition, cleared by starting() ──────────────
static uint8_t                  starting_flag = 0;

drive_t drive = {0}; 
// ─────────────────────────────────────────────────────────────────────────────
// drive_init
// ─────────────────────────────────────────────────────────────────────────────
void drive_init(void)
{
    state         = STATE_IDLE;
    state_prev    = STATE_IDLE;
    servo_on_req  = 0;
    open_loop_req = 0;
    stop_req      = 0;
    fault_req     = 0;
    starting_flag = 0;
    ol_theta      = 0.0f;
}

// Drive State
bool drive_is_idle(void)      { return state == STATE_IDLE; }
bool drive_is_open_loop(void) { return state == STATE_OPEN_LOOP; }
bool drive_is_aligned(void)   { return state == STATE_ALIGN; }
bool drive_is_servo_on(void)  { return state == STATE_SERVO_ON; }
bool drive_is_faulted(void)   { return state == STATE_FAULT; }

// ─────────────────────────────────────────────────────────────────────────────
// Request functions — called from DMA ISR or any ISR
// ─────────────────────────────────────────────────────────────────────────────
void drive_request_servo_on(void)
{
    servo_on_req = 1;
}

void drive_request_open_loop(float v_mag, float d_theta)
{
    ol_v_mag      = v_mag;
    ol_d_theta    = d_theta;
    open_loop_req = 1;
}

void drive_request_stop(void)
{
    stop_req = 1;
}

void drive_request_fault(void)
{
    fault_req = 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_get_state
// ─────────────────────────────────────────────────────────────────────────────
DriveState drive_get_state(void)
{
    return state;
}

// ─────────────────────────────────────────────────────────────────────────────
// starting_flag — returns 1 once on state entry, then 0
// (Still used by OPEN_LOOP and ALIGN entry; SERVO_ON no longer uses it.)
// ─────────────────────────────────────────────────────────────────────────────
uint8_t starting(void)
{
    uint8_t e = starting_flag;
    starting_flag = 0;
    return e;
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_align_rotor — apply fixed d-axis voltage at theta=0
//
// Forces rotor to known electrical angle before closing FOC loop.
// Required for AKM11E (N2 option — no Hall sensors).
// ─────────────────────────────────────────────────────────────────────────────
void drive_align_rotor(void)
{
    pwm_apply_dq(V_ALIGN, 0.0f, 0.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_sm_run — call from SysTick at 1 kHz
// ─────────────────────────────────────────────────────────────────────────────
void drive_sm_run(void)
{
    // fault takes priority from any state — latches immediately
    if (fault_req)
    {
        pwm_disable();
        state     = STATE_FAULT;
        fault_req = 0;
        return;
    }

    state_prev = state;

    switch (state) {

    // ── IDLE — waiting for Pi command ─────────────────────────────────────────
    case STATE_IDLE:
        if (open_loop_req)
        {
            open_loop_req = 0;
            state         = STATE_OPEN_LOOP;
            starting_flag = 1;
        }
        else if (servo_on_req)
        {
            servo_on_req = 0;

            // ── SERVO_ON init — runs at transition, NOT on entry tick ──
            // This MUST precede the state assignment below: SysTick will
            // call ring_pop() immediately after drive_sm_run() returns
            // this tick, and any reset of samples_consumed / first_sample_ready
            // must happen before that pop, not after it.
            loops_reset();
#ifdef SIM_MODE
            plant_init(&plant);
#endif
            ol_theta           = 0.0f;
            first_sample_ready = 0;
            samples_consumed   = 0;
            pwm_enable();

            state = STATE_SERVO_ON;
        }
        break;

    // ── OPEN_LOOP — spinning without encoder feedback ─────────────────────────
    // ol_v_mag and ol_d_theta set by drive_request_open_loop() from DMA ISR.
    // open_loop_step() lives in control.c — it's motor math, not state logic.
    case STATE_OPEN_LOOP:
        if (starting())
        {
            ol_theta = 0.0f;
            pwm_enable();
        }
        open_loop_step(&ol_theta, ol_v_mag, ol_d_theta);
        if (stop_req)
        {
            stop_req      = 0;
            pwm_disable();
            state         = STATE_IDLE;
            starting_flag = 1;
        }
        break;

    // ── ALIGN — lock rotor to theta=0 before closing FOC ─────────────────────
    // Applies fixed d-axis voltage with v_q=0 — no torque, rotor locks to theta=0.
    // Required for AKM11E (N2 encoder-only option, no Hall sensors).
    case STATE_ALIGN:
        if (starting())
            pwm_enable();
        drive_align_rotor();
        // TODO: tick counter → STATE_SERVO_ON after ALIGN_TIME_MS
        break;

    // ── SERVO_ON — closed-loop FOC with trajectory from Pi ───────────────────
    // Init happens at transition in the IDLE case above, not here.
    // This block only runs the running-state logic + the drain check.
    case STATE_SERVO_ON:
        if ((expected_samples > 0u && samples_consumed >= expected_samples) ||
            (expected_samples == 0u &&
             ring_count() == 0u && first_sample_ready && samples_consumed > 0))
        {   
            pwm_disable();
            state              = STATE_IDLE;
            starting_flag      = 1;
            first_sample_ready = 0;
        }
        break;

    // ── FAULT — latched, PWM disabled ────────────────────────────────────────
    // DRV8353RS already killed gate outputs when nFAULT asserted.
    // Requires fault reset opcode from Pi to recover → STATE_IDLE.
    case STATE_FAULT:
        pwm_disable();
        // TODO: SPI2_OP_FAULT_RESET → pulse ENABLE low 20µs → STATE_IDLE
        break;
    }
}
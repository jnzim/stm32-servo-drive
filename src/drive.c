// drive.c — servo drive state machine: IDLE -> SERVO_ON -> IDLE, plus FAULT
//
// RUN_MODE_CLOSED_LOOP only (SysTick does not call drive_sm_run() in sysid).
//
//   drive_sm_run()  SysTick, 1 kHz. The only place the state changes.
//   Readers         TIM1 ISR (foc_trajectory_step) and the position tick
//                   check drive_is_servo_on() before running the loops.
//   Requests        servo-on from the SPI DMA ISR on a MoveCmd; fault from
//                   the position tick on a following-error trip. Both only
//                   set a flag that drive_sm_run() acts on.

#include "drive.h"
#include "loops.h"
#include "pwm.h"
#include "foc_trajectory.h"
#include <stdint.h>

// ── State ─────────────────────────────────────────────────────────────────────
static DriveState state = STATE_IDLE;

// ── Request flags — set by ISRs, cleared by drive_sm_run() ───────────────────
static volatile uint8_t servo_on_req = 0;
static volatile uint8_t fault_req    = 0;

// ─────────────────────────────────────────────────────────────────────────────
// drive_init
// ─────────────────────────────────────────────────────────────────────────────
void drive_init(void)
{
    state        = STATE_IDLE;
    servo_on_req = 0;
    fault_req    = 0;
}

bool drive_is_idle(void)      { return state == STATE_IDLE; }
bool drive_is_servo_on(void)  { return state == STATE_SERVO_ON; }

// ─────────────────────────────────────────────────────────────────────────────
// Request functions — set a flag; drive_sm_run() acts on it next tick
// ─────────────────────────────────────────────────────────────────────────────
void drive_request_servo_on(void)
{
    servo_on_req = 1;
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

    switch (state) {

    // ── IDLE — waiting for Pi command ─────────────────────────────────────────
    case STATE_IDLE:
        if (servo_on_req)
        {
            servo_on_req = 0;

            // Reset before entering SERVO_ON. The next position tick in
            // the TIM1 ISR sets first_sample_ready and counts
            // samples_consumed for the move.
            loops_reset();
            first_sample_ready = 0;
            samples_consumed   = 0;
            pwm_enable();

            state = STATE_SERVO_ON;
        }
        break;

    // ── SERVO_ON — loops run; back to IDLE when the move generator is done ──
    case STATE_SERVO_ON:
        if (!foc_trajectory_move_active())
        {
            pwm_disable();
            state              = STATE_IDLE;
            first_sample_ready = 0;
            // Drop any request latched while this move ran, so finishing a
            // move can never re-enter SERVO_ON by itself (observed 2026-10-10:
            // samples_consumed 1875 -> 1 between moves, with no new command).
            servo_on_req       = 0;
        }
        break;

    // ── FAULT — latched, PWM off. Only a reset clears it. ───────────────────
    case STATE_FAULT:
        pwm_disable();
        break;
    }
}
// drive.c — servo drive state machine: IDLE -> SERVO_ON -> IDLE, plus FAULT
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

#include "drive.h"
#include "spi.h"
#include "loops.h"
#include "plant.h"
#include "ringBuffer.h"
#include "pwm.h"
#include "config.h"
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
// Request functions — called from DMA ISR or any ISR
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

            // ── SERVO_ON init — runs at transition, NOT on entry tick ──
            // This MUST precede the state assignment below: SysTick will
            // call ring_pop() immediately after drive_sm_run() returns
            // this tick, and any reset of samples_consumed / first_sample_ready
            // must happen before that pop, not after it.
            loops_reset();
#ifdef SIM_MODE
            plant_init(&plant);
#endif
            first_sample_ready = 0;
            samples_consumed   = 0;
            pwm_enable();

            state = STATE_SERVO_ON;
        }
        break;

    // ── SERVO_ON — closed-loop FOC with trajectory from Pi ───────────────────
    // Init happens at transition in the IDLE case above, not here.
    // This block only runs the running-state logic + the drain check.
    //
    // RUN_MODE_SYSID drains the ring/sample-count as before -- unrelated to
    // this rebuild. RUN_MODE_CLOSED_LOOP has no ring being drained, so its
    // exit is real completion: the trapezoidal generator reporting DONE.
    case STATE_SERVO_ON:
#if RUN_MODE == RUN_MODE_SYSID
        if ((expected_samples > 0u && samples_consumed >= expected_samples) ||
            (expected_samples == 0u &&
             ring_count() == 0u && first_sample_ready && samples_consumed > 0))
#else
        if (!foc_trajectory_move_active())
#endif
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

    // ── FAULT — latched, PWM disabled ────────────────────────────────────────
    // DRV8353RS already killed gate outputs when nFAULT asserted.
    // Requires fault reset opcode from Pi to recover → STATE_IDLE.
    case STATE_FAULT:
        pwm_disable();
        // TODO: SPI2_OP_FAULT_RESET → pulse ENABLE low 20µs → STATE_IDLE
        break;
    }
}
#pragma once

#include "control.h"

// ── Controller state ──────────────────────────────────────────────────────────
extern PIState current_loop;
extern PIState d_current_loop;
extern PIState velocity_loop;
extern PState  position_loop;

// ── Inter-loop setpoints ──────────────────────────────────────────────────────
// vel_cmd_rad_sec is volatile: written by SysTick (position loop), read by
// the TIM1 ISR's decimated velocity loop -- genuinely cross-ISR now that the
// position loop lives in SysTick rather than sharing the same 20kHz context.
extern volatile float vel_cmd_rad_sec;
extern float          iq_cmd;
extern volatile float v_q_cmd;

// ── Rate dividers ─────────────────────────────────────────────────────────────
extern uint32_t vel_div;

// ── Sequencing ────────────────────────────────────────────────────────────────
extern volatile uint8_t first_sample_ready;

// ── Trajectory consumption ────────────────────────────────────────────────────
extern volatile uint32_t samples_consumed;

// ── API ───────────────────────────────────────────────────────────────────────
void loops_reset(void);

// Velocity PI + friction feedforward, single source of truth for both sysid
// tests and real trajectory mode. See loops.c for the full rationale.
float velocity_loop_step(float vel_cmd_rad_sec, float vel_meas_rad, float dt);

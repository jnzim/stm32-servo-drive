#pragma once

#include "control.h"

// ── Controller state ──────────────────────────────────────────────────────────
extern PIState current_loop;
extern PIState d_current_loop;
extern PIState velocity_loop;
extern PState  position_loop;

// ── Inter-loop setpoints ──────────────────────────────────────────────────────
// vel_cmd_rad_sec: written by the position loop (SysTick), read by the
// velocity loop (TIM1 ISR).
extern volatile float vel_cmd_rad_sec;
extern float          iq_cmd;

// ── Rate dividers ─────────────────────────────────────────────────────────────
extern uint32_t vel_div;

// ── Sequencing ────────────────────────────────────────────────────────────────
extern volatile uint8_t first_sample_ready;

// ── Trajectory consumption ────────────────────────────────────────────────────
extern volatile uint32_t samples_consumed;

// ── API ───────────────────────────────────────────────────────────────────────
void loops_reset(void);

// Velocity PI + friction feedforward (see loops.c).
float velocity_loop_step(float vel_cmd_rad_sec, float vel_meas_rad, float dt);

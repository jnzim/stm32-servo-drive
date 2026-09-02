#pragma once

#include "control.h"

// ── Controller state ──────────────────────────────────────────────────────────
extern PIState current_loop;
extern PIState d_current_loop;
extern PIState velocity_loop;
extern PState  position_loop;

// ── Inter-loop setpoints ──────────────────────────────────────────────────────
extern float          vel_cmd_rad_sec;
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

// Open-loop electrical drive — called from TIM1 ISR when STATE_OPEN_LOOP active
void open_loop_step(float *theta, float v_mag, float d_theta);
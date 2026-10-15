// foc_trajectory.h — closed-loop trajectory mode FOC
//
// Called from TIM1_UP_TIM10_IRQHandler when RUN_MODE == RUN_MODE_CLOSED_LOOP.
//
// Owns the full ISR execution in trajectory mode:
//   - d-axis alignment stage
//   - closed current loop  @ 20 kHz
//   - closed velocity loop @ 5 kHz
//   - move generator and position loop live in SysTick @ 1 kHz
//     (foc_trajectory_position_tick) and write vel_cmd_rad_sec via loops.h

#pragma once

#include <stdbool.h>
#include <stdint.h>

// foc_trajectory_step — call from TIM1 ISR at 20 kHz, RUN_MODE_CLOSED_LOOP only
void foc_trajectory_step(void);

// foc_trajectory_position_tick — call from SysTick at 1 kHz, every tick
void foc_trajectory_position_tick(void);

// foc_trajectory_start_move — request a move. target_pos is a DELTA in
// counts from wherever the stage actually is when the move starts, not an
// absolute position -- resolved against the live encoder position inside
// foc_trajectory_position_tick(), not here. vel_max/accel_max are
// counts/s / counts/s^2 magnitudes. Safe to call before RUN begins (e.g.
// from the SPI RX ISR while still in ALIGN) — just latches the request;
// foc_trajectory_position_tick() starts the generator for real once
// traj_stage reaches RUN.
void foc_trajectory_start_move(int32_t target_pos, int32_t vel_max, int32_t accel_max);

// foc_trajectory_move_active — true from a requested/running move until the
// generator reports DONE.
bool foc_trajectory_move_active(void);
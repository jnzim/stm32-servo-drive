// foc_trajectory.h — closed-loop trajectory mode FOC
//
// Called from TIM1_UP_TIM10_IRQHandler when RUN_MODE == RUN_MODE_CLOSED_LOOP.
//
// Owns the full ISR execution in trajectory mode:
//   - d-axis alignment stage
//   - closed current loop  @ 20 kHz
//   - closed velocity loop @ 5 kHz
//   - position loop and ring buffer pop live in SysTick @ 1 kHz
//     and write vel_cmd_rad_sec via loops.h — this file consumes it

#pragma once

#include <stdbool.h>
#include <stdint.h>

// foc_trajectory_step — call from TIM1 ISR at 20 kHz, RUN_MODE_CLOSED_LOOP only
void foc_trajectory_step(void);

// foc_trajectory_position_tick — call from SysTick at 1 kHz, SERVO_ON only
void foc_trajectory_position_tick(void);

// foc_trajectory_reset — call on fault recovery or re-enable
void foc_trajectory_reset(void);

// foc_trajectory_start_move — request a move (counts, counts/s, counts/s^2,
// relative to the post-ALIGN encoder offset). Safe to call before RUN
// begins (e.g. from boot scaffolding, or later the SPI RX ISR) — just
// latches the request; foc_trajectory_position_tick() starts the generator
// for real once traj_stage reaches RUN.
void foc_trajectory_start_move(int32_t target_pos, int32_t vel_max, int32_t accel_max);

// foc_trajectory_move_active — true from a requested/running move until the
// generator reports DONE.
bool foc_trajectory_move_active(void);
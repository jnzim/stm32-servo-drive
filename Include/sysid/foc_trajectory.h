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

#include <stdint.h>
#include <stdbool.h>

// foc_trajectory_step — call from TIM1 ISR at 20 kHz, RUN_MODE_CLOSED_LOOP only
void foc_trajectory_step(void);

// foc_trajectory_position_tick — call from SysTick at 1 kHz, SERVO_ON only
void foc_trajectory_position_tick(void);

// foc_trajectory_start_move — request a move (M2). target_pos/vel_max/
// accel_max are counts relative to the post-align zero (same frame the
// position loop uses), not raw encoder_get_position() counts. Safe to call
// from any ISR context -- see the .c file for why.
void foc_trajectory_start_move(int32_t target_pos, int32_t vel_max, int32_t accel_max);

// foc_trajectory_move_active — true from the moment a move is requested
// (foc_trajectory_start_move()) until the generator reports DONE. drive.c's
// STATE_SERVO_ON exit uses this instead of the old ring-drain check
// (ring_count()==0) now that a move is a single command, not a streamed
// ring -- the ring is never populated under this path, so ring_count()
// would read 0 immediately and exit SERVO_ON one tick after entry.
bool foc_trajectory_move_active(void);

// foc_trajectory_reset — call on fault recovery or re-enable
void foc_trajectory_reset(void);
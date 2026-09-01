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

// foc_trajectory_step — call from TIM1 ISR at 20 kHz, RUN_MODE_CLOSED_LOOP only
void foc_trajectory_step(void);

// foc_trajectory_reset — call on fault recovery or re-enable
void foc_trajectory_reset(void);
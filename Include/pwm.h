#pragma once
#include <stdint.h>

// pwm_init — configure TIM1 + GPIO, outputs disabled (MOE=0)
void pwm_init(void);

// pwm_enable — set MOE, PWM signals reach gate driver
void pwm_enable(void);

// pwm_disable — clear MOE, all outputs low
void pwm_disable(void);

// pwm_apply_dq — full FOC output stage
// Convention: (vd, vq, theta) — matches inverse Park argument order directly:
//   v_alpha = vd*cos(θ) - vq*sin(θ)
//   v_beta  = vd*sin(θ) + vq*cos(θ)
// v_d: d-axis voltage (alignment / field weakening)
// v_q: q-axis voltage (current loop output — torque)
// theta: electrical rotor angle in radians (from encoder)
void pwm_apply_dq(float v_d, float v_q, float theta);

void pwm_apply_phase_volts(float va, float vb, float vc);
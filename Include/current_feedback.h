
#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * current_feedback.h
 *
 * Bring-up current feedback module.
 *
 * Current version:
 *   - ADC1 scans PC0/PC1/PC2 continuously
 *   - DMA2 Stream0 Channel0 fills current_adc_raw[3]
 *   - zero-current offsets are calibrated with PWM disabled
 *   - caller can read ia/ib/ic or transformed id/iq
 *
 * This is not yet final synchronized FOC sampling.
 * Later, ADC triggering should be tied to TIM1 at a quiet PWM point.
 */

 #define ADC_SR_JEOC_BIT   (1u << 2)
extern volatile uint16_t current_adc_raw[3];
extern volatile float adc_offset[3];

void current_feedback_init(void);
void current_feedback_calibrate(void);

bool current_feedback_sample_ready(void);
void current_feedback_clear_sample_ready(void);

void current_feedback_get_phase_amps(float *ia, float *ib, float *ic);
void current_feedback_get_dq(float theta, float *i_d, float *i_q);

float current_feedback_get_offset_a(void);
float current_feedback_get_offset_b(void);
float current_feedback_get_offset_c(void);

bool current_feedback_sample_valid(void);
uint32_t current_feedback_sample_count(void);


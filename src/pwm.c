// pwm.c — TIM1 3-phase PWM for DRV8353RS-EVM gate driver, 3x PWM mode
// STM32F411 bare metal
//
// Ownership:
//   This module owns:
//     - TIM1 PWM setup
//     - PA8 / PA9 / PA10 PWM pins
//     - phase voltage -> CCR conversion
//     - inverse Park / inverse Clarke voltage output
//
// DRV8353 3x PWM mode:
//   DRIVER_CONTROL = 0x0020
//
//   INHA / INHB / INHC receive PWM from TIM1 CH1/CH2/CH3.
//   INLA / INLB / INLC are per-phase enables and are tied high in hardware.
//   The DRV generates complementary low-side drive internally.
//
// Therefore:
//   - No TIM1 complementary outputs are used.
//   - No STM32 dead-time insertion is used.
//   - TIM1 only drives CH1 / CH2 / CH3.
//
// Pin map:
//   PA8  / TIM1_CH1 -> INHA
//   PA9  / TIM1_CH2 -> INHB
//   PA10 / TIM1_CH3 -> INHC
//
// Pins this module must not touch:
//   PA5 / PA6 / PA7 = DRV SPI1
//   PB0             = DRV ENABLE
//   PB1             = DRV nFAULT
//
// PWM timing:
//   Center-aligned PWM.
//   APB2 timer clock assumed 100 MHz.
//   ARR = 2499 gives:
//      f_pwm = 100 MHz / (2 * (2499 + 1)) = 20 kHz
//
// ADC trigger:
//   TIM1 CR2 MMS=111 routes OC4REF as TRGO.
//   ADC JEXTSEL=1 selects TIM1_TRGO as injected trigger.
//   CCR4 = PWM_ARR - PWM_SAMPLE_OFFSET; the ADC triggers on the up-count
//   crossing only. The 3 channels convert sequentially (1.08 us each at
//   15 cycles), so the offset centers the sequence on the peak: A sampled
//   ~1.08 us before, B at, C ~1.08 us after -- inside every phase's low-side
//   window up to ~95% duty. Verified 2026-09-19: scope (GLC vs PC4) and
//   PHASE_CHECK KCL at 62.5% C duty.
//
// volts_to_duty convention:
//   v is a centered phase voltage in the range [-V_BUS/2, +V_BUS/2].
//   normalized = v / (V_BUS / 2):
//     -1 -> min duty (DUTY_MIN)
//      0 -> 50% duty (PWM_CENTER)
//     +1 -> max duty (DUTY_MAX)
//
// pwm_apply_dq convention:
//   pwm_apply_dq(vd, vq, theta) — matches inverse Park argument order directly.
//   Do not swap vd/vq. Do not change signs in the transform.

#include "pwm.h"
#include "board_f411.h"
#include "stm32f4xx.h"
#include "config.h"

#include <math.h>
#include <stdint.h>


// =============================================================================
// PWM geometry
// =============================================================================

#define PWM_ARR           2499u
#define PWM_CENTER        (PWM_ARR / 2u)
#define PWM_SAMPLE_OFFSET 168u                 // 3-ch sequence (3x1.08us) centered on peak: B at peak
#define DUTY_MIN          (PWM_ARR * 0.04f)    // ~4%  = 100 counts
#define DUTY_MAX          (PWM_ARR * 0.96f)    // ~96% = 2399 counts


// =============================================================================
// pwm_init
// =============================================================================

void pwm_init(void)
{
    // -------------------------------------------------------------------------
    // Enable peripheral clocks
    // -------------------------------------------------------------------------

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    (void)RCC->APB2ENR;
    (void)RCC->AHB1ENR;

    // -------------------------------------------------------------------------
    // Configure PA8 / PA9 / PA10 as AF1 for TIM1_CH1/CH2/CH3
    // -------------------------------------------------------------------------

    GPIOA->MODER &= ~((3u << (PIN_PWM_A * 2u)) |
                      (3u << (PIN_PWM_B * 2u)) |
                      (3u << (PIN_PWM_C * 2u)));

    GPIOA->MODER |=  ((2u << (PIN_PWM_A * 2u)) |
                      (2u << (PIN_PWM_B * 2u)) |
                      (2u << (PIN_PWM_C * 2u)));

    GPIOA->OSPEEDR &= ~((3u << (PIN_PWM_A * 2u)) |
                        (3u << (PIN_PWM_B * 2u)) |
                        (3u << (PIN_PWM_C * 2u)));

    GPIOA->OSPEEDR |=  ((1u << (PIN_PWM_A * 2u)) |
                        (1u << (PIN_PWM_B * 2u)) |
                        (1u << (PIN_PWM_C * 2u)));

    GPIOA->AFR[1] &= ~((0xFu << ((PIN_PWM_A - 8u) * 4u)) |
                       (0xFu << ((PIN_PWM_B - 8u) * 4u)) |
                       (0xFu << ((PIN_PWM_C - 8u) * 4u)));

    GPIOA->AFR[1] |=  ((1u << ((PIN_PWM_A - 8u) * 4u)) |
                       (1u << ((PIN_PWM_B - 8u) * 4u)) |
                       (1u << ((PIN_PWM_C - 8u) * 4u)));

    // -------------------------------------------------------------------------
    // Clean TIM1 config
    // -------------------------------------------------------------------------

    TIM1->CR1  = 0u;
    TIM1->CR2  = 0u;
    TIM1->SMCR = 0u;
    TIM1->DIER = 0u;
    TIM1->CCER = 0u;
    TIM1->BDTR = 0u;

    // -------------------------------------------------------------------------
    // TIM1 base setup
    //
    // Center-aligned mode 1 (CMS=01).
    // RCR = 1 gives one update interrupt per full PWM period.
    // CR2 MMS=111: routes OC4REF as TRGO for ADC injected trigger.
    // -------------------------------------------------------------------------

    TIM1->CR1 = TIM_CR1_CMS_0;
    TIM1->CR2 = (7u << TIM_CR2_MMS_Pos);
    TIM1->PSC = 0u;
    TIM1->ARR = PWM_ARR;
    TIM1->RCR = 1u;

    // -------------------------------------------------------------------------
    // TIM1 CH1/CH2/CH3 — PWM mode 1, preload enabled.
    // TIM1 CH4 — PWM mode 1, no GPIO, OC4REF drives TRGO for ADC.
    // -------------------------------------------------------------------------

    TIM1->CCMR1 =
        TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE |
        TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;

    TIM1->CCMR2 =
        TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE |
        TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

    // -------------------------------------------------------------------------
    // Initial phase duty: 50%
    // -------------------------------------------------------------------------

    TIM1->CCR1 = PWM_CENTER;
    TIM1->CCR2 = PWM_CENTER;
    TIM1->CCR3 = PWM_CENTER;

    // -------------------------------------------------------------------------
    // ADC trigger: up-count CCR4 crossing only (JEXTEN=falling, see
    // current_feedback_init()), one injected sequence per period.
    // -------------------------------------------------------------------------

    TIM1->CCR4 = PWM_ARR - PWM_SAMPLE_OFFSET;

    // -------------------------------------------------------------------------
    // Enable CH1/CH2/CH3 outputs and CH4 compare event.
    // CC4E must be set even though CH4 drives no GPIO pin.
    // -------------------------------------------------------------------------

    TIM1->CCER = TIM_CCER_CC1E |
                 TIM_CCER_CC2E |
                 TIM_CCER_CC3E |
                 TIM_CCER_CC4E;

    // -------------------------------------------------------------------------
    // Keep bridge outputs disabled until pwm_enable().
    // -------------------------------------------------------------------------

    TIM1->BDTR = 0u;

    // -------------------------------------------------------------------------
    // Force preload transfer, clear flags, enable TIM1 update interrupt.
    // -------------------------------------------------------------------------

    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR  = 0u;

    TIM1->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    TIM1->CR1 |= TIM_CR1_CEN;
}


// =============================================================================
// pwm_enable / pwm_disable
// =============================================================================

void pwm_enable(void)
{
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void pwm_disable(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}


// =============================================================================
// pwm_set_duty — write one phase CCR
// =============================================================================

void pwm_set_duty(uint8_t phase, uint16_t duty)
{
    if (duty > PWM_ARR)
        duty = PWM_ARR;

    switch (phase)
    {
        case 0: TIM1->CCR1 = duty; break;
        case 1: TIM1->CCR3 = duty; break;
        case 2: TIM1->CCR2 = duty; break;
        default: break;
    }
}


// =============================================================================
// volts_to_duty — centered phase voltage -> CCR count
//
// v range: [-V_BUS/2, +V_BUS/2]
// =============================================================================

uint16_t volts_to_duty(float v)
{
    float normalized = v / (V_BUS / 2.0f);

    if (normalized >  1.0f) normalized =  1.0f;
    if (normalized < -1.0f) normalized = -1.0f;

    float ccr_f = normalized * (float)PWM_CENTER + (float)PWM_CENTER;

    if (ccr_f < DUTY_MIN) ccr_f = DUTY_MIN;
    if (ccr_f > DUTY_MAX) ccr_f = DUTY_MAX;

    return (uint16_t)ccr_f;
}


// =============================================================================
// pwm_apply_phase_volts
// =============================================================================

void pwm_apply_phase_volts(float va, float vb, float vc)
{
    
    pwm_set_duty(0, volts_to_duty(va));
    pwm_set_duty(1, volts_to_duty(vb));
    pwm_set_duty(2, volts_to_duty(vc));
}


// =============================================================================
// pwm_apply_dq — inverse Park + inverse Clarke voltage output
//
// Convention: pwm_apply_dq(vd, vq, theta)
//
// Inverse Park:
//   v_alpha = vd*cos(θ) - vq*sin(θ)
//   v_beta  = vd*sin(θ) + vq*cos(θ)
//
// Inverse Clarke (amplitude-invariant):
//   va =  v_alpha
//   vb = -0.5*v_alpha + (√3/2)*v_beta
//   vc = -0.5*v_alpha - (√3/2)*v_beta
//
// Do not swap vd/vq. Do not change signs.
// =============================================================================

void pwm_apply_dq(float v_d, float v_q, float theta)
{
    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    float v_alpha = (v_d * cos_t) - (v_q * sin_t);
    float v_beta  = (v_d * sin_t) + (v_q * cos_t);

    float va =  v_alpha;
    float vb = -0.5f * v_alpha + 0.86602540378f * v_beta;
    float vc = -0.5f * v_alpha - 0.86602540378f * v_beta;

    pwm_apply_phase_volts(va, vb, vc);
}
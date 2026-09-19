// current_feedback.c — 3-phase current sense via DRV8353RS onboard shunt amps
//
// STM32F411/F446 bare metal
//
// Hardware:
//   PC0 / ADC1_IN10 -> ISENA
//   PC1 / ADC1_IN11 -> ISENB
//   PC2 / ADC1_IN12 -> ISENC
//
// Assumptions:
//   shunt resistors = 0.007 ohm
//   DRV8353 CSA gain = 10 V/V
//   ADC reference = 3.3 V
//   zero current is approximately VREF / 2
//
// Sampling mode:
//   ADC1 injected sequence triggered via TIM1_TRGO (JEXTSEL=1).
//   TIM1 CR2 MMS=111 routes OC4REF as TRGO.
//   CCR4 controls the sample point within the PWM cycle.
//
//   ADC_IRQHandler fires on JEOC and reads JDR1/JDR2/JDR3 immediately.
//   PC4 pulses high during the read so the scope shows the actual sample point.
//
//   current_feedback_update() is a no-op — ADC IRQ owns the data.
//
// Calibration:
//   current_feedback_calibrate() uses software-start (JSWSTART).
//   Hardware trigger is restored after calibration completes.
//
// Important:
//   Do NOT use ADC_SR_JEOC from the current project headers.
//   On STM32F4/F411, JEOC is bit 2 = 0x04.

#include "current_feedback.h"
#include "board_f411.h"
#include "clock.h"
#include "stm32f4xx.h"
#include "config.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>


// =============================================================================
// Current-sense scaling
// =============================================================================




#if defined(ADC_SR_JEOC)
#if ADC_SR_JEOC != (1u << 2)
#error "ADC_SR_JEOC is wrong for STM32F4/F411"
#endif
#endif

#define ADC_SR_JEOC_BIT   (1u << 2)


// =============================================================================
// ADC injected data
// =============================================================================

volatile uint16_t current_adc_raw[3] =
{
    2048u,
    2048u,
    2048u
};


// =============================================================================
// Calibration offsets
// =============================================================================

volatile float adc_offset[3] =
{
    ADC_ZERO,
    ADC_ZERO,
    ADC_ZERO
};


// =============================================================================
// Sample status / debug counters
// =============================================================================

static volatile bool     current_sample_valid  = false;
static volatile uint32_t current_sample_count  = 0u;
static volatile uint32_t current_missed_count  = 0u;


// =============================================================================
// ADC_IRQHandler
//
// Fires on JEOC — ADC injected conversion complete.
// Reads JDR1/JDR2/JDR3 immediately.
// PC4 pulses high during the read so the scope shows the actual sample point
// relative to the PWM waveform.
// =============================================================================

void ADC_IRQHandler(void)
{
    if (ADC1->SR & ADC_SR_JEOC_BIT)
    {
        GPIOC->BSRR = (1u << 4);            /* PC4 high — sample point */

        /* Recursive IIR blend y = (y + x)/2 -- fires ONCE per period (measured
         * 2026-09-19), so this is a 1-pole low-pass in the current feedback
         * path, not a 2-sample average. */
        current_adc_raw[0] = (uint16_t)((current_adc_raw[0] + ADC1->JDR1) / 2u);
        current_adc_raw[1] = (uint16_t)((current_adc_raw[1] + ADC1->JDR2) / 2u);
        current_adc_raw[2] = (uint16_t)((current_adc_raw[2] + ADC1->JDR3) / 2u);

        ADC1->SR &= ~ADC_SR_JEOC_BIT;

        current_sample_valid = true;
        current_sample_count++;

        GPIOC->BSRR = (1u << (4 + 16));     /* PC4 low */
    }
}


// =============================================================================
// current_feedback_sample_once
//
// Software-start one injected ADC sequence and read JDR1/JDR2/JDR3.
// Used only by current_feedback_calibrate().
// Temporarily disables JEXTEN and JEOCIE so JSWSTART owns the conversion.
// =============================================================================

static bool current_feedback_sample_once(void)
{
    uint32_t timeout = 10000u;

    ADC1->CR1 &= ~ADC_CR1_JEOCIE;   /* disable JEOC interrupt during sw-start */
    ADC1->CR2 &= ~ADC_CR2_JEXTEN;
    ADC1->SR  &= ~ADC_SR_JEOC_BIT;
    ADC1->CR2 |=  ADC_CR2_JSWSTART;

    while ((ADC1->SR & ADC_SR_JEOC_BIT) == 0u)
    {
        if (timeout-- == 0u)
        {
            current_missed_count++;
            current_sample_valid = false;
            return false;
        }
    }

    current_adc_raw[0] = ADC1->JDR1;
    current_adc_raw[1] = ADC1->JDR2;
    current_adc_raw[2] = ADC1->JDR3;

    ADC1->SR &= ~ADC_SR_JEOC_BIT;

    current_sample_count++;
    current_sample_valid = true;

    return true;
}


// =============================================================================
// current_feedback_init
// =============================================================================

void current_feedback_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;

    /* PC0/PC1/PC2 analog inputs */
    GPIOC->MODER |=  (3u << (PIN_CUR_A * 2u));
    GPIOC->MODER |=  (3u << (PIN_CUR_B * 2u));
    GPIOC->MODER |=  (3u << (PIN_CUR_C * 2u));

    GPIOC->PUPDR &= ~(3u << (PIN_CUR_A * 2u));
    GPIOC->PUPDR &= ~(3u << (PIN_CUR_B * 2u));
    GPIOC->PUPDR &= ~(3u << (PIN_CUR_C * 2u));

    /* PC4 debug pulse — shows ADC sample point on scope */
    GPIOC->MODER &= ~(3u << (4 * 2));
    GPIOC->MODER |=  (1u << (4 * 2));    /* output */
    GPIOC->BSRR   =  (1u << (4 + 16));   /* idle low */

    ADC->CCR  = ADC_CCR_ADCPRE_0;

    ADC1->CR1 = 0u;
    ADC1->CR2 = 0u;
    ADC1->SR  = 0u;

    ADC1->CR1 |= ADC_CR1_SCAN;

    ADC1->SMPR1 &= ~((7u << 0) | (7u << 3) | (7u << 6));
    ADC1->SMPR1 |=  ((ADC_SMP_15_CYCLES << 0) |
                     (ADC_SMP_15_CYCLES << 3) |
                     (ADC_SMP_15_CYCLES << 6));



    ADC1->JSQR =
        (2u  << ADC_JSQR_JL_Pos)   |
        (10u << ADC_JSQR_JSQ2_Pos) |
        (11u << ADC_JSQR_JSQ3_Pos) |
        (12u << ADC_JSQR_JSQ4_Pos);

    /*
     * Hardware trigger: TIM1_TRGO, falling edge only.
     *
     * Per RM0383 Table 43:
     *   JEXTSEL = 0001 (1) = TIM1_TRGO
     *   JEXTEN  = 10   (2) = falling edge
     *
     * TIM1 CR2 MMS=111 routes OC4REF as TRGO. PWM mode 1: OC4REF is high
     * while CNT < CCR4, so the up-count crossing is a FALLING edge. The
     * 3-channel sequence (SMPR=15 cycles -> 3 x 1.08 us) starts there and is
     * centered on the counter peak by PWM_SAMPLE_OFFSET (pwm.c). The
     * down-count edge must not trigger: it would start a second, late
     * sequence after the peak.
     */
    ADC1->CR2 &= ~(ADC_CR2_JEXTSEL | ADC_CR2_JEXTEN);
    ADC1->CR2 |=  ((1u << ADC_CR2_JEXTSEL_Pos) |
                   (2u << ADC_CR2_JEXTEN_Pos));

    /* Enable JEOC interrupt — ADC_IRQHandler reads results */
    ADC1->CR1 |= ADC_CR1_JEOCIE;

    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->SR   = 0u;

    NVIC_SetPriority(ADC_IRQn, 1);
    NVIC_EnableIRQ(ADC_IRQn);

    current_adc_raw[0] = 2048u;
    current_adc_raw[1] = 2048u;
    current_adc_raw[2] = 2048u;

    adc_offset[0] = ADC_ZERO;
    adc_offset[1] = ADC_ZERO;
    adc_offset[2] = ADC_ZERO;

    current_sample_valid = false;
    current_sample_count = 0u;
    current_missed_count = 0u;
}


// =============================================================================
// current_feedback_calibrate
// =============================================================================

void current_feedback_calibrate(void)
{
    const uint32_t N = 512u;

    uint32_t sum_a = 0u;
    uint32_t sum_b = 0u;
    uint32_t sum_c = 0u;
    uint32_t good  = 0u;

    for (volatile uint32_t d = 0u; d < 200000u; d++) { __NOP(); }

    for (uint32_t i = 0u; i < 32u; i++)
    {
        (void)current_feedback_sample_once();
    }

    for (uint32_t i = 0u; i < N; i++)
    {
        if (current_feedback_sample_once())
        {
            sum_a += current_adc_raw[0];
            sum_b += current_adc_raw[1];
            sum_c += current_adc_raw[2];
            good++;
        }
    }

    if (good > 0u)
    {
        adc_offset[0] = (float)sum_a / (float)good;
        adc_offset[1] = (float)sum_b / (float)good;
        adc_offset[2] = (float)sum_c / (float)good;
    }
    else
    {
        adc_offset[0] = ADC_ZERO;
        adc_offset[1] = ADC_ZERO;
        adc_offset[2] = ADC_ZERO;
    }

    /*
     * Re-enable hardware trigger and JEOC interrupt after calibration.
     * Falling edge only — see current_feedback_init() for why.
     */
    ADC1->CR2 &= ~(ADC_CR2_JEXTSEL | ADC_CR2_JEXTEN);
    ADC1->CR2 |=  ((1u << ADC_CR2_JEXTSEL_Pos) |
                   (2u << ADC_CR2_JEXTEN_Pos));

    ADC1->SR = 0u;

    ADC1->CR2 &= ~ADC_CR2_ADON;
    for (volatile uint32_t d = 0u; d < 1000u; d++) { __NOP(); }
    ADC1->CR2 |=  ADC_CR2_ADON;
    for (volatile uint32_t d = 0u; d < 1000u; d++) { __NOP(); }
    ADC1->SR   = 0u;

    ADC1->CR1 |= ADC_CR1_JEOCIE;   /* re-enable JEOC interrupt */

    current_sample_valid = false;
    current_sample_count = 0u;
    current_missed_count = 0u;
}


// =============================================================================
// current_feedback_update
//
// No-op — ADC_IRQHandler owns data read.
// Called from TIM1 ISR for compatibility; does nothing.
// =============================================================================

void current_feedback_update(void)
{
    /* ADC_IRQHandler owns this — no polling needed */
}


// =============================================================================
// current_feedback_sample_valid
// =============================================================================

bool current_feedback_sample_valid(void)
{
    return current_sample_valid;
}


// =============================================================================
// current_feedback_sample_count
// =============================================================================

uint32_t current_feedback_sample_count(void)
{
    return current_sample_count;
}


// =============================================================================
// current_feedback_missed_count
// =============================================================================

uint32_t current_feedback_missed_count(void)
{
    return current_missed_count;
}


// =============================================================================
// current_feedback_get_phase_amps
// =============================================================================

void current_feedback_get_phase_amps(float *ia, float *ib, float *ic)
{
    float a = ((float)current_adc_raw[0] - adc_offset[0]) * AMPS_PER_COUNT;
    float b = ((float)current_adc_raw[1] - adc_offset[1]) * AMPS_PER_COUNT;
    float c = ((float)current_adc_raw[2] - adc_offset[2]) * AMPS_PER_COUNT;


    if (ia != 0) { *ia = a; }
    if (ib != 0) { *ib = b; }
    if (ic != 0) { *ic = c; }
}


// =============================================================================
// current_feedback_get_dq
// =============================================================================

void current_feedback_get_dq(float theta, float *i_d, float *i_q)
{
    float ia = 0.0f;
    float ib = 0.0f;

    current_feedback_get_phase_amps(&ia, &ib, 0);

    float i_alpha = ia;
    float i_beta  = (ia + (2.0f * ib)) * 0.57735026919f;

    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    if (i_d != 0) { *i_d = ( i_alpha * cos_t) + (i_beta * sin_t); }
    if (i_q != 0) { *i_q = (-i_alpha * sin_t) + (i_beta * cos_t); }
}


// =============================================================================
// Offset accessors
// =============================================================================

float current_feedback_get_offset_a(void) { return adc_offset[0]; }
float current_feedback_get_offset_b(void) { return adc_offset[1]; }
float current_feedback_get_offset_c(void) { return adc_offset[2]; }
#include "isr_timing.h"

volatile uint32_t isr_cycles_last[ISR_T_COUNT];
volatile uint32_t isr_cycles_max[ISR_T_COUNT];
volatile uint32_t isr_calls[ISR_T_COUNT];

void isr_timing_init(void)
{
    for (uint32_t i = 0u; i < ISR_T_COUNT; i++)
    {
        isr_cycles_last[i] = 0u;
        isr_cycles_max[i]  = 0u;
        isr_calls[i]       = 0u;
    }

    /* DWT is behind the trace enable; CYCCNT is free-running once armed and
     * is never reset again, so the subtraction in ISR_TIME_END is allowed to
     * wrap (unsigned, 43s at 100MHz -- longer than any single ISR). */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0u;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    /* PC4 / PC5 / PC7 / PC9 as push-pull outputs, idle low. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    const uint32_t pins[4] = { ISR_PIN_ADC, ISR_PIN_TIM1, ISR_PIN_DMA3, ISR_PIN_EXTI };

    for (uint32_t i = 0u; i < 4u; i++)
    {
        GPIOC->MODER   &= ~(3u << (pins[i] * 2u));
        GPIOC->MODER   |=  (1u << (pins[i] * 2u));   /* output */
        /* Low speed deliberately. These pulses are microseconds wide, so
         * slew rate buys nothing, and fast edges ring on a scope probe's
         * ground lead hard enough to double-trigger and read as two pulses
         * where there is one. */
        GPIOC->OSPEEDR &= ~(3u << (pins[i] * 2u));
        GPIOC->BSRR     =  (1u << (pins[i] + 16u));  /* idle low */
    }

}

#include "isr_timing.h"

volatile uint32_t isr_cycles_last[ISR_T_COUNT];
volatile uint32_t isr_cycles_max[ISR_T_COUNT];
volatile uint32_t isr_calls[ISR_T_COUNT];

static void cycle_counter_init(void)
{
    // DWT->CYCCNT counts CPU cycles (100 per us). Wraps every 43 s;
    // the unsigned subtraction in ISR_TIME_END still works across a wrap.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0u;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

// Push-pull output, low speed (fast edges ring on the scope probe), idle low.
static void marker_pin_init(uint32_t pin)
{
    GPIOC->MODER   = (GPIOC->MODER & ~(3u << (pin * 2u))) | (1u << (pin * 2u));
    GPIOC->OSPEEDR &= ~(3u << (pin * 2u));
    GPIOC->BSRR     = 1u << (pin + 16u);
}

void isr_timing_init(void)
{
    for (uint32_t i = 0u; i < ISR_T_COUNT; i++)
    {
        isr_cycles_last[i] = 0u;
        isr_cycles_max[i]  = 0u;
        isr_calls[i]       = 0u;
    }

    cycle_counter_init();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    marker_pin_init(ISR_PIN_TIM1);   // PC5
    marker_pin_init(ISR_PIN_ADC);    // PC4
    marker_pin_init(ISR_PIN_EXTI);   // PC9
}

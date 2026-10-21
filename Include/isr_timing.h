#ifndef ISR_TIMING_H
#define ISR_TIMING_H

#include "stm32f4xx.h"
#include <stdint.h>

// ISR execution timing -- two instruments at once, because they answer
// different questions:
//
//   DWT->CYCCNT  -- worst case, in cycles, accumulated over a whole run.
//                   A scope only ever shows you the pulses that happened to
//                   be on screen; isr_cycles_max[] catches the one bad tick
//                   in a 40s chirp. Read the arrays in the debugger.
//   GPIO marker  -- shows the *interleaving*: TIM1_UP preempting ADC, the
//                   SPI ISR being pushed out, tail-chaining. Cycle counts
//                   can't show that, and at 10kHz telemetry it's the thing
//                   that actually decides whether the budget holds.
//
// Budget: the control loop runs at 20kHz, so all three ISRs together have
// 50us (5000 cycles @ 100MHz) per tick. CPU load = sum of rate x duration.
//
// Markers are on GPIOC, set up in isr_timing_init().

#define ISR_T_TIM1      0u
#define ISR_T_ADC       1u
#define ISR_T_EXTI      2u
#define ISR_T_COUNT     3u

#define ISR_PIN_TIM1    5u      /* PC5 */
#define ISR_PIN_ADC     4u      /* PC4 */
#define ISR_PIN_EXTI    9u      /* PC9 -- NOT PC8: that is PIN_DRV_CS, the
                                 * DRV8353 SPI1 chip select (board_f411.h) */

extern volatile uint32_t isr_cycles_last[ISR_T_COUNT];
extern volatile uint32_t isr_cycles_max[ISR_T_COUNT];
extern volatile uint32_t isr_calls[ISR_T_COUNT];

// Enables the DWT cycle counter, the GPIOC clock, and the marker pins as
// outputs. Call once at startup.
void isr_timing_init(void);

// Marker goes high before the timestamp so the BSRR write isn't charged to
// the measurement; the closing timestamp is taken before the marker drops,
// for the same reason.
#define ISR_TIME_BEGIN(idx, pin)                                    \
    GPIOC->BSRR = (1u << (pin));                                    \
    const uint32_t _isr_t0 = DWT->CYCCNT

#define ISR_TIME_END(idx, pin)                                      \
    do {                                                            \
        const uint32_t _d = DWT->CYCCNT - _isr_t0;                  \
        isr_cycles_last[(idx)] = _d;                                \
        if (_d > isr_cycles_max[(idx)]) isr_cycles_max[(idx)] = _d; \
        isr_calls[(idx)]++;                                         \
        GPIOC->BSRR = (1u << ((pin) + 16u));                        \
    } while (0)



#endif /* ISR_TIMING_H */

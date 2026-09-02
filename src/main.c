#include "stm32f4xx.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "clock.h"
#include "spi.h"
#include "tim1.h"
#include "drive.h"
#include "ringBuffer.h"
#include "loops.h"
#include "protocol.h"
#include "config.h"
#include "encoder.h"
#include "drv8353.h"
#include "board_f411.h"
#include "pwm.h"
#include "current_feedback.h"
#include "foc_sysid.h"
#include "foc_trajectory.h"

volatile uint32_t tick_ms          = 0;
volatile bool     system_initialized = false;

// Captured first thing in main(), before anything else touches RCC.
// RCC->CSR reset-cause flags (PORRSTF/BORRSTF/PINRSTF/SFTRSTF/IWDGRSTF/
// WWDGRSTF/LPWRRSTF, top byte) are latched and persist across non-POR
// resets until software explicitly clears them (RMVF) -- deliberately
// never cleared here, so this accumulates the OR of every reset cause
// since the last real power-cycle. Wired into ALIGN-stage telemetry
// (foc_sysid.c) to check whether CL_VEL_CHIRP/VEL_CHIRP instability events
// are actually browning out or otherwise resetting the MCU, without
// relying on the debugger (register-view has been unreliable this session).
volatile uint32_t g_reset_cause    = 0;

void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM1->SR = ~TIM_SR_UIF;

    if (!system_initialized)
        return;

    encoder_update(tick_ms);

#if RUN_MODE == RUN_MODE_SYSID
    foc_sysid_step();
#else
    foc_trajectory_step();
#endif
}

int main(void)
{
    system_initialized = false;

    g_reset_cause = RCC->CSR;   // capture before anything else can touch RCC

    // Enable full access to FPU coprocessors CP10 and CP11. */
    SCB->CPACR |= ((3UL << (10u * 2u)) | (3UL << (11u * 2u)));

    // Barrier required before any FPU instruction (like __set_FPSCR below) --
    // without it the pipeline can still see the FPU as disabled and fault.
    __DSB();
    __ISB();

    // Flush-to-zero -- the hardware FPU takes a much slower software-assisted
    // path on denormal (subnormal) operands, sometimes 10-20x a normal
    // single-cycle op. Suspected cause of the VEL_CHIRP ISR slowdown (chirp
    // phase math accumulates many small floating-point values over a long
    // sweep). FZ has no cost here -- sub-normal precision is irrelevant for
    // control math.
    __set_FPSCR(__get_FPSCR() | (1u << 24));

    // DWT cycle counter -- passive per-tick timing diagnostics with no
    // debugger attached (SWD halts/polling would perturb the exact timing
    // we're trying to measure).
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    clock_init();
    encoder_init();
    drive_init();
#if SPI_TELEM_ENABLED
    spi_init();
#endif
    ring_init();

    // GPIOC clock -- normally enabled by spi_init(), which also owns PC13/PC4.
    // PC3 below (the GPIO start-trigger wait) needs it regardless of whether
    // SPI telemetry is compiled in; current_feedback_init() enables it too,
    // but not until after this wait loop, so it can't be relied on here.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    GPIOC->MODER &= ~(3u << (3u * 2u));
    GPIOC->PUPDR &= ~(3u << (3u * 2u));
    GPIOC->PUPDR |=  (1u << (3u * 2u));

    while (!(GPIOC->IDR & (1u << 3u))) {}
    while (GPIOC->IDR & (1u << 3u)) {}

    drv8353_init();
    drv8353_configure();

#if RUN_MODE == RUN_MODE_SYSID
    volatile uint16_t csa_ctrl = drv8353_read_reg(DRV8353_REG_CSA_CONTROL);
    csa_ctrl = drv8353_read_reg(DRV8353_REG_CSA_CONTROL);
    (void)csa_ctrl;
#endif

    pwm_init();
    current_feedback_init();
    drv_enable_high();
    current_feedback_calibrate();
    pwm_enable();

    SysTick_Config(SystemCoreClock / 1000u);

    system_initialized = true;

    while (1)
    {
        __WFI();
    }
}
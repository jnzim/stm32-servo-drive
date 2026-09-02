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

// =============================================================================
// SysTick_Handler — 1 kHz. Drive state machine + position tick --
// deliberately NOT in the 20kHz TIM1 ISR, per the architecture already
// documented across drive.c and foc_trajectory.h. Priority set explicitly
// below so this can never preempt the real-time control loop.
//
// foc_trajectory_position_tick() is called every tick, not just while
// servo is on: it only actually runs the generator/position loop when
// servo-on, but it always reports telemetry -- calling it conditionally
// meant the exact tick that flips drive_state to IDLE (inside
// drive_sm_run(), just above) never got a telemetry report for that new
// state, so the Pi never saw completion.
// =============================================================================
void SysTick_Handler(void)
{
    tick_ms++;

    if (!system_initialized)
        return;

    drive_sm_run();
    foc_trajectory_position_tick();
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

    // Strictly lower priority (higher number) than the 20kHz control loop
    // (TIM1_UP_TIM10_IRQn=0, pwm.c) and everything else (ADC=1, DMA1_Stream3/
    // EXTI15_10=2) -- SysTick must never preempt real-time control. 15 is
    // what ringBuffer.c's own comment already anticipates for this.
    NVIC_SetPriority(SysTick_IRQn, 15);

    system_initialized = true;

#if RUN_MODE == RUN_MODE_CLOSED_LOOP
    // Bench-test scaffolding (trajectory-streaming rebuild): force SERVO_ON
    // and request one hardcoded move, no SPI/Pi involved, so the SysTick +
    // position-loop + trapezoidal-generator path can be verified in
    // isolation before the real SPI MOVE_CMD path is wired up. Delete this
    // whole block once that real path exists. expected_samples set to a
    // placeholder large enough that drive.c's drain-check (SERVO_ON exit)
    // doesn't trip on the first tick against an empty, unused ring.
    expected_samples = 3000u;   // 3s @ 1kHz
    foc_trajectory_start_move(3900, 3900, 13000);  // ~3.0 rad, ~3.0 rad/s, ~10.0 rad/s^2
    drive_request_servo_on();
#endif

    while (1)
    {
        __WFI();
    }
}
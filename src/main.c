#include "stm32f4xx.h"

#include <stdbool.h>
#include <stdint.h>

#include "clock.h"
#include "spi.h"
#include "drive.h"
#include "ringBuffer.h"
#include "config.h"
#include "encoder.h"
#include "drv8353.h"
#include "board_f411.h"
#include "pwm.h"
#include "current_feedback.h"
#include "foc_sysid.h"
#include "foc_trajectory.h"
#include "isr_timing.h"
#include "observer.h"

volatile uint32_t tick_ms          = 0;

// 20kHz control-tick counter, stamped into every telemetry frame as the
// sample time (50us units). Wraps after ~59.6 hours.
volatile uint32_t g_control_tick   = 0;
volatile bool     system_initialized = false;

// RCC->CSR reset-cause flags, captured before anything touches RCC. Never
// cleared (RMVF), so this is the OR of every reset cause since power-up.
// Reported in sysid ALIGN-stage telemetry (foc_sysid.c).
volatile uint32_t g_reset_cause    = 0;

// DRV8353 fault registers, read once right after drv_enable_high(), before
// PWM runs. Reported in sysid ALIGN-stage telemetry (foc_sysid.c).
volatile uint16_t g_drv_fault_status_1 = 0;
volatile uint16_t g_drv_vgs_status_2   = 0;
volatile bool     g_drv_nfault_pin_ok  = false;

void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM1->SR = ~TIM_SR_UIF;

    if (!system_initialized)
        return;

    ISR_TIME_BEGIN(ISR_T_TIM1, ISR_PIN_TIM1);

    g_control_tick++;

    encoder_update(tick_ms);

#if RUN_MODE == RUN_MODE_SYSID
    foc_sysid_step();
#else
    foc_trajectory_step();
#endif

    ISR_TIME_END(ISR_T_TIM1, ISR_PIN_TIM1);
}

// =============================================================================
// SysTick_Handler — 1 kHz, lowest priority. Drive state machine and position
// tick. The position tick runs every tick, not only while servo-on, because
// it always sends telemetry; that is how the Pi sees the drive return to IDLE.
// =============================================================================
void SysTick_Handler(void)
{
    tick_ms++;

    if (!system_initialized)
        return;

    // Trajectory mode only; in sysid, foc_sysid_step() owns the motor.
#if RUN_MODE != RUN_MODE_SYSID
    drive_sm_run();
    foc_trajectory_position_tick();
#endif
}

int main(void)
{
    system_initialized = false;

    g_reset_cause = RCC->CSR;   // capture before anything else can touch RCC

    // Enable full access to FPU coprocessors CP10 and CP11.
    SCB->CPACR |= ((3UL << (10u * 2u)) | (3UL << (11u * 2u)));

    // Barrier required before any FPU instruction (like __set_FPSCR below) --
    // without it the pipeline can still see the FPU as disabled and fault.
    __DSB();
    __ISB();

    // Flush-to-zero: denormal operands are slow on the FPU, and their
    // precision does not matter for control math.
    __set_FPSCR(__get_FPSCR() | (1u << 24));

    clock_init();
    encoder_init();
    drive_init();
#if SPI_TELEM_ENABLED
    spi_init();
#endif
    ring_init();

    // DWT cycle counter, GPIOC clock, and the ISR marker pins.
    isr_timing_init();

    // Wait for the Pi's start signal on PC3 (input, pull-up): wait for high,
    // then for the Pi to pull it low. Nothing past here runs until then.
    GPIOC->MODER &= ~(3u << (PIN_PI_START * 2u));
    GPIOC->PUPDR &= ~(3u << (PIN_PI_START * 2u));
    GPIOC->PUPDR |=  (1u << (PIN_PI_START * 2u));

    while (!(GPIOC->IDR & (1u << PIN_PI_START))) {}
    while (GPIOC->IDR & (1u << PIN_PI_START)) {}

    drv8353_init();
    drv8353_configure();

    pwm_init();
    current_feedback_init();
    drv_enable_high();

    {
        Drv8353Status drv_status = drv8353_read_status();
        g_drv_fault_status_1 = drv_status.fault_status_1;
        g_drv_vgs_status_2   = drv_status.vgs_status_2;
        g_drv_nfault_pin_ok  = drv_status.n_fault_pin;
    }

    observer_init();
    current_feedback_calibrate();
    pwm_enable();

    SysTick_Config(SystemCoreClock / 1000u);

    // Lowest priority: TIM1 update is 0, ADC 1, SPI DMA/EXTI 2.
    NVIC_SetPriority(SysTick_IRQn, 15);

    system_initialized = true;

    while (1)
    {
        __WFI();
    }
}
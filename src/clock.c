// clock.c — HSI → PLL → 100MHz for STM32F411RE
// APB1 = 50MHz (÷2), APB2 = 100MHz (÷1)

#include "clock.h"
#include "stm32f4xx.h"

void clock_init(void)
{
    // ── Enable HSI ────────────────────────────────────────────────────────────
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}

    // ── Make sure PLL is off before reconfiguring ─────────────────────────────
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {}

    // ── Configure PLL: HSI(16MHz) / M=8 * N=100 / P=2 = 100MHz ───────────────
    RCC->PLLCFGR = (8u   << RCC_PLLCFGR_PLLM_Pos) |   // M=8   → VCO input = 2MHz
                   (100u << RCC_PLLCFGR_PLLN_Pos) |   // N=100 → VCO = 200MHz
                   (0u   << RCC_PLLCFGR_PLLP_Pos) |   // P=2   → SYSCLK = 100MHz
                   RCC_PLLCFGR_PLLSRC_HSI;

    // ── Flash latency for 100MHz, enable caches/prefetch ─────────────────────
    FLASH->ACR = FLASH_ACR_LATENCY_3WS |
                 FLASH_ACR_PRFTEN      |
                 FLASH_ACR_ICEN        |
                 FLASH_ACR_DCEN;

    // ── AHB/APB prescalers:
    //    AHB  = SYSCLK / 1 = 100MHz
    //    APB1 = HCLK   / 2 = 50MHz
    //    APB2 = HCLK   / 1 = 100MHz
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    // ── Enable PLL ────────────────────────────────────────────────────────────
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    // ── Switch SYSCLK to PLL ──────────────────────────────────────────────────
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |=  RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

    // ── Update CMSIS core clock variable ──────────────────────────────────────
    SystemCoreClock = 100000000u;
}

// board_f411.h — STM32F411RE project pin map
//
//   RPi SPI2:
//     RPi CS   -> PB12 / SPI2_NSS
//     RPi SCLK -> PB13 / SPI2_SCK
//     RPi MISO <- PB14 / SPI2_MISO
//     RPi MOSI -> PB15 / SPI2_MOSI
//
//   Encoder:
//     MAX3096 A -> PA0 / TIM2_CH1
//     MAX3096 B -> PA1 / TIM2_CH2
//
//   DRV8353 SPI1:
//     PA5 / SPI1_SCK  -> DRV SCLK
//     PA6 / SPI1_MISO <- DRV SDO
//     PA7 / SPI1_MOSI -> DRV SDI
//     PC8 / GPIO      -> DRV nSCS   (CN10 pin 2)
//
//     NOTE: DRV8353 SDO is open-drain.
//           Use 4.7k or 10k pull-up from SDO/MISO to 3.3 V.
//
//   DRV8353 control/status:
//     PB0 -> DRV ENABLE
//     PB1 <- DRV nFAULT
//
//     ENABLE low  = DRV disabled
//     ENABLE high = DRV enabled / awake
//     nFAULT high = no fault
//     nFAULT low  = fault active
//
//   Future PWM TIM1:
//     PA8  / TIM1_CH1 -> DRV PWM_A / INHA
//     PA9  / TIM1_CH2 -> DRV PWM_B / INHB
//     PA10 / TIM1_CH3 -> DRV PWM_C / INHC

#pragma once

// Profile refill request
 #define READY_RING_REFILL   13u 

// Rpi SPR2 slave
#define PIN_RPI_NSS      12  // PB12
#define PIN_RPI_SCK      13  // PB13
#define PIN_RPI_MISO     14  // PB14
#define PIN_RPI_MOSI     15  // PB15

// Encoder TIM2
#define PIN_ENC_A        0   // PA0 / TIM2_CH1
#define PIN_ENC_B        1   // PA1 / TIM2_CH2

// DRV SPI1
#define PIN_DRV_SCK      5   // PA5
#define PIN_DRV_MISO     6   // PA6
#define PIN_DRV_MOSI     7   // PA7
#define PIN_DRV_CS       8   // PC8  

// PWM TIM1, 3-PWM mode
#define PIN_PWM_A        8   // PA8  / TIM1_CH1
#define PIN_PWM_B        9   // PA9  / TIM1_CH2
#define PIN_PWM_C        10  // PA10 / TIM1_CH3

// Current feedback ADC
#define PIN_CUR_A        0   // PC0 / ADC_IN10
#define PIN_CUR_B        1   // PC1 / ADC_IN11
#define PIN_CUR_C        2   // PC2 / ADC_IN12

// DRV control/status
#define PIN_DRV_ENABLE   0   // PB0
#define PIN_DRV_NFAULT   1   // PB1

// RPi refill handshake
#define PIN_STM_READY    3   // PC3
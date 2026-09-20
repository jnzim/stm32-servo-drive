#include "encoder.h"
#include "stm32f4xx.h"

// Hardware mapping
#define ENC_TIM              TIM2
#define ENC_TIM_CLK_EN()     (RCC->APB1ENR |= RCC_APB1ENR_TIM2EN)

#define ENC_GPIO             GPIOA
#define ENC_GPIO_CLK_EN()    (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)

#define ENC_CH1_PIN          0u      // PA0 = TIM2_CH1
#define ENC_CH2_PIN          1u      // PA1 = TIM2_CH2
#define ENC_AF               1u      // AF1 = TIM2 on PA0/PA1

#define ENC_SAMPLE_HZ        20000   // encoder_update() called from 20 kHz TIM1 ISR

// defined here, declared extern in encoder.h
volatile EncoderState encoder = {0, 0, 0};

// previous position saved each velocity calculation
static int32_t last_position = 0;


// ---------------------------------------------------------------------------
// Velocity filter config
// ---------------------------------------------------------------------------
// Was 80, cut to 20 earlier tonight to fix a phase-margin problem in the
// CLOSED velocity loop chirp (traced to this filter's group delay, not a
// mechanical resonance -- see git history). That fix is still valid, but
// N=20 was designed around a since-abandoned aggressive position loop
// (Kp_pos=954.5, gc~70Hz) that needed the margin. With the position loop
// back down to Kp_pos=200 (gc~15-20Hz) after SYSID_TEST_POSITION_STEP
// showed continuous current/vq buzz at rest that persisted through BOTH a
// current-loop gain revert AND the position-loop gain revert -- pointing
// at this filter's noise (not either loop's gain) as the actual source --
// there's now room to trade back some of the unneeded phase margin for a
// quieter vel_meas.
//
// N=40 (2ms window, ~28 deg phase lag at 80Hz -- between N=20's ~13.7 and
// the old N=80's ~58) is the compromise being tried. Re-check against
// SYSID_TEST_CL_VEL_CHIRP (phase crossover/GM shouldn't collapse back
// toward the original problem) and SYSID_TEST_POSITION_STEP (does the
// buzz actually drop) before locking in.
// 2026-09-19: back to N=20. With the load attached and Kp_pos=260 (gc 32.5Hz,
// PM 47.4deg), the phase this filter costs matters again. Backing the plant out
// of the measured loop (P = L/C) showed phase passing -90deg -- -88.7 at 50Hz,
// -136.6 at 120Hz -- which a single-pole plant cannot do; the 2ms window's 1ms
// group delay (0.36 deg/Hz, ~14deg at the 40Hz velocity crossover) accounts for
// most of it, and for the ~1dB of extra magnitude droop above 10Hz.
//
// The noise trade that drove 40 is real: re-check SYSID_TEST_POSITION_STEP for
// the buzz-at-rest that N=20 caused last time, now that the loop gains differ.
#define VEL_FILTER_N  20   // 1 ms window
                           // resolution: 1 count/ms = 0.767 rad/s
                           // at 10 rad/s -> approximately 13 counts/ms

static int32_t  delta_buf[VEL_FILTER_N] = {0};
static uint32_t vel_buf_idx             = 0;
static int32_t  delta_sum               = 0;


// =============================================================================
// encoder_init
// Call once at startup after system clock setup.
// =============================================================================
void encoder_init(void)
{
    // -------------------------------------------------------------------------
    // STEP 1 — enable clocks
    // -------------------------------------------------------------------------
    ENC_GPIO_CLK_EN();    // GPIOA clock — PA0, PA1
    ENC_TIM_CLK_EN();     // TIM2 clock — APB1 bus

    // -------------------------------------------------------------------------
    // STEP 2 — configure PA0, PA1 as alternate function AF1 = TIM2_CH1/TIM2_CH2
    // -------------------------------------------------------------------------

    // MODER: alternate function mode = 10
    ENC_GPIO->MODER &= ~((3u << (ENC_CH1_PIN * 2u)) |
                         (3u << (ENC_CH2_PIN * 2u)));

    ENC_GPIO->MODER |=  ((2u << (ENC_CH1_PIN * 2u)) |
                         (2u << (ENC_CH2_PIN * 2u)));

    // AFR[0]: PA0/PA1 alternate function.
    // AF1 = TIM2.
    ENC_GPIO->AFR[0] &= ~((0xFu << (ENC_CH1_PIN * 4u)) |
                          (0xFu << (ENC_CH2_PIN * 4u)));

    ENC_GPIO->AFR[0] |=  ((ENC_AF << (ENC_CH1_PIN * 4u)) |
                          (ENC_AF << (ENC_CH2_PIN * 4u)));

    // No internal pullups/pulldowns.
    // MAX3096 actively drives Y1/Y2.
    ENC_GPIO->PUPDR &= ~((3u << (ENC_CH1_PIN * 2u)) |
                         (3u << (ENC_CH2_PIN * 2u)));

    // -------------------------------------------------------------------------
    // STEP 3 — configure TIM2 in encoder mode
    //
    // Encoder mode uses TIM2_CH1 and TIM2_CH2 as quadrature A/B inputs.
    //
    // SMCR.SMS:
    //   001 = encoder mode 1 — counts on TI1 edges only
    //   010 = encoder mode 2 — counts on TI2 edges only
    //   011 = encoder mode 3 — counts on both TI1 and TI2 edges, 4x resolution
    //
    // CCMR1:
    //   CC1S = 01 — CH1 input mapped to TI1
    //   CC2S = 01 — CH2 input mapped to TI2
    //   IC1F/IC2F = 0 — no digital filter initially
    // -------------------------------------------------------------------------

    ENC_TIM->CR1   = 0;
    ENC_TIM->CR2   = 0;
    ENC_TIM->SMCR  = 0;
    ENC_TIM->CCMR1 = 0;
    ENC_TIM->CCER  = 0;
    ENC_TIM->PSC   = 0;

    // CH1 and CH2 as inputs
    ENC_TIM->CCMR1 = (1u << 0)     // CC1S = 01 — input TI1
                   | (1u << 8);    // CC2S = 01 — input TI2

    // Encoder mode 3: count on both TI1 and TI2 edges
    ENC_TIM->SMCR = (3u << 0);     // SMS = 011

    // Non-inverted polarity on both channels
    ENC_TIM->CCER = 0;

    // TIM2 is 32-bit
    ENC_TIM->ARR = 0xFFFFFFFFu;
    ENC_TIM->CNT = 0;

    last_position = 0;
    encoder.position = 0;
    encoder.velocity = 0;
    encoder.timestamp_ms = 0;

    // Start counter
    ENC_TIM->CR1 |= TIM_CR1_CEN;
}


// =============================================================================
// encoder_get_position
// Returns current TIM2 counter as signed 32-bit encoder counts.
// Positive/negative direction depends on A/B wiring.
// =============================================================================
int32_t encoder_get_position(void)
{
    return (int32_t)ENC_TIM->CNT;
}


// =============================================================================
// encoder_get_velocity
// Returns last computed dx/dt in counts/sec.
// Updated by encoder_update(), currently called from the 20 kHz TIM1 ISR.
// =============================================================================
int32_t encoder_get_velocity(void)
{
    return encoder.velocity;
}


// =============================================================================
// encoder_zero
// Reset encoder position and velocity state.
// =============================================================================
void encoder_zero(void)
{
    ENC_TIM->CNT = 0;
    last_position = 0;

    encoder.position = 0;
    encoder.velocity = 0;
}

// ---------------------------------------------------------------------------
// encoder_update — call from TIM1 ISR at 20 kHz
// ---------------------------------------------------------------------------
void encoder_update(uint32_t timestamp_ms)
{
    int32_t current = (int32_t)ENC_TIM->CNT;
    int32_t delta   = current - last_position;

    encoder.position     = current;
    encoder.timestamp_ms = timestamp_ms;

    // Rolling moving average over VEL_FILTER_N samples
    delta_sum                -= delta_buf[vel_buf_idx];
    delta_buf[vel_buf_idx]    = delta;
    delta_sum                += delta;
    vel_buf_idx               = (vel_buf_idx + 1) % VEL_FILTER_N;

    // velocity [counts/s] — averaged delta scaled to per-second
    encoder.velocity = (delta_sum * ENC_SAMPLE_HZ) / VEL_FILTER_N;

    last_position = current;
}


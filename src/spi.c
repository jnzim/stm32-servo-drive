// spi.c — SPI2 slave link to the Pi: one 32-byte full-duplex frame per
// transaction, telemetry out (DMA1 Stream4), MoveCmd in (DMA1 Stream3).
//
// CS frames every transaction. On the CS rising edge (EXTI12) the handler
// parses the frame just received, then re-arms the link for the next one:
// SPI2 is reset (the only way to drop the byte it already pulled into its
// TX buffer), the latest telemetry is copied into the TX buffer, and both
// DMAs restart at byte 0. So every transaction starts on a frame boundary,
// and a frame is never rewritten while it is being sent.
//
// The control loop never touches the TX buffer: it publishes into a
// ping-pong staging pair, and the CS handler copies the latest one.
//
// The re-arm has to finish before the Pi starts the next transaction, so
// the Pi holds CS high for at least SPI2_MIN_CS_GAP_US (protocol.h).

#include "spi.h"
#include "isr_timing.h"
#include "board_f411.h"
#include "protocol.h"
#include "config.h"
#include "stm32f4xx.h"
#include "foc_trajectory.h"
#include "drive.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


/* =============================================================================
 * Debug counters (read with the debugger)
 * =============================================================================*/

volatile uint32_t cnt_telem      = 0;   /* whole 32-byte RX frames        */
volatile uint32_t cnt_error      = 0;   /* MoveCmd CRC failures           */
volatile uint32_t cnt_cs         = 0;   /* CS rising edges                */
volatile uint32_t cnt_rx_short   = 0;   /* transactions under 32 bytes    */
volatile uint32_t cnt_rearm_late = 0;   /* re-arm finished with CS low    */


/* =============================================================================
 * SPI DMA buffers
 * =============================================================================*/

/*
 * Both DMAs run one frame and stop; the CS handler re-arms them, so the
 * frame in each buffer is stable while the handler works on it.
 */
static volatile uint8_t spi2_rx_buf[SPI2_TRANSACTION_BYTES];
static uint8_t spi2_tx_buf[SPI2_TRANSACTION_BYTES] __attribute__((aligned(4)));

/*
 * Telemetry staging, ping-pong. The producer (TIM1 ISR in sysid, SysTick in
 * trajectory mode) writes the slot that is not published, then flips
 * tx_stage_pub; the CS handler copies the published slot. A producer
 * preempting that copy writes the other slot, so the copy is never torn.
 */
static uint8_t tx_stage[2][SPI2_TRANSACTION_BYTES] __attribute__((aligned(4)));
static volatile uint8_t tx_stage_pub = 0u;


extern volatile uint32_t g_control_tick;   /* main.c -- 20kHz, 50us per count */


typedef char SysIdSample_must_be_32_bytes[
    (sizeof(SysIdSample) == SPI2_TRANSACTION_BYTES) ? 1 : -1
];

typedef char TelemetryFrame_must_be_32_bytes[
    (sizeof(TelemetryFrame) == SPI2_TRANSACTION_BYTES) ? 1 : -1
];


/* =============================================================================
 * DMA flag clear
 * =============================================================================*/

static void spi2_dma_clear_flags(void)
{
    DMA1->LIFCR = DMA_LIFCR_CTCIF3  |
                  DMA_LIFCR_CHTIF3  |
                  DMA_LIFCR_CTEIF3  |
                  DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;

    DMA1->HIFCR = DMA_HIFCR_CTCIF4  |
                  DMA_HIFCR_CHTIF4  |
                  DMA_HIFCR_CTEIF4  |
                  DMA_HIFCR_CDMEIF4 |
                  DMA_HIFCR_CFEIF4;
}


/* =============================================================================
 * Telemetry staging — see tx_stage above
 * =============================================================================*/

static void spi_tx_publish(const void *frame)
{
    uint32_t w = tx_stage_pub ^ 1u;

    memcpy(tx_stage[w], frame, SPI2_TRANSACTION_BYTES);
    __DMB();
    tx_stage_pub = (uint8_t)w;
}


/* =============================================================================
 * spi_sysid_update_latest — sysid telemetry, called from the TIM1 ISR
 * =============================================================================*/
void spi_sysid_update_latest(int16_t ia_mA,
                             int16_t ib_mA,
                             int16_t ic_mA,
                             int16_t id_mA,
                             int16_t iq_mA,
                             int16_t vd_mV,
                             int16_t vq_mV,
                             int16_t theta_mrad,
                             uint16_t adc_a,
                             uint16_t adc_b,
                             uint16_t adc_c,
                             uint16_t flags,
                             uint16_t test_id)
{
    SysIdSample sample;
    SysIdSample *s = &sample;

    s->t          = g_control_tick;   /* exact 50us timestamp, not a sequence */
    s->ia_mA      = ia_mA;
    s->ib_mA      = ib_mA;
    s->ic_mA      = ic_mA;
    s->id_mA      = id_mA;
    s->iq_mA      = iq_mA;
    s->vd_mV      = vd_mV;
    s->vq_mV      = vq_mV;
    s->theta_mrad = theta_mrad;
    s->adc_a      = adc_a;
    s->adc_b      = adc_b;
    s->adc_c      = adc_c;
    s->flags      = flags;
    s->pad        = test_id;
    s->crc        = crc16_calc((const uint8_t *)s, SYSID_CRC_LEN);   /* last: covers pad */

    spi_tx_publish(s);
}


/* =============================================================================
 * spi2_rearm — set the link up for the next transaction
 *
 * Called at init and from the CS handler, with CS high. Resetting SPI2
 * through RCC is the only way to drop the byte it already holds in its TX
 * buffer; without it the next frame would go out shifted by that byte.
 * Then the newest staged telemetry goes into the TX buffer and both DMAs
 * restart at byte 0. SPE goes last, so DMA preloads byte 0 into DR before
 * the Pi's first clock.
 * =============================================================================*/

static void spi2_rearm(void)
{
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;   // stop RX DMA: SPI2->DR -> RX buffer (MoveCmd from the Pi)
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;   // stop TX DMA: TX buffer -> SPI2->DR (telemetry to the Pi)
    for (uint32_t i = 0u;
         ((DMA1_Stream3->CR | DMA1_Stream4->CR) & DMA_SxCR_EN) && i < 1000u;
         i++) { }

    RCC->APB1RSTR |=  RCC_APB1RSTR_SPI2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;

    spi2_dma_clear_flags();

    memcpy(spi2_tx_buf, tx_stage[tx_stage_pub], SPI2_TRANSACTION_BYTES);
    __DMB();

    DMA1_Stream3->NDTR = SPI2_TRANSACTION_BYTES;
    DMA1_Stream4->NDTR = SPI2_TRANSACTION_BYTES;
    DMA1_Stream3->CR  |= DMA_SxCR_EN;
    DMA1_Stream4->CR  |= DMA_SxCR_EN;

    SPI2->CR1  = SPI_CR1_CPHA;
    SPI2->CR2  = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
    SPI2->CR1 |= SPI_CR1_SPE;

    /* CS already low: the Pi started the next transaction before the link
     * was ready, so that frame is lost. */
    if (!(GPIOB->IDR & (1u << PIN_RPI_NSS)))
        cnt_rearm_late++;
}


/* =============================================================================
 * SPI init
 * =============================================================================*/

void spi_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN
                  | RCC_AHB1ENR_GPIOBEN;

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* SPI2 pins: PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI */
    GPIOB->MODER &= ~((3u << (PIN_RPI_NSS  * 2)) |
                      (3u << (PIN_RPI_SCK  * 2)) |
                      (3u << (PIN_RPI_MISO * 2)) |
                      (3u << (PIN_RPI_MOSI * 2)));

    GPIOB->MODER |=  ((2u << (PIN_RPI_NSS  * 2)) |
                      (2u << (PIN_RPI_SCK  * 2)) |
                      (2u << (PIN_RPI_MISO * 2)) |
                      (2u << (PIN_RPI_MOSI * 2)));

    GPIOB->AFR[1] &= ~((0xFu << ((PIN_RPI_NSS  - 8u) * 4u)) |
                       (0xFu << ((PIN_RPI_SCK  - 8u) * 4u)) |
                       (0xFu << ((PIN_RPI_MISO - 8u) * 4u)) |
                       (0xFu << ((PIN_RPI_MOSI - 8u) * 4u)));

    GPIOB->AFR[1] |=  ((5u << ((PIN_RPI_NSS  - 8u) * 4u)) |
                       (5u << ((PIN_RPI_SCK  - 8u) * 4u)) |
                       (5u << ((PIN_RPI_MISO - 8u) * 4u)) |
                       (5u << ((PIN_RPI_MOSI - 8u) * 4u)));

    GPIOB->OSPEEDR |= ((3u << (PIN_RPI_NSS  * 2)) |
                       (3u << (PIN_RPI_SCK  * 2)) |
                       (3u << (PIN_RPI_MISO * 2)) |
                       (3u << (PIN_RPI_MOSI * 2)));

    GPIOB->PUPDR &= ~(3u << (PIN_RPI_NSS * 2));
    GPIOB->PUPDR |=  (1u << (PIN_RPI_NSS * 2));

    /* Disable SPI and DMA before reconfig */
    SPI2->CR1 = 0;
    SPI2->CR2 = 0;

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN) {}

    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN) {}

    spi2_dma_clear_flags();

#if RUN_MODE == RUN_MODE_SYSID
    /* Boot sample — flags=0x1234 marks pre-TIM1 data */
    spi_sysid_update_latest(100, 200, 300,
                            400, 500,
                            600, 700,
                            800,
                            2048, 2050, 2046,
                            0x1234, 0);
#else
    /* Boot sample -- drive_state=DRIVE_IDLE (0), everything else zero, so
     * the Pi sees a valid, CRC-correct frame from the very first read
     * instead of uninitialized memory before the control loop writes
     * anything real. */
    {
        TelemetryFrame boot_frame = {0};
        spi_update_telem(&boot_frame);
    }
#endif

    /* Fixed DMA addresses; spi2_rearm() loads the counts and starts both. */
    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)spi2_rx_buf;
    DMA1_Stream3->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_MINC;

    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)spi2_tx_buf;
    DMA1_Stream4->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_DIR_0             |   /* memory -> peripheral */
        DMA_SxCR_MINC;

    spi2_rearm();

    /* CS rising edge IRQ on PB12 / EXTI12 -- end of transaction */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[3] &= ~(0xFu << 0);
    SYSCFG->EXTICR[3] |=  (0x1u << 0);

    EXTI->FTSR &= ~(1u << PIN_RPI_NSS);
    EXTI->RTSR |=  (1u << PIN_RPI_NSS);
    EXTI->IMR  |=  (1u << PIN_RPI_NSS);
    EXTI->PR    =  (1u << PIN_RPI_NSS);

    NVIC_SetPriority(EXTI15_10_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}


/* =============================================================================
 * spi_update_telem — closed-loop telemetry, called from the position tick.
 * Stamps the CRC and publishes the frame.
 * =============================================================================*/

#if RUN_MODE != RUN_MODE_SYSID
void spi_update_telem(const TelemetryFrame *frame)
{
    TelemetryFrame f = *frame;

    f.crc16 = crc16_calc((const uint8_t *)&f, TELEM_CRC_LEN);
    spi_tx_publish(&f);
}
#endif


/* =============================================================================
 * spi2_rx_parse — act on one complete 32-byte frame from the Pi
 * =============================================================================*/

#if RUN_MODE != RUN_MODE_SYSID
static volatile uint8_t move_seq_accepted = 0;

uint8_t spi_move_seq(void)
{
    return move_seq_accepted;
}

static void spi2_rx_parse(const uint8_t *frame)
{
    if (frame[0] == SPI2_OP_MOVE_CMD)
    {
        MoveCmd cmd;
        memcpy(&cmd, frame, sizeof(cmd));

        uint16_t calc = crc16_calc((const uint8_t *)&cmd, MOVE_CMD_CRC_LEN);
        if (calc == cmd.crc16)
        {
            // The Pi resends the MoveCmd until move_seq echoes its seq, so
            // a repeat of the accepted seq is ignored. A new seq waits while
            // a move is running; the Pi keeps sending it.
            if (cmd.seq != move_seq_accepted &&
                drive_is_idle() && !foc_trajectory_move_active())
            {
                foc_trajectory_start_move(cmd.target_pos, cmd.vel_max, cmd.accel_max);
                drive_request_servo_on();
                move_seq_accepted = cmd.seq;
            }
        }
        else
        {
            cnt_error++;
        }
    }
}
#endif  /* MoveCmd only in trajectory mode: sysid never starts a move */


/* =============================================================================
 * CS rising edge IRQ (EXTI12) — end of transaction
 * =============================================================================*/
/*
 * Fires when the Pi releases CS, once per transaction. The RX DMA stops
 * after 32 bytes, so NDTR == 0 means a whole frame arrived; a shorter
 * transaction (a glitch) is dropped. Then the link is re-armed for the next
 * transaction, whatever this one was.
 */
void EXTI15_10_IRQHandler(void)
{
    if (!(EXTI->PR & (1u << PIN_RPI_NSS))) return;

    ISR_TIME_BEGIN(ISR_T_EXTI, ISR_PIN_EXTI);

    EXTI->PR = (1u << PIN_RPI_NSS);
    cnt_cs++;

    if (DMA1_Stream3->NDTR == 0u)
    {
        cnt_telem++;
#if RUN_MODE != RUN_MODE_SYSID
        spi2_rx_parse((const uint8_t *)spi2_rx_buf);
#endif
    }
    else
    {
        cnt_rx_short++;
    }

    spi2_rearm();

    ISR_TIME_END(ISR_T_EXTI, ISR_PIN_EXTI);
}

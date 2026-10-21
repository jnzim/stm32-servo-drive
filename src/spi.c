// spi.c — SPI2 slave link to the Pi: 32-byte telemetry out, MoveCmd in.
//
// TX: DMA1 Stream4, free-running CIRC over one 32-byte buffer. The control
// loop overwrites the buffer in place and DMA keeps sending whatever is
// there, so no interrupt is in the TX path and the control loop stays the
// highest priority. TX is never realigned: after a short transaction its
// byte phase can drift, and the Pi resyncs by CRC (foc-sysid).
//
// RX: DMA1 Stream3, CIRC + double buffer, no DMA interrupt. CS marks the
// frame: on the CS rising edge (EXTI12) a 32-byte transaction is parsed, and
// any other length puts the RX DMA back at byte 0.

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

volatile uint32_t cnt_telem      = 0;   /* 32-byte RX frames at CS edge   */
volatile uint32_t cnt_error      = 0;   /* MoveCmd CRC failures           */
volatile uint32_t cnt_cs         = 0;   /* CS rising edges                */
volatile uint32_t cnt_rx_realign = 0;   /* RX DMA realigned after a short */
                                        /* transaction                    */


/* =============================================================================
 * SPI DMA buffers
 * =============================================================================*/

/*
 * Double-buffered RX: DMA fills one buffer while the CS handler reads the
 * frame that just completed in the other, so the next transaction cannot
 * overwrite a frame mid-read.
 */
static volatile uint8_t spi2_rx_buf[2][SPI2_TRANSACTION_BYTES];

/*
 * Single live TX buffer -- see file header. CIRC DMA reads it
 * continuously; the control loop writes it in place, unconditionally.
 */
static SysIdSample spi_tx_buf;

/*
 * TX buffer in RUN_MODE_CLOSED_LOOP (TelemetryFrame). Sysid sends
 * spi_tx_buf (SysIdSample) instead.
 */
#if RUN_MODE != RUN_MODE_SYSID
static TelemetryFrame telem_tx_buf;
#endif


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

static void dma_stream3_clear_flags(void)
{
    DMA1->LIFCR = DMA_LIFCR_CTCIF3  |
                  DMA_LIFCR_CHTIF3  |
                  DMA_LIFCR_CTEIF3  |
                  DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;
}

static void spi2_dma_clear_flags(void)
{
    dma_stream3_clear_flags();

    DMA1->HIFCR = DMA_HIFCR_CTCIF4  |
                  DMA_HIFCR_CHTIF4  |
                  DMA_HIFCR_CTEIF4  |
                  DMA_HIFCR_CDMEIF4 |
                  DMA_HIFCR_CFEIF4;
}


/* =============================================================================
 * spi_sysid_update_latest — sysid telemetry, called from the TIM1 ISR
 *
 * Writes only while the Pi's chip select is idle (NSS high), so a frame is
 * never rewritten mid-transmission. A transaction is 32 bytes at 1 MHz =
 * 256us, about every 294us, so the ~38us idle gap is hit regularly; a
 * skipped tick just means the Pi reads a slightly older frame.
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
    /* NSS is active low: high = no transaction in progress. */
    if (!(GPIOB->IDR & (1u << PIN_RPI_NSS)))
        return;

    SysIdSample *s = &spi_tx_buf;

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
    s->crc        = crc16_calc((const uint8_t *)s, SYSID_CRC_LEN);
    s->pad        = test_id;

    __DMB();
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

    /*
     * RX DMA: Pi MOSI -> SPI2->DR -> spi2_rx_buf[0/1]. CIRC + double buffer:
     * fills buffer 0 first, then alternates at each transfer complete.
     */
    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)spi2_rx_buf[0];
    DMA1_Stream3->M1AR = (uint32_t)spi2_rx_buf[1];
    DMA1_Stream3->NDTR = SPI2_TRANSACTION_BYTES;
    DMA1_Stream3->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC              |
        DMA_SxCR_DBM;

    DMA1_Stream3->CR |= DMA_SxCR_EN;

    /*
     * TX DMA: TX buffer -> SPI2->DR -> Pi MISO. Free-running CIRC, set up
     * once here and never touched again (see file header).
     */
    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
#if RUN_MODE == RUN_MODE_SYSID
    DMA1_Stream4->M0AR = (uint32_t)&spi_tx_buf;
#else
    DMA1_Stream4->M0AR = (uint32_t)&telem_tx_buf;
#endif
    DMA1_Stream4->NDTR = SPI2_TRANSACTION_BYTES;
    DMA1_Stream4->CR =
        (0u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_DIR_0             |   /* memory -> peripheral */
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC;

    DMA1_Stream4->CR |= DMA_SxCR_EN;

    SPI2->CR1 = SPI_CR1_CPHA;
    SPI2->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

    __DMB();

    SPI2->CR1 |= SPI_CR1_SPE;

    /* CS rising edge IRQ on PB12 / EXTI12 -- end of transaction, RX realign */
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
 * Copies the frame into the TX buffer and computes the CRC last.
 * =============================================================================*/

#if RUN_MODE != RUN_MODE_SYSID
void spi_update_telem(const TelemetryFrame *frame)
{
    telem_tx_buf         = *frame;
    telem_tx_buf.crc16   = crc16_calc((const uint8_t *)&telem_tx_buf, TELEM_CRC_LEN);
    __DMB();
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
 * Puts the RX DMA back at byte 0 of buffer 0 after a transaction that was
 * not a whole 32-byte frame (the Pi's resync reads, a glitch). The RX DMA is
 * free-running with nothing tying it to CS, so without this every later
 * frame would arrive rotated. TX is not touched; the Pi realigns to it.
 */
static void spi2_rx_realign(void)
{
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    for (uint32_t i = 0u; (DMA1_Stream3->CR & DMA_SxCR_EN) && i < 1000u; i++) { }

    /* Drop any byte left in the SPI receiver, and clear OVR (DR then SR). */
    while (SPI2->SR & SPI_SR_RXNE) { (void)SPI2->DR; }
    (void)SPI2->SR;

    DMA1_Stream3->CR  &= ~DMA_SxCR_CT;
    DMA1_Stream3->NDTR = SPI2_TRANSACTION_BYTES;
    DMA1_Stream3->CR  |= DMA_SxCR_EN;

    cnt_rx_realign++;
}

/*
 * Fires when the Pi releases CS, once per transaction. CS is the frame
 * boundary: a transaction of exactly 32 bytes is a frame and is parsed here;
 * anything else is discarded by a realign.
 *
 * NDTR == 32 means the DMA wrapped, so a whole frame landed and CT flipped:
 * the frame is in buffer 1 - CT, and the next transaction fills the other
 * one. The opcode byte is cleared after the parse, so a CS edge with no
 * new frame behind it (a glitch) cannot run the same command twice.
 */
void EXTI15_10_IRQHandler(void)
{
    if (!(EXTI->PR & (1u << PIN_RPI_NSS))) return;

    ISR_TIME_BEGIN(ISR_T_EXTI, ISR_PIN_EXTI);

    EXTI->PR = (1u << PIN_RPI_NSS);
    cnt_cs++;

    if (DMA1_Stream3->NDTR == SPI2_TRANSACTION_BYTES)
    {
        cnt_telem++;
#if RUN_MODE != RUN_MODE_SYSID
        uint32_t ct          = (DMA1_Stream3->CR & DMA_SxCR_CT) ? 1u : 0u;
        volatile uint8_t *rx = spi2_rx_buf[1u - ct];

        spi2_rx_parse((const uint8_t *)rx);
        rx[0] = 0u;
#endif
    }
    else
    {
        spi2_rx_realign();
    }

    ISR_TIME_END(ISR_T_EXTI, ISR_PIN_EXTI);
}
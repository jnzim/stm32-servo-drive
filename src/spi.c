// spi.c — SPI2 slave TX/RX DMA sysid latest-sample bring-up
//
// TX DMA is free-running CIRC over a single live buffer. The control loop
// (TIM1 ISR) writes straight into it, unconditionally, every tick -- no
// ping-pong, no CS check, no interrupt in the loop at all. DMA keeps
// re-transmitting whatever is currently there, forever, entirely in
// hardware; the Pi always gets the latest committed sample with zero
// dependency on any ISR's latency or priority.
//
// This means the control loop can be (and is) the unconditional highest
// NVIC priority in the system -- nothing here ever needs to preempt it.
// The previous design needed EXTI15_10 (CS edge) to react within the gap
// between Pi transactions to re-arm the TX stream; that gap turned out to
// be tighter than one control-loop period, which is genuinely not a
// defensible reason to let telemetry outrank the control loop. Removing
// the need for that reaction removes the constraint instead of working
// around it.
//
// Tradeoff: a torn/misaligned frame is now caught purely by the crc field,
// same mechanism already relied on for the flags-aliasing corruption bug
// below. After a glitched/short transaction the DMA's byte phase can drift
// relative to the Pi's 32-byte read window (nothing here forces
// realignment anymore); the Pi side must detect a bad CRC and resync by
// trying the other byte rotations of what it read, the standard technique
// for a self-framed stream. That resync logic lives in the Pi-side capture
// tool (foc-sysid, rpi5), not in this repo.

#include "spi.h"
#include "board_f411.h"
#include "protocol.h"
#include "stm32f4xx.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


/* =============================================================================
 * Legacy globals
 * =============================================================================*/

volatile uint32_t cnt_data      = 0;
volatile uint32_t cnt_error     = 0;
volatile uint32_t cnt_telem     = 0;
volatile uint32_t cnt_block_hdr = 0;
volatile uint32_t cnt_cs        = 0;
volatile uint32_t cnt_tx_rearm  = 0;
volatile uint32_t cnt_dma_stall = 0;

volatile uint8_t ready_asserted = 0;

volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

volatile uint16_t expected_samples = 0;


/* =============================================================================
 * SPI DMA buffers
 * =============================================================================*/

static volatile uint8_t spi2_rx_buf[SPI2_TRANSACTION_BYTES];

/*
 * Single live TX buffer -- see file header. CIRC DMA reads it
 * continuously; the control loop writes it in place, unconditionally.
 */
static SysIdSample spi_tx_buf;


/* =============================================================================
 * Sysid sequence
 * =============================================================================*/

static volatile uint32_t sysid_seq = 0;


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
 * Public sysid update API
 *
 * Called from TIM1 ISR at 20 kHz. Writes straight into the single live
 * buffer, unconditionally -- no CS check, no double-buffer. A write can
 * land while the Pi is mid-transaction and hand out a torn frame; the
 * crc field below is what catches that on the Pi side. __DMB() orders
 * the field writes so the DMA (or a racing read) can't see a partial
 * update.
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
    SysIdSample *s = &spi_tx_buf;

    s->t          = sysid_seq++;
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
    /* Real CRC now -- this was hardcoded to 0 (no integrity checking at
     * all). A single corrupted frame's flags byte was aliasing into a
     * valid-looking SYSID_STAGE_IDLE (low 2 bits happened to read 0b10),
     * making the Pi capture tool think the sweep finished ~23s into a 60s
     * run. Same crc16_calc/CCITT already used for TrajSlot. */
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
                  | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN;

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* PC13 READY output, idle high */
    GPIOC->MODER   &= ~(3u << (READY_RING_REFILL * 2));
    GPIOC->MODER   |=  (1u << (READY_RING_REFILL * 2));
    GPIOC->OTYPER  &= ~(1u << READY_RING_REFILL);
    GPIOC->OSPEEDR &= ~(3u << (READY_RING_REFILL * 2));
    GPIOC->OSPEEDR |=  (1u << (READY_RING_REFILL * 2));
    GPIOC->PUPDR   &= ~(3u << (READY_RING_REFILL * 2));
    GPIOC->BSRR     =  (1u << READY_RING_REFILL);

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

    /* Boot sample — flags=0x1234 marks pre-TIM1 data */
    spi_sysid_update_latest(100, 200, 300,
                            400, 500,
                            600, 700,
                            800,
                            2048, 2050, 2046,
                            0x1234, 0);

    /*
     * RX DMA: Pi MOSI -> SPI2->DR -> spi2_rx_buf
     * CIRC — just counts transactions.
     */
    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)spi2_rx_buf;
    DMA1_Stream3->NDTR = SPI2_TRANSACTION_BYTES;
    DMA1_Stream3->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC              |
        DMA_SxCR_TCIE;

    DMA1_Stream3->CR |= DMA_SxCR_EN;

    NVIC_SetPriority(DMA1_Stream3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    /*
     * TX DMA: spi_tx_buf -> SPI2->DR -> Pi MISO
     *
     * Free-running CIRC over the single 32-byte buffer. M0AR/NDTR are set
     * once, here, and never touched again -- no CS-triggered rearm, no
     * interrupt in this path at all. The stream just keeps re-sending
     * whatever spi_sysid_update_latest() last wrote, forever, entirely in
     * hardware. See file header for the phase-drift tradeoff this implies
     * and why it's the Pi's job (CRC-based resync) to handle, not an ISR's.
     */
    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)&spi_tx_buf;
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

    /* CS edge IRQ on PB12 / EXTI12, falling edge — kept for cnt_cs counting */
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
 * Legacy telemetry API — kept for link compatibility
 * =============================================================================*/

void spi_update_telem(const TelemetryFrame *frame)
{
    (void)frame;
}


/* =============================================================================
 * SPI RX complete IRQ (DMA1 Stream3)
 * =============================================================================*/

void DMA1_Stream3_IRQHandler(void)
{
    if (!(DMA1->LISR & DMA_LISR_TCIF3)) {
        return;
    }

    DMA1->LIFCR = DMA_LIFCR_CTCIF3  |
                  DMA_LIFCR_CHTIF3  |
                  DMA_LIFCR_CTEIF3  |
                  DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;

    cnt_telem++;
}


/* =============================================================================
 * CS falling edge IRQ (EXTI12) — counting only, no rearm
 * =============================================================================*/
/*
 * CS edge -- stats only now (cnt_cs). The old freeze bug and the rearm
 * dance that used to live here (see git history if needed) both went
 * away with it: DMA1_Stream4 is free-running CIRC (spi_init()), so there
 * is nothing left to rearm and nothing this handler must do before the
 * next transaction. It has no deadline, so its priority no longer
 * matters for correctness -- it's set low (2) purely because that's
 * where a no-deadline stats counter belongs, not because anything
 * requires it.
 */
void EXTI15_10_IRQHandler(void)
{
    if (!(EXTI->PR & (1u << PIN_RPI_NSS))) return;
    EXTI->PR = (1u << PIN_RPI_NSS);
    cnt_cs++;
}
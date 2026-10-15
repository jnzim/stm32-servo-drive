// ringBuffer.c — single-producer / single-consumer ring of TrajSlot frames,
// for streaming a trajectory from the Pi. Not used at present: moves arrive
// as one MoveCmd and the profile is generated on the STM. Only ring_init()
// is called (main.c).

#include "ringBuffer.h"
#include "stm32f4xx.h"

RingBuffer ring;

void ring_init(void)
{
    ring.write_idx = 0;
    ring.read_idx  = 0;
}

/* Writer side only: empties the ring without touching the reader's index.
 * Call only while the reader is not popping. */
void ring_reset(void)
{
    ring.write_idx = ring.read_idx;
}

/* Never set at present, so ring_get_crc_error() returns 0. */
static uint8_t traj_crc_error = 0;


uint32_t cnt_push = 0;
uint32_t cnt_pop = 0;
uint32_t cnt_pop_fail = 0;
uint32_t cnt_crc_fail = 0;

void ring_push(const TrajSlot* s)
{
    uint32_t w = ring.write_idx;
    uint32_t r = ring.read_idx;
    if ((w - r) >= RING_BUFFER_SIZE) return;
    
    /* Validate opcode and CRC */
    if (s->opcode != SPI2_OP_DATA)
    {
        cnt_crc_fail++;
        return;
    }
    
    uint16_t crc_calc = crc16_calc((uint8_t*)s, TRAJ_CRC_LEN);
    if (crc_calc != s->crc16)
    {
        cnt_crc_fail++;
        return;
    }
    
    ring.buf[w & RING_MASK] = *s;
    __DMB();
    ring.write_idx = w + 1u;
    cnt_push++;
}

//Single reader (SysTick, priority 15). 

int ring_pop(TrajSlot* s)
{
    uint32_t r = ring.read_idx;
    uint32_t w = ring.write_idx;
    if (w == r)
    {
        cnt_pop_fail++;
        return 0;
    }
    __DMB();
    *s = ring.buf[r & RING_MASK];
    __DMB();
    ring.read_idx = r + 1u;
    cnt_pop++;
    return 1;
}


/* Slots waiting. Unsigned subtraction works across index wrap as long as
 * the live difference stays below RING_BUFFER_SIZE. */
uint32_t ring_count(void)
{
    uint32_t w = ring.write_idx;
    uint32_t r = ring.read_idx;
    return w - r;
}

uint8_t ring_get_crc_error(void)
{
    uint8_t err = traj_crc_error;
    traj_crc_error = 0;
    return err;
}
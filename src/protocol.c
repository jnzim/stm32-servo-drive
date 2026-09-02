#include "protocol.h"
#include <stdint.h>
#include <stddef.h>

uint16_t crc16_calc(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
        {
            // Test the outgoing bit BEFORE shifting, not after: crc is
            // uint16_t, so "crc <<= 1; if (crc & 0x10000)" can never fire --
            // the compound shift-assign already truncates to 16 bits before
            // that check runs, silently discarding the exact bit being
            // tested. That made this function return 0x0000 for any input
            // >= 2 bytes, independent of content. Verified against the
            // standard CRC-16/CCITT-FALSE check value:
            // crc16_calc("123456789", 9) == 0x29B1.
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ CRC16_POLY);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

bool crc16_valid(const TrajSlot* s, size_t crc_len)
{
    uint16_t calc = crc16_calc((const uint8_t*)s, crc_len);
    return (calc == s->crc16);
}
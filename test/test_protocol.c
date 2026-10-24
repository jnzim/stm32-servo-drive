// test_protocol.c -- crc16_calc (src/protocol.c)

#include <string.h>

#include "test.h"
#include "protocol.h"

static void crc16_check_value(void)
{
    DESCRIBE("Standard CRC-16/CCITT-FALSE check value: crc(\"123456789\") == 0x29B1.");
    const char *s = "123456789";
    CHECK_CMP(crc16_calc((const uint8_t *)s, strlen(s)), ==, 0x29B1);
}

static void crc16_empty_is_init(void)
{
    DESCRIBE("Empty input returns the init value 0xFFFF.");
    CHECK_CMP(crc16_calc(NULL, 0), ==, 0xFFFF);
}

static void crc16_detects_bit_flip(void)
{
    DESCRIBE("Every one of the 240 single-bit flips in a 30-byte frame changes the CRC (missed == 0).");
    uint8_t frame[30];
    for (int i = 0; i < 30; i++) frame[i] = (uint8_t)(i * 37 + 11);
    uint16_t good = crc16_calc(frame, sizeof frame);
    int missed = 0;
    for (int byte = 0; byte < 30; byte++)
        for (int bit = 0; bit < 8; bit++)
        {
            frame[byte] ^= (uint8_t)(1u << bit);
            if (crc16_calc(frame, sizeof frame) == good) missed++;
            frame[byte] ^= (uint8_t)(1u << bit);
        }
    CHECK_CMP(missed, ==, 0);
}

void test_crc16(void)
{
    RUN(crc16_check_value);
    RUN(crc16_empty_is_init);
    RUN(crc16_detects_bit_flip);
}

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* =============================================================================
 * Protocol constants
 * =============================================================================
 */
#define SPI2_TRANSACTION_BYTES  32u
#define CRC16_POLY              0x1021  /* CCITT polynomial */

/* Opcodes — Pi → STM commands and STM → Pi responses */
#define SPI2_OP_NOP             0x00
#define SPI2_OP_BLOCK_HDR       0x01
#define SPI2_OP_DATA            0x02
#define SPI2_OP_READY_ACK       0x03
#define SPI2_OP_TELEM_REQ       0x04
#define SPI2_OP_OPEN_LOOP       0x05
#define SPI2_OP_STOP            0x06

/* Drive state machine values */
#define DRIVE_IDLE              0x00u
#define DRIVE_OPEN_LOOP         0x01u
#define DRIVE_ALIGN             0x02u
#define DRIVE_SERVO_ON          0x03u
#define DRIVE_FAULT             0x04u

/* Trajectory frame fault flags */
#define FAULT_TRAJ_CRC          0x01
#define FAULT_TRAJ_DROP         0x02
#define FAULT_TRAJ_OVF          0x04

/* =============================================================================
 * TelemetryFrame — 32 bytes
 * STM → Pi status frame
 * =============================================================================
 */
typedef struct __attribute__((packed)) {
    int32_t  pos_cmd;
    int32_t  pos_fbk;
    int32_t  vel_cmd;
    int32_t  vel_fbk;
    uint32_t timestamp_ms;
    uint8_t  drive_state;
    uint8_t  fault_flags;
    uint32_t samples_consumed;
    int16_t  iq_cmd;
    int16_t  i_q_fbk;
    int16_t  v_q_cmd;
} TelemetryFrame;

/*
 * Field order matters (2026-09-19): pad/flags first, crc last.
 *
 * The TX DMA keeps one byte fetched ahead in SPI2->DR, so the first byte(s)
 * of a frame can already be committed before the control loop rewrites the
 * buffer -- that byte is then stale while the rest is fresh, i.e. a torn
 * frame. pad (the compile-time test id) and flags (the sysid stage) sit at
 * offsets 0..3 because they are constant or near-constant, so a stale leading
 * byte is identical to the fresh one and cannot tear. Everything that changes
 * every tick sits past the prefetch window. crc moves to the end so it covers
 * all 30 preceding bytes.
 */
typedef struct __attribute__((packed))
{
    uint16_t pad;        /* SYSID_TEST id -- constant for a whole run */
    uint16_t flags;      /* sysid stage -- changes a handful of times per run */
    uint32_t t;
    int16_t ia_mA;
    int16_t ib_mA;
    int16_t ic_mA;
    int16_t id_mA;
    int16_t iq_mA;
    int16_t vd_mV;
    int16_t vq_mV;
    int16_t theta_mrad;
    uint16_t adc_a;
    uint16_t adc_b;
    uint16_t adc_c;
    uint16_t crc;        /* last: covers bytes 0..29 */
} SysIdSample;

/* =============================================================================
 * TrajSlot — 32 bytes (fixed-size, DMA-aligned)
 *
 * Pi generates trajectory at 1 kHz, sends 32-byte slots to STM.
 * Ring buffer: 4096 bytes / 32 bytes per slot = 128 slots exact.
 * DMA wrap always lands on slot boundary — no partial packets.
 *
 * Structure:
 *   [0]      opcode (0x02 = DATA, never scanned in payload)
 *   [1]      seq (sequence number, detects drops)
 *   [2..5]   pos_cmd (int32, counts)
 *   [6..9]   vel_cmd (int32, counts/s feedforward)
 *   [10..13] reserved (future expansion)
 *   [14..15] crc16 (covers bytes 0..13)
 *   [16..31] padding (zeros)
 *
 * Parser: fixed-offset memcpy, validate opcode + CRC.
 * If corrupt: reuse last valid (pos_cmd, vel_cmd) for current DT.
 * Never scan payload for frame delimiters — eliminates 0x01/0x02/0x03 corruption.
 * =============================================================================
 */
typedef struct __attribute__((packed)) {
    uint8_t  opcode;      /* 1 */
    uint8_t  seq;         /* 1 */
    int32_t  pos_cmd;     /* 4 */
    int32_t  vel_cmd;     /* 4 */
    uint32_t reserved;    /* 4 */
    uint16_t crc16;       /* 2 */
    uint8_t  padding[16]; /* 16 — total 32 */
} TrajSlot;

/* =============================================================================
 * Size assertions — C11/C++ compatible
 * =============================================================================
 */
#ifdef __cplusplus
static_assert(sizeof(TelemetryFrame) == 32, "TelemetryFrame must be exactly 32 bytes");
static_assert(sizeof(TrajSlot) == 32, "TrajSlot must be exactly 32 bytes");
#else
_Static_assert(sizeof(TelemetryFrame) == 32, "TelemetryFrame must be exactly 32 bytes");
_Static_assert(sizeof(TrajSlot) == 32, "TrajSlot must be exactly 32 bytes");
#endif

#define TRAJ_CRC_LEN   offsetof(TrajSlot, crc16)
#define SYSID_CRC_LEN  offsetof(SysIdSample, crc)

/* =============================================================================
 * CRC-16-CCITT utilities
 * =============================================================================
 */
uint16_t crc16_calc(const uint8_t* data, size_t len);
bool crc16_valid(const TrajSlot* s, size_t crc_len);

#endif
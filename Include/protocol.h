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

/* Minimum CS-high time between transactions. The STM re-arms SPI2 on every
 * CS rising edge (spi.c); that handler can wait behind the 20 kHz control
 * ISR, so the next transaction must not start sooner than this. Sized from
 * sysid, the heaviest mode (2026-10-20): control ISR up to 47 us of its
 * 50 us period, CS edge to re-arm done up to ~135 us; at 120 us about 0.15%
 * of re-arms still finish late (cnt_rearm_late) and that frame fails CRC. */
#define SPI2_MIN_CS_GAP_US      120u

/* Opcodes (wire values shared with the Pi; never renumber) */
#define SPI2_OP_DATA            0x02  /* TrajSlot (ring buffer) */
#define SPI2_OP_MOVE_CMD        0x07  /* single-command move -- see MoveCmd below */

/* Drive state machine values (wire values; 1 and 2 were OPEN_LOOP/ALIGN) */
#define DRIVE_IDLE              0x00u
#define DRIVE_SERVO_ON          0x03u
#define DRIVE_FAULT             0x04u

/* Fault flags */
#define FAULT_FOLLOWING_ERROR   0x08  /* |pos_cmd - pos_fbk| > FOLLOWING_ERROR_LIMIT (config.h) */

/* =============================================================================
 * TelemetryFrame — 32 bytes
 * STM → Pi status frame
 * =============================================================================
 */
typedef struct __attribute__((packed)) {
    int32_t  pos_cmd;           /* counts */
    int32_t  pos_fbk;           /* counts */
    int32_t  vel_cmd;           /* counts/s */
    int32_t  vel_fbk;           /* counts/s */
    uint32_t timestamp_ms;
    uint8_t  drive_state;       /* DRIVE_IDLE/DRIVE_SERVO_ON/etc above */
    uint8_t  fault_flags;
    uint16_t samples_consumed;  /* 1 kHz ticks into the move; wraps at 65 s */
    uint8_t  move_seq;          /* seq of the last MoveCmd accepted (the ack) */
    uint8_t  spare;
    int16_t  iq_cmd;            /* mA */
    int16_t  i_q_fbk;           /* mA */
    uint16_t crc16;             /* covers bytes 0..(offsetof-1) -- TELEM_CRC_LEN below.
                                 * Was v_q_cmd -- dropped to make room. This was the
                                 * one 32-byte frame in the protocol with no self-check
                                 * at all (TrajSlot and SysIdSample both have one). */
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
 * Ring buffer: RING_BUFFER_SIZE (2048) slots of 32 bytes.
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
 * MoveCmd — 32 bytes, one per move (SPI2_OP_MOVE_CMD)
 *
 * Pi -> STM: one command per move. The STM generates the profile itself
 * (trap_gen.c or minjerk_gen.c, per TRAJ_PROFILE) from these three numbers.
 *
 * seq is the handshake: the STM runs a MoveCmd only if its seq differs from
 * the last one it accepted, and echoes that seq in TelemetryFrame.move_seq.
 * The Pi picks seq = move_seq + 1 (skipping 0) and resends until the echo
 * matches, so a lost frame is retried and a repeat is never run twice.
 *
 * Structure:
 *   [0]      opcode (0x07 = MOVE_CMD)
 *   [1]      seq (never 0)
 *   [2..5]   target_pos (int32, counts, RELATIVE to wherever the stage
 *            actually is when the move starts -- a delta, not an absolute
 *            position. Sending the same MoveCmd twice moves twice.)
 *   [6..9]   vel_max    (int32, counts/s, magnitude)
 *   [10..13] accel_max  (int32, counts/s^2, magnitude)
 *   [14..15] crc16 (covers bytes 0..13)
 *   [16..31] padding (zeros)
 * =============================================================================
 */
typedef struct __attribute__((packed)) {
    uint8_t  opcode;       /* 1 */
    uint8_t  seq;          /* 1 */
    int32_t  target_pos;   /* 4 -- delta from current position, see above */
    int32_t  vel_max;      /* 4 */
    int32_t  accel_max;    /* 4 */
    uint16_t crc16;        /* 2 */
    uint8_t  padding[16];  /* 16 — total 32 */
} MoveCmd;

/* =============================================================================
 * Size assertions — C11/C++ compatible
 * =============================================================================
 */
#ifdef __cplusplus
static_assert(sizeof(TelemetryFrame) == 32, "TelemetryFrame must be exactly 32 bytes");
static_assert(sizeof(TrajSlot) == 32, "TrajSlot must be exactly 32 bytes");
static_assert(sizeof(MoveCmd) == 32, "MoveCmd must be exactly 32 bytes");
#else
_Static_assert(sizeof(TelemetryFrame) == 32, "TelemetryFrame must be exactly 32 bytes");
_Static_assert(sizeof(TrajSlot) == 32, "TrajSlot must be exactly 32 bytes");
_Static_assert(sizeof(MoveCmd) == 32, "MoveCmd must be exactly 32 bytes");
#endif

#define TRAJ_CRC_LEN      offsetof(TrajSlot, crc16)
#define SYSID_CRC_LEN     offsetof(SysIdSample, crc)
#define TELEM_CRC_LEN     offsetof(TelemetryFrame, crc16)
#define MOVE_CMD_CRC_LEN  offsetof(MoveCmd, crc16)

/* =============================================================================
 * CRC-16-CCITT utilities
 * =============================================================================
 */
uint16_t crc16_calc(const uint8_t* data, size_t len);

#endif
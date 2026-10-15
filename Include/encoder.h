// encoder.h — TIM2 quadrature encoder interface
// STM32F411RE bare metal

#pragma once
#include <stdint.h>

// =============================================================================
// EncoderState — current position and velocity
// Written by encoder_update(), read by velocity loop and telemetry.
// =============================================================================
typedef struct {
    int32_t  position;      // encoder counts, absolute from startup
    int32_t  velocity;      // dx/dt, counts/sec
    uint32_t timestamp_ms;  // time of last update
} EncoderState;

// Defined in encoder.c
extern volatile EncoderState encoder;

// =============================================================================
// Public interface
// =============================================================================
void    encoder_init(void);
void    encoder_zero(void);
void    encoder_update(uint32_t timestamp_ms);

int32_t encoder_get_position(void);
int32_t encoder_get_velocity(void);

// Electrical angle [0, 2*pi) relative to the alignment offset (counts).
float   encoder_elec_angle(int32_t offset);
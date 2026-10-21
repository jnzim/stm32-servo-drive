#pragma once
#include <stdint.h>
#include "protocol.h"

void spi_init(void);
void spi_update_telem(const TelemetryFrame *frame);
uint8_t spi_move_seq(void);   // seq of the last MoveCmd accepted (trajectory mode)

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
                             uint16_t test_id);

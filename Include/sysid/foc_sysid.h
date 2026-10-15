// foc_sysid.h — system identification tests (RUN_MODE_SYSID)
//
// The test is selected by SYSID_TEST in config.h; see foc_sysid.c for the list.

#pragma once

// foc_sysid_step — call from the TIM1 ISR at 20 kHz, RUN_MODE_SYSID only
void foc_sysid_step(void);

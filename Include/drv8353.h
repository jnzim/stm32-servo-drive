#pragma once

#include <stdint.h>
#include <stdbool.h>

#define DRV8353_REG_FAULT_STATUS_1   0x00u
#define DRV8353_REG_VGS_STATUS_2     0x01u
#define DRV8353_REG_DRIVER_CONTROL   0x02u
#define DRV8353_REG_GATE_DRIVE_HS    0x03u
#define DRV8353_REG_GATE_DRIVE_LS    0x04u
#define DRV8353_REG_OCP_CONTROL      0x05u
#define DRV8353_REG_CSA_CONTROL      0x06u

typedef struct
{
    uint16_t fault_status_1;
    uint16_t vgs_status_2;
    uint16_t driver_control;
    bool n_fault_pin;
} Drv8353Status;

void drv8353_init(void);
bool drv8353_configure(void);
Drv8353Status drv8353_read_status(void);
void drv_enable_high(void);

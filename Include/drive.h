#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

// Values match the drive_state byte the Pi reads in telemetry (protocol.h).
typedef enum {
    STATE_IDLE     = DRIVE_IDLE,
    STATE_SERVO_ON = DRIVE_SERVO_ON,
    STATE_FAULT    = DRIVE_FAULT
} DriveState;

// Lifecycle
void       drive_init(void);
void       drive_sm_run(void);
DriveState drive_get_state(void);

// Status queries — read-only
bool drive_is_idle(void);
bool drive_is_servo_on(void);

// Request functions — called from ISRs, flags consumed by drive_sm_run()
void drive_request_servo_on(void);
void drive_request_fault(void);

#endif // DRIVE_H

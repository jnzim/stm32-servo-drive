
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "clock.h"
#include "spi.h"
#include "drive.h"
#include "ringBuffer.h"
#include "loops.h"
#include "protocol.h"
#include "config.h"
#include "encoder.h"
#include "drv8353.h"
#include "board_f411.h"
#include "pwm.h"
#include "current_feedback.h"
#include "servo_sm.h"

State               state;              
RunMode             runMode             = INIT;
volatile uint32_t   tick_ms             = 0;
volatile bool       system_initialized  = false;

static float foc_vd_applied = 0.0f;
static float foc_vq_applied = 0.0f;



void servo_sm_20khz(void)
{
    
  
    if (!system_initialized)
        return;

    encoder_update(tick_ms);


    switch (state)
    {
        
        case INIT:
            state = ALIGN;
        break;
        case ALIGN:
            foc_vd_applied = V_ALIGN;
            foc_vq_applied = 0.0f;
            pwm_apply_dq(foc_vd_applied, foc_vq_applied, 0.0f);

            if (++sysid_align_tick >= 40000)
            {
                sysid_enc_offset = encoder_get_position();
                loops_reset();
                sysid_stage      = SYSID_STAGE_RUN;
            }
            break;
        case IDLE: 
            
            while (!rpi_ready()) {}  // Wait for RPi to signal ready
            while ( rpi_ready()) {}  // wait for release (falling edge)
            
            state = RUN;
            break;
        case RUN:
                if (runMode == RUN_MODE_NONE)
                {
                    // do nothing
                }
                elseif (runMode == RUN_MODE_PROFILE)
                {

                }
                elseif (runMode == RUN_MODE_CURRENT_STEP)
                {
                    
                }
                elseif (runMode == RUN_MODE_CURRENT_CHIRP)
                {
                    
                }
                elseif (runMode == RUN_MODE_VELOCITY_STEP)
                {
                    
                }    
                elseif (runMode == RUN_MODE_VELOCITY_CHIRP)
                {
                    
                }    
                elseif (runMode == RUN_MODE_POSITION_STEP)
                {
                    
                }    
                elseif (runMode == RUN_MODE_VELOCITY_STEP)
                {
                    
                }                
            break;

        default:
            pwm_apply_dq(0.0f, 0.0f, 0.0f);
            break;
    }
}
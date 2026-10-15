#include "plant.h"
#include "config.h"

PlantState plant;

void plant_init(PlantState *s)
{
    s->vel_rad      = 0.0f;
    s->pos_rad      = 0.0f;
    s->i_q          = 0.0f;
    s->pos_counts   = 0;
    s->vel_counts   = 0;
}

void plant_step(PlantState *s, float v_q, float dt)
{
    // Motor electrical model:
    // V = L*di/dt + R*i + Ke*w
    float di_dt = (v_q - (PLANT_R  * s->i_q) - (PLANT_KE * s->vel_rad)) / PLANT_L;

    s->i_q += di_dt * dt;

    // Motor torque
    float torque = PLANT_KT * s->i_q;

    // Mechanical model:
    // J*dw/dt = torque - B*w
    float accel = (torque - PLANT_B * s->vel_rad) / PLANT_J;

    s->vel_rad += accel * dt;
    s->pos_rad += s->vel_rad * dt;

    s->pos_counts = (int32_t)(s->pos_rad * COUNTS_PER_RAD);
    s->vel_counts = (int32_t)(s->vel_rad * COUNTS_PER_RAD);
}

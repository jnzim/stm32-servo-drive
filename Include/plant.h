#ifndef PLANT_H
#define PLANT_H

#include <stdint.h>
#include "protocol.h"

#define PLANT_KT 0.064f         // Nm/Arms — corrected from datasheet
#define PLANT_KE 0.0391f        // 4.1 Vrms/Krpm -> V / (rad/Sec)
#define PLANT_R  3.1f           // line-lind ohms 
#define PLANT_L  0.00204f       // H 2.04mH
#define PLANT_B  4.77e-6f       // 0.0005N / Krpm kg*m^2, plus load later
#define PLANT_J  1.7e-6f        // 0.017 kg·cm² (rotor + encoder) 

// 4.1 V/krpm * (1 / 1000) * 1 /2*pi





typedef struct {
    float   vel_rad;     // rad/s
    float   pos_rad;     // rad
    float   i_q;        // A
    int32_t pos_counts; // encoder counts
    int32_t vel_counts; // counts/sec
} PlantState;

void  plant_init(PlantState *s);
void  plant_step(PlantState *s, float v_q, float dt);
extern PlantState plant;

#endif
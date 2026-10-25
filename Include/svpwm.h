#ifndef SVPWM_H
#define SVPWM_H

// Min-max (common-mode) injection: the SVPWM-equivalent modulation.
// Adds the same offset to all three phase voltages so the largest and
// smallest are centred on zero. Line-to-line voltages are unchanged; the
// usable phase amplitude rises from Vbus/2 (sinusoidal) to Vbus/sqrt(3).
//
// Not called yet: pwm_apply_phase_volts() is still sinusoidal PWM.
void svpwm_inject(float *va, float *vb, float *vc);

#endif

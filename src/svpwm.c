#include "svpwm.h"

#include <math.h>

void svpwm_inject(float *va, float *vb, float *vc)
{
    float vmax = fmaxf(*va, fmaxf(*vb, *vc));
    float vmin = fminf(*va, fminf(*vb, *vc));
    float vo   = -0.5f * (vmax + vmin);   // common-mode offset: centres max and min on zero

    *va += vo;
    *vb += vo;
    *vc += vo;
}

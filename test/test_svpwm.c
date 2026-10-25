// test_svpwm.c -- svpwm_inject (src/svpwm.c)

#include "test.h"
#include "svpwm.h"

#define V_BUS  24.0f
#define TWO_PI 6.28318530718f

// Balanced three-phase set, amplitude a, at electrical angle th.
static void phases(float a, float th, float *va, float *vb, float *vc)
{
    *va = a * cosf(th);
    *vb = a * cosf(th - TWO_PI / 3.0f);
    *vc = a * cosf(th + TWO_PI / 3.0f);
}

static void svpwm_keeps_line_to_line(void)
{
    DESCRIBE("Injection adds the same offset to all three phases, so va-vb, vb-vc, vc-va are unchanged (360 angles).");
    float worst = 0.0f;
    for (int i = 0; i < 360; i++)
    {
        float va, vb, vc;
        phases(10.0f, i * TWO_PI / 360.0f, &va, &vb, &vc);
        float ab = va - vb, bc = vb - vc, ca = vc - va;
        svpwm_inject(&va, &vb, &vc);
        float e = fmaxf(fabsf((va - vb) - ab), fmaxf(fabsf((vb - vc) - bc), fabsf((vc - va) - ca)));
        if (e > worst) worst = e;
    }
    CHECK_CMP(worst, <, 1e-5);
}

static void svpwm_fits_vbus_over_sqrt3(void)
{
    DESCRIBE("At phase amplitude Vbus/sqrt(3) = 13.86 V, sinusoidal needs 13.86 V peak (over the 12 V Vbus/2 limit); injected peak stays within 12 V.");
    float a = V_BUS / sqrtf(3.0f);
    float peak_sin = 0.0f, peak_sv = 0.0f;
    for (int i = 0; i < 360; i++)
    {
        float va, vb, vc;
        phases(a, i * TWO_PI / 360.0f, &va, &vb, &vc);
        peak_sin = fmaxf(peak_sin, fmaxf(fabsf(va), fmaxf(fabsf(vb), fabsf(vc))));
        svpwm_inject(&va, &vb, &vc);
        peak_sv = fmaxf(peak_sv, fmaxf(fabsf(va), fmaxf(fabsf(vb), fabsf(vc))));
    }
    CHECK_CMP(peak_sin, >, V_BUS / 2.0f);
    CHECK_CMP(peak_sv, <=, V_BUS / 2.0f + 1e-3f);
}

static void svpwm_removes_common_mode(void)
{
    DESCRIBE("Three equal voltages are pure common mode: injection takes them to zero.");
    float va = 2.0f, vb = 2.0f, vc = 2.0f;
    svpwm_inject(&va, &vb, &vc);
    CHECK_NEAR(va, 0.0f, 1e-6);
    CHECK_NEAR(vb, 0.0f, 1e-6);
    CHECK_NEAR(vc, 0.0f, 1e-6);
}

void test_svpwm(void)
{
    RUN(svpwm_keeps_line_to_line);
    RUN(svpwm_fits_vbus_over_sqrt3);
    RUN(svpwm_removes_common_mode);
}

// test_control.c -- pi_step / p_step (src/control.c)

#include "test.h"
#include "control.h"

static void pi_proportional_only(void)
{
    PIState s;
    pi_init(&s, 2.0f, 0.0f, -10.0f, 10.0f);
    CHECK_NEAR(pi_step(&s, 1.5f, 1e-3f), 3.0f, 1e-6);
    CHECK_NEAR(pi_step(&s, -2.0f, 1e-3f), -4.0f, 1e-6);
}

// Constant error: the integrator ramps at ki*e per second.
static void pi_integrates(void)
{
    PIState s;
    pi_init(&s, 0.0f, 100.0f, -10.0f, 10.0f);
    float out = 0.0f;
    for (int i = 0; i < 50; i++) out = pi_step(&s, 1.0f, 1e-3f);
    CHECK_NEAR(out, 100.0f * 1.0f * 0.050f, 1e-4);   // 5.0 after 50 ms
}

// Output never leaves [out_min, out_max], however large the error.
static void pi_output_clamped(void)
{
    PIState s;
    pi_init(&s, 1000.0f, 0.0f, -12.0f, 12.0f);
    CHECK_NEAR(pi_step(&s,  1.0f, 5e-5f),  12.0f, 1e-6);
    CHECK_NEAR(pi_step(&s, -1.0f, 5e-5f), -12.0f, 1e-6);
}

// Anti-windup: a long saturation leaves the integrator at the limit, not far
// past it, so the output comes back as soon as the error reverses.
static void pi_integrator_clamped(void)
{
    PIState s;
    pi_init(&s, 0.0f, 1000.0f, -12.0f, 12.0f);
    for (int i = 0; i < 100000; i++) pi_step(&s, 1.0f, 5e-5f);   // 5 s saturated
    CHECK_NEAR(s.integrator, 12.0f, 1e-6);

    float out = pi_step(&s, -1.0f, 5e-5f);
    CHECK(out < 12.0f);                       // leaves saturation on the first tick
}

static void p_scales(void)
{
    PState s;
    p_init(&s, 380.0f);
    CHECK_NEAR(p_step(&s, 0.1f), 38.0f, 1e-4);
}

void test_pi(void)
{
    RUN(pi_proportional_only);
    RUN(pi_integrates);
    RUN(pi_output_clamped);
    RUN(pi_integrator_clamped);
    RUN(p_scales);
}

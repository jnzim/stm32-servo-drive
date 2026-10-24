// test_profiles.c -- trap_gen and minjerk_gen (src/trap_gen.c, src/minjerk_gen.c)
//
// Both generators run at 1 kHz in foc_trajectory.c, in encoder counts. The
// 10 mm move is 40960 counts at 4096 counts/mm; 10 mm/s and 250 mm/s^2 are
// the limits the Pi sends for the moves on the slides.

#include <stdlib.h>

#include "test.h"
#include "trap_gen.h"
#include "minjerk_gen.h"

#define DT          1e-3f
#define MOVE_COUNTS 40960
#define VEL_MAX     40960       // 10 mm/s
#define ACCEL_MAX   1024000     // 250 mm/s^2
#define MAX_TICKS   100000

typedef struct {
    int     ticks;
    int32_t final_pos;
    float   final_vel;
    float   peak_vel;           // magnitude
    float   peak_accel;         // magnitude, from tick-to-tick velocity change, excluding the last tick
    float   last_accel;         // magnitude, the last tick (trap_gen snaps to rest there)
    int     backsteps;          // ticks where the command moved away from the target
} Run;

static Run run_trap(int32_t start, int32_t target)
{
    TrapGen g;
    Run r = {0};
    trap_gen_start(&g, start, target, VEL_MAX, ACCEL_MAX);
    int32_t pos = start, prev = start;
    float   vel = 0.0f, prev_vel = 0.0f;
    int     dir = (target >= start) ? 1 : -1;
    while (trap_gen_active(&g) && r.ticks < MAX_TICKS)
    {
        trap_gen_step(&g, DT, &pos, &vel);
        r.ticks++;
        if (fabsf(vel) > r.peak_vel) r.peak_vel = fabsf(vel);
        float a = fabsf(vel - prev_vel) / DT;
        if (r.last_accel > r.peak_accel) r.peak_accel = r.last_accel;   // previous tick, now known not last
        r.last_accel = a;
        if ((pos - prev) * dir < 0) r.backsteps++;
        prev = pos;
        prev_vel = vel;
    }
    r.final_pos = pos;
    r.final_vel = vel;
    return r;
}

static Run run_minjerk(int32_t start, int32_t target)
{
    MinJerkGen g;
    Run r = {0};
    minjerk_gen_start(&g, start, target, VEL_MAX, ACCEL_MAX);
    int32_t pos = start, prev = start;
    float   vel = 0.0f, prev_vel = 0.0f;
    int     dir = (target >= start) ? 1 : -1;
    while (minjerk_gen_active(&g) && r.ticks < MAX_TICKS)
    {
        minjerk_gen_step(&g, DT, &pos, &vel);
        r.ticks++;
        if (fabsf(vel) > r.peak_vel) r.peak_vel = fabsf(vel);
        float a = fabsf(vel - prev_vel) / DT;
        if (r.last_accel > r.peak_accel) r.peak_accel = r.last_accel;   // previous tick, now known not last
        r.last_accel = a;
        if ((pos - prev) * dir < 0) r.backsteps++;
        prev = pos;
        prev_vel = vel;
    }
    r.final_pos = pos;
    r.final_vel = vel;
    return r;
}

// ---- trapezoid ---------------------------------------------------------------

static void trap_reaches_target(void)
{
    Run r = run_trap(0, MOVE_COUNTS);
    CHECK(r.final_pos == MOVE_COUNTS);
    CHECK_NEAR(r.final_vel, 0.0f, 0.0);
    CHECK(r.backsteps == 0);
}

static void trap_respects_limits(void)
{
    Run r = run_trap(0, MOVE_COUNTS);
    CHECK(r.peak_vel <= VEL_MAX * 1.0001f);
    CHECK(r.peak_accel <= ACCEL_MAX * 1.0001f);
}

// The last tick snaps the remaining 2 ticks' worth of speed to zero at once
// (2048 counts/s = 0.5 mm/s here), so that one tick runs at 2x accel_max.
// Documented behavior, not a limit violation over the move; this test pins it
// so a change to the snap logic shows up.
static void trap_final_snap(void)
{
    Run r = run_trap(0, MOVE_COUNTS);
    CHECK(r.last_accel <= 2.0f * ACCEL_MAX * 1.0001f);
}

// 10 mm at 10 mm/s with 40 ms ramps: 1.04 s, the length of the moves on the slides.
static void trap_duration(void)
{
    Run r = run_trap(0, MOVE_COUNTS);
    CHECK(abs(r.ticks - 1040) <= 2);
}

static void trap_reverse(void)
{
    Run r = run_trap(MOVE_COUNTS, 0);
    CHECK(r.final_pos == 0);
    CHECK(r.backsteps == 0);
}

// Too short to reach vel_max: triangular, peak below the limit.
static void trap_short_move_is_triangular(void)
{
    Run r = run_trap(0, 400);
    CHECK(r.final_pos == 400);
    CHECK(r.peak_vel < VEL_MAX);
}

static void trap_zero_length(void)
{
    TrapGen g;
    trap_gen_start(&g, 123, 123, VEL_MAX, ACCEL_MAX);
    CHECK(!trap_gen_active(&g));
    int32_t pos = 0; float vel = 1.0f;
    trap_gen_step(&g, DT, &pos, &vel);
    CHECK(pos == 123);
    CHECK_NEAR(vel, 0.0f, 0.0);
}

// ---- minimum jerk ------------------------------------------------------------

static void minjerk_reaches_target(void)
{
    Run r = run_minjerk(0, MOVE_COUNTS);
    CHECK(r.final_pos == MOVE_COUNTS);
    CHECK_NEAR(r.final_vel, 0.0f, 0.0);
    CHECK(r.backsteps == 0);
}

static void minjerk_respects_limits(void)
{
    Run r = run_minjerk(0, MOVE_COUNTS);
    CHECK(r.peak_vel <= VEL_MAX * 1.0001f);
    CHECK(r.peak_accel <= ACCEL_MAX * 1.01f);   // finite-difference estimate of a smooth peak
}

// Velocity-limited here: T = 1.875 * D / vel_max = 1.875 s.
static void minjerk_duration(void)
{
    MinJerkGen g;
    minjerk_gen_start(&g, 0, MOVE_COUNTS, VEL_MAX, ACCEL_MAX);
    CHECK_NEAR(g.duration, 1.875f, 1e-5);
    Run r = run_minjerk(0, MOVE_COUNTS);
    CHECK(abs(r.ticks - 1875) <= 1);
}

// Peak velocity sits at the midpoint, half way through the move.
static void minjerk_symmetric(void)
{
    MinJerkGen g;
    minjerk_gen_start(&g, 0, MOVE_COUNTS, VEL_MAX, ACCEL_MAX);
    int32_t pos = 0; float vel = 0.0f;
    int half = (int)(g.duration / DT / 2.0f);      // T/2 = 937.5 ticks: sample at 937
    for (int i = 0; i < half; i++) minjerk_gen_step(&g, DT, &pos, &vel);
    CHECK(abs(pos - MOVE_COUNTS / 2) <= VEL_MAX * DT);   // within one tick of travel of the midpoint
    CHECK_NEAR(vel, VEL_MAX, VEL_MAX * 1e-3);
}

// A long move at large counts: no step back in the last ticks (the reason
// minjerk_gen_step evaluates the polynomial in double).
static void minjerk_long_move_monotonic(void)
{
    Run r = run_minjerk(400000, 400000 + 8 * MOVE_COUNTS);
    CHECK(r.final_pos == 400000 + 8 * MOVE_COUNTS);
    CHECK(r.backsteps == 0);
}

void test_trap_gen(void)
{
    RUN(trap_reaches_target);
    RUN(trap_respects_limits);
    RUN(trap_final_snap);
    RUN(trap_duration);
    RUN(trap_reverse);
    RUN(trap_short_move_is_triangular);
    RUN(trap_zero_length);
}

void test_minjerk_gen(void)
{
    RUN(minjerk_reaches_target);
    RUN(minjerk_respects_limits);
    RUN(minjerk_duration);
    RUN(minjerk_symmetric);
    RUN(minjerk_long_move_monotonic);
}

// test.h -- minimal host unit-test helpers. No framework: each test is a
// function, CHECK records a failure and keeps going, and main() returns the
// failure count so the run script fails on any of them.

#pragma once

#include <math.h>
#include <stdio.h>

extern int g_failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
    do {                                                                     \
        double a_ = (double)(a), b_ = (double)(b);                           \
        if (fabs(a_ - b_) > (tol)) {                                         \
            printf("  FAIL %s:%d: %s = %g, expected %g (tol %g)\n",          \
                   __FILE__, __LINE__, #a, a_, b_, (double)(tol));           \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

#define RUN(test)                                                            \
    do { printf("%s\n", #test); test(); } while (0)

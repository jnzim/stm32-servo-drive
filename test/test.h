// test.h -- minimal host unit-test helpers. No framework: each test is a
// function that starts with DESCRIBE, every check prints its expression,
// the values and ok/FAIL, and main() returns the failure count so the run
// script fails on any of them.

#pragma once

#include <math.h>
#include <stdio.h>

extern int g_failures;
extern int g_checks;

// What the test checks, printed under its name.
#define DESCRIBE(text)  printf("    %s\n", text)

#define CHECK_RESULT_(ok)                                                    \
    do {                                                                     \
        g_checks++;                                                          \
        if (!(ok)) g_failures++;                                             \
    } while (0)

// Boolean condition with no values to show.
#define CHECK(cond)                                                          \
    do {                                                                     \
        int ok_ = (cond) ? 1 : 0;                                            \
        printf("      %s  %s\n", ok_ ? "ok  " : "FAIL", #cond);              \
        if (!ok_) printf("            at %s:%d\n", __FILE__, __LINE__);      \
        CHECK_RESULT_(ok_);                                                  \
    } while (0)

// a op b, printing both sides.
#define CHECK_CMP(a, op, b)                                                  \
    do {                                                                     \
        double a_ = (double)(a), b_ = (double)(b);                           \
        int ok_ = (a_ op b_);                                                \
        printf("      %s  %s %s %s   (%.10g %s %.10g)\n",                    \
               ok_ ? "ok  " : "FAIL", #a, #op, #b, a_, #op, b_);             \
        if (!ok_) printf("            at %s:%d\n", __FILE__, __LINE__);      \
        CHECK_RESULT_(ok_);                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
    do {                                                                     \
        double a_ = (double)(a), b_ = (double)(b);                           \
        int ok_ = fabs(a_ - b_) <= (tol);                                    \
        printf("      %s  %s = %.10g, expected %.10g (tol %g)\n",            \
               ok_ ? "ok  " : "FAIL", #a, a_, b_, (double)(tol));            \
        if (!ok_) printf("            at %s:%d\n", __FILE__, __LINE__);      \
        CHECK_RESULT_(ok_);                                                  \
    } while (0)

#define RUN(test)                                                            \
    do {                                                                     \
        int before_ = g_failures;                                            \
        printf("\n%s\n", #test);                                             \
        test();                                                              \
        printf("    -> %s\n", g_failures == before_ ? "PASS" : "FAIL");      \
        if (g_failures == before_) g_tests_passed++; else g_tests_failed++;  \
    } while (0)

extern int g_tests_passed, g_tests_failed;

// test_main.c -- runs every host unit test. Build and run: test/run_tests.sh
// The output is also written to build/host_tests/report.txt.

#include "test.h"

int g_failures = 0;
int g_checks = 0;
int g_tests_passed = 0, g_tests_failed = 0;

void test_pi(void);
void test_trap_gen(void);
void test_minjerk_gen(void);
void test_crc16(void);

int main(void)
{
    printf("Host unit tests: src/control.c, trap_gen.c, minjerk_gen.c, protocol.c\n");

    printf("\n==== PI / P controllers (control.c) ====\n");
    test_pi();
    printf("\n==== Trapezoid generator (trap_gen.c), 10 mm move, 10 mm/s, 250 mm/s^2 ====\n");
    test_trap_gen();
    printf("\n==== Minimum-jerk generator (minjerk_gen.c), same move and limits ====\n");
    test_minjerk_gen();
    printf("\n==== CRC-16/CCITT-FALSE (protocol.c) ====\n");
    test_crc16();

    printf("\n==== Summary ====\n");
    printf("tests:  %d passed, %d failed\n", g_tests_passed, g_tests_failed);
    printf("checks: %d run, %d failed\n", g_checks, g_failures);
    printf(g_failures ? "RESULT: FAILED\n" : "RESULT: all passed\n");
    return g_failures;
}

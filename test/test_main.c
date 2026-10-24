// test_main.c -- runs every host unit test. Build and run: test/run_tests.sh

#include "test.h"

int g_failures = 0;

void test_pi(void);
void test_trap_gen(void);
void test_minjerk_gen(void);
void test_crc16(void);

int main(void)
{
    test_pi();
    test_trap_gen();
    test_minjerk_gen();
    test_crc16();

    printf(g_failures ? "\n%d FAILED\n" : "\nall passed\n", g_failures);
    return g_failures;
}

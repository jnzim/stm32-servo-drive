#!/bin/sh
# Build and run the host unit tests with the system compiler. Prints every
# check and writes the same output to build/host_tests/report.txt. Exits
# nonzero if anything fails to build or any check fails.
set -e
cd "$(dirname "$0")/.."
mkdir -p build/host_tests
cc -std=c11 -Wall -Wextra -Werror -O1 -Iinclude -Itest \
    test/test_main.c test/test_control.c test/test_profiles.c test/test_protocol.c test/test_svpwm.c \
    src/control.c src/trap_gen.c src/minjerk_gen.c src/protocol.c src/svpwm.c \
    -lm -o build/host_tests/run_tests

report=build/host_tests/report.txt
set +e
build/host_tests/run_tests > "$report"
status=$?
cat "$report"
printf '\nreport: %s\n' "$(pwd)/$report"
exit $status

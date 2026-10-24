#!/bin/sh
# Build and run the host unit tests with the system compiler. Exits nonzero
# if anything fails to build or any check fails.
set -e
cd "$(dirname "$0")/.."
mkdir -p build/host_tests
cc -std=c11 -Wall -Wextra -Werror -O1 -Iinclude -Itest \
    test/test_main.c test/test_control.c test/test_profiles.c test/test_protocol.c \
    src/control.c src/trap_gen.c src/minjerk_gen.c src/protocol.c \
    -lm -o build/host_tests/run_tests
build/host_tests/run_tests

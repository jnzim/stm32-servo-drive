#!/usr/bin/env bash
# Build + flash the STM32, after checking it still agrees with the Pi.
#
# Workflow this supports (two VS Code windows, Cmd+Shift+B in each):
#   STM32 window (this repo) : builds firmware, flashes the board   <- this script
#   Pi window (foc-sysid)    : builds the capture tool AND runs ./drive
#
# Run this one FIRST, the Pi one SECOND. The Pi task restarts ./drive, and a
# ./drive that is already running keeps using the binary it started with --
# rebuilding while it runs changes nothing, which is how the two sides kept
# ending up mismatched.
#
# This script never writes to the Pi repo. It only reads the Pi's protocol
# source for the CRC cross-check, so Pi-side edits are safe.
set -euo pipefail

DRIVE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PI="${FOC_SYSID_HOST:-rpi5}"
MIRROR="${FOC_SYSID_MIRROR:-$HOME/dev/stm/projects/foc-sysid-rpi}"

echo "=== 1/4 check the shared CRC agrees with $PI ==="
# Only crc16_calc has to match byte-for-byte: the frame layout is declared
# separately on each side (Include/protocol.h here, a local struct in the Pi's
# src/main.cpp). CRC16_POLY (C) and the literal 0x1021 (C++) are the same value.
norm() { sed -n '/uint16_t crc16_calc/,/^}/p' "$1" | sed 's://.*::' | sed 's/CRC16_POLY/0x1021/g' | tr -d '[:space:]'; }
if ssh -o ConnectTimeout=5 "$PI" 'cat ~/foc-sysid/src/protocol.cpp' > /tmp/pi_protocol_src 2>/dev/null; then
    if [ "$(norm "$DRIVE/src/protocol.c" | shasum)" != "$(norm /tmp/pi_protocol_src | shasum)" ]; then
        echo "!!! crc16_calc differs between STM32 and Pi -- every frame would fail its check."
        echo "!!!   STM32: $DRIVE/src/protocol.c"
        echo "!!!   Pi   : $PI:~/foc-sysid/src/protocol.cpp"
        echo "!!! Not flashing."
        exit 1
    fi
    echo "    crc16_calc matches on both sides"
else
    echo "    (Pi unreachable -- skipping the cross-check)"
fi

echo "=== 2/4 build STM32 firmware ==="
cmake -S "$DRIVE" -B "$DRIVE/build" \
      -DCMAKE_TOOLCHAIN_FILE="$DRIVE/cmake/toolchain.cmake" \
      -DCMAKE_BUILD_TYPE=Debug >/dev/null
cmake --build "$DRIVE/build" -j | tail -3

echo "=== 3/4 flash board ==="
st-flash --reset write "$DRIVE/build/fw.bin" 0x08000000

echo "=== 4/4 refresh the local read-only mirror of the Pi repo ==="
rsync -a --delete --exclude .git --exclude build --exclude drive_data --exclude __pycache__ \
      "$PI":foc-sysid/ "$MIRROR"/ 2>/dev/null || echo "    (mirror refresh skipped)"

echo
echo "=== FLASH OK -- board is running this build ==="
echo "!!! NOW press Cmd+Shift+B in the Pi window."
echo "!!! A ./drive that is already running keeps using its OLD binary;"
echo "!!! skipping that step is how the two sides end up disagreeing (no data)."

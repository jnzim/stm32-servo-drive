#!/usr/bin/env python3
"""Board test: build, flash, run the Pi move queue, read the firmware
counters over SWD, and write build/board_report.txt.

    scripts/board_test.py              # build + flash + run + report
    scripts/board_test.py --no-build   # flash the existing build/fw/fw.bin

The firmware is built from this working tree, so the report header records
the commit and whether the tree was dirty. The moves are whatever is in
move_cmd.txt on the Pi. The motor moves.

Exits nonzero if the build, flash or capture fails, or any check fails.
Standard library only; needs cmake, the Arm toolchain, st-flash, openocd,
and ssh access to the Pi.
"""

import argparse
import datetime
import os
import re
import subprocess
import sys

DRIVE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(DRIVE, "build", "fw")
REPORT = os.path.join(DRIVE, "build", "board_report.txt")
PI = os.environ.get("FOC_SYSID_HOST", "rpi5")
PI_REPO = "~/foc-sysid"

# Pass limits. Everything else in the report is measured and printed, not judged.
ISR_BUDGET_CYCLES = 5000        # 50 us per 20 kHz tick at 100 MHz (isr_timing.h)
TORN_MAX_PERCENT = 1.0          # torn telemetry frames per move

COUNTERS = ["cnt_cs", "cnt_error", "cnt_telem", "cnt_rx_short", "cnt_rearm_late"]
ISR_ARRAYS = ["isr_calls", "isr_cycles_max", "isr_cycles_last"]
ISR_NAMES = ["TIM1", "ADC", "EXTI (CS)"]


def run(cmd, **kw):
    """Run cmd, return (exit code, combined output)."""
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, **kw)
    return p.returncode, p.stdout


def die(msg, output=""):
    if output:
        print(output)
    print(f"board_test: {msg}", file=sys.stderr)
    sys.exit(2)


class Report:
    def __init__(self):
        self.lines = []
        self.passed = 0
        self.failed = 0
        self._test_ok = True

    def out(self, s=""):
        self.lines.append(s)

    def test(self, title, description):
        self._test_ok = True
        self.out()
        self.out(title)
        self.out(f"    {description}")

    def check(self, ok, text):
        self.out(f"      {'ok  ' if ok else 'FAIL'}  {text}")
        if not ok:
            self._test_ok = False

    def info(self, text):
        self.out(f"            {text}")

    def end_test(self):
        self.out(f"    -> {'PASS' if self._test_ok else 'FAIL'}")
        if self._test_ok:
            self.passed += 1
        else:
            self.failed += 1


def build():
    code, out = run(["cmake", "-S", DRIVE, "-B", BUILD,
                     f"-DCMAKE_TOOLCHAIN_FILE={DRIVE}/cmake/toolchain.cmake",
                     "-DCMAKE_BUILD_TYPE=Debug"])
    if code != 0:
        die("cmake configure failed", out)
    code, out = run(["cmake", "--build", BUILD, "-j"])
    if code != 0:
        die("firmware build failed", out)


def elf_size(elf):
    _, out = run(["arm-none-eabi-size", elf])
    m = re.search(r"^\s*(\d+)\s+(\d+)\s+(\d+)", out, re.M)
    return f"text {m.group(1)} B, data {m.group(2)} B, bss {m.group(3)} B" if m else "size ?"


def symbol_addresses(elf):
    code, out = run(["arm-none-eabi-nm", elf])
    if code != 0:
        die("arm-none-eabi-nm failed", out)
    addrs = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] in COUNTERS + ISR_ARRAYS:
            addrs[parts[2]] = int(parts[0], 16)
    missing = [s for s in COUNTERS + ISR_ARRAYS if s not in addrs]
    if missing:
        die(f"symbols not found in {elf}: {', '.join(missing)}")
    return addrs


def read_counters(addrs):
    """Read every counter over SWD without halting the core."""
    cmds = ["-c", "init"]
    for s in COUNTERS:
        cmds += ["-c", f"mdw 0x{addrs[s]:08x}"]
    for s in ISR_ARRAYS:
        cmds += ["-c", f"mdw 0x{addrs[s]:08x} {len(ISR_NAMES)}"]
    cmds += ["-c", "exit"]
    code, out = run(["openocd", "-f", "interface/stlink.cfg",
                     "-f", "target/stm32f4x.cfg"] + cmds)
    if code != 0:
        die("openocd memory read failed", out)
    words = {}
    for m in re.finditer(r"^0x([0-9a-f]{8}):((?:\s+[0-9a-f]{8})+)", out, re.M):
        base = int(m.group(1), 16)
        for i, w in enumerate(m.group(2).split()):
            words[base + 4 * i] = int(w, 16)
    vals = {s: words[addrs[s]] for s in COUNTERS}
    for s in ISR_ARRAYS:
        vals[s] = [words[addrs[s] + 4 * i] for i in range(len(ISR_NAMES))]
    return vals


def pi_gpio_holder():
    code, out = run(["ssh", "-o", "ConnectTimeout=5", PI,
                     "lsof /dev/gpiochip0 2>/dev/null | awk 'NR>1 {print $2, $1}'"])
    if code not in (0, 1):
        die(f"cannot reach {PI}", out)
    return out.strip()


def run_moves():
    code, out = run(["ssh", PI, f"cd {PI_REPO}/build && printf '\\n' | timeout 120 ./move_telem 2>&1"])
    if code != 0:
        die(f"move_telem exited {code}", out)
    return out


def parse_moves(log):
    """One dict per move, from move_telem's lines and plot_move.py's lines.

    plot_move.py's output arrives first when stdout is a pipe (move_telem's
    own printf is block buffered), so the two streams are matched by order."""
    moves = []
    for m in re.finditer(r"--- move (\d+)/\d+: target=(\S+) mm.*?vel_max=(\S+) mm/s.*?"
                         r"accel_max=(\S+) mm/s\^2", log):
        moves.append({"n": int(m.group(1)), "target": float(m.group(2)),
                      "vel": float(m.group(3)), "accel": float(m.group(4))})
    # Split the move_telem part of the log at each move header.
    parts = re.split(r"--- move \d+/\d+:", log)[1:]
    for mv, part in zip(moves, parts):
        mv["fault"] = re.search(r"DRIVE FAULT.*", part)
        c = re.search(r"move complete after (\d+) frames \((\d+) torn\)", part)
        mv["frames"], mv["torn"] = (int(c.group(1)), int(c.group(2))) if c else (None, None)
        a = re.search(r"acked after (\d+) send", part)
        mv["acks"] = int(a.group(1)) if a else None
    analyses = re.findall(
        r"move: (\d+) ms, (\S+) -> (\S+) mm\s*\n"
        r"following error: peak (\S+) um, rms (\S+) um, final (\S+) um\s*\n"
        r"peak vel_cmd (\S+) mm/s, peak vel_fbk (\S+) mm/s", log)
    for mv, a in zip(moves, analyses):
        mv.update(ms=int(a[0]), start=float(a[1]), end=float(a[2]),
                  fe_peak=float(a[3]), fe_rms=float(a[4]), fe_final=float(a[5]),
                  vcmd=float(a[6]), vfbk=float(a[7]))
    return moves


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--no-build", action="store_true", help="flash the existing build")
    args = ap.parse_args()

    holder = pi_gpio_holder()
    if holder:
        die(f"GPIO on {PI} is held by: {holder}\n"
            f"  stop it first (a move_telem/drive waiting at its Enter prompt), e.g.\n"
            f"  ssh {PI} kill {holder.split()[0]}")

    _, commit = run(["git", "-C", DRIVE, "describe", "--always", "--dirty"])
    commit = commit.strip()
    if not args.no_build:
        build()
    elf = os.path.join(BUILD, "fw.elf")
    size = elf_size(elf)
    addrs = symbol_addresses(elf)

    code, out = run(["st-flash", "--reset", "write", os.path.join(BUILD, "fw.bin"), "0x08000000"])
    if code != 0 or "verified" not in out:
        die("flash failed", out)
    _, probe = run(["st-info", "--probe"])
    chip = re.search(r"dev-type:\s*(\S+)", probe)
    chipid = re.search(r"chipid:\s*(\S+)", probe)

    with open(os.path.join(DRIVE, "Include", "config.h")) as f:
        cfg = f.read()
    mode = re.search(r"#define RUN_MODE\s+(\w+)", cfg).group(1)
    profile = re.search(r"#define TRAJ_PROFILE\s+(\w+)", cfg).group(1)
    if mode != "RUN_MODE_CLOSED_LOOP":
        die(f"RUN_MODE is {mode}; the board test runs moves and needs RUN_MODE_CLOSED_LOOP")

    _, move_cmd = run(["ssh", PI, f"cat {PI_REPO}/move_cmd.txt"])
    log = run_moves()
    vals = read_counters(addrs)
    moves = parse_moves(log)

    r = Report()
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    r.out(f"BOARD TEST REPORT -- {now}")
    r.out("=" * 37)
    r.out()
    r.out(f"Board     : {chip.group(1) if chip else '?'} (chip id {chipid.group(1) if chipid else '?'})")
    r.out(f"Firmware  : {commit}  {mode}, {profile}, Debug, {size}")
    r.out(f"Flash     : st-flash --reset write fw.bin 0x08000000 -> verified")
    r.out(f"Host side : {PI} {PI_REPO}/build/move_telem")
    r.out(f"Moves     : move_cmd.txt = " + " | ".join(l.strip() for l in move_cmd.splitlines() if l.strip()))
    r.out(f"Counters  : read over SWD with openocd after the queue, core not halted")

    if not moves:
        r.test("MOVES", "move_telem output had no moves")
        r.check(False, "no '--- move' lines in the move_telem log")
        r.end_test()

    for mv in moves:
        r.test(f"TEST {mv['n']} -- move {mv['target']:+g} mm",
               f"{profile}, {mv['vel']:g} mm/s, {mv['accel']:g} mm/s^2. "
               f"Completes without a fault; telemetry torn <= {TORN_MAX_PERCENT}%.")
        r.check(not mv["fault"], "no drive fault" if not mv["fault"] else mv["fault"].group(0))
        r.check(mv["frames"] is not None, "move completed" if mv["frames"] is not None
                else "no 'move complete' line")
        if mv["frames"]:
            pct = 100.0 * mv["torn"] / mv["frames"]
            r.check(pct <= TORN_MAX_PERCENT,
                    f"torn frames {mv['torn']}/{mv['frames']} = {pct:.2f}%   (<= {TORN_MAX_PERCENT}%)")
        if "ms" in mv:
            r.info(f"duration          {mv['ms']} ms, {mv['start']:.4f} -> {mv['end']:.4f} mm")
            r.info(f"following error   peak {mv['fe_peak']:.2f} um, rms {mv['fe_rms']:.2f} um, "
                   f"final {mv['fe_final']:.2f} um")
            over = 100.0 * (mv["vfbk"] / mv["vcmd"] - 1.0) if mv["vcmd"] else 0.0
            r.info(f"peak velocity     cmd {mv['vcmd']:.3f} mm/s, fbk {mv['vfbk']:.3f} mm/s ({over:+.1f}%)")
        else:
            r.info("no plot_move.py analysis lines for this move")
        if mv["acks"] is not None:
            r.info(f"MoveCmd acked after {mv['acks']} sends")
        r.end_test()

    n = len(moves) + 1
    r.test(f"TEST {n} -- SPI link counters (spi.c)",
           "No MoveCmd CRC failures; every CS edge is a whole frame or a short one.")
    r.check(vals["cnt_error"] == 0, f"cnt_error = {vals['cnt_error']}   (MoveCmd CRC failures, == 0)")
    total = vals["cnt_telem"] + vals["cnt_rx_short"]
    r.check(total == vals["cnt_cs"],
            f"cnt_telem + cnt_rx_short = cnt_cs   ({vals['cnt_telem']} + {vals['cnt_rx_short']} = "
            f"{total}, cnt_cs {vals['cnt_cs']})")
    r.info(f"cnt_rearm_late = {vals['cnt_rearm_late']}   (re-arm finished with CS low)")
    pi_frames = sum(mv["frames"] or 0 for mv in moves)
    r.info(f"Pi received {pi_frames} frames")
    r.end_test()

    r.test(f"TEST {n + 1} -- ISR timing (isr_timing.c, DWT cycles)",
           f"Worst-case ISR time since boot against the {ISR_BUDGET_CYCLES}-cycle budget per 20 kHz tick.")
    r.out(f"                   {'calls':>8}  {'max cycles':>10}  {'last cycles':>11}")
    for i, name in enumerate(ISR_NAMES):
        r.out(f"      {name:<10} {vals['isr_calls'][i]:>9}  {vals['isr_cycles_max'][i]:>10}  "
              f"{vals['isr_cycles_last'][i]:>11}")
    for i, name in enumerate(ISR_NAMES):
        r.check(vals["isr_cycles_max"][i] < ISR_BUDGET_CYCLES,
                f"{name} max {vals['isr_cycles_max'][i]} < {ISR_BUDGET_CYCLES}")
    r.end_test()

    r.out()
    r.out("Files")
    r.out(f"    {PI}:{PI_REPO}/drive_data/latest/move_telem_<n>.csv, move_telem_plot_<n>.png")
    r.out()
    r.out("Summary")
    r.out(f"    {r.passed + r.failed} tests: {r.passed} passed, {r.failed} failed")
    r.out(f"    RESULT: {'all passed' if r.failed == 0 else 'FAILED'}")
    r.out()
    r.out("---- move_telem output ----")
    r.out(log.rstrip())

    text = "\n".join(r.lines) + "\n"
    with open(REPORT, "w") as f:
        f.write(text)
    print(text, end="")
    print(f"\nreport: {REPORT}")
    return 1 if r.failed else 0


if __name__ == "__main__":
    sys.exit(main())

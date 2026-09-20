# Motion Controller Architecture

[![Build Firmware](https://github.com/jnzim/stm32-servo-drive/actions/workflows/build.yml/badge.svg?branch=jz-dev)](https://github.com/jnzim/stm32-servo-drive/actions/workflows/build.yml)

## Status

**Full cascade closed on real hardware — current, velocity, and position — not sim. Stage
is now attached (no longer bare-motor).**

- Cascaded current (20 kHz), velocity (5 kHz), and position (1 kHz) loops are all closed
  and running on the actual STM32F411RE + DRV8353RS-EVM + AKM11E encoder stack, now driving
  a real stage rather than the bare motor.
- Every gain is derived from measured system identification, not a datasheet guess or a
  rule of thumb: chirp/step-response Bode plots and curve-fit plant models, re-run whenever
  the plant changes (e.g. the stage being attached).
- **Measurement path verified first (2026-09-19).** Two defects were found and fixed before
  any of the numbers below were trusted, because both corrupted the data every earlier
  identification run was built on:
  - The ADC converts the three phase currents sequentially. At 84-cycle sample time that
    put phase C's sample ~11 us after the trigger, outside its low-side conduction window
    above ~58% duty, so `ic` was wrong. Fixed by shortening the sample time and centering
    the 3-channel sequence on the PWM peak (`3720707`). Confirmed by a falsification run:
    with the old timing, the phase-check KCL sum breaks by -1119 mA; with the new timing it
    closes to +16 +/- 8 mA.
  - Every telemetry frame was stitched from several control ticks — the 20 kHz loop rewrote
    the DMA's live buffer mid-transmission — and the CRC meant to catch that had a
    truncation bug that made it blind to it. Fixed by writing only while the bus is idle,
    leading the frame with constant fields, and a correct table-driven CRC (`7ec52fb`).
    Torn frames went from 157637/157637 (100%) to 1/12089.

  Earlier figures in this README came through that path. Where they disagree with the
  values below, the values below are the ones that were measured after the fixes.

- **Current loop** (identified on the bare motor): plant R = 1.479 ohm, L = 1.327 mH,
  electrical pole 177 Hz. `CURRENT_LOOP_KP/KI = 2.07 / 2450`, zero-cancellation design.
  Measured closed loop: **crossover 256 Hz, PM 88.6 deg, closed-loop BW 256 Hz**, against a
  design predicting 248 Hz and 85-88 deg. Coherence >= 0.96 from 10 to 500 Hz; the ceiling
  is the 1 kHz telemetry Nyquist, not measurement quality.

- **Velocity loop** (stage attached): plant backed out of a P-only chirp with the friction
  feedforward bypassed — with it in the loop the controller is not the constant the
  back-out assumes, and the fit inverts (closed-loop DC gain reads +1.47 dB, which is
  impossible for P-only, and K comes out negative). Measured **K = 177.2 rad/s per A,
  tau = 14.19 ms (pole 11.2 Hz)**.

  The velocity gain is set by how much phase the feedback filter leaves, not by a
  bandwidth target. `VEL_FILTER_N` was 40 (a 2 ms window, 1 ms group delay, 0.36 deg/Hz
  inside the loop); backing the plant out of the measured loop showed phase passing -90 deg
  (-136.6 deg at 120 Hz), which a single-pole plant cannot do. At N = 20 that recovers to
  -115.8 deg. Shortening the filter costs velocity-measurement noise — the trade that
  drove it back to 40 previously — so it was re-checked at rest: iq 10 mA rms, vq 21 mV
  rms, against the +-400-800 mA / +-2-3 V buzz that caused the earlier revert.

  With that phase recovered, `VEL_KP/KI = 0.045 / 3.17` (zero on the measured 11.2 Hz
  pole): **crossover 86.9 Hz, PM 66.4 deg, closed-loop BW 158 Hz, peak |H| 0.03 dB**.
  `VEL_IQ_LIMIT` raised 0.5 -> 1.5 A; 0.5 A was a bring-up placeholder that clamped the PI
  for ~20% of a loaded position chirp and saturated the measurement.

- **Position loop** (stage attached): pure P around the velocity loop, `POSITION_LOOP_KP =
  260`. The binding constraint turned out to be the inner loop, not this gain: at 260 with
  the old velocity tuning the position loop crossed at 32.5 Hz with only 47.4 deg PM,
  because the velocity loop crossed at just 40 Hz — 1.2x the outer loop, where the usual
  guideline is 3-5x. Speeding the velocity loop to 86.9 Hz moved the position loop to
  **crossover 40.4 Hz, PM 60.2 deg, GM 12.2 dB at 81.9 Hz, closed-loop BW 53.5 Hz** with
  the outer gain untouched. A 1 rad step: **15 ms rise (10-90%), 0.4% overshoot, 17 ms
  settle to 2%**, repeatable across runs, with iq peaking at the 1.5 A clamp.

  Frequency-domain identification runs with the friction feedforward bypassed
  (`CL_VEL_CHIRP_DEPLOYED` / `CL_VEL_CHIRP_FF` / `CL_POS_CHIRP_DEPLOYED`): the Coulomb term
  is a tanh of vel_cmd, so it flips sign at every velocity zero crossing and no amplitude
  gives a linear Bode with it in the loop. The shipped loop keeps it, and is verified by
  step response instead.

- **Superseded by the above:** an earlier stage-attached fit of K = 1623.5 rad/s per A,
  tau = 123.67 ms, and a "mechanical resonance at 65-90 Hz" read as a two-inertia
  signature with amplitude-dependent frequency. Both came from the corrupted measurement
  path. The re-measured plant is 9x different in both K and tau, and the earlier numbers
  imply the stage *reduced* damping by 9x, which is backwards. Today's sweeps cover 65-90
  Hz with coherence >= 0.99 and show nothing there, and a step response through a 86.9 Hz
  crossover comes back with 0.4% overshoot and no ringing. `POSITION_LOOP_KP` had been
  backed off to 200 to stay clear of that band; that constraint no longer applies.

- A current-loop cross-coupling bias (`i_d` drifting with speed due to a missing d-axis
  PI) was root-caused and fixed. A persistent order-6 electrical / order-18 mechanical
  velocity ripple was characterized (real, not noise; best explanation by elimination is
  cogging torque) but not independently confirmed — a zero-current coast test to isolate
  it was tried and abandoned (too much bench friction to coast usefully). That ripple work
  also predates the measurement-path fixes and is worth re-checking.
- SPI telemetry to the Pi uses a free-running CIRC TX DMA with no CS-triggered rearm, so
  the control loop stays at **unconditional highest NVIC priority** with nothing needing to
  preempt it. The control loop now writes that buffer **only while the bus is idle** (NSS
  high) and the frame leads with constant fields, because the SPI keeps a byte fetched
  ahead: writing unconditionally every tick is what tore every frame. The Pi recovers frame
  alignment by clocking single filler bytes until the CRC checks out — a lost clock edge
  used to desync the stream permanently, which is what captures dying partway through and
  never recovering actually were. See [SPI Link](#spi-link-current).
- The Pi-streamed-trajectory loop and the full drive state machine (`drive.c`:
  IDLE/OPEN_LOOP/ALIGN/SERVO_ON/FAULT) are designed and scaffolded but **not yet wired into
  the running firmware** — see [State Machines](#state-machines) below. All loop-closure
  work above runs from the system-ID harness (`sysid/foc_sysid.c`), reusing the same
  cascade `RUN_MODE_CLOSED_LOOP` will eventually run. Next hardware steps: merge the
  trajectory-streaming branch (`MoveCmd`, `trap_gen.c`, position-loop velocity
  feedforward) onto this one and run a profile streamed from the Pi.

  Known open: two identical loaded velocity sweeps disagreed on the backed-out K by 2.3x
  (177 vs 76), so something mechanical varies run to run and is not yet explained. The SPI
  link also tears frames at higher motor current — recoverable now, but the coupling path
  is unfixed.

## Overview

This is a single-axis BLDC servo drive running on an STM32F411RE Nucleo, being brought up
and characterized against a DRV8353RS-EVM gate driver and AKM11E encoder. The end goal is a
Raspberry Pi 5 streaming trajectories over SPI while the STM32 closes nested position/
velocity/current loops in real time — but right now the firmware runs a dedicated
system-identification harness that drives the motor directly (chirp injection, step
response) and streams telemetry back to the Pi at 20 kHz for offline analysis.

There is no traditional main loop. After startup, `main()` sleeps forever in `while(1)
{ __WFI(); }`. All work happens in independent interrupt handlers — see
[Interrupts](#interrupts) below.

---

## Hardware

### MCU
- **STM32F411RE** on Nucleo-64
- 100MHz, Cortex-M4 with FPU
- Nucleo-64 morpho pinout image (F411RE label) is the correct physical reference

### Parts to Order / BOM

| Part               | Description                                      | Qty |
|--------------------|--------------------------------------------------|-----|
| NUCLEO-F411RE      | STM32F411RE Nucleo-64 board                      | 1   |
| MAX3096CPE+        | 3.3V quad differential line receiver (encoder)  | 1   |
| 120Ω resistor      | Line termination                                 | 2   |
| 0.1µF ceramic cap  | Bypass cap                                       | 1   |
| 10µF cap           | Bulk bypass cap                                  | 1   |

---

## Pin Assignment — Complete Map

| STM32 Pin | Nucleo Header | Function          | Peripheral       |
|-----------|---------------|-------------------|------------------|
| PA0       | CN8 pin 1     | Encoder A         | TIM5 CH1         |
| PA1       | CN8 pin 2     | Encoder B         | TIM5 CH2         |
| PA4       | CN8 pin 5     | Current sense PhA | ADC1 CH4         |
| PA5       | CN7 (pin TBD) | SPI1 SCLK         | SPI1             |
| PA6       | CN7 (pin TBD) | SPI1 MISO         | SPI1             |
| PA7       | CN5 pin 4     | TIM1 CH1N         | PWM Phase A low  |
| PA8       | CN9 pin 8     | TIM1 CH1          | PWM Phase A high |
| PA9       | CN5 pin 1     | TIM1 CH2          | PWM Phase B high |
| PA10      | CN9 pin 3     | TIM1 CH3          | PWM Phase C high |
| PB0       | CN8 pin 3     | TIM1 CH2N         | PWM Phase B low  |
| PB1       | CN10 pin 24   | TIM1 CH3N         | PWM Phase C low  |
| PB5       | CN7 (pin TBD) | SPI1 MOSI         | SPI1             |
| PB6       | CN7 (pin TBD) | SPI1 CS           | SPI1             |
| PB8       | CN10 pin 3    | ENABLE            | GPIO output      |
| PB12      | CN10 (pin TBD)| SPI2 NSS          | SPI2             |
| PB13      | CN10 (pin TBD)| SPI2 SCK          | SPI2             |
| PB14      | CN10 (pin TBD)| SPI2 MISO         | SPI2             |
| PB15      | CN10 (pin TBD)| SPI2 MOSI         | SPI2             |
| PC1       | CN8 pin 6     | Current sense PhB | ADC1 CH11        |
| PC4       | TBD           | Current sense PhC | ADC1 CH14        |
| PC6       | CN10 pin 4    | nFAULT            | GPIO input       |
| PC13      | CN7 pin 23    | READY to Pi       | GPIO output      |

---

## Hardware Wiring

### Pi 5 ↔ STM32 (SPI2 + READY)

Corrected 2026-09-01 — SPI2 (this table) is on **CN10**, not CN7 as previously
documented here; CN7 is actually the DRV8353's SPI1 (see below). Pin numbers
within CN10 are TBD, pending a check against the board silkscreen.

| Pi 5 Pin | Pi 5 Signal  | Wire Color | STM32 Pin | Nucleo Header  | STM32 Function     |
|----------|--------------|------------|-----------|----------------|--------------------|
| Pin 19   | GPIO 10 MOSI | Yellow     | PB15      | CN10 (pin TBD) | SPI2 MOSI          |
| Pin 9    | GND          | Black      | —         | J1 pin 3 (EVM) | GND common         |
| Pin 21   | GPIO 9 MISO  | Orange     | PB14      | CN10 (pin TBD) | SPI2 MISO          |
| Pin 22   | GPIO 25      | Green      | PC13      | CN7 pin 23     | READY (active low) |
| Pin 23   | GPIO 11 SCK  | Red        | PB13      | CN10 (pin TBD) | SPI2 SCK           |
| Pin 26   | GPIO 7 CE1   | Brown      | PB12      | CN10 (pin TBD) | SPI2 NSS           |

### STM32 ↔ DRV8353RS-EVM (3-phase PWM)

| STM32 Pin | Nucleo Header | TIM1 Channel | DRV8353RS-EVM Pin | Phase       |
|-----------|---------------|--------------|-------------------|-------------|
| PA8       | CN9 pin 8     | CH1          | J2 pin 2 (INHA)   | A high-side |
| PA7       | CN5 pin 4     | CH1N         | J2 pin 4 (INLA)   | A low-side  |
| PA9       | CN5 pin 1     | CH2          | J2 pin 6 (INHB)   | B high-side |
| PB0       | CN8 pin 3     | CH2N         | J2 pin 8 (INLB)   | B low-side  |
| PA10      | CN9 pin 3     | CH3          | J2 pin 10 (INHC)  | C high-side |
| PB1       | CN10 pin 24   | CH3N         | J2 pin 12 (INLC)  | C low-side  |

All PWM pins AF1 (TIM1). Dead time handled by DRV8353RS TDRIVE VGS monitoring.

### STM32 ↔ DRV8353RS-EVM (SPI1 — gate driver config)

Corrected 2026-09-01 — this bus is on **CN7** (the Morpho connector), not
CN10/CN9/CN5 as previously documented here. Pin numbers within CN7 are TBD,
pending a check against the board silkscreen.

| STM32 Pin | Nucleo Header | SPI1 Function | DRV8353RS-EVM Pin |
|-----------|---------------|---------------|-------------------|
| PA5       | CN7 (pin TBD) | SCLK          | J1 pin 14 (SCLK)  |
| PA6       | CN7 (pin TBD) | MISO          | J1 pin 13 (SDO)   |
| PB5       | CN7 (pin TBD) | MOSI          | J1 pin 11 (SDI)   |
| PB6       | CN7 (pin TBD) | CS            | J1 pin 17 (nSCS)  |

### STM32 ↔ DRV8353RS-EVM (Control/Fault)

| STM32 Pin | Nucleo Header | Function | DRV8353RS-EVM Pin |
|-----------|---------------|----------|-------------------|
| PB8       | CN10 pin 3    | ENABLE   | J1 pin 10         |
| PC6       | CN10 pin 4    | nFAULT   | J1 pin 16         |

ENABLE must be driven high before any PWM output. nFAULT is open-drain active low —
pullup already on EVM. Currently read as a plain GPIO status bit (`drv8353_fault()`); it is
not yet wired to an EXTI interrupt, so a fault does not automatically cut PWM in firmware
today (the DRV8353RS itself still kills its gate outputs in hardware).

### STM32 ↔ DRV8353RS-EVM (Current Sense)

| STM32 Pin | Nucleo Header | ADC1 Channel | DRV8353RS-EVM Pin | Phase |
|-----------|---------------|--------------|-------------------|-------|
| PA4       | CN8 pin 5     | CH4          | J1 pin 15 (ISENA) | A     |
| PC1       | CN8 pin 6     | CH11         | J1 pin 13 (ISENB) | B     |
| PC4       | TBD           | CH14         | J1 pin 11 (ISENC) | C     |

ADC1 injected sequence is hardware-triggered off `TIM1_TRGO`, sampling at the center of the
PWM cycle for noise-free readings — see `current_feedback.c`.

### AKM11E Encoder ↔ MAX3096 ↔ STM32

The AKM11E outputs RS-422 differential encoder signals. The MAX3096CPE+ is a 3.3V
quad differential line receiver that converts to single-ended 3.3V logic for the STM32.

**MAX3096 hookup:**

```
Encoder A+  → MAX3096 pin 2  (A1)
Encoder A-  → MAX3096 pin 1  (B1)
MAX3096 pin 3 (Y1) → PA0 (CN8 pin 1) — TIM5 CH1

Encoder B+  → MAX3096 pin 6  (A2)
Encoder B-  → MAX3096 pin 7  (B2)
MAX3096 pin 5 (Y2) → PA1 (CN8 pin 2) — TIM5 CH2
```

**MAX3096 power and enable:**

```
Pin 16 VCC  → 3.3V
Pin 8  GND  → GND
Pin 4  G    → 3.3V   (enable high)
Pin 12 /G   → GND    (enable low)
```

Outputs enabled when G=high AND /G=low.

**Passive components:**

```
120Ω termination resistor across A1/B1 (pins 1-2) at MAX3096 input
120Ω termination resistor across A2/B2 (pins 6-7) at MAX3096 input
0.1µF ceramic cap — VCC to GND, close to pin 16
10µF cap          — VCC to GND, bulk bypass
```

### GND Common

| Device | Connection     |
|--------|----------------|
| Pi 5   | Pin 9          |
| EVM    | J1 pin 3 (GND) |
| PS−    | J5 GND         |

---

## Execution Flow

### Startup — `main()` runs once

```
main()
  ├─ clock_init()                  — HSI → PLL → 100MHz
  ├─ encoder_init()                — configure TIM5 quadrature decoder
  ├─ drive_init()                  — zero drive state machine (not currently driven)
  ├─ spi_init()                    — configure SPI2 + DMA, free-running CIRC TX telemetry
  ├─ ring_init()                   — trajectory ring buffer (not currently consumed)
  ├─ (wait for user button, PC3)
  ├─ drv8353_init() / configure()  — SPI1 gate driver setup
  ├─ pwm_init()                    — TIM1 20kHz center-aligned PWM + GPIO, MOE=0
  ├─ current_feedback_init()       — ADC1 injected sequence, TIM1_TRGO hardware trigger
  ├─ drv_enable_high()             — assert DRV8353 ENABLE
  ├─ current_feedback_calibrate()  — software-start ADC offset calibration
  ├─ pwm_enable()                  — MOE=1, PWM live
  ├─ SysTick_Config(...)           — configured, but no SysTick_Handler is defined
  └─ while (1) { __WFI(); }        — main sleeps forever
```

### The only ISR driving the control loop — TIM1 at 20 kHz

```
TIM1_UP_TIM10_IRQHandler (every 50µs)
  ├─ encoder_update(tick_ms)
  └─ foc_sysid_step()              — RUN_MODE_SYSID is currently selected in config.h
       ├─ SYSID_STAGE_ALIGN  — lock rotor to theta=0 (d-axis voltage, no torque)
       ├─ SYSID_STAGE_RUN    — dispatches to the active test (config.h: SYSID_TEST),
       │                        e.g. run_cl_step(): closed current loop @ 20kHz,
       │                        closed velocity loop @ 5kHz (every 4th tick)
       └─ SYSID_STAGE_IDLE   — outputs zero, waits
```

Phase currents arrive independently via `ADC_IRQHandler`, which fires on the ADC's own
`JEOC` (injected end-of-conversion) interrupt — hardware-triggered by `TIM1_TRGO`, not
polled from the TIM1 ISR. Telemetry to the Pi is likewise independent: the TIM1 ISR writes
each sample straight into a single live buffer (`spi_sysid_update_latest()`), unconditionally,
every tick — no CS check, no double-buffering. A free-running circular TX DMA continuously
re-shifts whatever is currently in that buffer out over SPI2, entirely in hardware; the Pi
always reads the latest committed sample with zero dependency on any ISR's latency or
priority. The tradeoff is a torn/misaligned frame if a write lands mid-transaction — caught
by a CRC the Pi checks and discards on mismatch, not by any framing/rearm logic in the ISR.

---

## Nested Time Scales (currently active)

```
50µs  ── current loop + PWM update, every TIM1 tick (20 kHz)
200µs ── velocity loop, every 4th TIM1 tick (5 kHz)
1ms   ── position loop, every 20th TIM1 tick (1 kHz) -- only when SYSID_TEST selects a
          closed-position-loop test (e.g. SYSID_TEST_CL_POS_CHIRP); not part of the
          unconditional cascade the way current/velocity are
```

Position-loop closure has been verified (see [Status](#status)), but it only runs inside
the sysid harness when explicitly selected — the always-on Pi-trajectory-streaming cascade
(`RUN_MODE_CLOSED_LOOP`) is still not wired up.

---

## Interrupts

| IRQ                  | Priority | Rate      | Role                                          |
|----------------------|----------|-----------|------------------------------------------------|
| `TIM1_UP_TIM10_IRQn` | 0 (highest, unconditional) | 20 kHz | Encoder update, sysid stage machine, current + velocity loops |
| `ADC_IRQn`           | 1        | 20 kHz    | Reads phase currents on injected-conversion complete |
| `DMA1_Stream3_IRQn`  | 2        | per pkt   | SPI2 RX-complete from Pi (currently counts only) |
| `EXTI15_10_IRQn`     | 2        | per pkt   | SPI2 CS edge — stats only (`cnt_cs`); no rearm, nothing time-critical since TX is free-running |

Telemetry priority is no longer load-bearing for correctness the way it once was: the TX
DMA is free-running (see [SPI Link](#spi-link-current)), so nothing in the SPI path has a
deadline the control loop could ever be blocked by, or that could itself be starved into a
stale/corrupt frame. `TIM1_UP_TIM10_IRQn` is the unconditional highest priority in the
system precisely because nothing else needs to preempt it anymore.

`SysTick` is configured at boot but has no handler defined, so nothing currently runs at
1 kHz.

---

## State Machines

There are two state machines in this codebase — only one of them is actually running.

### Active — sysid stage sequence (`sysid/foc_sysid.c`)

This is what the firmware runs today, driven directly from the TIM1 ISR:

```
SYSID_STAGE_ALIGN → SYSID_STAGE_RUN → SYSID_STAGE_IDLE
     (lock rotor)     (dispatches to the      (outputs zero,
                        active test, e.g.       waits)
                        run_cl_step())
```

The active test is selected at compile time via `SYSID_TEST` in `config.h` — options span
open-loop current chirp/step, closed-velocity-loop chirp/step, a constant-iq ripple-debug
mode, closed-position-loop step and chirp (whole-system, for the 53.5 Hz BW / 60.2° PM
result above), and a slow "cine sweep" variant of the position chirp sized for filming
(visible 3-80 Hz sweep instead of the analysis range) rather than measurement.

### Designed, not yet wired up — `drive.c`

A full drive state machine exists for the eventual Pi-trajectory-streaming mode, but
`drive_sm_run()` is never called anywhere in the firmware (it was written to run from
`SysTick`, which has no handler). It's left in place as the target architecture for
`RUN_MODE_CLOSED_LOOP`:

```
         open_loop_req                    servo_on_req
              │                                │
    ┌─────────▼──────────┐                     │
    │      STATE_IDLE     │◄────────────────────┼──────────────┐
    └──┬──────────────────┘                     │              │
       │                            ┌───────────▼──────────┐   │
       │                            │    STATE_SERVO_ON     │───┘
    ┌──▼──────────────────┐         │  (closed-loop FOC)   │ ring empty
    │   STATE_OPEN_LOOP   │         └───────────────────────┘ pwm_disable()
    │  (no feedback)      │
    └──┬──────────────────┘
       │ stop_req / pwm_disable()
       ▼
    ┌─────────────────────┐
    │    STATE_FAULT       │  ← fault_req from any state
    │  (latched, PWM off)  │
    └─────────────────────┘
```

(A second, newer prototype FSM, `servo_sm.c`, also exists but isn't part of the build at
all — it's excluded from `CMakeLists.txt` and doesn't currently compile.)

---

## PWM — `pwm.c`

- `pwm_init()` — TIM1 + GPIO, MOE=0
- `pwm_enable()` — set MOE
- `pwm_disable()` — clear MOE
- `pwm_apply_dq(v_d, v_q, theta)` — inverse Park + Clarke → CCR1/2/3
- V_BUS = 12V

---

## SPI Link (current)

The active telemetry path is a fixed-format, continuous sample stream — not the
opcode-driven protocol described below (that protocol exists in `protocol.h` but is only
referenced by the currently-inactive `drive.c`/ring-buffer path).

- **Packet size:** 32 bytes, full duplex (`SysIdSample` struct, size-checked at compile time)
- **CS:** Pi GPIO7 (pin 26), manual, toggles per packet — stats only (`cnt_cs`) on the STM
  side, doesn't gate or rearm anything
- **STM mode:** SPI2 slave, `CPHA=1`, RX/TX DMA, no software framing/opcodes
- **TX is free-running CIRC** over a single live buffer: every 20 kHz tick, the TIM1 ISR
  writes a fresh sample (position, id/iq, vd/vq, theta, phase currents, stage flags) straight
  into it, unconditionally, and DMA continuously re-transmits whatever's currently there. The
  Pi always gets the latest committed sample with zero dependency on any ISR's latency or
  priority — this replaced an earlier design where a CS-edge interrupt had to rearm the TX
  DMA between transactions, which both caused a multi-ms freeze bug (fixed) and, even after
  that fix, forced telemetry to run at NVIC priority 0 above the control loop to hit its
  timing (also since removed).
- **Tradeoff:** a write landing mid-transaction can hand the Pi a torn/misaligned frame.
  Caught purely by the `crc` field in `SysIdSample` — same CCITT CRC used for `TrajSlot` — the
  Pi discards a CRC mismatch and resyncs by trying other byte rotations of what it read,
  rather than any framing logic on the STM side.

### Opcode protocol (designed, not currently consumed by the active SPI path)

| Opcode     | Value | Description                       |
|------------|-------|-----------------------------------|
| NOP        | 0x00  | No-op                             |
| BLOCK_HDR  | 0x03  | Start trajectory → STATE_SERVO_ON |
| DATA       | 0x04  | Trajectory sample packet          |
| READY_ACK  | 0x05  | Pi acknowledges READY signal      |
| TELEM_REQ  | 0x06  | Pi requests telemetry frame       |
| OPEN_LOOP  | 0x07  | Start open-loop → STATE_OPEN_LOOP |
| STOP       | 0x08  | Stop → STATE_IDLE                 |

---

## File Map

| File                      | Owns                                                          |
|---------------------------|----------------------------------------------------------------|
| `main.c`                  | Startup, TIM1 ISR entry point                                 |
| `clock.c`                 | HSI → PLL → 100MHz                                            |
| `tim1.c`                  | TIM1 timebase config (center-aligned PWM + update IRQ)        |
| `pwm.c`                   | GPIO/AF setup, enable/disable, `pwm_apply_dq`                 |
| `current_feedback.c`      | ADC1 injected sequence, `ADC_IRQHandler`, calibration         |
| `encoder.c`               | TIM5 quadrature decode, velocity filter                       |
| `drv8353.c`               | SPI1 gate driver config/status                                |
| `spi.c`                   | SPI2 + DMA telemetry link to the Pi                           |
| `sysid/foc_sysid.c`       | **Active** — sysid stage machine, all current test modes      |
| `sysid/foc_trajectory.c`  | Trajectory-following step (for `RUN_MODE_CLOSED_LOOP`, unused today) |
| `loops.c` / `control.c`   | PI controller state and step functions shared by both modes  |
| `drive.c`                 | Designed drive FSM — not currently called (see State Machines) |
| `servo_sm.c`              | Prototype FSM — not in the build                              |
| `ringBuffer.c`, `plant.c` | Ring buffer + simulated plant, used by the not-yet-active trajectory path |
| `protocol.h`              | `TrajSample`, `TelemetryFrame`, opcodes, drive states (design surface for `RUN_MODE_CLOSED_LOOP`) |

---

## Bringup Sequence

1. Rotor lock / alignment — **done**
2. Open-loop electrical spin / encoder polarity check — **done**
3. Closed-loop current validation — **done**, gains from measured system ID; d-axis
   cross-coupling PI added. Re-verified 2026-09-19 after the ADC-sampling and telemetry
   fixes: crossover 256 Hz, PM 88.6°, matching the design's 248 Hz / 85-88°
4. Closed-loop velocity — **done**, stage attached, re-identified 2026-09-19 on the
   verified measurement path with the friction feedforward bypassed: K = 177.2 rad/s per A,
   τ = 14.19 ms; loop at 86.9 Hz crossover, 66.4° PM. An order-6 electrical velocity ripple
   was characterized earlier (likely cogging) but not independently root-caused, and
   predates the measurement fixes
5. Closed-loop position, via the sysid harness — **done**, `POSITION_LOOP_KP=260`. Measured
   directly on the whole closed system (`SYSID_TEST_CL_POS_CHIRP`, not inferred): 40.4 Hz
   crossover, 60.2° phase margin, 12.2 dB gain margin, 53.5 Hz closed-loop bandwidth; 1 rad
   step in 15 ms with 0.4% overshoot. The earlier back-off to `POSITION_LOOP_KP=200` was to
   avoid a 65-90 Hz resonance that the verified path does not reproduce — see
   [Status](#status)
6. Closed-loop position via Pi-streamed trajectories (`RUN_MODE_CLOSED_LOOP`, trapezoidal
   velocity profiles) — **not started**; next step is merging the trajectory-streaming
   branch onto `jz-dev` and running a profile from the Pi's `move_cmd.txt`

<!-- Images pending: MCU/board photos, pinout reference to be added by user -->

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
- **Current loop:** re-identified against a fresh chirp (R=1.65Ω, L=1.41mH line-to-line),
  BW target bumped from 500Hz to 1000Hz — the 500Hz design had real margin to spare
  (PM=85-88°, heavily overdamped). `CURRENT_LOOP_KP/KI = 4.43 / 5184`.
- **Velocity loop:** re-identified with the stage attached — the plant changed substantially
  from the bare-motor fit (K=1623.5 rad/s/A, τ=123.67ms now, vs. a much lighter/faster bare
  motor before). `VEL_KP/KI = 0.0239 / 0.1935`, zero-cancellation design, 50Hz target.
- **Position loop:** a phase-margin-targeted design (crossover picked from the closed
  velocity loop's measured phase, not assumed) kept landing its crossover around 41-43Hz —
  right against a **real mechanical resonance measured at 65-90Hz** in the closed velocity
  loop's frequency response (dip-then-peak shape in `vel_meas/vel_cmd`, the classic
  signature of a two-inertia system — motor and load coupled through the screw's
  compliance). An amplitude sweep (0.35/0.5/0.75/0.9A) showed that resonance's frequency and
  damping both move non-monotonically with excitation amplitude, which a fixed linear mode
  wouldn't do — most likely backlash in the coupling modulating effective stiffness under
  load, not yet confirmed or fixed mechanically. Rather than chase a precise design against
  a moving target (or add a notch filter tuned to a resonance that's already been observed
  to shift), the position loop was deliberately backed off to `POSITION_LOOP_KP=200`
  (crossover ~15-20Hz, clear of the resonance band). Verified against the entire closed
  position loop (`SYSID_TEST_CL_POS_CHIRP`), reconstructing the open-loop response directly
  from measured closed-loop data (`L = H/(1-H)`): **21.5 Hz closed-loop bandwidth, 82.8°
  phase margin, 18.6 dB gain margin**, with the open-loop reconstruction rolling off
  smoothly straight through the 65-90Hz resonance band — no bump at all at this crossover.
  Velocity feedforward (not yet implemented) is the planned way to recover tracking
  performance without spending this margin back.
- A current-loop cross-coupling bias (`i_d` drifting with speed due to a missing d-axis
  PI) was root-caused and fixed. A persistent order-6 electrical / order-18 mechanical
  velocity ripple was characterized (real, not noise; best explanation by elimination is
  cogging torque) but not independently confirmed — a zero-current coast test to isolate
  it was tried and abandoned (too much bench friction to coast usefully). The mechanical
  resonance above is a separate, later finding — not yet connected to the ripple
  investigation, though both point at unmodeled mechanical/friction behavior worth
  revisiting together.
- SPI telemetry to the Pi was redesigned to remove an entire class of ISR-priority bug: the
  TX DMA is now free-running CIRC over a single live buffer that the control loop writes
  unconditionally every tick — no CS-triggered rearm, no dependency on any ISR's latency or
  priority. This is what lets the control loop sit at **unconditional highest NVIC
  priority** with nothing else ever needing to preempt it. See [SPI Link](#spi-link-current).
- The Pi-streamed-trajectory loop and the full drive state machine (`drive.c`:
  IDLE/OPEN_LOOP/ALIGN/SERVO_ON/FAULT) are designed and scaffolded but **not yet wired into
  the running firmware** — see [State Machines](#state-machines) below. All loop-closure
  work above runs from the system-ID harness (`sysid/foc_sysid.c`), reusing the same
  cascade `RUN_MODE_CLOSED_LOOP` will eventually run. Next hardware steps: check the stage
  coupling for backlash (the suspected source of the amplitude-dependent resonance above),
  add velocity feedforward, then build the trajectory-streaming motion controller
  (trapezoidal velocity profiles) on top.

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
mode, closed-position-loop step and chirp (whole-system, for the 21.5 Hz BW / 82.8° PM
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
3. Closed-loop current validation — **done**, gains from measured system ID (re-identified
   again this session, BW target bumped 500→1000Hz), d-axis cross-coupling PI added
4. Closed-loop velocity — **done**, re-identified with a real stage attached (previous
   82.5 Hz bare-motor result superseded — the plant changed substantially with load); a real
   order-6 electrical velocity ripple was characterized (likely cogging) but not
   independently root-caused, and a separate mechanical resonance (65-90Hz, amplitude-
   dependent, likely coupling backlash) was found via closed-loop chirp — see
   [Status](#status)
5. Closed-loop position, via the sysid harness — **done**, deliberately backed off from a
   phase-margin-targeted design (crossover was landing right against the resonance above) to
   a conservative `POSITION_LOOP_KP=200`. Measured directly on the whole closed system
   (`SYSID_TEST_CL_POS_CHIRP`, not inferred): 21.5 Hz closed-loop bandwidth, 82.8° phase
   margin, 18.6 dB gain margin
6. Closed-loop position via Pi-streamed trajectories (`RUN_MODE_CLOSED_LOOP`, trapezoidal
   velocity profiles) — **not started**; next steps are checking the stage coupling for
   backlash and adding velocity feedforward before building this

<!-- Images pending: MCU/board photos, pinout reference to be added by user -->

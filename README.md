# STM32 FOC Servo Drive

[![Build Firmware](https://github.com/jnzim/stm32-servo-drive/actions/workflows/build.yml/badge.svg?branch=jz-dev)](https://github.com/jnzim/stm32-servo-drive/actions/workflows/build.yml)

Bare metal field oriented control for a single servo axis: an STM32F411 driving a TI DRV8353RS
gate driver and a Kollmorgen AKM11E motor on a ball screw stage. No vendor motor control SDK.
Every loop is designed from frequency response data measured on the hardware.

**Write-up with plots and video: https://jnzim.github.io/foc-servo/**

## Results

| Loop     | Rate   | Crossover | Phase margin |
|----------|--------|-----------|--------------|
| Current  | 20 kHz | 504 Hz    | 72.7°        |
| Velocity | 5 kHz  | 117 Hz    | 67.0°        |
| Position | 1 kHz  | 61 Hz     | 61.2°        |

A 10 mm trapezoidal move at 10 mm/s tracks to ±0.73 µm (3 encoder counts) at cruise, with
friction and velocity feedforward.

## Hardware

- STM32F411RE Nucleo
- TI DRV8353RS gate driver, low side shunt current sensing
- Kollmorgen AKM11E with an incremental encoder (8192 counts/rev), driving a 2 mm lead ball screw stage
- Raspberry Pi 5 over SPI: sends move commands, logs telemetry, runs the analysis

## How it works

- **PWM and current sensing:** TIM1 runs 20 kHz center aligned PWM. The same timer triggers the
  ADC injected group at the counter peak, the middle of the low side on time, so phase currents
  are sampled while the shunts carry them.
- **Loops:** current PI every PWM tick, velocity PI every 4th tick, position P at 1 kHz.
  Friction (Coulomb + viscous) and velocity are fed forward.
- **Encoder:** read in hardware by TIM2 in encoder mode.
- **Telemetry:** SPI2 streams 32 byte frames with a CRC to the Pi. The Pi sends move commands back
  on the same link.

## Layout

```
src/
  main.c               ISR wiring: TIM1 update (control tick), SysTick (1 kHz)
  pwm.c                TIM1 center aligned PWM, ADC trigger, inverse Park/Clarke
  current_feedback.c   ADC injected conversions, Clarke/Park
  loops.c              current, velocity, and position controllers, friction feedforward
  observer.c           velocity observer
  encoder.c            TIM2 encoder
  trap_gen.c           trapezoidal move generator
  minjerk_gen.c        minimum jerk move generator
  drive.c              drive state machine
  drv8353.c            gate driver setup and fault monitoring over SPI1
  spi.c                SPI2 telemetry and move commands (DMA)
  sysid/
    foc_sysid.c        system identification tests (chirps, steps, friction sweep)
    foc_trajectory.c   commanded moves from the Pi
Include/config.h       gains, limits, and test selection, each with how it was measured
```

## Build and flash

Requires the Arm GNU toolchain, CMake, and `st-flash`.

```
cmake -S . -B build
cmake --build build
st-flash --reset write build/fw.bin 0x8000000
```

`scripts/build_all.sh` builds and flashes after checking the telemetry CRC matches the Pi side.

`Include/config.h` selects the mode:
- `RUN_MODE`: `RUN_MODE_SYSID` for identification tests, `RUN_MODE_CLOSED_LOOP` for commanded moves
- `TRAJ_PROFILE`: `TRAJ_PROFILE_TRAP` or `TRAJ_PROFILE_MINJERK`

## Analysis

The Pi side (telemetry capture, Bode analysis, model fits) is in
[jnzim/foc-sysid](https://github.com/jnzim/foc-sysid).

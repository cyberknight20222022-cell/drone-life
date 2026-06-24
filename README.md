# drone-life

A custom PCB drone built around the **Seeed Studio XIAO RP2040** microcontroller.

## Overview

This project is a fully custom PCB drone that uses the XIAO RP2040 as its flight controller brain. The RP2040 dual-core microcontroller handles motor control, sensor fusion, and flight stabilisation.

## Hardware

| Component | Details |
|-----------|---------|
| MCU | Seeed Studio XIAO RP2040 (RP2040, dual-core Cortex-M0+, 133 MHz) |
| IMU | MPU-6050 (6-axis accelerometer + gyroscope) via I2C |
| Motors | 4× brushless DC motors with ESCs (PWM) |
| Power | 1S/2S LiPo battery with onboard 3.3 V regulator |
| Frame | Custom PCB frame (PCB is the structural body) |

### XIAO RP2040 Pin Mapping

| XIAO Pin | GPIO | Function |
|----------|------|----------|
| D0 | GP26 | Motor 1 PWM (ESC) |
| D1 | GP27 | Motor 2 PWM (ESC) |
| D2 | GP28 | Motor 3 PWM (ESC) |
| D3 | GP29 | Motor 4 PWM (ESC) |
| D4 | GP6  | I2C SDA (IMU) |
| D5 | GP7  | I2C SCL (IMU) |
| D6 | GP0  | UART TX (debug) |
| D7 | GP1  | UART RX (debug) |

## Firmware

The firmware is written in MicroPython and located in the [`firmware/`](firmware/) directory.

### Structure

```
firmware/
├── main.py          # Entry point
├── imu.py           # MPU-6050 driver
├── motors.py        # Motor/ESC PWM control
├── pid.py           # PID controller
└── flight.py        # Flight control loop
```

### Getting Started

1. Install [MicroPython for RP2040](https://micropython.org/download/rp2-pico/) on the XIAO RP2040.
2. Copy all files from `firmware/` to the XIAO using a tool such as [Thonny](https://thonny.org/) or `mpremote`.
3. Power cycle the board — `main.py` runs automatically on boot.

## Status

- [x] PCB designed and manufactured
- [x] Firmware skeleton
- [ ] First flight test
- [ ] PID tuning

## License

MIT

"""Entry point for the XIAO RP2040 PCB drone firmware.

Runs automatically on boot when MicroPython starts.
"""

import time
from machine import I2C, Pin
from flight import FlightController

# I2C bus: SDA=GP6 (D4), SCL=GP7 (D5)
i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=400_000)

fc = FlightController(i2c)

# Arm ESCs
fc.arm()

loop_hz = 100          # target loop rate
loop_period_ms = 1000 // loop_hz
dt = loop_period_ms / 1000.0

throttle = 0.0         # set > 0.0 to spin motors; kept at 0 by default for safety

print("Flight controller ready. Loop rate:", loop_hz, "Hz")

while True:
    t0 = time.ticks_ms()
    fc.run_once(throttle, dt)
    elapsed = time.ticks_diff(time.ticks_ms(), t0)
    sleep_ms = loop_period_ms - elapsed
    if sleep_ms > 0:
        time.sleep_ms(sleep_ms)

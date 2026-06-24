"""Motor / ESC PWM control for XIAO RP2040 (MicroPython).

Standard ESC protocol: 1000 µs (min throttle) – 2000 µs (max throttle)
at 50 Hz update rate.
"""

from machine import Pin, PWM

# PWM frequency for ESCs (50 Hz is standard)
_ESC_FREQ = 50

# Pulse widths in microseconds
_ESC_MIN_US = 1000
_ESC_MAX_US = 2000
_ESC_ARM_US = 1000

# At 50 Hz the period is 20 000 µs.  MicroPython PWM duty_u16 uses a 16-bit
# scale (0–65535) representing 0–100 % of the period.
_PERIOD_US = 1_000_000 // _ESC_FREQ  # 20 000 µs


def _us_to_duty(us: int) -> int:
    return int(us * 65535 // _PERIOD_US)


class Motors:
    """Controls four ESC-driven motors via PWM.

    Motor layout (top view):
        M1 (front-left)   M2 (front-right)
        M3 (rear-right)   M4 (rear-left)
    """

    # XIAO RP2040 GPIO pins for motors
    _PINS = (26, 27, 28, 29)

    def __init__(self) -> None:
        self._pwms = [PWM(Pin(p)) for p in self._PINS]
        for pwm in self._pwms:
            pwm.freq(_ESC_FREQ)
            pwm.duty_u16(_us_to_duty(_ESC_ARM_US))

    def set_throttle(self, m1: float, m2: float, m3: float, m4: float) -> None:
        """Set throttle for each motor.

        Args:
            m1..m4: throttle in the range [0.0, 1.0].
        """
        values = (m1, m2, m3, m4)
        for pwm, val in zip(self._pwms, values):
            val = max(0.0, min(1.0, val))
            us = int(_ESC_MIN_US + val * (_ESC_MAX_US - _ESC_MIN_US))
            pwm.duty_u16(_us_to_duty(us))

    def arm(self) -> None:
        """Send minimum throttle signal to arm all ESCs."""
        self.set_throttle(0.0, 0.0, 0.0, 0.0)

    def stop(self) -> None:
        """Cut power to all motors immediately."""
        for pwm in self._pwms:
            pwm.duty_u16(0)

"""Flight control loop for XIAO RP2040 PCB drone."""

import math
import time
from imu import MPU6050
from motors import Motors
from pid import PID


# ------------------------------------------------------------------
# Tuning parameters (adjust during test flights)
# ------------------------------------------------------------------
ROLL_KP = 0.5
ROLL_KI = 0.01
ROLL_KD = 0.1

PITCH_KP = 0.5
PITCH_KI = 0.01
PITCH_KD = 0.1

# Base throttle (0.0 – 1.0); raise until the drone lifts off
BASE_THROTTLE = 0.0


def mix(throttle: float, roll: float, pitch: float) -> tuple:
    """Mix throttle + roll + pitch corrections into four motor values.

    Motor layout (top view, X-frame):
        M1 (front-left, CCW)   M2 (front-right, CW)
        M3 (rear-right, CCW)   M4 (rear-left, CW)
    """
    m1 = throttle - roll + pitch
    m2 = throttle + roll + pitch
    m3 = throttle + roll - pitch
    m4 = throttle - roll - pitch
    return m1, m2, m3, m4


class FlightController:
    def __init__(self, i2c) -> None:
        self._imu = MPU6050(i2c)
        self._motors = Motors()
        self._roll_pid = PID(ROLL_KP, ROLL_KI, ROLL_KD, output_limits=(-0.3, 0.3))
        self._pitch_pid = PID(PITCH_KP, PITCH_KI, PITCH_KD, output_limits=(-0.3, 0.3))

    def arm(self) -> None:
        self._motors.arm()
        time.sleep_ms(2000)

    def stop(self) -> None:
        self._motors.stop()

    def run_once(self, throttle: float, dt: float) -> None:
        """Execute one iteration of the stabilisation loop."""
        ax, ay, az = self._imu.read_accel()

        # Roll: rotation around the X-axis; Pitch: rotation around the Y-axis.
        # Using the standard formulas for accurate results at any tilt angle.
        roll_measured = math.atan2(ay, az)
        pitch_measured = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        roll_out = self._roll_pid.update(roll_measured, dt)
        pitch_out = self._pitch_pid.update(pitch_measured, dt)

        m1, m2, m3, m4 = mix(throttle, roll_out, pitch_out)
        self._motors.set_throttle(m1, m2, m3, m4)

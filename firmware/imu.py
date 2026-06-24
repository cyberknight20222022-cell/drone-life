"""MPU-6050 driver for XIAO RP2040 (MicroPython)."""

from machine import I2C
import struct

MPU6050_ADDR = 0x68

# Register addresses
_PWR_MGMT_1 = 0x6B
_ACCEL_XOUT_H = 0x3B
_GYRO_XOUT_H = 0x43
_WHO_AM_I = 0x75

_ACCEL_SCALE = 16384.0  # ±2 g (default)
_GYRO_SCALE = 131.0     # ±250 °/s (default)


class MPU6050:
    def __init__(self, i2c: I2C, addr: int = MPU6050_ADDR) -> None:
        self._i2c = i2c
        self._addr = addr
        self._wake()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _write(self, reg: int, value: int) -> None:
        self._i2c.writeto_mem(self._addr, reg, bytes([value]))

    def _read(self, reg: int, length: int) -> bytes:
        return self._i2c.readfrom_mem(self._addr, reg, length)

    def _read_word(self, reg: int) -> int:
        raw = self._read(reg, 2)
        value = struct.unpack(">h", raw)[0]
        return value

    def _wake(self) -> None:
        # Clear sleep bit to wake the sensor
        self._write(_PWR_MGMT_1, 0x00)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def who_am_i(self) -> int:
        return self._read(_WHO_AM_I, 1)[0]

    def read_accel(self) -> tuple:
        """Return (ax, ay, az) in g."""
        raw = self._read(_ACCEL_XOUT_H, 6)
        ax, ay, az = struct.unpack(">hhh", raw)
        return ax / _ACCEL_SCALE, ay / _ACCEL_SCALE, az / _ACCEL_SCALE

    def read_gyro(self) -> tuple:
        """Return (gx, gy, gz) in degrees/second."""
        raw = self._read(_GYRO_XOUT_H, 6)
        gx, gy, gz = struct.unpack(">hhh", raw)
        return gx / _GYRO_SCALE, gy / _GYRO_SCALE, gz / _GYRO_SCALE

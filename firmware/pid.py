"""Simple PID controller."""


class PID:
    """Discrete PID controller.

    Args:
        kp: Proportional gain.
        ki: Integral gain.
        kd: Derivative gain.
        setpoint: Desired value.
        output_limits: (min, max) tuple to clamp the output.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float = 0.0,
        output_limits: tuple = (-1.0, 1.0),
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._min_out, self._max_out = output_limits

        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, measured: float, dt: float) -> float:
        """Compute the PID output.

        Args:
            measured: Current measured value.
            dt: Time step in seconds since the last call.

        Returns:
            Control output clamped to output_limits.
        """
        if dt <= 0:
            return 0.0

        error = self.setpoint - measured

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        # Accumulate integral only when the output is not saturated
        # (anti-windup: conditional integration)
        unsaturated_output = self.kp * error + self.ki * self._integral + self.kd * derivative
        if self._min_out < unsaturated_output < self._max_out:
            self._integral += error * dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self._min_out, min(self._max_out, output))

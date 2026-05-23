"""
General PID controller (MicroPython compatible)

Standard PID algorithm with integral accumulation and output clamping.
Replaces hardware/PIDModule.py (which had a non-accumulating integral bug).
"""
from utils.helpers import clamp


class PID:
    """Standard PID controller"""

    def __init__(self, kp, ki, kd, output_limit=None):
        """
        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :param output_limit: Output clamp (absolute value), None = no limit
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.last_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, measurement, dt):
        """
        Compute PID output

        :param setpoint: Target value
        :param measurement: Current measured value
        :param dt: Time step (seconds)
        :return: Control output
        """
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.output_limit is not None:
            output = clamp(output, -self.output_limit, self.output_limit)
        return output

    def reset(self):
        """Reset integral and derivative state"""
        self.last_error = 0.0
        self.integral = 0.0

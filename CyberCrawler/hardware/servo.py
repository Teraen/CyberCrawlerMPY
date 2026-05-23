"""
Servo control abstraction layer
Drives standard servos (angle 0-180°) and 360° continuous rotation servos (speed control) via PCA9685
"""
from calibration import SERVO_DUTY_MIN, SERVO_DUTY_MAX, WHEEL


class ServoController:
    def __init__(self, pca9685):
        """
        :param pca9685: PCA9685 instance (frequency already initialized)
        """
        self.pwm = pca9685

    def set_angle(self, channel, angle, trim=0):
        """
        Set standard servo angle
        :param channel: PCA9685 channel number
        :param angle: Target angle 0-180°
        :param trim: Trim offset (angle)
        """
        angle = max(0, min(180, angle + trim))
        duty = int(SERVO_DUTY_MIN + (angle / 180.0) * (SERVO_DUTY_MAX - SERVO_DUTY_MIN))
        self.pwm.set_pwm(channel, 0, duty)

    def set_wheel_speed(self, channel, speed, trim=0):
        """
        Set 360° wheel servo speed
        :param channel: PCA9685 channel number
        :param speed: -1.0 ~ 1.0 (negative=reverse, 0=stop, positive=forward)
        :param trim: Duty trim offset for calibrating neutral position
        """
        speed = max(-1.0, min(1.0, speed))
        duty = int(WHEEL['neutral_duty'] + trim + speed * WHEEL['duty_range'])
        self.pwm.set_pwm(channel, 0, duty)

    def disable(self, channel):
        """Disable the specified channel (output 0)"""
        self.pwm.set_pwm(channel, 0, 0)

    def disable_all(self):
        """Disable all channels (0-15)"""
        for ch in range(16):
            self.disable(ch)

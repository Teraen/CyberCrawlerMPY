"""
IMU wrapper
Based on MPU6050 + Kalman1D filter, fusion outputs stable pitch / roll angles
Supports configurable axis mapping (adapts to different mounting orientations)
"""
from hardware.MPU6050 import MPU6050
from hardware.kalman1Dfilter import Kalman1D
from calibration import IMU_AXIS_MAP
import time


class IMU:
    def __init__(self, i2c):
        self.mpu = MPU6050(i2c)
        self.mpu.wake()
        time.sleep_ms(100)

        # Two independent Kalman filters: pitch / roll
        self.kf_pitch = Kalman1D()
        self.kf_roll = Kalman1D()

        self.pitch = 0.0
        self.roll = 0.0

        # Gyroscope zero bias (filled by calibrate_gyro())
        self.gyro_bias = (0.0, 0.0, 0.0)

    def calibrate_gyro(self, samples=100):
        """
        Calibrate gyroscope zero bias while stationary
        Averages samples readings as the zero bias
        """
        gx_sum = 0.0
        gy_sum = 0.0
        gz_sum = 0.0
        for _ in range(samples):
            gx, gy, gz = self.mpu.read_gyro_data()
            gx_sum += gx
            gy_sum += gy
            gz_sum += gz
            time.sleep_ms(5)
        n = float(samples)
        self.gyro_bias = (gx_sum / n, gy_sum / n, gz_sum / n)

    def _apply_axis_map(self, gx, gy, gz, ax, ay, az):
        """Adjust axis signs according to axis mapping"""
        gs = IMU_AXIS_MAP['gyro_sign']
        as_ = IMU_AXIS_MAP['accel_sign']
        return (
            gx * gs[0], gy * gs[1], gz * gs[2],
            ax * as_[0], ay * as_[1], az * as_[2],
        )

    def update(self, dt):
        """
        Update IMU data
        MPU6050 mounting orientation: Y axis forward, X axis right (relative to chassis)
        MPU6050 chip native: X forward, Y left
        Therefore chip native coordinate system has a 90° rotation relative to chassis:
          - Chip X → chassis right
          - Chip Y → chassis front
          - Chip Z → chassis up
        Chassis pitch = rotation around chassis right axis (X_chip) → uses chip_gyro_x + accel_body_pitch
        Chassis roll  = rotation around chassis front axis (Y_chip) → uses chip_gyro_y + accel_body_roll
        :param dt: Time step (seconds)
        :return: (pitch, roll) — pitch and roll angles (degrees)
        """
        # Read raw data
        gx_raw, gy_raw, gz_raw = self.mpu.read_gyro_data()
        ax_raw, ay_raw, az_raw = self.mpu.read_accel_data()

        # Remove zero bias
        gx = gx_raw - self.gyro_bias[0]
        gy = gy_raw - self.gyro_bias[1]
        gz = gz_raw - self.gyro_bias[2]

        # Axis sign correction (if angle direction is reversed, adjust sign in calibration.py)
        gs = IMU_AXIS_MAP['gyro_sign']
        as_ = IMU_AXIS_MAP['accel_sign']
        gx *= gs[0]; gy *= gs[1]; gz *= gs[2]
        ax = ax_raw * as_[0]
        ay = ay_raw * as_[1]
        az = az_raw * as_[2]

        # --- Compute approximate chassis attitude from accelerometer ---
        # Chassis roll  = atan2(chassis right acceleration, up acceleration)
        #                = atan2(chip_ax, chip_az)
        # Chassis pitch = atan2(-chassis front acceleration, up acceleration)
        #                = atan2(-chip_ay, chip_az)
        # Avoid division by zero
        from math import atan2, pi
        accel_roll_val = atan2(ax, az) * 180.0 / pi
        accel_pitch_val = atan2(-ay, az) * 180.0 / pi

        # --- Kalman fusion ---
        # Chassis pitch: gyro input chip_gyro_x (rotation rate around right axis)
        # Chassis roll:  gyro input chip_gyro_y (rotation rate around front axis)
        # Measured pitch and roll signs are both opposite to expected, flip final output
        self.pitch = -self.kf_pitch.update(gx, accel_pitch_val, dt)
        self.roll = -self.kf_roll.update(gy, accel_roll_val, dt)

        return self.pitch, self.roll

    def get_pitch(self):
        return self.pitch

    def get_roll(self):
        return self.roll

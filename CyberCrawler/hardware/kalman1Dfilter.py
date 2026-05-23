# created a class object of a 1D Kalman filter to be used with the MPU6050 sensor
# created Apr 2025 by Nadhir Busheri


class Kalman1D:
    def __init__(self, angle = 0.0, uncertainty = 4.0):
        self.angle = angle
        self.uncertainty = uncertainty  # Initial uncertainty
        self.Q_angle = 0.004 ** 2 * 4 ** 2  # Process noise
        self.R_measure = 3 ** 2  # Measurement noise

    def update(self, gyro_rate, accel_angle, dt):
        """
        updates the value of the given angle and uncertainty
        :param gyro_rate: pass the gyro rate in deg/s from the MPU6050
        :param accel_angle: pass the accel angle in deg from the MPU6050 (this is the angle that is computed by the
                            accelerometer)
        :param dt: pass the time step from the main loop
        :return: the stable angle
        """
        # Predict
        self.angle += dt * gyro_rate
        self.uncertainty += dt * dt * 4 * 4

        # Update
        kalman_gain = self.uncertainty / (self.uncertainty + self.R_measure)
        self.angle += kalman_gain * (accel_angle - self.angle)
        self.uncertainty *= (1 - kalman_gain)

        return self.angle
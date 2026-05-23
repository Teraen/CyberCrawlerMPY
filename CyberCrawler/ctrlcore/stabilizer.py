"""
Pose Stabilizer
PID closed-loop control based on IMU feedback to compensate for body tilt
"""
from calibration import STABILIZER
from utils.pid import PID


class Stabilizer:
    """
    Dual-channel PID stabilizer
    Handles both pitch and roll dimensions simultaneously
    """

    def __init__(self):
        cfg = STABILIZER
        self.enabled = cfg.get('enabled', True)
        self.deadzone = cfg.get('deadzone', 1.0)
        self.output_limit = cfg.get('output_limit', 15)

        self.pid_pitch = PID(
            cfg['pitch']['kp'], cfg['pitch']['ki'], cfg['pitch']['kd'],
            self.output_limit,
        )
        self.pid_roll = PID(
            cfg['roll']['kp'], cfg['roll']['ki'], cfg['roll']['kd'],
            self.output_limit,
        )

    def compute(self, pitch, roll, dt):
        """
        Compute pose correction values
        :param pitch: Current pitch angle (degrees)
        :param roll:  Current roll angle (degrees)
        :param dt:    Time step (seconds)
        :return: (pitch_correction, roll_correction)
        """
        # Dead zone: ignore small angle tilts to prevent oscillation
        p_err = pitch if abs(pitch) > self.deadzone else 0.0
        r_err = roll if abs(roll) > self.deadzone else 0.0

        p_out = self.pid_pitch.compute(0.0, p_err, dt)
        r_out = self.pid_roll.compute(0.0, r_err, dt)
        return p_out, r_out

    def reset(self):
        self.pid_pitch.reset()
        self.pid_roll.reset()

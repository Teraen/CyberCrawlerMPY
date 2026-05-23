"""
Active Suspension (Vehicle Mode)
Dual-channel PID control
"""
from calibration import SUSPENSION
from utils.pid import PID


class Suspension:
    def __init__(self):
        cfg = SUSPENSION
        self.enabled = cfg.get('enabled', True)
        self.deadzone = cfg.get('deadzone', 1.0)
        self.travel_limit = cfg.get('travel_limit', 30)

        self.pid_pitch = PID(
            cfg['pitch']['kp'], cfg['pitch']['ki'], cfg['pitch']['kd'],
            self.travel_limit,
        )
        self.pid_roll = PID(
            cfg['roll']['kp'], cfg['roll']['ki'], cfg['roll']['kd'],
            self.travel_limit,
        )

    def compute(self, pitch, roll, dt):
        if not self.enabled:
            return 0.0, 0.0

        p_err = pitch if abs(pitch) > self.deadzone else 0.0
        r_err = roll if abs(roll) > self.deadzone else 0.0

        pitch_corr = self.pid_pitch.compute(0.0, p_err, dt)
        roll_corr = self.pid_roll.compute(0.0, r_err, dt)

        return pitch_corr, roll_corr

    def reset(self):
        self.pid_pitch.reset()
        self.pid_roll.reset()

"""
Vehicle Mode
Differential drive + active suspension
"""
from calibration import LEG_ORDER, VEHICLE_ROT_TRIM


# Vehicle mode rotation base angles (hard-coded, not user-configurable)
# At 90°, legs point diagonally; need to rotate to direction of travel
# FR/BL=135, BR/FL=45 → after LEG_DIR all become 45° physical angle
_VEHICLE_ROT_BASE = {
    'FR': 135,
    'BR': 45,
    'BL': 135,
    'FL': 45,
}


class VehicleMode:
    """
    In vehicle mode:
      - throttle → all wheels rotate in the same direction
      - steer    → differential steering
      - suspension → active suspension compensation
    """

    def __init__(self, robot):
        self.robot = robot

    def update(self, commands, dt):
        """
        :param commands: Remote control command dictionary
        :param dt: Time step
        :return: Servo output dictionary {
            'FR': {'rot': x, 'lift': x, 'wheel': x},
            ...
        }
        """
        throttle = commands.get('L3', 0)   # -100 ~ 100 (L stick y)
        steer = commands.get('R2', 0)       # -100 ~ 100 (R stick x)
        l2_x = commands.get('L2', 0)        # -100 ~ 100 (L stick x)
        suspension_on = commands.get('suspension', True)

        # Left stick X → rot servo steering offset (mixed into vehicle default angle, not replacing it)
        rot_offset = (l2_x / 100.0) * 15   # ±15°

        # Differential steering: speed difference between left and right sides
        # Positive steer = turn right, left side accelerates, right side decelerates
        left_speed = throttle + steer
        right_speed = throttle - steer

        # Normalize to [-100, 100]
        max_val = max(abs(left_speed), abs(right_speed), 100)
        left_speed = max(-100, min(100, left_speed)) / 100.0
        right_speed = max(-100, min(100, right_speed)) / 100.0

        # Active suspension (skip if IMU not ready)
        pitch_corr = 0.0
        roll_corr = 0.0
        if suspension_on and self.robot.imu is not None:
            try:
                pitch = self.robot.imu.get_pitch()
                roll = self.robot.imu.get_roll()
                pitch_corr, roll_corr = self.robot.suspension.compute(pitch, roll, dt)
            except Exception:
                pass

        # Build output
        output = {}
        for leg in LEG_ORDER:
            is_left = leg in ('FL', 'BL')
            is_front = leg in ('FR', 'FL')
            # Suspension differential
            p = -pitch_corr if leg in ('FR', 'FL') else pitch_corr
            r = roll_corr if leg in ('FR', 'BR') else -roll_corr
            lift_susp = p + r
            lift_susp = max(-30, min(30, lift_susp))  # Clamp
            wheel_speed = left_speed if is_left else right_speed
            # Base steering angle = base ± VEHICLE_ROT_TRIM (FR/BL add, BR/FL subtract)
            trim = VEHICLE_ROT_TRIM[leg]
            if leg in ('FR', 'BL'):
                base_rot = _VEHICLE_ROT_BASE[leg] + trim
            else:
                base_rot = _VEHICLE_ROT_BASE[leg] - trim
            # Overlay left stick live steering offset, front legs inverted (Ackermann geometry)
            rot = base_rot + (-rot_offset if is_front else rot_offset)
            output[leg] = {
                'rot': rot,                                   # Mixed steering offset
                'lift': 90 + lift_susp,                       # Suspension compensation
                'wheel': wheel_speed,                         # Differential drive
            }

        return output

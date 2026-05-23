"""
Gait Controller
Generates gait trajectories for quadruped robots, supporting multiple gait patterns

Core design:
  phase always advances forward (only increases, never decreases)
  Motion direction is controlled by the direction parameter, not expressed through phase direction
  Forward/backward: stance phase trajectory is flipped by direction
  Turning: per-leg direction differs → differential left/right
"""
from calibration import GAIT


class GaitController:
    """
    Each leg's motion is driven by gait_phase (0.0 ~ 1.0)

    update(dt, cadence):
      phase always increments forward, cadence determines advancement rate
      Motion direction is expressed in get_leg_state(leg_index, direction)
        direction = +1: stance leg sweeps front→back (forward)
        direction = -1: stance leg sweeps back→front (backward)

    get_leg_state(leg_index, direction):
      Forward/backward does not change phase, only changes stance trajectory direction
      Swing phase is unaffected by direction (lift-and-return motion is identical)
    """

    GAIT_PATTERNS = {
        'trot':   {'offsets': [0.0, 0.5, 0.0, 0.5], 'swing': 0.50},
        'walk':   {'offsets': [0.0, 0.25, 0.5, 0.75], 'swing': 0.25},
        'ripple': {'offsets': [0.0, 0.75, 0.5, 0.25], 'swing': 0.25},
        'single': {'offsets': [0.0, 0.25, 0.5, 0.75], 'swing': 0.25},
    }

    def __init__(self):
        self.phase = 0.0
        self.gait_type = GAIT.get('default_gait', 'trot')
        self.cycle_time = GAIT.get('cycle_time', 0.5)
        self.step_length = GAIT.get('step_length', 30) / 2.0
        self.step_height = GAIT.get('step_height', 25)

    def set_gait(self, gait_type):
        if gait_type in self.GAIT_PATTERNS:
            self.gait_type = gait_type

    def update(self, dt, cadence):
        """
        Advance gait phase (always forward)
        :param dt: Time step (seconds)
        :param cadence: Step frequency scalar >= 0, 0 stops; phase only increases
        """
        if cadence <= 0:
            return
        delta = dt / self.cycle_time * cadence
        self.phase = (self.phase + delta) % 1.0

    def get_leg_state(self, leg_index, direction=1, reverse_swing=False):
        """
        Compute the state of a specific leg at the current phase
        :param leg_index: Leg index 0=FR, 1=BR, 2=BL, 3=FL
        :param direction: +1=forward (stance front→back), -1=backward (stance back→front)
        :param reverse_swing: True=reverse swing order (used when turning right)
        :return: (rotation_offset, lift_offset)
        phase always moves forward; motion direction is independently controlled by the direction parameter
        """
        from math import sin, cos, pi, sqrt

        pattern = self.GAIT_PATTERNS.get(self.gait_type, self.GAIT_PATTERNS['trot'])
        offsets = pattern['offsets']
        swing = pattern['swing']
        stance = 1.0 - swing

        # Reverse swing order when turning right so forward and backward leg phases match
        # Original order (FR=0.0, BR=0.25, BL=0.5, FL=0.75): FL→BL→BR→FR
        # Reversed order (FR=0.75, BR=0.5, BL=0.25, FL=0.0): FR→BR→BL→FL
        if reverse_swing:
            rev = list(reversed(offsets))
            leg_phase = (self.phase + rev[leg_index]) % 1.0
        else:
            leg_phase = (self.phase + offsets[leg_index]) % 1.0

        if leg_phase < stance:
            # Stance phase — provides propulsion (slow then fast)
            # Forward: leg sweeps from front to back (rot: +step→-step)
            # Backward: leg sweeps from back to front (rot: -step→+step)
            t = leg_phase / stance
            st = sqrt(t)  # Fast then slow (square root curve)
            rot = self.step_length * cos(pi * st)
            lift = 0
        else:
            # Swing phase — lift leg and return for next stance
            # Forward: leg sweeps from back to front (rot: -step→+step)
            # Backward: leg sweeps from front to back (rot: +step→-step)
            t = (leg_phase - stance) / swing
            st = sqrt(t)
            rot = self.step_length * cos(pi * (1.0 + st))
            lift = self.step_height * sin(pi * t)

        # direction < 0 (backward): flip entire trajectory, continuity is maintained automatically
        if direction < 0:
            rot = -rot

        # Left side legs (BL=2, FL=3) invert rotation to compensate for mechanical mounting direction difference
        if leg_index in (2, 3):
            rot = -rot

        return rot, lift

"""
LocomotionMixer - Motion mixing layer

Core responsibilities:
  1. Smooth forward/turn velocity (acceleration limiting)
  2. Compute per-leg stride scale factor
  3. Output current smoothed motion vector

Design principles:
  - forward/turn parameters change continuously, never abruptly
  - Does not directly command servos, only outputs motion parameters
  - Body compensation is handled in PoseMixer
"""
from calibration import LOCOMOTION, LEG_ORDER
from utils.helpers import rate_limit


class LocomotionMixer:
    """
    Locomotion Mixer

    Receives raw forward/turn commands, outputs smoothed motion vector and per-leg scale.

    Usage:
        mixer = LocomotionMixer()
        forward, turn = mixer.update(dt, target_forward, target_turn)
        for i, leg in enumerate(LEG_ORDER):
            scale = mixer.get_leg_scale(i)
            ...
    """

    # Leg grouping: right legs (FR=0, BR=1), left legs (BL=2, FL=3)
    RIGHT_LEGS = {0, 1}  # indices into LEG_ORDER
    LEFT_LEGS  = {2, 3}

    def __init__(self):
        cfg = LOCOMOTION
        # Current smoothed values
        self._forward = 0.0
        self._turn = 0.0
        # Acceleration limits (units: 1/sec, i.e. rate of change of velocity)
        self._fwd_accel = cfg.get('velocity_accel', 3.0)
        self._turn_accel = cfg.get('turn_accel', 4.0)
        # Max stride scale (prevents excessive stride)
        self._max_scale = cfg.get('max_stride_scale', 1.0)
        self._min_scale = cfg.get('min_stride_scale', -1.0)

    def update(self, dt, target_forward, target_turn):
        """
        Smoothly update velocity

        :param dt: Time step (seconds)
        :param target_forward: Target forward velocity [-1.0, 1.0]
        :param target_turn: Target turn velocity [-1.0, 1.0]
        :return: (forward, turn) Current smoothed values
        """
        # Rate-limited update
        self._forward = rate_limit(
            self._forward, target_forward,
            self._fwd_accel * dt
        )
        self._turn = rate_limit(
            self._turn, target_turn,
            self._turn_accel * dt
        )
        return self._forward, self._turn

    @property
    def forward(self):
        """Current smoothed forward velocity"""
        return self._forward

    @property
    def turn(self):
        """Current smoothed turn velocity"""
        return self._turn

    @property
    def speed(self):
        """
        Combined speed scalar (used for gait phase update)
        Always positive; phase only increases.
        Motion direction is controlled by per-leg direction parameter, not expressed through phase direction.
        """
        from math import sqrt
        return sqrt(self._forward ** 2 + self._turn ** 2)

    def get_leg_scale(self, leg_index):
        """
        Compute stride scale factor for a specific leg

        Differential steering principle:
          Right leg scale = forward + turn
          Left leg scale = forward - turn

        Example (turn right, forward=0, turn=0.3):
          Right leg scale = 0 + 0.3 =  0.3 → forward
          Left leg scale = 0 - 0.3 = -0.3 → backward
          Result: right forward, left backward → spin right in place

        Spin left (forward=0, turn=-0.3):
          Right leg scale = 0 + (-0.3) = -0.3 → backward
          Left leg scale = 0 - (-0.3) =  0.3 → forward
          Result: right backward, left forward → spin left in place

        :param leg_index: Leg index 0=FR, 1=BR, 2=BL, 3=FL
        :return: stride scale factor
        """
        is_right = leg_index in self.RIGHT_LEGS
        scale = (self._forward + self._turn) if is_right else (self._forward - self._turn)
        # Clamp to reasonable range
        return max(self._min_scale, min(self._max_scale, scale))

    def reset(self):
        """Reset to stopped state"""
        self._forward = 0.0
        self._turn = 0.0

"""
PoseMixer - Centralized pose mixing layer

Core responsibilities:
  1. Receive pose contributions from each controller (layered input)
  2. Blend all layers by weight
  3. Output final servo pose
  4. Serve as the system's complexity firewall

Design principles:
  - All controllers only generate pose contributions
  - Does not directly command servos, only handles pose offsets
  - Supports future systems (animation, body sway, terrain adaptation)
  - Stabilizer disabled by default (stabilizer_enabled = False)

Architecture:
  pose_mixer.add_layer(name, pose, weight)
    ↓
  final_pose = Σ(layer_pose * weight)
    ↓
  apply_body_compensation()
    ↓
  Servo Output
"""
from calibration import POSE_MIXER, LEG_ORDER, NEUTRAL
from ctrlcore.body import BodyState


class PoseMixer:
    """
    Pose Mixer — centralized layered pose synthesis

    Supported layers:
      - gait         (weight 1.0)
      - stabilizer   (weight 0.3, disabled by default)
      - suspension   (weight 0.5)
      - animation    (weight 0.0, disabled by default)
      - manual       (weight 0.0, disabled by default)

    Usage:
        pose_mixer = PoseMixer(robot)
        pose_mixer.add_layer('gait', gait_pose, 1.0)
        pose_mixer.add_layer('stabilizer', stab_pose, 0.3)
        final = pose_mixer.update(dt)
    """

    # Valid layer names
    VALID_LAYERS = {'gait', 'stabilizer', 'suspension', 'animation', 'manual'}

    def __init__(self, robot):
        self.robot = robot
        self.stabilizer = robot.stabilizer
        self.stabilizer_enabled = False  # Disabled by default, must be enabled manually

        cfg = POSE_MIXER
        self._turn_body_roll = cfg.get('turn_body_roll', 0)
        self._turn_body_pitch = cfg.get('turn_body_pitch', 0)
        self._height_offset = cfg.get('height_offset', 0)

        # Per-layer pose contributions (set by add_layer)
        self._layers = {name: None for name in self.VALID_LAYERS}

        # Per-layer weights (adjustable at runtime)
        self._weights = {
            'gait': 1.0,
            'stabilizer': cfg.get('stabilizer_weight', 0.3),
            'suspension': cfg.get('suspension_weight', 0.5),
            'animation': 0.0,   # Disabled by default
            'manual': 0.0,       # Disabled by default
        }

        # Current body compensation values
        self._body_comp = {
            'pitch': 0.0,
            'roll': 0.0,
            'yaw': 0.0,
            'height': 0.0,
        }

    def add_layer(self, name, pose, weight=None):
        """
        Add a pose layer

        :param name: Layer name ('gait', 'stabilizer', 'suspension', 'animation', 'manual')
        :param pose: Pose contribution dict {leg: {'rot':, 'lift':}}
        :param weight: Weight (optional, defaults to self._weights[name])
        """
        if name not in self.VALID_LAYERS:
            print(f"[POSE_MIXER] Invalid layer name: {name}")
            return
        self._layers[name] = pose
        if weight is not None:
            self._weights[name] = weight

    def set_weight(self, name, weight):
        """Dynamically set layer weight"""
        if name in self._weights:
            self._weights[name] = weight

    def update(self, dt):
        """
        Blend all layers, output final pose

        :param dt: Time step (seconds)
        :return: Blended pose dict {leg: {'rot':, 'lift':, 'wheel':}}
        """
        # 1. Compute body compensation
        self._compute_body_compensation()

        # 2. Blend all layers by weight
        final = self._blend_layers()

        # 3. Apply body compensation
        self._apply_body_compensation(final)

        # 4. Stabilizer correction (if enabled)
        if self.stabilizer_enabled:
            self._apply_stabilizer(final, dt)

        # 5. Ensure all legs have a wheel field
        for leg in LEG_ORDER:
            if leg in final and 'wheel' not in final[leg]:
                final[leg]['wheel'] = 0

        return final

    def _blend_layers(self):
        """
        Blend all layers by weight

        Formula:
            final[leg]['rot'] = Σ (layer[leg]['rot'] * weight)
            final[leg]['lift'] = Σ (layer[leg]['lift'] * weight)
        """
        final = {}
        for leg in LEG_ORDER:
            final[leg] = {'rot': 0.0, 'lift': 0.0, 'wheel': 0}

        for name, weight in self._weights.items():
            layer = self._layers.get(name)
            if layer is None or weight == 0.0:
                continue

            for leg in LEG_ORDER:
                if leg in layer:
                    final[leg]['rot'] += layer[leg].get('rot', 0.0) * weight
                    final[leg]['lift'] += layer[leg].get('lift', 0.0) * weight
                    # wheel is only taken from the gait layer (or other non-zero weight layers)
                    if 'wheel' in layer[leg]:
                        final[leg]['wheel'] += layer[leg]['wheel'] * weight

        return final

    def _compute_body_compensation(self):
        """
        Compute body compensation based on current motion state

        Current implementation (reserved):
          - turn_body_roll: body tilts slightly inward during turns
          - turn_body_pitch: reserved
          - height_offset: reserved
        """
        # Read motion state from robot.body
        body = self.robot.body if hasattr(self.robot, 'body') else None
        if body:
            turn = body.locomotion.get('turn', 0.0)
        else:
            turn = 0.0

        # Body roll compensation during turns (tilt inward)
        roll_comp = self._turn_body_roll * abs(turn)
        self._body_comp['roll'] = roll_comp if turn >= 0 else -roll_comp
        self._body_comp['pitch'] = 0.0
        self._body_comp['yaw'] = 0.0
        self._body_comp['height'] = 0.0

    def _apply_body_compensation(self, final):
        """Apply body compensation to the final pose"""
        roll = self._body_comp['roll']
        if abs(roll) < 0.01:
            return

        # Simplified: distribute roll compensation based on leg x-position
        leg_positions = {
            'FR': (+1, +1),
            'BR': (-1, +1),
            'BL': (-1, -1),
            'FL': (+1, -1),
        }

        roll_factor = 0.5
        for leg in LEG_ORDER:
            if leg not in final:
                continue
            x_sign, _ = leg_positions[leg]
            lift_adj = x_sign * roll * roll_factor
            final[leg]['lift'] += lift_adj

    def _apply_stabilizer(self, final, dt):
        """Apply IMU Stabilizer correction (differential: front/rear opposite for pitch, left/right opposite for roll)"""
        imu = self.robot.imu
        if imu is None:
            return

        try:
            pitch_corr, roll_corr = self.stabilizer.compute(
                imu.pitch, imu.roll, dt
            )
            factor = 0.3
            for leg in final:
                # pitch: front legs −, rear legs +
                p = pitch_corr * factor
                if leg in ('FR', 'FL'):
                    p = -p
                # roll: right leg +, left leg −
                r = roll_corr * factor
                if leg in ('BL', 'FL'):
                    r = -r
                final[leg]['lift'] += p + r
        except Exception:
            pass

    def enable_stabilizer(self):
        """Enable IMU stabilization correction"""
        self.stabilizer_enabled = True
        self.stabilizer.reset()
        self.set_weight('stabilizer', self._weights.get('stabilizer', 0.3))

    def disable_stabilizer(self):
        """Disable IMU stabilization correction"""
        self.stabilizer_enabled = False
        self.set_weight('stabilizer', 0.0)

    def reset(self):
        """Reset all layers and body compensation"""
        self._layers = {name: None for name in self.VALID_LAYERS}
        self._body_comp = {
            'pitch': 0.0,
            'roll': 0.0,
            'yaw': 0.0,
            'height': 0.0,
        }

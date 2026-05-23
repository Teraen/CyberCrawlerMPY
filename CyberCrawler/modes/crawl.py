"""
Crawl Mode — Behavior Orchestration Layer

Responsibilities (per architecture design doc):
  ✅ Enable/disable controllers
  ✅ Configure parameters
  ✅ Define behavior strategies

Prohibited Responsibilities:
  ❌ Compute leg trajectories
  ❌ Compute IMU corrections
  ❌ Directly output servo angles

Design Principles:
  - crawl.py is the behavior orchestration layer
  - All controllers only generate pose contributions
  - Final pose is synthesized by PoseMixer
"""
from calibration import LEG_ORDER, NEUTRAL, ADC_DEADZONE, CRAWL_LIFT_BASE
from utils.helpers import apply_deadzone


class CrawlMode:

    GAIT_LIST = ['trot', 'walk', 'ripple', 'single']

    def __init__(self, robot):
        self.robot = robot
        self.body = robot.body
        self.mixer = robot.locomotion_mixer
        self.pose_mixer = robot.pose_mixer
        self.gait = robot.gait

        # Sync gait index (read from GaitController, default from calibration.py)
        default = self.gait.gait_type
        self._gait_idx = self.GAIT_LIST.index(default) if default in self.GAIT_LIST else 0
        self._r1_was_active = False  # R1 debounce
        self._configure_behavior()

    def _configure_behavior(self):
        """Configure crawl behavior parameters"""
        # Enable gait controller
        self.body.runtime['enabled_controllers'] = ['gait', 'locomotion']

        # Enable stabilizer
        self.pose_mixer.enable_stabilizer()
        self.body.runtime['enabled_controllers'].append('stabilizer')

    def update(self, commands, dt):
        """
        Behavior orchestration update

        Flow:
          1. Parse remote input → update BodyState.motion_intent
          2. LocomotionMixer smooths velocity → update BodyState.locomotion
          3. GaitController updates phase → update BodyState.gait
          4. Collect pose contributions from all controllers → BodyState.pose_layers
          5. PoseMixer blends all layers → final pose
        """
        # === 1. Parse remote input ===
        l3 = commands.get('L3', 0)
        r2 = commands.get('R2', 0)
        target_forward = apply_deadzone(l3 / 100.0, ADC_DEADZONE / 2048.0)
        target_turn = -apply_deadzone(r2 / 100.0, ADC_DEADZONE / 2048.0)  # Invert: push right stick right → turn right

        # Update BodyState.motion_intent
        self.body.motion_intent['forward_velocity'] = target_forward
        self.body.motion_intent['turn_velocity'] = target_turn
        self.body.motion_intent['mode'] = 1  # crawl mode

        # === 2. LocomotionMixer smooths velocity ===
        forward, turn = self.mixer.update(dt, target_forward, target_turn)

        # Compute per-leg scales
        leg_scales = [self.mixer.get_leg_scale(i) for i in range(4)]

        # Update BodyState.locomotion
        self.body.update_locomotion(forward, turn, leg_scales)

        # === 3. GaitController updates phase (always forward) ===
        cadence = self.mixer.speed  # magnitude only, speed always positive
        self.gait.update(dt, cadence)

        # === R1 gait switching ===
        r1 = commands.get('R1', 0)
        # Diagnostics: print R1 value every 150 frames
        if not hasattr(self, '_dbg_r1'): self._dbg_r1 = 0
        self._dbg_r1 += 1
        if self._dbg_r1 >= 150:
            self._dbg_r1 = 0
            print("[CRAWL] R1=" + str(r1) + " idx=" + str(self._gait_idx) +
                  " gait=" + self.gait.gait_type)
        if abs(r1) > 50:
            if not self._r1_was_active:
                if r1 > 50:
                    self._gait_idx = (self._gait_idx + 1) % len(self.GAIT_LIST)
                else:
                    self._gait_idx = (self._gait_idx - 1) % len(self.GAIT_LIST)
                new_gait = self.GAIT_LIST[self._gait_idx]
                self.gait.set_gait(new_gait)
                print("[CRAWL] Gait: " + new_gait)
                self._r1_was_active = True
        else:
            self._r1_was_active = False

        # Update BodyState.gait
        self.body.update_gait_state(
            self.gait.phase,
            self.gait.gait_type,
            cadence
        )

        # === 4. Collect pose contributions from all controllers ===
        self._collect_pose_contributions()

        # === 5. PoseMixer blends all layers ===
        output = self.pose_mixer.update(dt)

        return output

    def _collect_pose_contributions(self):
        """
        Collect pose contributions from all controllers

        Each controller computes its own pose and writes into BodyState.pose_layers

        Per-leg direction mechanism:
          scale sign → leg_direction → gait.get_leg_state(i, direction)
          +scale → direction=+1 → stance front→back (forward)
          -scale → direction=-1 → stance back→front (backward)
          During spot turn, left/right leg scales have opposite signs → opposite directions → differential steering
        """
        # --- Gait pose contribution ---
        gait_pose = {}
        # Reverse swing order when turning right
        new_rev = self.mixer.turn < -0.01
        # Adjust phase on reverse_swing toggle to keep FR leg phase continuous
        if hasattr(self, '_prev_rev') and new_rev != self._prev_rev:
            pattern = self.gait.GAIT_PATTERNS[self.gait.gait_type]
            off = pattern['offsets']
            rev = list(reversed(off))
            shift = (off[0] - rev[0])  # FR offset difference between two modes
            if new_rev:
                self.gait.phase = (self.gait.phase + shift) % 1.0
            else:
                self.gait.phase = (self.gait.phase - shift) % 1.0
        self._prev_rev = new_rev
        reverse_swing = new_rev
        for i, leg in enumerate(LEG_ORDER):
            leg_scale = self.body.locomotion['leg_scales'][i]
            leg_direction = 1 if leg_scale >= 0 else -1

            rot_offset, lift_offset = self.gait.get_leg_state(
                i, leg_direction, reverse_swing
            )

            # Stride scaling (magnitude only, direction expressed in gait trajectory)
            rot_offset *= abs(leg_scale)

            gait_pose[leg] = {
                'rot': NEUTRAL['rotation'] + rot_offset,
                'lift': CRAWL_LIFT_BASE + lift_offset,
                'wheel': 0,
            }

        # Write to BodyState
        self.body.set_pose_layer('gait', gait_pose)
        self.pose_mixer.add_layer('gait', gait_pose)

    def enable_stabilizer(self):
        """Enable the stabilizer controller"""
        self.pose_mixer.enable_stabilizer()
        if 'stabilizer' not in self.body.runtime['enabled_controllers']:
            self.body.runtime['enabled_controllers'].append('stabilizer')

    def disable_stabilizer(self):
        """Disable the stabilizer controller"""
        self.pose_mixer.disable_stabilizer()
        if 'stabilizer' in self.body.runtime['enabled_controllers']:
            self.body.runtime['enabled_controllers'].remove('stabilizer')

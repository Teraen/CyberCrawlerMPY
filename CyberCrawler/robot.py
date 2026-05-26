"""
CyberCrawler robot master control
Initialize all hardware and modules, drive main loop updates
"""
import math
from hardware.pca9685 import PCA9685
from hardware.servo import ServoController

# IDLE wave: adjacent legs rotate toward the lifted leg and extend for support
_LEAN_MAP = {
    'FR': {'BR': {'rot': 30, 'lift': -45}, 'FL': {'rot': -30, 'lift': -45}},
    'BR': {'FR': {'rot': -30, 'lift': -45}, 'BL': {'rot': 30, 'lift': -45}},
    'BL': {'BR': {'rot': -30, 'lift': -45}, 'FL': {'rot': 30, 'lift': -45}},
    'FL': {'FR': {'rot': 30, 'lift': -45}, 'BL': {'rot': -30, 'lift': -45}},
}
from hardware.servo import ServoController
from hardware.imu import IMU
from ctrlcore.gait import GaitController
from ctrlcore.stabilizer import Stabilizer
from ctrlcore.locomotion import LocomotionMixer
from ctrlcore.pose_mixer import PoseMixer
from ctrlcore.body import BodyState
from ctrlcore.suspension import Suspension
from modes.vehicle import VehicleMode
from modes.crawl import CrawlMode
from calibration import (
    PWM_FREQ, LEG_CHANNELS, LEG_ORDER, LEG_DIR,
    NEUTRAL, LIMITS, TRIM, WHEEL, VEHICLE_ROT_TRIM,
    IDLE_LIFT_BASE,
)
from hardware.led import LightController


class Robot:
    MODE_VEHICLE = 0
    MODE_CRAWL = 1
    MODE_IDLE = 2

    def __init__(self, i2c_pca=None, i2c_imu=None):
        """
        :param i2c_pca: I2C bus instance (for PCA9685)
        :param i2c_imu: I2C bus instance (for MPU6050)
        """
        # --- PCA9685 + Servos ---
        self.pwm = PCA9685(i2c_pca)
        self.pwm.set_pwm_freq(PWM_FREQ)
        self.servos = ServoController(self.pwm)

        # --- IMU (using separate I2C bus) ---
        self.imu = None
        if i2c_imu is not None:
            try:
                self.imu = IMU(i2c_imu)
                self.imu.calibrate_gyro()
                print("[ROBOT] IMU initialized")
            except Exception as e:
                print(f"[ROBOT] IMU init failed: {e}")

        # --- Control ---
        self.gait = GaitController()
        self.stabilizer = Stabilizer()
        self.locomotion_mixer = LocomotionMixer()
        self.pose_mixer = PoseMixer(self)
        self.suspension = Suspension()

        # --- BodyState (centralized state) ---
        self.body = BodyState()

        # --- Modes ---
        self.vehicle_mode = VehicleMode(self)
        self.crawl_mode = CrawlMode(self)

        self.mode = self.MODE_IDLE

        # Emergency stop flag
        self._emergency = False
        self._lights = LightController()
        self._lights_on = True
        self._btn_last = 1
        self._idle_wave_leg = None   # Currently waving leg (None = idle)
        self._idle_wave_timer = 0.0  # Wave animation timer
        self._idle_wave_last_output = None  # Last wave pose (on release → smooth return)
        self._idle_wave_entry_t = 0.0  # Wave entry transition remaining time
        self._idle_return_t = 0.0    # Return-to-idle transition timer
        self._idle_return_pose = None  # Starting pose for return transition
        # Mode transition state
        self._trans_active = False
        self._trans_target = 0
        self._trans_legs = []
        self._trans_idx = 0
        self._trans_phase = 0  # 0=lift, 1=rotate, 2=lower
        self._trans_wait = 0
        # Initialize all legs to neutral position
        self.home()

    def set_mode(self, mode):
        if mode == self.mode and not self._trans_active:
            return
        # After emergency stop, switch mode to restore servos
        if self._emergency:
            self._emergency = False
            self.home()
            print("[ROBOT] Emergency cleared")
        # IDLE ↔ VEHICLE leg-by-leg transition; others jump directly
        if self._trans_active:
            if mode == self._trans_target:
                return  # Transition in progress, ignore duplicate requests
            # Mode switch mid-transition → force terminate transition
            self._trans_active = False
        if {self.mode, mode} == {self.MODE_IDLE, self.MODE_VEHICLE}:
            self._start_transition(mode)
        else:
            self._trans_active = False
            self.mode = mode
            self.stabilizer.reset()
            if mode == self.MODE_CRAWL:
                self.gait.phase = 0.0  # Reset gait phase
                for leg in LEG_ORDER:
                    ch = LEG_CHANNELS[leg]
                    self.servos.disable(ch['wheel'])
                print("[ROBOT] Wheel servos disabled for crawl mode")
            elif mode == self.MODE_IDLE:
                self.home()
                for leg in LEG_ORDER:
                    ch = LEG_CHANNELS[leg]
                    self.servos.disable(ch['wheel'])
            elif mode == self.MODE_VEHICLE:
                self.suspension.reset()
                pass
            print("[ROBOT] Mode: " + (
                "CRAWL" if mode == self.MODE_CRAWL else
                "IDLE" if mode == self.MODE_IDLE else
                "VEHICLE"
            ))

    def _start_transition(self, target_mode):
        """Start leg-by-leg transition (IDLE ↔ VEHICLE)"""
        if self._trans_active:
            return
        print("[ROBOT] Transition to " + ("VEHICLE" if target_mode == 0 else "IDLE"))
        self._trans_active = True
        self._trans_target = target_mode
        self._trans_idx = 0
        self._trans_phase = 0
        self._trans_wait = 0
        if target_mode == self.MODE_VEHICLE:
            self._trans_legs = ['FR', 'BR', 'BL', 'FL']
            # Pre-adjust: all lift servos return to neutral (V mode default angle)
            for leg in LEG_ORDER:
                ch = LEG_CHANNELS[leg]
                self.servos.set_angle(ch['lift'], NEUTRAL['lift'], TRIM[leg]['lift'])
        else:
            self._trans_legs = ['FL', 'BL', 'BR', 'FR']
        if self.mode == self.MODE_CRAWL:
            self.home()
            self.mode = self.MODE_IDLE

    def _update_transition(self):
        if not self._trans_active:
            return
        self._trans_wait -= 1
        if self._trans_wait > 0:
            return
        leg = self._trans_legs[self._trans_idx]
        ch = LEG_CHANNELS[leg]
        trim_lift = TRIM[leg]['lift']
        trim_rot = TRIM[leg]['rot']
        ld = LEG_DIR[leg]['lift']

        if self._trans_phase == 0:
            lift = NEUTRAL['lift'] + (45 if ld == 1 else -45)
            self.servos.set_angle(ch['lift'], int(lift), trim_lift)
            self._trans_wait = 4
            self._trans_phase = 1
            print("[TRANS] " + leg + " lift")

        elif self._trans_phase == 1:
            if self._trans_target == self.MODE_VEHICLE:
                vt = VEHICLE_ROT_TRIM[leg]
                rot = (135 + vt) if leg in ('FR', 'BL') else (45 - vt)
            else:
                rot = NEUTRAL['rotation']
            self.servos.set_angle(ch['rot'], int(rot), trim_rot)
            self._trans_wait = 3
            self._trans_phase = 2
            print("[TRANS] " + leg + " rot→" + str(int(rot)))

        elif self._trans_phase == 2:
            # Lower: IDLE mode goes to IDLE_LIFT_BASE, V mode goes to NEUTRAL
            if self._trans_target == self.MODE_IDLE:
                lower = NEUTRAL['lift'] + (IDLE_LIFT_BASE - NEUTRAL['lift']) * LEG_DIR[leg]['lift']
            else:
                lower = NEUTRAL['lift']
            self.servos.set_angle(ch['lift'], int(lower), trim_lift)
            self._trans_wait = 4
            self._trans_phase = 0
            self._trans_idx += 1
            print("[TRANS] " + leg + " lower")
            if self._trans_idx >= 4:
                self._trans_active = False
                self.mode = self._trans_target
                self.stabilizer.reset()
                if self.mode == self.MODE_VEHICLE:
                    self.suspension.reset()
                    print("[ROBOT] Transition → VEHICLE")
                else:
                    # home() would re-write lift, instead only restore rot and disable wheels
                    for l in LEG_ORDER:
                        ch2 = LEG_CHANNELS[l]
                        self.servos.set_angle(ch2['rot'], NEUTRAL['rotation'], TRIM[l]['rot'])
                    for l in LEG_ORDER:
                        self.servos.disable(LEG_CHANNELS[l]['wheel'])
                    print("[ROBOT] Transition → IDLE")

    def home(self):
        """Reset all legs to IDLE position (with LEG_DIR correction)"""
        for leg in LEG_ORDER:
            ch = LEG_CHANNELS[leg]
            ld = LEG_DIR[leg]
            lift = NEUTRAL['lift'] + (IDLE_LIFT_BASE - NEUTRAL['lift']) * ld['lift']
            self.servos.set_angle(
                ch['rot'], NEUTRAL['rotation'], TRIM[leg]['rot'],
            )
            self.servos.set_angle(
                ch['lift'], int(lift), TRIM[leg]['lift'],
            )

    def update(self, dt, commands):
        """
        Main update function (using BodyState architecture)

        :param dt: Time step (seconds)
        :param commands: Remote control command dict
        """
        # Update BodyState.runtime
        self.body.runtime['dt'] = dt
        self.body.runtime['timestamp'] += dt

        # Update IMU and write to BodyState (IDLE mode skips attitude processing)
        if self.imu and self.mode != self.MODE_IDLE:
            try:
                self.imu.update(dt)
                self.body.update_imu(
                    self.imu.pitch,
                    self.imu.roll,
                    getattr(self.imu, 'yaw', 0.0),
                    dt
                )
            except Exception as e:
                print(f"[ROBOT] IMU error: {e}")

        # Emergency stop detection (non-IDLE mode)
        if self.imu and not self._emergency and self.mode != self.MODE_IDLE:
            if abs(self.imu.pitch) > 30 or abs(self.imu.roll) > 30:
                print("[ROBOT] EMERGENCY: pitch=" + str(round(self.imu.pitch, 1)) +
                      " roll=" + str(round(self.imu.roll, 1)) + " disabling servos")
                self.emergency_stop()
                self._emergency = True

        # Mode selection (L1: 0=vehicle, 1=crawl, 2=idle)
        new_mode_raw = commands.get('L1', self.MODE_IDLE)
        new_mode = commands.get('L1', self.MODE_IDLE)
        connected = commands.get('connected', False)

        # Disconnected → immediate IDLE (no delay)
        if not connected and self.mode != self.MODE_IDLE and not self._trans_active:
            print("[ROBOT] RC disconnected → IDLE")
            new_mode = self.MODE_IDLE

        self.set_mode(new_mode)
        self.body.motion_intent['mode'] = new_mode

        # Mode transition in progress: skip normal logic, servos controlled by _update_transition
        if self._trans_active:
            output = None
            self._update_transition()
        # Delegate to current mode
        elif self.mode == self.MODE_VEHICLE:
            output = self.vehicle_mode.update(commands, dt)
        elif self.mode == self.MODE_CRAWL:
            output = self.crawl_mode.update(commands, dt)
        else:
            l2 = commands.get('L2', 0)
            l3 = commands.get('L3', 0)
            r1 = commands.get('R1', 0)
            r2 = commands.get('R2', 0)
            r3 = commands.get('R3', 0)
            wave_active = False

            # --- R2/R3 right stick → body tilt gesture (smooth directional) ---
            if abs(r2) > 20 or abs(r3) > 20:
                wave_active = True
                mag = math.sqrt(r2 * r2 + r3 * r3)
                scale = min(1.0, (mag - 20.0) / 80.0)

                # Tilt direction vector: (r2, -r3) where r3 negative = forward
                nx = r2 / mag
                ny = -r3 / mag

                # Each leg's corner direction, dot product gives smooth offset
                _CORNERS = {'FR': (1, -1), 'FL': (-1, -1),
                            'BR': (1, 1),  'BL': (-1, 1)}
                lift_off = {}
                for leg, (cx, cy) in _CORNERS.items():
                    val = -scale * 30.0 * (nx * cx + ny * cy)
                    lift_off[leg] = max(-30, min(30, int(val)))

                if self._idle_wave_leg != 'R_STICK':
                    self._idle_wave_leg = 'R_STICK'
                    self._idle_wave_timer = 0.0
                    self._idle_wave_entry_t = 0.2

                output = {leg: {
                    'rot': NEUTRAL['rotation'],
                    'lift': IDLE_LIFT_BASE + lift_off[leg],
                    'wheel': 0,
                } for leg in LEG_ORDER}
                self._idle_wave_last_output = output

            # --- L2/L3 left stick → wave gesture ---
            elif abs(l2) > 20 or abs(l3) > 20:
                wave_active = True
                new_wave = self._idle_wave_leg

                if l3 < -20 and l2 > 20:
                    new_wave = 'BR'
                elif l3 < -20 and l2 < -20:
                    new_wave = 'BL'
                elif l3 > 20 and l2 > 20:
                    new_wave = 'FR'
                elif l3 > 20 and l2 < -20:
                    new_wave = 'FL'

                if new_wave != self._idle_wave_leg:
                    # Save current pose for smooth transition between legs
                    if self._idle_wave_last_output:
                        self._idle_return_pose = self._idle_wave_last_output
                    self._idle_wave_leg = new_wave
                    self._idle_wave_timer = 0.0
                    self._idle_wave_entry_t = 0.2

                self._idle_wave_timer += dt
                freq = 2.5
                phase = 2.0 * math.pi * self._idle_wave_timer * freq
                wave_rot = 30.0 * math.cos(phase)
                wave_lift = 30.0 * math.sin(phase)
                lean = _LEAN_MAP.get(self._idle_wave_leg, {})
                _DIAG = {'FR': 'BL', 'BR': 'FL', 'BL': 'FR', 'FL': 'BR'}
                diag_leg = _DIAG.get(self._idle_wave_leg)

                output = {leg: {
                    'rot': NEUTRAL['rotation'] + (
                        wave_rot if leg == self._idle_wave_leg else
                        lean.get(leg, {}).get('rot', 0.0)
                    ),
                    'lift': (
                        NEUTRAL['lift'] + 40 + wave_lift  if leg == self._idle_wave_leg else
                        NEUTRAL['lift'] + 45              if leg == diag_leg else
                        NEUTRAL['lift'] + lean.get(leg, {}).get('lift', 0)
                        if leg in lean else
                        IDLE_LIFT_BASE
                    ),
                    'wheel': 0,
                } for leg in LEG_ORDER}
                self._idle_wave_last_output = output

            # --- Entry transition for both gestures ---
            if wave_active and self._idle_wave_entry_t > 0:
                self._idle_wave_entry_t = max(0.0, self._idle_wave_entry_t - dt)
                t = 1.0 - self._idle_wave_entry_t / 0.2
                from_pose = self._idle_return_pose
                self._idle_return_pose = None  # One-shot
                output = {leg: {
                    'rot': int((from_pose[leg]['rot'] if from_pose else NEUTRAL['rotation']) +
                               (output[leg]['rot'] - (from_pose[leg]['rot'] if from_pose else NEUTRAL['rotation'])) * t),
                    'lift': int((from_pose[leg]['lift'] if from_pose else IDLE_LIFT_BASE) +
                                (output[leg]['lift'] - (from_pose[leg]['lift'] if from_pose else IDLE_LIFT_BASE)) * t),
                    'wheel': 0,
                } for leg in LEG_ORDER}

            # --- No gesture: smooth return to idle ---
            if not wave_active:
                if self._idle_wave_last_output:
                    self._idle_return_pose = self._idle_wave_last_output
                    self._idle_return_t = 0.2
                self._idle_wave_leg = None
                self._idle_wave_timer = 0.0
                self._idle_wave_last_output = None

                if self._idle_return_t > 0 and self._idle_return_pose:
                    self._idle_return_t = max(0.0, self._idle_return_t - dt)
                    t = 1.0 - self._idle_return_t / 0.2
                    output = {leg: {
                        'rot': int(self._idle_return_pose[leg]['rot'] +
                                   (NEUTRAL['rotation'] - self._idle_return_pose[leg]['rot']) * t),
                        'lift': int(self._idle_return_pose[leg]['lift'] +
                                    (IDLE_LIFT_BASE - self._idle_return_pose[leg]['lift']) * t),
                        'wheel': 0,
                    } for leg in LEG_ORDER}
                else:
                    output = {leg: {
                        'rot': NEUTRAL['rotation'],
                        'lift': IDLE_LIFT_BASE,
                        'wheel': 0,
                    } for leg in LEG_ORDER}

        # Lights: K1 short press toggle
        k1 = commands.get('K1', 1)
        if k1 == 0 and self._btn_last == 1:     # Falling edge trigger
            self._lights_on = not self._lights_on
            self._lights.set_enabled(self._lights_on)
        self._btn_last = k1
        gait_type = self.gait.gait_type if self.mode == self.MODE_CRAWL else None
        self._lights.update(self.mode, dt, commands, gait_type, self._emergency)

        # Emergency state: suppress all servo output
        if self._emergency:
            output = None

        # Execute servo output (catch I2C errors, prevent ENODEV from crashing main loop)
        if output is not None:
            try:
                self._apply_output(output)
            except OSError as e:
                print("[ROBOT] Servo I2C error: " + str(e))

    def _apply_output(self, output):
        """Apply output dict to each servo, accounting for per-leg direction correction"""
        for leg in LEG_ORDER:
            if leg not in output:
                continue
            state = output[leg]
            ch = LEG_CHANNELS[leg]
            trim = TRIM[leg]
            leg_dir = LEG_DIR[leg]

            # Rotation servo: apply direction correction to offset
            rot = state.get('rot', NEUTRAL['rotation'])
            rot = NEUTRAL['rotation'] + (rot - NEUTRAL['rotation']) * leg_dir['rot']
            rot = max(LIMITS['rotation'][0], min(LIMITS['rotation'][1], rot))
            self.servos.set_angle(ch['rot'], rot, trim['rot'])

            # Lift servo: apply direction correction to offset
            lift = state.get('lift', NEUTRAL['lift'])
            lift = NEUTRAL['lift'] + (lift - NEUTRAL['lift']) * leg_dir['lift']
            lift = max(LIMITS['lift'][0], min(LIMITS['lift'][1], lift))
            self.servos.set_angle(ch['lift'], lift, trim['lift'])

            # Wheel servo: forced off in crawl/IDLE mode
            if self.mode == self.MODE_CRAWL or self.mode == self.MODE_IDLE:
                self.servos.disable(ch['wheel'])
            else:
                wheel = state.get('wheel', 0) * leg_dir['wheel']
                self.servos.set_wheel_speed(ch['wheel'], wheel, WHEEL.get('trim', 0))

    def emergency_stop(self):
        """Emergency stop: disable all servo outputs"""
        self.servos.disable_all()
        print("[ROBOT] EMERGENCY STOP")

"""
CyberCrawler robot master control
Initialize all hardware and modules, drive main loop updates
"""
from hardware.pca9685 import PCA9685
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
            # IDLE: hold position after transition ends, no servo output (eliminate continuous write jitter)
            output = None

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

"""
CyberCrawler calibration parameters
All angle units in degrees, speed normalized [-1.0, 1.0]
Adjust parameters based on actual measurements after mechanical testing
"""

# ===== Per-servo trim offsets =====
# Individual compensation per servo (angle offset), calibrate leg-by-leg after assembly
TRIM = {
    'FR': {'rot': 0, 'lift': 0},   # Front Right
    'BR': {'rot': 0, 'lift': 0},   # Back Right
    'BL': {'rot': 0, 'lift': 0},   # Back Left
    'FL': {'rot': 0, 'lift': 0},   # Front Left
}

# ===== Servo neutral positions =====
# Rotation servo: 90° = leg pointing diagonally (neutral)
# Lift servo: 90° = leg horizontal / mid height
NEUTRAL = {
    'rotation': 90,
    'lift': 90,
}

# ===== Servo angle limits =====
# All rotation/lift servo final outputs are clamped within this range
LIMITS = {
    'rotation': (20, 160),
    'lift': (30, 150),
}

# ===== Vehicle mode rotation trim =====
# Recommended adjustment range ±15 per change to prevent exceeding motion limits
VEHICLE_ROT_TRIM = {
    'FR': 10,   # Front Right
    'BR': 10,   # Back Right
    'BL': 10,   # Back Left
    'FL': 10,   # Front Left
}

# ===== Vehicle mode attitude stabilization PID parameters =====
SUSPENSION = {
    'enabled': True,        # Enabled by default
    'travel_limit': 30,     # Max travel (degrees)
    'pitch': {'kp': 4, 'ki': 0, 'kd': 0},
    'roll':  {'kp': 2, 'ki': 0, 'kd': 0},
    'deadzone': 1.0,        # Dead zone (degrees)
}

# ===== 360° wheel servos =====
WHEEL = {
    'neutral_duty': 307,      # Stop (1500µs)
    'duty_range': 205,  
    'trim': 59,   # Center offset compensation
}

# ===== Gait parameters =====
GAIT = {
    'cycle_time': 0.8,       # Step cycle (seconds)
    'step_length': 100,       # Step length (total angle)
    'step_height': 45,       # Step height (degrees)
    'default_gait': 'single',  # Default gait
}

# ===== Crawl mode attitude stabilization PID parameters =====
STABILIZER = {
    'enabled': True,         # Enabled by default
    'pitch': {'kp': 8, 'ki': 0.1, 'kd': 0.1},
    'roll':  {'kp': 8, 'ki': 0.1, 'kd': 0.1},
    'deadzone': 1.0,       # Dead zone (degrees), no response below this
    'output_limit': 30,    # Maximum correction (degrees)
}

# Crawl mode leg extension base angle (<90=raise body, >90=lower body)
CRAWL_LIFT_BASE = 75
# IDLE mode leg extension base angle (default 90° neutral)
IDLE_LIFT_BASE = 90

# ===== Joystick dead zone =====
# Global effect
ADC_DEADZONE = 150

# ===== I2C pin configuration (ESP32) =====
# PCA9685 uses hardware I2C(0), MPU6050 uses SoftI2C (independent pins)
I2C_PCA = {
    'type': 'hardware',
    'bus': 0,
    'scl': 2,
    'sda': 3,
}
I2C_IMU = {
    'type': 'soft',       # Software-emulated I2C, usable on any pins
    'scl': 1,
    'sda': 0,
}

# ===== Remote control channel mapping =====
# rc_module.rc_slave_data() returns 10-element list: [L1, L2, L3, R1, R2, R3, K1, K2, K3, K4]
# L1~L6: Joystick/switch ADC 0~4096, mid=2048
# K1~K4: Digital buttons, 0=pressed
REMOTE = {
    'L1': 0,   # L Shoulder 3-position → Mode selection
    'L2': 1,   # L stick x    → (Unassigned)
    'L3': 2,   # L stick y    → Throttle / Forward-reverse
    'R1': 3,   # R Shoulder stick → (Unassigned)
    'R2': 4,   # R stick x    → Steering
    'R3': 5,   # R stick y    → (Unassigned)
    'K1': 6,   # L Shoulder Button → Light toggle
    'K2': 7,   # (Reserved)
    'K3': 8,   # (Reserved)
    'K4': 9,   # (Reserved)
}

# 3-position switch L1 mode thresholds (ADC values)
# Low < vehicle_max → VEHICLE
# Middle → IDLE
# High > crawl_min → CRAWL
L1_MODE_THRESHOLDS = {
    'vehicle_max': 600,    # ADC ≤ 600 → Vehicle mode
    'crawl_min': 3500,     # ADC ≥ 3500 → Crawl mode
}

# ===== PCA9685 servo channel mapping =====
# Each leg has 3 channels: rotation(rot), lift(lift), wheel(wheel)
LEG_CHANNELS = {
    'FR': {'rot': 0, 'lift': 4, 'wheel': 8},   # Front Right
    'BR': {'rot': 1, 'lift': 5, 'wheel': 9},   # Back Right
    'BL': {'rot': 2, 'lift': 6, 'wheel': 10},  # Back Left
    'FL': {'rot': 3, 'lift': 7, 'wheel': 11},  # Front Left
}

# Leg ordering (for loop iteration)
LEG_ORDER = ['FR', 'BR', 'BL', 'FL']   # Front Right → Back Right → Back Left → Front Left

# ===== Per-leg motion direction correction =====
# +1=normal (angle increase/forward), -1=reverse (angle decrease/backward)
LEG_DIR = {
    'FR': {'lift': -1, 'wheel': -1, 'rot':  1},   # Front Right
    'BR': {'lift':  1, 'wheel': -1, 'rot':  1},   # Back Right
    'BL': {'lift': -1, 'wheel':  1, 'rot':  1},   # Back Left
    'FL': {'lift':  1, 'wheel':  1, 'rot':  1},   # Front Left
}

# ===== PCA9685 PWM parameters =====
PWM_FREQ = 50  # Standard servo frequency
SERVO_DUTY_MIN = 105   # Corresponds to ~0.5ms pulse (0°)
SERVO_DUTY_MAX = 510   # Corresponds to ~2.5ms pulse (180°)

# ===== IMU axis mapping =====
IMU_AXIS_MAP = {
    'gyro_sign': (1, 1, 1),     # (gx, gy, gz) sign
    'accel_sign': (1, 1, 1),    # (ax, ay, az) sign
}

# ===== Locomotion mixer parameters =====
# Controls smooth transition of forward/turn velocity
LOCOMOTION = {
    'velocity_accel': 3.0,     # Forward velocity max acceleration (1/s)
    'turn_accel': 4.0,         # Turn velocity max acceleration (1/s)
    'max_stride_scale': 1.0,  # Stride scaling upper limit
    'min_stride_scale': -1.0,  # Stride scaling lower limit (negative = backward)
}

# ===== Pose mixer parameters =====
# Gait + body compensation + IMU stabilization
POSE_MIXER = {
    'turn_body_roll': 0,      # Body roll compensation during turns (degrees), 0=disabled
    'turn_body_pitch': 0,      # Body pitch compensation during turns (degrees), 0=disabled
    'height_offset': 0,       # Body height offset (degrees), 0=disabled
}




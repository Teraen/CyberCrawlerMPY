"""
Remote control input interface
CyberBrick rc_module adaptation layer

Channel mapping (RC Mode 2 style):
  L1 (3-position) → mode selection
  L2 (L stick x)  → (unassigned)
  L3 (L stick y)  → throttle / forward-backward
  R1 (R Shoulder) → (unassigned)
  R2 (R stick x)  → steering
  R3 (R stick y)  → (unassigned)
  K1 (Button)     → suspension toggle
"""
try:
    import rc_module
    HAS_RC_MODULE = True
except ImportError:
    HAS_RC_MODULE = False

from calibration import REMOTE, L1_MODE_THRESHOLDS, ADC_DEADZONE

ADC_MID = 2048
BUTTON_PRESSED = 0


def _adc_to_percent(value, mid=ADC_MID, deadzone=ADC_DEADZONE):
    """Map 12-bit ADC value to -100 ~ 100, deadzone read from calibration"""
    if abs(value - mid) < deadzone:
        return 0
    if value < mid:
        return int((value - mid) * 100.0 / (mid - 0))
    else:
        return int((value - mid) * 100.0 / (4095 - mid))


class RemoteControl:
    """
    CyberBrick remote control receiver wrapper

    get_commands() returns a dictionary, keyed by channel name:
      L1: 0=vehicle / 1=crawl (parsed from 3-position switch)
      L2~L6: -100~100 (joystick ADC mapping)
      K1~K4: 0=pressed / 1=released
      suspension: bool (suspension toggle state, toggled by K1)
      connected: bool
    """

    def __init__(self):
        self._initialized = False
        self._prev_mode = -1

        if HAS_RC_MODULE:
            result = rc_module.rc_slave_init()
            if result is not False:
                self._initialized = True
                print("[REMOTE] RC slave initialized")

    def get_commands(self):
        cmds = {
            'L1': 2,   # mode: 0=vehicle, 1=crawl, 2=idle (default)
            'L2': 0,   # L stick x (unused)
            'L3': 0,   # L stick y → throttle/forward-backward
            'R1': 0,   # R Shoulder Stick (unused)
            'R2': 0,   # R stick x → steering
            'R3': 0,   # R stick y (unused)
            'K1': 1,   # L Shoulder Button (1=released)
            'K2': 0,
            'K3': 0,
            'K4': 0,
            'suspension': True,  # Default on, K1 no longer controls suspension
            'connected': False,
        }

        if not self._initialized or not HAS_RC_MODULE:
            return cmds

        rc_data = rc_module.rc_slave_data()
        # First frame diagnostics
        if not hasattr(self, '_dbg_rc'):
            self._dbg_rc = True
            if rc_data is None:
                print("[REMOTE] rc_slave_data = None")
            else:
                print("[REMOTE] rc_data len=" + str(len(rc_data)) +
                      " raw=" + str([rc_data[i] for i in range(min(10, len(rc_data)))]))
        if rc_data is None or len(rc_data) < 10:
            return cmds

        # --- Mode selection: L1 3-position switch (0=V, 2=IDLE, 1=CRAWL) ---
        l1_val = rc_data[REMOTE['L1']]
        if l1_val <= L1_MODE_THRESHOLDS['vehicle_max']:
            cmds['L1'] = 0   # VEHICLE
        elif l1_val >= L1_MODE_THRESHOLDS['crawl_min']:
            cmds['L1'] = 1   # CRAWL
        else:
            cmds['L1'] = 2   # IDLE

        # --- Joystick channels ---
        for ch in ('L2', 'L3', 'R1', 'R2', 'R3'):
            raw = rc_data[REMOTE[ch]]
            cmds[ch] = _adc_to_percent(raw)

        # --- Button channels ---
        for ch in ('K1', 'K2', 'K3', 'K4'):
            cmds[ch] = rc_data[REMOTE[ch]]

        cmds['connected'] = True
        return cmds

    def is_connected(self):
        return self._initialized and HAS_RC_MODULE

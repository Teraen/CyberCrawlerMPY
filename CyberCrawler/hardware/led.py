"""
WS2812 lighting control (based on MicroPython neopixel, ESP32 built-in RMT driver)

Four LEDs: index 0=FR, 1=BR, 2=BL, 3=FL
GPIO: D1 = 21
"""
from machine import Pin
from neopixel import NeoPixel
import math


# Colors
WHITE = (255, 255, 255)
DIM_WHITE = (30, 30, 30)
AMBER = (255, 120, 0)
RED = (200, 0, 0)
BRIGHT_RED = (200, 20, 20)
DIM_RED = (30, 0, 0)
GREEN = (0, 220, 10)
CYAN = (0, 200, 200)
BLUE = (0, 80, 255)
OFF = (0, 0, 0)

# Gait → color mapping
GAIT_COLORS = {
    'trot': RED,      # Red — trot
    'walk': GREEN,    # Green — walk
    'ripple': AMBER,  # Amber — ripple crawl
    'single': BLUE,   # Blue — single leg step
}


class LightController:
    MODE_IDLE = 2
    MODE_CRAWL = 1
    MODE_VEHICLE = 0

    def __init__(self, pin=21, n=4):
        self.np = NeoPixel(Pin(pin), n, timing=1)
        self._enabled = True
        self._timer = 0.0
        # Force all LEDs off on power-up (WS2812 retains last state after power loss)
        self.np.fill(OFF)
        self.np.write()

    def set_enabled(self, on):
        self._enabled = on
        if not on:
            self.np.fill(OFF)
            self.np.write()

    def _flush(self):
        try:
            self.np.write()
        except OSError:
            pass

    # ---- IDLE: Breathing white ----
    def _idle_effect(self, dt):
        period = 2000.0
        phase = (self._timer * 1000) % period
        brightness = 0.5 * (1.0 - 0.5 * (1.0 + math.sin(2 * 3.14159 * phase / period - 1.5708)))
        b = int(brightness * 40)
        self.np.fill((b, b, b))
        self._flush()

    # ---- CRAWL: Solid gait color + breathing on steer ----
    def _crawl_effect(self, commands, dt, gait_type):
        steer = commands.get('R2', 0) if commands else 0
        gait_color = GAIT_COLORS.get(gait_type, GREEN)

        if abs(steer) > 20:
            # Steering: uniform breathing, brightness matches solid-on level (peak 60)
            period = 1.2
            phase = (self._timer % period) / period
            brightness = 0.5 + 0.5 * math.sin(2 * math.pi * phase - 1.5708)
            b = int(brightness * 60)
            r = int(gait_color[0] * b / 255)
            g = int(gait_color[1] * b / 255)
            bl = int(gait_color[2] * b / 255)
            self.np.fill((r, g, bl))
        else:
            # Solid on, reduced brightness
            b = 60
            r = int(gait_color[0] * b / 255.0)
            g = int(gait_color[1] * b / 255.0)
            bl = int(gait_color[2] * b / 255.0)
            self.np.fill((r, g, bl))

        self._flush()

    # ---- VEHICLE: Lighting logic ----
    def _vehicle_effect(self, commands, dt):
        throttle = commands.get('L3', 0)
        steer = commands.get('R2', 0)
        l2_x = commands.get('L2', 0)
        total_steer = steer + l2_x
        is_reverse = throttle < -15
        is_forward = throttle > 15

        colors = [OFF, OFF, OFF, OFF]

        if is_reverse:
            colors[0] = DIM_WHITE
            colors[3] = DIM_WHITE
            colors[1] = BRIGHT_RED
            colors[2] = BRIGHT_RED
        else:
            # Forward/idle share base lighting: white when moving, dim white when stopped
            front = WHITE if is_forward else DIM_WHITE
            colors[0] = front
            colors[3] = front
            colors[1] = DIM_RED
            colors[2] = DIM_RED
            # Differential steering: amber flowing light on the turning side (independent of forward/stop)
            if abs(total_steer) > 30:
                phase = (self._timer % 0.8) / 0.8
            # Differential steering: all four LEDs cycle amber in a flowing pattern
            if abs(total_steer) > 30:
                period = 0.8
                phase = (self._timer % period) / period
                dir_sign = 1 if total_steer > 0 else -1
                for i in range(4):
                    offset = (i * dir_sign) % 4
                    lp = (phase + offset * 0.25) % 1.0
                    t = lp * 2.0 if lp < 0.5 else 2.0 - lp * 2.0
                    b = int(t * 200)
                    colors[i] = (int(AMBER[0]*b/255), int(AMBER[1]*b/255), int(AMBER[2]*b/255))

        for i, c in enumerate(colors):
            self.np[i] = c
        self._flush()

    # ---- Emergency: Red flashing ----
    def _emergency_effect(self, dt):
        fast = 0.15
        phase = (self._timer % fast) / fast
        if phase < 0.5:
            self.np.fill(RED)
        else:
            self.np.fill(OFF)
        self._flush()

    def update(self, mode, dt, commands=None, gait_type=None, emergency=False):
        if not self._enabled:
            return
        self._timer += dt
        if emergency:
            self._emergency_effect(dt)
        elif mode == self.MODE_VEHICLE:
            self._vehicle_effect(commands or {}, dt)
        elif mode == self.MODE_CRAWL:
            self._crawl_effect(commands or {}, dt, gait_type)
        else:
            self._idle_effect(dt)

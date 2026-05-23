"""
General utility functions (MicroPython compatible)
"""
import math


def clamp(value, lo, hi):
    """Clamp value to [lo, hi] range"""
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def rate_limit(current, target, max_delta):
    """
    Rate limiter, limits the maximum change per step

    :param current: Current value
    :param target: Target value
    :param max_delta: Maximum allowed change per step (positive value)
    :return: Clamped value
    """
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    elif delta < -max_delta:
        return current - max_delta
    return target


def fmt_float(value, decimals=2):
    """
    MicroPython-compatible float formatting (does not rely on zfill/format features)

    :return: Fixed-precision decimal string, e.g. "-1.50", "0.33"
    """
    sign = "" if value >= 0 else "-"
    aval = abs(value)
    scale = 10 ** decimals
    integer_part = int(aval)
    frac_part = int((aval - integer_part) * scale + 0.5)
    if frac_part >= scale:
        integer_part += 1
        frac_part = 0
    frac_str = str(frac_part)
    while len(frac_str) < decimals:
        frac_str = "0" + frac_str
    return sign + str(integer_part) + "." + frac_str


def apply_deadzone(value, deadzone):
    """
    Remove deadzone offset to linearize the response curve

    :param value: Normalized value [-1.0, 1.0]
    :param deadzone: Deadzone width (normalized value)
    :return: Deadzone-processed value
    """
    if abs(value) < deadzone:
        return 0.0
    sign = 1 if value >= 0 else -1
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)

#!/usr/bin/env python3

import sensor_msgs.msg
import numpy as np
from template_joystick_control.joystick_mapping import *

def joystick_simple(joystick: sensor_msgs.msg.Joy):
    # Replace the following line
    u0, u1, u2, a1, a2 = 0, 0, 0, 0, 0

    # Tunnel
    u0 = (joystick.axes[RIGHT_TRIGGER] - joystick.axes[LEFT_TRIGGER]) / 2.0

    # Starboard
    u1 = np.linalg.norm(np.array((
        joystick.axes[LEFT_STICK_HORIZONTAL], joystick.axes[LEFT_STICK_LATHERAL]
    )))
    a1 = np.arctan2(joystick.axes[LEFT_STICK_LATHERAL], joystick.axes[LEFT_STICK_HORIZONTAL])

    # Starboard
    u2 = np.linalg.norm(np.array((
        joystick.axes[RIGHT_STICK_HORIZONTAL], joystick.axes[RIGHT_STICK_LATHERAL]
    )))
    a2 = np.arctan2(joystick.axes[RIGHT_STICK_LATHERAL], joystick.axes[RIGHT_STICK_HORIZONTAL])

    return (u0, u1, u2, a1, a2)
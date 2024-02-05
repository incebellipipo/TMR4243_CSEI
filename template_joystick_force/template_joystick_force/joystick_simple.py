#!/usr/bin/env python3

import sensor_msgs.msg
import numpy as np

def joystick_simple(joystick: sensor_msgs.msg.Joy):
    # Replace the following line
    u0, u1, u2, a1, a2 = 0

    # Tunnel
    u0 = (joystick.axes[5] - joystick.axes[4]) / 2.0

    # Starboard
    u1 = np.linalg.norm(np.array((
        joystick.axes[0], joystick.axis[1]
    )))
    a1 = np.arctan2(joystick.axes[0], joystick.axis[1])

    # Port
    u2 = np.linalg.norm(np.array((
        joystick.axes[2], joystick.axis[3]
    )))
    a2 = np.arctan2(joystick.axes[2], joystick.axis[3])

    return (u0, u1, u2, a1, a2)
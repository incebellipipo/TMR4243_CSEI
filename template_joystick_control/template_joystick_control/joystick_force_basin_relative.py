#!/usr/bin/env python3

import geometry_msgs.msg
import sensor_msgs.msg
import numpy as np


def joystick_force_basin_relative(joystick: sensor_msgs.msg.Joy, position: geometry_msgs.msg.TransformStamped):
    # Replace the following line
    u0, u1, u2, a1, a2 = 0, 0, 0, 0, 0

    #
    ## Write your code below
    #

    return (u0, u1, u2, a1, a2)

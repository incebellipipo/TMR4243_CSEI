#!/usr/bin/env python3
#
# This file is part of CyberShip Enterpries Suite.
#
# CyberShip Enterpries Suite software is free software: you can redistribute it
# and/or modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# CyberShip Enterpries Suite is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# CyberShip Enterpries Suite. If not, see <https://www.gnu.org/licenses/>.
#
# Maintainer: Emir Cem Gezer
# Email: emir.cem.gezer@ntnu.no
# Year: 2022
# Copyright (C) 2023 NTNU Marine Cybernetics Laboratory

import rospy
import numpy as np
# from common_tools.lib import guidanceNodeInit, nodeEnd, ps4, tau, observer, reference, s_p
from common_tools.math_tools import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float64

### Custom code here ###
class Guidance:
    def __init__(self):
        print("Not implemented!")


if __name__ == '__main__':

    rospy.init_node("guidance_node")

    guidance = Guidance()

    # Frequency in hertz
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        ### Function and method calls here ###
        r.sleep()

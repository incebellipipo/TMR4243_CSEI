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
from common_tools.tools.lib import tau, u_data, thrustAllocationNodeInit, nodeEnd
from common_tools.math_tools import *
from std_msgs import Float64MultiArray

class ThrustAllocation:
    def __init__(self):
        pass

    def odom_callback(self, msg):
        pass

    def joy_callback(self, msg):
        pass


def main():
    rospy.init_node("thrust_allocation")

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        # You may place your code here
        r.sleep()

if __name__ == '__main__':
    main()

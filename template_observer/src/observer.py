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
import math
import numpy as np
from commom_tools.lib import ps4, tau, u_data, observer, observerNodeInit, nodeEnd
from common_tools.math_tools import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from cse_messages import observer_message

# Write your code here


if __name__ == '__main__':
    node = observerNodeInit()
    r = rospy.Rate(100) # Usually set to 100 Hz
    while not rospy.is_shutdown():
        # Calls to any functions and methods should be handled inside this while-loop
        loop()
        r.sleep()

    nodeEnd(node)


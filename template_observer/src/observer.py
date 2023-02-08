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

################################################################################
# Use this template as a suggestion.
################################################################################

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from cse_messages import observer_message

class Observer:
    def __init__(self):
        print("not implemented!")


def main():
    rospy.init_node("observer_node")

    # Usually set to 20 Hz
    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        # Place your custom code here


        # Sleep as much as rate 'r'
        r.sleep()

if __name__ == '__main__':
    main()


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
# Year: 2023
# Copyright (C) 2023 NTNU Marine Cybernetics Laboratory

################################################################################
# Use this template as a suggestion.
################################################################################

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

class ThrustAllocation:
    def __init__(self):
        pass

    def odom_callback(self, msg):
        pass

    def force_callback(self, msg):
        pass

    def publish_u_command(self):
        pass


def main():
    rospy.init_node("thrust_allocation")

    r = rospy.Rate(10)

    thrust_allocation = ThrustAllocation()

    while not rospy.is_shutdown():
        # You may place your code here

        thrust_allocation.publish_u_command()

        # Sleep as much as 'r'
        r.sleep()

if __name__ == '__main__':
    main()

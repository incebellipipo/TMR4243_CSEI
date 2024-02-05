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
from std_msgs.msg import Float64MultiArray

class JoystickForce:
    def __init__(self):
        """
        Write your custom code here. You can either implement this class or
        re-write this script as you like.
        """
        print("Not implemented!")

    def joy_callback(self, msg):
        """
        Implement your joystick callback here. You can use 'cse_teleop' package
        as an example.
        """
        print("Not implemented!")


def main():
    # Initialize the node
    rospy.init_node("joystick_force_mapper")

    # Create the joystick force object
    joystick_force = JoystickForce()

    # Let the ros spin
    rospy.spin()

if __name__ == '__main__':
    main()

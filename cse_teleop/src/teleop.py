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
from sensor_msgs.msg import Joy
from enum import Enum


class JoyType(Enum):
    """Simple ENUM class as an example."""
    PSDS4 = 0
    PSDS3 = 1


class CSETeleop:
    """
    This class listens to joystick inputs and productes one-to-one mapping for
    the actuators. This can be taken as an example for how to read (with proper
    ROS terminology, 'subscribe') joystick commands and how to write (again with
    proper ROS terminology, 'publish') to a topic.
    """
    def __init__(self):
        """Constructor for the CSETeleop Class."""


        # Initialize publishers and subscribers
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.u_pub = rospy.Publisher("CSEI/u", Float64MultiArray, queue_size=5)

        joy_type_str = rospy.get_param("~joy_type", "psds4")

        self.l_stick_x = 0
        self.l_stick_y = 0
        self.r_stick_x = 0
        self.r_stick_y = 0
        self.l2_trigger = 0
        self.r2_trigger = 0

        if joy_type_str == "psds4":
            self.joy_type = JoyType.PSDS4
        elif joy_type_str == "psds3":
            self.joy_type = JoyType.PSDS3

        self.u = np.zeros(5)

    def joy_callback(self, msg):

        # Translate sticks
        if (self.joy_type == JoyType.PSDS4) and (self.joy_type == JoyType.PSDS3):
            self.l_stick_x = msg.axes[0]
            self.l_stick_y = msg.axes[1]
            self.r_stick_x = msg.axes[3]
            self.r_stick_y = msg.axes[4]
            self.l2_trigger = msg.axes[2]
            self.r2_trigger = msg.axes[5]
        # Implement here for other types of controllers

        # Measure thust forces
        self.u = self.joystict_to_u()

        # Publish the hell out of it.
        u_msg = Float64MultiArray()
        u_msg.data = self.u
        self.u_pub.publish(u_msg)

    def joystict_to_u(self):

        u = numpy.zeros(5)

        ### Acutator commands ###
        u[0] = -0.5 * (self.l2_trigger - self.r2_trigger)

        u[1] = np.clip(np.sqrt(self.l_stick_x ** 2 + self.l_stick_y ** 2), -1, 1)
        u[2] = np.clip(np.sqrt(self.r_stick_x ** 2 + self.r_stick_y ** 2), -1, 1)

        ### VSD angles as described in the handbook ###
        u[3] = np.arctan2(self.l_stick_x, self.l_stick_y)
        u[4] = np.arctan2(self.r_stick_x, self.r_stick_y)

        return u



def main():
    rospy.init_node("csei_joy_teleop")

    r = rospy.Rate(10)

    teleop = CSETeleop()

    rospy.spin()


if __name__ == '__main__':
    main()

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
        self.u_cmd_pub = rospy.Publisher("CSEI/u_cmd", Float64MultiArray, queue_size=5)

        # Here I read a ros parameter called 'joy_type' with private namespace
        # with default value 'psds4' as PlayStation DualShock 4
        joy_type_str = rospy.get_param("~joy_type", "psds4")

        # After reading it as string, parsing this information and recording it
        # as enum, so that whenever I want to check, I don't need to do a string
        # comparison.
        if joy_type_str == "psds4":
            self.joy_type = JoyType.PSDS4
        elif joy_type_str == "psds3":
            self.joy_type = JoyType.PSDS3
        else:
            self.joy_type = None

        # Creating all the member variables. I am having them as member
        # variables because I need them elsewhere in multiple places.
        # I could have created them as global variables but good coding
        # practices often suggests namespacing, having class members over global
        # variables.
        self.l_stick_x = 0
        self.l_stick_y = 0
        self.r_stick_x = 0
        self.r_stick_y = 0
        self.l2_trigger = 0
        self.r2_trigger = 0

    def joy_callback(self, msg):
        """
        This callback function will be called whenever there is a message on
        '/joy' topic, and 'msg' variable will carry the data that is being
        streamed on that topic.
        """

        # Check the joystick type
        if (self.joy_type == JoyType.PSDS4) or (self.joy_type == JoyType.PSDS3):
            # Translate sticks
            self.l_stick_x = msg.axes[0]
            self.l_stick_y = msg.axes[1]
            self.r_stick_x = msg.axes[3]
            self.r_stick_y = msg.axes[4]
            self.l2_trigger = msg.axes[2]
            self.r2_trigger = msg.axes[5]
        else:
            # Unimplemented controller, we can not do anything.
            #
            # ROS can work with different joystick brands and not all of them
            # have the exact same mapping.
            return

        # Create a message object.
        u_cmd_msg = Float64MultiArray()

        # Measure the control command
        u_cmd_msg.data = self.joystick_to_u()

        # Publish the u_msg message using the publisher
        self.u_cmd_pub.publish(u_cmd_msg)

    def joystick_to_u(self):
        """
        This code translates joystick data to 'u' vector to control the vehicle.
        """

        # Create a control command vector
        # u[0] -> bow tunnel thruster
        # u[1] -> port voith Voith Schneider Propeller force
        # u[2] -> starboard voith Voith Schneider Propeller force
        # u[3] -> force angle for port VSP thruster
        # u[4] -> force angle for starboard VSP thruster
        u = np.zeros(5)

        ### Acutator commands ###
        u[0] = -0.5 * (self.l2_trigger - self.r2_trigger)

        u[1] = np.clip(np.sqrt(self.l_stick_x ** 2 + self.l_stick_y ** 2), -1, 1)
        u[2] = np.clip(np.sqrt(self.r_stick_x ** 2 + self.r_stick_y ** 2), -1, 1)

        ### VSD angles as described in the handbook ###
        u[3] = np.arctan2(self.l_stick_x, self.l_stick_y)
        u[4] = np.arctan2(self.r_stick_x, self.r_stick_y)

        return u


def main():
    """Main function"""

    # Initialize the ROS node
    rospy.init_node("csei_joy_teleop")

    # Create the Teleop object
    teleop = CSETeleop()

    # Let the ROS 'spin'. All the callbacks will continue to execute themselves.
    rospy.spin()


# Call the 'main()' function if the module have called as executable.
if __name__ == '__main__':
    main()

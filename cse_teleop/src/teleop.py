#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from enum import Enum


class JoyType(Enum):
    PSDS4 = 0
    PSDS3 = 1


class CSETeleop:

    def __init__(self):

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
        if self.joy_type == JoyType.PSDS4:
            self.l_stick_x = msg.axes[0]
            self.l_stick_y = msg.axes[1]
            self.r_stick_x = msg.axes[3]
            self.r_stick_y = msg.axes[4]
            self.l2_trigger = msg.axes[2]
            self.r2_trigger = msg.axes[5]
        elif self.joy_type == JoyType.PSDS3:
            # TODO: PSDS4 controller haven't implement yet!
            pass


        # Measure thust forces
        self.sixaxis2thruster()

        # Publish the hell out of it.
        u_msg = Float64MultiArray()
        u_msg.data = self.u
        self.u_pub.publish(u_msg)

    def sixaxis2thruster(self):
        """
        sixaxis2thruster() directly maps the sixaxis playstation controller inputs
        to the vessel actuators.
        """
        ### Acutator commands ###
        self.u[0] = -0.5 * (self.l2_trigger - self.r2_trigger)

        self.u[1] = np.clip(np.sqrt(self.l_stick_x ** 2 + self.l_stick_y ** 2), -1, 1)
        self.u[2] = np.clip(np.sqrt(self.r_stick_x ** 2 + self.r_stick_y ** 2), -1, 1)

        ### VSD angles as described in the handbook ###
        self.u[3] = np.arctan2(self.l_stick_x, self.l_stick_y)
        self.u[4] = np.arctan2(self.r_stick_x, self.r_stick_y)


def main():
    rospy.init_node("csei_joy_teleop")

    r = rospy.Rate(10)

    teleop = CSETeleop()

    rospy.spin()


if __name__ == '__main__':
    main()


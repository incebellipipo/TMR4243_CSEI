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
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem
# Year: 2024
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

import math
import numpy as np

import rclpy
import rclpy.node
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

import template_joystick_control.joystick_mapping
from template_joystick_control.joystick_simple import joystick_simple
from template_joystick_control.joystick_force_basin_relative import joystick_force_basin_relative
from template_joystick_control.joystick_force_body_relative import joystick_force_body_relative


class JoystickControlNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('tmr4243_joystick_control_node')

        self.pubs = {}
        self.subs = {}

        self.subs["joy"] = self.create_subscription(
            sensor_msgs.msg.Joy, '/joy', self.joy_callback, 10)

        self.subs["eta"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/tmr4243/state/eta', self.eta_callback, 10)

        self.pubs["u_cmd"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/tmr4243/command/u', 10)

        self.current_task = self.declare_parameter('task', 'simple')
        self.current_task.value
        self.last_eta_msg = None
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.current_task = self.get_parameter('task')

        self.get_logger().info(
            f"Parameter task: {self.current_task.value}", throttle_duration_sec=1.0)

    def joy_callback(self, msg):

        result = []

        if "simple" in self.current_task.value:
            result = joystick_simple(msg)
        elif "basin" in self.current_task.value:
            if self.last_eta_msg is None:
                self.get_logger().warn(f"Last eta message is {self.last_eta_msg}, cannot basin relative", throttle_duration_sec=1.0)
                return
            result = joystick_force_basin_relative(msg, self.last_eta_msg)
        elif "body" in self.current_task.value:
            result = joystick_force_body_relative(msg)

        if len(result) != 5:
            self.get_logger().warn(
                f"Result has length of {len(result)} but it should be 5", throttle_duration_sec=1.0)

        [u0, u1, u2, a1, a2] = result

        u_cmd = std_msgs.msg.Float32MultiArray()
        u_cmd.data = result
        self.pubs["u_cmd"].publish(u_cmd)


    def eta_callback(self, msg):
        self.last_eta_msg = msg

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    rclpy.spin(JoystickControlNode())

    rclpy.shutdown()


if __name__ == '__main__':
    main()

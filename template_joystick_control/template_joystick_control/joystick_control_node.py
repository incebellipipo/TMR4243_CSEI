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

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import template_joystick_control.joystick_mapping
from template_joystick_control.joystick_simple import joystick_simple
from template_joystick_control.joystick_force_basin_relative import joystick_force_basin_relative
from template_joystick_control.joystick_force_body_relative import joystick_force_body_relative


class JoystickForce(rclpy.node.Node):
    def __init__(self):
        super().__init__('cse_teleop')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pubs = {}
        self.subs = {}

        self.pubs["tunnel"] = self.create_publisher(
            geometry_msgs.msg.Wrench, '/CSEI/thrusters/tunnel/command', 1)
        self.pubs["port"] = self.create_publisher(
            geometry_msgs.msg.Wrench, '/CSEI/thrusters/port/command', 1)
        self.pubs["starboard"] = self.create_publisher(
            geometry_msgs.msg.Wrench, '/CSEI/thrusters/starboard/command', 1)
        self.pubs["tau"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/CSEI/control/tau', 1)

        self.subs["joy"] = self.create_subscription(
            sensor_msgs.msg.Joy, '/joy', self.joy_callback, 10)

        self.joystick_name = self.declare_parameter('joystick_name', 'PS4')

        self.current_task = self.declare_parameter('task', 'simple')
        self.current_task.value

        self.last_transform = None
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):

        try:
            self.last_transform = self.tf_buffer.lookup_transform(
                "base_link",
                "world",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform : {ex}')

        self.current_task = self.get_parameter('task')

        self.get_logger().info(
            f"Parameter task: {self.current_task.value}", throttle_duration_sec=1.0)

    def joy_callback(self, msg):

        result = []

        if "simple" in self.current_task.value:
            result = joystick_simple(msg)
        elif "basin" in self.current_task.value:
            result = joystick_force_basin_relative(msg, self.last_transform)
        elif "body" in self.current_task.value:
            result = joystick_force_body_relative(msg)

        if len(result) != 5:
            self.get_logger().warn(
                f"Result has length of {len(result)} but it should be 5", throttle_duration_sec=1.0)

        [u0, u1, u2, a1, a2] = result

        f = geometry_msgs.msg.Wrench()
        f.force.x = u0
        self.pubs['tunnel'].publish(f)

        f = geometry_msgs.msg.Wrench()
        f.force.x = u1 * math.cos(a1)
        f.force.y = u1 * math.sin(a1)
        self.pubs['port'].publish(f)

        f = geometry_msgs.msg.Wrench()
        f.force.x = u2 * math.cos(a2)
        f.force.y = u2 * math.sin(a2)
        self.pubs['starboard'].publish(f)

        tau = std_msgs.msg.Float32MultiArray()
        tau.data = list(msg.axes[template_joystick_control.joystick_mapping.LEFT_STICK_VERTICAL],
                        msg.axes[template_joystick_control.joystick_mapping.LEFT_STICK_HORIZONTAL],
                        msg.axes[template_joystick_control.joystick_mapping.RIGHT_STICK_VERTICAL])

        self.pubs['tau'].publish(tau)


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    rclpy.spin(JoystickForce())

    rclpy.shutdown()


if __name__ == '__main__':
    main()

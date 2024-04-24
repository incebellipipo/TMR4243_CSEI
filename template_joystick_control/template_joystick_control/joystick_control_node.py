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
import sensor_msgs.msg
import rcl_interfaces.msg

from template_joystick_control.joystick_mapping import JoystickMapping
from template_joystick_control.joystick_simple import joystick_simple
from template_joystick_control.joystick_force_basin_relative import joystick_force_basin_relative
from template_joystick_control.joystick_force_body_relative import joystick_force_body_relative


class JoystickControl(rclpy.node.Node):
    TASK_SIMPLE = 'stationkeeping'
    TASK_BASIN = 'straight_line'
    TASK_BODY = 'body'
    TASKS = [TASK_SIMPLE, TASK_BODY, TASK_BASIN]

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

        self.task = JoystickControl.TASK_SIMPLE
        self.declare_parameter(
            'task',
            self.task,
            rcl_interfaces.msg.ParameterDescriptor(
                description="Task",
                type=rcl_interfaces.msg.ParameterType.PARAMETER_STRING,
                read_only=False,
                additional_constraints=f"Allowed values: {' '.join(JoystickControl.TASKS)}"
            )
        )

        self.joystick_mapping = JoystickMapping()

        self.joystick_mapping.LEFT_STICK_HORIZONTAL = self.declare_parameter(
            'LEFT_STICK_HORIZONTAL', 0).get_parameter_value().integer_value
        self.joystick_mapping.LEFT_STICK_VERTICAL = self.declare_parameter(
            'LEFT_STICK_VERTICAL', 1).get_parameter_value().integer_value
        self.joystick_mapping.RIGHT_STICK_HORIZONTAL = self.declare_parameter(
            'RIGHT_STICK_HORIZONTAL', 2).get_parameter_value().integer_value
        self.joystick_mapping.RIGHT_STICK_VERTICAL = self.declare_parameter(
            'RIGHT_STICK_VERTICAL', 3).get_parameter_value().integer_value
        self.joystick_mapping.LEFT_TRIGGER = self.declare_parameter(
            'LEFT_TRIGGER', 4).get_parameter_value().integer_value
        self.joystick_mapping.RIGHT_TRIGGER = self.declare_parameter(
            'RIGHT_TRIGGER', 5).get_parameter_value().integer_value
        self.joystick_mapping.A_BUTTON = self.declare_parameter(
            'A_BUTTON', 0).get_parameter_value().integer_value
        self.joystick_mapping.B_BUTTON = self.declare_parameter(
            'B_BUTTON', 1).get_parameter_value().integer_value
        self.joystick_mapping.X_BUTTON = self.declare_parameter(
            'X_BUTTON', 2).get_parameter_value().integer_value
        self.joystick_mapping.Y_BUTTON = self.declare_parameter(
            'Y_BUTTON', 3).get_parameter_value().integer_value

        self.last_eta_msg = std_msgs.msg.Float32MultiArray()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.task = self.get_parameter('task').get_parameter_value().string_value

        self.get_logger().info(
            f"Parameter task: {self.current_task.value}", throttle_duration_sec=1.0)

    def joy_callback(self, msg):
        result = np.zeros((5, 1), dtype=float)

        if JoystickControl.TASK_SIMPLE in self.task:
            result = joystick_simple(msg, self.joystick_mapping)

        elif JoystickControl.TASK_BASIN in self.task:

            if self.last_eta_msg is None:
                self.get_logger().warn(f"Last eta message is {self.last_eta_msg}, cannot basin relative", throttle_duration_sec=1.0)
                return

            if len(self.last_eta_msg.data) != 3:
                self.get_logger().warn(
                    f"Last eta message has length of {len(self.last_eta_msg.data)} but it should be 3. Aborting...", throttle_duration_sec=1.0)
                return

            result = joystick_force_basin_relative(msg, np.ndarray(self.last_eta_msg.data, dtype=float), self.joystick_mapping)

        elif JoystickControl.TASK_BODY in self.task:
            result = joystick_force_body_relative(msg, self.joystick_mapping)


        if len(result) != 5:
            self.get_logger().warn(
                f"Result has length of {len(result)} but it should be 5. Aborting...", throttle_duration_sec=1.0)
            return

        u_cmd = std_msgs.msg.Float32MultiArray()
        u_cmd.data = result.flatten().tolist()
        self.pubs["u_cmd"].publish(u_cmd)


    def eta_callback(self, msg):
        self.last_eta_msg = msg

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    rclpy.spin(JoystickControl())

    rclpy.shutdown()


if __name__ == '__main__':
    main()

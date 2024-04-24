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
# Maintainer: Emir Cem Gezer, Petter Hangerhageen
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem.com, petthang@stud.ntnu.no
# Year: 2024
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

import rclpy
import rclpy.node
import rcl_interfaces.msg
import tmr4243_interfaces.msg
import numpy as np

from template_guidance.straight_line import straight_line, update_law
from template_guidance.stationkeeping import stationkeeping
from template_guidance.path import path

class Guidance(rclpy.node.Node):
    TASK_STATIONKEEPING = 'stationkeeping'
    TASK_STRAIGHT_LINE = 'straight_line'
    TASKS = [TASK_STRAIGHT_LINE, TASK_STATIONKEEPING]

    def __init__(self):
        super().__init__("tmr4243_guidance")

        self.pubs = {}
        self.subs = {}

        self.pubs["reference"] = self.create_publisher(
            tmr4243_interfaces.msg.Reference, '/tmr4243/control/reference', 1)

        self.subs["observed_eta"] = self.create_subscription(
            tmr4243_interfaces.msg.Observer, '/tmr4243/observer/eta', self.observer_callback, 10)

        self.last_observation = None

        self.task = Guidance.TASK_STATIONKEEPING
        self.declare_parameter(
            'task',
            self.task,
            rcl_interfaces.msg.ParameterDescriptor(
                description="Task",
                type=rcl_interfaces.msg.ParameterType.PARAMETER_STRING,
                read_only=False,
                additional_constraints=f"Allowed values: {' '.join(Guidance.TASKS)}"
            )
        )
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        guidance_period = 0.1 # seconds
        self.guidance_timer = self.create_timer(guidance_period, self.guidance_callback)


    def timer_callback(self):

        self.task = self.get_parameter('task').get_parameter_value().string_value


        self.get_logger().info(f"Parameter task: {self.task}", throttle_duration_sec=1.0)


    def guidance_callback(self):

        if Guidance.TASK_STATIONKEEPING in self.task:
            eta_d, eta_ds, eta_ds2 = stationkeeping()

            msg = tmr4243_interfaces.msg.Reference()
            msg.eta_d = eta_d.flatten().tolist()
            msg.eta_ds = eta_ds.flatten().tolist()
            msg.eta_ds2 = eta_ds2.flatten().tolist()

            self.pubs["reference"].publish(msg)

        elif Guidance.TASK_STRAIGHT_LINE in self.task:
            eta_d, eta_ds, eta_ds2 = straight_line()
            w, v_s, v_ss = update_law()

            msg = tmr4243_interfaces.msg.Reference()
            msg.eta_d = eta_d.flatten().tolist()
            msg.eta_ds = eta_ds.flatten().tolist()
            msg.eta_ds2 = eta_ds2.flatten().tolist()
            msg.w = w
            msg.v_s = v_s
            msg.v_ss = v_ss
            self.pubs["reference"].publish(msg)

    def observer_callback(self, msg):

        self.last_observation = msg


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    node = Guidance()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


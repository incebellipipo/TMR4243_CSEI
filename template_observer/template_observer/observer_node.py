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
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem.com
# Year: 2022
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

import rclpy
import rclpy.node
import numpy as np

import std_msgs.msg
import tmr4243_interfaces.msg

from template_observer.luenberg import luenberg
from template_observer.wrap import wrap


class Observer(rclpy.node.Node):
    TASK_DEADRECKONING = 'deadreckoning'
    TASK_LUENBERG = 'luenberg'
    TASK_LIST = [TASK_DEADRECKONING, TASK_LUENBERG]

    def __init__(self):
        super().__init__('cse_observer')

        self.L1 = self.declare_parameter('L1', [1.0] * 3)
        self.L2 = self.declare_parameter('L2', [1.0] * 3)
        self.L3 = self.declare_parameter('L3', [1.0] * 3)

        self.subs = {}
        self.pubs = {}

        self.subs["tau"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/CSEI/state/tau', self.tau_callback, 10
        )
        self.subs["eta"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/CSEI/state/eta', self.eta_callback, 10
        )
        self.pubs['observer'] = self.create_publisher(
            tmr4243_interfaces.msg.Observer, '/CSEI/observer/state', 1
        )

        self.last_transform = None
        self.last_joystick_msg = None
        self.last_eta_msg = None
        self.last_tau_msg = None

        self.observer_runner = self.create_timer(0.1, self.observer_loop)

    def observer_loop(self):
        self.L1 = self.get_parameter('L1')
        self.L2 = self.get_parameter('L2')
        self.L3 = self.get_parameter('L3')

        if \
                self.last_eta_msg is None or \
                self.last_tau_msg is None:
            return

        eta_hat, nu_hat, bias_hat = luenberg(
            self.last_eta_msg.data,
            self.last_tau_msg.data,
            self.L1.value,
            self.L2.value,
            self.L3.value
        )

        obs = tmr4243_interfaces.msg.Observer()
        obs.eta = eta_hat
        obs.nu = nu_hat
        obs.bias = bias_hat
        self.pubs['observer'].publish(obs)

    def tau_callback(self, msg: std_msgs.msg.Float32MultiArray):
        self.last_tau_msg = msg

    def eta_callback(self, msg: std_msgs.msg.Float32MultiArray):
        self.last_eta_msg = msg


def main():
    rclpy.init()

    node = Observer()

    rclpy.spin(node)


if __name__ == '__main__':
    main()

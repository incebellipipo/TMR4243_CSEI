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
import std_msgs.msg
import tmr4243_interfaces.msg
import geometry_msgs.msg
import numpy as np
import math


from template_thrust_allocation.thrust_allocation_matrix import thrust_configuration_matrix

from template_thrust_allocation.thruster_allocation import thruster_allocation

class ThrustAllocation(rclpy.node.Node):
    def __init__(self):
        super().__init__("tmr4243_thrust_allocation_node")


        self.pubs = {}
        self.subs = {}

        self.pubs["u_cmd"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/CSEI/control/u_cmd', 1)

        self.B = thrust_configuration_matrix()

        self.last_transform = None
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        thrust_allocation_period = 0.1 # seconds
        self.thrust_allocation_timer = self.create_timer(thrust_allocation_period, self.thrust_allocation_callback)


    def timer_callback(self):

        self.current_controller = self.get_parameter('current_controller')


        self.get_logger().info(f"Parameter task: {self.current_controller.value}", throttle_duration_sec=1.0)


    def thrust_allocation_callback(self, msg):

        if self.last_recived_forces is not None:
            u = thruster_allocation(self.recived_forces, self.B)

            f = geometry_msgs.msg.Wrench()
            f.force.x = u[0]
            self.pubs['tunnel'].publish(f)

            f = geometry_msgs.msg.Wrench()
            f.force.x = u[1] * np.cos(u[3])
            f.force.y = u[1] * np.sin(u[3])
            self.pubs['port'].publish(f)

            f = geometry_msgs.msg.Wrench()
            f.force.x = u[2] * np.cos(u[4])
            f.force.y = u[2] * np.sin(u[4])
            self.pubs['starboard'].publish(f)

            self.last_recived_forces = None

    def recived_forces(self, msg):
        self.last_recived_forces = msg


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    node = ThrustAllocation()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


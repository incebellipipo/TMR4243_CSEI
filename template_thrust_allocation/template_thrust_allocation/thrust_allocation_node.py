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
import tmr4243_interfaces.msg
import geometry_msgs.msg

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from template_thrust_allocation.thurster_configuration_matrix import thrust_configuration_matrix
from template_thrust_allocation.thruster_allocation import thruster_allocation

class ThrustAllocation(rclpy.node.Node):
    def __init__(self):
        super().__init__("cse_thrust_allocation")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pubs = {}
        self.subs = {}

        self.subs["generalized_forces"] = self.create_subscription(
             geometry_msgs.msg.Wrench, '/CSEI/generalized_forces', self.recived_forces, 10) 

        
        self.pubs["tunnel"] = self.create_publisher(
            geometry_msgs.msg.Wrench, '/CSEI/thrusters/tunnel/command', 1)
        self.pubs["port"] = self.create_publisher(
            geometry_msgs.msg.Wrench, '/CSEI/thrusters/port/command', 1)
        self.pubs["starboard"] = self.create_publisher(
            geometry_msgs.msg.Wrench, '/CSEI/thrusters/starboard/command', 1)

        self.B = thrust_configuration_matrix()

        self.last_transform = None
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        thrust_allocation_period = 0.1 # seconds
        self.thrust_allocation_timer = self.create_timer(thrust_allocation_period, self.thrust_allocation_callback)


    def timer_callback(self):

        try:
            self.last_transform = self.tf_buffer.lookup_transform(
                "base_link",
                "world",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform : {ex}')

        self.current_controller = self.get_parameter('current_controller')

        
        self.get_logger().info(f"Parameter task: {self.current_controller.value}", throttle_duration_sec=1.0)


    def thrust_allocation_callback(self):

        if self.last_recived_forces is not None:
            u = [u0, u1, u2, a1, a2]
            u = thruster_allocation(self.recived_forces, self.B)

            f = geometry_msgs.msg.Wrench()
            f.force.x = u[0]
            self.pubs['tunnel'].publish(f)

            f = geometry_msgs.msg.Wrench()
            f.force.x = u[1] * math.cos(u[3])
            f.force.y = u[1] * math.sin(u[3])
            self.pubs['port'].publish(f)

            f = geometry_msgs.msg.Wrench()
            f.force.x = u[2] * math.cos(u[4])
            f.force.y = u[2] * math.sin(u[4])
            self.pubs['starboard'].publish(f)

            self.last_recived_forces = None

        else:
            f = geometry_msgs.msg.Wrench()
            f.force.x = 0
            self.pubs['tunnel'].publish(f)

            f = geometry_msgs.msg.Wrench()
            f.force.x = 0
            f.force.y = 0
            self.pubs['port'].publish(f)

            f = geometry_msgs.msg.Wrench()
            f.force.x = 0
            f.force.y = 0
            self.pubs['starboard'].publish(f)


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


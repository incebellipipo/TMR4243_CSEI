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


from template_guidance.straight_line import straight_line, update_law
from template_guidance.stationkeeping import stationkeeping
from template_guidance.path import path

class Guidance(rclpy.node):
    def __init__(self):
        super().__init__("cse_thrust_allocation")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pubs = {}
        self.subs = {}

        self.pubs["reference"] = self.create_publisher(
            tmr4243_interfaces.Reference, '/CSEI/control/reference', 1)

        self.current_guidance = self.declare_parameter('guidance', 'stationkeeping')
        self.current_guidance = self.declare_parameter('guidance', 'straight_line')
        self.current_guidance.value
        
        self.last_transform = None
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        guidance_period = 0.1 # seconds
        self.guidance_timer = self.create_timer(guidance_period, self.guidance_callback)


    def timer_callback(self):

        try:
            self.last_transform = self.tf_buffer.lookup_transform(
                "base_link",
                "world",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform : {ex}')

        self.guidance = self.get_parameter('guidance')

        
        self.get_logger().info(f"Parameter task: {self.guidance.value}", throttle_duration_sec=1.0)
        

    def guidance_callback(self):

        if "stationkeeping" in self.current_guidance.value:
            eta_d, eta_ds, eta_ds2 = stationkeeping()

            n = tmr4243_interfaces.msg.Reference()
            n.eta_d = eta_d
            n.eta_ds = eta_ds
            n.eta_ds2 = eta_ds2
            self.pubs["reference"].publish(n)
        
        elif "straight_line" in self.current_guidance.value:
            eta_d, eta_ds, eta_ds2 = straight_line()
            w, v_s, v_ss = update_law()
            n = tmr4243_interfaces.msg.Reference()
            n.eta_d = eta_d
            n.eta_ds = eta_ds
            n.eta_ds2 = eta_ds2
            self.pubs["reference"].publish(n)

            v = tmr4243_interfaces.msg.Reference()
            v.w = w
            v.v_s = v_s
            v.v_ss = v_ss
            self.pubs["reference"].publish(v)




def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    node = Guidance()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


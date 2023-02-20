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

################################################################################
# Use this template as a suggestion.
################################################################################

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import dynamic_reconfigure.server
from cse_gain_server.cfg import gainsConfig
from cse_messages.msg import observer_message

class Observer:
    def __init__(self):
        print("not implemented!")

        # Read parameters from the parameter server. Use the provided launch
        #   file to use this functionality. Second argument is the default
        #   value.
        self.L1 = rospy.get_param('L1', 1)
        self.L2 = rospy.get_param('L2', 1)
        self.L3 = rospy.get_param('L3', 1)

        # Dynamic reconfigure initialization
        self.dynamic_reconfigure_server = dynamic_reconfigure.server.Server(
            gainsConfig, self.dynamic_reconfigure_callback)



    def dynamic_reconfigure_callback(self, config, level):
        """
        Dynamic reconfigure callback.
        The programmer can utilize this function for fine tuning the parameters
        """
        self.L1 = config.L1
        self.L2 = config.L2
        self.L3 = config.L3

        print(config)

        return config



def main():
    rospy.init_node("observer_node")

    observer = Observer()

    # Set your rate in Hertz
    r = rospy.Rate(100)

    while not rospy.is_shutdown():
        # Place your custom code here


        # Sleep as much as rate 'r'
        r.sleep()

if __name__ == '__main__':
    main()


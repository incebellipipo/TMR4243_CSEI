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

import rospy

from dynamic_reconfigure.server import Server
from cse_gain_server.cfg import gainsConfig


def callback(config, level):
    """
    The callback function prints the updated gains of the observer and controller everytime the
    """
    rospy.loginfo("""Reconfigure Request: \n
    Observer gains: \n
    --------------- \n
    L1: [{L1}] \n
    L2: [{L2}] \n
    L3: [{L3}] \n
    --------------- \n
    Controller gains: \n
    --------------- \n
    K1: [{K1}]\n
    K2: [{K2}]\n
    Ki: [{Ki}]\n
    mu: {mu}\n
    U_ref: {U_ref} \n""".format(**config))
    return config

if __name__=="__main__":
    rospy.init_node("gain_server", anonymous = False)
    srv = Server(gainsConfig, callback)
    rospy.spin()
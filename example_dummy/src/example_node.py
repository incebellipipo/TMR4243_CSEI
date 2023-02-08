#!/usr/bin/env python3

### 1.
#       ^
#       |
#       |
# This line is important. Linux system uses this line to find out which
# interpreter will read this file. Up there, we say, 'hey computer, use python3
# to read this file.

### 2.
# You need to make python file an executable file.
#   $ chmod +x <file_name>.py

### 3.
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

### 4.
# You need to import rospy
import rospy
import std_msgs.msg

class ReverseString:
    def __init__(self):

        pass


def main():
    rospy.init_node("example_dummy")


if __name__ == "__main__":
    main()
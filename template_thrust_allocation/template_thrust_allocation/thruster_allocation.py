#!/usr/bin/env python3

import numpy as np
import geometry_msgs.msg
import std_msgs.msg

import typing

def thruster_allocation(tau: geometry_msgs.msg.Wrench, allocation_matrix: np.ndarray):
   # Replace the following line
   Fx = tau.force.x
   Fy = tau.force.y
   Mz = tau.torque.z
   tau = np.array([Fx, Fy, Mz])

   # u = [u0, u1, u2, a1, a2]
   u = np.zeros(5)

   #
   ## Write your code below
   #


   return u
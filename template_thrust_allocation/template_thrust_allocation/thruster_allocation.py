 #!/usr/bin/env python3
import numpy as np
import geometry_msgs.msg

def thruster_allocation(generalized_forces: geometry_msgs.msg.Wrench, thrust_configuration_matrix: np.ndarray):

   tau = np.array([generalized_forces.force.x, generalized_forces.force.y, generalized_forces.torque.z])

   B = thrust_configuration_matrix

   # u = [u0, u1, u2, a1, a2]
   # Replace the following line
   u = np.array([0, 0, 0, 0, 0, 0])

   #
   ## Write your code below
   #

   return u
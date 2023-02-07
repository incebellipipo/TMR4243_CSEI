# CyberShip Enterprise Software Suite

This software suite designed for to be used as a course material. It should not
be taken as a basis for other projects as the software might change for the
vehicle.

## Installation

> This package is designed for Python 3, ROS Noetic, and Ubuntu 20.04 (Focal).
Before proceeding with the packages, please follow the installations for ROS and
python.

1. Create a workspace
    ```bash
    mkdir -p ros_ws/src
    ```

1. Clone the repository inside the workspace

1. Install ROS dependencies
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

1. After installation, install the required python packages.

    ```bash
    pip install numpy
    pip install scipy
    ```

1. If there is any problem with the instructions, create a github issue.


## Topics

- **topic**: `/qualisys/CSEI/odom`

  **type**: nav_msgs/Odometry

  **description**: Holds the navigation data for the vehicle. Position and
  orientation

  $\eta = [x, y, \psi]^\top$

  - $x \iff $ `pose.pose.position.x`
  - $y \iff $ `pose.pose.position.y`
  - $psi \iff $ `pose.pose.orientation.`

- **topic**: `/CSEI/u`

  **type**: `std_msgs/Float64MultiArray`

  **description**: Arbitrary control inputs for the actuators. It can be
  published by teleop node or your custom control node.

  $u = [u_1, u_2, u_3, \alpha_1, \alpha_2]^\top$

  - $u_1 \in [-1, 1]$, Controls the tunnel thruster
  - $u_2 \in [0, 1]$, Controls the force for port VST
  - $u_3 \in [0, 1]$, Controls the force for starboard VST
  - $\alpha_1 \in [-\pi, \pi]$, Controls the angle for port VST
  - $\alpha_2 \in [-\pi, \pi]$, Controls the angle for starboard VST

- **topic**: `/CSEI/tau`

  **type**: `std_msgs/Float64MultiArray`

  **description**: body fixed force. It is published by the simulator.

  $\tau = [f_x, f_y, \psi_z]^\top$

- **topic**: `/joy`

  **type**: `sensor_msgs/Joy`

  **description**: Joystick inputs

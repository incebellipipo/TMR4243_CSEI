# CyberShip Enterprise Software Suite

This software suite designed for to be used as a course material. It should not
be taken as a basis for other projects as the software might change for the
vehicle.
<!--
## Crash course!

Do take a look at the [Jupyter Notebooks inside](notebooks) and the code inside of the [`example_dummy`](example_dummy) package. -->

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

<!--
## Topics

- **topic**: `/qualisys/CSEI/odom`

  **type**: nav_msgs/Odometry

  **description**: Holds the navigation data for the vehicle. Position and
  orientation

  $\eta = [x, y, \psi]^\top$

  - $x \iff$ `pose.pose.position.x`
  - $y \iff$ `pose.pose.position.y`
  - $psi \iff$ yaw angle
      ```python
      [_, _, yaw] = common_tools.math_tools.quat2eul(
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
      )
      ```

- **topic**: `/CSEI/u_cmd`

  **type**: `std_msgs/Float64MultiArray`

  **description**: Control inputs for the actuators. It can be published by
  teleop node or your custom control node.

  $u_{cmd} = [u_0, u_1, u_2, \alpha_1, \alpha_2]^\top$

  - $u_0 \in [-1, 1]$, Controls the tunnel thruster
  - $u_1 \in [0, 1]$, Controls the RPM of the port VSP thruster
  - $u_2 \in [0, 1]$, Controls the RPM of the starboard VSP thruster
  - $\alpha_1 \in [-\pi, \pi]$, Controls the angle for port VSP thruster
  - $\alpha_2 \in [-\pi, \pi]$, Controls the angle for starboard VSP thruster

- **topic**: `/CSEI/tau`

  **type**: `std_msgs/Float64MultiArray`

  **description**: body fixed force. It is published by the simulator.

  $\tau = [f_x, f_y, \psi_z]^\top$

- **topic**: `/joy`

  **type**: `sensor_msgs/Joy`

  **description**: Joystick inputs

## Custom Messages

- `cse_messages/observer_message.msg`
    ```
    float64[] eta
    float64[] nu
    float64[] bias
    ```
- `cse_messages/reference_message.msg`
    ```
    float64[] eta_d
    float64[] eta_ds
    float64[] eta_ds2
    float64 w
    float64 v_s
    float64 v_ss
    ```
- `cse_messages/s_message.msg`
    ```
    float64 s
    float64 s_dot
    ```

## Glossary of variables

- $\eta \rightarrow$ State variable
- $u \rightarrow$ Control command
- $\tau \rightarrow$ Body fixed force -->

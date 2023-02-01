cmake_minimum_required(VERSION 3.0.2)
project(cse_messages)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  nav_msgs
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  observer_message.msg
  reference_message.msg
  s_message.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)

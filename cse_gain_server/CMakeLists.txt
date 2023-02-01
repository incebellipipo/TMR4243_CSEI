cmake_minimum_required(VERSION 3.0.2)
project(cse_gain_server)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/gains.cfg
)

catkin_package()

cmake_minimum_required(VERSION 3.0.2)
project(template_controller)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  cse_messages
)

catkin_package()

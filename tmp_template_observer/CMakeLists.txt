cmake_minimum_required(VERSION 3.0.2)
project(template_observer)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  nav_msgs
  rospy
  std_msgs

  cse_messages
)

catkin_package()

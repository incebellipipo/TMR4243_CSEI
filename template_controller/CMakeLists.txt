cmake_minimum_required(VERSION 3.8)
project(template_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})
ament_cmake_symlink_install_directory(DIRECTORY ${PROJECT_SOURCE_DIR})

install(
  DIRECTORY 
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    ${PROJECT_NAME}/controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

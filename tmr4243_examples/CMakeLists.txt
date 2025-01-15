cmake_minimum_required(VERSION 3.8)
project(tmr4243_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(joy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_cmake_symlink_install_directory(DIRECTORY ${PROJECT_SOURCE_DIR})

# Install all the scripts under `scripts` directory
install(
  DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
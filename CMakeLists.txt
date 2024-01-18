cmake_minimum_required(VERSION 3.8)
project(solo12_quadruped_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_subdirectory(solo12_description)
add_subdirectory(solo12_gazebo)

ament_package()

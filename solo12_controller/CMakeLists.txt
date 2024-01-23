cmake_minimum_required(VERSION 3.8)
project(solo12_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  solo12_controller
  SHARED
  src/solo12_controller.cpp
)

target_compile_features(solo12_controller PUBLIC cxx_std_17)
target_include_directories(solo12_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/solo12_controller>
)

ament_target_dependencies(
  solo12_controller PUBLIC
  ${CONTROLLER_INCLUDE_DEPENDS}
)

#### INSTALL

install(
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/solo12_controller
#)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS solo12_controller
  EXPORT export_solo12_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface solo12_controller.xml)

ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
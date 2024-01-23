cmake_minimum_required(VERSION 3.8)
project(solo12_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  solo12_hardware
  SHARED
  src/solo12_hardware.cpp
)

target_compile_features(solo12_hardware PUBLIC cxx_std_17)
target_include_directories(solo12_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/solo12_hardware>
)

ament_target_dependencies(
  solo12_hardware PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
)

#### INSTALL

install(
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/solo12_hardware
#)

#install(
#  DIRECTORY solo12_hardware/include/
#  DESTINATION include/solo12_hardware
#)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS solo12_hardware
  EXPORT export_solo12_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export controller plugins
pluginlib_export_plugin_description_file(hardware_interface solo12_hardware.xml)

ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})
ament_package()
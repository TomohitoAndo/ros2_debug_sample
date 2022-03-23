cmake_minimum_required(VERSION 3.5)
project(hello_world_debug)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(talker_component SHARED
  src/talker_component.cpp
)

rclcpp_components_register_node(talker_component
  PLUGIN "hello_world_debug::TalkerComponent"
  EXECUTABLE talker_component_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)

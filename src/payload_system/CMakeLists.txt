cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(payload_system)

find_package(gz-cmake4 REQUIRED)
find_package(gz-plugin3 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin3_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(agent_msgs REQUIRED)

add_library(${CMAKE_PROJECT_NAME} SHARED payload_system/PayloadSystem.cc)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "PayloadSystem"
)

set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY CXX_STANDARD 17)

target_link_libraries(${CMAKE_PROJECT_NAME}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
)

ament_target_dependencies(${CMAKE_PROJECT_NAME} rclcpp std_msgs agent_msgs)
ament_package()

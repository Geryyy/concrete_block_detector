cmake_minimum_required(VERSION 3.16)
project(concrete_block_detector)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(concrete_block_world_model_interfaces REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)

add_executable(concrete_block_detector_node
  src/concrete_block_detector_node.cpp
)
target_include_directories(concrete_block_detector_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(concrete_block_detector_node PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(concrete_block_detector_node ${PCL_LIBRARIES})
ament_target_dependencies(concrete_block_detector_node
  rclcpp sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros
  tf2_sensor_msgs pcl_conversions
  concrete_block_world_model_interfaces
)

install(TARGETS concrete_block_detector_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

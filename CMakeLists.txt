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
find_package(point_cloud_transport REQUIRED)
find_package(concrete_block_world_model_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(concrete_block_detector_node
  src/concrete_block_detector_node.cpp
)
target_include_directories(concrete_block_detector_node PRIVATE include)
target_link_libraries(concrete_block_detector_node Eigen3::Eigen ${OpenCV_LIBS})
ament_target_dependencies(concrete_block_detector_node
  rclcpp sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros
  tf2_sensor_msgs point_cloud_transport
  concrete_block_world_model_interfaces
)

# Offline, reproducible runner for recorded Blockpose snapshots.  It uses the
# same header-only detector core as the ROS adapter, without requiring a ROS
# graph or point-cloud transport plugin.
add_executable(concrete_block_detector_snapshot_runner
  src/concrete_block_detector_snapshot_runner.cpp
)
target_include_directories(concrete_block_detector_snapshot_runner PRIVATE include)
target_link_libraries(concrete_block_detector_snapshot_runner Eigen3::Eigen nlohmann_json::nlohmann_json yaml-cpp)

install(TARGETS concrete_block_detector_node concrete_block_detector_snapshot_runner
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  function(add_detector_core_test name)
    ament_add_gtest(${name} test/${name}.cpp)
    target_include_directories(${name} PRIVATE include)
    target_link_libraries(${name} Eigen3::Eigen ${OpenCV_LIBS})
  endfunction()

  foreach(name geometry proposals pipeline refine evidence gripper_self_filter rgb_edge_prior)
    add_detector_core_test(test_detector_core_${name})
  endforeach()

endif()

ament_package()

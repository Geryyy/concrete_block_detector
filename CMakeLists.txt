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
find_package(point_cloud_transport REQUIRED)
find_package(concrete_block_world_model_interfaces REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(Eigen3 REQUIRED)

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
  tf2_sensor_msgs pcl_conversions point_cloud_transport
  concrete_block_world_model_interfaces
)

install(TARGETS concrete_block_detector_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_synthesis
    test/test_pose_synthesis.cpp
  )
  target_include_directories(test_pose_synthesis PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(test_pose_synthesis PRIVATE ${PCL_DEFINITIONS})
  target_link_libraries(test_pose_synthesis ${PCL_LIBRARIES})

  ament_add_gtest(test_parity_fixture_contract
    test/test_parity_fixture_contract.cpp
  )
  target_compile_definitions(test_parity_fixture_contract PRIVATE
    CONCRETE_BLOCK_DETECTOR_PARITY_MANIFEST="${CMAKE_CURRENT_SOURCE_DIR}/test/parity/fixtures_manifest.json"
  )

  ament_add_gtest(test_detector_core_geometry
    test/test_detector_core_geometry.cpp
  )
  target_include_directories(test_detector_core_geometry PRIVATE include)
  target_link_libraries(test_detector_core_geometry Eigen3::Eigen)

  ament_add_gtest(test_detector_core_proposals
    test/test_detector_core_proposals.cpp
  )
  target_include_directories(test_detector_core_proposals PRIVATE include)
  target_link_libraries(test_detector_core_proposals Eigen3::Eigen)

  ament_add_gtest(test_detector_core_pipeline
    test/test_detector_core_pipeline.cpp
  )
  target_include_directories(test_detector_core_pipeline PRIVATE include)
  target_link_libraries(test_detector_core_pipeline Eigen3::Eigen)

  ament_add_gtest(test_detector_core_refine
    test/test_detector_core_refine.cpp
  )
  target_include_directories(test_detector_core_refine PRIVATE include)
  target_link_libraries(test_detector_core_refine Eigen3::Eigen)

  ament_add_gtest(test_detector_core_evidence
    test/test_detector_core_evidence.cpp
  )
  target_include_directories(test_detector_core_evidence PRIVATE include)
  target_link_libraries(test_detector_core_evidence Eigen3::Eigen)

  ament_add_gtest(test_detector_core_python_parity
    test/test_detector_core_python_parity.cpp
  )
  target_include_directories(test_detector_core_python_parity PRIVATE include)
  target_link_libraries(test_detector_core_python_parity Eigen3::Eigen)
  target_compile_definitions(test_detector_core_python_parity PRIVATE
    BLOCKPOSE_SOURCE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../blockpose"
  )
endif()

ament_package()

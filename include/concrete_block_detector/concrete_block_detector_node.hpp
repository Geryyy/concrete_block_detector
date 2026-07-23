#pragma once

#include <geometry_msgs/msg/pose_array.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <concrete_block_world_model_interfaces/msg/block.hpp>
#include <concrete_block_world_model_interfaces/srv/get_coarse_blocks.hpp>
#include <concrete_block_world_model_interfaces/srv/upsert_block.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace concrete_block_detector
{

class ConcreteBlockDetectorNode : public rclcpp::Node
{
public:
  explicit ConcreteBlockDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  // Must be called after the node is owned by a shared_ptr, because
  // point_cloud_transport requires that shared ownership for its subscription.
  void start();

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  void update_world_model(const std::vector<geometry_msgs::msg::Pose> & poses);
  void refresh_world_model_cache();

  struct TrackedBlock
  {
    std::string id;
    geometry_msgs::msg::Pose pose;
    int32_t task_status{concrete_block_world_model_interfaces::msg::Block::TASK_FREE};
  };

  std::string world_frame_;
  double voxel_leaf_size_m_;
  double ground_distance_threshold_m_;
  int ground_max_iterations_;
  double ground_max_tilt_deg_;
  double cluster_tolerance_m_;
  int cluster_min_points_;
  int cluster_max_points_;
  double dimension_tolerance_m_;
  bool enforce_max_dimensions_;
  double minimum_long_axis_span_m_;
  double minimum_secondary_axis_span_m_;
  double transform_timeout_s_;
  std::string point_cloud_transport_name_;
  bool world_model_enabled_;
  std::string world_model_frame_;
  std::string get_coarse_blocks_service_;
  std::string upsert_block_service_;
  std::string world_model_id_prefix_;
  double world_model_association_max_distance_m_;
  double world_model_cache_refresh_s_;
  double world_model_confidence_;

  std::shared_ptr<point_cloud_transport::PointCloudTransport> point_cloud_transport_;
  point_cloud_transport::Subscriber cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Client<concrete_block_world_model_interfaces::srv::GetCoarseBlocks>::SharedPtr
    get_coarse_blocks_client_;
  rclcpp::Client<concrete_block_world_model_interfaces::srv::UpsertBlock>::SharedPtr
    upsert_block_client_;
  std::mutex tracked_blocks_mutex_;
  std::vector<TrackedBlock> tracked_blocks_;
  std::uint64_t next_block_id_{0};
  bool world_model_refresh_in_flight_{false};
  rclcpp::Time last_world_model_refresh_{0, 0, RCL_ROS_TIME};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace concrete_block_detector

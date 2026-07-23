#pragma once

#include <geometry_msgs/msg/pose_array.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <concrete_block_world_model_interfaces/msg/block_array.hpp>
#include <concrete_block_world_model_interfaces/srv/discover_blocks.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
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
  void handle_discover_blocks(
    const std::shared_ptr<concrete_block_world_model_interfaces::srv::DiscoverBlocks::Request> request,
    std::shared_ptr<concrete_block_world_model_interfaces::srv::DiscoverBlocks::Response> response);
  concrete_block_world_model_interfaces::msg::BlockArray discover(
    const sensor_msgs::msg::PointCloud2 & cloud_world);

  std::string world_frame_;
  double transform_timeout_s_;
  bool scene_bounds_enabled_;
  std::array<double, 3> scene_bounds_min_m_;
  std::array<double, 3> scene_bounds_max_m_;
  std::string point_cloud_transport_name_;
  std::string discover_service_;
  bool refine_enabled_;

  std::shared_ptr<point_cloud_transport::PointCloudTransport> point_cloud_transport_;
  point_cloud_transport::Subscriber cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Service<concrete_block_world_model_interfaces::srv::DiscoverBlocks>::SharedPtr
    discover_blocks_srv_;
  std::mutex cached_cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr cached_cloud_world_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace concrete_block_detector

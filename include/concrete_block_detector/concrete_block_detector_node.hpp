#pragma once

#include "concrete_block_detector/detector_core_proposals.hpp"
#include "concrete_block_detector/gripper_self_filter.hpp"

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
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace concrete_block_detector
{

struct GripperRailBoxConfig
{
  std::string parent_frame;
  std::string rail_frame;
  detector_core::Point outward_axis_local{detector_core::Point::UnitZ()};
};

struct GripperFilterBox
{
  detector_core::OrientedBox box;
  detector_core::Point rail_start{detector_core::Point::Zero()};
  detector_core::Point rail_end{detector_core::Point::Zero()};
  detector_core::Point outward_normal{detector_core::Point::UnitZ()};
};

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
    const sensor_msgs::msg::PointCloud2 & cloud_world,
    const detector_core::SensorContext & sensor_context,
    const std::vector<GripperFilterBox> & gripper_boxes);

  std::string world_frame_;
  double transform_timeout_s_;
  bool scene_bounds_enabled_;
  std::array<double, 3> scene_bounds_min_m_;
  std::array<double, 3> scene_bounds_max_m_;
  std::string point_cloud_transport_name_;
  std::string discover_service_;
  double cached_cloud_max_age_s_;
  bool refine_enabled_;
  bool gripper_self_filter_enabled_;
  bool gripper_self_filter_publish_markers_;
  double gripper_self_filter_outboard_extent_m_;
  double gripper_self_filter_cross_rail_extent_m_;
  double gripper_self_filter_rail_end_margin_m_;
  std::vector<GripperRailBoxConfig> gripper_self_filter_rails_;
  detector_core::DetectionParameters detector_parameters_;

  point_cloud_transport::Subscriber cloud_sub_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Service<concrete_block_world_model_interfaces::srv::DiscoverBlocks>::SharedPtr
    discover_blocks_srv_;
  std::mutex cached_cloud_mutex_;
  std::condition_variable cached_cloud_cv_;
  sensor_msgs::msg::PointCloud2::SharedPtr cached_cloud_world_;
  std::shared_ptr<detector_core::SensorContext> cached_sensor_context_;
  std::vector<GripperFilterBox> cached_gripper_boxes_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace concrete_block_detector
#include "concrete_block_detector/detector_core_proposals.hpp"

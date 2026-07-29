#pragma once

#include "concrete_block_detector/detector_core_proposals.hpp"
#include "concrete_block_detector/gripper_self_filter.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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

struct FkPosePriorConfig
{
  std::string tcp_frame;
  detector_core::Point tcp_to_block_xyz{detector_core::Point::Zero()};
  detector_core::Point tcp_to_block_rpy{detector_core::Point::Zero()};
  double weight{0.0};
  double translation_tolerance_m{0.30};
  double orientation_tolerance_rad{0.70};
};

struct RgbEdgePriorConfig
{
  bool enabled{false};
  std::string image_topic;
  std::string camera_info_topic;
  double max_sync_delta_s{0.08};
  double weight{0.05};
  double edge_percentile{92.0};
  double sample_spacing_px{4.0};
  double distance_scale_px{3.0};
  double min_support_fraction{0.15};
  int min_samples{12};
  int max_image_dimension_px{768};
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
    const std::vector<GripperFilterBox> & gripper_boxes,
    const detector_core::PosePriors & priors,
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info);
  void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr image);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info);

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
  FkPosePriorConfig fk_pose_prior_;
  RgbEdgePriorConfig rgb_edge_prior_;
  detector_core::DetectionParameters detector_parameters_;

  point_cloud_transport::Subscriber cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
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
  detector_core::PosePriors cached_fk_priors_;
  sensor_msgs::msg::Image::ConstSharedPtr cached_rgb_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr cached_camera_info_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace concrete_block_detector
#include "concrete_block_detector/detector_core_proposals.hpp"

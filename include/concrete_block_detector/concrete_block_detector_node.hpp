#pragma once

#include "concrete_block_detector/detector_core_pipeline.hpp"
#include "concrete_block_detector/gripper_self_filter.hpp"
#include "concrete_block_detector/rgb_edge_prior.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <concrete_block_world_model_interfaces/msg/block_array.hpp>
#include <concrete_block_world_model_interfaces/msg/ground_model.hpp>
#include <concrete_block_world_model_interfaces/srv/discover_blocks.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
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

// The PZS100 rail-to-outboard-box geometry belongs to the portable Blockpose
// core.  This ROS package resolves the named TF frames and renders the result.
using GripperFilterBox = detector_core::Pzs100GripperFilterBox;

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

// What compute_rgb_prior resolved: the scorer discover() feeds into
// detect()/detect_without_refinement(), plus the gate outcome discover()
// needs for its debug diagnostics. The scorer may close over the caller's
// rgb_prior storage (see compute_rgb_prior's doc comment) -- it must not
// outlive that storage.
struct RgbPriorOutcome
{
  detector_core::VisualScorer visual_scorer;
  bool rgb_edge_available{false};
  double rgb_sync_delta_s{std::numeric_limits<double>::infinity()};
  std::string rgb_gate_reason;
};

// Request-scoped outlets. They describe the detector's own decisions and are
// deliberately separate from the world-model's human-facing final overlay.
struct DetectorDebugConfig
{
  bool enabled{false};
  bool publish_clouds{true};
  bool publish_markers{true};
  bool publish_diagnostics{true};
  bool publish_rgb_input{true};
  bool publish_rejected_candidates{true};
  std::string topic_prefix{"/cbp/debug/scene_discovery/detector"};
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
    std::size_t request_prior_count,
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    double & ground_height_m,
    concrete_block_world_model_interfaces::msg::GroundModel & ground_model);

  // discover()'s stages, in the order discover() runs them. Each is pure
  // plumbing over its arguments and this node's config/publishers -- none of
  // them touch the geometric/refinement algorithm itself.

  // Scene bounds, the gripper self-filter, and the debug input/geometry
  // clouds. Appends the primary-output gripper/prior markers to `markers`
  // (they depend only on gripper_boxes/priors, not on detection).
  detector_core::Points filter_scene_points(
    const sensor_msgs::msg::PointCloud2 & cloud_world,
    const std::vector<GripperFilterBox> & gripper_boxes, const detector_core::PosePriors & priors,
    visualization_msgs::msg::MarkerArray & markers, std::size_t & gripper_points_removed);

  // The RGB edge-prior sync/validity gate and scorer construction. The
  // returned scorer may close over `rgb_prior_storage` by reference -- that
  // storage must outlive every use of the returned scorer (i.e. it must be
  // owned by discover()'s own frame, not by a temporary).
  RgbPriorOutcome compute_rgb_prior(
    const std_msgs::msg::Header & cloud_header, const std::vector<GripperFilterBox> & gripper_boxes,
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    std::optional<RgbEdgePrior> & rgb_prior_storage);

  // Converts detection.hypotheses into the response BlockArray, appending
  // the primary pose/marker output for each.
  concrete_block_world_model_interfaces::msg::BlockArray build_block_array(
    const std_msgs::msg::Header & header, const detector_core::DetectionResult & detection,
    geometry_msgs::msg::PoseArray & poses, visualization_msgs::msg::MarkerArray & markers);

  // All of discover()'s debug-only outlets: candidate markers, the replay
  // RGB image, and the diagnostics JSON.
  void publish_debug(
    const std_msgs::msg::Header & header, const std::vector<GripperFilterBox> & gripper_boxes,
    const detector_core::PosePriors & priors, const detector_core::DetectionResult & detection,
    std::size_t request_prior_count, std::size_t gripper_points_removed,
    const RgbPriorOutcome & rgb_outcome, const sensor_msgs::msg::Image::ConstSharedPtr & rgb);

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
  bool sdf_refinement_module_enabled_{true};
  bool gripper_self_filter_enabled_;
  bool gripper_self_filter_module_enabled_{true};
  bool fk_prior_module_enabled_{true};
  bool request_priors_module_enabled_{true};
  bool rgb_edge_prior_module_enabled_{true};
  bool gripper_self_filter_publish_markers_;
  double gripper_self_filter_outboard_extent_m_;
  double gripper_self_filter_cross_rail_extent_m_;
  double gripper_self_filter_rail_end_margin_m_;
  std::vector<GripperRailBoxConfig> gripper_self_filter_rails_;
  FkPosePriorConfig fk_pose_prior_;
  RgbEdgePriorConfig rgb_edge_prior_;
  DetectorDebugConfig debug_;
  detector_core::DetectionParameters detector_parameters_;
  // Ceiling used instead of cluster_max_center_z for a request that asks about a pose above
  // it (elevated_prior_gates.hpp); measured on the carried-block captures.
  double elevated_prior_cluster_max_center_z_{3.0};

  point_cloud_transport::Subscriber cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_input_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_geometry_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_above_ground_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_rgb_input_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_diagnostics_pub_;
  // Latched: the crop is static configuration, published once in start().
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_scene_bounds_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_ground_cells_pub_;
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

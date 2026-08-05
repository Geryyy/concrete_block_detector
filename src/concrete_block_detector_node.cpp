#include "concrete_block_detector/concrete_block_detector_node.hpp"
#include "concrete_block_detector/detector_core_pipeline.hpp"
#include "concrete_block_detector/detector_debug_publish.hpp"
#include "concrete_block_detector/rgb_edge_prior.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace concrete_block_detector
{
namespace
{
constexpr std::array<double, 3> kDefaultDims{{0.9, 0.6, 0.6}};

detector_core::PosePrior pose_prior_from_msg(
  const concrete_block_world_model_interfaces::msg::PosePrior & message)
{
  const auto & q = message.pose.orientation;
  const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
  if (!std::isfinite(message.weight) || !std::isfinite(message.translation_tolerance_m) ||
    !std::isfinite(message.orientation_tolerance_rad) || orientation.norm() <= 1.0e-9)
  {
    throw std::invalid_argument("pose prior has non-finite values or a zero quaternion");
  }
  detector_core::PosePrior prior;
  prior.source = message.source;
  prior.position = detector_core::Point(
    message.pose.position.x, message.pose.position.y, message.pose.position.z);
  prior.rotation = orientation.normalized().toRotationMatrix();
  prior.dims = {{message.dimensions[0], message.dimensions[1], message.dimensions[2]}};
  prior.weight = message.weight;
  prior.translation_tolerance_m = message.translation_tolerance_m;
  prior.orientation_tolerance_rad = message.orientation_tolerance_rad;
  return prior;
}

detector_core::Point transform_point(
  const geometry_msgs::msg::TransformStamped & world_from_frame,
  const detector_core::Point & point_in_frame)
{
  const auto & transform = world_from_frame.transform;
  const Eigen::Quaterniond rotation(
    transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  return rotation.normalized() * point_in_frame + detector_core::Point(
    transform.translation.x, transform.translation.y, transform.translation.z);
}

Eigen::Isometry3d isometry_from_transform(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & value = transform.transform;
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = Eigen::Quaterniond(
    value.rotation.w, value.rotation.x, value.rotation.y, value.rotation.z).normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(value.translation.x, value.translation.y, value.translation.z);
  return result;
}

std::optional<cv::Mat> grayscale_image(const sensor_msgs::msg::Image & image)
{
  if (image.width == 0U || image.height == 0U || image.step == 0U) {return std::nullopt;}
  if (image.encoding == "mono8") {
    if (image.step < image.width || image.data.size() < static_cast<std::size_t>(image.step) * image.height) {return std::nullopt;}
    return cv::Mat(static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC1,
      const_cast<std::uint8_t *>(image.data.data()), image.step).clone();
  }
  if (image.encoding != "rgb8" && image.encoding != "bgr8") {return std::nullopt;}
  if (image.step < image.width * 3U || image.data.size() < static_cast<std::size_t>(image.step) * image.height) {return std::nullopt;}
  cv::Mat color(static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC3,
    const_cast<std::uint8_t *>(image.data.data()), image.step);
  cv::Mat gray;
  cv::cvtColor(color, gray, image.encoding == "rgb8" ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
  return gray;
}

std::optional<cv::Matx33d> camera_matrix_from_info(const sensor_msgs::msg::CameraInfo & info)
{
  if (info.width == 0U || info.height == 0U) {return std::nullopt;}
  for (const auto value : info.k) {if (!std::isfinite(value)) {return std::nullopt;}}
  if (info.k[0] <= 0.0 || info.k[4] <= 0.0) {return std::nullopt;}
  return cv::Matx33d(info.k[0], info.k[1], info.k[2], info.k[3], info.k[4], info.k[5], info.k[6], info.k[7], info.k[8]);
}

cv::Mat gripper_occlusion_mask(
  const std::vector<GripperFilterBox> & boxes, const cv::Matx33d & matrix,
  const cv::Mat & distortion, const Eigen::Isometry3d & world_from_camera,
  const cv::Size & size)
{
  cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
  for (const auto & entry : boxes) {
    std::vector<cv::Point3d> camera_corners;
    camera_corners.reserve(8U);
    for (const double x : {-0.5, 0.5}) {for (const double y : {-0.5, 0.5}) {for (const double z : {-0.5, 0.5}) {
      const detector_core::Point world = entry.box.center + entry.box.rotation * detector_core::Point(
        x * entry.box.size.x(), y * entry.box.size.y(), z * entry.box.size.z());
      const Eigen::Vector3d camera = world_from_camera.linear().transpose() *
        (world - world_from_camera.translation());
      if (camera.z() > 1e-6) {camera_corners.emplace_back(camera.x(), camera.y(), camera.z());}
    }}}
    if (camera_corners.size() < 3U) {continue;}
    std::vector<cv::Point2d> projected;
    cv::projectPoints(camera_corners, cv::Vec3d::all(0.0), cv::Vec3d::all(0.0), matrix, distortion, projected);
    std::vector<cv::Point> polygon;
    for (const auto & point : projected) {if (std::isfinite(point.x) && std::isfinite(point.y)) {polygon.emplace_back(cvRound(point.x), cvRound(point.y));}}
    if (polygon.size() < 3U) {continue;}
    cv::convexHull(polygon, polygon);
    cv::fillConvexPoly(mask, polygon, cv::Scalar(255), cv::LINE_8);
  }
  return mask;
}

detector_core::Points ros_points(const sensor_msgs::msg::PointCloud2 & cloud)
{
  detector_core::Points points;
  points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
      points.emplace_back(*x, *y, *z);
    }
  }
  return points;
}

detector_core::SensorContext make_sensor_context(
  const sensor_msgs::msg::PointCloud2 & cloud_world,
  const geometry_msgs::msg::TransformStamped & world_from_sensor)
{
  detector_core::SensorContext context;
  context.origin = detector_core::Point(
    world_from_sensor.transform.translation.x,
    world_from_sensor.transform.translation.y,
    world_from_sensor.transform.translation.z);
  const auto raw_returns = ros_points(cloud_world);
  context.ray_directions.reserve(raw_returns.size());
  context.ranges.reserve(raw_returns.size());
  for (const auto & point : raw_returns) {
    const auto ray = point - context.origin;
    const double range = ray.norm();
    if (range > 1e-9 && std::isfinite(range)) {
      context.ray_directions.push_back(ray / range);
      context.ranges.push_back(range);
    }
  }
  return context;
}

}  // namespace

ConcreteBlockDetectorNode::ConcreteBlockDetectorNode(const rclcpp::NodeOptions & options)
: Node("concrete_block_detector", options),
  world_frame_(declare_parameter<std::string>("world_frame", "world")),
  transform_timeout_s_(declare_parameter<double>("transform_timeout_s", 0.10)),
  scene_bounds_enabled_(declare_parameter<bool>("scene_bounds.enabled", false)),
  scene_bounds_min_m_([this]() {
      const auto values = declare_parameter<std::vector<double>>(
        "scene_bounds.min_m", std::vector<double>{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()});
      if (values.size() != 3U) {throw std::invalid_argument("scene_bounds.min_m must have exactly three values");}
      return std::array<double, 3>{values[0], values[1], values[2]};
    }()),
  scene_bounds_max_m_([this]() {
      const auto values = declare_parameter<std::vector<double>>(
        "scene_bounds.max_m", std::vector<double>{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()});
      if (values.size() != 3U) {throw std::invalid_argument("scene_bounds.max_m must have exactly three values");}
      return std::array<double, 3>{values[0], values[1], values[2]};
    }()),
  point_cloud_transport_name_(declare_parameter<std::string>("point_cloud_transport", "cloudini")),
  discover_service_(declare_parameter<std::string>("discover_service", "~/discover_blocks")),
  cached_cloud_max_age_s_(declare_parameter<double>("cached_cloud_max_age_s", 2.0)),
  refine_enabled_(declare_parameter<bool>("refine_enabled", true)),
  gripper_self_filter_enabled_(declare_parameter<bool>("gripper_self_filter.enabled", false)),
  gripper_self_filter_publish_markers_(declare_parameter<bool>("gripper_self_filter.publish_markers", true)),
  gripper_self_filter_outboard_extent_m_(declare_parameter<double>("gripper_self_filter.outboard_extent_m", 1.50)),
  gripper_self_filter_cross_rail_extent_m_(declare_parameter<double>("gripper_self_filter.cross_rail_extent_m", 1.20)),
  gripper_self_filter_rail_end_margin_m_(declare_parameter<double>("gripper_self_filter.rail_end_margin_m", 0.15)),
  tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  if (world_frame_.empty() || transform_timeout_s_ < 0.0 || !std::isfinite(cached_cloud_max_age_s_) ||
    cached_cloud_max_age_s_ <= 0.0 || point_cloud_transport_name_.empty() || discover_service_.empty() ||
    scene_bounds_min_m_[0] > scene_bounds_max_m_[0] || scene_bounds_min_m_[1] > scene_bounds_max_m_[1] || scene_bounds_min_m_[2] > scene_bounds_max_m_[2])
  {throw std::invalid_argument("invalid concrete_block_detector parameter");}
  const auto nonnegative_size = [this](const std::string & name, std::size_t value) {
      const int configured = declare_parameter<int>(name, static_cast<int>(value));
      if (configured < 0) {throw std::invalid_argument(name + " must be non-negative");}
      return static_cast<std::size_t>(configured);
    };
  detector_parameters_.voxel_size = declare_parameter<double>("detector.voxel_size", detector_parameters_.voxel_size);
  detector_parameters_.ground_thickness = declare_parameter<double>("detector.ground_thickness", detector_parameters_.ground_thickness);
  detector_parameters_.ground_ransac_distance = declare_parameter<double>("detector.ground_ransac_distance", detector_parameters_.ground_ransac_distance);
  detector_parameters_.ground_normal_min_z = declare_parameter<double>("detector.ground_normal_min_z", detector_parameters_.ground_normal_min_z);
  detector_parameters_.ground_ransac_iterations = nonnegative_size("detector.ground_ransac_iterations", detector_parameters_.ground_ransac_iterations);
  detector_parameters_.local_ground_cell_size = declare_parameter<double>("detector.local_ground_cell_size", detector_parameters_.local_ground_cell_size);
  detector_parameters_.local_ground_clearance = declare_parameter<double>("detector.local_ground_clearance", detector_parameters_.local_ground_clearance);
  detector_parameters_.dbscan_eps = declare_parameter<double>("detector.dbscan_eps", detector_parameters_.dbscan_eps);
  detector_parameters_.dbscan_min_points = nonnegative_size("detector.dbscan_min_points", detector_parameters_.dbscan_min_points);
  detector_parameters_.cluster_min_size = nonnegative_size("detector.cluster_min_size", detector_parameters_.cluster_min_size);
  detector_parameters_.cluster_max_size = nonnegative_size("detector.cluster_max_size", detector_parameters_.cluster_max_size);
  detector_parameters_.cluster_min_extent_xy = declare_parameter<double>("detector.cluster_min_extent_xy", detector_parameters_.cluster_min_extent_xy);
  detector_parameters_.cluster_max_extent_xy = declare_parameter<double>("detector.cluster_max_extent_xy", detector_parameters_.cluster_max_extent_xy);
  detector_parameters_.region_max_extent_xy = declare_parameter<double>("detector.region_max_extent_xy", detector_parameters_.region_max_extent_xy);
  detector_parameters_.cluster_min_extent_z = declare_parameter<double>("detector.cluster_min_extent_z", detector_parameters_.cluster_min_extent_z);
  detector_parameters_.cluster_max_extent_z = declare_parameter<double>("detector.cluster_max_extent_z", detector_parameters_.cluster_max_extent_z);
  detector_parameters_.cluster_max_center_z = declare_parameter<double>("detector.cluster_max_center_z", detector_parameters_.cluster_max_center_z);
  detector_parameters_.ransac_distance = declare_parameter<double>("detector.ransac_distance", detector_parameters_.ransac_distance);
  detector_parameters_.ransac_iterations = nonnegative_size("detector.ransac_iterations", detector_parameters_.ransac_iterations);
  detector_parameters_.ransac_search_max_points = nonnegative_size("detector.ransac_search_max_points", detector_parameters_.ransac_search_max_points);
  detector_parameters_.ransac_seed = static_cast<std::uint64_t>(nonnegative_size("detector.ransac_seed", detector_parameters_.ransac_seed));
  detector_parameters_.max_planes = nonnegative_size("detector.max_planes", detector_parameters_.max_planes);
  detector_parameters_.min_inliers = nonnegative_size("detector.min_inliers", detector_parameters_.min_inliers);
  detector_parameters_.top_plane_angle_deg = declare_parameter<double>("detector.top_plane_angle_deg", detector_parameters_.top_plane_angle_deg);
  detector_parameters_.side_plane_angle_deg = declare_parameter<double>("detector.side_plane_angle_deg", detector_parameters_.side_plane_angle_deg);
  detector_parameters_.min_score = declare_parameter<double>("detector.min_score", detector_parameters_.min_score);
  detector_parameters_.fk_seed_min_evidence_gain = declare_parameter<double>(
    "detector.fk_seed_min_evidence_gain", detector_parameters_.fk_seed_min_evidence_gain);
  detector_parameters_.fit_wide_proposals = declare_parameter<bool>(
    "detector.fit_wide_proposals", detector_parameters_.fit_wide_proposals);
  detector_parameters_.conflict_alternatives = nonnegative_size("detector.conflict_alternatives", detector_parameters_.conflict_alternatives);
  if (detector_parameters_.conflict_alternatives < 1U) {
    throw std::invalid_argument("detector.conflict_alternatives must be at least one");
  }
  detector_parameters_.proposal_max_components = nonnegative_size("detector.proposal_max_components", detector_parameters_.proposal_max_components);
  const auto dims = declare_parameter<std::vector<double>>(
    "detector.block_dims", {0.9, 0.6, 0.6});
  if (dims.size() != 3U) {throw std::invalid_argument("detector.block_dims must contain three values");}
  detector_parameters_.block_dims = {dims[0], dims[1], dims[2]};
  const auto candidate_dims = declare_parameter<std::vector<double>>(
    "detector.candidate_dims", std::vector<double>{});
  if (candidate_dims.size() % 3U != 0U) {throw std::invalid_argument("detector.candidate_dims must be a flattened sequence of triples");}
  for (std::size_t index = 0; index < candidate_dims.size(); index += 3U) {
    detector_parameters_.candidate_dims.push_back(
      {candidate_dims[index], candidate_dims[index + 1U], candidate_dims[index + 2U]});
  }
  detector_parameters_.refine_band = declare_parameter<double>("detector.refine_band", detector_parameters_.refine_band);
  detector_parameters_.refine_iterations = nonnegative_size("detector.refine_iterations", detector_parameters_.refine_iterations);
  detector_parameters_.refine_min_points = nonnegative_size("detector.refine_min_points", detector_parameters_.refine_min_points);
  detector_parameters_.refine_huber_scale = declare_parameter<double>("detector.refine_huber_scale", detector_parameters_.refine_huber_scale);
  detector_parameters_.refine_max_translation = declare_parameter<double>("detector.refine_max_translation", detector_parameters_.refine_max_translation);
  detector_parameters_.refine_max_rotation_deg = declare_parameter<double>("detector.refine_max_rotation_deg", detector_parameters_.refine_max_rotation_deg);
  detector_parameters_.refine_preserve_top_axis_if_gravity_worsens = declare_parameter<bool>(
    "detector.refine_preserve_top_axis_if_gravity_worsens",
    detector_parameters_.refine_preserve_top_axis_if_gravity_worsens);
  const auto local_point_parameter = [this](const std::string & name, const std::vector<double> & default_value) {
      const auto values = declare_parameter<std::vector<double>>(name, default_value);
      if (values.size() != 3U) {throw std::invalid_argument(name + " must contain three values");}
      return detector_core::Point(values[0], values[1], values[2]);
    };
  // Declare module switches before validating their individual configuration:
  // an explicitly disabled module is a real no-op, including for an otherwise
  // incomplete FK/RGB setup used by an ablation file.
  sdf_refinement_module_enabled_ = declare_parameter<bool>(
    "modules.sdf_refinement.enabled", sdf_refinement_module_enabled_);
  gripper_self_filter_module_enabled_ = declare_parameter<bool>(
    "modules.gripper_self_filter.enabled", gripper_self_filter_module_enabled_);
  fk_prior_module_enabled_ = declare_parameter<bool>(
    "modules.priors.fk.enabled", fk_prior_module_enabled_);
  request_priors_module_enabled_ = declare_parameter<bool>(
    "modules.priors.request.enabled", request_priors_module_enabled_);
  rgb_edge_prior_module_enabled_ = declare_parameter<bool>(
    "modules.rgb_edge_prior.enabled", rgb_edge_prior_module_enabled_);
  fk_pose_prior_.tcp_frame = declare_parameter<std::string>("pose_priors.fk.tcp_frame", "");
  fk_pose_prior_.tcp_to_block_xyz = local_point_parameter(
    "pose_priors.fk.tcp_to_block_xyz", {0.0, 0.0, 0.0});
  fk_pose_prior_.tcp_to_block_rpy = local_point_parameter(
    "pose_priors.fk.tcp_to_block_rpy", {0.0, 0.0, 0.0});
  fk_pose_prior_.weight = declare_parameter<double>("pose_priors.fk.weight", 0.0);
  fk_pose_prior_.translation_tolerance_m = declare_parameter<double>(
    "pose_priors.fk.translation_tolerance_m", 0.30);
  fk_pose_prior_.orientation_tolerance_rad = declare_parameter<double>(
    "pose_priors.fk.orientation_tolerance_rad", 0.70);
  if (fk_prior_module_enabled_ && (fk_pose_prior_.weight < 0.0 || !std::isfinite(fk_pose_prior_.weight) ||
    !std::isfinite(fk_pose_prior_.translation_tolerance_m) ||
    !std::isfinite(fk_pose_prior_.orientation_tolerance_rad) ||
    fk_pose_prior_.translation_tolerance_m <= 0.0 ||
    fk_pose_prior_.orientation_tolerance_rad <= 0.0 ||
    (!fk_pose_prior_.tcp_to_block_xyz.allFinite()) ||
    (!fk_pose_prior_.tcp_to_block_rpy.allFinite()) ||
    (fk_pose_prior_.weight > 0.0 && fk_pose_prior_.tcp_frame.empty()))) {
    throw std::invalid_argument("invalid pose_priors.fk configuration");
  }
  rgb_edge_prior_.enabled = declare_parameter<bool>("rgb_edge_prior.enabled", false);
  rgb_edge_prior_.image_topic = declare_parameter<std::string>(
    "rgb_edge_prior.image_topic", "/blackfly_rotated/image_rect");
  rgb_edge_prior_.camera_info_topic = declare_parameter<std::string>(
    "rgb_edge_prior.camera_info_topic", "/blackfly_rotated/camera_info");
  rgb_edge_prior_.max_sync_delta_s = declare_parameter<double>(
    "rgb_edge_prior.max_sync_delta_s", 0.08);
  rgb_edge_prior_.weight = declare_parameter<double>("rgb_edge_prior.weight", 0.05);
  rgb_edge_prior_.edge_percentile = declare_parameter<double>(
    "rgb_edge_prior.edge_percentile", 92.0);
  rgb_edge_prior_.sample_spacing_px = declare_parameter<double>(
    "rgb_edge_prior.sample_spacing_px", 4.0);
  rgb_edge_prior_.distance_scale_px = declare_parameter<double>(
    "rgb_edge_prior.distance_scale_px", 3.0);
  rgb_edge_prior_.min_support_fraction = declare_parameter<double>(
    "rgb_edge_prior.min_support_fraction", 0.15);
  rgb_edge_prior_.min_samples = declare_parameter<int>("rgb_edge_prior.min_samples", 12);
  rgb_edge_prior_.max_image_dimension_px = declare_parameter<int>(
    "rgb_edge_prior.max_image_dimension_px", 768);
  if (rgb_edge_prior_module_enabled_ && rgb_edge_prior_.enabled && (rgb_edge_prior_.image_topic.empty() ||
    rgb_edge_prior_.camera_info_topic.empty() || !std::isfinite(rgb_edge_prior_.max_sync_delta_s) ||
    rgb_edge_prior_.max_sync_delta_s < 0.0 || !std::isfinite(rgb_edge_prior_.weight) ||
    rgb_edge_prior_.weight <= 0.0 || rgb_edge_prior_.min_samples < 1 ||
    rgb_edge_prior_.max_image_dimension_px < 2)) {
    throw std::invalid_argument("invalid rgb_edge_prior configuration");
  }
  debug_.enabled = declare_parameter<bool>("debug.enabled", debug_.enabled);
  debug_.publish_clouds = declare_parameter<bool>("debug.publish_clouds", debug_.publish_clouds);
  debug_.publish_markers = declare_parameter<bool>("debug.publish_markers", debug_.publish_markers);
  debug_.publish_diagnostics = declare_parameter<bool>(
    "debug.publish_diagnostics", debug_.publish_diagnostics);
  debug_.publish_rgb_input = declare_parameter<bool>(
    "debug.publish_rgb_input", debug_.publish_rgb_input);
  debug_.publish_rejected_candidates = declare_parameter<bool>(
    "debug.publish_rejected_candidates", debug_.publish_rejected_candidates);
  debug_.topic_prefix = declare_parameter<std::string>("debug.topic_prefix", debug_.topic_prefix);
  if (debug_.enabled && (debug_.topic_prefix.empty() || debug_.topic_prefix.front() != '/')) {
    throw std::invalid_argument("debug.topic_prefix must be an absolute topic prefix");
  }
  // The existing numeric parameters remain the source of truth. These module
  // switches only make controlled ablations explicit and are recorded in the
  // request debug trace, so disabled is never confused with unavailable.
  refine_enabled_ = refine_enabled_ && sdf_refinement_module_enabled_;
  gripper_self_filter_enabled_ = gripper_self_filter_enabled_ && gripper_self_filter_module_enabled_;
  if (!fk_prior_module_enabled_) {fk_pose_prior_.weight = 0.0;}
  rgb_edge_prior_.enabled = rgb_edge_prior_.enabled && rgb_edge_prior_module_enabled_;
  for (const std::string & rail : {"left_rail", "right_rail"}) {
    const std::string prefix = "gripper_self_filter." + rail;
    GripperRailBoxConfig config;
    config.parent_frame = declare_parameter<std::string>(prefix + ".parent_frame", "");
    config.rail_frame = declare_parameter<std::string>(prefix + ".rail_frame", "");
    config.outward_axis_local = local_point_parameter(prefix + ".outward_axis_local", {0.0, 0.0, 1.0});
    gripper_self_filter_rails_.push_back(std::move(config));
  }
  if (gripper_self_filter_enabled_) {
    detector_core::validate_pzs100_self_filter_parameters({
      gripper_self_filter_outboard_extent_m_, gripper_self_filter_cross_rail_extent_m_,
      gripper_self_filter_rail_end_margin_m_});
    for (const auto & rail : gripper_self_filter_rails_) {
      if (rail.parent_frame.empty() || rail.rail_frame.empty() || rail.outward_axis_local.norm() <= 1e-9) {
        throw std::invalid_argument("enabled gripper self-filter requires parent, rail, and outward-axis configuration");
      }
    }
  }
  cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("poses", 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  if (debug_.enabled) {
    const auto debug_qos = rclcpp::QoS(1).reliable().transient_local();
    if (debug_.publish_markers) {
      debug_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        debug_.topic_prefix + "/markers", debug_qos);
    }
    if (debug_.publish_clouds) {
      debug_input_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        debug_.topic_prefix + "/input_cloud", debug_qos);
      debug_geometry_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        debug_.topic_prefix + "/geometry_cloud", debug_qos);
      debug_above_ground_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        debug_.topic_prefix + "/above_ground_cloud", debug_qos);
    }
    if (debug_.publish_rgb_input) {
      debug_rgb_input_pub_ = create_publisher<sensor_msgs::msg::Image>(
        debug_.topic_prefix + "/rgb_input", debug_qos);
    }
    if (debug_.publish_diagnostics) {
      debug_diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
        debug_.topic_prefix + "/diagnostics", debug_qos);
    }
  }
  // Preserve the selected replay image as a debug outlet even when the RGB
  // scorer is disabled for an ablation.
  if (rgb_edge_prior_.enabled || (debug_.enabled && debug_.publish_rgb_input)) {
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_edge_prior_.image_topic, rclcpp::SensorDataQoS(),
      std::bind(&ConcreteBlockDetectorNode::rgb_callback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      rgb_edge_prior_.camera_info_topic, rclcpp::SensorDataQoS(),
      std::bind(&ConcreteBlockDetectorNode::camera_info_callback, this, std::placeholders::_1));
  }
  discover_blocks_srv_ = create_service<concrete_block_world_model_interfaces::srv::DiscoverBlocks>(discover_service_, std::bind(&ConcreteBlockDetectorNode::handle_discover_blocks, this, std::placeholders::_1, std::placeholders::_2));
}

void ConcreteBlockDetectorNode::start()
{
  if (cloud_sub_) {return;}
  rclcpp::SubscriptionOptions options;
  options.callback_group = cloud_callback_group_;
  cloud_sub_ = point_cloud_transport::create_subscription(
    shared_from_this(), "points",
    std::bind(&ConcreteBlockDetectorNode::cloud_callback, this, std::placeholders::_1),
    point_cloud_transport_name_, rclcpp::SensorDataQoS().get_rmw_qos_profile(), options);
  RCLCPP_INFO(get_logger(), "Concrete block discovery: points (%s transport) -> %s-frame cached snapshots", point_cloud_transport_name_.c_str(), world_frame_.c_str());
}

void ConcreteBlockDetectorNode::rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr image)
{
  std::lock_guard<std::mutex> lock(cached_cloud_mutex_);
  cached_rgb_ = image;
}

void ConcreteBlockDetectorNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
{
  std::lock_guard<std::mutex> lock(cached_cloud_mutex_);
  cached_camera_info_ = camera_info;
}

void ConcreteBlockDetectorNode::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (cloud->header.frame_id.empty()) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring point cloud without a frame_id"); return;}
  sensor_msgs::msg::PointCloud2 world;
  detector_core::SensorContext sensor_context;
  std::vector<GripperFilterBox> gripper_boxes;
  detector_core::PosePriors fk_priors;
  try {
    const auto transform = tf_buffer_.lookupTransform(
      world_frame_, cloud->header.frame_id, cloud->header.stamp,
      tf2::durationFromSec(transform_timeout_s_));
    tf2::doTransform(*cloud, world, transform);
    sensor_context = make_sensor_context(world, transform);
  } catch (const tf2::TransformException & error) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring cloud: no transform %s -> %s at its timestamp: %s", cloud->header.frame_id.c_str(), world_frame_.c_str(), error.what()); return;}
  if (fk_pose_prior_.weight > 0.0) {
    try {
      const auto world_from_tcp = tf_buffer_.lookupTransform(
        world_frame_, fk_pose_prior_.tcp_frame, cloud->header.stamp,
        tf2::durationFromSec(transform_timeout_s_));
      const auto & q = world_from_tcp.transform.rotation;
      const Eigen::Matrix3d world_from_tcp_rotation =
        Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
      detector_core::PosePrior prior;
      prior.source = "fk";
      prior.position = transform_point(world_from_tcp, fk_pose_prior_.tcp_to_block_xyz);
      prior.rotation = world_from_tcp_rotation * detector_core::rotation_from_rpy(fk_pose_prior_.tcp_to_block_rpy);
      prior.dims = detector_parameters_.block_dims;
      prior.weight = fk_pose_prior_.weight;
      prior.translation_tolerance_m = fk_pose_prior_.translation_tolerance_m;
      prior.orientation_tolerance_rad = fk_pose_prior_.orientation_tolerance_rad;
      fk_priors.push_back(std::move(prior));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "FK pose prior skipped: no world TF for '%s' at cloud stamp: %s",
        fk_pose_prior_.tcp_frame.c_str(), error.what());
    }
  }
  if (gripper_self_filter_enabled_) {
    const detector_core::Pzs100SelfFilterParameters filter_parameters{
      gripper_self_filter_outboard_extent_m_, gripper_self_filter_cross_rail_extent_m_,
      gripper_self_filter_rail_end_margin_m_};
    for (const auto & rail : gripper_self_filter_rails_) {
      try {
        const auto world_from_parent = tf_buffer_.lookupTransform(
          world_frame_, rail.parent_frame, cloud->header.stamp,
          tf2::durationFromSec(transform_timeout_s_));
        const auto world_from_rail = tf_buffer_.lookupTransform(
          world_frame_, rail.rail_frame, cloud->header.stamp,
          tf2::durationFromSec(transform_timeout_s_));
        detector_core::Pzs100RailPose rail_pose;
        rail_pose.parent_position = transform_point(world_from_parent, detector_core::Point::Zero());
        rail_pose.rail_position = transform_point(world_from_rail, detector_core::Point::Zero());
        const auto & parent_rotation = world_from_parent.transform.rotation;
        rail_pose.world_from_parent_rotation = Eigen::Quaterniond(
          parent_rotation.w, parent_rotation.x, parent_rotation.y, parent_rotation.z).
          normalized().toRotationMatrix();
        rail_pose.outward_axis_parent = rail.outward_axis_local;
        gripper_boxes.push_back(
          detector_core::make_pzs100_gripper_filter_box(rail_pose, filter_parameters));
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Gripper self-filter skipped rail '%s' -> '%s': no world TF at cloud stamp: %s",
          rail.parent_frame.c_str(), rail.rail_frame.c_str(), error.what());
      } catch (const std::invalid_argument & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Gripper self-filter skipped rail '%s' -> '%s': invalid FK geometry: %s",
          rail.parent_frame.c_str(), rail.rail_frame.c_str(), error.what());
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(cached_cloud_mutex_);
    cached_cloud_world_ = std::make_shared<sensor_msgs::msg::PointCloud2>(world);
    cached_sensor_context_ = std::make_shared<detector_core::SensorContext>(std::move(sensor_context));
    cached_gripper_boxes_ = std::move(gripper_boxes);
    cached_fk_priors_ = std::move(fk_priors);
  }
  cached_cloud_cv_.notify_all();
}

void ConcreteBlockDetectorNode::handle_discover_blocks(const std::shared_ptr<concrete_block_world_model_interfaces::srv::DiscoverBlocks::Request> request, std::shared_ptr<concrete_block_world_model_interfaces::srv::DiscoverBlocks::Response> response)
{
  if (!std::isfinite(request->timeout_s) || request->timeout_s <= 0.0F) {
    response->success = false;
    response->message = "timeout_s must be positive.";
    return;
  }
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  std::shared_ptr<detector_core::SensorContext> sensor_context;
  std::vector<GripperFilterBox> gripper_boxes;
  detector_core::PosePriors priors;
  std::size_t request_prior_count = 0U;
  sensor_msgs::msg::Image::ConstSharedPtr rgb;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
  if (request_priors_module_enabled_) {
    try {
      priors.reserve(request->priors.size());
      for (const auto & message : request->priors) {priors.push_back(pose_prior_from_msg(message));}
      request_prior_count = priors.size();
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("Invalid request pose prior: ") + error.what();
      return;
    }
  }
  const auto cached_cloud_is_current = [this]() {
      if (!cached_cloud_world_ || !cached_sensor_context_) {
        return false;
      }
      const rclcpp::Time stamp(cached_cloud_world_->header.stamp, get_clock()->get_clock_type());
      const double age_s = (now() - stamp).seconds();
      return age_s >= -0.1 && age_s <= cached_cloud_max_age_s_;
    };
  {
    std::unique_lock<std::mutex> lock(cached_cloud_mutex_);
    const auto ready = cached_cloud_cv_.wait_for(
      lock, std::chrono::duration<float>(request->timeout_s),
      cached_cloud_is_current);
    if (!ready) {
      response->success = false;
      response->message = "Timed out waiting for a current valid world-frame point cloud.";
      return;
    }
    cloud = cached_cloud_world_;
    sensor_context = cached_sensor_context_;
    gripper_boxes = cached_gripper_boxes_;
    priors.insert(priors.end(), cached_fk_priors_.begin(), cached_fk_priors_.end());
    rgb = cached_rgb_;
    camera_info = cached_camera_info_;
  }
  try {
    response->blocks = discover(
      *cloud, *sensor_context, gripper_boxes, priors, request_prior_count, rgb, camera_info,
      response->ground_height_m);
    response->success = true;
    response->message = "Discovered " + std::to_string(response->blocks.blocks.size()) + " block(s).";
  } catch (const std::exception & error) {response->success = false; response->message = std::string("Discovery failed: ") + error.what(); RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());}
}

detector_core::Points ConcreteBlockDetectorNode::filter_scene_points(
  const sensor_msgs::msg::PointCloud2 & cloud_world, const std::vector<GripperFilterBox> & gripper_boxes,
  const detector_core::PosePriors & priors, visualization_msgs::msg::MarkerArray & markers,
  std::size_t & gripper_points_removed)
{
  auto points = ros_points(cloud_world);
  if (scene_bounds_enabled_) {points.erase(std::remove_if(points.begin(), points.end(), [this](const auto & point) {return point.x() < scene_bounds_min_m_[0] || point.x() > scene_bounds_max_m_[0] || point.y() < scene_bounds_min_m_[1] || point.y() > scene_bounds_max_m_[1] || point.z() < scene_bounds_min_m_[2] || point.z() > scene_bounds_max_m_[2];}), points.end());}
  if (debug_input_cloud_pub_) {
    debug_input_cloud_pub_->publish(cloud_from_points(cloud_world.header, points));
  }
  gripper_points_removed = 0U;
  if (!gripper_boxes.empty()) {
    std::vector<detector_core::OrientedBox> boxes;
    boxes.reserve(gripper_boxes.size());
    for (const auto & filter_box : gripper_boxes) {boxes.push_back(filter_box.box);}
    points = detector_core::remove_points_inside_oriented_boxes(points, boxes, &gripper_points_removed);
  }
  if (debug_geometry_cloud_pub_) {
    debug_geometry_cloud_pub_->publish(cloud_from_points(cloud_world.header, points));
  }
  if (gripper_self_filter_publish_markers_ && !gripper_boxes.empty()) {
    markers.markers.push_back(make_gripper_centerlines_marker(cloud_world.header, gripper_boxes));
    markers.markers.push_back(make_gripper_outward_arrows_marker(cloud_world.header, gripper_boxes));
    int marker_id = 0;
    for (const auto & filter_box : gripper_boxes) {
      markers.markers.push_back(make_gripper_box_marker(cloud_world.header, filter_box, marker_id++));
    }
  }
  int prior_marker_id = 0;
  for (const auto & prior : priors) {
    if (prior.weight > 0.0) {markers.markers.push_back(make_prior_marker(cloud_world.header, prior, prior_marker_id++));}
  }
  return points;
}

RgbPriorOutcome ConcreteBlockDetectorNode::compute_rgb_prior(
  const std_msgs::msg::Header & cloud_header, const std::vector<GripperFilterBox> & gripper_boxes,
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
  std::optional<RgbEdgePrior> & rgb_prior_storage)
{
  RgbPriorOutcome outcome;
  outcome.rgb_gate_reason = rgb_edge_prior_.enabled ? "not_evaluated" : "disabled";
  if (!rgb_edge_prior_.enabled) {return outcome;}
  const auto cloud_stamp_ns = rclcpp::Time(cloud_header.stamp).nanoseconds();
  const auto image_stamp_ns = rgb ? rclcpp::Time(rgb->header.stamp).nanoseconds() : 0;
  const double delta_s = rgb ? std::abs(static_cast<double>(cloud_stamp_ns - image_stamp_ns)) * 1.0e-9 : std::numeric_limits<double>::infinity();
  outcome.rgb_sync_delta_s = delta_s;
  const auto gray = rgb ? grayscale_image(*rgb) : std::nullopt;
  const auto matrix = camera_info ? camera_matrix_from_info(*camera_info) : std::nullopt;
  if (!rgb || !camera_info || !gray || !matrix || delta_s > rgb_edge_prior_.max_sync_delta_s ||
    gray->cols != static_cast<int>(camera_info->width) || gray->rows != static_cast<int>(camera_info->height) ||
    camera_info->header.frame_id.empty()) {
    outcome.rgb_gate_reason = "missing_or_invalid_input_or_sync";
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "RGB edge prior skipped: missing/unsupported RGB or CameraInfo, or image-cloud delta %.3f s exceeds %.3f s",
      delta_s, rgb_edge_prior_.max_sync_delta_s);
    return outcome;
  }
  try {
    const auto world_from_camera = tf_buffer_.lookupTransform(
      world_frame_, camera_info->header.frame_id, cloud_header.stamp,
      tf2::durationFromSec(transform_timeout_s_));
    cv::Mat distortion(1, static_cast<int>(camera_info->d.size()), CV_64F);
    for (std::size_t index = 0; index < camera_info->d.size(); ++index) {
      distortion.at<double>(0, static_cast<int>(index)) = camera_info->d[index];
    }
    cv::Mat excluded = gripper_occlusion_mask(
      gripper_boxes, *matrix, distortion, isometry_from_transform(world_from_camera), gray->size());
    RgbEdgePriorParameters parameters;
    parameters.weight = rgb_edge_prior_.weight;
    parameters.edge_percentile = rgb_edge_prior_.edge_percentile;
    parameters.sample_spacing_px = rgb_edge_prior_.sample_spacing_px;
    parameters.distance_scale_px = rgb_edge_prior_.distance_scale_px;
    parameters.min_support_fraction = rgb_edge_prior_.min_support_fraction;
    parameters.min_samples = rgb_edge_prior_.min_samples;
    parameters.max_image_dimension_px = rgb_edge_prior_.max_image_dimension_px;
    // Stores its own copies of gray/matrix/distortion/excluded internally
    // (see RgbEdgePrior's constructor), so the scorer below outliving these
    // block-scoped locals is fine -- only rgb_prior_storage's lifetime,
    // owned by the caller, matters.
    rgb_prior_storage.emplace(*gray, *matrix, distortion, isometry_from_transform(world_from_camera), excluded, parameters);
    if (rgb_prior_storage->ready()) {
      outcome.visual_scorer = [&rgb_prior_storage](const detector_core::Pose & pose) {return rgb_prior_storage->score(pose);};
      outcome.rgb_edge_available = true;
      outcome.rgb_gate_reason = "applied";
    } else {
      outcome.rgb_gate_reason = "edge_map_has_no_support";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "RGB edge prior skipped: edge map has no usable support");
    }
  } catch (const tf2::TransformException & error) {
    outcome.rgb_gate_reason = "camera_tf_unavailable";
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "RGB edge prior skipped: no world TF for camera '%s' at cloud stamp: %s",
      camera_info->header.frame_id.c_str(), error.what());
  }
  return outcome;
}

concrete_block_world_model_interfaces::msg::BlockArray ConcreteBlockDetectorNode::build_block_array(
  const std_msgs::msg::Header & header, const detector_core::DetectionResult & detection,
  geometry_msgs::msg::PoseArray & poses, visualization_msgs::msg::MarkerArray & markers)
{
  concrete_block_world_model_interfaces::msg::BlockArray result;
  result.header = header;
  int marker_id = 0;
  for (const auto & hypothesis : detection.hypotheses) {
    const Eigen::Quaterniond orientation(hypothesis.pose.rotation);
    geometry_msgs::msg::Pose pose;
    pose.position.x = hypothesis.pose.position.x();
    pose.position.y = hypothesis.pose.position.y();
    pose.position.z = hypothesis.pose.position.z();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    poses.poses.push_back(pose);
    markers.markers.push_back(make_marker(poses.header, pose, hypothesis.pose.dims, marker_id++));

    concrete_block_world_model_interfaces::msg::Block block;
    block.pose = pose;
    const auto observed_faces = static_cast<std::uint8_t>(
      std::min<std::size_t>(hypothesis.evidence.observed_geometry_faces, 255U));
    // A top face alone leaves the in-plane pose underconstrained. Two or more
    // observed faces geometrically constrain the fitted cuboid.
    block.pose_status = observed_faces >= 2U ?
      concrete_block_world_model_interfaces::msg::Block::POSE_PRECISE :
      concrete_block_world_model_interfaces::msg::Block::POSE_COARSE;
    block.task_status = concrete_block_world_model_interfaces::msg::Block::TASK_FREE;
    block.confidence = static_cast<float>(std::clamp(hypothesis.evidence.score, 0.0, 1.0));
    block.observed_faces = observed_faces;
    block.last_seen = header.stamp;
    result.blocks.push_back(std::move(block));
  }
  return result;
}

void ConcreteBlockDetectorNode::publish_debug(
  const std_msgs::msg::Header & header, const std::vector<GripperFilterBox> & gripper_boxes,
  const detector_core::PosePriors & priors, const detector_core::DetectionResult & detection,
  std::size_t request_prior_count, std::size_t gripper_points_removed,
  const RgbPriorOutcome & rgb_outcome, const sensor_msgs::msg::Image::ConstSharedPtr & rgb)
{
  if (debug_above_ground_cloud_pub_) {
    debug_above_ground_cloud_pub_->publish(cloud_from_points(header, detection.above_support_points));
  }
  if (debug_markers_pub_) {
    debug_markers_pub_->publish(build_debug_candidate_markers(
      header, gripper_boxes, priors, detection, debug_.publish_rejected_candidates));
  }
  if (debug_rgb_input_pub_ && rgb) {debug_rgb_input_pub_->publish(*rgb);}
  if (debug_diagnostics_pub_) {
    // Bundled once so a debug-only field (RGB gate outcome, points the
    // gripper self-filter removed, ...) can't go stale by being recomputed
    // differently than what the rest of this function already resolved.
    DiscoveryDebugState debug_state;
    debug_state.request_prior_count = request_prior_count;
    debug_state.gripper_points_removed = gripper_points_removed;
    debug_state.rgb_edge_available = rgb_outcome.rgb_edge_available;
    debug_state.rgb_gate_reason = rgb_outcome.rgb_gate_reason;
    debug_state.rgb_sync_delta_s = rgb_outcome.rgb_sync_delta_s;
    debug_state.publish_rejected_candidates = debug_.publish_rejected_candidates;
    debug_state.scene_bounds_enabled = scene_bounds_enabled_;
    debug_state.gripper_self_filter_module_enabled = gripper_self_filter_module_enabled_;
    debug_state.gripper_self_filter_enabled = gripper_self_filter_enabled_;
    debug_state.sdf_refinement_module_enabled = sdf_refinement_module_enabled_;
    debug_state.refine_enabled = refine_enabled_;
    debug_state.fit_wide_proposals = detector_parameters_.fit_wide_proposals;
    debug_state.fk_prior_module_enabled = fk_prior_module_enabled_;
    debug_state.fk_prior_weight_positive = fk_pose_prior_.weight > 0.0;
    debug_state.request_priors_module_enabled = request_priors_module_enabled_;
    debug_state.rgb_edge_prior_module_enabled = rgb_edge_prior_module_enabled_;
    debug_state.rgb_edge_prior_enabled = rgb_edge_prior_.enabled;
    debug_diagnostics_pub_->publish(
      build_diagnostics_message(header, gripper_boxes, priors, detection, debug_state));
  }
}

concrete_block_world_model_interfaces::msg::BlockArray ConcreteBlockDetectorNode::discover(
  const sensor_msgs::msg::PointCloud2 & cloud_world,
  const detector_core::SensorContext & sensor_context,
  const std::vector<GripperFilterBox> & gripper_boxes,
  const detector_core::PosePriors & priors,
  std::size_t request_prior_count,
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
  double & ground_height_m)
{
  geometry_msgs::msg::PoseArray poses; poses.header = cloud_world.header;
  visualization_msgs::msg::MarkerArray markers; visualization_msgs::msg::Marker clear; clear.header = cloud_world.header; clear.action = visualization_msgs::msg::Marker::DELETEALL; markers.markers.push_back(clear);
  std::size_t gripper_points_removed = 0U;
  const auto points = filter_scene_points(cloud_world, gripper_boxes, priors, markers, gripper_points_removed);
  std::optional<RgbEdgePrior> rgb_prior;
  const auto rgb_outcome = compute_rgb_prior(cloud_world.header, gripper_boxes, rgb, camera_info, rgb_prior);
  const auto * scorer = rgb_outcome.visual_scorer ? &rgb_outcome.visual_scorer : nullptr;
  const auto detection = refine_enabled_ ? detector_core::detect(points, detector_parameters_, &sensor_context, &priors, scorer) :
    detector_core::detect_without_refinement(points, detector_parameters_, &sensor_context, &priors, scorer);
  auto result = build_block_array(cloud_world.header, detection, poses, markers);
  publish_debug(
    cloud_world.header, gripper_boxes, priors, detection, request_prior_count, gripper_points_removed,
    rgb_outcome, rgb);
  RCLCPP_INFO(
    get_logger(), "Scene discovery input: %zu point(s), gripper self-filter removed %zu point(s) using %zu rail box(es), %zu pose prior(s)",
    points.size() + gripper_points_removed, gripper_points_removed, gripper_boxes.size(), priors.size());
  // Free side effect of the ground-plane fit `detection` already computed:
  // report the local support height near the scene centroid so callers that
  // just want "what's the ground z here" (e.g. wall-origin setup) don't need
  // their own ground-fitting pass. Single scalar, locally-planar assumption
  // -- not a terrain map.
  if (points.empty()) {
    ground_height_m = std::numeric_limits<double>::quiet_NaN();
  } else {
    detector_core::Point centroid = detector_core::Point::Zero();
    for (const auto & point : points) {centroid += point;}
    centroid /= static_cast<double>(points.size());
    ground_height_m = detection.ground.support_z(centroid);
  }
  poses_pub_->publish(poses); markers_pub_->publish(markers); return result;
}
}  // namespace concrete_block_detector
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<concrete_block_detector::ConcreteBlockDetectorNode>();
  node->start();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

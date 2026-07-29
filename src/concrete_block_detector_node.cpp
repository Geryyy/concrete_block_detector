#include "concrete_block_detector/concrete_block_detector_node.hpp"
#include "concrete_block_detector/detector_core_pipeline.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

namespace concrete_block_detector
{
namespace
{
constexpr std::array<double, 3> kDefaultDims{{0.9, 0.6, 0.6}};

visualization_msgs::msg::Marker make_marker(
  const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose,
  const std::array<double, 3> & dims, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "concrete_blocks";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = dims[0];
  marker.scale.y = dims[1];
  marker.scale.z = dims[2];
  marker.color.r = 0.95F;
  marker.color.g = 0.55F;
  marker.color.b = 0.10F;
  marker.color.a = 0.75F;
  return marker;
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
  cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("poses", 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
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

void ConcreteBlockDetectorNode::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (cloud->header.frame_id.empty()) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring point cloud without a frame_id"); return;}
  sensor_msgs::msg::PointCloud2 world;
  detector_core::SensorContext sensor_context;
  try {
    const auto transform = tf_buffer_.lookupTransform(
      world_frame_, cloud->header.frame_id, cloud->header.stamp,
      tf2::durationFromSec(transform_timeout_s_));
    tf2::doTransform(*cloud, world, transform);
    sensor_context = make_sensor_context(world, transform);
  } catch (const tf2::TransformException & error) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring cloud: no transform %s -> %s at its timestamp: %s", cloud->header.frame_id.c_str(), world_frame_.c_str(), error.what()); return;}
  {
    std::lock_guard<std::mutex> lock(cached_cloud_mutex_);
    cached_cloud_world_ = std::make_shared<sensor_msgs::msg::PointCloud2>(world);
    cached_sensor_context_ = std::make_shared<detector_core::SensorContext>(std::move(sensor_context));
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
  }
  try {response->blocks = discover(*cloud, *sensor_context); response->success = true; response->message = "Discovered " + std::to_string(response->blocks.blocks.size()) + " block(s).";} catch (const std::exception & error) {response->success = false; response->message = std::string("Discovery failed: ") + error.what(); RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());}
}

concrete_block_world_model_interfaces::msg::BlockArray ConcreteBlockDetectorNode::discover(
  const sensor_msgs::msg::PointCloud2 & cloud_world,
  const detector_core::SensorContext & sensor_context)
{
  concrete_block_world_model_interfaces::msg::BlockArray result; result.header = cloud_world.header;
  geometry_msgs::msg::PoseArray poses; poses.header = cloud_world.header;
  visualization_msgs::msg::MarkerArray markers; visualization_msgs::msg::Marker clear; clear.header = cloud_world.header; clear.action = visualization_msgs::msg::Marker::DELETEALL; markers.markers.push_back(clear);
  auto points = ros_points(cloud_world);
  if (scene_bounds_enabled_) {points.erase(std::remove_if(points.begin(), points.end(), [this](const auto & point) {return point.x() < scene_bounds_min_m_[0] || point.x() > scene_bounds_max_m_[0] || point.y() < scene_bounds_min_m_[1] || point.y() > scene_bounds_max_m_[1] || point.z() < scene_bounds_min_m_[2] || point.z() > scene_bounds_max_m_[2];}), points.end());}
  auto detection = refine_enabled_ ? detector_core::detect(points, detector_parameters_, &sensor_context) :
    detector_core::detect_without_refinement(points, detector_parameters_, &sensor_context);
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
    block.last_seen = cloud_world.header.stamp;
    result.blocks.push_back(std::move(block));
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

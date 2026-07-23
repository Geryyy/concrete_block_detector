#include "concrete_block_detector/concrete_block_detector_node.hpp"
#include "concrete_block_detector/detector_core_pipeline.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
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

detector_core::DetectionParameters core_parameters(const ConcreteBlockDetectorNode &)
{
  // Defaults intentionally mirror blockpose.DetectionParams.  Parameters are
  // declared here when the adapter is constructed; no legacy PCA controls are
  // reachable from this path.
  return {};
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
  point_cloud_transport_name_(declare_parameter<std::string>("point_cloud_transport", "raw")),
  discover_service_(declare_parameter<std::string>("discover_service", "~/discover_blocks")),
  refine_enabled_(declare_parameter<bool>("refine_enabled", true)),
  tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  if (world_frame_.empty() || transform_timeout_s_ < 0.0 || point_cloud_transport_name_.empty() || discover_service_.empty() ||
    scene_bounds_min_m_[0] > scene_bounds_max_m_[0] || scene_bounds_min_m_[1] > scene_bounds_max_m_[1] || scene_bounds_min_m_[2] > scene_bounds_max_m_[2])
  {throw std::invalid_argument("invalid concrete_block_detector parameter");}
  poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("poses", 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  discover_blocks_srv_ = create_service<concrete_block_world_model_interfaces::srv::DiscoverBlocks>(discover_service_, std::bind(&ConcreteBlockDetectorNode::handle_discover_blocks, this, std::placeholders::_1, std::placeholders::_2));
}

void ConcreteBlockDetectorNode::start()
{
  if (cloud_sub_) {return;}
  point_cloud_transport_ = std::make_shared<point_cloud_transport::PointCloudTransport>(shared_from_this());
  const point_cloud_transport::TransportHints hints(point_cloud_transport_name_);
  cloud_sub_ = point_cloud_transport_->subscribe("points", rclcpp::SensorDataQoS().get_rmw_qos_profile(), std::bind(&ConcreteBlockDetectorNode::cloud_callback, this, std::placeholders::_1), {}, &hints);
  RCLCPP_INFO(get_logger(), "Concrete block discovery: points (%s transport) -> %s-frame cached snapshots", point_cloud_transport_name_.c_str(), world_frame_.c_str());
}

void ConcreteBlockDetectorNode::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (cloud->header.frame_id.empty()) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring point cloud without a frame_id"); return;}
  sensor_msgs::msg::PointCloud2 world;
  try {const auto transform = tf_buffer_.lookupTransform(world_frame_, cloud->header.frame_id, cloud->header.stamp, tf2::durationFromSec(transform_timeout_s_)); tf2::doTransform(*cloud, world, transform);} catch (const tf2::TransformException & error) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring cloud: no transform %s -> %s at its timestamp: %s", cloud->header.frame_id.c_str(), world_frame_.c_str(), error.what()); return;}
  std::lock_guard<std::mutex> lock(cached_cloud_mutex_); cached_cloud_world_ = std::make_shared<sensor_msgs::msg::PointCloud2>(world);
}

void ConcreteBlockDetectorNode::handle_discover_blocks(const std::shared_ptr<concrete_block_world_model_interfaces::srv::DiscoverBlocks::Request> request, std::shared_ptr<concrete_block_world_model_interfaces::srv::DiscoverBlocks::Response> response)
{
  (void)request; sensor_msgs::msg::PointCloud2::SharedPtr cloud; {std::lock_guard<std::mutex> lock(cached_cloud_mutex_); cloud = cached_cloud_world_;}
  if (!cloud) {response->success = false; response->message = "No valid world-frame point cloud has been received yet."; return;}
  try {response->blocks = discover(*cloud); response->success = true; response->message = "Discovered " + std::to_string(response->blocks.blocks.size()) + " block(s).";} catch (const std::exception & error) {response->success = false; response->message = std::string("Discovery failed: ") + error.what(); RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());}
}

concrete_block_world_model_interfaces::msg::BlockArray ConcreteBlockDetectorNode::discover(const sensor_msgs::msg::PointCloud2 & cloud_world)
{
  concrete_block_world_model_interfaces::msg::BlockArray result; result.header = cloud_world.header;
  geometry_msgs::msg::PoseArray poses; poses.header = cloud_world.header;
  visualization_msgs::msg::MarkerArray markers; visualization_msgs::msg::Marker clear; clear.header = cloud_world.header; clear.action = visualization_msgs::msg::Marker::DELETEALL; markers.markers.push_back(clear);
  auto points = ros_points(cloud_world);
  if (scene_bounds_enabled_) {points.erase(std::remove_if(points.begin(), points.end(), [this](const auto & point) {return point.x() < scene_bounds_min_m_[0] || point.x() > scene_bounds_max_m_[0] || point.y() < scene_bounds_min_m_[1] || point.y() > scene_bounds_max_m_[1] || point.z() < scene_bounds_min_m_[2] || point.z() > scene_bounds_max_m_[2];}), points.end());}
  auto detection = refine_enabled_ ? detector_core::detect(points, core_parameters(*this)) :
    detector_core::detect_without_refinement(points, core_parameters(*this));
  int marker_id = 0; for (const auto & hypothesis : detection.hypotheses) {const Eigen::Quaterniond orientation(hypothesis.pose.rotation); geometry_msgs::msg::Pose pose; pose.position.x = hypothesis.pose.position.x(); pose.position.y = hypothesis.pose.position.y(); pose.position.z = hypothesis.pose.position.z(); pose.orientation.x = orientation.x(); pose.orientation.y = orientation.y(); pose.orientation.z = orientation.z(); pose.orientation.w = orientation.w(); poses.poses.push_back(pose); markers.markers.push_back(make_marker(poses.header, pose, hypothesis.pose.dims, marker_id++)); concrete_block_world_model_interfaces::msg::Block block; block.pose = pose; block.pose_status = concrete_block_world_model_interfaces::msg::Block::POSE_COARSE; block.task_status = concrete_block_world_model_interfaces::msg::Block::TASK_FREE; block.confidence = static_cast<float>(std::clamp(hypothesis.evidence.score, 0.0, 1.0)); block.last_seen = cloud_world.header.stamp; result.blocks.push_back(std::move(block));}
  poses_pub_->publish(poses); markers_pub_->publish(markers); return result;
}
}  // namespace concrete_block_detector
int main(int argc, char * argv[]) {rclcpp::init(argc, argv); auto node = std::make_shared<concrete_block_detector::ConcreteBlockDetectorNode>(); node->start(); rclcpp::spin(node); rclcpp::shutdown(); return 0;}

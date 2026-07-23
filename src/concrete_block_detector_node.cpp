#include "concrete_block_detector/concrete_block_detector_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace concrete_block_detector
{
namespace
{

constexpr std::array<float, 3> kBlockDimensions{0.9F, 0.6F, 0.6F};
constexpr double kPi = 3.14159265358979323846;

double squared_distance(
  const geometry_msgs::msg::Pose & lhs,
  const geometry_msgs::msg::Pose & rhs)
{
  const double dx = lhs.position.x - rhs.position.x;
  const double dy = lhs.position.y - rhs.position.y;
  const double dz = lhs.position.z - rhs.position.z;
  return dx * dx + dy * dy + dz * dz;
}

bool is_finite(const pcl::PointXYZ & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

struct OrientedBounds
{
  Eigen::Vector3f minimum{Eigen::Vector3f::Zero()};
  Eigen::Vector3f maximum{Eigen::Vector3f::Zero()};
};

OrientedBounds oriented_bounds(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cluster,
  const Eigen::Matrix3f & rotation)
{
  OrientedBounds bounds;
  bounds.minimum = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  bounds.maximum = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
  for (const auto & point : cluster->points) {
    const Eigen::Vector3f local = rotation.transpose() * Eigen::Vector3f(point.x, point.y, point.z);
    bounds.minimum = bounds.minimum.cwiseMin(local);
    bounds.maximum = bounds.maximum.cwiseMax(local);
  }
  return bounds;
}

bool observed_spans_fit_block(const OrientedBounds & bounds, double tolerance_m)
{
  const Eigen::Vector3f spans = bounds.maximum - bounds.minimum;
  for (std::size_t index = 0; index < kBlockDimensions.size(); ++index) {
    if (spans[static_cast<Eigen::Index>(index)] > kBlockDimensions[index] + tolerance_m) {
      return false;
    }
  }
  return true;
}

Eigen::Matrix3f canonicalize_long_axis(
  const Eigen::Matrix3f & pca_rotation,
  Eigen::Index long_axis_index)
{
  Eigen::Matrix3f rotation;
  rotation.col(0) = pca_rotation.col(long_axis_index);
  Eigen::Index short_axis_index = long_axis_index == 0 ? 1 : 0;
  rotation.col(1) = pca_rotation.col(short_axis_index);
  // The two 0.6 m dimensions are symmetric, so their ordering has no physical meaning.
  rotation.col(2) = rotation.col(0).cross(rotation.col(1)).normalized();
  return rotation;
}

Eigen::Vector3f estimate_cuboid_center(
  const OrientedBounds & bounds,
  const Eigen::Matrix3f & rotation,
  const Eigen::Vector3f & sensor_origin_world,
  double tolerance_m)
{
  const Eigen::Vector3f spans = bounds.maximum - bounds.minimum;
  const Eigen::Vector3f sensor_local = rotation.transpose() * sensor_origin_world;
  Eigen::Vector3f center_local;
  for (std::size_t index = 0; index < kBlockDimensions.size(); ++index) {
    const auto axis = static_cast<Eigen::Index>(index);
    const float half_dimension = kBlockDimensions[index] / 2.0F;
    const float midpoint = (bounds.minimum[axis] + bounds.maximum[axis]) / 2.0F;
    const float fully_observed_margin = static_cast<float>(
      std::min(tolerance_m, static_cast<double>(kBlockDimensions[index]) * 0.1));
    if (spans[axis] >= kBlockDimensions[index] - fully_observed_margin) {
      center_local[axis] = midpoint;
    } else if (sensor_local[axis] >= midpoint) {
      // The sensor sees the positive face; place that observed face at +half-dimension.
      center_local[axis] = bounds.maximum[axis] - half_dimension;
    } else {
      // The sensor sees the negative face; place that observed face at -half-dimension.
      center_local[axis] = bounds.minimum[axis] + half_dimension;
    }
  }
  return rotation * center_local;
}

visualization_msgs::msg::Marker make_marker(
  const std_msgs::msg::Header & header,
  const geometry_msgs::msg::Pose & pose,
  int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "concrete_blocks";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = kBlockDimensions[0];
  marker.scale.y = kBlockDimensions[1];
  marker.scale.z = kBlockDimensions[2];
  marker.color.r = 0.95F;
  marker.color.g = 0.55F;
  marker.color.b = 0.10F;
  marker.color.a = 0.75F;
  return marker;
}

}  // namespace

ConcreteBlockDetectorNode::ConcreteBlockDetectorNode(const rclcpp::NodeOptions & options)
: Node("concrete_block_detector", options),
  world_frame_(declare_parameter<std::string>("world_frame", "world")),
  voxel_leaf_size_m_(declare_parameter<double>("voxel_leaf_size_m", 0.04)),
  ground_distance_threshold_m_(declare_parameter<double>("ground_distance_threshold_m", 0.05)),
  ground_max_iterations_(declare_parameter<int>("ground_max_iterations", 500)),
  ground_max_tilt_deg_(declare_parameter<double>("ground_max_tilt_deg", 20.0)),
  cluster_tolerance_m_(declare_parameter<double>("cluster_tolerance_m", 0.10)),
  cluster_min_points_(declare_parameter<int>("cluster_min_points", 50)),
  cluster_max_points_(declare_parameter<int>("cluster_max_points", 50000)),
  dimension_tolerance_m_(declare_parameter<double>("dimension_tolerance_m", 0.30)),
  enforce_max_dimensions_(declare_parameter<bool>("enforce_max_dimensions", true)),
  minimum_long_axis_span_m_(declare_parameter<double>("minimum_long_axis_span_m", 0.70)),
  minimum_secondary_axis_span_m_(declare_parameter<double>("minimum_secondary_axis_span_m", 0.30)),
  transform_timeout_s_(declare_parameter<double>("transform_timeout_s", 0.10)),
  world_model_enabled_(declare_parameter<bool>("world_model.enabled", true)),
  world_model_frame_(declare_parameter<std::string>("world_model.frame_id", "world")),
  get_coarse_blocks_service_(declare_parameter<std::string>(
      "world_model.get_coarse_blocks_service", "/world_model_node/get_coarse_blocks")),
  upsert_block_service_(declare_parameter<std::string>(
      "world_model.upsert_block_service", "/world_model_node/upsert_block")),
  world_model_id_prefix_(declare_parameter<std::string>("world_model.id_prefix", "detected_block_")),
  world_model_association_max_distance_m_(declare_parameter<double>(
      "world_model.association_max_distance_m", 0.45)),
  world_model_cache_refresh_s_(declare_parameter<double>(
      "world_model.cache_refresh_s", 1.0)),
  world_model_confidence_(declare_parameter<double>("world_model.confidence", 0.8)),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  if (world_frame_.empty() || voxel_leaf_size_m_ <= 0.0 || ground_distance_threshold_m_ <= 0.0 ||
    ground_max_iterations_ < 1 || ground_max_tilt_deg_ < 0.0 || ground_max_tilt_deg_ > 90.0 ||
    cluster_tolerance_m_ <= 0.0 || cluster_min_points_ < 1 ||
    cluster_max_points_ < cluster_min_points_ || dimension_tolerance_m_ < 0.0 ||
    minimum_long_axis_span_m_ <= kBlockDimensions[1] ||
    minimum_long_axis_span_m_ > kBlockDimensions[0] ||
    minimum_secondary_axis_span_m_ <= 0.0 ||
    minimum_secondary_axis_span_m_ > kBlockDimensions[1] ||
    transform_timeout_s_ < 0.0 || world_model_frame_.empty() || world_model_id_prefix_.empty() ||
    world_model_association_max_distance_m_ <= 0.0 || world_model_cache_refresh_s_ <= 0.0 ||
    world_model_confidence_ < 0.0 || world_model_confidence_ > 1.0)
  {
    throw std::invalid_argument("invalid concrete_block_detector parameter");
  }

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "points", rclcpp::SensorDataQoS(),
    std::bind(&ConcreteBlockDetectorNode::cloud_callback, this, std::placeholders::_1));
  poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("poses", 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  if (world_model_enabled_) {
    get_coarse_blocks_client_ = create_client<
      concrete_block_world_model_interfaces::srv::GetCoarseBlocks>(get_coarse_blocks_service_);
    upsert_block_client_ = create_client<
      concrete_block_world_model_interfaces::srv::UpsertBlock>(upsert_block_service_);
  }
  RCLCPP_INFO(get_logger(), "Concrete block discovery: points -> %s-frame poses", world_frame_.c_str());
}

void ConcreteBlockDetectorNode::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (cloud->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring point cloud without a frame_id");
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_world;
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      world_frame_, cloud->header.frame_id, cloud->header.stamp,
      tf2::durationFromSec(transform_timeout_s_));
    tf2::doTransform(*cloud, cloud_world, transform);
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring cloud: no transform %s -> %s at its timestamp: %s",
      cloud->header.frame_id.c_str(), world_frame_.c_str(), error.what());
    return;
  }

  auto input = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(cloud_world, *input);
  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(*input, *input, finite_indices);

  geometry_msgs::msg::PoseArray poses;
  poses.header = cloud_world.header;
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header = cloud_world.header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  if (input->empty()) {
    poses_pub_->publish(poses);
    markers_pub_->publish(markers);
    return;
  }

  auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(input);
  const auto leaf = static_cast<float>(voxel_leaf_size_m_);
  voxel_grid.setLeafSize(leaf, leaf, leaf);
  voxel_grid.filter(*downsampled);
  if (downsampled->size() < static_cast<std::size_t>(cluster_min_points_)) {
    poses_pub_->publish(poses);
    markers_pub_->publish(markers);
    return;
  }

  pcl::SACSegmentation<pcl::PointXYZ> ground_segmenter;
  ground_segmenter.setOptimizeCoefficients(true);
  ground_segmenter.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  ground_segmenter.setMethodType(pcl::SAC_RANSAC);
  ground_segmenter.setDistanceThreshold(ground_distance_threshold_m_);
  ground_segmenter.setMaxIterations(ground_max_iterations_);
  ground_segmenter.setAxis(Eigen::Vector3f::UnitZ());
  ground_segmenter.setEpsAngle(static_cast<float>(ground_max_tilt_deg_ * kPi / 180.0));
  ground_segmenter.setInputCloud(downsampled);
  auto ground_inliers = std::make_shared<pcl::PointIndices>();
  auto ground_coefficients = std::make_shared<pcl::ModelCoefficients>();
  ground_segmenter.segment(*ground_inliers, *ground_coefficients);

  auto candidates = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (ground_inliers->indices.empty()) {
    candidates = downsampled;
  } else {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(downsampled);
    extract.setIndices(ground_inliers);
    extract.setNegative(true);
    extract.filter(*candidates);
  }
  if (candidates->size() < static_cast<std::size_t>(cluster_min_points_)) {
    poses_pub_->publish(poses);
    markers_pub_->publish(markers);
    return;
  }

  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud(candidates);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor;
  cluster_extractor.setClusterTolerance(cluster_tolerance_m_);
  cluster_extractor.setMinClusterSize(cluster_min_points_);
  cluster_extractor.setMaxClusterSize(cluster_max_points_);
  cluster_extractor.setSearchMethod(tree);
  cluster_extractor.setInputCloud(candidates);
  std::vector<pcl::PointIndices> cluster_indices;
  cluster_extractor.extract(cluster_indices);

  int marker_id = 0;
  for (const auto & indices : cluster_indices) {
    auto cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cluster->reserve(indices.indices.size());
    for (const int index : indices.indices) {
      const auto & point = candidates->points.at(static_cast<std::size_t>(index));
      if (is_finite(point)) {
        cluster->push_back(point);
      }
    }
    if (cluster->size() < 3U) {
      continue;
    }

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cluster);
    Eigen::Matrix3f rotation = pca.getEigenVectors();
    if (rotation.determinant() < 0.0F) {
      rotation.col(2) *= -1.0F;
    }
    const OrientedBounds pca_bounds = oriented_bounds(cluster, rotation);
    const Eigen::Vector3f pca_spans = pca_bounds.maximum - pca_bounds.minimum;
    Eigen::Index long_axis_index = 0;
    const float long_axis_span = pca_spans.maxCoeff(&long_axis_index);
    float secondary_axis_span = 0.0F;
    for (Eigen::Index axis = 0; axis < pca_spans.size(); ++axis) {
      if (axis != long_axis_index) {
        secondary_axis_span = std::max(secondary_axis_span, pca_spans[axis]);
      }
    }
    if (long_axis_span < minimum_long_axis_span_m_ ||
      secondary_axis_span < minimum_secondary_axis_span_m_)
    {
      continue;
    }
    rotation = canonicalize_long_axis(rotation, long_axis_index);
    const OrientedBounds bounds = oriented_bounds(cluster, rotation);
    if (enforce_max_dimensions_ && !observed_spans_fit_block(bounds, dimension_tolerance_m_)) {
      continue;
    }
    const Eigen::Vector3f sensor_origin(
      static_cast<float>(transform.transform.translation.x),
      static_cast<float>(transform.transform.translation.y),
      static_cast<float>(transform.transform.translation.z));
    const Eigen::Vector3f center = estimate_cuboid_center(
      bounds, rotation, sensor_origin, dimension_tolerance_m_);
    const Eigen::Quaternionf orientation(rotation);

    geometry_msgs::msg::Pose pose;
    pose.position.x = center.x();
    pose.position.y = center.y();
    pose.position.z = center.z();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    poses.poses.push_back(pose);
    markers.markers.push_back(make_marker(poses.header, pose, marker_id++));
  }

  poses_pub_->publish(poses);
  markers_pub_->publish(markers);
  update_world_model(poses.poses);
}

void ConcreteBlockDetectorNode::refresh_world_model_cache()
{
  if (!world_model_enabled_ || world_model_refresh_in_flight_ ||
    !get_coarse_blocks_client_->service_is_ready())
  {
    return;
  }

  world_model_refresh_in_flight_ = true;
  auto request = std::make_shared<concrete_block_world_model_interfaces::srv::GetCoarseBlocks::Request>();
  request->force_refresh = false;
  request->timeout_s = 0.0F;
  request->query_stamp = now();
  get_coarse_blocks_client_->async_send_request(
    request,
    [this](rclcpp::Client<concrete_block_world_model_interfaces::srv::GetCoarseBlocks>::SharedFuture future) {
      world_model_refresh_in_flight_ = false;
      const auto response = future.get();
      if (!response->success) {
        RCLCPP_WARN(get_logger(), "World-model cache refresh failed: %s", response->message.c_str());
        return;
      }

      std::lock_guard<std::mutex> lock(tracked_blocks_mutex_);
      for (const auto & block : response->blocks.blocks) {
        if (block.id.empty()) {
          continue;
        }
        const auto existing = std::find_if(
          tracked_blocks_.begin(), tracked_blocks_.end(),
          [&block](const TrackedBlock & tracked) { return tracked.id == block.id; });
        if (existing == tracked_blocks_.end()) {
          tracked_blocks_.push_back({block.id, block.pose, block.task_status});
        } else {
          existing->pose = block.pose;
          existing->task_status = block.task_status;
        }
      }
    });
  last_world_model_refresh_ = now();
}

void ConcreteBlockDetectorNode::update_world_model(const std::vector<geometry_msgs::msg::Pose> & poses)
{
  if (!world_model_enabled_) {
    return;
  }
  if (!upsert_block_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "World-model upsert service '%s' is unavailable; poses remain available on ~/poses",
      upsert_block_service_.c_str());
    return;
  }

  if ((now() - last_world_model_refresh_).seconds() >= world_model_cache_refresh_s_) {
    refresh_world_model_cache();
  }

  const double max_distance_sq =
    world_model_association_max_distance_m_ * world_model_association_max_distance_m_;
  std::vector<std::string> assigned_ids;
  for (const auto & pose : poses) {
    std::string id;
    bool overlaps_non_free_block = false;
    {
      std::lock_guard<std::mutex> lock(tracked_blocks_mutex_);
      for (const auto & candidate : tracked_blocks_) {
        if (candidate.task_status != concrete_block_world_model_interfaces::msg::Block::TASK_FREE &&
          candidate.task_status != concrete_block_world_model_interfaces::msg::Block::TASK_UNKNOWN &&
          squared_distance(candidate.pose, pose) <= max_distance_sq)
        {
          overlaps_non_free_block = true;
          break;
        }
      }
      if (overlaps_non_free_block) {
        continue;
      }
      auto best = tracked_blocks_.end();
      double best_distance_sq = max_distance_sq;
      for (auto candidate = tracked_blocks_.begin(); candidate != tracked_blocks_.end(); ++candidate) {
        if (candidate->task_status != concrete_block_world_model_interfaces::msg::Block::TASK_FREE &&
          candidate->task_status != concrete_block_world_model_interfaces::msg::Block::TASK_UNKNOWN)
        {
          continue;
        }
        if (std::find(assigned_ids.begin(), assigned_ids.end(), candidate->id) != assigned_ids.end()) {
          continue;
        }
        const double distance_sq = squared_distance(candidate->pose, pose);
        if (distance_sq <= best_distance_sq) {
          best = candidate;
          best_distance_sq = distance_sq;
        }
      }
      if (best == tracked_blocks_.end()) {
        id = world_model_id_prefix_ + std::to_string(next_block_id_++);
        tracked_blocks_.push_back({id, pose, concrete_block_world_model_interfaces::msg::Block::TASK_FREE});
      } else {
        id = best->id;
        best->pose = pose;
      }
    }
    assigned_ids.push_back(id);

    auto request = std::make_shared<concrete_block_world_model_interfaces::srv::UpsertBlock::Request>();
    request->block_id = id;
    request->pose = pose;
    request->frame_id = world_model_frame_;
    request->pose_status = concrete_block_world_model_interfaces::msg::Block::POSE_COARSE;
    request->task_status = concrete_block_world_model_interfaces::msg::Block::TASK_FREE;
    request->confidence = static_cast<float>(world_model_confidence_);
    upsert_block_client_->async_send_request(
      request,
      [this, id](rclcpp::Client<concrete_block_world_model_interfaces::srv::UpsertBlock>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(get_logger(), "World-model upsert for '%s' failed: %s", id.c_str(), response->message.c_str());
        }
      });
  }
}

}  // namespace concrete_block_detector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<concrete_block_detector::ConcreteBlockDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

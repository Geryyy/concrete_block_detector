#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace concrete_block_detector
{

// A deliberately small, gravity-constrained fallback for partial cuboid views.
// The horizontal axes are estimated from the top slab only: returns from a
// visible side must not rotate the block frame as whole-cluster PCA does.
struct TopSlabPose
{
  Eigen::Vector3f center{Eigen::Vector3f::Zero()};
  Eigen::Matrix3f rotation{Eigen::Matrix3f::Identity()};
  std::size_t top_point_count{0};
};

inline bool synthesize_pose_from_top_slab(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cluster,
  const Eigen::Vector3f & ground_normal,
  float block_height_m,
  float slab_thickness_m,
  float top_plane_distance_threshold_m,
  std::size_t minimum_top_points,
  TopSlabPose * result)
{
  if (!cluster || !result || cluster->empty() || block_height_m <= 0.0F ||
    slab_thickness_m <= 0.0F || top_plane_distance_threshold_m <= 0.0F)
  {
    return false;
  }

  Eigen::Vector3f z_axis = ground_normal;
  const float normal_length = z_axis.norm();
  if (!std::isfinite(normal_length) || normal_length < 1.0e-6F) {
    return false;
  }
  z_axis /= normal_length;
  if (z_axis.z() < 0.0F) {
    z_axis = -z_axis;
  }

  float maximum_height = -std::numeric_limits<float>::infinity();
  for (const auto & point : cluster->points) {
    const Eigen::Vector3f position(point.x, point.y, point.z);
    maximum_height = std::max(maximum_height, z_axis.dot(position));
  }
  if (!std::isfinite(maximum_height)) {
    return false;
  }

  auto top_slab = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  top_slab->reserve(cluster->size());
  for (const auto & point : cluster->points) {
    const Eigen::Vector3f position(point.x, point.y, point.z);
    if (z_axis.dot(position) >= maximum_height - slab_thickness_m) {
      top_slab->push_back(point);
    }
  }
  if (top_slab->size() < minimum_top_points) {
    return false;
  }

  // A height band includes the upper strip of a side face. Retain only its
  // dominant near-horizontal plane before calculating the top centroid/yaw.
  pcl::SACSegmentation<pcl::PointXYZ> top_segmenter;
  top_segmenter.setOptimizeCoefficients(true);
  top_segmenter.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  top_segmenter.setMethodType(pcl::SAC_RANSAC);
  top_segmenter.setDistanceThreshold(top_plane_distance_threshold_m);
  top_segmenter.setMaxIterations(100);
  top_segmenter.setAxis(z_axis);
  top_segmenter.setEpsAngle(0.174532925F);  // 10 degrees
  top_segmenter.setInputCloud(top_slab);
  auto top_inliers = std::make_shared<pcl::PointIndices>();
  pcl::ModelCoefficients top_coefficients;
  top_segmenter.segment(*top_inliers, top_coefficients);
  if (top_inliers->indices.size() < minimum_top_points) {
    return false;
  }

  Eigen::Vector3f top_centroid = Eigen::Vector3f::Zero();
  for (const int index : top_inliers->indices) {
    const auto & point = top_slab->points.at(static_cast<std::size_t>(index));
    top_centroid += Eigen::Vector3f(point.x, point.y, point.z);
  }
  const std::size_t top_point_count = top_inliers->indices.size();
  top_centroid /= static_cast<float>(top_point_count);

  // PCA of the top-slab footprint, projected onto the fitted ground plane.
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  for (const int index : top_inliers->indices) {
    const auto & point = top_slab->points.at(static_cast<std::size_t>(index));
    const Eigen::Vector3f position(point.x, point.y, point.z);
    Eigen::Vector3f offset = position - top_centroid;
    offset -= z_axis * z_axis.dot(offset);
    covariance.noalias() += offset * offset.transpose();
  }
  covariance /= static_cast<float>(top_point_count);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return false;
  }
  Eigen::Vector3f x_axis = solver.eigenvectors().col(2);
  x_axis -= z_axis * z_axis.dot(x_axis);
  const float x_length = x_axis.norm();
  if (!std::isfinite(x_length) || x_length < 1.0e-6F) {
    return false;
  }
  x_axis /= x_length;
  Eigen::Vector3f y_axis = z_axis.cross(x_axis);
  const float y_length = y_axis.norm();
  if (!std::isfinite(y_length) || y_length < 1.0e-6F) {
    return false;
  }
  y_axis /= y_length;

  result->center = top_centroid - z_axis * (block_height_m / 2.0F);
  result->rotation.col(0) = x_axis;
  result->rotation.col(1) = y_axis;
  result->rotation.col(2) = z_axis;
  result->top_point_count = top_point_count;
  return true;
}

}  // namespace concrete_block_detector

#include "concrete_block_detector/pose_synthesis.hpp"

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <string>

namespace concrete_block_detector
{
namespace
{

pcl::PointCloud<pcl::PointXYZ>::Ptr make_top_and_side_cluster(const Eigen::Matrix3f & rotation)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  const Eigen::Vector3f center = rotation * Eigen::Vector3f(0.0F, 0.0F, 0.30F);
  const auto add_point = [&cloud, &rotation, &center](float x, float y, float z) {
      const Eigen::Vector3f point = center + rotation * Eigen::Vector3f(x, y, z - 0.30F);
      cloud->push_back({point.x(), point.y(), point.z()});
    };

  // A complete top face plus a dense visible long side: this is the geometry
  // that previously tilted whole-cluster PCA by roughly 45 degrees.
  for (int x = 0; x <= 18; ++x) {
    for (int y = 0; y <= 12; ++y) {
      add_point(-0.45F + 0.05F * x, -0.30F + 0.05F * y, 0.60F);
    }
  }
  for (int y = 0; y <= 12; ++y) {
    for (int z = 0; z < 12; ++z) {
      add_point(0.45F, -0.30F + 0.05F * y, 0.05F * z);
    }
  }
  return cloud;
}

TEST(TopSlabPoseSynthesis, IgnoresVisibleSideWhenEstimatingYaw)
{
  const auto cluster = make_top_and_side_cluster(Eigen::Matrix3f::Identity());
  TopSlabPose pose;

  ASSERT_TRUE(synthesize_pose_from_top_slab(
      cluster, Eigen::Vector3f::UnitZ(), 0.60F, 0.08F, 0.02F, 12U, &pose));
  EXPECT_NEAR(pose.center.x(), 0.0F, 1.0e-5F);
  EXPECT_NEAR(pose.center.y(), 0.0F, 1.0e-5F);
  EXPECT_NEAR(pose.center.z(), 0.30F, 1.0e-5F);
  EXPECT_NEAR(std::abs(pose.rotation.col(0).dot(Eigen::Vector3f::UnitX())), 1.0F, 1.0e-5F);
  EXPECT_NEAR(pose.rotation.col(2).dot(Eigen::Vector3f::UnitZ()), 1.0F, 1.0e-5F);
  EXPECT_EQ(pose.top_point_count, 247U);
}

TEST(TopSlabPoseSynthesis, UsesTiltedGroundNormalAsLocalZ)
{
  const Eigen::Matrix3f rotation =
    (Eigen::AngleAxisf(0.12F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.05F, Eigen::Vector3f::UnitX())).toRotationMatrix();
  const auto cluster = make_top_and_side_cluster(rotation);
  TopSlabPose pose;

  ASSERT_TRUE(synthesize_pose_from_top_slab(
      cluster, rotation.col(2), 0.60F, 0.08F, 0.02F, 12U, &pose));
  EXPECT_NEAR((pose.center - rotation * Eigen::Vector3f(0.0F, 0.0F, 0.30F)).norm(), 0.0F, 1.0e-5F);
  EXPECT_NEAR(std::abs(pose.rotation.col(0).dot(rotation.col(0))), 1.0F, 1.0e-5F);
  EXPECT_NEAR(pose.rotation.col(2).dot(rotation.col(2)), 1.0F, 1.0e-5F);
}

}  // namespace
}  // namespace concrete_block_detector

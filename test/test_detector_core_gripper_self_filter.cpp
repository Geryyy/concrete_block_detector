#include "concrete_block_detector/gripper_self_filter.hpp"

#include <gtest/gtest.h>

namespace concrete_block_detector::detector_core
{
namespace
{

TEST(GripperSelfFilter, RemovesPointsInsideCylinderAndRoundedEndCaps)
{
  const CylinderSegment rail{Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), 0.10};
  const Points input{
    Point(0.5, 0.05, 0.0),  // inside rail body
    Point(-0.05, 0.0, 0.0),  // inside start cap
    Point(1.05, 0.0, 0.0),  // inside end cap
    Point(0.5, 0.101, 0.0),  // just outside body
    Point(-0.101, 0.0, 0.0)};  // just outside start cap
  std::size_t removed = 0U;
  const auto retained = remove_points_inside_cylinders(input, {rail}, &removed);
  EXPECT_EQ(removed, 3U);
  ASSERT_EQ(retained.size(), 2U);
  EXPECT_NEAR(retained[0].y(), 0.101, 1e-12);
  EXPECT_NEAR(retained[1].x(), -0.101, 1e-12);
}

TEST(GripperSelfFilter, RejectsInvalidCylinder)
{
  EXPECT_THROW(
    remove_points_inside_cylinders(Points{}, {CylinderSegment{Point::Zero(), Point::Zero(), 0.1}}),
    std::invalid_argument);
  EXPECT_THROW(
    remove_points_inside_cylinders(Points{}, {CylinderSegment{Point::Zero(), Point::UnitX(), 0.0}}),
    std::invalid_argument);
}

}  // namespace
}  // namespace concrete_block_detector::detector_core

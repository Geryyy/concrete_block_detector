#pragma once

// Small, deterministic point self-filter.  It deliberately stays in the
// detector core rather than converting a cloud to and from PCL just to test
// distance to the two gripper rails.
#include "concrete_block_detector/detector_core_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace concrete_block_detector::detector_core
{

struct CylinderSegment
{
  Point start{Point::Zero()};
  Point end{Point::Zero()};
  double radius{0.0};
};

inline double squared_distance_to_segment(const Point & point, const CylinderSegment & segment)
{
  const Point axis = segment.end - segment.start;
  const double axis_squared = axis.squaredNorm();
  if (!std::isfinite(axis_squared) || axis_squared <= 1e-18) {
    throw std::invalid_argument("cylinder segment must have a non-zero length");
  }
  const double fraction = std::clamp((point - segment.start).dot(axis) / axis_squared, 0.0, 1.0);
  return (point - (segment.start + fraction * axis)).squaredNorm();
}

inline Points remove_points_inside_cylinders(
  const Points & input, const std::vector<CylinderSegment> & cylinders,
  std::size_t * removed_count = nullptr)
{
  for (const auto & cylinder : cylinders) {
    if (!std::isfinite(cylinder.radius) || cylinder.radius <= 0.0) {
      throw std::invalid_argument("cylinder radius must be finite and positive");
    }
    (void)squared_distance_to_segment(cylinder.start, cylinder);
  }

  Points output;
  output.reserve(input.size());
  std::size_t removed = 0U;
  for (const auto & point : input) {
    const bool inside = std::any_of(
      cylinders.begin(), cylinders.end(), [&point](const auto & cylinder) {
        return squared_distance_to_segment(point, cylinder) <= cylinder.radius * cylinder.radius;
      });
    if (inside) {
      ++removed;
    } else {
      output.push_back(point);
    }
  }
  if (removed_count != nullptr) {
    *removed_count = removed;
  }
  return output;
}

}  // namespace concrete_block_detector::detector_core

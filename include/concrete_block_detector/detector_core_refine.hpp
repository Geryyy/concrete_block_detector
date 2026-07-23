#pragma once

// Pure Eigen port of blockpose.refine; no ROS or PCL types.
#include "concrete_block_detector/detector_core_geometry.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace concrete_block_detector::detector_core
{
struct CuboidPose
{
  Point position{Point::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};  // R_world_block
  std::array<double, 3> dims{{0.9, 0.6, 0.6}};
  double confidence{0.0};
  std::string source;
};
struct RefineParameters
{
  double huber_scale{0.05};
  double max_translation{0.15};
  double max_rotation_deg{20.0};
  std::size_t min_points{20};
  std::size_t max_evaluations{200};
};
inline Eigen::Matrix3d skew(const Point & v)
{
  Eigen::Matrix3d r;
  r << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return r;
}
inline Eigen::Matrix3d exp_so3(const Point & v)
{
  const double angle = v.norm();
  if (angle < 1.0e-12) {return Eigen::Matrix3d::Identity();}
  const Eigen::Matrix3d axis = skew(v / angle);
  return Eigen::Matrix3d::Identity() + std::sin(angle) * axis + (1.0 - std::cos(angle)) * axis * axis;
}
inline double box_sdf(const Point & local, const Point & half)
{
  const Point delta = local.cwiseAbs() - half;
  return delta.cwiseMax(Point::Zero()).norm() + std::min(delta.maxCoeff(), 0.0);
}
inline std::vector<double> surface_distance(const CuboidPose & pose, const Points & points)
{
  const Point half(pose.dims[0] / 2.0, pose.dims[1] / 2.0, pose.dims[2] / 2.0);
  std::vector<double> result; result.reserve(points.size());
  for (const auto & point : points) {result.push_back(box_sdf(pose.rotation.transpose() * (point - pose.position), half));}
  return result;
}
inline std::vector<Points> assign_points(const std::vector<CuboidPose> & poses, const Points & points, double band)
{
  std::vector<Points> result(poses.size());
  for (const auto & point : points) {
    std::size_t owner = 0; double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < poses.size(); ++i) {
      const Point half(poses[i].dims[0] / 2.0, poses[i].dims[1] / 2.0, poses[i].dims[2] / 2.0);
      const double distance = std::abs(box_sdf(poses[i].rotation.transpose() * (point - poses[i].position), half));
      if (distance < nearest) {nearest = distance; owner = i;}
    }
    if (!poses.empty() && nearest <= band) {result[owner].push_back(point);}
  }
  return result;
}
inline double huber_cost(double r, double scale)
{
  const double a = std::abs(r); return a <= scale ? 0.5 * r * r : scale * (a - 0.5 * scale);
}
inline double rotation_angle_deg(const Eigen::Matrix3d & r)
{
  return std::acos(std::clamp((r.trace() - 1.0) / 2.0, -1.0, 1.0)) * 57.2957795130823208768;
}
// Facts captured before the correction safety guard.  Keeping them with the
// result makes Python/C++ optimizer parity observable without changing the
// published pose or the refinement decision.
struct RefineDiagnostics
{
  bool attempted{false};
  std::size_t point_count{0U};
  std::size_t evaluations{0U};
  std::size_t iterations{0U};
  double initial_cost{std::numeric_limits<double>::quiet_NaN()};
  double final_cost{std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 6> correction{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double translation_norm{0.0};
  double rotation_deg{0.0};
  bool guard_accepted{false};
};
struct RefineResult {CuboidPose pose; bool refined{false}; RefineDiagnostics diagnostics;};

inline RefineResult refine_pose(const CuboidPose & initial, const Points & points, const RefineParameters & params = {})
{
  RefineDiagnostics diagnostics;
  diagnostics.point_count = points.size();
  if (points.size() < params.min_points || params.huber_scale <= 0.0 || params.max_evaluations < 7U) {return {initial, false, diagnostics};}
  diagnostics.attempted = true;
  const Point half(initial.dims[0] / 2.0, initial.dims[1] / 2.0, initial.dims[2] / 2.0);
  const auto residuals = [&initial, &points, &half](const Eigen::Matrix<double, 6, 1> & x) {
      const Eigen::Matrix3d r = initial.rotation * exp_so3(x.head<3>());
      std::vector<double> output; output.reserve(points.size());
      for (const auto & point : points) {output.push_back(box_sdf(r.transpose() * (point - (initial.position + x.tail<3>())), half));}
      return output;
    };
  const auto cost_for = [&params](const std::vector<double> & values) {double total = 0.0; for (const double value : values) {total += huber_cost(value, params.huber_scale);} return total;};
  Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Zero();
  std::vector<double> current = residuals(x); std::size_t evaluations = 1U; double cost = cost_for(current);
  diagnostics.initial_cost = cost;
  constexpr double h = 1.0e-5;
  for (std::size_t iteration = 0; evaluations + 12U <= params.max_evaluations && iteration < 30U; ++iteration) {
    diagnostics.iterations = iteration + 1U;
    Eigen::MatrixXd jacobian(points.size(), 6);
    for (int column = 0; column < 6; ++column) {
      auto plus = x; auto minus = x; plus[column] += h; minus[column] -= h;
      const auto forward = residuals(plus); const auto backward = residuals(minus); evaluations += 2U;
      for (std::size_t row = 0; row < points.size(); ++row) {jacobian(static_cast<Eigen::Index>(row), column) = (forward[row] - backward[row]) / (2.0 * h);}
    }
    Eigen::VectorXd weighted(points.size());
    for (std::size_t row = 0; row < points.size(); ++row) {
      const double weight = std::abs(current[row]) <= params.huber_scale ? 1.0 : params.huber_scale / std::abs(current[row]);
      jacobian.row(static_cast<Eigen::Index>(row)) *= std::sqrt(weight); weighted[static_cast<Eigen::Index>(row)] = std::sqrt(weight) * current[row];
    }
    const Eigen::Matrix<double, 6, 6> normal = jacobian.transpose() * jacobian + 1.0e-8 * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 1> delta = normal.ldlt().solve(-jacobian.transpose() * weighted);
    if (!delta.allFinite() || delta.norm() < 1.0e-8) {break;}
    bool accepted = false;
    for (double step = 1.0; step >= 1.0 / 64.0 && evaluations < params.max_evaluations; step *= 0.5) {
      const auto proposal = x + step * delta; const auto candidate = residuals(proposal); ++evaluations;
      const double candidate_cost = cost_for(candidate);
      if (candidate_cost < cost) {x = proposal; current = candidate; cost = candidate_cost; accepted = true; break;}
    }
    if (!accepted) {break;}
  }
  const Eigen::Matrix3d delta_rotation = exp_so3(x.head<3>());
  diagnostics.evaluations = evaluations;
  diagnostics.final_cost = cost;
  for (int index = 0; index < 6; ++index) {diagnostics.correction[static_cast<std::size_t>(index)] = x[index];}
  diagnostics.translation_norm = x.tail<3>().norm();
  diagnostics.rotation_deg = rotation_angle_deg(delta_rotation);
  if (diagnostics.translation_norm > params.max_translation || diagnostics.rotation_deg > params.max_rotation_deg) {return {initial, false, diagnostics};}
  CuboidPose refined = initial; refined.position += x.tail<3>(); refined.rotation *= delta_rotation; refined.source += "+sdf";
  diagnostics.guard_accepted = true;
  return {refined, true, diagnostics};
}
inline std::vector<CuboidPose> refine_poses(std::vector<CuboidPose> poses, const Points & points, double band = 0.10, std::size_t iterations = 2U, const RefineParameters & params = {})
{
  for (std::size_t iteration = 0; iteration < std::max<std::size_t>(iterations, 1U); ++iteration) {const auto assigned = assign_points(poses, points, band); for (std::size_t i = 0; i < poses.size(); ++i) {poses[i] = refine_pose(poses[i], assigned[i], params).pose;}}
  return poses;
}
}  // namespace concrete_block_detector::detector_core

// Offline diagnostic for the Python -> C++ boundary immediately before SDF
// refinement.  Unlike the full parity runner this deliberately calls
// detect_without_refinement(): any reported mismatch is proposal, plane-fit,
// pose-synthesis, scoring, or pre-refinement selection -- never the solver.
#include "concrete_block_detector/detector_core_pipeline.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace cb = concrete_block_detector::detector_core;
namespace
{
cb::Points load_xyz(const std::filesystem::path & path)
{
  cb::Points points;
  std::ifstream stream(path);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  while (stream >> x >> y >> z) {
    points.emplace_back(x, y, z);
  }
  return points;
}

cb::Pose pose_from_json(const nlohmann::json & value)
{
  cb::Pose pose;
  for (int axis = 0; axis < 3; ++axis) {
    pose.position[axis] = value.at("position").at(axis).get<double>();
    pose.dims[axis] = value.at("dims").at(axis).get<double>();
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      pose.rotation(row, column) = value.at("rotation").at(row).at(column).get<double>();
    }
  }
  return pose;
}

double symmetry_aware_angle_deg(const Eigen::Matrix3d & actual, const Eigen::Matrix3d & expected)
{
  const std::array<Eigen::Matrix3d, 4> symmetries{{
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(-1.0, -1.0, 1.0).asDiagonal(),
      Eigen::Vector3d(-1.0, 1.0, -1.0).asDiagonal(),
      Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal(),
    }};
  double error = std::numeric_limits<double>::infinity();
  for (const auto & symmetry : symmetries) {
    const Eigen::Matrix3d delta = (actual * symmetry).transpose() * expected;
    error = std::min(error, std::acos(std::clamp((delta.trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI);
  }
  return error;
}

std::vector<std::size_t> minimum_position_assignment(
  const std::vector<cb::Pose> & actual, const std::vector<cb::Pose> & expected)
{
  std::vector<std::size_t> permutation(actual.size());
  std::iota(permutation.begin(), permutation.end(), 0U);
  std::vector<std::size_t> best = permutation;
  double best_cost = std::numeric_limits<double>::infinity();
  do {
    double cost = 0.0;
    for (std::size_t index = 0; index < actual.size(); ++index) {
      cost += (actual[index].position - expected[permutation[index]].position).squaredNorm();
    }
    if (cost < best_cost) {
      best_cost = cost;
      best = permutation;
    }
  } while (std::next_permutation(permutation.begin(), permutation.end()));
  return best;
}

int run_case(const std::filesystem::path & fixture_dir, const std::string & id)
{
  std::ifstream stream(fixture_dir / (id + ".refinement.json"));
  if (!stream.is_open()) {
    std::cerr << "missing fixture: " << id << ".refinement.json\n";
    return 2;
  }
  nlohmann::json fixture;
  stream >> fixture;
  std::vector<cb::Pose> expected;
  for (const auto & initial : fixture.at("initial")) {
    expected.push_back(pose_from_json(initial));
  }
  const cb::Points points = load_xyz(fixture_dir / (id + ".xyz"));
  if (points.empty()) {
    std::cerr << "missing or empty fixture: " << id << ".xyz\n";
    return 2;
  }
  const auto actual_result = cb::detect_without_refinement(points);
  const auto & actual = actual_result.poses;
  std::cout << std::fixed << std::setprecision(3);
  std::cout << id << " pre_refinement python=" << expected.size() << " cpp=" << actual.size()
            << " raw_hypotheses=" << actual_result.counts.raw_hypotheses
            << " refinement_candidates=" << actual_result.counts.refinement_candidates << "\n";
  if (actual.size() != expected.size()) {
    return 1;
  }
  const auto assignment = minimum_position_assignment(actual, expected);
  bool matches = true;
  for (std::size_t index = 0; index < actual.size(); ++index) {
    const auto & cpp_pose = actual[index];
    const auto & python_pose = expected[assignment[index]];
    const double centre_error_m = (cpp_pose.position - python_pose.position).norm();
    const double rotation_error_deg = symmetry_aware_angle_deg(cpp_pose.rotation, python_pose.rotation);
    const bool dims_equal = cpp_pose.dims == python_pose.dims;
    matches = matches && centre_error_m <= 0.05 && rotation_error_deg <= 5.0 && dims_equal;
    std::cout << "  cpp[" << index << "] -> python[" << assignment[index] << "]"
              << " centre_error_m=" << centre_error_m
              << " rotation_error_deg=" << rotation_error_deg
              << " dims_equal=" << (dims_equal ? "true" : "false") << "\n";
  }
  return matches ? 0 : 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 3) {
    std::cerr << "usage: concrete_block_detector_proposal_parity_runner <fixture-dir> <case-id>\n";
    return 2;
  }
  return run_case(argv[1], argv[2]);
}

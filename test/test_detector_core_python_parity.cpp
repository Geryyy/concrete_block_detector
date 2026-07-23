#include "concrete_block_detector/detector_core_pipeline.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <map>
#include <limits>
#include <numeric>
#include <string>

#ifndef BLOCKPOSE_SOURCE_ROOT
#error "BLOCKPOSE_SOURCE_ROOT must be set by CMake"
#endif

namespace concrete_block_detector::detector_core
{
namespace
{
struct Expected
{
  std::size_t poses{0};
  std::vector<Pose> values;
  std::vector<double> scores;
  std::map<std::string, std::size_t> counts;
};

Points load_points(const std::filesystem::path & path)
{
  Points result;
  std::ifstream stream(path);
  double x = 0.0, y = 0.0, z = 0.0;
  while (stream >> x >> y >> z) {result.emplace_back(x, y, z);}
  return result;
}

Expected load_expected(const std::filesystem::path & path)
{
  Expected result;
  std::ifstream stream(path);
  std::string tag;
  while (stream >> tag) {
    if (tag == "poses") {stream >> result.poses; continue;}
    if (tag == "count") {std::string name; std::size_t value = 0; stream >> name >> value; result.counts[name] = value; continue;}
    if (tag == "pose") {
      Pose pose;
      for (int index = 0; index < 3; ++index) {stream >> pose.position[index];}
      for (int row = 0; row < 3; ++row) {for (int column = 0; column < 3; ++column) {stream >> pose.rotation(row, column);}}
      for (int index = 0; index < 3; ++index) {stream >> pose.dims[index];}
      double score = 0.0; stream >> score; result.values.push_back(pose); result.scores.push_back(score);
    }
  }
  return result;
}

std::vector<std::size_t> optimal_assignment(const std::vector<Pose> & actual, const std::vector<Pose> & expected)
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
    if (cost < best_cost) {best_cost = cost; best = permutation;}
  } while (std::next_permutation(permutation.begin(), permutation.end()));
  return best;
}

void compare_case(const std::filesystem::path & fixture_dir, const std::string & id)
{
  const Expected expected = load_expected(fixture_dir / (id + ".expected"));
  const Points points = load_points(fixture_dir / (id + ".xyz"));
  const auto actual = detect(points);
  EXPECT_EQ(actual.poses.size(), expected.poses) << id;
  ASSERT_EQ(actual.poses.size(), expected.values.size()) << id;
  const auto assignment = optimal_assignment(actual.poses, expected.values);
  for (std::size_t index = 0; index < actual.poses.size(); ++index) {
    const std::size_t expected_index = assignment[index];
    EXPECT_LE((actual.poses[index].position - expected.values[expected_index].position).norm(), 0.05) << id;
    EXPECT_EQ(actual.poses[index].dims, expected.values[expected_index].dims) << id;
    const std::array<Eigen::Matrix3d, 4> symmetries{{
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(-1.0, -1.0, 1.0).asDiagonal(),
        Eigen::Vector3d(-1.0, 1.0, -1.0).asDiagonal(),
        Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal(),
      }};
    double rotation_error = std::numeric_limits<double>::infinity();
    for (const Eigen::Matrix3d & symmetry : symmetries) {
      const Eigen::Matrix3d delta = (actual.poses[index].rotation * symmetry).transpose() * expected.values[expected_index].rotation;
      rotation_error = std::min(rotation_error, std::acos(std::clamp((delta.trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI);
    }
    EXPECT_LE(rotation_error, 5.0) << id;
    EXPECT_NEAR(actual.hypotheses[index].evidence.score, expected.scores[expected_index], 0.02) << id;
  }
  const std::map<std::string, std::size_t> actual_counts{{"input_points", actual.counts.input_points}, {"downsampled_points", actual.counts.downsampled_points}, {"above_support_points", actual.counts.above_support_points}, {"proposal_components", actual.counts.proposal_components}, {"plane_regions", actual.counts.plane_regions}, {"plane_fit_calls", actual.counts.plane_fit_calls}, {"plane_search_points", actual.counts.plane_search_points}, {"plane_full_points_scored", actual.counts.plane_full_points_scored}, {"plane_trials_evaluated", actual.counts.plane_trials_evaluated}, {"plane_valid_trials", actual.counts.plane_valid_trials}, {"raw_hypotheses", actual.counts.raw_hypotheses}, {"refinement_candidates", actual.counts.refinement_candidates}, {"selected_hypotheses", actual.counts.selected_hypotheses}};
  for (const auto & [name, expected_value] : expected.counts) {const auto found = actual_counts.find(name); if (found != actual_counts.end()) {EXPECT_EQ(found->second, expected_value) << id << " " << name;}}
  auto shuffled = points; std::reverse(shuffled.begin(), shuffled.end()); const auto repeat = detect(points), shuffled_result = detect(shuffled); ASSERT_EQ(repeat.poses.size(), actual.poses.size()); ASSERT_EQ(shuffled_result.poses.size(), actual.poses.size()); for (std::size_t index = 0; index < actual.poses.size(); ++index) {EXPECT_EQ(repeat.poses[index].position, actual.poses[index].position); EXPECT_EQ(shuffled_result.poses[index].position, actual.poses[index].position);}
}

TEST(PythonParity, RuntimeMaterializedGoldenInputsAndExpectedOutputs)
{
  const std::filesystem::path blockpose(BLOCKPOSE_SOURCE_ROOT);
  const std::filesystem::path fixture_dir = std::filesystem::temp_directory_path() / "concrete_block_detector_python_parity";
  std::filesystem::remove_all(fixture_dir);
  const std::string command =
    "PYTHONPATH='" + (blockpose / "python").string() + "' python3 '" +
    (blockpose / "tools/export_cpp_parity_fixture.py").string() + "' --manifest '" +
    (blockpose.parent_path() / "concrete_block_detector/test/parity/fixtures_manifest.json").string() +
    "' --output '" + fixture_dir.string() + "' --include-real --plain-text";
  ASSERT_EQ(std::system(command.c_str()), 0);
  for (const std::string id : {"isolated", "top_and_side", "top_only", "tilted_support", "touching_pair", "scatter_and_clutter", "real_1783428141_224458752_seq3"}) {
    ASSERT_TRUE(std::filesystem::exists(fixture_dir / (id + ".xyz"))) << id;
    ASSERT_TRUE(std::filesystem::exists(fixture_dir / (id + ".expected"))) << id;
    compare_case(fixture_dir, id);
  }
}
}  // namespace
}  // namespace concrete_block_detector::detector_core

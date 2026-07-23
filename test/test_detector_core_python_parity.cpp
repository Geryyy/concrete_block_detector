#include "concrete_block_detector/detector_core_pipeline.hpp"
#include "concrete_block_detector/detector_core_refine.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <algorithm>
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
  auto actual = detect_without_refinement(load_points(fixture_dir / (id + ".xyz")));
  const auto initial_poses = actual.poses;
  std::vector<CuboidPose> initial;
  for (const auto & hypothesis : actual.hypotheses) {
    initial.push_back({hypothesis.pose.position, hypothesis.pose.rotation, hypothesis.pose.dims, hypothesis.pose.confidence, "plane_fit"});
  }
  const auto refined = refine_poses(std::move(initial), actual.above_support_points);
  for (std::size_t index = 0; index < actual.hypotheses.size(); ++index) {
    actual.poses[index].position = refined[index].position;
    actual.poses[index].rotation = refined[index].rotation;
  }
  EXPECT_EQ(actual.poses.size(), expected.poses) << id;
  ASSERT_EQ(actual.poses.size(), expected.values.size()) << id;
  const auto assignment = optimal_assignment(actual.poses, expected.values);
  for (std::size_t index = 0; index < actual.poses.size(); ++index) {
    const std::size_t expected_index = assignment[index];
    EXPECT_LE((actual.poses[index].position - expected.values[expected_index].position).norm(), 0.05) << id << " initial=" << initial_poses[index].position.transpose() << " actual=" << actual.poses[index].position.transpose() << " expected=" << expected.values[expected_index].position.transpose() << " faces=" << actual.hypotheses[index].evidence.observed_geometry_faces << " support=" << actual.hypotheses[index].evidence.support_points;
    EXPECT_NEAR(actual.hypotheses[index].evidence.score, expected.scores[expected_index], 0.02) << id;
  }
  const auto expected_selected = expected.counts.find("selected_hypotheses");
  ASSERT_NE(expected_selected, expected.counts.end()) << id;
  EXPECT_EQ(actual.counts.selected_hypotheses, expected_selected->second) << id;
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

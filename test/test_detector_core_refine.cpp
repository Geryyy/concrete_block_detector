#include "concrete_block_detector/detector_core_refine.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>

#ifndef BLOCKPOSE_SOURCE_ROOT
#error "BLOCKPOSE_SOURCE_ROOT must be set by CMake"
#endif

namespace concrete_block_detector::detector_core { namespace {
Points surface(const CuboidPose & pose) {Points p; Point h(pose.dims[0]/2.,pose.dims[1]/2.,pose.dims[2]/2.); auto add=[&](Point local){p.push_back(pose.position+pose.rotation*local);}; for(int x=-4;x<=4;++x)for(int y=-3;y<=3;++y)add(Point(h.x()*x/4.,h.y()*y/3.,h.z())); for(int y=-3;y<=3;++y)for(int z=-3;z<=3;++z)add(Point(h.x(),h.y()*y/3.,h.z()*z/3.)); return p;}
TEST(Refine, SdfAndExpMatchPrototype) {Point h(.45,.3,.3); EXPECT_NEAR(box_sdf(Point(.45,0,0),h),0,1e-12); EXPECT_NEAR(box_sdf(Point(0,0,0),h),-.3,1e-12); EXPECT_NEAR(box_sdf(Point(.55,0,0),h),.1,1e-12); EXPECT_NEAR(rotation_angle_deg(exp_so3(Point(0,0,.2))),11.4591559,1e-6);}
TEST(Refine, AssignmentUsesNearestSurfaceAndBand) {CuboidPose a,b;a.position=Point(-1,0,0);b.position=Point(1,0,0);auto owned=assign_points({a,b},{Point(-.55,0,0),Point(1.45,0,0),Point(0,4,0)},.1);EXPECT_EQ(owned[0].size(),1U);EXPECT_EQ(owned[1].size(),1U);}
TEST(Refine, RobustFitCorrectsSmallPoseError) {CuboidPose truth;truth.position=Point(.2,-.1,.3);truth.rotation=exp_so3(Point(0,0,.22));truth.source="prototype";const auto points=surface(truth);CuboidPose initial=truth;initial.position+=Point(.045,-.035,.02);initial.rotation*=exp_so3(Point(0,0,-.1));const auto result=refine_pose(initial,points);ASSERT_TRUE(result.refined);EXPECT_LT((result.pose.position-truth.position).norm(),.006);EXPECT_LT(rotation_angle_deg(truth.rotation.transpose()*result.pose.rotation),.8);EXPECT_EQ(result.pose.source,"prototype+sdf");}
TEST(Refine, FailuresKeepInitialPose) {CuboidPose pose;EXPECT_FALSE(refine_pose(pose,{Point::Zero()}).refined);const auto points=surface(pose);CuboidPose far=pose;far.position.x()=.5;const auto guarded=refine_pose(far,points);EXPECT_FALSE(guarded.refined);EXPECT_NEAR(guarded.pose.position.x(),.5,1e-12);}

CuboidPose cuboid_from_json(const nlohmann::json & value)
{
  CuboidPose pose;
  for (int index = 0; index < 3; ++index) {
    pose.position[index] = value.at("position").at(index).get<double>();
    pose.dims[index] = value.at("dims").at(index).get<double>();
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      pose.rotation(row, column) = value.at("rotation").at(row).at(column).get<double>();
    }
  }
  return pose;
}

Points load_xyz(const std::filesystem::path & path)
{
  Points points;
  std::ifstream stream(path);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  while (stream >> x >> y >> z) {
    points.emplace_back(x, y, z);
  }
  return points;
}

double symmetry_aware_angle_deg(const Eigen::Matrix3d & actual, const Eigen::Matrix3d & expected)
{
  const std::array<Eigen::Matrix3d, 4> symmetries{{
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(-1.0, -1.0, 1.0).asDiagonal(),
      Eigen::Vector3d(-1.0, 1.0, -1.0).asDiagonal(),
      Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal(),
    }};
  double result = std::numeric_limits<double>::infinity();
  for (const auto & symmetry : symmetries) {
    result = std::min(result, rotation_angle_deg((actual * symmetry).transpose() * expected));
  }
  return result;
}

// This is intentionally a separate gate from detector parity. It supplies the
// C++ optimizer with the exact Python selected initial poses and assignment
// cloud, so a failure cannot be attributed to RANSAC, NMS, or association.
TEST(Refine, PythonSolverBoundaryParity)
{
  const std::filesystem::path blockpose(BLOCKPOSE_SOURCE_ROOT);
  const std::filesystem::path fixture_dir =
    std::filesystem::temp_directory_path() / "concrete_block_detector_refine_parity";
  std::filesystem::remove_all(fixture_dir);
  const std::string command =
    "PYTHONPATH='" + (blockpose / "python").string() + "' python3 '" +
    (blockpose / "tools/export_cpp_parity_fixture.py").string() + "' --manifest '" +
    (blockpose.parent_path() / "concrete_block_detector/test/parity/fixtures_manifest.json").string() +
    "' --output '" + fixture_dir.string() + "' --include-real --plain-text";
  ASSERT_EQ(std::system(command.c_str()), 0);

  for (const std::string id : {"isolated", "top_and_side", "top_only", "tilted_support",
      "touching_pair", "scatter_and_clutter", "real_1783428141_224458752_seq3"}) {
    std::ifstream stream(fixture_dir / (id + ".refinement.json"));
    ASSERT_TRUE(stream.is_open()) << id;
    nlohmann::json fixture;
    stream >> fixture;
    std::vector<CuboidPose> current;
    for (const auto & initial : fixture.at("initial")) {
      current.push_back(cuboid_from_json(initial));
    }
    const Points points = load_xyz(fixture_dir / (id + ".refine_points.xyz"));
    ASSERT_FALSE(points.empty()) << id;
    ASSERT_EQ(current.size(), fixture.at("passes").front().at("poses").size()) << id;
    for (const auto & pass : fixture.at("passes")) {
      const auto assigned = assign_points(current, points, 0.10);
      ASSERT_EQ(assigned.size(), pass.at("assigned_counts").size()) << id;
      for (std::size_t index = 0; index < current.size(); ++index) {
        EXPECT_EQ(assigned[index].size(), pass.at("assigned_counts").at(index).get<std::size_t>())
          << id << " assigned points";
        const auto actual = refine_pose(current[index], assigned[index]);
        const CuboidPose expected = cuboid_from_json(pass.at("poses").at(index));
        EXPECT_EQ(actual.refined, pass.at("refined").at(index).get<bool>()) << id;
        EXPECT_LE((actual.pose.position - expected.position).norm(), 0.005) << id;
        EXPECT_LE(symmetry_aware_angle_deg(actual.pose.rotation, expected.rotation), 0.5) << id;
        current[index] = actual.pose;
      }
    }
  }
}
}}  // namespace

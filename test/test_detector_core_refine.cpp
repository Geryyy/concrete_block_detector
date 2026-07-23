#include "concrete_block_detector/detector_core_refine.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>

#ifndef BLOCKPOSE_SOURCE_ROOT
#error "BLOCKPOSE_SOURCE_ROOT must be set by CMake"
#endif

namespace concrete_block_detector::detector_core { namespace {
Points surface(const CuboidPose & pose) {Points p; Point h(pose.dims[0]/2.,pose.dims[1]/2.,pose.dims[2]/2.); auto add=[&](Point local){p.push_back(pose.position+pose.rotation*local);}; for(int x=-4;x<=4;++x)for(int y=-3;y<=3;++y)add(Point(h.x()*x/4.,h.y()*y/3.,h.z())); for(int y=-3;y<=3;++y)for(int z=-3;z<=3;++z)add(Point(h.x(),h.y()*y/3.,h.z()*z/3.)); return p;}
TEST(Refine, SdfAndExpMatchPrototype) {Point h(.45,.3,.3); EXPECT_NEAR(box_sdf(Point(.45,0,0),h),0,1e-12); EXPECT_NEAR(box_sdf(Point(0,0,0),h),-.3,1e-12); EXPECT_NEAR(box_sdf(Point(.55,0,0),h),.1,1e-12); EXPECT_NEAR(rotation_angle_deg(exp_so3(Point(0,0,.2))),11.4591559,1e-6);}
TEST(Refine, AssignmentUsesNearestSurfaceAndBand) {CuboidPose a,b;a.position=Point(-1,0,0);b.position=Point(1,0,0);auto owned=assign_points({a,b},{Point(-.55,0,0),Point(1.45,0,0),Point(0,4,0)},.1);EXPECT_EQ(owned[0].size(),1U);EXPECT_EQ(owned[1].size(),1U);}
TEST(Refine, RobustFitCorrectsSmallPoseError) {CuboidPose truth;truth.position=Point(.2,-.1,.3);truth.rotation=exp_so3(Point(0,0,.22));truth.source="prototype";const auto points=surface(truth);CuboidPose initial=truth;initial.position+=Point(.045,-.035,.02);initial.rotation*=exp_so3(Point(0,0,-.1));const auto result=refine_pose(initial,points);ASSERT_TRUE(result.refined);EXPECT_LT((result.pose.position-truth.position).norm(),.006);EXPECT_LT(rotation_angle_deg(truth.rotation.transpose()*result.pose.rotation),.8);EXPECT_EQ(result.pose.source,"prototype+sdf");}
// The analytic Jacobian replaced a central-difference one, so pin it against
// central differences. Points sit on faces but away from edges/corners, and the
// probe rotations are finite (non-zero x.head<3>()) because the rotation block
// only agrees there once the SO(3) right Jacobian is included.
TEST(Refine, AnalyticJacobianMatchesCentralDifferences) {
  CuboidPose base; base.position = Point(.2, -.1, .35); base.rotation = exp_so3(Point(.05, -.03, .22));
  const Point half(base.dims[0] / 2., base.dims[1] / 2., base.dims[2] / 2.);
  Points probes;
  for (int u = -2; u <= 2; ++u) {for (int v = -2; v <= 2; ++v) {
      probes.push_back(base.position + base.rotation * Point(half.x() * u / 3., half.y() * v / 3., half.z() * 1.15));
      probes.push_back(base.position + base.rotation * Point(half.x() * u / 3., half.y() * v / 3., half.z() * .55));
    }}
  for (const auto & offset : {Point(0., 0., 0.), Point(.02, -.01, .015), Point(-.03, .012, -.02)}) {
    for (const auto & rotation_vector : {Point(0., 0., 0.), Point(.03, -.02, .12), Point(-.09, .05, -.18)}) {
      const Eigen::Matrix3d r = base.rotation * exp_so3(rotation_vector);
      const Eigen::Matrix3d jr = right_jacobian_so3(rotation_vector);
      const auto residual_at = [&](const Point & rv, const Point & t, const Point & p) {
          return box_sdf((base.rotation * exp_so3(rv)).transpose() * (p - (base.position + t)), half);
        };
      for (const auto & point : probes) {
        const Point q = r.transpose() * (point - (base.position + offset));
        Point gradient; box_sdf_gradient(q, half, &gradient);
        Eigen::Matrix<double, 1, 6> analytic;
        analytic.block<1, 3>(0, 0) = (gradient.transpose() * skew(q)) * jr;
        analytic.block<1, 3>(0, 3) = -(gradient.transpose() * r.transpose());
        constexpr double h = 1.0e-6;
        for (int column = 0; column < 6; ++column) {
          Point rv_plus = rotation_vector, rv_minus = rotation_vector, t_plus = offset, t_minus = offset;
          if (column < 3) {rv_plus[column] += h; rv_minus[column] -= h;} else {t_plus[column - 3] += h; t_minus[column - 3] -= h;}
          const double numeric =
            (residual_at(rv_plus, t_plus, point) - residual_at(rv_minus, t_minus, point)) / (2. * h);
          EXPECT_NEAR(analytic[column], numeric, 2.0e-6)
            << "column=" << column << " rv=" << rotation_vector.transpose() << " t=" << offset.transpose();
        }
      }
    }
  }
}

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

std::string solver_comparison(
  const RefineDiagnostics & actual,
  const nlohmann::json & expected)
{
  std::ostringstream stream;
  stream << "python=" << expected.dump()
         << " cxx={attempted=" << actual.attempted
         << ", evaluations=" << actual.evaluations
         << ", iterations=" << actual.iterations
         << ", initial_cost=" << actual.initial_cost
         << ", final_cost=" << actual.final_cost
         << ", correction=[";
  for (std::size_t index = 0; index < actual.correction.size(); ++index) {
    if (index != 0U) {stream << ',';}
    stream << actual.correction[index];
  }
  stream << "]"
         << ", translation_norm=" << actual.translation_norm
         << ", rotation_deg=" << actual.rotation_deg
         << ", guard_accepted=" << actual.guard_accepted << '}';
  return stream.str();
}

Eigen::Matrix<double, 6, 1> correction_from_json(const nlohmann::json & values)
{
  EXPECT_EQ(values.size(), 6U);
  Eigen::Matrix<double, 6, 1> correction;
  for (int index = 0; index < 6; ++index) {
    correction[index] = values.at(static_cast<std::size_t>(index)).get<double>();
  }
  return correction;
}

// This answers the narrowest useful solver question before changing any
// optimization code: does SciPy's reported correction improve the *same C++
// SDF+Huber objective more than the C++ correction does?  If it does, then the
// remaining mismatch is solver trajectory/termination, not objective math.
TEST(Refine, ScatterPassZeroComparesPythonAndCxxObjective)
{
  const std::filesystem::path blockpose(BLOCKPOSE_SOURCE_ROOT);
  const std::filesystem::path fixture_dir = std::filesystem::temp_directory_path() /
    ("concrete_block_detector_refine_objective_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::remove_all(fixture_dir);
  const std::string command =
    "PYTHONPATH='" + (blockpose / "python").string() + "' python3 '" +
    (blockpose / "tools/export_cpp_parity_fixture.py").string() + "' --manifest '" +
    (blockpose.parent_path() / "concrete_block_detector/test/parity/fixtures_manifest.json").string() +
    "' --output '" + fixture_dir.string() + "' --plain-text";
  ASSERT_EQ(std::system(command.c_str()), 0);

  std::ifstream stream(fixture_dir / "scatter_and_clutter.refinement.json");
  ASSERT_TRUE(stream.is_open());
  nlohmann::json fixture;
  stream >> fixture;
  std::vector<CuboidPose> initial;
  for (const auto & value : fixture.at("initial")) {initial.push_back(cuboid_from_json(value));}
  ASSERT_FALSE(initial.empty());
  const Points points = load_xyz(fixture_dir / "scatter_and_clutter.refine_points.xyz");
  const auto assigned = assign_points(initial, points, 0.10);
  ASSERT_FALSE(assigned.at(0).empty());
  const auto actual = refine_pose(initial.at(0), assigned.at(0));
  ASSERT_TRUE(actual.diagnostics.attempted);
  const auto & solver = fixture.at("passes").at(0).at("solver").at(0);
  const auto python_correction = correction_from_json(solver.at("solution_x"));
  Eigen::Matrix<double, 6, 1> cxx_correction;
  for (int index = 0; index < 6; ++index) {
    cxx_correction[index] = actual.diagnostics.correction[static_cast<std::size_t>(index)];
  }
  const double python_objective = refinement_huber_objective(
    initial.at(0), assigned.at(0), python_correction, 0.05);
  const double cxx_objective = refinement_huber_objective(
    initial.at(0), assigned.at(0), cxx_correction, 0.05);
  const double objective_tolerance = 1.0e-10;
  ::testing::Test::RecordProperty("python_solution_cxx_objective", std::to_string(python_objective));
  ::testing::Test::RecordProperty("cxx_solution_cxx_objective", std::to_string(cxx_objective));
  ::testing::Test::RecordProperty(
    "python_solution_is_no_worse", python_objective <= cxx_objective + objective_tolerance ? "true" : "false");
  SCOPED_TRACE(
    "scatter_and_clutter pass=0 candidate=0 cxx_objective_at_python_solution=" +
    std::to_string(python_objective) + " cxx_objective_at_cxx_solution=" +
    std::to_string(cxx_objective) + " python_reported_cost=" +
    std::to_string(solver.at("cost").get<double>()) + " cxx_reported_final_cost=" +
    std::to_string(actual.diagnostics.final_cost));
  EXPECT_NEAR(python_objective, solver.at("cost").get<double>(), 1.0e-10);
  EXPECT_NEAR(cxx_objective, actual.diagnostics.final_cost, 1.0e-12);
  // SciPy's trust-region solution should be no worse in this shared objective.
  EXPECT_LE(python_objective, cxx_objective + objective_tolerance);
}

// This is intentionally a separate gate from detector parity. It supplies the
// C++ optimizer with the exact Python selected initial poses and assignment
// cloud, so a failure cannot be attributed to RANSAC, NMS, or association.
TEST(Refine, PythonSolverBoundaryParity)
{
  const std::filesystem::path blockpose(BLOCKPOSE_SOURCE_ROOT);
  const std::filesystem::path fixture_dir = std::filesystem::temp_directory_path() /
    ("concrete_block_detector_refine_parity_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
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
    std::size_t pass_index = 0U;
    for (const auto & pass : fixture.at("passes")) {
      const auto assigned = assign_points(current, points, 0.10);
      ASSERT_EQ(assigned.size(), pass.at("assigned_counts").size()) << id;
      for (std::size_t index = 0; index < current.size(); ++index) {
        if (assigned[index].size() != pass.at("assigned_counts").at(index).get<std::size_t>()) {
          ADD_FAILURE() << id << " pass=" << pass_index << " candidate=" << index
                        << " assigned_points cxx=" << assigned[index].size()
                        << " python=" << pass.at("assigned_counts").at(index);
          return;
        }
        const auto actual = refine_pose(current[index], assigned[index]);
        const CuboidPose expected = cuboid_from_json(pass.at("poses").at(index));
        const auto & expected_solver = pass.at("solver").at(index);
        const std::string detail = solver_comparison(actual.diagnostics, expected_solver);
        const double position_error = (actual.pose.position - expected.position).norm();
        const double rotation_error = symmetry_aware_angle_deg(actual.pose.rotation, expected.rotation);
        if (actual.refined != pass.at("refined").at(index).get<bool>() ||
          position_error > 0.005 || rotation_error > 0.5)
        {
          ADD_FAILURE() << id << " pass=" << pass_index << " candidate=" << index
                        << " refined cxx=" << actual.refined
                        << " python=" << pass.at("refined").at(index)
                        << " position_error=" << position_error
                        << " rotation_error_deg=" << rotation_error << ' ' << detail;
          return;
        }
        current[index] = actual.pose;
      }
      ++pass_index;
    }
  }
}
}}  // namespace

#include "concrete_block_detector/detector_core_pipeline.hpp"
#include "concrete_block_detector/gripper_self_filter.hpp"

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace concrete_block_detector::snapshot_runner
{
namespace
{
using detector_core::DetectionParameters;
using detector_core::DetectionResult;
using detector_core::Point;
using detector_core::Points;
using detector_core::SensorContext;
using json = nlohmann::json;

struct RuntimeParameters
{
  DetectionParameters detector;
  bool refine_enabled{true};
  bool scene_bounds_enabled{false};
  std::array<double, 3> scene_bounds_min{};
  std::array<double, 3> scene_bounds_max{};
  bool gripper_self_filter_enabled{false};
  double gripper_self_filter_outboard_extent_m{1.50};
  double gripper_self_filter_cross_rail_extent_m{1.20};
  double gripper_self_filter_rail_end_margin_m{0.15};
  struct GripperRail
  {
    std::string parent_frame;
    std::string rail_frame;
    Point outward_axis_local{Point::UnitZ()};
  };
  std::vector<GripperRail> gripper_self_filter_rails;
  struct FkPrior
  {
    std::string tcp_frame;
    Point tcp_to_block_xyz{Point::Zero()};
    Point tcp_to_block_rpy{Point::Zero()};
    double weight{0.0};
    double translation_tolerance_m{0.30};
    double orientation_tolerance_rad{0.70};
  } fk_prior;
};

struct Arguments
{
  std::vector<std::filesystem::path> snapshots;
  std::vector<std::filesystem::path> params_files;
  bool disable_fk_prior{false};
};

[[noreturn]] void usage(const std::string & message)
{
  throw std::invalid_argument(
          message + "\nusage: concrete_block_detector_snapshot_runner "
          "--snapshot <snapshot-dir> [--snapshot <snapshot-dir> ...] "
          "[--params <ros-parameters.yaml> ...] [--no-fk-prior]");
}

Arguments parse_arguments(int argc, char * argv[])
{
  Arguments result;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--no-fk-prior") {
      result.disable_fk_prior = true;
      continue;
    }
    if (argument == "--snapshot" || argument == "--params") {
      if (++index >= argc) {usage("missing value for " + argument);}
      const std::filesystem::path value(argv[index]);
      if (argument == "--snapshot") {result.snapshots.push_back(value);} else {result.params_files.push_back(value);}
      continue;
    }
    if (argument == "--help" || argument == "-h") {usage("offline detector snapshot runner");}
    usage("unknown argument: " + argument);
  }
  if (result.snapshots.empty()) {usage("at least one --snapshot is required");}
  return result;
}

template<typename T>
void assign_if_present(const YAML::Node & node, const char * name, T & output)
{
  if (const auto value = node[name]) {output = value.as<T>();}
}

std::array<double, 3> yaml_vec3(const YAML::Node & node, const char * name)
{
  const auto value = node[name];
  if (!value || !value.IsSequence() || value.size() != 3U) {
    throw std::invalid_argument(std::string(name) + " must contain exactly three values");
  }
  return {{value[0].as<double>(), value[1].as<double>(), value[2].as<double>()}};
}

void load_parameters_file(const std::filesystem::path & path, RuntimeParameters & runtime)
{
  const YAML::Node document = YAML::LoadFile(path.string());
  const YAML::Node root = document["concrete_block_detector"];
  const YAML::Node parameters = root ? root["ros__parameters"] : YAML::Node{};
  if (!parameters) {
    throw std::invalid_argument("missing concrete_block_detector.ros__parameters in " + path.string());
  }
  assign_if_present(parameters, "refine_enabled", runtime.refine_enabled);
  if (const auto bounds = parameters["scene_bounds"]) {
    assign_if_present(bounds, "enabled", runtime.scene_bounds_enabled);
    if (bounds["min_m"]) {runtime.scene_bounds_min = yaml_vec3(bounds, "min_m");}
    if (bounds["max_m"]) {runtime.scene_bounds_max = yaml_vec3(bounds, "max_m");}
  }
  if (const auto filter = parameters["gripper_self_filter"]) {
    assign_if_present(filter, "enabled", runtime.gripper_self_filter_enabled);
    assign_if_present(filter, "outboard_extent_m", runtime.gripper_self_filter_outboard_extent_m);
    assign_if_present(filter, "cross_rail_extent_m", runtime.gripper_self_filter_cross_rail_extent_m);
    assign_if_present(filter, "rail_end_margin_m", runtime.gripper_self_filter_rail_end_margin_m);
    runtime.gripper_self_filter_rails.clear();
    for (const std::string & rail_name : {"left_rail", "right_rail"}) {
      const auto rail = filter[rail_name];
      if (!rail) {continue;}
      RuntimeParameters::GripperRail parsed;
      parsed.parent_frame = rail["parent_frame"].as<std::string>();
      parsed.rail_frame = rail["rail_frame"].as<std::string>();
      const auto outward = yaml_vec3(rail, "outward_axis_local");
      parsed.outward_axis_local = Point(outward[0], outward[1], outward[2]);
      runtime.gripper_self_filter_rails.push_back(std::move(parsed));
    }
  }
  if (const auto priors = parameters["pose_priors"]) {
    if (const auto fk = priors["fk"]) {
      assign_if_present(fk, "tcp_frame", runtime.fk_prior.tcp_frame);
      if (fk["tcp_to_block_xyz"]) {
        const auto values = yaml_vec3(fk, "tcp_to_block_xyz");
        runtime.fk_prior.tcp_to_block_xyz = Point(values[0], values[1], values[2]);
      }
      if (fk["tcp_to_block_rpy"]) {
        const auto values = yaml_vec3(fk, "tcp_to_block_rpy");
        runtime.fk_prior.tcp_to_block_rpy = Point(values[0], values[1], values[2]);
      }
      assign_if_present(fk, "weight", runtime.fk_prior.weight);
      assign_if_present(fk, "translation_tolerance_m", runtime.fk_prior.translation_tolerance_m);
      assign_if_present(fk, "orientation_tolerance_rad", runtime.fk_prior.orientation_tolerance_rad);
    }
  }
  const auto detector = parameters["detector"];
  if (!detector) {return;}
  auto & p = runtime.detector;
  assign_if_present(detector, "voxel_size", p.voxel_size);
  assign_if_present(detector, "ground_thickness", p.ground_thickness);
  assign_if_present(detector, "ground_ransac_distance", p.ground_ransac_distance);
  assign_if_present(detector, "ground_ransac_iterations", p.ground_ransac_iterations);
  assign_if_present(detector, "ground_normal_min_z", p.ground_normal_min_z);
  assign_if_present(detector, "local_ground_cell_size", p.local_ground_cell_size);
  assign_if_present(detector, "local_ground_clearance", p.local_ground_clearance);
  assign_if_present(detector, "dbscan_eps", p.dbscan_eps);
  assign_if_present(detector, "dbscan_min_points", p.dbscan_min_points);
  assign_if_present(detector, "cluster_min_size", p.cluster_min_size);
  assign_if_present(detector, "cluster_max_size", p.cluster_max_size);
  assign_if_present(detector, "cluster_min_extent_xy", p.cluster_min_extent_xy);
  assign_if_present(detector, "cluster_max_extent_xy", p.cluster_max_extent_xy);
  assign_if_present(detector, "region_max_extent_xy", p.region_max_extent_xy);
  assign_if_present(detector, "cluster_min_extent_z", p.cluster_min_extent_z);
  assign_if_present(detector, "cluster_max_extent_z", p.cluster_max_extent_z);
  assign_if_present(detector, "cluster_max_center_z", p.cluster_max_center_z);
  assign_if_present(detector, "ransac_distance", p.ransac_distance);
  assign_if_present(detector, "ransac_iterations", p.ransac_iterations);
  assign_if_present(detector, "ransac_search_max_points", p.ransac_search_max_points);
  assign_if_present(detector, "ransac_seed", p.ransac_seed);
  assign_if_present(detector, "max_planes", p.max_planes);
  assign_if_present(detector, "min_inliers", p.min_inliers);
  assign_if_present(detector, "top_plane_angle_deg", p.top_plane_angle_deg);
  assign_if_present(detector, "side_plane_angle_deg", p.side_plane_angle_deg);
  assign_if_present(detector, "refine_band", p.refine_band);
  assign_if_present(detector, "refine_iterations", p.refine_iterations);
  assign_if_present(detector, "refine_min_points", p.refine_min_points);
  assign_if_present(detector, "refine_huber_scale", p.refine_huber_scale);
  assign_if_present(detector, "refine_max_translation", p.refine_max_translation);
  assign_if_present(detector, "refine_max_rotation_deg", p.refine_max_rotation_deg);
  assign_if_present(detector, "refine_preserve_top_axis_if_gravity_worsens", p.refine_preserve_top_axis_if_gravity_worsens);
  assign_if_present(detector, "min_score", p.min_score);
  assign_if_present(detector, "fk_seed_min_evidence_gain", p.fk_seed_min_evidence_gain);
  assign_if_present(detector, "conflict_alternatives", p.conflict_alternatives);
  assign_if_present(detector, "proposal_max_components", p.proposal_max_components);
  if (detector["block_dims"]) {p.block_dims = yaml_vec3(detector, "block_dims");}
  if (detector["candidate_dims"]) {
    const auto dims = detector["candidate_dims"];
    if (!dims.IsSequence() || dims.size() % 3U != 0U) {throw std::invalid_argument("detector.candidate_dims must be a flattened sequence of triples");}
    p.candidate_dims.clear();
    for (std::size_t index = 0; index < dims.size(); index += 3U) {
      p.candidate_dims.push_back({{dims[index].as<double>(), dims[index + 1U].as<double>(), dims[index + 2U].as<double>()}});
    }
  }
}

Points load_ascii_xyz_pcd(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {throw std::runtime_error("unable to open PCD: " + path.string());}
  bool fields_xyz = false;
  bool ascii = false;
  std::string line;
  while (std::getline(stream, line)) {
    std::istringstream words(line);
    std::string key;
    words >> key;
    if (key == "FIELDS") {
      std::string fields;
      std::getline(words, fields);
      fields_xyz = fields == " x y z";
    } else if (key == "DATA") {
      std::string encoding;
      words >> encoding;
      ascii = encoding == "ascii";
      break;
    }
  }
  if (!fields_xyz || !ascii) {
    throw std::invalid_argument("only DATA ascii PCD with exactly FIELDS x y z is supported: " + path.string());
  }
  Points points;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  while (stream >> x >> y >> z) {
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {points.emplace_back(x, y, z);}
  }
  if (!stream.eof()) {throw std::runtime_error("invalid XYZ data in PCD: " + path.string());}
  if (points.empty()) {throw std::invalid_argument("PCD has no finite XYZ points: " + path.string());}
  return points;
}

Eigen::Matrix4d load_transform(
  const std::filesystem::path & path, const std::string & parent, const std::string & child)
{
  const YAML::Node document = YAML::LoadFile(path.string());
  const auto transforms = document["transforms"];
  if (!transforms || !transforms.IsSequence()) {throw std::invalid_argument("tf.yaml has no transforms sequence: " + path.string());}
  for (const auto & transform : transforms) {
    if (!transform["parent"] || !transform["child"] ||
      transform["parent"].as<std::string>() != parent || transform["child"].as<std::string>() != child) {
      continue;
    }
    if (transform["available"] && !transform["available"].as<bool>()) {break;}
    const auto matrix = transform["matrix"];
    if (!matrix || !matrix.IsSequence() || matrix.size() != 4U) {break;}
    Eigen::Matrix4d result;
    for (int row = 0; row < 4; ++row) {
      if (!matrix[row].IsSequence() || matrix[row].size() != 4U) {throw std::invalid_argument("TF matrix must be 4x4: " + path.string());}
      for (int column = 0; column < 4; ++column) {result(row, column) = matrix[row][column].as<double>();}
    }
    if (!result.allFinite()) {throw std::invalid_argument("TF matrix contains non-finite values: " + path.string());}
    return result;
  }
  throw std::invalid_argument(
          "tf.yaml has no available transform " + parent + " <- " + child + ": " + path.string());
}

Eigen::Matrix4d load_world_from_cloud(const std::filesystem::path & path)
{
  return load_transform(path, "world", "seyond");
}

Eigen::Matrix3d rotation_from_rpy(const Point & rpy)
{
  return Eigen::AngleAxisd(rpy.z(), Point::UnitZ()).toRotationMatrix() *
         Eigen::AngleAxisd(rpy.y(), Point::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(rpy.x(), Point::UnitX()).toRotationMatrix();
}

detector_core::PosePriors fk_prior_from_snapshot(
  const std::filesystem::path & tf_path, const RuntimeParameters & runtime,
  std::vector<std::string> * unavailable_frames)
{
  if (runtime.fk_prior.weight <= 0.0) {return {};}
  if (runtime.fk_prior.tcp_frame.empty() ||
    runtime.fk_prior.translation_tolerance_m <= 0.0 ||
    runtime.fk_prior.orientation_tolerance_rad <= 0.0 ||
    !std::isfinite(runtime.fk_prior.weight) ||
    !runtime.fk_prior.tcp_to_block_xyz.allFinite() ||
    !runtime.fk_prior.tcp_to_block_rpy.allFinite())
  {
    throw std::invalid_argument("invalid pose_priors.fk configuration");
  }
  try {
    const Eigen::Matrix4d world_from_tcp = load_transform(
      tf_path, "world", runtime.fk_prior.tcp_frame);
    detector_core::PosePrior prior;
    prior.source = "fk";
    prior.position = world_from_tcp.topLeftCorner<3, 3>() *
      runtime.fk_prior.tcp_to_block_xyz + world_from_tcp.topRightCorner<3, 1>();
    prior.rotation = world_from_tcp.topLeftCorner<3, 3>() *
      rotation_from_rpy(runtime.fk_prior.tcp_to_block_rpy);
    prior.dims = runtime.detector.block_dims;
    prior.weight = runtime.fk_prior.weight;
    prior.translation_tolerance_m = runtime.fk_prior.translation_tolerance_m;
    prior.orientation_tolerance_rad = runtime.fk_prior.orientation_tolerance_rad;
    return {prior};
  } catch (const std::invalid_argument &) {
    if (unavailable_frames != nullptr) {unavailable_frames->push_back(runtime.fk_prior.tcp_frame);}
    return {};
  }
}

Points transform_points(const Points & sensor_points, const Eigen::Matrix4d & world_from_sensor)
{
  Points world_points;
  world_points.reserve(sensor_points.size());
  const Eigen::Matrix3d rotation = world_from_sensor.topLeftCorner<3, 3>();
  const Point translation = world_from_sensor.topRightCorner<3, 1>();
  for (const auto & point : sensor_points) {world_points.push_back(rotation * point + translation);}
  return world_points;
}

SensorContext sensor_context_from_world_returns(const Points & returns, const Point & origin)
{
  SensorContext context;
  context.origin = origin;
  context.ray_directions.reserve(returns.size());
  context.ranges.reserve(returns.size());
  for (const auto & point : returns) {
    const Point ray = point - origin;
    const double range = ray.norm();
    if (range > 1e-9 && std::isfinite(range)) {
      context.ray_directions.push_back(ray / range);
      context.ranges.push_back(range);
    }
  }
  return context;
}

void apply_scene_bounds(Points & points, const RuntimeParameters & runtime)
{
  if (!runtime.scene_bounds_enabled) {return;}
  points.erase(std::remove_if(points.begin(), points.end(), [&runtime](const Point & point) {
      for (std::size_t axis = 0; axis < 3U; ++axis) {
        if (point[static_cast<Eigen::Index>(axis)] < runtime.scene_bounds_min[axis] ||
          point[static_cast<Eigen::Index>(axis)] > runtime.scene_bounds_max[axis]) {return true;}
      }
      return false;
    }), points.end());
}

std::vector<detector_core::OrientedBox> gripper_boxes_from_snapshot(
  const std::filesystem::path & tf_path, const RuntimeParameters & runtime,
  std::vector<std::string> * unavailable_frames)
{
  std::vector<detector_core::OrientedBox> result;
  if (!runtime.gripper_self_filter_enabled) {return result;}
  if (!std::isfinite(runtime.gripper_self_filter_outboard_extent_m) ||
    !std::isfinite(runtime.gripper_self_filter_cross_rail_extent_m) ||
    !std::isfinite(runtime.gripper_self_filter_rail_end_margin_m) ||
    runtime.gripper_self_filter_outboard_extent_m <= 0.0 ||
    runtime.gripper_self_filter_cross_rail_extent_m <= 0.0 || runtime.gripper_self_filter_rail_end_margin_m < 0.0) {
    throw std::invalid_argument("gripper self-filter box extents must be finite and positive");
  }
  if (runtime.gripper_self_filter_rails.empty()) {
    throw std::invalid_argument("enabled gripper self-filter requires rail configuration");
  }
  for (const auto & rail : runtime.gripper_self_filter_rails) {
    if (rail.parent_frame.empty() || rail.rail_frame.empty() || rail.outward_axis_local.norm() <= 1e-9) {
      throw std::invalid_argument("enabled gripper self-filter requires parent, rail, and outward-axis configuration");
    }
    try {
      const auto world_from_parent = load_transform(tf_path, "world", rail.parent_frame);
      const auto world_from_rail = load_transform(tf_path, "world", rail.rail_frame);
      const Point rail_start = world_from_parent.topRightCorner<3, 1>();
      const Point rail_end = world_from_rail.topRightCorner<3, 1>();
      const Point rail_axis = rail_end - rail_start;
      const double rail_length = rail_axis.norm();
      if (rail_length <= 1e-9) {throw std::invalid_argument("gripper rail TF endpoints coincide");}
      const Point rail_direction = rail_axis / rail_length;
      Point outward = world_from_parent.topLeftCorner<3, 3>() * rail.outward_axis_local;
      outward -= rail_direction * outward.dot(rail_direction);
      const double outward_length = outward.norm();
      if (outward_length <= 1e-9) {throw std::invalid_argument("gripper outward axis is parallel to rail");}
      outward /= outward_length;
      Eigen::Matrix3d rotation;
      rotation.col(0) = rail_direction;
      rotation.col(1) = outward;
      rotation.col(2) = rail_direction.cross(outward).normalized();
      result.push_back({
        (rail_start + rail_end) * 0.5 + outward * (runtime.gripper_self_filter_outboard_extent_m * 0.5),
        rotation,
        Point(rail_length + 2.0 * runtime.gripper_self_filter_rail_end_margin_m,
          runtime.gripper_self_filter_outboard_extent_m,
          runtime.gripper_self_filter_cross_rail_extent_m)});
    } catch (const std::invalid_argument &) {
      if (unavailable_frames != nullptr) {
        unavailable_frames->push_back(rail.parent_frame);
        unavailable_frames->push_back(rail.rail_frame);
      }
    }
  }
  return result;
}

json pose_json(const detector_core::CuboidHypothesis & hypothesis)
{
  const auto & pose = hypothesis.pose;
  const auto & evidence = hypothesis.evidence;
  return {
    {"position", {pose.position.x(), pose.position.y(), pose.position.z()}},
    {"rotation", {{pose.rotation(0, 0), pose.rotation(0, 1), pose.rotation(0, 2)}, {pose.rotation(1, 0), pose.rotation(1, 1), pose.rotation(1, 2)}, {pose.rotation(2, 0), pose.rotation(2, 1), pose.rotation(2, 2)}}},
    {"dims", pose.dims},
    {"confidence", pose.confidence},
    {"evidence", {{"score", evidence.score}, {"support_points", evidence.support_points}, {"top_height_error_m", evidence.top_height_error_m}, {"expected_visible_faces", evidence.expected_visible_faces}, {"covered_visible_faces", evidence.covered_visible_faces}, {"free_space_violations", evidence.free_space_violations}, {"supported_rays", evidence.supported_rays}, {"observed_geometry_faces", evidence.observed_geometry_faces}, {"incident_rays", evidence.incident_rays}}},
    {"proposal_scale_m", hypothesis.proposal_scale_m ? json(*hypothesis.proposal_scale_m) : json(nullptr)},
    {"support_height_m", hypothesis.support_height_m},
    {"prior_match", {{"source", hypothesis.prior_match.source}, {"score", hypothesis.prior_match.score}, {"translation_error_m", hypothesis.prior_match.translation_error_m}, {"orientation_error_rad", hypothesis.prior_match.orientation_error_rad}}},
  };
}

json raw_lineage_json(const detector_core::RawHypothesisLineage & lineage)
{
  const auto pose = pose_json(detector_core::CuboidHypothesis{
    lineage.synthesized_pose, lineage.evidence, {}, {}, 0.0, {}, {}});
  return {
    {"id", lineage.id},
    {"proposal_component", lineage.proposal_component}, {"region", lineage.region},
    {"top_plane", lineage.top_plane},
    {"side_plane", lineage.side_plane ? json(*lineage.side_plane) : json(nullptr)},
    {"top_normal", {lineage.top_normal.x(), lineage.top_normal.y(), lineage.top_normal.z()}},
    {"top_centroid", {lineage.top_centroid.x(), lineage.top_centroid.y(), lineage.top_centroid.z()}},
    {"side_normal", lineage.side_normal ? json({lineage.side_normal->x(), lineage.side_normal->y(), lineage.side_normal->z()}) : json(nullptr)},
    {"side_centroid", lineage.side_centroid ? json({lineage.side_centroid->x(), lineage.side_centroid->y(), lineage.side_centroid->z()}) : json(nullptr)},
    {"candidate_dims", lineage.candidate_dims}, {"top_only", lineage.top_only},
    {"synthesized_pose", pose},
    {"top_support_height_m", lineage.top_support_height_m},
    {"top_patch_inliers", lineage.top_patch_inliers},
    {"top_patch_extent_major_m", lineage.top_patch_extent_major_m},
    {"top_patch_extent_minor_m", lineage.top_patch_extent_minor_m},
    {"top_patch_extent_local_x_m", lineage.top_patch_extent_local_x_m},
    {"top_patch_extent_local_y_m", lineage.top_patch_extent_local_y_m},
    {"accepted_to_raw", lineage.accepted_to_raw},
    {"passed_score_threshold", lineage.passed_score_threshold}, {"fate", lineage.fate},
  };
}

json counts_json(const detector_core::PipelineCounts & counts)
{
  return {{"input_points", counts.input_points}, {"downsampled_points", counts.downsampled_points}, {"above_support_points", counts.above_support_points}, {"proposal_components", counts.proposal_components}, {"plane_regions", counts.plane_regions}, {"plane_fit_calls", counts.plane_fit_calls}, {"plane_search_points", counts.plane_search_points}, {"plane_full_points_scored", counts.plane_full_points_scored}, {"plane_trials_evaluated", counts.plane_trials_evaluated}, {"plane_valid_trials", counts.plane_valid_trials}, {"raw_hypotheses", counts.raw_hypotheses}, {"refinement_candidates", counts.refinement_candidates}, {"selected_hypotheses", counts.selected_hypotheses}};
}

// Diagnostic-only stages before hypothesis synthesis (see the struct
// comments in detector_core_proposals.hpp). raw_lineage above only ever sees
// a (proposal, region, dims, top, side) combination that made it as far as
// synthesize_pose -- these four cover the funnel from raw DBSCAN labels down
// to that point, so a target that never produced any raw_lineage entry is
// still explainable from this JSON alone.
json proposal_diagnostics_json(const detector_core::ProposalComponentDiagnostics & diag)
{
  return {
    {"dbscan_label", diag.dbscan_label}, {"point_count", diag.point_count},
    {"extent", diag.extent}, {"centroid", {diag.centroid.x(), diag.centroid.y(), diag.centroid.z()}},
    {"center_ground_height_m", diag.center_ground_height},
    {"size_gate_ok", diag.size_gate_ok}, {"extent_xy_gate_ok", diag.extent_xy_gate_ok},
    {"extent_z_gate_ok", diag.extent_z_gate_ok}, {"center_z_gate_ok", diag.center_z_gate_ok},
    {"gate_passed", diag.gate_passed}, {"truncated_by_max_components", diag.truncated_by_max_components},
    {"accepted_rank", diag.accepted_rank ? json(*diag.accepted_rank) : json(nullptr)},
  };
}

json wide_proposal_skip_json(const detector_core::WideProposalSkipDiagnostics & diag)
{
  return {
    {"proposal_component", diag.proposal_component}, {"point_count", diag.point_count},
    {"region_count", diag.region_count}, {"extent", diag.extent},
  };
}

json plane_region_diagnostics_json(const detector_core::PlaneRegionDiagnostics & diag)
{
  return {
    {"proposal_component", diag.proposal_component}, {"region", diag.region},
    {"plane_count", diag.plane_count}, {"leftover_points", diag.leftover_points},
    {"stop_reason", diag.stop_reason},
  };
}

json plane_patch_diagnostics_json(const detector_core::PlanePatchDiagnostics & diag)
{
  return {
    {"proposal_component", diag.proposal_component}, {"region", diag.region}, {"plane_index", diag.plane_index},
    {"point_count", diag.point_count},
    {"normal", {diag.normal.x(), diag.normal.y(), diag.normal.z()}},
    {"centroid", {diag.centroid.x(), diag.centroid.y(), diag.centroid.z()}},
    {"residual_mad", diag.residual_mad},
  };
}

json pairing_diagnostics_json(const detector_core::RegionPairingDiagnostics & diag)
{
  json planes = json::array();
  for (const auto & plane : diag.planes) {
    json attempts = json::array();
    for (const auto & attempt : plane.side_attempts) {
      attempts.push_back({
        {"side_plane_index", attempt.side_plane_index},
        {"drop_m", attempt.drop_m}, {"gap_m", attempt.gap_m}, {"tangential_overlap_m", attempt.tangential_overlap_m},
        {"drop_range_ok", attempt.drop_range_ok}, {"support_band_ok", attempt.support_band_ok},
        {"tangential_overlap_ok", attempt.tangential_overlap_ok}, {"gap_ok", attempt.gap_ok},
        {"selected", attempt.selected},
      });
    }
    planes.push_back({
      {"plane_index", plane.plane_index}, {"is_top", plane.is_top}, {"is_side", plane.is_side},
      {"chosen_side_plane_index", plane.chosen_side_plane_index ? json(*plane.chosen_side_plane_index) : json(nullptr)},
      {"side_attempts", attempts},
    });
  }
  return {
    {"proposal_component", diag.proposal_component}, {"region", diag.region}, {"dims_index", diag.dims_index},
    {"planes", planes},
  };
}

json run_snapshot(const std::filesystem::path & snapshot, const RuntimeParameters & runtime)
{
  const Points sensor_points = load_ascii_xyz_pcd(snapshot / "cloud.pcd");
  const Eigen::Matrix4d world_from_sensor = load_world_from_cloud(snapshot / "tf.yaml");
  const Points world_returns = transform_points(sensor_points, world_from_sensor);
  const Point origin = world_from_sensor.topRightCorner<3, 1>();
  const SensorContext context = sensor_context_from_world_returns(world_returns, origin);
  Points detection_points = world_returns;
  apply_scene_bounds(detection_points, runtime);
  std::vector<std::string> unavailable_gripper_frames;
  const auto gripper_boxes = gripper_boxes_from_snapshot(
    snapshot / "tf.yaml", runtime, &unavailable_gripper_frames);
  std::vector<std::string> unavailable_prior_frames;
  const auto priors = fk_prior_from_snapshot(
    snapshot / "tf.yaml", runtime, &unavailable_prior_frames);
  std::size_t gripper_points_removed = 0U;
  if (!gripper_boxes.empty()) {
    detection_points = detector_core::remove_points_inside_oriented_boxes(
      detection_points, gripper_boxes, &gripper_points_removed);
  }
  const auto started = std::chrono::steady_clock::now();
  const DetectionResult detection = runtime.refine_enabled ?
    detector_core::detect(detection_points, runtime.detector, &context, &priors) :
    detector_core::detect_without_refinement(detection_points, runtime.detector, &context, &priors);
  const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  json poses = json::array();
  for (const auto & hypothesis : detection.hypotheses) {poses.push_back(pose_json(hypothesis));}
  json raw_lineage = json::array();
  for (const auto & lineage : detection.raw_lineage) {raw_lineage.push_back(raw_lineage_json(lineage));}
  json proposal_diagnostics = json::array();
  for (const auto & diag : detection.proposal_diagnostics) {proposal_diagnostics.push_back(proposal_diagnostics_json(diag));}
  json wide_proposal_skips = json::array();
  for (const auto & diag : detection.wide_proposal_skips) {wide_proposal_skips.push_back(wide_proposal_skip_json(diag));}
  json plane_region_diagnostics = json::array();
  for (const auto & diag : detection.plane_region_diagnostics) {plane_region_diagnostics.push_back(plane_region_diagnostics_json(diag));}
  json plane_patch_diagnostics = json::array();
  for (const auto & diag : detection.plane_patch_diagnostics) {plane_patch_diagnostics.push_back(plane_patch_diagnostics_json(diag));}
  json pairing_diagnostics = json::array();
  for (const auto & diag : detection.pairing_diagnostics) {pairing_diagnostics.push_back(pairing_diagnostics_json(diag));}
  return {
    {"snapshot", snapshot.filename().string()},
    {"snapshot_path", snapshot.string()},
    {"schema_version", 1},
    {"world_frame", "world"},
    {"cloud_frame", "seyond"},
    {"raw_sensor_points", sensor_points.size()},
    {"world_points_before_scene_bounds", world_returns.size()},
    {"detector_input_points", detection_points.size()},
    {"sensor_origin_world", {origin.x(), origin.y(), origin.z()}},
    {"refine_enabled", runtime.refine_enabled},
    {"scene_bounds_enabled", runtime.scene_bounds_enabled},
    {"gripper_self_filter_enabled", runtime.gripper_self_filter_enabled},
    {"gripper_filter_boxes", gripper_boxes.size()},
    {"gripper_filter_points_removed", gripper_points_removed},
    {"gripper_filter_unavailable_frames", unavailable_gripper_frames},
    {"pose_priors", priors.size()},
    {"pose_prior_unavailable_frames", unavailable_prior_frames},
    {"runtime_ms", elapsed},
    {"counts", counts_json(detection.counts)},
    {"poses", poses}, {"raw_lineage", raw_lineage},
    {"proposal_diagnostics", proposal_diagnostics},
    {"wide_proposal_skips", wide_proposal_skips},
    {"plane_region_diagnostics", plane_region_diagnostics},
    {"plane_patch_diagnostics", plane_patch_diagnostics},
    {"pairing_diagnostics", pairing_diagnostics},
  };
}
}  // namespace
}  // namespace concrete_block_detector::snapshot_runner

int main(int argc, char * argv[])
{
  using namespace concrete_block_detector::snapshot_runner;
  try {
    const Arguments arguments = parse_arguments(argc, argv);
    RuntimeParameters parameters;
    for (const auto & path : arguments.params_files) {load_parameters_file(path, parameters);}
    if (arguments.disable_fk_prior) {parameters.fk_prior.weight = 0.0;}
    json output;
    output["schema_version"] = 1;
    output["runner"] = "concrete_block_detector_snapshot_runner";
    output["parameter_files"] = json::array();
    for (const auto & path : arguments.params_files) {output["parameter_files"].push_back(path.string());}
    output["fk_prior_enabled"] = parameters.fk_prior.weight > 0.0;
    output["snapshots"] = json::array();
    for (const auto & snapshot : arguments.snapshots) {output["snapshots"].push_back(run_snapshot(snapshot, parameters));}
    std::cout << output.dump(2) << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "concrete_block_detector_snapshot_runner: " << error.what() << '\n';
    return 2;
  }
}

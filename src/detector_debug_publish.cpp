#include "concrete_block_detector/detector_debug_publish.hpp"

#include <rclcpp/time.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <sstream>

namespace concrete_block_detector
{

sensor_msgs::msg::PointCloud2 cloud_from_points(
  const std_msgs::msg::Header & header, const detector_core::Points & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const auto & point : points) {
    *x = static_cast<float>(point.x());
    *y = static_cast<float>(point.y());
    *z = static_cast<float>(point.z());
    ++x; ++y; ++z;
  }
  return cloud;
}

geometry_msgs::msg::Pose pose_message(const detector_core::Pose & pose)
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose.position.x();
  message.position.y = pose.position.y();
  message.position.z = pose.position.z();
  const Eigen::Quaterniond orientation(pose.rotation);
  message.orientation.x = orientation.x();
  message.orientation.y = orientation.y();
  message.orientation.z = orientation.z();
  message.orientation.w = orientation.w();
  return message;
}

visualization_msgs::msg::Marker make_marker(
  const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose,
  const std::array<double, 3> & dims, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "concrete_blocks";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = dims[0];
  marker.scale.y = dims[1];
  marker.scale.z = dims[2];
  marker.color.r = 0.95F;
  marker.color.g = 0.55F;
  marker.color.b = 0.10F;
  marker.color.a = 0.75F;
  return marker;
}

visualization_msgs::msg::Marker make_prior_marker(
  const std_msgs::msg::Header & header, const detector_core::PosePrior & prior, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "concrete_block_priors";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = prior.position.x();
  marker.pose.position.y = prior.position.y();
  marker.pose.position.z = prior.position.z();
  const Eigen::Quaterniond orientation(prior.rotation);
  marker.pose.orientation.x = orientation.x();
  marker.pose.orientation.y = orientation.y();
  marker.pose.orientation.z = orientation.z();
  marker.pose.orientation.w = orientation.w();
  marker.scale.x = prior.dims[0];
  marker.scale.y = prior.dims[1];
  marker.scale.z = prior.dims[2];
  marker.color.r = 0.85F;
  marker.color.g = 0.10F;
  marker.color.b = 0.95F;
  marker.color.a = 0.30F;
  marker.text = prior.source;
  return marker;
}

visualization_msgs::msg::Marker make_gripper_box_marker(
  const std_msgs::msg::Header & header, const GripperFilterBox & filter_box, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "gripper_self_filter";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = filter_box.box.center.x();
  marker.pose.position.y = filter_box.box.center.y();
  marker.pose.position.z = filter_box.box.center.z();
  const Eigen::Quaterniond orientation(filter_box.box.rotation);
  marker.pose.orientation.x = orientation.x();
  marker.pose.orientation.y = orientation.y();
  marker.pose.orientation.z = orientation.z();
  marker.pose.orientation.w = orientation.w();
  marker.scale.x = filter_box.box.size.x();
  marker.scale.y = filter_box.box.size.y();
  marker.scale.z = filter_box.box.size.z();
  marker.color.r = 0.10F;
  marker.color.g = 0.85F;
  marker.color.b = 1.00F;
  marker.color.a = 0.30F;
  return marker;
}

visualization_msgs::msg::Marker make_gripper_centerlines_marker(
  const std_msgs::msg::Header & header, const std::vector<GripperFilterBox> & filter_boxes)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "gripper_self_filter_centerlines";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.025;
  marker.color.r = 0.0F;
  marker.color.g = 0.95F;
  marker.color.b = 1.0F;
  marker.color.a = 1.0F;
  marker.points.reserve(filter_boxes.size() * 2U);
  for (const auto & filter_box : filter_boxes) {
    geometry_msgs::msg::Point start;
    start.x = filter_box.rail_start.x(); start.y = filter_box.rail_start.y(); start.z = filter_box.rail_start.z();
    geometry_msgs::msg::Point end;
    end.x = filter_box.rail_end.x(); end.y = filter_box.rail_end.y(); end.z = filter_box.rail_end.z();
    marker.points.push_back(start);
    marker.points.push_back(end);
  }
  return marker;
}

visualization_msgs::msg::Marker make_gripper_outward_arrows_marker(
  const std_msgs::msg::Header & header, const std::vector<GripperFilterBox> & filter_boxes)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "gripper_self_filter_outward";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.035;
  marker.scale.y = 0.090;
  marker.scale.z = 0.120;
  marker.color.r = 1.0F;
  marker.color.g = 0.85F;
  marker.color.b = 0.0F;
  marker.color.a = 1.0F;
  marker.points.reserve(filter_boxes.size() * 2U);
  for (const auto & filter_box : filter_boxes) {
    const auto midpoint = (filter_box.rail_start + filter_box.rail_end) * 0.5;
    geometry_msgs::msg::Point start;
    start.x = midpoint.x(); start.y = midpoint.y(); start.z = midpoint.z();
    const auto end_point = midpoint + filter_box.outward_normal * 0.35;
    geometry_msgs::msg::Point end;
    end.x = end_point.x(); end.y = end_point.y(); end.z = end_point.z();
    marker.points.push_back(start);
    marker.points.push_back(end);
  }
  return marker;
}

visualization_msgs::msg::Marker make_stage_marker(
  const std_msgs::msg::Header & header, const detector_core::Pose & pose,
  const std::string & name_space, int id, float red, float green, float blue,
  float alpha, bool wireframe)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = name_space;
  marker.id = id;
  marker.type = wireframe ? visualization_msgs::msg::Marker::CUBE : visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose_message(pose);
  marker.scale.x = pose.dims[0];
  marker.scale.y = pose.dims[1];
  marker.scale.z = pose.dims[2];
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = alpha;
  return marker;
}

visualization_msgs::msg::Marker make_stage_text(
  const std_msgs::msg::Header & header, const detector_core::Pose & pose,
  const std::string & name_space, int id, const std::string & text, float red,
  float green, float blue)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = name_space;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose_message(pose);
  marker.pose.position.z += pose.dims[2] * 0.5 + 0.08;
  marker.scale.z = 0.12;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0F;
  marker.text = text;
  return marker;
}

visualization_msgs::msg::Marker make_scene_bounds_marker(
  const std_msgs::msg::Header & header,
  const std::array<double, 3> & min_m,
  const std::array<double, 3> & max_m)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "scene_bounds";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.color.r = 1.0F;
  marker.color.g = 0.25F;
  marker.color.b = 0.85F;
  marker.color.a = 0.9F;

  // Corner i takes max on axis a when bit a of i is set.
  const auto corner = [&min_m, &max_m](int index) {
      geometry_msgs::msg::Point point;
      point.x = ((index & 1) != 0) ? max_m[0] : min_m[0];
      point.y = ((index & 2) != 0) ? max_m[1] : min_m[1];
      point.z = ((index & 4) != 0) ? max_m[2] : min_m[2];
      return point;
    };
  // The 12 edges of the box: pairs differing in exactly one bit.
  constexpr std::array<std::array<int, 2>, 12> kEdges{{
    {{0, 1}}, {{0, 2}}, {{0, 4}}, {{1, 3}}, {{1, 5}}, {{2, 3}},
    {{2, 6}}, {{3, 7}}, {{4, 5}}, {{4, 6}}, {{5, 7}}, {{6, 7}}
  }};
  marker.points.reserve(kEdges.size() * 2U);
  for (const auto & edge : kEdges) {
    marker.points.push_back(corner(edge[0]));
    marker.points.push_back(corner(edge[1]));
  }
  return marker;
}

concrete_block_world_model_interfaces::msg::GroundModel ground_model_message(
  const detector_core::LocalGroundModel & ground, bool valid)
{
  concrete_block_world_model_interfaces::msg::GroundModel message;
  message.valid = valid;
  message.plane_normal.x = ground.normal.x();
  message.plane_normal.y = ground.normal.y();
  message.plane_normal.z = ground.normal.z();
  message.plane_offset = ground.offset;
  message.plane_source = ground.source;
  message.cell_size = ground.cell_size;
  message.cell_ix.reserve(ground.cell_z.size());
  message.cell_iy.reserve(ground.cell_z.size());
  message.cell_z.reserve(ground.cell_z.size());
  for (const auto & cell : ground.cell_z) {
    message.cell_ix.push_back(cell.first.first);
    message.cell_iy.push_back(cell.first.second);
    message.cell_z.push_back(cell.second);
  }
  return message;
}

visualization_msgs::msg::MarkerArray make_ground_cell_markers(
  const std_msgs::msg::Header & header, const detector_core::LocalGroundModel & ground)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header = header;
  clear.ns = "ground_cells";
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  if (ground.cell_z.empty()) {
    return markers;
  }

  // One CUBE_LIST carries every tile: per-cell colour lives in `colors`, so
  // the whole terrain costs a single marker regardless of scene size.
  visualization_msgs::msg::Marker tiles;
  tiles.header = header;
  tiles.ns = "ground_cells";
  tiles.id = 0;
  tiles.type = visualization_msgs::msg::Marker::CUBE_LIST;
  tiles.action = visualization_msgs::msg::Marker::ADD;
  tiles.pose.orientation.w = 1.0;
  // Slightly undersized in x/y so cell edges stay visible as a grid; thin in z
  // so a block resting on the terrain reads as touching rather than sunk.
  tiles.scale.x = ground.cell_size * 0.92;
  tiles.scale.y = ground.cell_size * 0.92;
  tiles.scale.z = 0.02;
  tiles.color.a = 1.0F;

  auto [low, high] = std::minmax_element(
    ground.cell_z.begin(), ground.cell_z.end(),
    [](const auto & a, const auto & b) {return a.second < b.second;});
  const double min_z = low->second;
  const double span = std::max(high->second - min_z, 1e-3);

  tiles.points.reserve(ground.cell_z.size());
  tiles.colors.reserve(ground.cell_z.size());
  for (const auto & cell : ground.cell_z) {
    geometry_msgs::msg::Point point;
    point.x = (static_cast<double>(cell.first.first) + 0.5) * ground.cell_size;
    point.y = (static_cast<double>(cell.first.second) + 0.5) * ground.cell_size;
    point.z = cell.second;
    tiles.points.push_back(point);

    // Blue (low) -> cyan -> yellow -> red (high): monotone in luminance, so it
    // still reads as a height ramp on a greyscale screenshot.
    const auto fraction = static_cast<float>((cell.second - min_z) / span);
    std_msgs::msg::ColorRGBA color;
    color.r = std::clamp(fraction * 2.0F - 0.6F, 0.0F, 1.0F);
    color.g = std::clamp(1.6F - std::abs(fraction - 0.45F) * 3.0F, 0.0F, 1.0F);
    color.b = std::clamp(1.0F - fraction * 2.0F, 0.0F, 1.0F);
    color.a = 0.85F;
    tiles.colors.push_back(color);
  }
  markers.markers.push_back(tiles);
  return markers;
}

visualization_msgs::msg::MarkerArray build_debug_candidate_markers(
  const std_msgs::msg::Header & header, const std::vector<GripperFilterBox> & gripper_boxes,
  const detector_core::PosePriors & priors, const detector_core::DetectionResult & detection,
  bool publish_rejected_candidates)
{
  visualization_msgs::msg::MarkerArray debug_markers;
  visualization_msgs::msg::Marker debug_clear;
  debug_clear.header = header;
  debug_clear.action = visualization_msgs::msg::Marker::DELETEALL;
  debug_markers.markers.push_back(debug_clear);
  if (!gripper_boxes.empty()) {
    debug_markers.markers.push_back(make_gripper_centerlines_marker(header, gripper_boxes));
    debug_markers.markers.push_back(make_gripper_outward_arrows_marker(header, gripper_boxes));
    for (std::size_t index = 0; index < gripper_boxes.size(); ++index) {
      auto marker = make_gripper_box_marker(header, gripper_boxes[index], static_cast<int>(index));
      marker.ns = "module/gripper_self_filter";
      debug_markers.markers.push_back(std::move(marker));
    }
  }
  for (std::size_t index = 0; index < priors.size(); ++index) {
    if (priors[index].weight <= 0.0) {continue;}
    auto marker = make_prior_marker(header, priors[index], static_cast<int>(index));
    marker.ns = "prior/" + priors[index].source;
    debug_markers.markers.push_back(std::move(marker));
  }
  int candidate_id = 0;
  // The trace is exactly the post-refinement set NMS received. In
  // particular it includes cloud-supported FK seeds, which raw plane-fit
  // lineage cannot describe. This makes the visual outlet and the offline
  // runner agree on the candidates being compared.
  for (const auto & candidate : detection.refined_candidate_trace) {
    const bool final = candidate.fate == "final";
    if (!final && !publish_rejected_candidates) {continue;}
    const float red = final ? 1.0F : 0.95F;
    const float green = final ? 0.55F : 0.15F;
    const float blue = final ? 0.10F : 0.15F;
    debug_markers.markers.push_back(make_stage_marker(
      header, candidate.pose,
      final ? "candidate/final" : "candidate/pre_nms_rejected",
      candidate_id, red, green, blue, final ? 0.78F : 0.20F, !final));
    std::ostringstream label;
    label << candidate.id << "\n" << candidate.source
          << " " << candidate.fate << " g=" << candidate.evidence.score;
    if (candidate.prior_match.score > 0.0) {
      label << " " << candidate.prior_match.source << "=" << candidate.prior_match.score;
    }
    if (candidate.visual_evidence.available) {label << " rgb=" << candidate.visual_evidence.score;}
    debug_markers.markers.push_back(make_stage_text(
      header, candidate.pose, "candidate/evidence", candidate_id,
      label.str(), red, green, blue));
    ++candidate_id;
  }
  return debug_markers;
}

std_msgs::msg::String build_diagnostics_message(
  const std_msgs::msg::Header & header, const std::vector<GripperFilterBox> & gripper_boxes,
  const detector_core::PosePriors & priors, const detector_core::DetectionResult & detection,
  const DiscoveryDebugState & state)
{
  using nlohmann::json;
  const auto pose_json = [](const detector_core::Pose & pose) {
      const Eigen::Quaterniond orientation(pose.rotation);
      return json{{"position", {pose.position.x(), pose.position.y(), pose.position.z()}},
        {"orientation_xyzw", {orientation.x(), orientation.y(), orientation.z(), orientation.w()}},
        {"dimensions", pose.dims}};
    };
  json report{
    {"schema", "cbp.scene_discovery.debug/v1"},
    {"cloud_stamp_ns", rclcpp::Time(header.stamp).nanoseconds()},
    {"frame", header.frame_id},
    {"modules", {
      {"scene_bounds", {{"enabled", state.scene_bounds_enabled}}},
      {"gripper_self_filter", {{"enabled", state.gripper_self_filter_module_enabled && state.gripper_self_filter_enabled}, {"available", !gripper_boxes.empty()}, {"boxes", gripper_boxes.size()}, {"removed_points", state.gripper_points_removed}}},
      {"geometry", {{"sdf_refinement_enabled", state.sdf_refinement_module_enabled && state.refine_enabled}, {"fit_wide_proposals", state.fit_wide_proposals}}},
      {"fk_prior", {{"enabled", state.fk_prior_module_enabled && state.fk_prior_weight_positive}, {"available", std::any_of(priors.begin(), priors.end(), [](const auto & prior) {return prior.source == "fk";})}}},
      {"request_priors", {{"enabled", state.request_priors_module_enabled}, {"count", state.request_prior_count}}},
      {"rgb_edge_prior", {{"enabled", state.rgb_edge_prior_module_enabled && state.rgb_edge_prior_enabled}, {"available", state.rgb_edge_available}, {"gate_reason", state.rgb_gate_reason}, {"sync_delta_s", state.rgb_sync_delta_s}}}
    }},
    {"counts", {{"input_points", detection.counts.input_points}, {"downsampled_points", detection.counts.downsampled_points}, {"above_ground_points", detection.counts.above_support_points}, {"proposal_components", detection.counts.proposal_components}, {"plane_regions", detection.counts.plane_regions}, {"raw_hypotheses", detection.counts.raw_hypotheses}, {"refinement_candidates", detection.counts.refinement_candidates}, {"selected_hypotheses", detection.counts.selected_hypotheses}}}
  };
  report["proposals"] = json::array();
  for (const auto & proposal : detection.proposal_diagnostics) {
    report["proposals"].push_back({
        {"label", proposal.dbscan_label}, {"points", proposal.point_count},
        {"extent", proposal.extent}, {"center_ground_height_m", proposal.center_ground_height},
        {"passed", proposal.gate_passed}, {"truncated", proposal.truncated_by_max_components}});
  }
  report["raw_hypotheses"] = json::array();
  for (const auto & lineage : detection.raw_lineage) {
    report["raw_hypotheses"].push_back({
        {"id", lineage.id}, {"fate", lineage.fate}, {"pose", pose_json(lineage.synthesized_pose)},
        {"geometry_score", lineage.evidence.score}, {"support_points", lineage.evidence.support_points},
        {"observed_faces", lineage.evidence.observed_geometry_faces}});
  }
  report["candidate_trace"] = json::array();
  for (const auto & candidate : detection.refined_candidate_trace) {
    report["candidate_trace"].push_back({
        {"id", candidate.id}, {"source", candidate.source},
        {"fate", candidate.fate}, {"pose", pose_json(candidate.pose)},
        {"geometry_score", candidate.evidence.score},
        {"prior", { {"source", candidate.prior_match.source}, {"score", candidate.prior_match.score} }},
        {"visual", { {"available", candidate.visual_evidence.available}, {"score", candidate.visual_evidence.score} }},
        {"selection_score", candidate.selection_score}});
  }
  report["final"] = json::array();
  for (const auto & hypothesis : detection.hypotheses) {
    report["final"].push_back({
        {"pose", pose_json(hypothesis.pose)}, {"geometry_score", hypothesis.evidence.score},
        {"visual", {{"available", hypothesis.visual_evidence.available}, {"score", hypothesis.visual_evidence.score}}},
        {"prior", {{"source", hypothesis.prior_match.source}, {"score", hypothesis.prior_match.score}, {"translation_error_m", hypothesis.prior_match.translation_error_m}, {"orientation_error_rad", hypothesis.prior_match.orientation_error_rad}}}});
  }
  std_msgs::msg::String message;
  message.data = report.dump();
  return message;
}

}  // namespace concrete_block_detector

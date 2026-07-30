#pragma once

// Diagnostic/debug marker and JSON serialization for discover(). Every
// function here is pure over an already-computed DetectionResult and node
// configuration -- no ROS node state, TF, or detection logic lives here.
#include "concrete_block_detector/concrete_block_detector_node.hpp"
#include "concrete_block_detector/detector_core_pipeline.hpp"

#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace concrete_block_detector
{

sensor_msgs::msg::PointCloud2 cloud_from_points(
  const std_msgs::msg::Header & header, const detector_core::Points & points);

geometry_msgs::msg::Pose pose_message(const detector_core::Pose & pose);

visualization_msgs::msg::Marker make_marker(
  const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose,
  const std::array<double, 3> & dims, int id);

visualization_msgs::msg::Marker make_prior_marker(
  const std_msgs::msg::Header & header, const detector_core::PosePrior & prior, int id);

visualization_msgs::msg::Marker make_gripper_box_marker(
  const std_msgs::msg::Header & header, const GripperFilterBox & filter_box, int id);

visualization_msgs::msg::Marker make_gripper_centerlines_marker(
  const std_msgs::msg::Header & header, const std::vector<GripperFilterBox> & filter_boxes);

visualization_msgs::msg::Marker make_gripper_outward_arrows_marker(
  const std_msgs::msg::Header & header, const std::vector<GripperFilterBox> & filter_boxes);

visualization_msgs::msg::Marker make_stage_marker(
  const std_msgs::msg::Header & header, const detector_core::Pose & pose,
  const std::string & name_space, int id, float red, float green, float blue,
  float alpha, bool wireframe);

visualization_msgs::msg::Marker make_stage_text(
  const std_msgs::msg::Header & header, const detector_core::Pose & pose,
  const std::string & name_space, int id, const std::string & text, float red,
  float green, float blue);

// Module/gate state discover() has already resolved by the time it is ready
// to publish debug output (module-enable vs. per-request-applied, the RGB
// sync gate outcome, points the gripper self-filter removed, ...). Bundled
// explicitly, built exactly once per discover() call, so a debug field can
// never go stale by being recomputed differently at a second call site.
struct DiscoveryDebugState
{
  std::size_t request_prior_count{0};
  std::size_t gripper_points_removed{0};
  bool rgb_edge_available{false};
  std::string rgb_gate_reason;
  double rgb_sync_delta_s{0.0};
  bool publish_rejected_candidates{true};
  bool scene_bounds_enabled{false};
  bool gripper_self_filter_module_enabled{true};
  bool gripper_self_filter_enabled{false};
  bool sdf_refinement_module_enabled{true};
  bool refine_enabled{true};
  bool fit_wide_proposals{false};
  bool fk_prior_module_enabled{true};
  bool fk_prior_weight_positive{false};
  bool request_priors_module_enabled{true};
  bool rgb_edge_prior_module_enabled{true};
  bool rgb_edge_prior_enabled{false};
};

// The debug-only overlay: gripper boxes, priors, and every refined candidate
// (final and, unless gated off, pre-NMS rejected). Distinct from the
// request's primary MarkerArray, which only ever shows final blocks.
visualization_msgs::msg::MarkerArray build_debug_candidate_markers(
  const std_msgs::msg::Header & header, const std::vector<GripperFilterBox> & gripper_boxes,
  const detector_core::PosePriors & priors, const detector_core::DetectionResult & detection,
  bool publish_rejected_candidates);

// The `cbp.scene_discovery.debug/v1` diagnostics report: module gates,
// pipeline counts, and per-stage candidate lineage.
std_msgs::msg::String build_diagnostics_message(
  const std_msgs::msg::Header & header, const std::vector<GripperFilterBox> & gripper_boxes,
  const detector_core::PosePriors & priors, const detector_core::DetectionResult & detection,
  const DiscoveryDebugState & state);

}  // namespace concrete_block_detector

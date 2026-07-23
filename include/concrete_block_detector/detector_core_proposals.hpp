#pragma once

// Proposal, plane, hypothesis and selection stages of the pure DetectFree core.
// This is deliberately independent of ROS/PCL so the adapter cannot diverge.
#include "concrete_block_detector/detector_core_geometry.hpp"
#include "concrete_block_detector/detector_core_evidence.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <map>
#include <optional>
#include <tuple>

namespace concrete_block_detector::detector_core
{
struct DetectionParameters
{
  double voxel_size{0.04}, dbscan_eps{0.07};
  double ground_thickness{0.15}, ground_ransac_distance{0.05}, ground_normal_min_z{0.7};
  std::size_t ground_ransac_iterations{500};
  double local_ground_cell_size{0.5}, local_ground_clearance{0.05};
  std::size_t dbscan_min_points{5}, cluster_min_size{30}, cluster_max_size{50000};
  double cluster_min_extent_xy{0.3}, cluster_max_extent_xy{1.5}, region_max_extent_xy{3.0};
  double cluster_min_extent_z{0.3}, cluster_max_extent_z{3.5}, cluster_max_center_z{3.0};
  double ransac_distance{0.02}; std::size_t ransac_iterations{1000}, ransac_search_max_points{2048}; std::uint64_t ransac_seed{0};
  std::size_t max_planes{5}, min_inliers{40}; double top_plane_angle_deg{30.0}, side_plane_angle_deg{20.0}, max_plane_center_dist{0.6};
  std::array<double, 3> block_dims{{0.9, 0.6, 0.6}}; std::vector<std::array<double, 3>> candidate_dims;
  double refine_band{0.10}; std::size_t refine_iterations{2}, refine_min_points{20};
  double refine_huber_scale{0.05}, refine_max_translation{0.15}, refine_max_rotation_deg{20.0};
  double min_score{0.0}; std::size_t conflict_alternatives{1}, proposal_max_components{8}, proposal_max_points{30000};
  bool multiscale_proposals{false};
};
struct PlanePatch {Point normal{Point::UnitZ()}; Point centroid{Point::Zero()}; Points points; double residual_mad{0.0};};
struct Pose {Point position{Point::Zero()}; Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()}; std::array<double, 3> dims{{0.9, 0.6, 0.6}}; double confidence{1.0};};
struct HypothesisEvidence {std::size_t support_points{0}; double top_height_error_m{0.0}, score{0.0}; std::size_t expected_visible_faces{0}, covered_visible_faces{0}, free_space_violations{0}, supported_rays{0}, observed_geometry_faces{0}, incident_rays{0};};
// A provenance record is deliberately kept separate from the numerical
// hypothesis.  It makes a Python/C++ pre-refinement disagreement inspectable
// without changing any score, threshold, or selection behaviour.
struct RawHypothesisLineage
{
  std::string id;
  std::size_t proposal_component{0}, region{0}, top_plane{0};
  std::optional<std::size_t> side_plane;
  Point top_normal{Point::UnitZ()}, top_centroid{Point::Zero()};
  std::optional<Point> side_normal, side_centroid;
  std::array<double, 3> candidate_dims{{0.9, 0.6, 0.6}};
  bool top_only{false}, accepted_to_raw{false}, passed_score_threshold{false};
  Pose synthesized_pose;
  HypothesisEvidence evidence;
  double top_support_height_m{0.0};
  std::string fate{"geometric_rejected"};
};
struct CuboidHypothesis {Pose pose; HypothesisEvidence evidence; std::optional<double> proposal_scale_m; double support_height_m{0.0}; std::optional<std::size_t> lineage_index;};
struct PlaneFitCounts {std::size_t calls{0}, search_points{0}, full_points_scored{0}, trials_evaluated{0}, valid_trials{0};};

// Direct port of blockpose._canonicalize_pose.  The swapped horizontal
// dimensions describe the same physical cuboid; rotate its local frame into
// the package's canonical 0.9 x 0.6 x 0.6 convention before refinement/NMS.
inline Pose canonicalize_pose(Pose pose)
{
  constexpr double tolerance = 1.0e-9;
  if (std::abs(pose.dims[0] - 0.6) <= tolerance &&
    std::abs(pose.dims[1] - 0.9) <= tolerance &&
    std::abs(pose.dims[2] - 0.6) <= tolerance)
  {
    Eigen::Matrix3d local_swap;
    local_swap << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    pose.rotation *= local_swap;
    pose.dims = {{0.9, 0.6, 0.6}};
  }
  return pose;
}

inline std::array<double, 3> extent(const Points & points)
{ Point lower = points.empty() ? Point::Zero() : points.front(), upper = lower; for (const auto & p : points) {lower = lower.cwiseMin(p); upper = upper.cwiseMax(p);} const Point e = upper - lower; return {{e.x(), e.y(), e.z()}}; }
inline Point centroid(const Points & points)
{ Point result = Point::Zero(); for (const auto & p : points) {result += p;} return points.empty() ? result : result / static_cast<double>(points.size()); }
inline double median(std::vector<double> values) {return percentile_linear(std::move(values), 50.0);}

inline std::tuple<long long, long long, long long> spatial_cell(const Point & point, double cell_size)
{
  return std::make_tuple(
    static_cast<long long>(std::floor(point.x() / cell_size)),
    static_cast<long long>(std::floor(point.y() / cell_size)),
    static_cast<long long>(std::floor(point.z() / cell_size)));
}

inline std::map<std::tuple<long long, long long, long long>, std::vector<std::size_t>> make_spatial_grid(
  const Points & points, double cell_size)
{
  std::map<std::tuple<long long, long long, long long>, std::vector<std::size_t>> grid;
  for (std::size_t index = 0; index < points.size(); ++index) {grid[spatial_cell(points[index], cell_size)].push_back(index);}
  return grid;
}

inline std::vector<std::size_t> grid_neighbors(const Points & points,
  const std::map<std::tuple<long long, long long, long long>, std::vector<std::size_t>> & grid,
  std::size_t index, double cell_size, double radius_squared)
{
  const auto [x, y, z] = spatial_cell(points[index], cell_size);
  std::vector<std::size_t> result;
  for (long long dx = -1; dx <= 1; ++dx) for (long long dy = -1; dy <= 1; ++dy) for (long long dz = -1; dz <= 1; ++dz) {
    const auto bucket = grid.find(std::make_tuple(x + dx, y + dy, z + dz));
    if (bucket == grid.end()) {continue;}
    for (const auto other : bucket->second) {if ((points[index] - points[other]).squaredNorm() <= radius_squared) {result.push_back(other);}}
  }
  std::sort(result.begin(), result.end());
  return result;
}

inline std::vector<Points> dbscan_proposals(
  const Points & input, const DetectionParameters & params, const GroundPlane * ground = nullptr)
{
  const Points points = canonical_order(input); const std::size_t count = points.size(); std::vector<int> labels(count, -2); const double radius_sq = params.dbscan_eps * params.dbscan_eps;
  const auto grid = make_spatial_grid(points, params.dbscan_eps);
  const auto neighbors = [&](std::size_t index) {return grid_neighbors(points, grid, index, params.dbscan_eps, radius_sq);};
  int label = 0;
  for (std::size_t index = 0; index < count; ++index) {if (labels[index] != -2) {continue;} auto seeds = neighbors(index); if (seeds.size() < params.dbscan_min_points) {labels[index] = -1; continue;} labels[index] = label;
    for (std::size_t cursor = 0; cursor < seeds.size(); ++cursor) {const auto candidate = seeds[cursor]; if (labels[candidate] == -1) {labels[candidate] = label;} if (labels[candidate] != -2) {continue;} labels[candidate] = label; const auto adjacent = neighbors(candidate); if (adjacent.size() >= params.dbscan_min_points) {for (const auto next : adjacent) {if (std::find(seeds.begin(), seeds.end(), next) == seeds.end()) {seeds.push_back(next);}}}} ++label;}
  std::vector<Points> components(static_cast<std::size_t>(label)); for (std::size_t i = 0; i < count; ++i) {if (labels[i] >= 0) {components[static_cast<std::size_t>(labels[i])].push_back(points[i]);}}
  std::vector<Points> accepted; for (auto & proposal : components) {const auto e = extent(proposal); const bool valid = proposal.size() >= params.cluster_min_size && proposal.size() <= params.cluster_max_size && e[0] >= params.cluster_min_extent_xy && e[0] <= params.region_max_extent_xy && e[1] >= params.cluster_min_extent_xy && e[1] <= params.region_max_extent_xy && e[2] >= params.cluster_min_extent_z && e[2] <= params.cluster_max_extent_z; if (!valid) {continue;} const Point actual_center = centroid(proposal); if (ground != nullptr && ground->height(actual_center) > params.cluster_max_center_z) {continue;} accepted.push_back(std::move(proposal));}
  std::sort(accepted.begin(), accepted.end(), [&params](const Points & a, const Points & b) {if (a.size() != b.size()) {return a.size() > b.size();} const auto ea = extent(a), eb = extent(b); const auto compat = [&params](const std::array<double, 3> & e) {double value = 0.0; for (std::size_t axis = 0; axis < 2U; ++axis) {const double count = std::clamp(std::round(e[axis] / params.block_dims[axis]), 1.0, 8.0); value += std::abs(e[axis] - count * params.block_dims[axis]) / params.block_dims[axis];} const double height_count = std::clamp(std::round(e[2] / params.block_dims[2]), 1.0, 6.0); return value + 0.5 * std::abs(e[2] - height_count * params.block_dims[2]) / params.block_dims[2];}; const double ca = compat(ea), cb = compat(eb); return ca == cb ? canonical_less(centroid(a), centroid(b)) : ca < cb;}); if (accepted.size() > params.proposal_max_components) {accepted.resize(params.proposal_max_components);} return accepted;
}

inline std::vector<PlanePatch> split_plane_patch(const PlanePatch & patch, const DetectionParameters & params)
{
  if (patch.points.size() < 2U * params.min_inliers) {return {patch};} Point normal = patch.normal.normalized(), seed = std::abs(normal.x()) > .9 ? Point::UnitY() : Point::UnitX(); Point first = (seed - seed.dot(normal) * normal).normalized(), second = normal.cross(first); const double radius = std::max(params.voxel_size * 2.5, params.dbscan_eps * 1.25), radius_sq = radius * radius; const auto count = patch.points.size(); std::vector<std::size_t> parent(count); for (std::size_t i = 0; i < count; ++i) {parent[i] = i;}
  const auto root = [&parent](std::size_t value) {std::size_t r = value; while (parent[r] != r) {r = parent[r];} while (parent[value] != value) {const auto next = parent[value]; parent[value] = r; value = next;} return r;};
  Points projected; projected.reserve(count); for (const auto & point : patch.points) {projected.emplace_back(point.dot(first), point.dot(second), 0.0);} const auto grid = make_spatial_grid(projected, radius); for (std::size_t i = 0; i < count; ++i) {for (const auto j : grid_neighbors(projected, grid, i, radius, radius_sq)) {if (j <= i) {continue;} const auto a = root(i), b = root(j); if (a != b) {parent[std::max(a, b)] = std::min(a, b);}}}
  std::map<std::size_t, Points> groups; for (std::size_t i = 0; i < count; ++i) {groups[root(i)].push_back(patch.points[i]);} std::size_t valid = 0; for (const auto & [_, group] : groups) {if (group.size() >= params.min_inliers) {++valid;}} if (valid < 2) {return {patch};} std::vector<PlanePatch> output{patch};
  for (const auto & [_, group] : groups) {if (group.size() < params.min_inliers) {continue;} const Point center = centroid(group); std::vector<double> residuals; for (const auto & point : group) {residuals.push_back((point - center).dot(normal));} const double middle = median(residuals); for (auto & value : residuals) {value = std::abs(value - middle);} output.push_back({normal, center, group, median(std::move(residuals))});} return output;
}

inline std::pair<std::vector<PlanePatch>, PlaneFitCounts> fit_planes(const Points & region, const DetectionParameters & params)
{
  Points remaining = canonical_order(region); std::vector<PlanePatch> output; PlaneFitCounts counts;
  for (std::size_t index = 0; index < params.max_planes && remaining.size() >= params.min_inliers; ++index) {RansacParameters config; config.distance_threshold = params.ransac_distance; config.num_iterations = params.ransac_iterations; config.seed = params.ransac_seed + index; config.max_search_points = params.ransac_search_max_points; PlaneFitResult fitted; try {fitted = segment_plane(remaining, config);} catch (const std::exception &) {break;} ++counts.calls; counts.search_points += fitted.diagnostics.search_points; counts.full_points_scored += fitted.diagnostics.input_points; counts.trials_evaluated += fitted.diagnostics.trials_evaluated; counts.valid_trials += fitted.diagnostics.valid_trials; if (fitted.inlier_indices.size() < params.min_inliers) {break;} Points inliers; std::vector<bool> retained(remaining.size(), true); for (const auto i : fitted.inlier_indices) {inliers.push_back(remaining[i]); retained[i] = false;} const Point normal = fitted.plane.normal.normalized(), center = centroid(inliers); std::vector<double> residuals; for (const auto & point : inliers) {residuals.push_back((point - center).dot(normal));} const double middle = median(residuals); for (auto & value : residuals) {value = std::abs(value - middle);} const auto split = split_plane_patch({normal, center, inliers, median(std::move(residuals))}, params); output.insert(output.end(), split.begin(), split.end()); Points next; for (std::size_t i = 0; i < remaining.size(); ++i) {if (retained[i]) {next.push_back(remaining[i]);}} remaining = std::move(next);}
  return {output, counts};
}

inline std::vector<std::array<double, 3>> candidate_dims(const DetectionParameters & params)
{ std::vector<std::array<double, 3>> output; const auto configured = params.candidate_dims.empty() ? std::vector<std::array<double, 3>>{params.block_dims} : params.candidate_dims; for (const auto dims : configured) {for (const auto variant : {dims, std::array<double, 3>{{dims[1], dims[0], dims[2]}}}) {if (std::find(output.begin(), output.end(), variant) == output.end()) {output.push_back(variant);}}} return output; }

inline std::vector<std::pair<const PlanePatch *, const PlanePatch *>> candidate_plane_sets(const std::vector<PlanePatch> & planes, const DetectionParameters & params, const GroundPlane & ground, const std::array<double, 3> & dims)
{
  const Point up = ground.normal.normalized(); const double top_cos = std::cos(params.top_plane_angle_deg * M_PI / 180.0), side_sin = std::sin(params.side_plane_angle_deg * M_PI / 180.0); std::vector<const PlanePatch *> tops, sides; for (const auto & patch : planes) {if (std::abs(patch.normal.dot(up)) > top_cos) {tops.push_back(&patch);} if (std::abs(patch.normal.dot(up)) < side_sin) {sides.push_back(&patch);}} std::sort(tops.begin(), tops.end(), [&up](const auto * a, const auto * b) {return a->centroid.dot(up) > b->centroid.dot(up);}); std::vector<std::pair<const PlanePatch *, const PlanePatch *>> output;
  for (const auto * top : tops) {const PlanePatch * best = nullptr; double best_score = std::numeric_limits<double>::infinity(), top_height = top->centroid.dot(up); for (const auto * side : sides) {Point n = side->normal - side->normal.dot(up) * up; if (n.norm() < 1e-8) {continue;} n.normalize(); double low = std::numeric_limits<double>::infinity(), high = -low; for (const auto & point : side->points) {const double h = point.dot(up); low = std::min(low, h); high = std::max(high, h);} const double drop = top_height - side->centroid.dot(up); if (drop < .15 || drop > dims[2] - .1 || high < top_height - .15 || low > top_height - .3) {continue;} double gap = std::numeric_limits<double>::infinity(); for (const auto & point : top->points) {gap = std::min(gap, std::abs((point - side->centroid).dot(n)));} const Point tangent = up.cross(n).normalized(); double top_low = std::numeric_limits<double>::infinity(), top_high = -top_low, side_low = std::numeric_limits<double>::infinity(), side_high = -side_low; for (const auto & point : top->points) {const double projection = point.dot(tangent); top_low = std::min(top_low, projection); top_high = std::max(top_high, projection);} for (const auto & point : side->points) {const double projection = point.dot(tangent); side_low = std::min(side_low, projection); side_high = std::max(side_high, projection);} if (std::min(top_high, side_high) - std::max(top_low, side_low) < .15) {continue;} const double score = gap + std::abs(drop - dims[2] / 2.0); if (gap <= .1 && (score < best_score || (score == best_score && best != nullptr && canonical_less(side->centroid, best->centroid)))) {best = side; best_score = score;}} output.emplace_back(top, best);} return output;
}

inline Pose synthesize_pose(const PlanePatch & top, const PlanePatch * side, const Points & cluster, const std::array<double, 3> & dims)
{
  Point z = top.normal.normalized(); if (z.z() < 0.0) {z = -z;} Point x; if (side != nullptr) {x = side->normal - side->normal.dot(z) * z; if (x.norm() < 1e-8) {x = Point::UnitX() - Point::UnitX().dot(z) * z;} x.normalize(); if ((centroid(cluster) - side->centroid).dot(x) < 0.0) {x = -x;}} else {Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(); const Point center = centroid(cluster); for (const auto & point : cluster) {Point d = point - center; d -= d.dot(z) * z; covariance.noalias() += d * d.transpose();} Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solve(covariance); if (solve.info() == Eigen::Success) {x = solve.eigenvectors().col(2);} else {x = Point::UnitX();} x -= x.dot(z) * z; if (x.norm() < 1e-8) {x = Point::UnitX() - Point::UnitX().dot(z) * z;} x.normalize();} Point y = z.cross(x).normalized(); Pose pose; pose.rotation.col(0) = x; pose.rotation.col(1) = y; pose.rotation.col(2) = z; pose.position = top.centroid - z * (dims[2] / 2.0); pose.dims = dims; pose.confidence = side == nullptr ? .5 : 1.; if (side != nullptr) {pose.position += x * (dims[0] / 2.0 - (pose.position - side->centroid).dot(x));} return canonicalize_pose(pose);
}

template<typename GroundModel>
inline CuboidHypothesis make_hypothesis(const Pose & pose, std::size_t support_points, const GroundModel & ground, std::size_t observed_faces, const SensorContext * sensor_context = nullptr)
{ const Point bottom = pose.position - pose.rotation.col(2) * (pose.dims[2] / 2.0); HypothesisEvidence evidence; evidence.support_points = support_points; evidence.top_height_error_m = std::abs(ground.height(bottom)); evidence.observed_geometry_faces = observed_faces; VisibilityEvidence visibility; if (sensor_context != nullptr) {visibility = visibility_evidence(pose.position, pose.rotation, pose.dims, *sensor_context); visibility.violations += free_space_violations_from_misses(pose.position, pose.rotation, pose.dims, *sensor_context); evidence.expected_visible_faces = visibility.expected_faces; evidence.covered_visible_faces = visibility.covered_faces; evidence.free_space_violations = visibility.violations; evidence.supported_rays = visibility.supported_rays; evidence.incident_rays = visibility.incident_rays;} evidence.score = normalized_evidence_score(normalized_evidence_features(evidence.support_points, evidence.top_height_error_m, visibility)); return {pose, evidence, {}, ground.height(bottom)}; }

inline bool boxes_overlap(const Pose & first, const Pose & second, double shrink = .08)
{
  Eigen::Vector3d a, b; for (int i = 0; i < 3; ++i) {a[i] = std::max(first.dims[i] / 2. - shrink, 1e-3); b[i] = std::max(second.dims[i] / 2. - shrink, 1e-3);} const Eigen::Vector3d relative = first.rotation.transpose() * (second.position - first.position); const Eigen::Matrix3d coupling = first.rotation.transpose() * second.rotation, absolute = coupling.cwiseAbs().array() + 1e-9; for (int axis = 0; axis < 3; ++axis) {if (std::abs(relative[axis]) > a[axis] + absolute.row(axis).dot(b)) {return false;}} for (int axis = 0; axis < 3; ++axis) {if (std::abs(relative.dot(coupling.col(axis))) > absolute.col(axis).dot(a) + b[axis]) {return false;}} for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) {const double ra = a[(i + 1) % 3] * absolute((i + 2) % 3, j) + a[(i + 2) % 3] * absolute((i + 1) % 3, j), rb = b[(j + 1) % 3] * absolute(i, (j + 2) % 3) + b[(j + 2) % 3] * absolute(i, (j + 1) % 3), distance = std::abs(relative[(i + 2) % 3] * coupling((i + 1) % 3, j) - relative[(i + 1) % 3] * coupling((i + 2) % 3, j)); if (distance > ra + rb) {return false;}}} return true;
}
inline std::vector<CuboidHypothesis> select_hypotheses(std::vector<CuboidHypothesis> hypotheses, double shrink = .08)
{ std::sort(hypotheses.begin(), hypotheses.end(), [](const auto & a, const auto & b) {return a.evidence.score == b.evidence.score ? canonical_less(a.pose.position, b.pose.position) : a.evidence.score > b.evidence.score;}); std::vector<CuboidHypothesis> selected; for (const auto & candidate : hypotheses) {bool conflict = false; for (const auto & prior : selected) {if (boxes_overlap(candidate.pose, prior.pose, shrink)) {conflict = true; break;}} if (!conflict) {selected.push_back(candidate);}} return selected; }

inline double pose_rotation_error_deg(const Pose & first, const Pose & second)
{
  const Eigen::Matrix3d delta = first.rotation.transpose() * second.rotation;
  return std::acos(std::clamp((delta.trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI;
}

// Direct port of blockpose._select_conflict_alternatives.  Keeping more than
// one diverse seed per overlap group lets refinement choose between competing
// closed-form plane fits; the final greedy NMS still returns one pose/group.
inline std::vector<CuboidHypothesis> select_conflict_alternatives(
  std::vector<CuboidHypothesis> hypotheses, std::size_t limit, double shrink = .08)
{
  if (limit < 1U) {throw std::invalid_argument("conflict_alternatives must be at least one");}
  std::sort(hypotheses.begin(), hypotheses.end(), [](const auto & a, const auto & b) {
    return a.evidence.score == b.evidence.score ? canonical_less(a.pose.position, b.pose.position) :
           a.evidence.score > b.evidence.score;
  });
  std::vector<std::vector<CuboidHypothesis>> groups;
  for (const auto & candidate : hypotheses) {
    std::vector<std::size_t> overlapping;
    for (std::size_t index = 0; index < groups.size(); ++index) {
      if (std::any_of(groups[index].begin(), groups[index].end(), [&candidate, shrink](const auto & member) {
          return boxes_overlap(candidate.pose, member.pose, shrink);
        })) {overlapping.push_back(index);}
    }
    if (overlapping.empty()) {groups.push_back({candidate}); continue;}
    const std::size_t destination = overlapping.front();
    groups[destination].push_back(candidate);
    for (auto iterator = overlapping.rbegin(); iterator != overlapping.rend(); ++iterator) {
      if (*iterator == destination) {continue;}
      groups[destination].insert(groups[destination].end(), groups[*iterator].begin(), groups[*iterator].end());
      groups.erase(groups.begin() + static_cast<std::ptrdiff_t>(*iterator));
    }
  }
  std::vector<CuboidHypothesis> selected;
  for (const auto & group : groups) {
    std::vector<CuboidHypothesis> diverse;
    for (const auto & candidate : group) {
      const bool duplicate = std::any_of(diverse.begin(), diverse.end(), [&candidate](const auto & prior) {
          return (candidate.pose.position - prior.pose.position).norm() < .02 &&
                 pose_rotation_error_deg(candidate.pose, prior.pose) < 2.0;
        });
      if (!duplicate) {diverse.push_back(candidate);}
      if (diverse.size() == limit) {break;}
    }
    selected.insert(selected.end(), diverse.begin(), diverse.end());
  }
  return selected;
}
}  // namespace concrete_block_detector::detector_core

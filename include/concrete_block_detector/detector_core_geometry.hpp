#pragma once

// Pure detector-core geometry; deliberately no ROS or PCL dependency.
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace concrete_block_detector::detector_core
{
using Point = Eigen::Vector3d;
using Points = std::vector<Point>;

struct PlaneFitDiagnostics {std::size_t input_points{0}, search_points{0}, trials_evaluated{0}, valid_trials{0};};
struct PlaneModel {Point normal{Point::UnitZ()}; double offset{0.0};};
struct PlaneFitResult {PlaneModel plane; std::vector<std::size_t> inlier_indices; PlaneFitDiagnostics diagnostics;};
struct RansacParameters {double distance_threshold{0.02}; std::size_t num_iterations{1000}; std::uint64_t seed{0}; double confidence{0.9999}; std::size_t max_search_points{0};};

struct GroundPlane
{
  Point normal{Point::UnitZ()};
  double offset{0.0};
  std::string source{"empty"};
  [[nodiscard]] double height(const Point & p) const {return normal.dot(p) + offset;}
  [[nodiscard]] double support_z(const Point & p) const {return -(normal.x() * p.x() + normal.y() * p.y() + offset) / normal.z();}
};

struct LocalGroundModel : public GroundPlane
{
  double cell_size{0.5};
  std::vector<Eigen::Vector2d> cell_xy;
  std::vector<double> cell_z;
  [[nodiscard]] double support_z(const Point & p) const
  {
    const double planar = GroundPlane::support_z(p);
    if (cell_xy.empty()) {return planar;}
    std::size_t nearest = 0; double distance_sq = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < cell_xy.size(); ++i) {
      const double candidate = (p.head<2>() - cell_xy[i]).squaredNorm();
      if (candidate < distance_sq) {distance_sq = candidate; nearest = i;}
    }
    if (std::sqrt(distance_sq) > cell_size * 1.75) {return planar;}
    return planar + cell_z[nearest] - GroundPlane::support_z(Point(cell_xy[nearest].x(), cell_xy[nearest].y(), 0.0));
  }
  [[nodiscard]] double height(const Point & p) const {return p.z() - support_z(p);}
};

struct GroundParameters
{
  double z_percentile{5.0}, thickness{0.15}, ransac_distance{0.05}, normal_min_z{0.7};
  std::size_t ransac_iterations{500}; std::uint64_t ransac_seed{0};
  double local_cell_size{0.5}, local_clearance{0.05};
};
struct GroundRemovalResult {Points above_ground; LocalGroundModel ground;};

inline bool canonical_less(const Point & a, const Point & b)
{
  if (a.x() != b.x()) {return a.x() < b.x();}
  if (a.y() != b.y()) {return a.y() < b.y();}
  return a.z() < b.z();
}
inline Points canonical_order(Points p) {std::sort(p.begin(), p.end(), canonical_less); return p;}
inline double percentile_linear(std::vector<double> values, double percentile)
{
  if (values.empty() || !std::isfinite(percentile) || percentile < 0.0 || percentile > 100.0) {throw std::invalid_argument("invalid percentile");}
  std::sort(values.begin(), values.end()); const double idx = percentile / 100.0 * static_cast<double>(values.size() - 1U);
  const auto low = static_cast<std::size_t>(std::floor(idx)), high = static_cast<std::size_t>(std::ceil(idx));
  return values[low] + (idx - static_cast<double>(low)) * (values[high] - values[low]);
}
inline PlaneModel plane_from_triplet(const Point & a, const Point & b, const Point & c, bool * valid)
{
  Point normal = (b - a).cross(c - a); const double length = normal.norm();
  if (!std::isfinite(length) || length < 1e-12) {*valid = false; return {};}
  *valid = true; normal /= length; return {normal, -normal.dot(a)};
}

// NumPy default_rng(seed) is PCG64(SeedSequence(seed)).  This is a compact,
// direct port of NumPy 1.24's SeedSequence mixer and PCG XSL RR 128/64 path.
// It intentionally uses the same 32-bit bounded-integer path as
// Generator.integers(0, high), including its cached second uint32.
class NumpyPcg64
{
public:
  explicit NumpyPcg64(std::uint64_t seed)
  {
    const auto words = seed_sequence_state(seed);
    const Uint128 init_state = (static_cast<Uint128>(words[0]) << 64U) | words[1];
    const Uint128 init_sequence = (static_cast<Uint128>(words[2]) << 64U) | words[3];
    increment_ = (init_sequence << 1U) | 1U;
    state_ = 0U;
    step();
    state_ += init_state;
    step();
  }

  [[nodiscard]] std::uint32_t bounded_uint32(std::uint32_t bound)
  {
    if (bound == 0U) {throw std::invalid_argument("PCG bound must be positive");}
    const std::uint32_t threshold = static_cast<std::uint32_t>(-bound) % bound;
    while (true) {
      const std::uint64_t product = static_cast<std::uint64_t>(next_uint32()) * bound;
      if (static_cast<std::uint32_t>(product) >= threshold) {
        return static_cast<std::uint32_t>(product >> 32U);
      }
    }
  }

private:
  // PCG64 requires modulo-2^128 arithmetic. GCC/Clang provide this extension;
  // spell it explicitly so the package's -Wpedantic does not hide a warning.
  __extension__ typedef unsigned __int128 Uint128;
  static constexpr Uint128 kMultiplier =
    (static_cast<Uint128>(0x2360ed051fc65da4ULL) << 64U) | 0x4385df649fccf645ULL;
  static constexpr std::uint32_t kInitA = 0x43b0d7e5U;
  static constexpr std::uint32_t kMultA = 0x931e8875U;
  static constexpr std::uint32_t kInitB = 0x8b51f9ddU;
  static constexpr std::uint32_t kMultB = 0x58f38dedU;
  static constexpr std::uint32_t kMixMultL = 0xca01f9ddU;
  static constexpr std::uint32_t kMixMultR = 0x4973f715U;
  Uint128 state_{0U};
  Uint128 increment_{0U};
  bool has_cached_uint32_{false};
  std::uint32_t cached_uint32_{0U};

  static std::uint32_t hashmix(std::uint32_t value, std::uint32_t * hash_constant)
  {
    value ^= *hash_constant;
    *hash_constant *= kMultA;
    value *= *hash_constant;
    return value ^ (value >> 16U);
  }
  static std::uint32_t mix(std::uint32_t x, std::uint32_t y)
  {
    std::uint32_t result = kMixMultL * x - kMixMultR * y;
    return result ^ (result >> 16U);
  }
  static std::array<std::uint64_t, 4> seed_sequence_state(std::uint64_t seed)
  {
    std::vector<std::uint32_t> entropy{static_cast<std::uint32_t>(seed)};
    if ((seed >> 32U) != 0U) {entropy.push_back(static_cast<std::uint32_t>(seed >> 32U));}
    std::array<std::uint32_t, 4> pool{};
    std::uint32_t hash_constant = kInitA;
    for (std::size_t i = 0; i < pool.size(); ++i) {
      pool[i] = hashmix(i < entropy.size() ? entropy[i] : 0U, &hash_constant);
    }
    for (std::size_t source = 0; source < pool.size(); ++source) {
      for (std::size_t destination = 0; destination < pool.size(); ++destination) {
        if (source != destination) {pool[destination] = mix(pool[destination], hashmix(pool[source], &hash_constant));}
      }
    }
    std::array<std::uint32_t, 8> state{};
    hash_constant = kInitB;
    for (std::size_t i = 0; i < state.size(); ++i) {
      std::uint32_t value = pool[i % pool.size()] ^ hash_constant;
      hash_constant *= kMultB;
      value *= hash_constant;
      state[i] = value ^ (value >> 16U);
    }
    std::array<std::uint64_t, 4> result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
      result[i] = static_cast<std::uint64_t>(state[2U * i]) |
        (static_cast<std::uint64_t>(state[2U * i + 1U]) << 32U);
    }
    return result;
  }
  void step() {state_ = state_ * kMultiplier + increment_;}
  [[nodiscard]] std::uint64_t next_uint64()
  {
    step();
    const std::uint64_t high = static_cast<std::uint64_t>(state_ >> 64U);
    const std::uint64_t low = static_cast<std::uint64_t>(state_);
    const std::uint64_t xorshifted = high ^ low;
    const unsigned int rotation = static_cast<unsigned int>(state_ >> 122U);
    return (xorshifted >> rotation) | (xorshifted << ((-rotation) & 63U));
  }
  [[nodiscard]] std::uint32_t next_uint32()
  {
    if (has_cached_uint32_) {has_cached_uint32_ = false; return cached_uint32_;}
    const std::uint64_t value = next_uint64();
    cached_uint32_ = static_cast<std::uint32_t>(value >> 32U);
    has_cached_uint32_ = true;
    return static_cast<std::uint32_t>(value);
  }
};

// Port of blockpose.ransac.segment_plane semantics, including NumPy's
// default_rng(seed) PCG64 sampling stream.
inline PlaneFitResult segment_plane(const Points & input, const RansacParameters & params)
{
  if (input.size() < 3U) {throw std::invalid_argument("plane fitting needs at least 3 points");}
  if (!std::isfinite(params.distance_threshold) || params.distance_threshold <= 0.0 || params.num_iterations == 0U) {throw std::invalid_argument("invalid RANSAC parameters");}
  if (!std::isfinite(params.confidence) || params.confidence <= 0.0 || params.confidence >= 1.0) {throw std::invalid_argument("confidence must be in (0,1)");}
  const Points points = canonical_order(input); Points search = points;
  if (params.max_search_points >= 3U && points.size() > params.max_search_points) {
    search.clear(); search.reserve(params.max_search_points);
    for (std::size_t i = 0; i < params.max_search_points; ++i) {
      search.push_back(points[static_cast<std::size_t>(std::floor(static_cast<double>(i) * (points.size() - 1U) / (params.max_search_points - 1U)))]);
    }
  }
  PlaneFitResult result; result.diagnostics.input_points = points.size(); result.diagnostics.search_points = search.size();
  NumpyPcg64 generator(params.seed);
  std::size_t best_count = 0U, required = params.num_iterations;
  for (std::size_t trial = 0; trial < params.num_iterations && trial < required; ++trial) {
    const auto first = static_cast<std::size_t>(generator.bounded_uint32(static_cast<std::uint32_t>(search.size()))), second = static_cast<std::size_t>(generator.bounded_uint32(static_cast<std::uint32_t>(search.size()))), third = static_cast<std::size_t>(generator.bounded_uint32(static_cast<std::uint32_t>(search.size()))); ++result.diagnostics.trials_evaluated;
    if (first == second || first == third || second == third) {continue;}
    bool valid = false; const PlaneModel candidate = plane_from_triplet(search[first], search[second], search[third], &valid);
    if (!valid) {continue;} ++result.diagnostics.valid_trials; std::size_t count = 0U;
    for (const auto & p : search) {if (std::abs(candidate.normal.dot(p) + candidate.offset) <= params.distance_threshold) {++count;}}
    if (count <= best_count) {continue;} best_count = count; result.plane = candidate;
    const double ratio = static_cast<double>(count) / static_cast<double>(search.size());
    if (ratio >= 1.0) {required = trial + 1U;} else if (ratio > 0.0) {
      const double denominator = std::log1p(-(ratio * ratio * ratio));
      if (denominator < 0.0) {required = std::min(params.num_iterations, trial + 1U + static_cast<std::size_t>(std::ceil(std::log1p(-params.confidence) / denominator)));}
    }
  }
  if (best_count < 3U) {throw std::runtime_error("RANSAC found no plane");}
  for (std::size_t i = 0; i < points.size(); ++i) {if (std::abs(result.plane.normal.dot(points[i]) + result.plane.offset) <= params.distance_threshold) {result.inlier_indices.push_back(i);}}
  if (result.inlier_indices.size() < 3U) {throw std::runtime_error("RANSAC full score has fewer than 3 inliers");}
  Point centroid = Point::Zero(); for (const auto i : result.inlier_indices) {centroid += points[i];} centroid /= static_cast<double>(result.inlier_indices.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(); for (const auto i : result.inlier_indices) {const Point d = points[i] - centroid; covariance.noalias() += d * d.transpose();}
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance); if (solver.info() != Eigen::Success) {throw std::runtime_error("least-squares plane refit failed");}
  Point normal = solver.eigenvectors().col(0); if (normal.dot(result.plane.normal) < 0.0) {normal = -normal;} const PlaneModel refit{normal, -normal.dot(centroid)};
  std::vector<std::size_t> refined; for (std::size_t i = 0; i < points.size(); ++i) {if (std::abs(refit.normal.dot(points[i]) + refit.offset) <= params.distance_threshold) {refined.push_back(i);}}
  if (refined.size() >= result.inlier_indices.size()) {result.plane = refit; result.inlier_indices = std::move(refined);} return result;
}

inline LocalGroundModel make_local_ground_model(const Points & points, const GroundPlane & plane, const GroundParameters & params)
{
  LocalGroundModel local; local.normal = plane.normal; local.offset = plane.offset; local.source = plane.source; local.cell_size = params.local_cell_size;
  struct Sample {long long x, y; Point point;}; std::vector<Sample> samples; samples.reserve(points.size());
  for (const auto & p : points) {samples.push_back({static_cast<long long>(std::floor(p.x() / params.local_cell_size)), static_cast<long long>(std::floor(p.y() / params.local_cell_size)), p});}
  std::sort(samples.begin(), samples.end(), [](const Sample & a, const Sample & b) {return a.x == b.x ? a.y < b.y : a.x < b.x;});
  for (std::size_t begin = 0; begin < samples.size();) {
    std::size_t end = begin + 1U; while (end < samples.size() && samples[end].x == samples[begin].x && samples[end].y == samples[begin].y) {++end;}
    if (end - begin >= 3U) {std::vector<double> residuals; for (std::size_t i = begin; i < end; ++i) {residuals.push_back(samples[i].point.z() - plane.support_z(samples[i].point));}
      const double low = percentile_linear(residuals, 10.0); if (low <= params.thickness) {const Eigen::Vector2d center((samples[begin].x + 0.5) * params.local_cell_size, (samples[begin].y + 0.5) * params.local_cell_size); local.cell_xy.push_back(center); local.cell_z.push_back(plane.support_z(Point(center.x(), center.y(), 0.0)) + low);}}
    begin = end;
  }
  return local;
}

inline GroundRemovalResult remove_ground(
  const Points & points, const GroundParameters & params,
  const Points * fitting_points = nullptr)
{
  if (points.empty()) {return {};}
  std::vector<double> zs; for (const auto & p : points) {zs.push_back(p.z());}
  const auto fallback = [&]() {GroundRemovalResult output; output.ground.normal = Point::UnitZ(); output.ground.offset = -percentile_linear(zs, params.z_percentile); output.ground.source = "percentile"; output.ground.cell_size = params.local_cell_size; for (const auto & p : points) {if (output.ground.height(p) >= params.thickness) {output.above_ground.push_back(p);}} return output;};
  const Points & fit_points = fitting_points == nullptr ? points : *fitting_points;
  if (fit_points.size() < 3U) {return fallback();}
  try {RansacParameters ransac; ransac.distance_threshold = params.ransac_distance; ransac.num_iterations = params.ransac_iterations; ransac.seed = params.ransac_seed; const auto fit = segment_plane(fit_points, ransac); PlaneModel model = fit.plane;
    if (std::abs(model.normal.z()) < params.normal_min_z) {return fallback();} if (model.normal.z() < 0.0) {model.normal = -model.normal; model.offset = -model.offset;}
    GroundRemovalResult output; output.ground = make_local_ground_model(points, GroundPlane{model.normal, model.offset, "ransac"}, params); for (const auto & p : points) {if (output.ground.height(p) >= params.local_clearance) {output.above_ground.push_back(p);}} return output;
  } catch (const std::exception &) {return fallback();}
}
}  // namespace concrete_block_detector::detector_core

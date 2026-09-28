#pragma once
#include <algorithm>
#include <cstddef>

#include "concrete_block_detector/blockpose_core_compat.hpp"

namespace concrete_block_detector
{

// Scene discovery is gated for blocks resting on the ground: cluster_max_center_z drops a
// block hanging in the gripper at 2.5 m, in the post-refinement height filter, after the
// prior has already seeded and refined it. Measured on the carried-high captures (i176 and
// re-run for i178): raising the ceiling alone recovers all 8 frames at 0.070 m / <=2.7 deg
// from the FK prediction, where the shipped ceiling returns nothing. cluster_min_extent_z is
// deliberately left alone -- the seeded path never goes through the cluster proposal gates,
// so relaxing it buys this case nothing and only widens the scene's candidate surface.
//
// The relaxation is scoped to the call that asked for it: only the first
// `request_prior_count` priors come from the request, the rest the node adds from FK on every
// cloud, and a high gripper must not quietly raise the ceiling of an ordinary scene
// discovery. Source "fk" is the same label the detector core requires before a prior may seed
// a hypothesis, so it marks exactly the request that needs an off-the-ground gate. The
// trigger compares the prior's world z against a gate on height above ground -- the ground
// fit only exists once detection has run, and the two agree to the relief of the site.
inline detector_core::DetectionParameters parameters_for_request(
  const detector_core::DetectionParameters & base,
  const detector_core::PosePriors & priors,
  std::size_t request_prior_count,
  double elevated_cluster_max_center_z)
{
  auto parameters = base;
  const std::size_t count = std::min(request_prior_count, priors.size());
  for (std::size_t index = 0; index < count; ++index) {
    const auto & prior = priors[index];
    if (prior.source != "fk" || prior.weight <= 0.0 ||
      prior.position.z() <= base.cluster_max_center_z)
    {
      continue;
    }
    parameters.cluster_max_center_z =
      std::max(base.cluster_max_center_z, elevated_cluster_max_center_z);
    break;
  }
  return parameters;
}

// A prior may only seed a hypothesis when its dimensions match the detector's block_dims
// exactly, and the two come from different config files in different repositories. A
// mismatch makes REFINE_GRASPED a silent no-op, so the caller reports it.
inline bool request_prior_dims_mismatch(
  const detector_core::DetectionParameters & base,
  const detector_core::PosePriors & priors,
  std::size_t request_prior_count)
{
  const std::size_t count = std::min(request_prior_count, priors.size());
  for (std::size_t index = 0; index < count; ++index) {
    const auto & prior = priors[index];
    if (prior.source == "fk" && prior.weight > 0.0 && prior.dims != base.block_dims) {
      return true;
    }
  }
  return false;
}

}  // namespace concrete_block_detector

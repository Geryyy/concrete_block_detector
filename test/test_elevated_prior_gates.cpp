#include <gtest/gtest.h>

#include <string>

#include "concrete_block_detector/elevated_prior_gates.hpp"

namespace concrete_block_detector
{
namespace
{

detector_core::PosePrior make_prior(const std::string & source, double z)
{
  detector_core::PosePrior prior;
  prior.source = source;
  prior.position = detector_core::Point(1.0, 2.0, z);
  prior.dims = {{0.9, 0.6, 0.6}};
  prior.weight = 0.35;
  return prior;
}

detector_core::DetectionParameters shipped()
{
  detector_core::DetectionParameters parameters;
  parameters.cluster_max_center_z = 1.5;
  parameters.cluster_min_extent_z = 0.3;
  parameters.block_dims = {{0.9, 0.6, 0.6}};
  return parameters;
}

TEST(ElevatedPriorGates, CarriedBlockRequestRaisesOnlyTheHeightCeiling)
{
  const detector_core::PosePriors priors{make_prior("fk", 2.51)};
  const auto parameters = parameters_for_request(shipped(), priors, 1U, 3.0);
  EXPECT_DOUBLE_EQ(parameters.cluster_max_center_z, 3.0);
  EXPECT_DOUBLE_EQ(parameters.cluster_min_extent_z, 0.3);
}

TEST(ElevatedPriorGates, GroundLevelRequestKeepsTheShippedCeiling)
{
  const detector_core::PosePriors priors{make_prior("fk", 0.3)};
  EXPECT_DOUBLE_EQ(
    parameters_for_request(shipped(), priors, 1U, 3.0).cluster_max_center_z, 1.5);
}

// The node appends its own FK prior to every cloud, so a high gripper during an ordinary
// scene discovery would otherwise raise a ceiling that request never asked about.
TEST(ElevatedPriorGates, NodeSideFkPriorAboveTheCeilingDoesNotRelax)
{
  const detector_core::PosePriors priors{make_prior("registered_block:block_0", 0.3),
    make_prior("fk", 2.51)};
  EXPECT_DOUBLE_EQ(
    parameters_for_request(shipped(), priors, 1U, 3.0).cluster_max_center_z, 1.5);
}

TEST(ElevatedPriorGates, DisabledPriorAndNonFkSourceDoNotRelax)
{
  detector_core::PosePriors priors{make_prior("fk", 2.51), make_prior("wall_plan:block_0", 2.51)};
  priors[0].weight = 0.0;
  EXPECT_DOUBLE_EQ(
    parameters_for_request(shipped(), priors, priors.size(), 3.0).cluster_max_center_z, 1.5);
}

// Dimensions come from a different config file in a different repository, and a mismatch
// makes the prior unable to seed without any other symptom.
TEST(ElevatedPriorGates, DimsMismatchIsReported)
{
  detector_core::PosePriors priors{make_prior("fk", 2.51)};
  EXPECT_FALSE(request_prior_dims_mismatch(shipped(), priors, 1U));
  priors[0].dims = {{0.6, 0.9, 0.6}};
  EXPECT_TRUE(request_prior_dims_mismatch(shipped(), priors, 1U));
  // Only the request's own priors are the caller's to get wrong.
  EXPECT_FALSE(request_prior_dims_mismatch(shipped(), priors, 0U));
}

}  // namespace
}  // namespace concrete_block_detector

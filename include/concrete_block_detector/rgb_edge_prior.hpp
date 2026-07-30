#pragma once

// Transitional include for downstream ROS code.  The reusable implementation
// lives in the portable Blockpose vision library.
#include <blockpose/vision/rgb_edge_prior.hpp>

namespace concrete_block_detector
{
using RgbEdgePriorParameters = ::blockpose::vision::RgbEdgePriorParameters;
using RgbEdgePrior = ::blockpose::vision::RgbEdgePrior;
}  // namespace concrete_block_detector

#pragma once

// Transitional source-compatibility layer. The production algorithm lives in
// the portable Blockpose C++ package; this ROS package owns only adapters.
#include <blockpose/core/blockpose_core.hpp>

namespace concrete_block_detector
{
namespace detector_core = ::blockpose::core;
}  // namespace concrete_block_detector

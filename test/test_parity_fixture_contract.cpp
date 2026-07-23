#include <gtest/gtest.h>

#include <fstream>
#include <string>

#ifndef CONCRETE_BLOCK_DETECTOR_PARITY_MANIFEST
#error "CMAKE must provide the parity manifest path"
#endif

namespace
{

TEST(BlockposeParityFixtureContract, HasVersionedManifestAndRequiredScenarios)
{
  std::ifstream manifest(CONCRETE_BLOCK_DETECTOR_PARITY_MANIFEST);
  ASSERT_TRUE(manifest.good()) << CONCRETE_BLOCK_DETECTOR_PARITY_MANIFEST;
  const std::string content(
    (std::istreambuf_iterator<char>(manifest)), std::istreambuf_iterator<char>());

  EXPECT_NE(content.find("\"schema_version\": 1"), std::string::npos);
  for (const char * scenario :
    {"isolated", "top_and_side", "top_only", "tilted_support", "touching_pair", "scatter_and_clutter"})
  {
    EXPECT_NE(content.find(scenario), std::string::npos) << scenario;
  }
  EXPECT_NE(content.find("1783428141_224458752_seq3"), std::string::npos);
  EXPECT_NE(content.find("\"translation_tolerance_m\": 0.05"), std::string::npos);
  EXPECT_NE(content.find("\"rotation_tolerance_deg\": 5.0"), std::string::npos);
}

}  // namespace

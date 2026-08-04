#include <gtest/gtest.h>

#include "../include/cnpy.h"
#include "../include/startup_reference.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

TEST(StartupReference, LoadsFloat32JointAndQuaternionFrames) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "g1_startup_reference_test.npz";
  const std::vector<float> joint_pos(2 * 29, 0.25F);
  const std::vector<float> joint_vel(2 * 29, 0.0F);
  const std::vector<float> body_quat{1.0F, 0.0F, 0.0F, 0.0F,
                                     0.9F, 0.1F, 0.0F, 0.0F};
  cnpy::npz_save(path.string(), "joint_pos", joint_pos.data(), {2, 29}, "w");
  cnpy::npz_save(path.string(), "joint_vel", joint_vel.data(), {2, 29}, "a");
  cnpy::npz_save(path.string(), "body_quat_w", body_quat.data(), {2, 4}, "a");

  const auto motion = startup_reference::Load(path.string());
  ASSERT_NE(motion, nullptr);
  EXPECT_EQ(motion->name, "startup_reference");
  EXPECT_EQ(motion->timesteps, 2);
  EXPECT_EQ(motion->GetEncodeMode(), 0);
  EXPECT_DOUBLE_EQ(motion->JointPositions(0)[0], 0.25);
  EXPECT_DOUBLE_EQ(motion->JointVelocities(1)[28], 0.0);
  EXPECT_NEAR(motion->BodyQuaternions(1)[0][0], 0.9, 1e-6);

  std::filesystem::remove(path);
}

TEST(StartupReference, LoadsPreparedFaceeCatalogAsset) {
  const char* path = std::getenv("FACEE_STARTUP_REFERENCE_TEST_NPZ");
  if (path == nullptr || std::string(path).empty()) {
    GTEST_SKIP() << "FACEE_STARTUP_REFERENCE_TEST_NPZ is not set";
  }

  const auto motion = startup_reference::Load(path);
  ASSERT_NE(motion, nullptr);
  EXPECT_EQ(motion->timesteps, 650);
  EXPECT_EQ(motion->GetNumJoints(), 29);
  EXPECT_EQ(motion->GetNumBodyQuaternions(), 1);
  EXPECT_NEAR(motion->JointPositions(0)[0], 0.25924626, 1e-6);
  EXPECT_NEAR(motion->BodyQuaternions(0)[0][0], 0.99081236, 1e-6);
}

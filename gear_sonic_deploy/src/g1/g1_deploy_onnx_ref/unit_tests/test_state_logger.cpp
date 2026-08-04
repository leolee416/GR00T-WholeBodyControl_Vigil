#include <gtest/gtest.h>

#include "../include/state_logger.hpp"

#include <array>
#include <span>
#include <vector>

namespace {

void LogState(StateLogger& logger, double value, double action) {
  const std::array<double, 4> quat{1.0, 0.0, 0.0, 0.0};
  const std::array<double, 3> vec{value, value + 1.0, value + 2.0};
  std::vector<double> q{value, value + 0.25};
  std::vector<double> dq{value + 0.5, value + 0.75};
  std::vector<double> last_action{action, action};
  std::vector<double> temperature(4, 0.0);
  std::vector<double> motor(2, 0.0);
  std::vector<double> hand(7, 0.0);
  logger.LogFullState(
      quat, vec, vec, quat, vec, vec, std::span<double>(q),
      std::span<double>(dq), std::span<double>(last_action),
      std::span<double>(temperature), std::span<double>(motor),
      std::span<double>(motor), std::span<double>(hand),
      std::span<double>(hand), std::span<double>(hand),
      std::span<double>(hand), std::span<double>(hand),
      std::span<double>(hand));
}

}  // namespace

TEST(StateLogger, FirstSampleFillsTenFrameHistory) {
  StateLogger logger("", 32, 2, 2, 0.02, false);
  LogState(logger, 3.0, 0.0);

  const auto history = logger.GetLatest(10, 0.02, false);
  ASSERT_EQ(history.size(), 10U);
  for (const auto& entry : history) {
    ASSERT_EQ(entry.body_q.size(), 2U);
    EXPECT_DOUBLE_EQ(entry.body_q[0], 3.0);
    EXPECT_DOUBLE_EQ(entry.body_q[1], 3.25);
    ASSERT_EQ(entry.last_action.size(), 2U);
    EXPECT_DOUBLE_EQ(entry.last_action[0], 0.0);
    EXPECT_DOUBLE_EQ(entry.last_action[1], 0.0);
  }
}

TEST(StateLogger, PartialHistoryPadsWithEarliestRealSample) {
  StateLogger logger("", 32, 2, 2, 0.02, false);
  LogState(logger, 1.0, 0.0);
  LogState(logger, 2.0, 1.0);
  LogState(logger, 3.0, 2.0);

  const auto history = logger.GetLatest(5, false);
  ASSERT_EQ(history.size(), 5U);
  EXPECT_DOUBLE_EQ(history[0].body_q[0], 1.0);
  EXPECT_DOUBLE_EQ(history[1].body_q[0], 1.0);
  EXPECT_DOUBLE_EQ(history[2].body_q[0], 1.0);
  EXPECT_DOUBLE_EQ(history[3].body_q[0], 2.0);
  EXPECT_DOUBLE_EQ(history[4].body_q[0], 3.0);
}

TEST(StateLogger, ResetHistoryPreventsStaleResumeSamples) {
  StateLogger logger("", 32, 2, 2, 0.02, false);
  LogState(logger, 1.0, 0.0);
  LogState(logger, 2.0, 1.0);
  logger.ResetHistory();
  EXPECT_EQ(logger.size(), 0U);

  LogState(logger, 7.0, 0.0);
  const auto history = logger.GetLatest(10, 0.02, false);
  ASSERT_EQ(history.size(), 10U);
  for (const auto& entry : history) {
    EXPECT_DOUBLE_EQ(entry.body_q[0], 7.0);
    EXPECT_DOUBLE_EQ(entry.last_action[0], 0.0);
  }
}

TEST(StateLogger, ExplicitReferenceEntryPadsMissingStartupHistory) {
  StateLogger logger("", 32, 2, 2, 0.02, false);
  StateLogger::Entry reference;
  reference.base_quat = {1.0, 0.0, 0.0, 0.0};
  reference.body_q = {4.0, 4.25};
  reference.body_dq = {0.0, 0.0};
  reference.last_action = {0.0, 0.0};
  logger.ResetHistory(reference);

  LogState(logger, 7.0, 1.0);
  const auto history = logger.GetLatest(10, 0.02, false);
  ASSERT_EQ(history.size(), 10U);
  for (size_t i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(history[i].body_q[0], 4.0);
    EXPECT_DOUBLE_EQ(history[i].body_dq[0], 0.0);
    EXPECT_DOUBLE_EQ(history[i].last_action[0], 0.0);
  }
  EXPECT_DOUBLE_EQ(history[9].body_q[0], 7.0);
  EXPECT_DOUBLE_EQ(history[9].last_action[0], 1.0);
}

#include "state_logger.hpp"

#include <array>
#include <cassert>
#include <iostream>
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

int main() {
  StateLogger first_tick("", 32, 2, 2, 0.02, false);
  LogState(first_tick, 3.0, 0.0);
  const auto repeated = first_tick.GetLatest(10, 0.02, false);
  assert(repeated.size() == 10U);
  for (const auto& entry : repeated) {
    assert(entry.body_q == std::vector<double>({3.0, 3.25}));
    assert(entry.last_action == std::vector<double>({0.0, 0.0}));
  }

  StateLogger partial("", 32, 2, 2, 0.02, false);
  LogState(partial, 1.0, 0.0);
  LogState(partial, 2.0, 1.0);
  LogState(partial, 3.0, 2.0);
  const auto history = partial.GetLatest(5, false);
  assert(history.size() == 5U);
  const std::array<double, 5> expected{1.0, 1.0, 1.0, 2.0, 3.0};
  for (size_t index = 0; index < expected.size(); ++index) {
    assert(history[index].body_q[0] == expected[index]);
  }

  partial.ResetHistory();
  assert(partial.size() == 0U);
  LogState(partial, 7.0, 0.0);
  const auto resumed = partial.GetLatest(10, 0.02, false);
  assert(resumed.size() == 10U);
  for (const auto& entry : resumed) {
    assert(entry.body_q == std::vector<double>({7.0, 7.25}));
    assert(entry.last_action == std::vector<double>({0.0, 0.0}));
  }
  std::cout << "PASS state_logger repeat-reset startup history" << std::endl;
  return 0;
}

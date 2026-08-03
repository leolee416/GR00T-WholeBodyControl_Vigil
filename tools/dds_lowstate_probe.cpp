// Read-only DDS smoke probe for repository-mode MuJoCo validation.
//
// This executable never publishes rt/lowcmd.  It only counts G1 LowState
// messages so DDS compatibility can be checked before launching a policy.

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;
using unitree_hg::msg::dds_::LowState_;

int main(int argc, char** argv) {
  const std::string interface = argc > 1 ? argv[1] : "lo";
  const int duration_seconds = argc > 2 ? std::atoi(argv[2]) : 5;
  std::atomic<std::uint64_t> count{0};

  ChannelFactory::Instance()->Init(0, interface);
  ChannelSubscriber<LowState_> subscriber("rt/lowstate");
  subscriber.InitChannel(
      [&count](const void* message) {
        const auto* state = static_cast<const LowState_*>(message);
        const auto sample = ++count;
        if (sample <= 3) {
          std::cout << "RX " << sample << " tick=" << state->tick()
                    << " q0=" << state->motor_state().at(0).q() << std::endl;
        }
      },
      10);

  std::this_thread::sleep_for(std::chrono::seconds(duration_seconds));
  std::cout << "COUNT " << count.load() << std::endl;
  return count.load() > 0 ? 0 : 2;
}

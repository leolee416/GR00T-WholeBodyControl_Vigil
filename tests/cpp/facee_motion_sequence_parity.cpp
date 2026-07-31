#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion_data_reader.hpp"
#include "input_interface/streamed_motion_merger.hpp"

namespace {

template <typename T>
T ReadScalar(const std::vector<char>& bytes, std::size_t& offset) {
    if (offset + sizeof(T) > bytes.size()) {
        throw std::runtime_error("truncated protocol-v1 payload");
    }
    T value{};
    std::memcpy(&value, bytes.data() + offset, sizeof(T));
    offset += sizeof(T);
    return value;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: probe PAYLOAD OUTPUT FRAME_COUNT\n";
        return 2;
    }
    const int frame_count = std::stoi(argv[3]);
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) {
        throw std::runtime_error("cannot open payload");
    }
    const std::vector<char> bytes(
        (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>()
    );
    std::size_t offset = 0;
    StreamedMotionMerger::IncomingData incoming;
    incoming.protocol_version = 1;
    incoming.catch_up_enabled = true;
    incoming.num_frames = frame_count;
    incoming.num_joints = 29;
    incoming.num_quat_bodies = 1;
    incoming.joint_pos.assign(frame_count, std::vector<double>(29));
    incoming.joint_vel.assign(frame_count, std::vector<double>(29));
    incoming.body_quat.assign(
        frame_count, std::vector<std::array<double, 4>>(1)
    );
    incoming.frame_indices.resize(frame_count);

    for (int frame = 0; frame < frame_count; ++frame) {
        for (int joint = 0; joint < 29; ++joint) {
            incoming.joint_pos[frame][joint] =
                static_cast<double>(ReadScalar<float>(bytes, offset));
        }
    }
    for (int frame = 0; frame < frame_count; ++frame) {
        for (int joint = 0; joint < 29; ++joint) {
            incoming.joint_vel[frame][joint] =
                static_cast<double>(ReadScalar<float>(bytes, offset));
        }
    }
    for (int frame = 0; frame < frame_count; ++frame) {
        for (int component = 0; component < 4; ++component) {
            incoming.body_quat[frame][0][component] =
                static_cast<double>(ReadScalar<float>(bytes, offset));
        }
    }
    static_cast<void>(ReadScalar<std::int32_t>(bytes, offset));  // encode_mode
    static_cast<void>(ReadScalar<std::int32_t>(bytes, offset));  // motion_id
    for (int frame = 0; frame < frame_count; ++frame) {
        incoming.frame_indices[frame] = ReadScalar<std::int64_t>(bytes, offset);
    }
    static_cast<void>(ReadScalar<std::uint8_t>(bytes, offset));  // catch_up
    if (offset != bytes.size()) {
        throw std::runtime_error("unexpected trailing protocol-v1 payload bytes");
    }

    StreamedMotionMerger merger;
    const auto result = merger.MergeIncomingData(incoming, 0);
    if (!result.motion || result.motion->timesteps != frame_count) {
        throw std::runtime_error("C++ streamed-motion merge failed");
    }

    std::ofstream output(argv[2], std::ios::binary);
    if (!output) {
        throw std::runtime_error("cannot open output");
    }
    for (int frame = 0; frame < 50; frame += 5) {
        output.write(
            reinterpret_cast<const char*>(result.motion->JointPositions(frame)),
            29 * sizeof(double)
        );
    }
    for (int frame = 0; frame < 50; frame += 5) {
        output.write(
            reinterpret_cast<const char*>(result.motion->JointVelocities(frame)),
            29 * sizeof(double)
        );
    }
    return output ? 0 : 3;
}

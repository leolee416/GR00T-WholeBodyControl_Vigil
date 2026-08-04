/**
 * @file startup_reference.hpp
 * @brief Strict loader for an opt-in startup reference stored as an NPZ file.
 */

#pragma once

#include "cnpy.h"
#include "motion_data_reader.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

namespace startup_reference {

inline void ValidateArray(const cnpy::NpyArray& array,
                          const std::string& name,
                          size_t frame_count,
                          size_t width) {
  if (array.shape.size() != 2 || array.shape[0] != frame_count ||
      array.shape[1] != width) {
    throw std::runtime_error(
        "startup reference " + name + " must have shape [" +
        std::to_string(frame_count) + ", " + std::to_string(width) + "]");
  }
  if (array.fortran_order) {
    throw std::runtime_error("startup reference " + name + " must use C order");
  }
  if (array.word_size != sizeof(float) && array.word_size != sizeof(double)) {
    throw std::runtime_error("startup reference " + name + " must be float32 or float64");
  }
}

inline double ReadValue(const cnpy::NpyArray& array, size_t index) {
  if (array.word_size == sizeof(float)) {
    return static_cast<double>(array.data<float>()[index]);
  }
  return array.data<double>()[index];
}

inline std::shared_ptr<MotionSequence> Load(const std::string& path) {
  const cnpy::NpyArray joint_pos = cnpy::npz_load(path, "joint_pos");
  if (joint_pos.shape.size() != 2 || joint_pos.shape[0] == 0 ||
      joint_pos.shape[1] != 29) {
    throw std::runtime_error("startup reference joint_pos must have shape [frames, 29]");
  }
  const size_t frame_count = joint_pos.shape[0];
  ValidateArray(joint_pos, "joint_pos", frame_count, 29);

  const cnpy::NpyArray joint_vel = cnpy::npz_load(path, "joint_vel");
  const cnpy::NpyArray body_quat = cnpy::npz_load(path, "body_quat_w");
  ValidateArray(joint_vel, "joint_vel", frame_count, 29);
  ValidateArray(body_quat, "body_quat_w", frame_count, 4);

  auto motion = std::make_shared<MotionSequence>();
  motion->name = "startup_reference";
  motion->timesteps = static_cast<int>(frame_count);
  motion->SetEncodeMode(0);
  motion->SetBodyPartIndexes({0});
  motion->ReserveCapacity(motion->timesteps, 29, 1, 1, 0, 0);

  for (size_t frame = 0; frame < frame_count; ++frame) {
    for (size_t joint = 0; joint < 29; ++joint) {
      const size_t index = frame * 29 + joint;
      motion->JointPositions(static_cast<int>(frame))[joint] = ReadValue(joint_pos, index);
      motion->JointVelocities(static_cast<int>(frame))[joint] = ReadValue(joint_vel, index);
    }
    for (size_t component = 0; component < 4; ++component) {
      motion->BodyQuaternions(static_cast<int>(frame))[0][component] =
          ReadValue(body_quat, frame * 4 + component);
    }
  }
  return motion;
}

}  // namespace startup_reference

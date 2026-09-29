//
// Created by qiayuan on 2022/7/26.
//

#pragma once

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace humanoid {
using namespace centroidal_model;
class SafetyChecker {
 public:
  explicit SafetyChecker(const CentroidalModelInfo& info) : info_(info) {}

  bool check(const SystemObservation& observation, const vector_t& /*optimized_state*/, const vector_t& /*optimized_input*/) {
    return checkOrientation(observation);
  }

 protected:
  bool checkOrientation(const SystemObservation& observation) {
    vector_t pose = getBasePose(observation.state, info_);
    // 基座位姿顺序为 [x, y, z, yaw, pitch, roll]，人形最常见失稳模式是前倾/后仰（pitch），
    // 因此 pitch 必须检查；阈值取 ±45°，roll 保持 ±90°。
    constexpr scalar_t pitchLimit = M_PI_4;
    constexpr scalar_t rollLimit = M_PI_2;
    if (pose(4) > pitchLimit || pose(4) < -pitchLimit) {
      std::cerr << "[SafetyChecker] Pitch safety check failed! pitch = " << pose(4) << std::endl;
      return false;
    }
    if (pose(5) > rollLimit || pose(5) < -rollLimit) {
      std::cerr << "[SafetyChecker] Roll safety check failed! roll = " << pose(5) << std::endl;
      return false;
    }
    return true;
  }

  const CentroidalModelInfo& info_;
};

}  // namespace humanoid
}  // namespace ocs2

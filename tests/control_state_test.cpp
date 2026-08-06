#include <cassert>
#include <cmath>
#include <iostream>

#include "control_state.h"

namespace {
constexpr float kTolerance = 1e-4f;

struct Quaternion {
  float x, y, z, w;
};

Quaternion fromEuler(float roll, float pitch, float yaw) {
  const float cr = std::cos(roll * .5f), sr = std::sin(roll * .5f);
  const float cp = std::cos(pitch * .5f), sp = std::sin(pitch * .5f);
  const float cy = std::cos(yaw * .5f), sy = std::sin(yaw * .5f);
  return {
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy,
  };
}
}  // namespace

int main() {
  control_state::State state;

  // Reproduce the level attitude reported by the installed BNO08X. It must
  // become the neutral pose even though its raw standard roll is near -pi.
  const float startRoll = -3.0927f;
  const float startPitch = -0.0099f;
  const float startYaw = -0.6844f;
  const Quaternion start = fromEuler(startRoll, startPitch, startYaw);
  state.ingestRotation(1'000'000, 1'000'000, 3, start.x, start.y, start.z,
                       start.w);
  auto snapshot = state.snapshot(1'000'000);
  assert(std::fabs(snapshot.rollRad) < kTolerance);
  assert(std::fabs(snapshot.pitchRad) < kTolerance);
  assert(std::fabs(snapshot.yawRad) < kTolerance);

  // Later values are relative to that program-start pose in the established
  // boat convention: roll=standard pitch, pitch=-standard roll.
  const Quaternion moved =
      fromEuler(startRoll - .10f, startPitch + .05f, startYaw + .20f);
  state.ingestRotation(1'100'000, 1'100'000, 3, moved.x, moved.y, moved.z,
                       moved.w);
  snapshot = state.snapshot(1'100'000);
  assert(std::fabs(snapshot.rollRad - .05f) < kTolerance);
  assert(std::fabs(snapshot.pitchRad - .10f) < kTolerance);
  assert(std::fabs(snapshot.yawRad - .20f) < kTolerance);

  std::cout << "control state tests passed\n";
}

// Differential-drive kinematics and wheel-encoder odometry.
//
// Author: Akram Hssaine (Enkefalos) - MIT License
#pragma once

#include <cmath>

namespace diffbot {

struct Pose {
  float x = 0.0f;      // m
  float y = 0.0f;      // m
  float theta = 0.0f;  // rad, wrapped to [-pi, pi]
};

struct WheelSpeeds {
  float left = 0.0f;   // m/s
  float right = 0.0f;  // m/s
};

inline float wrapAngle(float a) {
  const float pi = 3.14159265358979f;
  while (a > pi) a -= 2 * pi;
  while (a < -pi) a += 2 * pi;
  return a;
}

class DiffDrive {
 public:
  explicit DiffDrive(float wheel_base) : wheel_base_(wheel_base) {}

  // Body twist (v, omega) -> wheel speeds.
  WheelSpeeds inverse(float v, float omega) const {
    return {v - omega * wheel_base_ / 2.0f, v + omega * wheel_base_ / 2.0f};
  }

  // Wheel speeds -> body twist.
  void forward(const WheelSpeeds& w, float& v, float& omega) const {
    v = (w.left + w.right) / 2.0f;
    omega = (w.right - w.left) / wheel_base_;
  }

  // Integrate wheel travel (m) since the last call into the pose
  // using the exact arc model (midpoint for straight motion).
  void integrate(Pose& p, float d_left, float d_right) const {
    const float ds = (d_left + d_right) / 2.0f;
    const float dth = (d_right - d_left) / wheel_base_;
    if (std::fabs(dth) < 1e-6f) {
      p.x += ds * std::cos(p.theta);
      p.y += ds * std::sin(p.theta);
    } else {
      const float r = ds / dth;
      p.x += r * (std::sin(p.theta + dth) - std::sin(p.theta));
      p.y -= r * (std::cos(p.theta + dth) - std::cos(p.theta));
    }
    p.theta = wrapAngle(p.theta + dth);
  }

  float wheelBase() const { return wheel_base_; }

 private:
  float wheel_base_;
};

}  // namespace diffbot

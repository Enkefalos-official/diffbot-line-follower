// Weighted-average position estimate from an IR reflectance sensor array
// (e.g. QTR-8A / TCRT5000 x N). Returns line offset in [-1, 1].
//
// Author: Akram Hssaine (Enkefalos) - MIT License
#pragma once

#include <cstddef>

namespace diffbot {

template <std::size_t N>
class LineSensor {
 public:
  // readings: 0 = white, 1 = black (already normalised/calibrated)
  // Returns true and writes `offset` if the line is visible.
  bool estimate(const float (&readings)[N], float& offset) {
    float sum = 0.0f, weighted = 0.0f;
    for (std::size_t i = 0; i < N; ++i) {
      const float pos = (N == 1) ? 0.0f : -1.0f + 2.0f * float(i) / float(N - 1);
      sum += readings[i];
      weighted += readings[i] * pos;
    }
    if (sum < threshold_) {
      // Line lost: remember which side it disappeared on.
      offset = last_ > 0 ? 1.0f : -1.0f;
      return false;
    }
    last_ = weighted / sum;
    offset = last_;
    return true;
  }

  void setThreshold(float t) { threshold_ = t; }

 private:
  float threshold_ = 0.2f;
  float last_ = 0.0f;
};

}  // namespace diffbot

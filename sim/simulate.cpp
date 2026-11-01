// Desktop simulator: a differential-drive robot follows a curved track
// using the same PID + line-sensor code that runs on the microcontroller.
//
// Output: CSV (t,x,y,theta,offset) on stdout, summary on stderr.
//   ./diffbot_sim > run.csv
//   python sim/plot.py run.csv
//
// Author: Akram Hssaine (Enkefalos) - MIT License
#include <cmath>
#include <cstdio>
#include <random>

#include "diffbot/line_sensor.hpp"
#include "diffbot/odometry.hpp"
#include "diffbot/pid.hpp"

using namespace diffbot;

namespace {

constexpr int kSensors = 8;
constexpr float kSensorSpacing = 0.01f;  // 1 cm between IR sensors
constexpr float kSensorAhead = 0.06f;    // array is 6 cm in front of axle
constexpr float kLineHalfWidth = 0.009f; // 18 mm electrical tape

// Track: a sine wave y = A sin(kx). Returns signed distance of point to the line
// (approximate, good enough for gentle curves).
float trackDistance(float x, float y) {
  const float A = 0.25f, k = 2.0f;
  const float yl = A * std::sin(k * x);
  const float slope = A * k * std::cos(k * x);
  return (y - yl) / std::sqrt(1.0f + slope * slope);
}

}  // namespace

int main() {
  DiffDrive drive(0.12f);  // 12 cm wheel base
  LineSensor<kSensors> sensor;
  PID pid(2.2f, 0.4f, 0.12f, -6.0f, 6.0f);  // output: angular velocity (rad/s)

  std::mt19937 rng(42);
  std::normal_distribution<float> noise(0.0f, 0.03f);

  Pose pose;
  pose.y = 0.03f;  // start slightly off the line
  const float v = 0.25f;  // m/s
  const float dt = 0.01f;
  const float max_wheel = 0.6f;

  float sum_sq = 0.0f;
  int steps = 0, lost = 0;

  std::printf("t,x,y,theta,offset\n");
  for (float t = 0.0f; t < 12.0f; t += dt) {
    // --- sense ---
    float readings[kSensors];
    const float cx = pose.x + kSensorAhead * std::cos(pose.theta);
    const float cy = pose.y + kSensorAhead * std::sin(pose.theta);
    for (int i = 0; i < kSensors; ++i) {
      // Sensor 0 is on the robot's right, N-1 on its left.
      const float lat = (i - (kSensors - 1) / 2.0f) * kSensorSpacing;
      const float sx = cx - lat * std::sin(pose.theta);
      const float sy = cy + lat * std::cos(pose.theta);
      const float d = std::fabs(trackDistance(sx, sy));
      float r = d < kLineHalfWidth ? 1.0f : std::exp(-(d - kLineHalfWidth) * 300.0f);
      r += noise(rng);
      readings[i] = r < 0 ? 0 : (r > 1 ? 1 : r);
    }
    float offset;
    if (!sensor.estimate(readings, offset)) ++lost;

    // --- control --- (positive offset = line on the left -> turn left)
    const float omega = -pid.update(0.0f, offset, dt);
    WheelSpeeds w = drive.inverse(v, omega);
    if (w.left > max_wheel) w.left = max_wheel;
    if (w.left < -max_wheel) w.left = -max_wheel;
    if (w.right > max_wheel) w.right = max_wheel;
    if (w.right < -max_wheel) w.right = -max_wheel;

    // --- act / physics ---
    drive.integrate(pose, w.left * dt, w.right * dt);

    const float err = trackDistance(pose.x, pose.y);
    sum_sq += err * err;
    ++steps;
    std::printf("%.2f,%.4f,%.4f,%.4f,%.3f\n", t, pose.x, pose.y, pose.theta, offset);
  }

  const float rms_mm = std::sqrt(sum_sq / steps) * 1000.0f;
  std::fprintf(stderr, "distance %.2f m | RMS tracking error %.1f mm | line lost %d/%d steps\n",
               pose.x, rms_mm, lost, steps);
  return rms_mm < 20.0f ? 0 : 1;  // non-zero exit if tracking is poor
}

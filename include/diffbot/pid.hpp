// PID controller with anti-windup, derivative-on-measurement and output limits.
// Header-only, no dynamic allocation: runs on Arduino/ESP32 and on desktop.
//
// Author: Akram Hssaine (Enkefalos) - MIT License
#pragma once

namespace diffbot {

class PID {
 public:
  PID(float kp, float ki, float kd, float out_min = -1.0f, float out_max = 1.0f)
      : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max) {}

  // Compute the control output. dt in seconds.
  float update(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return last_out_;
    const float error = setpoint - measurement;

    // Derivative on measurement avoids "derivative kick" when setpoint jumps.
    float d = 0.0f;
    if (has_prev_) d = -(measurement - prev_meas_) / dt;
    prev_meas_ = measurement;
    has_prev_ = true;

    // Tentative integral; only keep it if output is not saturated (anti-windup).
    const float new_integral = integral_ + error * dt;
    float out = kp_ * error + ki_ * new_integral + kd_ * d;

    if (out > out_max_) {
      out = out_max_;
      if (error < 0) integral_ = new_integral;   // allow unwinding
    } else if (out < out_min_) {
      out = out_min_;
      if (error > 0) integral_ = new_integral;
    } else {
      integral_ = new_integral;
    }
    last_out_ = out;
    return out;
  }

  void reset() {
    integral_ = 0.0f;
    has_prev_ = false;
    last_out_ = 0.0f;
  }

  void setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  float integral() const { return integral_; }

 private:
  float kp_, ki_, kd_;
  float out_min_, out_max_;
  float integral_ = 0.0f;
  float prev_meas_ = 0.0f;
  float last_out_ = 0.0f;
  bool has_prev_ = false;
};

}  // namespace diffbot

// Minimal dependency-free unit tests.
#include <cmath>
#include <cstdio>

#include "diffbot/line_sensor.hpp"
#include "diffbot/odometry.hpp"
#include "diffbot/pid.hpp"

using namespace diffbot;

static int failures = 0;
#define CHECK(cond)                                                    \
  do {                                                                 \
    if (!(cond)) {                                                     \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
      ++failures;                                                      \
    }                                                                  \
  } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) < (tol))

static void testPidProportional() {
  PID pid(2.0f, 0.0f, 0.0f, -10, 10);
  NEAR(pid.update(1.0f, 0.0f, 0.01f), 2.0f, 1e-6f);
}

static void testPidSaturationAndAntiWindup() {
  PID pid(1.0f, 10.0f, 0.0f, -1, 1);
  for (int i = 0; i < 1000; ++i) CHECK(pid.update(5.0f, 0.0f, 0.01f) <= 1.0f);
  CHECK(pid.integral() < 1.0f);  // integral must not wind up while saturated
}

static void testPidConvergesOnFirstOrderPlant() {
  PID pid(3.0f, 2.0f, 0.0f, -5, 5);
  float y = 0.0f;
  for (int i = 0; i < 2000; ++i) {
    const float u = pid.update(1.0f, y, 0.01f);
    y += (u - y) * 0.01f;  // dy/dt = u - y
  }
  NEAR(y, 1.0f, 0.01f);
}

static void testDiffDriveRoundTrip() {
  DiffDrive d(0.2f);
  WheelSpeeds w = d.inverse(0.5f, 1.0f);
  float v, om;
  d.forward(w, v, om);
  NEAR(v, 0.5f, 1e-6f);
  NEAR(om, 1.0f, 1e-6f);
}

static void testOdometryStraightAndCircle() {
  DiffDrive d(0.2f);
  Pose p;
  d.integrate(p, 1.0f, 1.0f);
  NEAR(p.x, 1.0f, 1e-5f);
  NEAR(p.y, 0.0f, 1e-5f);

  // Full circle of radius 0.5 m in small steps -> back to start.
  Pose c;
  const float R = 0.5f, pi = 3.14159265f;
  const int n = 1000;
  const float arc = 2 * pi * R / n;
  for (int i = 0; i < n; ++i)
    d.integrate(c, arc * (R - 0.1f) / R, arc * (R + 0.1f) / R);
  NEAR(c.x, 0.0f, 1e-3f);
  NEAR(c.y, 0.0f, 1e-3f);
}

static void testLineSensor() {
  LineSensor<5> s;
  float off;
  const float centre[5] = {0, 0, 1, 0, 0};
  CHECK(s.estimate(centre, off));
  NEAR(off, 0.0f, 1e-6f);
  const float left[5] = {0, 0, 0, 0, 1};
  CHECK(s.estimate(left, off));
  NEAR(off, 1.0f, 1e-6f);
  const float none[5] = {0, 0, 0, 0, 0};
  CHECK(!s.estimate(none, off));
  NEAR(off, 1.0f, 1e-6f);  // remembers last side
}

int main() {
  testPidProportional();
  testPidSaturationAndAntiWindup();
  testPidConvergesOnFirstOrderPlant();
  testDiffDriveRoundTrip();
  testOdometryStraightAndCircle();
  testLineSensor();
  if (failures == 0) std::printf("All tests passed\n");
  return failures == 0 ? 0 : 1;
}

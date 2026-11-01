// Line-following robot firmware (Arduino Uno/Nano or ESP32).
// Hardware: 8x IR array (analog), L298N / TB6612 motor driver, 2 DC gear motors.
// Uses the exact same PID + LineSensor code as the desktop simulator.
//
// Copy the include/diffbot folder next to this sketch (or into your
// Arduino libraries folder) before compiling.
//
// Author: Akram Hssaine (Enkefalos) - MIT License
#include "diffbot/line_sensor.hpp"
#include "diffbot/pid.hpp"

using namespace diffbot;

// ---------- Pins (Arduino Uno) ----------
const uint8_t SENSOR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const uint8_t L_PWM = 5, L_IN1 = 7, L_IN2 = 8;
const uint8_t R_PWM = 6, R_IN1 = 9, R_IN2 = 10;
const uint8_t BUTTON = 2;

// ---------- Tuning ----------
const float BASE_SPEED = 0.55f;  // 0..1 of full PWM
PID pid(0.9f, 0.05f, 0.08f, -1.0f, 1.0f);
LineSensor<8> line;

int minVal[8], maxVal[8];

void setMotor(uint8_t pwm, uint8_t in1, uint8_t in2, float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  digitalWrite(in1, speed >= 0);
  digitalWrite(in2, speed < 0);
  analogWrite(pwm, (int)(fabs(speed) * 255));
}

void calibrate() {
  for (int i = 0; i < 8; ++i) { minVal[i] = 1023; maxVal[i] = 0; }
  // Spin in place for ~3 s sweeping the sensors over the line.
  unsigned long start = millis();
  while (millis() - start < 3000) {
    float dir = ((millis() - start) / 750) % 2 ? 1.0f : -1.0f;
    setMotor(L_PWM, L_IN1, L_IN2, 0.35f * dir);
    setMotor(R_PWM, R_IN1, R_IN2, -0.35f * dir);
    for (int i = 0; i < 8; ++i) {
      int v = analogRead(SENSOR_PINS[i]);
      minVal[i] = min(minVal[i], v);
      maxVal[i] = max(maxVal[i], v);
    }
  }
  setMotor(L_PWM, L_IN1, L_IN2, 0);
  setMotor(R_PWM, R_IN1, R_IN2, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_PWM, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  Serial.println("Press button to calibrate");
  while (digitalRead(BUTTON) == HIGH) {}
  calibrate();
  Serial.println("Calibrated. Press button to start");
  delay(300);
  while (digitalRead(BUTTON) == HIGH) {}
}

void loop() {
  static unsigned long last = micros();
  unsigned long now = micros();
  float dt = (now - last) * 1e-6f;
  last = now;

  float r[8];
  for (int i = 0; i < 8; ++i) {
    int range = max(1, maxVal[i] - minVal[i]);
    r[i] = constrain((analogRead(SENSOR_PINS[i]) - minVal[i]) / (float)range, 0.0f, 1.0f);
  }

  float offset;
  bool seen = line.estimate(r, offset);
  float turn = -pid.update(0.0f, offset, dt);
  float base = seen ? BASE_SPEED : 0.3f;  // slow down when the line is lost

  setMotor(L_PWM, L_IN1, L_IN2, base - turn);
  setMotor(R_PWM, R_IN1, R_IN2, base + turn);
}

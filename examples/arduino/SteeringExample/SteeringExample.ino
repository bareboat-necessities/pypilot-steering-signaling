#include <Arduino.h>
#include <pypilot_steering_signaling.hpp>

void setup() {
  Serial.begin(9600);
  pypilot_steering_signaling::RudderCalibration c(0.0f, 512.0f, 1024.0f, -45.0f, 45.0f);
  pypilot_steering_signaling::RudderSample s(768.0f, 1000000ULL);
  pypilot_steering_signaling::RudderAngle a = pypilot_steering_signaling::apply_rudder_calibration(s, c);
  Serial.println(a.angle_deg, 3);
}

void loop() {
}

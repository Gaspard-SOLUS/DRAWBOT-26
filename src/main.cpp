#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

#include "pins.h"
#include "moteurs.h"
#include "encodeurs.h"
#include "logger.h"
#include "sensors.h"
#include "odometry.h"
#include "teleplot.h"
#include "web_server_app.h"
#include "soutenance2.h"
#include "pen_inverse_follower.h"
#include "motor_calibration.h"
#include "compass_arrow.h"

// ==================================================
// TIMERS
// ==================================================
static unsigned long lastControlUpdate = 0;
static unsigned long lastLedToggle = 0;

const unsigned long CONTROL_PERIOD_MS = 20;

void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  Logger::begin();
  Logger::log("=== DRAWBOT - BRANCHE SOUTENANCE 2 ===");

  Wire.begin(SDA, SCL);

  initMotors();
  initEncoders();
  resetEncoders();
  stopMotors();

  Sensors::begin();
  Odometry::begin();
  WebApp::begin();
  Teleplot::begin();

  Soutenance2::begin();
  PenInverseFollower::begin();
  CompassArrow::begin();

  MotorCalibration::begin();

  Logger::log("Initialisation terminee");
}

void loop() {
  unsigned long now = millis();

  if (now - lastControlUpdate >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlUpdate) / 1000.0f;
    lastControlUpdate = now;

    Sensors::update(now, dt);
    Odometry::update(now);
    Soutenance2::update(now, dt);
    CompassArrow::update(now, dt);
    PenInverseFollower::update(now, dt);
    MotorCalibration::update(now);
  }

  WebApp::handleClient();
  Teleplot::update(now);

  if (now - lastLedToggle >= 500) {
    lastLedToggle = now;
    digitalWrite(LEDU1, !digitalRead(LEDU1));
  }
}

#pragma once
#include <Arduino.h>

namespace Sensors {
  void begin();
  void update(unsigned long now, float dt);

  void resetGyroYaw();
  void startMagCalibration();

  float normalizeAngleDeg(float angle);
}
#pragma once
#include <Arduino.h>

namespace Odometry {
  void begin();
  void update(unsigned long now);
  void reset();

  float radToDeg(float rad);
  float normalizeAngleDeg(float angle);
}
#pragma once
#include <Arduino.h>

namespace Odometry {
  void begin();
  void update(unsigned long now);
  void reset();

  void setGeometry(float wheelBaseCm, float penOffsetCm);
  float getWheelBaseCm();
  float getPenOffsetCm();

  void resetPose(float xCm, float yCm, float thetaRad);

  float radToDeg(float rad);
  float normalizeAngleDeg(float angle);
}

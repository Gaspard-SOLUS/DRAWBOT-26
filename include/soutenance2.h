#pragma once
#include <Arduino.h>

namespace Soutenance2 {
  void begin();
  void update(unsigned long now, float dt);

  bool startSpinCircle(float radiusCm, bool clockwise, int pwm, float stopAdvanceDeg);
  bool startWheelDistanceCircle(float radiusCm, bool clockwise, int pwm, float penOffsetCm, float wheelBaseCm, float radiusScale, float shapeLeadDeg);
  void stop();
}

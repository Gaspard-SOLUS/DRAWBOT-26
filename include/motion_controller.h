#pragma once
#include <Arduino.h>

namespace MotionController {

  enum State {
    IDLE = 0,
    STRAIGHT,
    ROT_IN_PLACE,
    ROT_AROUND_PEN,
    ARC
  };

  // Paramètres de calibration
  extern float wheelBaseCm;
  extern float turnWheelBaseCm;
  extern float penOffsetCm;

  extern int pwmMin;
  extern int pwmMax;

  extern float kSync;
  extern float slowdownFraction;

  void init();

  void startMoveStraight(float distanceCm, int pwm);
  void startRotateInPlace(float angleDeg, int pwm);
  void startRotateAroundPen(float angleDeg, int pwm);
  void startArcAtPen(float radiusCm, float angleDeg, bool leftTurn, int pwm);

  void update();

  void stop();
  bool isBusy();
  State getState();
  const char* getStateName();

  float getProgress();
  float getCurrentDistanceCm();
  float getTargetDistanceCm();
}

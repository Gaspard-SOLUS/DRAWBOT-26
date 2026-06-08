#pragma once
#include <Arduino.h>

namespace CompassArrow {
  struct Config {
    float shaftCm = 10.0f;
    float headSideCm = 3.0f;
    float fillStepCm = 0.35f;
    float drawSpeedCms = 8.0f;

    int alignPwm = 180;
    int calibrationPwm = 190;
    bool clockwise = true;

    unsigned long settleMs = 250;
    unsigned long alignTimeoutMs = 15000;
  };

  struct Status {
    bool running = false;
    bool calibrating = false;
    bool drawing = false;
    bool finished = false;

    String phaseName = "IDLE";
    String message = "";

    float headingDeg = 0.0f;
    float northErrorDeg = 0.0f;
    bool inNorthWindow = false;

    float alignInitialHeadingDeg = 0.0f;
    float alignTargetDeg = 0.0f;
    float alignProgressDeg = 0.0f;
    float alignRemainingDeg = 0.0f;

    int pwmLeft = 0;
    int pwmRight = 0;
    int trajectorySegments = 0;
  };

  void begin();
  void update(unsigned long now, float dt);

  bool startCalibration(const Config& config);
  bool startAlignNorth(const Config& config);
  bool startDrawOnly(const Config& config);
  bool startArrow(const Config& config);
  void stop();

  Config defaultConfig();
  Status getStatus();

  float northErrorDeg(float headingDeg);
  bool isInNorthWindow(float headingDeg);
}

#pragma once
#include <Arduino.h>

namespace PenInverseFollower {
  struct Point {
    float x;
    float y;
  };

  struct Segment {
    Point a;
    Point b;
  };

  struct Config {
    float wheelBaseCm = 8.3f;
    float penOffsetCm = 13.0f;

    float penSpeedCms = 4.0f;
    float lineGain = 1.5f;
    float targetGain = 0.8f;
    float lookaheadCm = 2.0f;

    float penSpeedMaxCms = 8.0f;
    float wheelSpeedMaxCms = 10.0f;

    float kp = 0.20f;
    float ki = 0.00f;
    float kd = 0.05f;
    float integralLimit = 10.0f;

    float coefLeftCmsPerPwm = 0.080f;
    float coefRightCmsPerPwm = 0.080f;

    int minPwm = 160;

    float segmentToleranceCm = 0.20f;
  };

  struct Status {
    bool running = false;
    bool finished = false;

    int currentSegment = 0;
    int segmentCount = 0;

    float penX = 0.0f;
    float penY = 0.0f;

    float targetX = 0.0f;
    float targetY = 0.0f;

    float lateralErrorCm = 0.0f;
    float maxLateralErrorCm = 0.0f;

    float progressCm = 0.0f;
    float segmentLengthCm = 0.0f;

    float vPenX = 0.0f;
    float vPenY = 0.0f;

    float vCenterCms = 0.0f;
    float omegaRadS = 0.0f;

    float vLeftCms = 0.0f;
    float vRightCms = 0.0f;

    int pwmLeft = 0;
    int pwmRight = 0;
  };

  void begin();

  void setConfig(const Config& cfg);
  Config getConfig();
  Status getStatus();

  void startLine(float distanceCm);
  void startOneAngle(float d1Cm, float angleDeg, float d2Cm);
  void startStair(float d1Cm, float angleLeftDeg, float d2Cm, float angleRightDeg, float d3Cm);

  void stop();
  void update(unsigned long now, float dt);

  bool isRunning();
  bool isFinished();
}
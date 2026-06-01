#pragma once
#include <Arduino.h>

namespace MotorCalibration {
  struct Config {
    int pwmLeft = 180;
    int pwmRight = 180;
    unsigned long durationMs = 2000;
  };

  struct Result {
    bool running = false;
    bool finished = false;

    int pwmLeft = 0;
    int pwmRight = 0;

    unsigned long durationMs = 0;
    unsigned long elapsedMs = 0;

    float startDistLeftCm = 0.0f;
    float startDistRightCm = 0.0f;

    float endDistLeftCm = 0.0f;
    float endDistRightCm = 0.0f;

    float deltaLeftCm = 0.0f;
    float deltaRightCm = 0.0f;

    float speedLeftCms = 0.0f;
    float speedRightCms = 0.0f;

    float coefLeftCmsPerPwm = 0.0f;
    float coefRightCmsPerPwm = 0.0f;
  };

  void begin();
  void start(const Config& config);
  void stop();
  void update(unsigned long now);

  bool isRunning();
  bool isFinished();

  Result getResult();
  String resultJson();
}

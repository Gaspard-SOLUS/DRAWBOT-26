#pragma once
#include <Arduino.h>

struct MotorState {
  int pwmLeft = 0;
  int pwmRight = 0;
  String mode = "IDLE";
};

struct SensorState {
  bool imuOk = false;
  bool magOk = false;

  float accX = 0.0f;
  float accY = 0.0f;
  float accZ = 0.0f;

  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;

  float yawGyroDeg = 0.0f;

  float magX = 0.0f;
  float magY = 0.0f;
  float magZ = 0.0f;

  float magRawX = 0.0f;
  float magRawY = 0.0f;
  float magRawZ = 0.0f;

  float headingMagDeg = 0.0f;
  float magHeadingDeltaDeg = 0.0f;

  bool magCalibrationRunning = false;
  bool magCalibrationDone = false;
  bool magCalibrationLoaded = false;

  float magOffsetX = 0.0f;
  float magOffsetY = 0.0f;
  float magScaleX = 1.0f;
  float magScaleY = 1.0f;

  float magCalibrationRangeX = 0.0f;
  float magCalibrationRangeY = 0.0f;

  uint8_t magAddress = 0;
  unsigned long magReadCount = 0;
  unsigned long magLastReadMs = 0;
  unsigned long magLastHeadingChangeMs = 0;
};

struct OdometryState {
  float speedLeftCms = 0.0f;
  float speedRightCms = 0.0f;

  float xCm = 0.0f;
  float yCm = 0.0f;
  float thetaRad = 0.0f;

  float penXCm = 0.0f;
  float penYCm = 0.0f;
};

extern MotorState motorState;
extern SensorState sensorState;
extern OdometryState odometryState;

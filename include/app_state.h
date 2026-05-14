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

  float headingMagDeg = 0.0f;

  bool magCalibrationRunning = false;
  bool magCalibrationDone = false;
};

struct OdometryState {
  float speedLeftCms = 0.0f;
  float speedRightCms = 0.0f;

  float xCm = 0.0f;
  float yCm = 0.0f;
  float thetaRad = 0.0f;
};

extern MotorState motorState;
extern SensorState sensorState;
extern OdometryState odometryState;
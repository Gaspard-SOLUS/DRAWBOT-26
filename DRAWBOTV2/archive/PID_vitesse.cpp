#include <Arduino.h>
#include "PID_vitesse.h"
#include "encodeurs.h"
#include "moteurs.h"

// ==================================================
// ================= PARAMETRES PID ==================
// ==================================================
static float kp = 6.0f;
static float ki = 1.6f;
static float kd = 0.0f;

// ==================================================
// ================= ETAT PID GAUCHE ================
// ==================================================
static float leftTargetSpeed = 0.0f;
static float leftError = 0.0f;
static float leftPrevError = 0.0f;
static float leftIntegral = 0.0f;
static float leftOutput = 0.0f;

// ==================================================
// ================= ETAT PID DROIT =================
// ==================================================
static float rightTargetSpeed = 0.0f;
static float rightError = 0.0f;
static float rightPrevError = 0.0f;
static float rightIntegral = 0.0f;
static float rightOutput = 0.0f;

// Anti-windup simple
static const float INTEGRAL_LIMIT = 100.0f;

// Saturation PWM
static int clampPwm(int value) {
  if (value > 255) return 255;
  if (value < -255) return -255;
  return value;
}

static float clampFloat(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

void initPidVitesse() {
  leftTargetSpeed = 0.0f;
  rightTargetSpeed = 0.0f;

  leftError = 0.0f;
  rightError = 0.0f;

  leftPrevError = 0.0f;
  rightPrevError = 0.0f;

  leftIntegral = 0.0f;
  rightIntegral = 0.0f;

  leftOutput = 0.0f;
  rightOutput = 0.0f;
}

void setSpeedTargetsCmPerSec(float leftTarget, float rightTarget) {
  leftTargetSpeed = leftTarget;
  rightTargetSpeed = rightTarget;
}

void stopSpeedControl() {
  leftTargetSpeed = 0.0f;
  rightTargetSpeed = 0.0f;

  leftError = 0.0f;
  rightError = 0.0f;

  leftPrevError = 0.0f;
  rightPrevError = 0.0f;

  leftIntegral = 0.0f;
  rightIntegral = 0.0f;

  leftOutput = 0.0f;
  rightOutput = 0.0f;

  stopMotors();
}

void setPidGains(float newKp, float newKi, float newKd) {
  kp = newKp;
  ki = newKi;
  kd = newKd;

  // option utile : remettre les intégrales à zéro après changement
  leftIntegral = 0.0f;
  rightIntegral = 0.0f;
  leftPrevError = 0.0f;
  rightPrevError = 0.0f;
}

float getKp() {
  return kp;
}

float getKi() {
  return ki;
}

float getKd() {
  return kd;
}

int applyDeadzone(float output) {
  int pwm = static_cast<int>(output);
  if (pwm > -50 && pwm < 50) {
    return 0;
  }

  if (pwm > 0 && pwm < 180) {
    pwm = 180;
  } 
  else if (pwm < 0 && pwm > -180) {
    pwm = -180;
  }

  return clampPwm(pwm);
}

void updatePidVitesse(float dtSec) {
  if (dtSec <= 0.0f) {
    return;
  }
  if (leftTargetSpeed == 0.0f && rightTargetSpeed == 0.0f) {
    leftError = 0.0f;
    rightError = 0.0f;
    leftPrevError = 0.0f;
    rightPrevError = 0.0f;
    leftIntegral = 0.0f;
    rightIntegral = 0.0f;
    leftOutput = 0.0f;
    rightOutput = 0.0f;

    stopMotors();
    return;
  }

  float leftMeasured = getLeftSpeedCmParSec();
  float rightMeasured = getRightSpeedCmParSec();

  // ================= GAUCHE =================
  leftError = leftTargetSpeed - leftMeasured;
  leftIntegral += leftError * dtSec;
  leftIntegral = clampFloat(leftIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float leftDerivative = (leftError - leftPrevError) / dtSec;
  leftOutput = kp * leftError + ki * leftIntegral + kd * leftDerivative;

  // ================= DROIT ==================
  rightError = rightTargetSpeed - rightMeasured;
  rightIntegral += rightError * dtSec;
  rightIntegral = clampFloat(rightIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float rightDerivative = (rightError - rightPrevError) / dtSec;
  rightOutput = kp * rightError + ki * rightIntegral + kd * rightDerivative;

  leftPrevError = leftError;
  rightPrevError = rightError;

  int leftPwm = applyDeadzone(leftOutput);
  int rightPwm = applyDeadzone(rightOutput);

  setMotors(leftPwm, rightPwm);
}

float getLeftTargetSpeedCmPerSec() {
  return leftTargetSpeed;
}

float getRightTargetSpeedCmPerSec() {
  return rightTargetSpeed;
}

float getLeftPidError() {
  return leftError;
}

float getRightPidError() {
  return rightError;
}

float getLeftPidOutput() {
  return leftOutput;
}

float getRightPidOutput() {
  return rightOutput;
}
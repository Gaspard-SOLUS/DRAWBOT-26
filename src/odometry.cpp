#include <Arduino.h>
#include <math.h>

#include "odometry.h"
#include "encodeurs.h"
#include "app_state.h"
#include "logger.h"

static const float WHEEL_BASE_CM = 8.3f;
static const float PEN_OFFSET_CM = 13.0f;

static float lastDistL = 0.0f;
static float lastDistR = 0.0f;
static unsigned long lastOdoTime = 0;

namespace Odometry {
  float radToDeg(float rad) {
    return rad * 180.0f / PI;
  }

  float normalizeAngleDeg(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
  }

  void begin() {
    reset();
  }

  void update(unsigned long now) {
    updateEncoderMeasurements(now);

    if (lastOdoTime == 0) {
      lastOdoTime = now;
      lastDistL = getLeftDistanceCm();
      lastDistR = getRightDistanceCm();
      return;
    }

    float dt = (now - lastOdoTime) / 1000.0f;
    if (dt <= 0.0f) return;

    float currentDistL = getLeftDistanceCm();
    float currentDistR = getRightDistanceCm();

    float dL = currentDistL - lastDistL;
    float dR = currentDistR - lastDistR;

    odometryState.speedLeftCms = dL / dt;
    odometryState.speedRightCms = dR / dt;

    float dCenter = (dL + dR) * 0.5f;
    float dTheta = (dR - dL) / WHEEL_BASE_CM;

    odometryState.thetaRad += dTheta;

    float thetaMid = odometryState.thetaRad - dTheta * 0.5f;

    odometryState.xCm += dCenter * cos(thetaMid);
    odometryState.yCm += dCenter * sin(thetaMid);

    odometryState.penXCm = odometryState.xCm + PEN_OFFSET_CM * cos(odometryState.thetaRad);
    odometryState.penYCm = odometryState.yCm + PEN_OFFSET_CM * sin(odometryState.thetaRad);

    lastDistL = currentDistL;
    lastDistR = currentDistR;
    lastOdoTime = now;
  }

  void reset() {
    resetEncoders();

    lastDistL = 0.0f;
    lastDistR = 0.0f;
    lastOdoTime = 0;

    odometryState.speedLeftCms = 0.0f;
    odometryState.speedRightCms = 0.0f;

    odometryState.xCm = 0.0f;
    odometryState.yCm = 0.0f;
    odometryState.thetaRad = 0.0f;

    odometryState.penXCm = PEN_OFFSET_CM;
    odometryState.penYCm = 0.0f;

    Logger::log("Odometrie remise a zero");
  }
}
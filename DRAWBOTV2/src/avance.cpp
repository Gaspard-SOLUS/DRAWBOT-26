#include <Arduino.h>
#include "include/avance.h"
#include "include/moteurs.h"
#include "include/encodeurs.h"

static bool avanceActive = false;
static bool avanceTermine = true;

static float avanceTargetCm = 0.0f;
static float avanceSlowZoneCm = 0.0f;

static int avanceCruisePwm = 0;
static int avanceSlowPwm = 0;

void startAvanceForwardDistance(float targetCm, int cruisePwm, int slowPwm, float slowZoneCm) {
  avanceTargetCm = targetCm;
  avanceCruisePwm = cruisePwm;
  avanceSlowPwm = slowPwm;
  avanceSlowZoneCm = slowZoneCm;

  resetEncoders();

  avanceActive = true;
  avanceTermine = false;
}

void updateAvance() {
  if (!avanceActive) {
    return;
  }

  float distanceCm = getAverageDistanceCm();
  float remainingCm = avanceTargetCm - distanceCm;

  if (remainingCm <= 0.0f) {
    stopMotors();
    avanceActive = false;
    avanceTermine = true;
    return;
  }

  if (remainingCm <= avanceSlowZoneCm) {
    setMotors(avanceSlowPwm, avanceSlowPwm);
  } else {
    setMotors(avanceCruisePwm, avanceCruisePwm);
  }
}

bool isAvanceTermine() {
  return avanceTermine;
}

float getAvanceTargetCm() {
  return avanceTargetCm;
}

float getAvanceCurrentCm() {
  return getAverageDistanceCm();
}
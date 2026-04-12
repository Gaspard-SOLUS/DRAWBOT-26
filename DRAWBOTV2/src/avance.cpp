#include <Arduino.h>
#include "include/avance.h"
#include "include/moteurs.h"
#include "include/encodeurs.h"

// ==================================================
// ================= ETAT INTERNE ===================
// ==================================================
enum AvanceState {
  AVANCE_IDLE,
  AVANCE_RUNNING,
  AVANCE_BRAKING,
  AVANCE_FINISHED
};

static AvanceState avanceState = AVANCE_IDLE;

static float avanceTargetCm = 0.0f;
static float avanceSlowZoneCm = 0.0f;
static float avanceKpStraight = 0.0f;

static int avanceCruisePwm = 0;
static int avanceSlowPwm = 0;

static int leftCommand = 0;
static int rightCommand = 0;

static float straightErrorTicks = 0.0f;

// durée du freinage actif final
static const unsigned long BRAKE_TIME_MS = 150;
static unsigned long brakeStartTime = 0;

// Saturation de commande
static int clampCommand(int value) {
  if (value > 255) return 255;
  if (value < -255) return -255;
  return value;
}

// ==================================================
// ================== DEMARRAGE =====================
// ==================================================
void startAvanceForwardDistance(float targetCm, int cruisePwm, int slowPwm, float slowZoneCm, float kpStraight) {
  avanceTargetCm = targetCm;
  avanceCruisePwm = cruisePwm;
  avanceSlowPwm = slowPwm;
  avanceSlowZoneCm = slowZoneCm;
  avanceKpStraight = kpStraight;

  leftCommand = 0;
  rightCommand = 0;
  straightErrorTicks = 0.0f;

  resetEncoders();

  avanceState = AVANCE_RUNNING;
}

// ==================================================
// ==================== UPDATE ======================
// ==================================================
void updateAvance(unsigned long now) {
  switch (avanceState) {
    case AVANCE_IDLE:
      stopMotors();
      leftCommand = 0;
      rightCommand = 0;
      break;

    case AVANCE_RUNNING: {
      float distanceCm = getAverageDistanceCm();
      float remainingCm = avanceTargetCm - distanceCm;

      // cible atteinte -> freinage
      if (remainingCm <= 0.0f) {
        brakeMotors();
        brakeStartTime = now;
        avanceState = AVANCE_BRAKING;
        leftCommand = 0;
        rightCommand = 0;
        return;
      }

      // Choix du PWM de base
      int basePwm = (remainingCm <= avanceSlowZoneCm) ? avanceSlowPwm : avanceCruisePwm;

      // Erreur de ligne droite
      long leftTicks = getLeftEncoderTicks();
      long rightTicks = getRightEncoderTicks();
      straightErrorTicks = static_cast<float>(leftTicks - rightTicks);

      // Correction proportionnelle
      float correction = avanceKpStraight * straightErrorTicks;

      int cmdLeft = basePwm - static_cast<int>(correction);
      int cmdRight = basePwm + static_cast<int>(correction);

      cmdLeft = clampCommand(cmdLeft);
      cmdRight = clampCommand(cmdRight);

      leftCommand = cmdLeft;
      rightCommand = cmdRight;

      setMotors(leftCommand, rightCommand);
      break;
    }

    case AVANCE_BRAKING:
      brakeMotors();

      if (now - brakeStartTime >= BRAKE_TIME_MS) {
        stopMotors();
        avanceState = AVANCE_FINISHED;
      }
      break;

    case AVANCE_FINISHED:
      stopMotors();
      leftCommand = 0;
      rightCommand = 0;
      break;
  }
}

// ==================================================
// ==================== GETTERS =====================
// ==================================================
bool isAvanceTermine() {
  return (avanceState == AVANCE_FINISHED);
}

float getAvanceTargetCm() {
  return avanceTargetCm;
}

float getAvanceCurrentCm() {
  return getAverageDistanceCm();
}

float getAvanceStraightErrorTicks() {
  return straightErrorTicks;
}

int getAvanceLeftCommand() {
  return leftCommand;
}

int getAvanceRightCommand() {
  return rightCommand;
}
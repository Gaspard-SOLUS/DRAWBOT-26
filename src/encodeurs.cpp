#include <Arduino.h>
#include "pins.h"
#include "encodeurs.h"
#include "robot.h"

// Compteurs encodeurs
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// MESURES DERIVEES / FILTREES
static long previousLeftTicks = 0;
static long previousRightTicks = 0;

static unsigned long previousUpdateMs = 0;

static float leftSpeedTicksParSec = 0.0f;
static float rightSpeedTicksParSec = 0.0f;

static float leftSpeedCmParSec = 0.0f;
static float rightSpeedCmParSec = 0.0f;

// ==================================================
// ================== INTERRUPTION ==================
// ==================================================
// Encodeur gauche : interruption sur voie A
void IRAM_ATTR leftEncoderISR() {
  int a = digitalRead(ENC_G_CH_A);
  int b = digitalRead(ENC_G_CH_B);

  // Détermination du sens
  if (a == b) {
    leftTicks--;
  } else {
    leftTicks++;
  }
}

// Encodeur droit : interruption sur voie A
void IRAM_ATTR rightEncoderISR() {
  int a = digitalRead(ENC_D_CH_A);
  int b = digitalRead(ENC_D_CH_B);

  // Détermination du sens
  if (a == b) {
    rightTicks++;
  } else {
    rightTicks--;
  }
}

// ==================================================
// ================= INITIALISATION =================
// ==================================================
void initEncoders() {
  pinMode(ENC_G_CH_A, INPUT);
  pinMode(ENC_G_CH_B, INPUT);

  pinMode(ENC_D_CH_A, INPUT);
  pinMode(ENC_D_CH_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), rightEncoderISR, CHANGE);

  previousUpdateMs = millis();
  previousLeftTicks = 0;
  previousRightTicks = 0;
}

// ==================================================
// ==================== LECTURES ====================
// ==================================================
long getLeftEncoderTicks() {
  noInterrupts();
  long ticks = leftTicks;
  interrupts();
  return ticks;
}

long getRightEncoderTicks() {
  noInterrupts();
  long ticks = rightTicks;
  interrupts();
  return ticks;
}

// ==================================================
// ===================== RESET ======================
// ==================================================
void resetLeftEncoder() {
  noInterrupts();
  leftTicks = 0;
  interrupts();
}

void resetRightEncoder() {
  noInterrupts();
  rightTicks = 0;
  interrupts();
}

void resetEncoders() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  previousLeftTicks = 0;
  previousRightTicks = 0;

  leftSpeedTicksParSec = 0.0f;
  rightSpeedTicksParSec = 0.0f;
  leftSpeedCmParSec = 0.0f;
  rightSpeedCmParSec = 0.0f;

  previousUpdateMs = millis();
}

// CALCUL VITESSES / DISTANCES
void updateEncoderMeasurements(unsigned long nowMs) {
  unsigned long dtMs = nowMs - previousUpdateMs;

  if (dtMs == 0) {
    return;
  }

  long currentLeftTicks = getLeftEncoderTicks();
  long currentRightTicks = getRightEncoderTicks();

  long deltaLeftTicks = currentLeftTicks - previousLeftTicks;
  long deltaRightTicks = currentRightTicks - previousRightTicks;

  float dtSec = dtMs / 1000.0f;

  leftSpeedTicksParSec = deltaLeftTicks / dtSec;
  rightSpeedTicksParSec = deltaRightTicks / dtSec;

  leftSpeedCmParSec = leftSpeedTicksParSec * RobotParams::CM_PAR_TICK;
  rightSpeedCmParSec = rightSpeedTicksParSec * RobotParams::CM_PAR_TICK;

  previousLeftTicks = currentLeftTicks;
  previousRightTicks = currentRightTicks;
  previousUpdateMs = nowMs;
}

// ==================================================
// ==================== DISTANCES ===================
// ==================================================
float getLeftDistanceCm() {
  return getLeftEncoderTicks() * RobotParams::CM_PAR_TICK;
}

float getRightDistanceCm() {
  return getRightEncoderTicks() * RobotParams::CM_PAR_TICK;
}

float getAverageDistanceCm() {
  return (getLeftDistanceCm() + getRightDistanceCm()) * 0.5f;
}

// ==================================================
// ==================== VITESSES ====================
// ==================================================
float getLeftSpeedTicksParSec() {
  return leftSpeedTicksParSec;
}

float getRightSpeedTicksParSec() {
  return rightSpeedTicksParSec;
}

float getLeftSpeedCmParSec() {
  return leftSpeedCmParSec;
}

float getRightSpeedCmParSec() {
  return rightSpeedCmParSec;
}
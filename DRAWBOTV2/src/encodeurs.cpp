#include <Arduino.h>
#include "include/pins.h"
#include "include/encodeurs.h"
#include "include/robot.h"

// ===================== COMPTEURS BRUTS =====================
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// ===================== VITESSES CALCULÉES =====================
static long  previousLeftTicks  = 0;
static long  previousRightTicks = 0;
static unsigned long previousUpdateMs = 0;

static float leftSpeedCmParSec  = 0.0f;
static float rightSpeedCmParSec = 0.0f;
static float leftSpeedTicksParSec  = 0.0f;
static float rightSpeedTicksParSec = 0.0f;

// ===================== INTERRUPTIONS =====================
// Encodeur GAUCHE – voie A
void IRAM_ATTR leftEncoderISR() {
    int a = digitalRead(ENC_G_CH_A);
    int b = digitalRead(ENC_G_CH_B);
    // a == b → sens horaire vu du moteur → avance
    if (a == b) leftTicks++;
    else        leftTicks--;
}

// Encodeur DROIT – voie A
void IRAM_ATTR rightEncoderISR() {
    int a = digitalRead(ENC_D_CH_A);
    int b = digitalRead(ENC_D_CH_B);
    // Sens inversé mécaniquement pour le moteur droit
    if (a == b) rightTicks--;
    else        rightTicks++;
}

// ===================== INITIALISATION =====================
void initEncoders() {
    pinMode(ENC_G_CH_A, INPUT);
    pinMode(ENC_G_CH_B, INPUT);
    pinMode(ENC_D_CH_A, INPUT);
    pinMode(ENC_D_CH_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), leftEncoderISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), rightEncoderISR, CHANGE);

    previousUpdateMs    = millis();
    previousLeftTicks   = 0;
    previousRightTicks  = 0;
}

// ===================== LECTURES ATOMIQUES =====================
long getLeftEncoderTicks() {
    noInterrupts();
    long t = leftTicks;
    interrupts();
    return t;
}

long getRightEncoderTicks() {
    noInterrupts();
    long t = rightTicks;
    interrupts();
    return t;
}

// ===================== RESET =====================
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
    leftTicks  = 0;
    rightTicks = 0;
    interrupts();

    previousLeftTicks       = 0;
    previousRightTicks      = 0;
    leftSpeedCmParSec       = 0.0f;
    rightSpeedCmParSec      = 0.0f;
    leftSpeedTicksParSec    = 0.0f;
    rightSpeedTicksParSec   = 0.0f;
    previousUpdateMs        = millis();
}

// ===================== CALCUL VITESSES =====================
void updateEncoderMeasurements(unsigned long nowMs) {
    unsigned long dtMs = nowMs - previousUpdateMs;
    if (dtMs == 0) return;

    long curLeft  = getLeftEncoderTicks();
    long curRight = getRightEncoderTicks();

    long dL = curLeft  - previousLeftTicks;
    long dR = curRight - previousRightTicks;

    float dtSec = dtMs / 1000.0f;

    leftSpeedTicksParSec  = dL / dtSec;
    rightSpeedTicksParSec = dR / dtSec;
    leftSpeedCmParSec     = leftSpeedTicksParSec  * RobotParams::CM_PAR_TICK;
    rightSpeedCmParSec    = rightSpeedTicksParSec * RobotParams::CM_PAR_TICK;

    previousLeftTicks   = curLeft;
    previousRightTicks  = curRight;
    previousUpdateMs    = nowMs;
}

// ===================== DISTANCES =====================
float getLeftDistanceCm()    { return getLeftEncoderTicks()  * RobotParams::CM_PAR_TICK; }
float getRightDistanceCm()   { return getRightEncoderTicks() * RobotParams::CM_PAR_TICK; }
float getAverageDistanceCm() { return (getLeftDistanceCm() + getRightDistanceCm()) * 0.5f; }

// ===================== VITESSES =====================
float getLeftSpeedCmParSec()      { return leftSpeedCmParSec;       }
float getRightSpeedCmParSec()     { return rightSpeedCmParSec;      }
float getLeftSpeedTicksParSec()   { return leftSpeedTicksParSec;    }
float getRightSpeedTicksParSec()  { return rightSpeedTicksParSec;   }
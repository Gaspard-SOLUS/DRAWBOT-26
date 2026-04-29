#include <Arduino.h>
#include "include/pid_vitesse.h"
#include "include/encodeurs.h"
#include "include/moteurs.h"
#include "include/robot.h"

// ==================================================
// PARAMÈTRES PID (réglables depuis l'IHM web)
// ==================================================
static float kp = 6.0f;
static float ki = 1.6f;
static float kd = 0.0f;

// Anti-windup
static const float INTEGRAL_LIMIT = 100.0f;

// ==================================================
// ÉTAT PID GAUCHE
// ==================================================
static float leftTargetSpeed  = 0.0f;
static float leftError        = 0.0f;
static float leftPrevError    = 0.0f;
static float leftIntegral     = 0.0f;
static float leftOutput       = 0.0f;

// ==================================================
// ÉTAT PID DROIT
// ==================================================
static float rightTargetSpeed = 0.0f;
static float rightError       = 0.0f;
static float rightPrevError   = 0.0f;
static float rightIntegral    = 0.0f;
static float rightOutput      = 0.0f;

// ==================================================
// UTILITAIRES
// ==================================================
static int clampPwm(int v) {
    if (v >  255) return  255;
    if (v < -255) return -255;
    return v;
}
static float clampFloat(float v, float mn, float mx) {
    if (v < mn) return mn;
    if (v > mx) return mx;
    return v;
}

// Deadzone mécanique : en dessous de PWM_MIN les moteurs ne bougent pas
static int applyDeadzone(float output) {
    int pwm = static_cast<int>(output);
    const int DZ  = 50;  // zone morte absolue (en-dessous → 0)
    const int MIN = RobotParams::PWM_MIN;

    if (pwm > -DZ && pwm < DZ) return 0;

    if (pwm > 0 && pwm < MIN)  pwm = MIN;
    else if (pwm < 0 && pwm > -MIN) pwm = -MIN;

    return clampPwm(pwm);
}

// ==================================================
// API
// ==================================================
void initPidVitesse() {
    leftTargetSpeed = rightTargetSpeed = 0.0f;
    leftError = leftPrevError = leftIntegral = leftOutput = 0.0f;
    rightError = rightPrevError = rightIntegral = rightOutput = 0.0f;
}

void setSpeedTargetsCmPerSec(float leftTarget, float rightTarget) {
    leftTargetSpeed  = leftTarget;
    rightTargetSpeed = rightTarget;
}

void stopSpeedControl() {
    leftTargetSpeed = rightTargetSpeed = 0.0f;
    leftError = leftPrevError = leftIntegral = leftOutput = 0.0f;
    rightError = rightPrevError = rightIntegral = rightOutput = 0.0f;
    stopMotors();
}

void setPidGains(float newKp, float newKi, float newKd) {
    kp = newKp; ki = newKi; kd = newKd;
    leftIntegral = rightIntegral = 0.0f;
    leftPrevError = rightPrevError = 0.0f;
}

float getKp() { return kp; }
float getKi() { return ki; }
float getKd() { return kd; }

void updatePidVitesse(float dtSec) {
    if (dtSec <= 0.0f) return;

    if (leftTargetSpeed == 0.0f && rightTargetSpeed == 0.0f) {
        initPidVitesse();
        stopMotors();
        return;
    }

    float leftMeasured  = getLeftSpeedCmParSec();
    float rightMeasured = getRightSpeedCmParSec();

    // ----- GAUCHE -----
    leftError     = leftTargetSpeed - leftMeasured;
    leftIntegral += leftError * dtSec;
    leftIntegral  = clampFloat(leftIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float leftDeriv = (leftError - leftPrevError) / dtSec;
    leftOutput    = kp * leftError + ki * leftIntegral + kd * leftDeriv;
    leftPrevError = leftError;

    // ----- DROIT -----
    rightError     = rightTargetSpeed - rightMeasured;
    rightIntegral += rightError * dtSec;
    rightIntegral  = clampFloat(rightIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float rightDeriv = (rightError - rightPrevError) / dtSec;
    rightOutput    = kp * rightError + ki * rightIntegral + kd * rightDeriv;
    rightPrevError = rightError;

    setMotors(applyDeadzone(leftOutput), applyDeadzone(rightOutput));
}

// ---- Getters debug ----
float getLeftTargetSpeedCmPerSec()  { return leftTargetSpeed;  }
float getRightTargetSpeedCmPerSec() { return rightTargetSpeed; }
float getLeftPidError()             { return leftError;         }
float getRightPidError()            { return rightError;        }
float getLeftPidOutput()            { return leftOutput;        }
float getRightPidOutput()           { return rightOutput;       }
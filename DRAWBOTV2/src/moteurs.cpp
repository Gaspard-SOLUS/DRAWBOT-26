#include <Arduino.h>
#include "pins.h"
#include "moteurs.h"

// Réglages PWM
static const int PWM_FREQ = 20000;// 20 kHz
static const int PWM_RESOLUTION = 8;// 8 bits

// Canaux LEDC
static const int CH_IN1_G = 0;
static const int CH_IN2_G = 1;
static const int CH_IN1_D = 2;
static const int CH_IN2_D = 3;

static int clampPwm(int pwm) {
  if (pwm > 255) return 255;
  if (pwm < -255) return -255;
  return pwm;
}

// Initialisation moteurs
void initMotors() {
  pinMode(EN_G, OUTPUT);
  pinMode(EN_D, OUTPUT);

  digitalWrite(EN_G, HIGH);
  digitalWrite(EN_D, HIGH);

  ledcSetup(CH_IN1_G, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_IN2_G, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_IN1_D, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_IN2_D, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(IN_1_G, CH_IN1_G);
  ledcAttachPin(IN_2_G, CH_IN2_G);
  ledcAttachPin(IN_1_D, CH_IN1_D);
  ledcAttachPin(IN_2_D, CH_IN2_D);

  stopMotors();
}

void setLeftMotor(int pwm) {
  pwm = clampPwm(pwm);

  if (pwm > 0) {
    ledcWrite(CH_IN1_G, pwm);
    ledcWrite(CH_IN2_G, 0);
  }
  else if (pwm < 0) {
    ledcWrite(CH_IN1_G, 0);
    ledcWrite(CH_IN2_G, -pwm);
  }
  else {
    ledcWrite(CH_IN1_G, 0);
    ledcWrite(CH_IN2_G, 0);
  }
}

void setRightMotor(int pwm) {
  pwm = clampPwm(pwm);

  if (pwm > 0) {
    ledcWrite(CH_IN1_D, pwm);
    ledcWrite(CH_IN2_D, 0);
  }
  else if (pwm < 0) {
    ledcWrite(CH_IN1_D, 0);
    ledcWrite(CH_IN2_D, -pwm);
  }
  else {
    ledcWrite(CH_IN1_D, 0);
    ledcWrite(CH_IN2_D, 0);
  }
}

void setMotors(int leftPwm, int rightPwm) {
  setLeftMotor(leftPwm);
  setRightMotor(rightPwm);
}

void stopMotors() {
  ledcWrite(CH_IN1_G, 0);
  ledcWrite(CH_IN2_G, 0);
  ledcWrite(CH_IN1_D, 0);
  ledcWrite(CH_IN2_D, 0);
}

void brakeMotors() {
  ledcWrite(CH_IN1_G, 255);
  ledcWrite(CH_IN2_G, 255);
  ledcWrite(CH_IN1_D, 255);
  ledcWrite(CH_IN2_D, 255);
}
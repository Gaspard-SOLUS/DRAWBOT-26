#include <Arduino.h>
#include "include/pins.h"
#include "include/moteurs.h"
#include "include/encodeurs.h"
#include "include/robot.h"

enum RobotState {
  AVANCE,
  RECULE,
  STOP
};

RobotState state = AVANCE;
unsigned long stateStartTime = 0;

// LED de vie
unsigned long lastBlinkTime = 0;
bool ledState = false;

// Affichage série périodique
unsigned long lastEncoderUpdateTime = 0;
unsigned long lastPrintTime = 0;

void updateSequence(unsigned long now) {
  switch (state) {
    case AVANCE:
      setMotors(200, 200);

      if (now - stateStartTime >= 2000) {
        state = RECULE;
        stateStartTime = now;
        Serial.println("Transition -> RECULE");
      }
      break;

    case RECULE:
      setMotors(-200, -200);

      if (now - stateStartTime >= 2000) {
        state = STOP;
        stateStartTime = now;
        Serial.println("Transition -> STOP");
      }
      break;

    case STOP:
      stopMotors();
      // brakeMotors();

      if (now - stateStartTime >= 2000) {
        state = AVANCE;
        stateStartTime = now;
        resetEncoders();
        Serial.println("Transition -> AVANCE");
      }
      break;
  }
}

// LED de vie
void updateHeartbeat(unsigned long now) {
  if (now - lastBlinkTime >= 500) {
    lastBlinkTime = now;
    ledState = !ledState;
    digitalWrite(LEDU1, ledState);
  }
}

// Mise à jour des encoreurs
void updateEncoderTask(unsigned long now) {
  if (now - lastEncoderUpdateTime >= 50) {
    lastEncoderUpdateTime = now;
    updateEncoderMeasurements(now);
  }
}

void printEncoders(unsigned long now) {
  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;

    Serial.print("L ticks = ");
    Serial.print(getLeftEncoderTicks());
    Serial.print(" | R ticks = ");
    Serial.print(getRightEncoderTicks());

    Serial.print(" || L dist(cm) = ");
    Serial.print(getLeftDistanceCm(), 2);
    Serial.print(" | R dist(cm) = ");
    Serial.print(getRightDistanceCm(), 2);
    Serial.print(" | Avg dist(cm) = ");
    Serial.print(getAverageDistanceCm(), 2);

    Serial.print(" || L speed(cm/s) = ");
    Serial.print(getLeftSpeedCmParSec(), 2);
    Serial.print(" | R speed(cm/s) = ");
    Serial.println(getRightSpeedCmParSec(), 2);
  }
}

// ==================================================
// ===================== SETUP ======================
// ==================================================
void setup() {
  Serial.begin(115200);

  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  digitalWrite(LEDU1, LOW);
  digitalWrite(LEDU2, LOW);

  initMotors();
  initEncoders();
  resetEncoders();

  stateStartTime = millis();
  lastBlinkTime = millis();
  lastEncoderUpdateTime = millis();
  lastPrintTime = millis();

  Serial.println("Robot initialise");
}

// ==================================================
// ===================== LOOP= ======================
// ==================================================
void loop() {
  unsigned long now = millis();

  updateSequence(now);
  updateHeartbeat(now);
  updateEncoderTask(now);
  printEncoders(now);

  // updatePid();
  // sendTelemetry();
}
#include <Arduino.h>
#include "include/pins.h"
#include "include/moteurs.h"
#include "include/encodeurs.h"
#include "include/robot.h"
#include "include/avance.h"

unsigned long lastBlinkTime = 0;
bool ledState = false;
unsigned long lastEncoderUpdateTime = 0;
unsigned long lastPrintTime = 0;

bool commandStarted = false;

void updateHeartbeat(unsigned long now) {
  if (now - lastBlinkTime >= 500) {
    lastBlinkTime = now;
    ledState = !ledState;
    digitalWrite(LEDU1, ledState);
  }
}

void updateEncoderTask(unsigned long now) {
  if (now - lastEncoderUpdateTime >= 20) {
    lastEncoderUpdateTime = now;
    updateEncoderMeasurements(now);
  }
}

void printTelemetry(unsigned long now) {
  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;

    Serial.print("L ticks = ");
    Serial.print(getLeftEncoderTicks());
    Serial.print(" | R ticks = ");
    Serial.print(getRightEncoderTicks());

    Serial.print(" || Avg dist(cm) = ");
    Serial.print(getAverageDistanceCm(), 2);

    Serial.print(" || L speed(cm/s) = ");
    Serial.print(getLeftSpeedCmParSec(), 2);
    Serial.print(" | R speed(cm/s) = ");
    Serial.println(getRightSpeedCmParSec(), 2);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  initMotors();
  initEncoders();
  resetEncoders();

  lastBlinkTime = millis();
  lastEncoderUpdateTime = millis();
  lastPrintTime = millis();

  delay(1000);

  Serial.println("Test avance distance");
}

void loop() {
  unsigned long now = millis();

  updateHeartbeat(now);
  updateEncoderTask(now);

  if (!commandStarted) {
    startAvanceForwardDistance(20.0f, 180, 90, 2.5f);
    commandStarted = true;
    Serial.println("Demarrage avance 20 cm");
  }

  updateAvance();
  printTelemetry(now);

  if (commandStarted && isAvanceTermine()) {
    Serial.println("Deplacement termine");
    while (true) {
      stopMotors();
      //brakeMotors();
      updateHeartbeat(millis());
    }
  }
}
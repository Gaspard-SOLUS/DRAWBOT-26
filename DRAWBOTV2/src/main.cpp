#include <Arduino.h>
#include "include/pins.h"
#include "include/moteurs.h"
#include "include/encodeurs.h"
#include "include/pid_vitesse.h"
#include "include/wifi_param.h"

unsigned long lastBlinkTime = 0;
bool ledState = false;

unsigned long lastControlTime = 0;
unsigned long lastWifiTime = 0;
unsigned long lastPrintTime = 0;

// LED de vie
void updateHeartbeat(unsigned long now) {
  if (now - lastBlinkTime >= 500) {
    lastBlinkTime = now;
    ledState = !ledState;
    digitalWrite(LEDU1, ledState);
  }
}

// ==========================================
// Boucle de contrôle à 20 ms
// ==========================================
void updateControlTask(unsigned long now) {
  if (now - lastControlTime >= 20) {
    float dtSec = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;

    updateEncoderMeasurements(now);
    updatePidVitesse(dtSec);
  }
}

// ==========================================
// Envoi Teleplot à 50 ms
// ==========================================
void updateWifiTelemetryTask(unsigned long now) {
  if (now - lastWifiTime >= 50) {
    lastWifiTime = now;
    wifiSendTelemetry();
    wifiSendToGUI();
  }
}

// ==========================================
// Affichage série à 200 ms
// ==========================================
void updateSerialPrintTask(unsigned long now) {
  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;

    Serial.print("targetL=");
    Serial.print(getLeftTargetSpeedCmPerSec(), 2);
    Serial.print(" targetR=");
    Serial.print(getRightTargetSpeedCmPerSec(), 2);

    Serial.print(" | speedL=");
    Serial.print(getLeftSpeedCmParSec(), 2);
    Serial.print(" speedR=");
    Serial.print(getRightSpeedCmParSec(), 2);

    Serial.print(" | errL=");
    Serial.print(getLeftPidError(), 2);
    Serial.print(" errR=");
    Serial.print(getRightPidError(), 2);

    Serial.print(" | outL=");
    Serial.print(getLeftPidOutput(), 2);
    Serial.print(" outR=");
    Serial.println(getRightPidOutput(), 2);
  }
}

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  initMotors();
  initEncoders();
  initPidVitesse();
  resetEncoders();

  wifiInit();

  unsigned long now = millis();
  lastBlinkTime = now;
  lastControlTime = now;
  lastWifiTime = now;
  lastPrintTime = now;

  setSpeedTargetsCmPerSec(0.0f, 0.0f);

  delay(1000);
  Serial.println("Test avec GUI");
}

// ==========================================
// Loop
// ==========================================
void loop() {
  unsigned long now = millis();

  updateHeartbeat(now);
  wifiHandleClient();

  updateControlTask(now);
  updateWifiTelemetryTask(now);
  updateSerialPrintTask(now);
}
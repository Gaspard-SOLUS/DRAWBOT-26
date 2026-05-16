#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

#include "pins.h"
#include "moteurs.h"
#include "encodeurs.h"
#include "logger.h"
#include "sensors.h"
#include "odometry.h"
#include "teleplot.h"
#include "web_server_app.h"
#include "soutenance2.h"

// ==================================================
// TIMERS PRINCIPAUX
// ==================================================

static unsigned long lastControlUpdate = 0;

const unsigned long CONTROL_PERIOD_MS = 20;


// ==================================================
// LED 1 : HEARTBEAT / PROGRAMME VIVANT
// ==================================================
// LEDU1 clignote toutes les 500 ms.
// Son rôle est de montrer que le programme tourne encore.
// Si elle ne clignote plus, l'ESP32 est probablement bloqué ou a redémarré.

static unsigned long lastHeartbeatUpdate = 0;
static bool heartbeatState = false;

const unsigned long HEARTBEAT_PERIOD_MS = 500;

void updateHeartbeatLed(unsigned long now) {
  if (now - lastHeartbeatUpdate >= HEARTBEAT_PERIOD_MS) {
    lastHeartbeatUpdate = now;

    heartbeatState = !heartbeatState;
    digitalWrite(LEDU1, heartbeatState ? HIGH : LOW);
  }
}


// ==================================================
// LED 2 : CONNEXION WIFI
// ==================================================
// LEDU2 est allumée si au moins un appareil est connecté
// au point d'accès WiFi de l'ESP32.
// Elle s'éteint dès qu'il n'y a plus aucun appareil connecté.

static unsigned long lastWifiLedUpdate = 0;
static bool previousWifiConnected = false;

const unsigned long WIFI_LED_PERIOD_MS = 100;

void updateWifiConnectionLed(unsigned long now) {
  if (now - lastWifiLedUpdate < WIFI_LED_PERIOD_MS) {
    return;
  }

  lastWifiLedUpdate = now;

  int connectedDevices = WiFi.softAPgetStationNum();
  bool wifiConnected = connectedDevices > 0;

  digitalWrite(LEDU2, wifiConnected ? HIGH : LOW);

  if (wifiConnected != previousWifiConnected) {
    previousWifiConnected = wifiConnected;

    if (wifiConnected) {
      Logger::log("Appareil connecte au WiFi ESP32. Nombre : " + String(connectedDevices));
    } else {
      Logger::log("Aucun appareil connecte au WiFi ESP32.");
    }
  }
}


// ==================================================
// SETUP
// ==================================================

void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  digitalWrite(LEDU1, LOW);
  digitalWrite(LEDU2, LOW);

  Logger::begin();
  Logger::log("=== DRAWBOT - BRANCHE SOUTENANCE 2 ===");

  Wire.begin(SDA, SCL);

  initMotors();
  initEncoders();
  resetEncoders();
  stopMotors();

  Sensors::begin();
  Odometry::begin();

  // WebApp::begin() lance le WiFi en point d'acces
  // et initialise le serveur web.
  WebApp::begin();

  Teleplot::begin();

  Soutenance2::begin();

  lastControlUpdate = millis();
  lastHeartbeatUpdate = millis();
  lastWifiLedUpdate = millis();

  Logger::log("Initialisation terminee");
}


// ==================================================
// LOOP
// ==================================================

void loop() {
  unsigned long now = millis();

  // Gestion des LEDs d'etat
  updateHeartbeatLed(now);
  updateWifiConnectionLed(now);

  // Serveur web : reception des commandes depuis l'interface
  WebApp::handleClient();

  // Boucle de controle principale du robot
  if (now - lastControlUpdate >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlUpdate) / 1000.0f;
    lastControlUpdate = now;

    Sensors::update(now, dt);
    Odometry::update(now);
    Soutenance2::update(now, dt);
  }

  // Envoi Teleplot
  Teleplot::update(now);

  // Laisse du temps au WiFi / serveur web de l'ESP32
  yield();
}
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "web_server_app.h"
#include "web_pages.h"
#include "moteurs.h"
#include "encodeurs.h"
#include "sensors.h"
#include "odometry.h"
#include "logger.h"
#include "app_state.h"

namespace WebApp {

static WebServer server(80);

static const char* WIFI_SSID = "Drawbot";
static const char* WIFI_PASSWORD = "12345678";

static String jsonStatus() {
  String j = "{";
  j += "\"state\":\"OK\",";
  j += "\"mode\":\"" + motorState.mode + "\",";
  j += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  j += "\"uptime\":" + String(millis() / 1000.0f, 1) + ",";

  j += "\"pwmL\":" + String(motorState.pwmLeft) + ",";
  j += "\"pwmR\":" + String(motorState.pwmRight) + ",";

  j += "\"ticksL\":" + String(getLeftEncoderTicks()) + ",";
  j += "\"ticksR\":" + String(getRightEncoderTicks()) + ",";
  j += "\"distL\":" + String(getLeftDistanceCm(), 3) + ",";
  j += "\"distR\":" + String(getRightDistanceCm(), 3) + ",";
  j += "\"speedL\":" + String(getLeftSpeedCmParSec(), 3) + ",";
  j += "\"speedR\":" + String(getRightSpeedCmParSec(), 3) + ",";

  j += "\"accX\":" + String(sensorState.accX, 3) + ",";
  j += "\"accY\":" + String(sensorState.accY, 3) + ",";
  j += "\"accZ\":" + String(sensorState.accZ, 3) + ",";

  j += "\"gyroX\":" + String(sensorState.gyroX, 3) + ",";
  j += "\"gyroY\":" + String(sensorState.gyroY, 3) + ",";
  j += "\"gyroZ\":" + String(sensorState.gyroZ, 3) + ",";
  j += "\"yawGyro\":" + String(sensorState.yawGyroDeg, 3) + ",";

  j += "\"magX\":" + String(sensorState.magX, 3) + ",";
  j += "\"magY\":" + String(sensorState.magY, 3) + ",";
  j += "\"magZ\":" + String(sensorState.magZ, 3) + ",";
  j += "\"headingMag\":" + String(sensorState.headingMagDeg, 3) + ",";
  j += "\"magCalib\":\"" + String(sensorState.magCalibrationDone ? "OK" : "NON") + "\",";

  j += "\"odoX\":" + String(odometryState.xCm, 3) + ",";
  j += "\"odoY\":" + String(odometryState.yCm, 3) + ",";
  j += "\"odoTheta\":" + String(Odometry::radToDeg(odometryState.thetaRad), 3);
  j += "}";
  return j;
}

static void sendOk(const String& msg = "OK") {
  server.send(200, "text/plain", msg);
}

static void handleManual() {
  int l = server.hasArg("l") ? server.arg("l").toInt() : 0;
  int r = server.hasArg("r") ? server.arg("r").toInt() : 0;
  setMotors(l, r);
  motorState.pwmLeft = l;
  motorState.pwmRight = r;
  motorState.mode = "MANUAL";
  sendOk();
}

static void handleStop() {
  stopMotors();
  motorState.pwmLeft = 0;
  motorState.pwmRight = 0;
  motorState.mode = "STOP";
  sendOk();
}

static void handleResetEncoders() {
  resetEncoders();
  Odometry::reset();
  sendOk();
}

static void handleNotImplemented(const String& name) {
  Logger::log(name + " appelee mais sequence non reliee dans ce fichier minimal");
  sendOk(name + " OK");
}

void begin() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  Logger::log("WiFi AP demarre : " + String(WIFI_SSID));
  Logger::log("IP interface : " + WiFi.softAPIP().toString());

  server.on("/", []() { server.send(200, "text/html", WebPages::home()); });
  server.on("/soutenance1", []() { server.send(200, "text/html", WebPages::soutenance1()); });
  server.on("/soutenance2", []() { server.send(200, "text/html", WebPages::soutenance2()); });
  server.on("/soutenance2/escalier", []() { server.send(200, "text/html", WebPages::soutenance2Escalier()); });
  server.on("/soutenance2/cercle", []() { server.send(200, "text/html", WebPages::soutenance2Cercle()); });
  server.on("/soutenance2/rose-des-vents", []() { server.send(200, "text/html", WebPages::soutenance2RoseDesVents()); });

  server.on("/api/status", []() { server.send(200, "application/json", jsonStatus()); });
  server.on("/api/logs", []() { server.send(200, "text/plain", Logger::getBuffer()); });
  server.on("/api/manual", handleManual);
  server.on("/api/stop", handleStop);
  server.on("/api/reset-encoders", handleResetEncoders);
  server.on("/api/reset-imu", []() { Sensors::resetGyroYaw(); sendOk(); });
  server.on("/api/calibrate-mag", []() { Sensors::startMagCalibration(); sendOk(); });
  server.on("/api/clear-mag-calibration", []() { Sensors::clearMagCalibration(); sendOk(); });

  // Routes minimales pour eviter les erreurs HTTP si les pages les appellent.
  server.on("/api/s2/escalier/start", []() { handleNotImplemented("Escalier start"); });
  server.on("/api/s2/escalier/stop", []() { handleStop(); });
  server.on("/api/s2/escalier/set", []() { handleNotImplemented("Escalier set"); });
  server.on("/api/s2/escalier/save", []() { handleNotImplemented("Escalier save"); });
  server.on("/api/s2/escalier/config", []() { server.send(200, "application/json", "{}"); });
  server.on("/api/s2/escalier/status", []() { server.send(200, "application/json", "{\"state\":\"IDLE\"}"); });
  server.on("/api/s2/cercle/start", []() { handleNotImplemented("Cercle start"); });
  server.on("/api/s2/rose/start", []() { handleNotImplemented("Rose start"); });

  server.begin();
  Logger::log("Serveur web demarre");
}

void handleClient() {
  server.handleClient();
}

}

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "web_server_app.h"
#include "web_pages.h"
#include "app_state.h"
#include "moteurs.h"
#include "encodeurs.h"
#include "odometry.h"
#include "sensors.h"
#include "logger.h"
#include "s2_escalier.h"

static const char* DRAWBOT_AP_SSID = "DRAWBOT_#S4S25-G-2314";
static const char* DRAWBOT_AP_PASS = "!12345678!";

static WebServer server(80);

static String robotStateName() {
  if (motorState.pwmLeft == 0 && motorState.pwmRight == 0) return "STOP";
  if (motorState.pwmLeft > 0 && motorState.pwmRight > 0) return "AVANCE";
  if (motorState.pwmLeft < 0 && motorState.pwmRight < 0) return "RECULE";
  if (motorState.pwmLeft < 0 && motorState.pwmRight > 0) return "ROTATION_GAUCHE";
  if (motorState.pwmLeft > 0 && motorState.pwmRight < 0) return "ROTATION_DROITE";
  return "MANUEL";
}

static void setRobotMotors(int leftPwm, int rightPwm) {
  motorState.pwmLeft = constrain(leftPwm, -255, 255);
  motorState.pwmRight = constrain(rightPwm, -255, 255);
  motorState.mode = "MANUAL";

  setMotors(motorState.pwmLeft, motorState.pwmRight);
}

static void stopRobot() {
  motorState.pwmLeft = 0;
  motorState.pwmRight = 0;
  motorState.mode = "IDLE";
  stopMotors();
}

static void handleHome() {
  server.send(200, "text/html", WebPages::home());
}

static void handleSoutenance1() {
  server.send(200, "text/html", WebPages::soutenance1());
}

static void handleSoutenance2() {
  server.send(200, "text/html", WebPages::soutenance2());
}

static void handleSoutenance2Escalier() {
  server.send(200, "text/html", WebPages::soutenance2Escalier());
}

static void handleSoutenance2Cercle() {
  server.send(200, "text/html", WebPages::soutenance2Cercle());
}

static void handleSoutenance2RoseDesVents() {
  server.send(200, "text/html", WebPages::soutenance2RoseDesVents());
}

static void handleStatus() {
  String magCalibState = "NON";
  if (sensorState.magCalibrationRunning) magCalibState = "EN COURS";
  if (sensorState.magCalibrationDone) magCalibState = "OK";

  String json = "{";

  json += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  json += "\"uptime\":" + String(millis() / 1000.0f, 2) + ",";
  json += "\"state\":\"" + robotStateName() + "\",";
  json += "\"mode\":\"" + motorState.mode + "\",";

  json += "\"pwmL\":" + String(motorState.pwmLeft) + ",";
  json += "\"pwmR\":" + String(motorState.pwmRight) + ",";

  json += "\"accX\":" + String(sensorState.accX, 4) + ",";
  json += "\"accY\":" + String(sensorState.accY, 4) + ",";
  json += "\"accZ\":" + String(sensorState.accZ, 4) + ",";

  json += "\"gyroX\":" + String(sensorState.gyroX, 3) + ",";
  json += "\"gyroY\":" + String(sensorState.gyroY, 3) + ",";
  json += "\"gyroZ\":" + String(sensorState.gyroZ, 3) + ",";
  json += "\"yawGyro\":" + String(sensorState.yawGyroDeg, 3) + ",";

  json += "\"magX\":" + String(sensorState.magX, 3) + ",";
  json += "\"magY\":" + String(sensorState.magY, 3) + ",";
  json += "\"magZ\":" + String(sensorState.magZ, 3) + ",";
  json += "\"headingMag\":" + String(sensorState.headingMagDeg, 3) + ",";
  json += "\"magCalib\":\"" + magCalibState + "\",";
  json += "\"magCalibrationLoaded\":" + String(sensorState.magCalibrationLoaded ? "true" : "false") + ",";
  json += "\"magOffsetX\":" + String(sensorState.magOffsetX, 4) + ",";
  json += "\"magOffsetY\":" + String(sensorState.magOffsetY, 4) + ",";
  json += "\"magScaleX\":" + String(sensorState.magScaleX, 4) + ",";
  json += "\"magScaleY\":" + String(sensorState.magScaleY, 4) + ",";

  json += "\"ticksL\":" + String(getLeftEncoderTicks()) + ",";
  json += "\"ticksR\":" + String(getRightEncoderTicks()) + ",";

  json += "\"distL\":" + String(getLeftDistanceCm(), 3) + ",";
  json += "\"distR\":" + String(getRightDistanceCm(), 3) + ",";

  json += "\"speedL\":" + String(odometryState.speedLeftCms, 3) + ",";
  json += "\"speedR\":" + String(odometryState.speedRightCms, 3) + ",";

  json += "\"odoX\":" + String(odometryState.xCm, 3) + ",";
  json += "\"odoY\":" + String(odometryState.yCm, 3) + ",";
  json += "\"odoTheta\":" + String(Odometry::normalizeAngleDeg(Odometry::radToDeg(odometryState.thetaRad)), 3);

  json += "}";

  server.send(200, "application/json", json);
}

static void handleLogs() {
  server.send(200, "text/plain", Logger::getBuffer());
}

static void handleManual() {
  int l = server.hasArg("l") ? server.arg("l").toInt() : 0;
  int r = server.hasArg("r") ? server.arg("r").toInt() : 0;

  setRobotMotors(l, r);

  Logger::log("Commande manuelle : L=" + String(motorState.pwmLeft) + " R=" + String(motorState.pwmRight));
  server.send(200, "text/plain", "OK");
}

static void handleStop() {
  stopRobot();
  Logger::log("STOP moteurs");
  server.send(200, "text/plain", "STOP");
}

static void handleResetEncoders() {
  stopRobot();
  Odometry::reset();
  server.send(200, "text/plain", "RESET_ENCODERS");
}

static void handleResetIMU() {
  Sensors::resetGyroYaw();
  server.send(200, "text/plain", "RESET_IMU");
}

static void handleCalibrateMag() {
  Sensors::startMagCalibration();
  server.send(200, "text/plain", "CALIB_MAG");
}

static void handleClearMagCalibration() {
  Sensors::clearMagCalibration();
  server.send(200, "text/plain", "MAG_CALIB_CLEARED");
}

static void handleS2Test() {
  Logger::log("Test soutenance 2 appele");
  server.send(200, "text/plain", "S2_TEST");
}

static bool hasNonEmptyArg(const char* name) {
  return server.hasArg(name) && server.arg(name).length() > 0;
}

static void readS2EscalierArgs(S2Escalier::Config& cfg) {
  if (hasNonEmptyArg("dist1Cm")) cfg.dist1Cm = server.arg("dist1Cm").toFloat();
  if (hasNonEmptyArg("dist2Cm")) cfg.dist2Cm = server.arg("dist2Cm").toFloat();
  if (hasNonEmptyArg("dist3Cm")) cfg.dist3Cm = server.arg("dist3Cm").toFloat();

  if (hasNonEmptyArg("wheelBaseCm")) cfg.wheelBaseCm = server.arg("wheelBaseCm").toFloat();
  if (hasNonEmptyArg("penOffsetCm")) cfg.penOffsetCm = server.arg("penOffsetCm").toFloat();
  if (hasNonEmptyArg("slowZoneCm")) cfg.slowZoneCm = server.arg("slowZoneCm").toFloat();
  if (hasNonEmptyArg("endToleranceCm")) cfg.endToleranceCm = server.arg("endToleranceCm").toFloat();

  if (hasNonEmptyArg("minPwm")) cfg.minPwm = server.arg("minPwm").toInt();
  if (hasNonEmptyArg("maxPwm")) cfg.maxPwm = server.arg("maxPwm").toInt();
  if (hasNonEmptyArg("maxWheelSpeedCms")) cfg.maxWheelSpeedCms = server.arg("maxWheelSpeedCms").toFloat();

  if (hasNonEmptyArg("speedA")) cfg.pidA.penSpeedCms = server.arg("speedA").toFloat();
  if (hasNonEmptyArg("kpA")) cfg.pidA.kpLat = server.arg("kpA").toFloat();
  if (hasNonEmptyArg("kiA")) cfg.pidA.kiLat = server.arg("kiA").toFloat();
  if (hasNonEmptyArg("kdA")) cfg.pidA.kdLat = server.arg("kdA").toFloat();
  if (hasNonEmptyArg("maxCorrA")) cfg.pidA.maxLatCorrectionCms = server.arg("maxCorrA").toFloat();

  if (hasNonEmptyArg("speedB")) cfg.pidB.penSpeedCms = server.arg("speedB").toFloat();
  if (hasNonEmptyArg("kpB")) cfg.pidB.kpLat = server.arg("kpB").toFloat();
  if (hasNonEmptyArg("kiB")) cfg.pidB.kiLat = server.arg("kiB").toFloat();
  if (hasNonEmptyArg("kdB")) cfg.pidB.kdLat = server.arg("kdB").toFloat();
  if (hasNonEmptyArg("maxCorrB")) cfg.pidB.maxLatCorrectionCms = server.arg("maxCorrB").toFloat();

  if (hasNonEmptyArg("integralLimit")) cfg.integralLimit = server.arg("integralLimit").toFloat();
  if (hasNonEmptyArg("headingSource")) cfg.headingSource = server.arg("headingSource").toInt();

  // Compatibilite avec l'ancienne page qui envoyait d1/d2/d3
  if (hasNonEmptyArg("d1")) cfg.dist1Cm = server.arg("d1").toFloat();
  if (hasNonEmptyArg("d2")) cfg.dist2Cm = server.arg("d2").toFloat();
  if (hasNonEmptyArg("d3")) cfg.dist3Cm = server.arg("d3").toFloat();
}

static void handleS2EscalierStart() {
  S2Escalier::Config cfg = S2Escalier::getConfig();
  readS2EscalierArgs(cfg);
  S2Escalier::setConfig(cfg);
  S2Escalier::start();

  Logger::log("Demande sequence ESCALIER 2 PID");
  server.send(200, "text/plain", "S2_ESCALIER_STARTED");
}

static void handleS2EscalierConfig() {
  server.send(200, "application/json", S2Escalier::configJson());
}

static void handleS2EscalierStatus() {
  server.send(200, "application/json", S2Escalier::statusJson());
}

static void handleS2EscalierSet() {
  S2Escalier::Config cfg = S2Escalier::getConfig();
  readS2EscalierArgs(cfg);
  S2Escalier::setConfig(cfg);
  server.send(200, "text/plain", "OK");
}

static void handleS2EscalierSave() {
  S2Escalier::saveConfig();
  server.send(200, "text/plain", "SAVED");
}

static void handleS2EscalierStop() {
  S2Escalier::stop();
  server.send(200, "text/plain", "STOPPED");
}

static void handleS2CercleStart() {
  float radius = server.hasArg("r") ? server.arg("r").toFloat() : 10.0f;
  int segments = server.hasArg("n") ? server.arg("n").toInt() : 36;
  int pwm = server.hasArg("pwm") ? server.arg("pwm").toInt() : 170;

  Logger::log("Demande sequence CERCLE");
  Logger::log("rayon=" + String(radius, 1) +
              " segments=" + String(segments) +
              " pwm=" + String(pwm));

  server.send(200, "text/plain", "S2_CERCLE_START");
}

static void handleS2RoseStart() {
  float length = server.hasArg("length") ? server.arg("length").toFloat() : 10.0f;
  int pwm = server.hasArg("pwm") ? server.arg("pwm").toInt() : 150;

  Logger::log("Demande sequence ROSE DES VENTS");
  Logger::log("longueur=" + String(length, 1) +
              " pwm=" + String(pwm) +
              " capMag=" + String(sensorState.headingMagDeg, 1));

  server.send(200, "text/plain", "S2_ROSE_START");
}

namespace WebApp {
  void begin() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(DRAWBOT_AP_SSID, DRAWBOT_AP_PASS);

    Logger::log("WiFi AP demarre");
    Logger::log("SSID : " + String(DRAWBOT_AP_SSID));
    Logger::log("Mot de passe : " + String(DRAWBOT_AP_PASS));
    Logger::log("Adresse web : http://" + WiFi.softAPIP().toString());

    server.on("/", handleHome);
    server.on("/soutenance1", handleSoutenance1);
    server.on("/soutenance2", handleSoutenance2);
    server.on("/soutenance2/escalier", handleSoutenance2Escalier);
    server.on("/soutenance2/cercle", handleSoutenance2Cercle);
    server.on("/soutenance2/rose-des-vents", handleSoutenance2RoseDesVents);
    server.on("/api/status", handleStatus);
    server.on("/api/logs", handleLogs);
    server.on("/api/manual", handleManual);
    server.on("/api/stop", handleStop);
    server.on("/api/reset-encoders", handleResetEncoders);
    server.on("/api/reset-imu", handleResetIMU);
    server.on("/api/calibrate-mag", handleCalibrateMag);
    server.on("/api/clear-mag-calibration", handleClearMagCalibration);

    server.on("/api/s2/test", handleS2Test);
    server.on("/api/s2/escalier/start", handleS2EscalierStart);
    server.on("/api/s2/escalier/config", handleS2EscalierConfig);
    server.on("/api/s2/escalier/status", handleS2EscalierStatus);
    server.on("/api/s2/escalier/set", handleS2EscalierSet);
    server.on("/api/s2/escalier/save", handleS2EscalierSave);
    server.on("/api/s2/escalier/stop", handleS2EscalierStop);
    server.on("/api/s2/cercle/start", handleS2CercleStart);
    server.on("/api/s2/rose/start", handleS2RoseStart);

    server.begin();
    Logger::log("Serveur web lance sur le port 80");
  }

  void handleClient() {
    server.handleClient();
  }
}
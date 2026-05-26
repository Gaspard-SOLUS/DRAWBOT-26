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
#include "s2_cercle.h"

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

  // Compatibilite avec les anciens noms utilises par la page initiale.
  if (hasNonEmptyArg("d1")) cfg.dist1Cm = server.arg("d1").toFloat();
  if (hasNonEmptyArg("d2")) cfg.dist2Cm = server.arg("d2").toFloat();
  if (hasNonEmptyArg("d3")) cfg.dist3Cm = server.arg("d3").toFloat();

  if (hasNonEmptyArg("wheelBaseCm")) cfg.wheelBaseCm = server.arg("wheelBaseCm").toFloat();
  if (hasNonEmptyArg("penOffsetCm")) cfg.penOffsetCm = server.arg("penOffsetCm").toFloat();

  // La page precedente envoyait penSpeedCms. Dans cette version, c'est lineSpeedCms.
  if (hasNonEmptyArg("lineSpeedCms")) cfg.lineSpeedCms = server.arg("lineSpeedCms").toFloat();
  if (hasNonEmptyArg("penSpeedCms")) cfg.lineSpeedCms = server.arg("penSpeedCms").toFloat();
  if (hasNonEmptyArg("slowZoneCm")) cfg.slowZoneCm = server.arg("slowZoneCm").toFloat();
  if (hasNonEmptyArg("endToleranceCm")) cfg.endToleranceCm = server.arg("endToleranceCm").toFloat();
  if (hasNonEmptyArg("cornerPauseMs")) cfg.cornerPauseMs = (unsigned long)server.arg("cornerPauseMs").toInt();

  if (hasNonEmptyArg("minPwm")) cfg.minPwm = server.arg("minPwm").toInt();
  if (hasNonEmptyArg("maxPwm")) cfg.maxPwm = server.arg("maxPwm").toInt();
  if (hasNonEmptyArg("maxWheelSpeedCms")) cfg.maxWheelSpeedCms = server.arg("maxWheelSpeedCms").toFloat();
  if (hasNonEmptyArg("pwmRampStep")) cfg.pwmRampStep = server.arg("pwmRampStep").toInt();

  // Ancien PID lateral : on le mappe vers le maintien de cap si la vieille page l'envoie encore.
  if (hasNonEmptyArg("kpHeading")) cfg.kpHeading = server.arg("kpHeading").toFloat();
  if (hasNonEmptyArg("kiHeading")) cfg.kiHeading = server.arg("kiHeading").toFloat();
  if (hasNonEmptyArg("kdHeading")) cfg.kdHeading = server.arg("kdHeading").toFloat();
  if (hasNonEmptyArg("maxOmegaRadS")) cfg.maxOmegaRadS = server.arg("maxOmegaRadS").toFloat();
  if (hasNonEmptyArg("headingIntegralLimit")) cfg.headingIntegralLimit = server.arg("headingIntegralLimit").toFloat();

  if (hasNonEmptyArg("kpLat")) cfg.kpHeading = server.arg("kpLat").toFloat();
  if (hasNonEmptyArg("kiLat")) cfg.kiHeading = server.arg("kiLat").toFloat();
  if (hasNonEmptyArg("kdLat")) cfg.kdHeading = server.arg("kdLat").toFloat();
  if (hasNonEmptyArg("maxLatCorrectionCms")) cfg.maxOmegaRadS = server.arg("maxLatCorrectionCms").toFloat();
  if (hasNonEmptyArg("integralLimit")) cfg.headingIntegralLimit = server.arg("integralLimit").toFloat();

  if (hasNonEmptyArg("pivotSlowPwm")) cfg.pivotSlowPwm = server.arg("pivotSlowPwm").toInt();
  if (hasNonEmptyArg("pivotRatio")) cfg.pivotRatio = server.arg("pivotRatio").toFloat();
  if (hasNonEmptyArg("pivotMinPwm")) cfg.pivotMinPwm = server.arg("pivotMinPwm").toInt();
  if (hasNonEmptyArg("pivotMaxPwm")) cfg.pivotMaxPwm = server.arg("pivotMaxPwm").toInt();
  if (hasNonEmptyArg("pivotAngleToleranceDeg")) cfg.pivotAngleToleranceDeg = server.arg("pivotAngleToleranceDeg").toFloat();
  if (hasNonEmptyArg("pivotSlowdownDeg")) cfg.pivotSlowdownDeg = server.arg("pivotSlowdownDeg").toFloat();

  if (hasNonEmptyArg("headingSource")) cfg.headingSource = server.arg("headingSource").toInt();
}

static void handleS2EscalierStart() {
  S2Escalier::Config cfg = S2Escalier::getConfig();
  readS2EscalierArgs(cfg);
  S2Escalier::setConfig(cfg);
  S2Escalier::start();

  Logger::log("Demande sequence ESCALIER : start");
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
  server.send(200, "text/plain", "S2_ESCALIER_CONFIG_SET");
}

static void handleS2EscalierSave() {
  S2Escalier::saveConfig();
  server.send(200, "text/plain", "S2_ESCALIER_CONFIG_SAVED");
}

static void handleS2EscalierStop() {
  S2Escalier::stop();
  server.send(200, "text/plain", "S2_ESCALIER_STOPPED");
}

static void readS2CercleArgs(S2Cercle::Config& cfg) {
  if (hasNonEmptyArg("radiusCm")) cfg.radiusCm = server.arg("radiusCm").toFloat();
  if (hasNonEmptyArg("r")) cfg.radiusCm = server.arg("r").toFloat();
  if (hasNonEmptyArg("radiusScale")) cfg.radiusScale = server.arg("radiusScale").toFloat();
  if (hasNonEmptyArg("radiusOffsetCm")) cfg.radiusOffsetCm = server.arg("radiusOffsetCm").toFloat();

  if (hasNonEmptyArg("wheelBaseCm")) cfg.wheelBaseCm = server.arg("wheelBaseCm").toFloat();
  if (hasNonEmptyArg("penOffsetCm")) cfg.penOffsetCm = server.arg("penOffsetCm").toFloat();
  if (hasNonEmptyArg("direction")) cfg.direction = server.arg("direction").toInt();
  if (hasNonEmptyArg("outerWheelSpeedCms")) cfg.outerWheelSpeedCms = server.arg("outerWheelSpeedCms").toFloat();
  if (hasNonEmptyArg("closureFactor")) cfg.closureFactor = server.arg("closureFactor").toFloat();
  if (hasNonEmptyArg("stopTurnFactor")) cfg.stopTurnFactor = server.arg("stopTurnFactor").toFloat();
  if (hasNonEmptyArg("endSlowdownEnabled")) cfg.endSlowdownEnabled = server.arg("endSlowdownEnabled").toInt();
  if (hasNonEmptyArg("endSlowdownStart")) cfg.endSlowdownStart = server.arg("endSlowdownStart").toFloat();
  if (hasNonEmptyArg("endSlowdownMinScale")) cfg.endSlowdownMinScale = server.arg("endSlowdownMinScale").toFloat();

  if (hasNonEmptyArg("minPwm")) cfg.minPwm = server.arg("minPwm").toInt();
  if (hasNonEmptyArg("maxPwm")) cfg.maxPwm = server.arg("maxPwm").toInt();
  if (hasNonEmptyArg("pwmRampStep")) cfg.pwmRampStep = server.arg("pwmRampStep").toInt();

  if (hasNonEmptyArg("kpSpeed")) cfg.kpSpeed = server.arg("kpSpeed").toFloat();
  if (hasNonEmptyArg("kiSpeed")) cfg.kiSpeed = server.arg("kiSpeed").toFloat();
  if (hasNonEmptyArg("kdSpeed")) cfg.kdSpeed = server.arg("kdSpeed").toFloat();
  if (hasNonEmptyArg("integralLimit")) cfg.integralLimit = server.arg("integralLimit").toFloat();
  if (hasNonEmptyArg("kFF")) cfg.kFF = server.arg("kFF").toFloat();
  if (hasNonEmptyArg("minReliableSpeedCms")) cfg.minReliableSpeedCms = server.arg("minReliableSpeedCms").toFloat();
  if (hasNonEmptyArg("pulsePeriodMs")) cfg.pulsePeriodMs = (unsigned long)server.arg("pulsePeriodMs").toInt();
  if (hasNonEmptyArg("speedFilterAlpha")) cfg.speedFilterAlpha = server.arg("speedFilterAlpha").toFloat();
  if (hasNonEmptyArg("maxPidCorrectionPwm")) cfg.maxPidCorrectionPwm = server.arg("maxPidCorrectionPwm").toFloat();
  if (hasNonEmptyArg("speedDeadbandCms")) cfg.speedDeadbandCms = server.arg("speedDeadbandCms").toFloat();
  if (hasNonEmptyArg("ratioTrimKp")) cfg.ratioTrimKp = server.arg("ratioTrimKp").toFloat();
  if (hasNonEmptyArg("maxRatioTrimPwm")) cfg.maxRatioTrimPwm = server.arg("maxRatioTrimPwm").toFloat();
}

static void handleS2CercleStart() {
  S2Cercle::Config cfg = S2Cercle::getConfig();
  readS2CercleArgs(cfg);
  S2Cercle::setConfig(cfg);
  S2Cercle::start();

  Logger::log("Demande sequence CERCLE : start");
  server.send(200, "text/plain", "S2_CERCLE_STARTED");
}

static void handleS2CercleConfig() {
  server.send(200, "application/json", S2Cercle::configJson());
}

static void handleS2CercleStatus() {
  server.send(200, "application/json", S2Cercle::statusJson());
}

static void handleS2CercleSet() {
  S2Cercle::Config cfg = S2Cercle::getConfig();
  readS2CercleArgs(cfg);
  S2Cercle::setConfig(cfg);
  server.send(200, "text/plain", "S2_CERCLE_CONFIG_SET");
}

static void handleS2CercleSave() {
  S2Cercle::saveConfig();
  server.send(200, "text/plain", "S2_CERCLE_CONFIG_SAVED");
}

static void handleS2CercleStop() {
  S2Cercle::stop();
  server.send(200, "text/plain", "S2_CERCLE_STOPPED");
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
    server.on("/api/s2/cercle/config", handleS2CercleConfig);
    server.on("/api/s2/cercle/status", handleS2CercleStatus);
    server.on("/api/s2/cercle/set", handleS2CercleSet);
    server.on("/api/s2/cercle/save", handleS2CercleSave);
    server.on("/api/s2/cercle/stop", handleS2CercleStop);
    server.on("/api/s2/rose/start", handleS2RoseStart);

    server.begin();
    Logger::log("Serveur web lance sur le port 80");
  }

  void handleClient() {
    server.handleClient();
  }
}
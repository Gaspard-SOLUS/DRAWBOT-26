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
#include "pen_inverse_follower.h"
#include "soutenance2.h"
#include "page_s2_cercle.h"
#include "trajectory_generator.h"
#include "circle_trajectory_helper.h"

// ==================================================
// WIFI ESP32 EN POINT D'ACCES
// ==================================================
static const char* DRAWBOT_AP_SSID = "DRAWBOT_#S4S25-G-2314";
static const char* DRAWBOT_AP_PASS = "!12345678!";

static WebServer server(80);

// ==================================================
// ETAT ROBOT / COMMANDES BAS NIVEAU
// ==================================================
static String robotStateName() {
  if (motorState.pwmLeft == 0 && motorState.pwmRight == 0) return "STOP";
  if (motorState.pwmLeft > 0 && motorState.pwmRight > 0) return "AVANCE";
  if (motorState.pwmLeft < 0 && motorState.pwmRight < 0) return "RECULE";
  if (motorState.pwmLeft < 0 && motorState.pwmRight > 0) return "ROTATION_GAUCHE";
  if (motorState.pwmLeft > 0 && motorState.pwmRight < 0) return "ROTATION_DROITE";
  return "MANUEL";
}

static void setRobotMotors(int leftPwm, int rightPwm) {
  // Si une commande manuelle arrive, on arrête le suivi automatique du stylo.
  PenInverseFollower::stop();
  Soutenance2::stop();

  motorState.pwmLeft = constrain(leftPwm, -255, 255);
  motorState.pwmRight = constrain(rightPwm, -255, 255);
  motorState.mode = "MANUAL";

  setMotors(motorState.pwmLeft, motorState.pwmRight);
}

static void stopRobot() {
  PenInverseFollower::stop();
  Soutenance2::stop();

  motorState.pwmLeft = 0;
  motorState.pwmRight = 0;
  motorState.mode = "IDLE";

  stopMotors();
}

// ==================================================
// PAGES HTML
// ==================================================
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
  server.send(200, "text/html", PageS2Cercle::html());
}

static void handleSoutenance2RoseDesVents() {
  server.send(200, "text/html", WebPages::soutenance2RoseDesVents());
}

static void handleSimulation() {
  server.send(200, "text/html", WebPages::simulation());
}

// ==================================================
// CONFIGURATION DU SUIVI INVERSE DU STYLO
// ==================================================
static void applyPenInverseConfigFromRequest() {
  PenInverseFollower::Config cfg = PenInverseFollower::getConfig();

  if (server.hasArg("wheelBase")) cfg.wheelBaseCm = server.arg("wheelBase").toFloat();
  if (server.hasArg("penOffset")) cfg.penOffsetCm = server.arg("penOffset").toFloat();

  if (server.hasArg("distanceScale")) cfg.distanceScale = server.arg("distanceScale").toFloat();

  if (server.hasArg("penSpeed")) cfg.penSpeedCms = server.arg("penSpeed").toFloat();
  if (server.hasArg("lineGain")) cfg.lineGain = server.arg("lineGain").toFloat();
  if (server.hasArg("targetGain")) cfg.targetGain = server.arg("targetGain").toFloat();
  if (server.hasArg("lookahead")) cfg.lookaheadCm = server.arg("lookahead").toFloat();

  if (server.hasArg("penSpeedMax")) cfg.penSpeedMaxCms = server.arg("penSpeedMax").toFloat();
  if (server.hasArg("wheelSpeedMax")) cfg.wheelSpeedMaxCms = server.arg("wheelSpeedMax").toFloat();

  if (server.hasArg("kp")) cfg.kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) cfg.ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) cfg.kd = server.arg("kd").toFloat();
  if (server.hasArg("iLimit")) cfg.integralLimit = server.arg("iLimit").toFloat();

  if (server.hasArg("coefL")) cfg.coefLeftCmsPerPwm = server.arg("coefL").toFloat();
  if (server.hasArg("coefR")) cfg.coefRightCmsPerPwm = server.arg("coefR").toFloat();

  if (server.hasArg("minPwm")) cfg.minPwm = server.arg("minPwm").toInt();
  if (server.hasArg("pwmSlewStep")) cfg.pwmSlewStep = server.arg("pwmSlewStep").toInt();
  if (server.hasArg("pwmDither")) cfg.pwmDither = (server.arg("pwmDither").toInt() != 0);
  if (server.hasArg("allowReverse")) cfg.allowReverse = (server.arg("allowReverse").toInt() != 0);
  if (server.hasArg("minForwardSpeed")) cfg.minForwardSpeedCms = server.arg("minForwardSpeed").toFloat();

  if (server.hasArg("segTol")) cfg.segmentToleranceCm = server.arg("segTol").toFloat();
  if (server.hasArg("stairMiddleExtra")) cfg.stairMiddleExtraCm = server.arg("stairMiddleExtra").toFloat();
  if (server.hasArg("stairAngleTrim")) cfg.stairSecondAngleTrimDeg = server.arg("stairAngleTrim").toFloat();

  PenInverseFollower::setConfig(cfg);
}

static String penInverseConfigJson() {
  PenInverseFollower::Config cfg = PenInverseFollower::getConfig();

  String json = "{";

  json += "\"wheelBase\":" + String(cfg.wheelBaseCm, 3) + ",";
  json += "\"penOffset\":" + String(cfg.penOffsetCm, 3) + ",";

  json += "\"distanceScale\":" + String(cfg.distanceScale, 3) + ",";
  json += "\"penSpeed\":" + String(cfg.penSpeedCms, 3) + ",";
  json += "\"lineGain\":" + String(cfg.lineGain, 3) + ",";
  json += "\"targetGain\":" + String(cfg.targetGain, 3) + ",";
  json += "\"lookahead\":" + String(cfg.lookaheadCm, 3) + ",";

  json += "\"penSpeedMax\":" + String(cfg.penSpeedMaxCms, 3) + ",";
  json += "\"wheelSpeedMax\":" + String(cfg.wheelSpeedMaxCms, 3) + ",";

  json += "\"kp\":" + String(cfg.kp, 4) + ",";
  json += "\"ki\":" + String(cfg.ki, 4) + ",";
  json += "\"kd\":" + String(cfg.kd, 4) + ",";
  json += "\"iLimit\":" + String(cfg.integralLimit, 3) + ",";

  json += "\"coefL\":" + String(cfg.coefLeftCmsPerPwm, 5) + ",";
  json += "\"coefR\":" + String(cfg.coefRightCmsPerPwm, 5) + ",";

  json += "\"minPwm\":" + String(cfg.minPwm) + ",";
  json += "\"pwmSlewStep\":" + String(cfg.pwmSlewStep) + ",";
  json += "\"pwmDither\":" + String(cfg.pwmDither ? "true" : "false") + ",";
  json += "\"allowReverse\":" + String(cfg.allowReverse ? "true" : "false") + ",";
  json += "\"minForwardSpeed\":" + String(cfg.minForwardSpeedCms, 3) + ",";

  json += "\"segTol\":" + String(cfg.segmentToleranceCm, 3) + ",";
  json += "\"stairMiddleExtra\":" + String(cfg.stairMiddleExtraCm, 3) + ",";
  json += "\"stairAngleTrim\":" + String(cfg.stairSecondAngleTrimDeg, 3);

  json += "}";

  return json;
}

static String penInverseStatusJson() {
  PenInverseFollower::Status st = PenInverseFollower::getStatus();

  String json = "{";

  json += "\"running\":" + String(st.running ? "true" : "false") + ",";
  json += "\"finished\":" + String(st.finished ? "true" : "false") + ",";

  json += "\"currentSegment\":" + String(st.currentSegment) + ",";
  json += "\"segmentCount\":" + String(st.segmentCount) + ",";

  json += "\"penX\":" + String(st.penX, 3) + ",";
  json += "\"penY\":" + String(st.penY, 3) + ",";

  json += "\"targetX\":" + String(st.targetX, 3) + ",";
  json += "\"targetY\":" + String(st.targetY, 3) + ",";

  json += "\"lateralError\":" + String(st.lateralErrorCm, 3) + ",";
  json += "\"maxLateralError\":" + String(st.maxLateralErrorCm, 3) + ",";

  json += "\"progress\":" + String(st.progressCm, 3) + ",";
  json += "\"segmentLength\":" + String(st.segmentLengthCm, 3) + ",";

  json += "\"vPenX\":" + String(st.vPenX, 3) + ",";
  json += "\"vPenY\":" + String(st.vPenY, 3) + ",";

  json += "\"vCenter\":" + String(st.vCenterCms, 3) + ",";
  json += "\"omega\":" + String(st.omegaRadS, 3) + ",";

  json += "\"vLeft\":" + String(st.vLeftCms, 3) + ",";
  json += "\"vRight\":" + String(st.vRightCms, 3) + ",";

  json += "\"pwmLeft\":" + String(st.pwmLeft) + ",";
  json += "\"pwmRight\":" + String(st.pwmRight);

  json += "}";

  return json;
}

// ==================================================
// API STATUS GLOBAL
// ==================================================
static void handleStatus() {
  String magCalibState = "NON";
  if (sensorState.magCalibrationRunning) magCalibState = "EN COURS";
  if (sensorState.magCalibrationDone) magCalibState = "OK";

  PenInverseFollower::Status pf = PenInverseFollower::getStatus();

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
  json += "\"penX\":" + String(odometryState.penXCm, 3) + ",";
  json += "\"penY\":" + String(odometryState.penYCm, 3) + ",";
  json += "\"odoTheta\":" + String(Odometry::normalizeAngleDeg(Odometry::radToDeg(odometryState.thetaRad)), 3) + ",";

  json += "\"followerRunning\":" + String(pf.running ? "true" : "false") + ",";
  json += "\"followerFinished\":" + String(pf.finished ? "true" : "false") + ",";
  json += "\"followerSegment\":" + String(pf.currentSegment) + ",";
  json += "\"followerSegmentCount\":" + String(pf.segmentCount) + ",";

  json += "\"followerPenX\":" + String(pf.penX, 3) + ",";
  json += "\"followerPenY\":" + String(pf.penY, 3) + ",";
  json += "\"followerTargetX\":" + String(pf.targetX, 3) + ",";
  json += "\"followerTargetY\":" + String(pf.targetY, 3) + ",";

  json += "\"followerLateralError\":" + String(pf.lateralErrorCm, 3) + ",";
  json += "\"followerMaxLateralError\":" + String(pf.maxLateralErrorCm, 3) + ",";
  json += "\"followerProgress\":" + String(pf.progressCm, 3) + ",";
  json += "\"followerSegmentLength\":" + String(pf.segmentLengthCm, 3) + ",";

  json += "\"followerVPenX\":" + String(pf.vPenX, 3) + ",";
  json += "\"followerVPenY\":" + String(pf.vPenY, 3) + ",";
  json += "\"followerVCenter\":" + String(pf.vCenterCms, 3) + ",";
  json += "\"followerOmega\":" + String(pf.omegaRadS, 3) + ",";

  json += "\"followerVLeft\":" + String(pf.vLeftCms, 3) + ",";
  json += "\"followerVRight\":" + String(pf.vRightCms, 3) + ",";
  json += "\"followerPwmLeft\":" + String(pf.pwmLeft) + ",";
  json += "\"followerPwmRight\":" + String(pf.pwmRight);

  json += "}";

  server.send(200, "application/json", json);
}

// ==================================================
// API LOGS
// ==================================================
static void handleLogs() {
  server.send(200, "text/plain", Logger::getBuffer());
}

// ==================================================
// API COMMANDES MANUELLES
// ==================================================
static void handleManual() {
  int l = server.hasArg("l") ? server.arg("l").toInt() : 0;
  int r = server.hasArg("r") ? server.arg("r").toInt() : 0;

  setRobotMotors(l, r);

  Logger::log("Commande manuelle : L=" + String(motorState.pwmLeft) +
              " R=" + String(motorState.pwmRight));

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

// ==================================================
// API MAGNETOMETRE
// ==================================================
static void handleCalibrateMag() {
  Sensors::startMagCalibration();
  server.send(200, "text/plain", "CALIB_MAG");
}

static void handleClearMagCalibration() {
  Sensors::clearMagCalibration();
  server.send(200, "text/plain", "MAG_CALIB_CLEARED");
}

// ==================================================
// API SOUTENANCE 2 - ESCALIER / SUIVI DU STYLO
// ==================================================
static void handleS2EscalierConfigGet() {
  server.send(200, "application/json", penInverseConfigJson());
}

static void handleS2EscalierConfigSet() {
  applyPenInverseConfigFromRequest();
  server.send(200, "text/plain", "S2_ESCALIER_CONFIG_SET");
}

static void handleS2EscalierFollowerStatus() {
  server.send(200, "application/json", penInverseStatusJson());
}

static void handleS2EscalierStartLine() {
  applyPenInverseConfigFromRequest();

  float distance = server.hasArg("d") ? server.arg("d").toFloat() : 20.0f;

  Odometry::reset();
  PenInverseFollower::startLine(distance);

  Logger::log("Test ligne stylo lance : d=" + String(distance, 1));

  server.send(200, "text/plain", "S2_ESCALIER_LINE_START");
}

static void handleS2EscalierStartAngle() {
  applyPenInverseConfigFromRequest();

  float d1 = server.hasArg("d1") ? server.arg("d1").toFloat() : 20.0f;
  float angle = server.hasArg("a") ? server.arg("a").toFloat() : 90.0f;
  float d2 = server.hasArg("d2") ? server.arg("d2").toFloat() : 10.0f;

  Odometry::reset();
  PenInverseFollower::startOneAngle(d1, angle, d2);

  Logger::log("Test un angle stylo lance : d1=" + String(d1, 1) +
              " angle=" + String(angle, 1) +
              " d2=" + String(d2, 1));

  server.send(200, "text/plain", "S2_ESCALIER_ONE_ANGLE_START");
}

static void handleS2EscalierStart() {
  applyPenInverseConfigFromRequest();

  float d1 = server.hasArg("d1") ? server.arg("d1").toFloat() : 20.0f;
  float aL = server.hasArg("aL") ? server.arg("aL").toFloat() : 90.0f;
  float d2 = server.hasArg("d2") ? server.arg("d2").toFloat() : 10.0f;
  float aR = server.hasArg("aR") ? server.arg("aR").toFloat() : 90.0f;
  float d3 = server.hasArg("d3") ? server.arg("d3").toFloat() : 40.0f;

  Odometry::reset();
  PenInverseFollower::startStair(d1, aL, d2, aR, d3);

  Logger::log("Escalier inverse stylo lance depuis API : d1=" + String(d1, 1) +
              " aL=" + String(aL, 1) +
              " d2=" + String(d2, 1) +
              " aR=" + String(aR, 1) +
              " d3=" + String(d3, 1));

  server.send(200, "text/plain", "S2_ESCALIER_INVERSE_START");
}

static void handleS2EscalierStop() {
  PenInverseFollower::stop();
  server.send(200, "text/plain", "S2_ESCALIER_INVERSE_STOP");
}

// ==================================================
// API SOUTENANCE 2 - CERCLE / ROSE DES VENTS
// Pour l'instant, ces routes sont des points d'entrée à compléter plus tard.
// ==================================================
static void handleS2Test() {
  Logger::log("Test soutenance 2 appele");
  server.send(200, "text/plain", "S2_TEST");
}

static bool parseBoolArg(const char* name, bool defaultValue) {
  if (!server.hasArg(name)) return defaultValue;

  String value = server.arg(name);
  value.toLowerCase();

  return value == "1" || value == "true" || value == "yes" || value == "oui";
}

static void handleS2CercleStartSmall() {
  Soutenance2::stop();
  applyPenInverseConfigFromRequest();

  PenInverseFollower::Config cfg = PenInverseFollower::getConfig();

  CircleTrajectoryHelper::CircleRequest req;
  req.radiusCm = server.hasArg("radius") ? server.arg("radius").toFloat() : 5.0f;
  req.segments = server.hasArg("segments") ? server.arg("segments").toInt() : 96;
  req.clockwise = parseBoolArg("clockwise", true);
  req.distanceScale = cfg.distanceScale;

  String startMode = server.hasArg("startMode") ? server.arg("startMode") : "bottom";
  startMode.toLowerCase();

  if (startMode == "right") {
    req.startMode = CircleTrajectoryHelper::StartMode::Right;
  } else {
    req.startMode = CircleTrajectoryHelper::StartMode::Bottom;
  }

  TrajectoryGenerator::Trajectory trajectory;

  if (!CircleTrajectoryHelper::buildSmallCircle(trajectory, req)) {
    Logger::log("Erreur : impossible de generer le petit cercle");
    server.send(400, "text/plain", "CIRCLE_GENERATION_ERROR");
    return;
  }

  float theta0 = 0.0f;
  String initialMode = server.hasArg("initialHeadingMode") ? server.arg("initialHeadingMode") : "tangent";
  initialMode.toLowerCase();

  if (initialMode == "tangent") {
    theta0 = CircleTrajectoryHelper::initialHeadingRad(trajectory);
  } else {
    theta0 = 0.0f;
  }

  float startPenX = trajectory.segments[0].a.x;
  float startPenY = trajectory.segments[0].a.y;

  float baseX = startPenX - cfg.penOffsetCm * cos(theta0);
  float baseY = startPenY - cfg.penOffsetCm * sin(theta0);

  Odometry::resetPose(baseX, baseY, theta0);

  if (!PenInverseFollower::startTrajectory(trajectory)) {
    server.send(500, "text/plain", "FOLLOWER_START_ERROR");
    return;
  }

  Logger::log("Petit cercle lance : r=" + String(req.radiusCm, 2) +
              " cm segments=" + String(req.segments) +
              " sens=" + String(req.clockwise ? "horaire" : "antihoraire") +
              " theta0=" + String(Odometry::normalizeAngleDeg(Odometry::radToDeg(theta0)), 1));

  server.send(200, "text/plain", "S2_SMALL_CIRCLE_START");
}

static void handleS2CercleStart() {
  // Compatibilité avec l'ancienne route.
  handleS2CercleStartSmall();
}

static void handleS2CercleStartSpin() {
  float radius = server.hasArg("radius") ? server.arg("radius").toFloat() : 8.0f;
  bool clockwise = parseBoolArg("clockwise", true);
  int pwm = server.hasArg("pwm") ? server.arg("pwm").toInt() : 200;
  float stopAdvance = server.hasArg("stopAdvance") ? server.arg("stopAdvance").toFloat() : 8.0f;

  if (!Soutenance2::startSpinCircle(radius, clockwise, pwm, stopAdvance)) {
    server.send(400, "text/plain", "S2_CIRCLE_SPIN_ERROR");
    return;
  }

  server.send(200, "text/plain", "S2_CIRCLE_SPIN_START");
}

static void handleS2CercleStop() {
  PenInverseFollower::stop();
  Soutenance2::stop();
  server.send(200, "text/plain", "S2_CERCLE_STOP");
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

// ==================================================
// ROUTES SERVEUR WEB
// ==================================================
namespace WebApp {
  void begin() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(DRAWBOT_AP_SSID, DRAWBOT_AP_PASS);

    Logger::log("WiFi AP demarre");
    Logger::log("SSID : " + String(DRAWBOT_AP_SSID));
    Logger::log("Mot de passe : " + String(DRAWBOT_AP_PASS));
    Logger::log("Adresse web : http://" + WiFi.softAPIP().toString());

    // Pages HTML
    server.on("/", handleHome);
    server.on("/soutenance1", handleSoutenance1);
    server.on("/soutenance2", handleSoutenance2);
    server.on("/soutenance2/escalier", handleSoutenance2Escalier);
    server.on("/soutenance2/cercle", handleSoutenance2Cercle);
    server.on("/soutenance2/rose-des-vents", handleSoutenance2RoseDesVents);
    server.on("/simulation", handleSimulation);

    // API globale
    server.on("/api/status", handleStatus);
    server.on("/api/logs", handleLogs);

    // API commandes
    server.on("/api/manual", handleManual);
    server.on("/api/stop", handleStop);
    server.on("/api/reset-encoders", handleResetEncoders);
    server.on("/api/reset-imu", handleResetIMU);

    // API capteurs
    server.on("/api/calibrate-mag", handleCalibrateMag);
    server.on("/api/clear-mag-calibration", handleClearMagCalibration);

    // API soutenance 2
    server.on("/api/s2/test", handleS2Test);

    // API escalier avec suivi inverse du stylo
    server.on("/api/s2/escalier/config", handleS2EscalierConfigGet);
    server.on("/api/s2/escalier/config/set", handleS2EscalierConfigSet);
    server.on("/api/s2/escalier/follower/status", handleS2EscalierFollowerStatus);

    server.on("/api/s2/escalier/start-line", handleS2EscalierStartLine);
    server.on("/api/s2/escalier/start-angle", handleS2EscalierStartAngle);
    server.on("/api/s2/escalier/start", handleS2EscalierStart);
    server.on("/api/s2/escalier/stop", handleS2EscalierStop);

    // API futures séquences
    server.on("/api/s2/cercle/start", handleS2CercleStart);
    server.on("/api/s2/cercle/start-small", handleS2CercleStartSmall);
    server.on("/api/s2/cercle/start-spin", handleS2CercleStartSpin);
    server.on("/api/s2/cercle/stop", handleS2CercleStop);
    server.on("/api/s2/rose/start", handleS2RoseStart);

    server.begin();

    Logger::log("Serveur web lance sur le port 80");
  }

  void handleClient() {
    server.handleClient();
  }
}

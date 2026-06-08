#include <Arduino.h>
#include <WebServer.h>
#include <math.h>

#include "web_api_s2_escalier.h"
#include "pen_inverse_follower.h"
#include "odometry.h"
#include "encodeurs.h"
#include "logger.h"
#include "robot.h"

namespace {
  WebServer* serverPtr = nullptr;

  WebServer& server() {
    return *serverPtr;
  }

  void applyPenInverseConfigFromRequest() {
    PenInverseFollower::Config cfg = PenInverseFollower::getConfig();

    if (server().hasArg("wheelBase")) cfg.wheelBaseCm = server().arg("wheelBase").toFloat();
    if (server().hasArg("penOffset")) cfg.penOffsetCm = server().arg("penOffset").toFloat();
    if (server().hasArg("distanceScale")) cfg.distanceScale = server().arg("distanceScale").toFloat();

    if (server().hasArg("penSpeed")) cfg.penSpeedCms = server().arg("penSpeed").toFloat();
    if (server().hasArg("lineGain")) cfg.lineGain = server().arg("lineGain").toFloat();
    if (server().hasArg("targetGain")) cfg.targetGain = server().arg("targetGain").toFloat();
    if (server().hasArg("lookahead")) cfg.lookaheadCm = server().arg("lookahead").toFloat();

    if (server().hasArg("penSpeedMax")) cfg.penSpeedMaxCms = server().arg("penSpeedMax").toFloat();
    if (server().hasArg("wheelSpeedMax")) cfg.wheelSpeedMaxCms = server().arg("wheelSpeedMax").toFloat();
    if (server().hasArg("maxOmega")) cfg.maxOmegaRadS = server().arg("maxOmega").toFloat();
    if (server().hasArg("maxNormalCorrection")) cfg.maxNormalCorrectionCms = server().arg("maxNormalCorrection").toFloat();

    if (server().hasArg("allowReverse")) cfg.allowReverse = server().arg("allowReverse").toInt() != 0;
    if (server().hasArg("minForwardSpeed")) cfg.minForwardSpeedCms = server().arg("minForwardSpeed").toFloat();

    if (server().hasArg("kp")) cfg.kp = server().arg("kp").toFloat();
    if (server().hasArg("ki")) cfg.ki = server().arg("ki").toFloat();
    if (server().hasArg("kd")) cfg.kd = server().arg("kd").toFloat();
    if (server().hasArg("iLimit")) cfg.integralLimit = server().arg("iLimit").toFloat();

    if (server().hasArg("coefL")) cfg.coefLeftCmsPerPwm = server().arg("coefL").toFloat();
    if (server().hasArg("coefR")) cfg.coefRightCmsPerPwm = server().arg("coefR").toFloat();

    if (server().hasArg("minPwm")) cfg.minPwm = server().arg("minPwm").toInt();
    if (server().hasArg("minCommandSpeed")) cfg.minCommandSpeedCms = server().arg("minCommandSpeed").toFloat();
    if (server().hasArg("pwmSlewStep")) cfg.pwmSlewStep = server().arg("pwmSlewStep").toInt();

    if (server().hasArg("segTol")) cfg.segmentToleranceCm = server().arg("segTol").toFloat();
    if (server().hasArg("stairMiddleExtra")) cfg.stairMiddleExtraCm = server().arg("stairMiddleExtra").toFloat();
    if (server().hasArg("stairAngleTrim")) cfg.stairSecondAngleTrimDeg = server().arg("stairAngleTrim").toFloat();

    if (server().hasArg("cornerMode")) cfg.cornerMode = server().arg("cornerMode").toInt() != 0;
    if (server().hasArg("cornerApproach")) cfg.cornerApproachCm = server().arg("cornerApproach").toFloat();
    if (server().hasArg("cornerSpeed")) cfg.cornerSpeedCms = server().arg("cornerSpeed").toFloat();
    if (server().hasArg("cornerOmega")) cfg.cornerOmegaRadS = server().arg("cornerOmega").toFloat();
    if (server().hasArg("cornerExitAngle")) cfg.cornerExitAngleDeg = server().arg("cornerExitAngle").toFloat();
    if (server().hasArg("cornerMaxDuration")) cfg.cornerMaxDurationS = server().arg("cornerMaxDuration").toFloat();

    PenInverseFollower::setConfig(cfg);
  }

  String configJson() {
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
    json += "\"maxOmega\":" + String(cfg.maxOmegaRadS, 3) + ",";
    json += "\"maxNormalCorrection\":" + String(cfg.maxNormalCorrectionCms, 3) + ",";

    json += "\"allowReverse\":" + String(cfg.allowReverse ? "true" : "false") + ",";
    json += "\"minForwardSpeed\":" + String(cfg.minForwardSpeedCms, 3) + ",";

    json += "\"kp\":" + String(cfg.kp, 4) + ",";
    json += "\"ki\":" + String(cfg.ki, 4) + ",";
    json += "\"kd\":" + String(cfg.kd, 4) + ",";
    json += "\"iLimit\":" + String(cfg.integralLimit, 3) + ",";

    json += "\"coefL\":" + String(cfg.coefLeftCmsPerPwm, 5) + ",";
    json += "\"coefR\":" + String(cfg.coefRightCmsPerPwm, 5) + ",";
    json += "\"minPwm\":" + String(cfg.minPwm) + ",";
    json += "\"minCommandSpeed\":" + String(cfg.minCommandSpeedCms, 3) + ",";
    json += "\"pwmSlewStep\":" + String(cfg.pwmSlewStep) + ",";

    json += "\"segTol\":" + String(cfg.segmentToleranceCm, 3) + ",";
    json += "\"stairMiddleExtra\":" + String(cfg.stairMiddleExtraCm, 3) + ",";
    json += "\"stairAngleTrim\":" + String(cfg.stairSecondAngleTrimDeg, 3) + ",";

    json += "\"cornerMode\":" + String(cfg.cornerMode ? "true" : "false") + ",";
    json += "\"cornerApproach\":" + String(cfg.cornerApproachCm, 3) + ",";
    json += "\"cornerSpeed\":" + String(cfg.cornerSpeedCms, 3) + ",";
    json += "\"cornerOmega\":" + String(cfg.cornerOmegaRadS, 3) + ",";
    json += "\"cornerExitAngle\":" + String(cfg.cornerExitAngleDeg, 3) + ",";
    json += "\"cornerMaxDuration\":" + String(cfg.cornerMaxDurationS, 3);

    json += "}";
    return json;
  }

  String followerStatusJson() {
    PenInverseFollower::Status st = PenInverseFollower::getStatus();
    String json = "{";

    json += "\"running\":" + String(st.running ? "true" : "false") + ",";
    json += "\"finished\":" + String(st.finished ? "true" : "false") + ",";
    json += "\"currentSegment\":" + String(st.currentSegment) + ",";
    json += "\"segmentCount\":" + String(st.segmentCount) + ",";

    json += "\"phase\":\"" + st.phaseName + "\",";
    json += "\"cornerActive\":" + String(st.cornerActive ? "true" : "false") + ",";
    json += "\"remainingToCorner\":" + String(st.remainingToCornerCm, 3) + ",";
    json += "\"cornerAngleError\":" + String(st.cornerAngleErrorDeg, 3) + ",";

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
    json += "\"pwmRight\":" + String(st.pwmRight) + ",";
    json += "\"targetPwmLeft\":" + String(st.targetPwmLeft) + ",";
    json += "\"targetPwmRight\":" + String(st.targetPwmRight) + ",";

    json += "\"reverseLimited\":" + String(st.reverseLimited ? "true" : "false") + ",";
    json += "\"omegaLimited\":" + String(st.omegaLimited ? "true" : "false") + ",";
    json += "\"normalCorrectionLimited\":" + String(st.normalCorrectionLimited ? "true" : "false");

    json += "}";
    return json;
  }

  void handleConfigGet() {
    server().send(200, "application/json", configJson());
  }

  void handleConfigSet() {
    applyPenInverseConfigFromRequest();
    server().send(200, "text/plain", "S2_ESCALIER_CONFIG_SET");
  }

  void handleConfigSave() {
    applyPenInverseConfigFromRequest();
    bool ok = PenInverseFollower::saveConfig();
    server().send(ok ? 200 : 500, "text/plain", ok ? "S2_ESCALIER_CONFIG_SAVED" : "S2_ESCALIER_CONFIG_SAVE_FAILED");
  }

  void handleConfigLoad() {
    PenInverseFollower::loadConfig();
    server().send(200, "application/json", configJson());
  }

  void handleConfigDefaults() {
    PenInverseFollower::resetConfigToDefaults();
    server().send(200, "application/json", configJson());
  }

  void handleFollowerStatus() {
    server().send(200, "application/json", followerStatusJson());
  }

  void handleStartLine() {
    applyPenInverseConfigFromRequest();

    float distance = server().hasArg("d") ? server().arg("d").toFloat() : 20.0f;

    Odometry::reset();
    PenInverseFollower::startLine(distance);

    Logger::log("Test ligne stylo lance : d=" + String(distance, 1));
    server().send(200, "text/plain", "S2_ESCALIER_LINE_START");
  }

  void handleStartAngle() {
    applyPenInverseConfigFromRequest();

    float d1 = server().hasArg("d1") ? server().arg("d1").toFloat() : 20.0f;
    float angle = server().hasArg("a") ? server().arg("a").toFloat() : 90.0f;
    float d2 = server().hasArg("d2") ? server().arg("d2").toFloat() : 10.0f;

    Odometry::reset();
    PenInverseFollower::startOneAngle(d1, angle, d2);

    Logger::log("Test un angle stylo lance : d1=" + String(d1, 1) +
                " angle=" + String(angle, 1) +
                " d2=" + String(d2, 1));

    server().send(200, "text/plain", "S2_ESCALIER_ONE_ANGLE_START");
  }

  void handleStartStair() {
    applyPenInverseConfigFromRequest();

    float d1 = server().hasArg("d1") ? server().arg("d1").toFloat() : 20.0f;
    float aL = server().hasArg("aL") ? server().arg("aL").toFloat() : 90.0f;
    float d2 = server().hasArg("d2") ? server().arg("d2").toFloat() : 10.0f;
    float aR = server().hasArg("aR") ? server().arg("aR").toFloat() : 90.0f;
    float d3 = server().hasArg("d3") ? server().arg("d3").toFloat() : 40.0f;

    Odometry::reset();
    PenInverseFollower::startStair(d1, aL, d2, aR, d3);

    Logger::log("Escalier inverse stylo lance depuis API : d1=" + String(d1, 1) +
                " aL=" + String(aL, 1) +
                " d2=" + String(d2, 1) +
                " aR=" + String(aR, 1) +
                " d3=" + String(d3, 1));

    server().send(200, "text/plain", "S2_ESCALIER_INVERSE_START");
  }

  void handleStop() {
    PenInverseFollower::stop();
    server().send(200, "text/plain", "S2_ESCALIER_INVERSE_STOP");
  }

  void handleEncoderCalibrationReset() {
    Odometry::reset();
    Logger::log("Calibration encodeurs : reset ticks / odometrie");
    server().send(200, "text/plain", "ENCODER_CALIBRATION_RESET");
  }

  String encoderCalibrationJson(float measuredDistanceCm, float wheelDiameterCm) {
    long leftTicks = getLeftEncoderTicks();
    long rightTicks = getRightEncoderTicks();

    float absLeftTicks = fabs((float)leftTicks);
    float absRightTicks = fabs((float)rightTicks);
    float avgTicks = (absLeftTicks + absRightTicks) * 0.5f;

    float ticksPerCm = 0.0f;
    float cmPerTickMeasured = 0.0f;
    float recommendedTicksPerTour = 0.0f;
    float measuredDistanceFromCurrentConstant = getAverageDistanceCm();

    if (measuredDistanceCm > 0.001f && avgTicks > 0.001f) {
      ticksPerCm = avgTicks / measuredDistanceCm;
      cmPerTickMeasured = measuredDistanceCm / avgTicks;
      recommendedTicksPerTour = ticksPerCm * PI * wheelDiameterCm;
    }

    String json = "{";
    json += "\"leftTicks\":" + String(leftTicks) + ",";
    json += "\"rightTicks\":" + String(rightTicks) + ",";
    json += "\"avgTicks\":" + String(avgTicks, 3) + ",";
    json += "\"measuredDistanceCm\":" + String(measuredDistanceCm, 3) + ",";
    json += "\"wheelDiameterCm\":" + String(wheelDiameterCm, 3) + ",";
    json += "\"ticksPerCm\":" + String(ticksPerCm, 5) + ",";
    json += "\"cmPerTickMeasured\":" + String(cmPerTickMeasured, 7) + ",";
    json += "\"currentTicksPerTour\":" + String(RobotParams::TICKS_PAR_TOUR, 3) + ",";
    json += "\"recommendedTicksPerTour\":" + String(recommendedTicksPerTour, 3) + ",";
    json += "\"currentCmPerTick\":" + String(RobotParams::CM_PAR_TICK, 7) + ",";
    json += "\"currentOdoDistanceCm\":" + String(measuredDistanceFromCurrentConstant, 3);
    json += "}";

    return json;
  }

  void handleEncoderCalibrationStatus() {
    float defaultDistance = 20.0f;
    float wheelDiameter = server().hasArg("wheelDiameter") ? server().arg("wheelDiameter").toFloat() : RobotParams::ROUE_DIAMETER_CM;
    server().send(200, "application/json", encoderCalibrationJson(defaultDistance, wheelDiameter));
  }

  void handleEncoderCalibrationCompute() {
    float measuredDistance = server().hasArg("distance") ? server().arg("distance").toFloat() : 20.0f;
    float wheelDiameter = server().hasArg("wheelDiameter") ? server().arg("wheelDiameter").toFloat() : RobotParams::ROUE_DIAMETER_CM;

    if (measuredDistance <= 0.001f || wheelDiameter <= 0.001f) {
      server().send(400, "text/plain", "distance and wheelDiameter must be positive");
      return;
    }

    Logger::log("Calibration encodeurs : distance=" + String(measuredDistance, 2) +
                " cm, diametre roue=" + String(wheelDiameter, 2) + " cm");

    server().send(200, "application/json", encoderCalibrationJson(measuredDistance, wheelDiameter));
  }
}

namespace WebApiS2Escalier {
  void registerRoutes(WebServer& server) {
    serverPtr = &server;

    server.on("/api/s2/escalier/config", handleConfigGet);
    server.on("/api/s2/escalier/config/set", handleConfigSet);
    server.on("/api/s2/escalier/config/save", handleConfigSave);
    server.on("/api/s2/escalier/config/load", handleConfigLoad);
    server.on("/api/s2/escalier/config/defaults", handleConfigDefaults);
    server.on("/api/s2/escalier/follower/status", handleFollowerStatus);

    server.on("/api/s2/escalier/start-line", handleStartLine);
    server.on("/api/s2/escalier/start-angle", handleStartAngle);
    server.on("/api/s2/escalier/start", handleStartStair);
    server.on("/api/s2/escalier/stop", handleStop);

    // Outils de calibration encodeurs.
    server.on("/api/s2/calibration/encoders/reset", handleEncoderCalibrationReset);
    server.on("/api/s2/calibration/encoders/status", handleEncoderCalibrationStatus);
    server.on("/api/s2/calibration/encoders/compute", handleEncoderCalibrationCompute);
  }
}

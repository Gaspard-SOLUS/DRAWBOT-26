#include <Arduino.h>
#include <math.h>
#include <Preferences.h>

#include "pen_inverse_follower.h"
#include "app_state.h"
#include "odometry.h"
#include "moteurs.h"
#include "logger.h"

namespace {
  struct PID {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float integral = 0.0f;
    float previousError = 0.0f;
    bool first = true;
  };

  static Preferences prefs;

  PenInverseFollower::Config cfg;
  PenInverseFollower::Status status;

  struct PwmPair {
    int left;
    int right;
  };

  static const int MAX_SEGMENTS_LOCAL = TrajectoryGenerator::MAX_SEGMENTS;
  PenInverseFollower::Segment segments[MAX_SEGMENTS_LOCAL];

  int segmentCount = 0;
  int segmentIndex = 0;

  bool running = false;
  bool finished = false;

  PID pidLine;

  int lastPwmLeft = 0;
  int lastPwmRight = 0;

  float ditherAccumulatorLeft = 0.0f;
  float ditherAccumulatorRight = 0.0f;
  float lastNormalCorrection = 0.0f;

  float clampFloat(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
  }

  float degToRad(float deg) {
    return deg * PI / 180.0f;
  }

  void resetPid(PID& pid) {
    pid.integral = 0.0f;
    pid.previousError = 0.0f;
    pid.first = true;
  }

  float updatePid(PID& pid, float error, float dt, float integralLimit) {
    pid.integral += error * dt;
    pid.integral = clampFloat(pid.integral, -integralLimit, integralLimit);

    float derivative = 0.0f;

    if (!pid.first && dt > 0.0f) {
      derivative = (error - pid.previousError) / dt;
    }

    pid.previousError = error;
    pid.first = false;

    return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
  }

  void applyPidConfig() {
    pidLine.kp = cfg.kp;
    pidLine.ki = cfg.ki;
    pidLine.kd = cfg.kd;

    resetPid(pidLine);
  }

  void computePenPosition(float& penX, float& penY) {
    penX = odometryState.xCm + cfg.penOffsetCm * cos(odometryState.thetaRad);
    penY = odometryState.yCm + cfg.penOffsetCm * sin(odometryState.thetaRad);
  }

  int applyMinPwm(int pwm) {
    if (pwm == 0) {
      return 0;
    }

    int sign = (pwm > 0) ? 1 : -1;
    int absPwm = abs(pwm);

    if (absPwm < cfg.minPwm) {
      absPwm = cfg.minPwm;
    }

    if (absPwm > 255) {
      absPwm = 255;
    }

    return sign * absPwm;
  }

  int enforcePwmFloor(int pwm) {
    return applyMinPwm(constrain(pwm, -255, 255));
  }

  int applySlew(int previous, int target) {
    int step = cfg.pwmSlewStep;
    if (step < 1) step = 1;

    if (target > previous + step) return previous + step;
    if (target < previous - step) return previous - step;
    return target;
  }

  int applyContinuousSlew(int previous, int target) {
    target = applyMinPwm(target);

    if (target == 0) {
      return 0;
    }

    if (previous == 0) {
      return target;
    }

    if ((previous > 0 && target < 0) || (previous < 0 && target > 0)) {
      return target;
    }

    return applyMinPwm(applySlew(previous, target));
  }

  int leftSpeedToRawPwm(float speedCms) {
    if (fabs(cfg.coefLeftCmsPerPwm) < 0.001f) return 0;
    return (int)round(speedCms / cfg.coefLeftCmsPerPwm);
  }

  int rightSpeedToRawPwm(float speedCms) {
    if (fabs(cfg.coefRightCmsPerPwm) < 0.001f) return 0;
    return (int)round(speedCms / cfg.coefRightCmsPerPwm);
  }

  int ditherPwmCommand(int rawPwm, float& accumulator) {
    if (rawPwm == 0) {
      accumulator = 0.0f;
      return 0;
    }

    int sign = (rawPwm > 0) ? 1 : -1;
    int absRaw = abs(rawPwm);

    if (!cfg.pwmDither) {
      return applyMinPwm(rawPwm);
    }

    if (absRaw >= cfg.minPwm) {
      accumulator = 0.0f;
      return constrain(rawPwm, -255, 255);
    }

    if (cfg.minPwm <= 0) {
      return constrain(rawPwm, -255, 255);
    }

    // Pulse-density modulation:
    // exemple raw=45 et minPwm=180 => duty=0.25.
    // On envoie une impulsion à 180 environ un cycle sur quatre.
    float duty = ((float)absRaw) / ((float)cfg.minPwm);
    duty = clampFloat(duty, 0.0f, 1.0f);

    accumulator += duty;

    if (accumulator >= 1.0f) {
      accumulator -= 1.0f;
      return sign * cfg.minPwm;
    }

    return 0;
  }

  PwmPair floorSameDirectionPair(int rawLeft, int rawRight, int sign) {
    int absLeft = abs(rawLeft);
    int absRight = abs(rawRight);

    if (absLeft == 0 && absRight == 0) {
      return {0, 0};
    }

    if (cfg.minPwm <= 0) {
      return {
        constrain(sign * absLeft, -255, 255),
        constrain(sign * absRight, -255, 255)
      };
    }

    int maxAbs = max(absLeft, absRight);
    int minAbs = min(absLeft, absRight);

    int outMax = 0;
    int outMin = 0;

    if (minAbs == 0) {
      outMax = 255;
      outMin = cfg.minPwm;
    } else {
      float scale = ((float)cfg.minPwm) / ((float)minAbs);
      outMin = cfg.minPwm;
      outMax = (int)round(maxAbs * scale);

      if (outMax > 255) {
        outMax = 255;
        outMin = max(cfg.minPwm, (int)round(255.0f * ((float)minAbs / (float)maxAbs)));
      }
    }

    outMax = constrain(outMax, cfg.minPwm, 255);
    outMin = constrain(outMin, cfg.minPwm, 255);

    int left = (absLeft >= absRight) ? outMax : outMin;
    int right = (absRight > absLeft) ? outMax : outMin;

    return {sign * left, sign * right};
  }

  PwmPair forwardOnlyPairForStair(int rawLeft, int rawRight) {
    if (rawLeft == 0 && rawRight == 0) {
      return {0, 0};
    }

    if (rawLeft < 0 && rawRight < 0) {
      return {0, 0};
    }

    if (rawLeft < 0 || rawRight < 0) {
      // En escalier, on evite la marche arriere : le virage le plus serre
      // disponible devient 180/255 au lieu de -180/180.
      if (rawLeft > rawRight) {
        return {255, cfg.minPwm};
      }
      return {cfg.minPwm, 255};
    }

    return floorSameDirectionPair(rawLeft, rawRight, 1);
  }

  bool isSmallDitherCommand(int rawPwm) {
    return cfg.pwmDither && cfg.minPwm > 0 && abs(rawPwm) > 0 && abs(rawPwm) < cfg.minPwm;
  }

  float pwmToLeftSpeed(int pwm) {
    return ((float)pwm) * cfg.coefLeftCmsPerPwm;
  }

  float pwmToRightSpeed(int pwm) {
    return ((float)pwm) * cfg.coefRightCmsPerPwm;
  }

  void setMotorPwmTracked(int pwmLeft, int pwmRight) {
    pwmLeft = enforcePwmFloor(pwmLeft);
    pwmRight = enforcePwmFloor(pwmRight);

    motorState.pwmLeft = pwmLeft;
    motorState.pwmRight = pwmRight;
    motorState.mode = "PEN_INVERSE";

    setMotors(pwmLeft, pwmRight);
  }

  void hardStopMotorsTracked() {
    lastPwmLeft = 0;
    lastPwmRight = 0;
    ditherAccumulatorLeft = 0.0f;
    ditherAccumulatorRight = 0.0f;
    lastNormalCorrection = 0.0f;
    status.pwmLeft = 0;
    status.pwmRight = 0;
    status.targetPwmLeft = 0;
    status.targetPwmRight = 0;
    status.phaseName = "IDLE";
    status.cornerActive = false;
    status.reverseLimited = false;
    status.omegaLimited = false;
    status.normalCorrectionLimited = false;
    status.vLeftCms = 0.0f;
    status.vRightCms = 0.0f;
    status.vCenterCms = 0.0f;
    status.omegaRadS = 0.0f;

    setMotorPwmTracked(0, 0);
    stopMotors();

    motorState.mode = "IDLE";
  }

  void resetStatus() {
    status = PenInverseFollower::Status();

    status.running = running;
    status.finished = finished;
    status.currentSegment = segmentIndex;
    status.segmentCount = segmentCount;
    status.phaseName = running ? "FOLLOW_SEGMENT" : (finished ? "FINISHED" : "IDLE");
  }

  void startSegments(int count) {
    segmentCount = constrain(count, 0, MAX_SEGMENTS_LOCAL);
    segmentIndex = 0;

    running = (segmentCount > 0);
    finished = (segmentCount <= 0);

    lastPwmLeft = 0;
    lastPwmRight = 0;
    ditherAccumulatorLeft = 0.0f;
    ditherAccumulatorRight = 0.0f;
    lastNormalCorrection = 0.0f;

    applyPidConfig();
    resetStatus();

    Logger::log("PenInverseFollower demarre avec " + String(segmentCount) + " segment(s)");
  }

  void copySegment(int index, float ax, float ay, float bx, float by) {
    segments[index].a.x = ax;
    segments[index].a.y = ay;
    segments[index].b.x = bx;
    segments[index].b.y = by;
  }
}

namespace PenInverseFollower {
  void begin() {
    running = false;
    finished = false;

    segmentCount = 0;
    segmentIndex = 0;

    loadConfig();

    applyPidConfig();
    resetStatus();

    Logger::log("PenInverseFollower pret");
  }

  void setConfig(const Config& newCfg) {
    cfg = newCfg;

    if (cfg.wheelBaseCm < 1.0f) cfg.wheelBaseCm = 8.3f;
    if (cfg.penOffsetCm < 0.1f) cfg.penOffsetCm = 13.0f;
    if (cfg.distanceScale <= 0.0f) cfg.distanceScale = 1.0f;
    if (cfg.penSpeedCms < 0.0f) cfg.penSpeedCms = 0.0f;
    if (cfg.penSpeedMaxCms < 0.1f) cfg.penSpeedMaxCms = 0.1f;
    if (cfg.wheelSpeedMaxCms < 0.1f) cfg.wheelSpeedMaxCms = 0.1f;
    if (cfg.maxOmegaRadS < 0.1f) cfg.maxOmegaRadS = 0.1f;
    if (cfg.maxNormalCorrectionCms < 0.0f) cfg.maxNormalCorrectionCms = 0.0f;
    if (cfg.minCommandSpeedCms < 0.0f) cfg.minCommandSpeedCms = 0.0f;
    if (cfg.cornerMaxDurationS < 0.1f) cfg.cornerMaxDurationS = 0.1f;
    if (cfg.minPwm < 0) cfg.minPwm = 0;
    if (cfg.minPwm > 255) cfg.minPwm = 255;
    if (cfg.pwmSlewStep < 1) cfg.pwmSlewStep = 1;
    if (cfg.pwmSlewStep > 255) cfg.pwmSlewStep = 255;
    if (cfg.segmentToleranceCm < 0.01f) cfg.segmentToleranceCm = 0.01f;
    cfg.stairMiddleExtraCm = clampFloat(cfg.stairMiddleExtraCm, 0.0f, 12.0f);
    cfg.stairSecondAngleTrimDeg = clampFloat(cfg.stairSecondAngleTrimDeg, 0.0f, 60.0f);
    cfg.stairLineDeadbandCm = clampFloat(cfg.stairLineDeadbandCm, 0.0f, 2.0f);
    cfg.stairNormalSlewCms = clampFloat(cfg.stairNormalSlewCms, 0.0f, 5.0f);

    applyPidConfig();

    Logger::log("Config PenInverseFollower mise a jour");
  }

  Config getConfig() {
    return cfg;
  }

  Status getStatus() {
    return status;
  }

  bool loadConfig() {
    prefs.begin("penFollower", true);

    bool hasConfig = prefs.getBool("hasConfig", false);

    if (hasConfig) {
      cfg.wheelBaseCm = prefs.getFloat("wheelBase", cfg.wheelBaseCm);
      cfg.penOffsetCm = prefs.getFloat("penOffset", cfg.penOffsetCm);
      cfg.distanceScale = prefs.getFloat("distScale", cfg.distanceScale);

      cfg.penSpeedCms = prefs.getFloat("penSpeed", cfg.penSpeedCms);
      cfg.lineGain = prefs.getFloat("lineGain", cfg.lineGain);
      cfg.targetGain = prefs.getFloat("targetGain", cfg.targetGain);
      cfg.lookaheadCm = prefs.getFloat("lookahead", cfg.lookaheadCm);

      cfg.penSpeedMaxCms = prefs.getFloat("penSpMax", cfg.penSpeedMaxCms);
      cfg.wheelSpeedMaxCms = prefs.getFloat("wheelSpMax", cfg.wheelSpeedMaxCms);
      cfg.maxOmegaRadS = prefs.getFloat("maxOmega", cfg.maxOmegaRadS);
      cfg.maxNormalCorrectionCms = prefs.getFloat("maxNorm", cfg.maxNormalCorrectionCms);

      cfg.allowReverse = prefs.getBool("allowRev", cfg.allowReverse);
      cfg.minForwardSpeedCms = prefs.getFloat("minFwd", cfg.minForwardSpeedCms);
      cfg.minCommandSpeedCms = prefs.getFloat("minCmd", cfg.minCommandSpeedCms);

      cfg.cornerMode = prefs.getBool("cornerMode", cfg.cornerMode);
      cfg.cornerApproachCm = prefs.getFloat("cornerAp", cfg.cornerApproachCm);
      cfg.cornerSpeedCms = prefs.getFloat("cornerSp", cfg.cornerSpeedCms);
      cfg.cornerOmegaRadS = prefs.getFloat("cornerOm", cfg.cornerOmegaRadS);
      cfg.cornerExitAngleDeg = prefs.getFloat("cornerEx", cfg.cornerExitAngleDeg);
      cfg.cornerMaxDurationS = prefs.getFloat("cornerDur", cfg.cornerMaxDurationS);
      cfg.stairMiddleExtraCm = prefs.getFloat("stairMidEx", cfg.stairMiddleExtraCm);
      cfg.stairSecondAngleTrimDeg = prefs.getFloat("stairAngTr", cfg.stairSecondAngleTrimDeg);
      cfg.stairLineDeadbandCm = prefs.getFloat("stairDead", cfg.stairLineDeadbandCm);
      cfg.stairNormalSlewCms = prefs.getFloat("stairNSlew", cfg.stairNormalSlewCms);

      cfg.kp = prefs.getFloat("kp", cfg.kp);
      cfg.ki = prefs.getFloat("ki", cfg.ki);
      cfg.kd = prefs.getFloat("kd", cfg.kd);
      cfg.integralLimit = prefs.getFloat("iLimit", cfg.integralLimit);

      cfg.coefLeftCmsPerPwm = prefs.getFloat("coefL", cfg.coefLeftCmsPerPwm);
      cfg.coefRightCmsPerPwm = prefs.getFloat("coefR", cfg.coefRightCmsPerPwm);

      cfg.minPwm = prefs.getInt("minPwm", cfg.minPwm);
      cfg.pwmSlewStep = prefs.getInt("slew", cfg.pwmSlewStep);
      cfg.pwmDither = prefs.getBool("dither", cfg.pwmDither);

      cfg.allowReverse = prefs.getBool("reverse", cfg.allowReverse);
      cfg.minForwardSpeedCms = prefs.getFloat("minFwd", cfg.minForwardSpeedCms);

      cfg.segmentToleranceCm = prefs.getFloat("segTol", cfg.segmentToleranceCm);
    }

    prefs.end();

    applyPidConfig();

    Logger::log(hasConfig ? "Config PenInverseFollower chargee depuis flash"
                          : "Config PenInverseFollower par defaut");

    return hasConfig;
  }

  bool saveConfig() {
    prefs.begin("penFollower", false);

    prefs.putBool("hasConfig", true);

    prefs.putFloat("wheelBase", cfg.wheelBaseCm);
    prefs.putFloat("penOffset", cfg.penOffsetCm);
    prefs.putFloat("distScale", cfg.distanceScale);

    prefs.putFloat("penSpeed", cfg.penSpeedCms);
    prefs.putFloat("lineGain", cfg.lineGain);
    prefs.putFloat("targetGain", cfg.targetGain);
    prefs.putFloat("lookahead", cfg.lookaheadCm);

    prefs.putFloat("penSpMax", cfg.penSpeedMaxCms);
    prefs.putFloat("wheelSpMax", cfg.wheelSpeedMaxCms);
    prefs.putFloat("maxOmega", cfg.maxOmegaRadS);
    prefs.putFloat("maxNorm", cfg.maxNormalCorrectionCms);

    prefs.putBool("allowRev", cfg.allowReverse);
    prefs.putFloat("minFwd", cfg.minForwardSpeedCms);
    prefs.putFloat("minCmd", cfg.minCommandSpeedCms);

    prefs.putBool("cornerMode", cfg.cornerMode);
    prefs.putFloat("cornerAp", cfg.cornerApproachCm);
    prefs.putFloat("cornerSp", cfg.cornerSpeedCms);
    prefs.putFloat("cornerOm", cfg.cornerOmegaRadS);
    prefs.putFloat("cornerEx", cfg.cornerExitAngleDeg);
    prefs.putFloat("cornerDur", cfg.cornerMaxDurationS);
    prefs.putFloat("stairMidEx", cfg.stairMiddleExtraCm);
    prefs.putFloat("stairAngTr", cfg.stairSecondAngleTrimDeg);
    prefs.putFloat("stairDead", cfg.stairLineDeadbandCm);
    prefs.putFloat("stairNSlew", cfg.stairNormalSlewCms);

    prefs.putFloat("kp", cfg.kp);
    prefs.putFloat("ki", cfg.ki);
    prefs.putFloat("kd", cfg.kd);
    prefs.putFloat("iLimit", cfg.integralLimit);

    prefs.putFloat("coefL", cfg.coefLeftCmsPerPwm);
    prefs.putFloat("coefR", cfg.coefRightCmsPerPwm);

    prefs.putInt("minPwm", cfg.minPwm);
    prefs.putInt("slew", cfg.pwmSlewStep);
    prefs.putBool("dither", cfg.pwmDither);

    prefs.putBool("reverse", cfg.allowReverse);
    prefs.putFloat("minFwd", cfg.minForwardSpeedCms);

    prefs.putFloat("segTol", cfg.segmentToleranceCm);

    prefs.end();

    Logger::log("Config PenInverseFollower enregistree en flash");
    return true;
  }

  void resetConfigToDefaults() {
    cfg = Config();
    applyPidConfig();
    Logger::log("Config PenInverseFollower remise aux valeurs par defaut");
  }

  void startLine(float distanceCm) {
    float d = distanceCm * cfg.distanceScale;

    copySegment(0, 0.0f, 0.0f, d, 0.0f);

    Odometry::resetPose(-cfg.penOffsetCm, 0.0f, 0.0f);
    startSegments(1);

    Logger::log("Test ligne stylo : distance=" + String(distanceCm, 1) +
                " cm scale=" + String(cfg.distanceScale, 3));
  }

  void startOneAngle(float d1Cm, float angleDeg, float d2Cm) {
    float d1 = d1Cm * cfg.distanceScale;
    float d2 = d2Cm * cfg.distanceScale;

    Point p0 {0.0f, 0.0f};

    float theta = 0.0f;

    Point p1 {
      p0.x + d1 * cos(theta),
      p0.y + d1 * sin(theta)
    };

    theta += degToRad(angleDeg);

    Point p2 {
      p1.x + d2 * cos(theta),
      p1.y + d2 * sin(theta)
    };

    segments[0] = {p0, p1};
    segments[1] = {p1, p2};

    Odometry::resetPose(-cfg.penOffsetCm, 0.0f, 0.0f);
    startSegments(2);

    Logger::log("Angle stylo lance : d1=" + String(d1Cm, 1) +
                " angle=" + String(angleDeg, 1) +
                " d2=" + String(d2Cm, 1));
  }

  void startStair(float d1Cm, float angleLeftDeg, float d2Cm, float angleRightDeg, float d3Cm) {
    float d1 = d1Cm * cfg.distanceScale;
    float d2 = (d2Cm + cfg.stairMiddleExtraCm) * cfg.distanceScale;
    float d3 = d3Cm * cfg.distanceScale;
    float effectiveAngleRightDeg = angleRightDeg - cfg.stairSecondAngleTrimDeg;
    effectiveAngleRightDeg = clampFloat(effectiveAngleRightDeg, 10.0f, angleRightDeg);

    Point p0 {0.0f, 0.0f};

    float theta = 0.0f;

    Point p1 {
      p0.x + d1 * cos(theta),
      p0.y + d1 * sin(theta)
    };

    theta += degToRad(angleLeftDeg);

    Point p2 {
      p1.x + d2 * cos(theta),
      p1.y + d2 * sin(theta)
    };

    theta -= degToRad(effectiveAngleRightDeg);

    Point p3 {
      p2.x + d3 * cos(theta),
      p2.y + d3 * sin(theta)
    };

    segments[0] = {p0, p1};
    segments[1] = {p1, p2};
    segments[2] = {p2, p3};

    Odometry::resetPose(-cfg.penOffsetCm, 0.0f, 0.0f);
    startSegments(3);

    Logger::log("Escalier stylo lance : compensation trait2=" +
                String(cfg.stairMiddleExtraCm, 2) +
                " cm correction angle2=" +
                String(cfg.stairSecondAngleTrimDeg, 1) +
                " deg angle2 effectif=" +
                String(effectiveAngleRightDeg, 1) + " deg");
  }

  bool startTrajectory(const TrajectoryGenerator::Trajectory& trajectory) {
    if (trajectory.count <= 0) {
      Logger::log("Erreur : trajectoire vide");
      return false;
    }

    int count = trajectory.count;
    if (count > MAX_SEGMENTS_LOCAL) {
      count = MAX_SEGMENTS_LOCAL;
    }

    for (int i = 0; i < count; i++) {
      segments[i].a.x = trajectory.segments[i].a.x;
      segments[i].a.y = trajectory.segments[i].a.y;
      segments[i].b.x = trajectory.segments[i].b.x;
      segments[i].b.y = trajectory.segments[i].b.y;
    }

    startSegments(count);

    Logger::log("Trajectoire stylo lancee : " + String(count) + " segment(s)");
    return true;
  }

  void stop() {
    running = false;
    finished = false;

    hardStopMotorsTracked();

    resetStatus();

    Logger::log("PenInverseFollower stop");
  }

  void update(unsigned long now, float dt) {
    (void)now;

    if (!running || finished) {
      return;
    }

    if (dt <= 0.0f) {
      dt = 0.02f;
    }

    if (segmentIndex >= segmentCount) {
      running = false;
      finished = true;

      hardStopMotorsTracked();

      status.running = running;
      status.finished = finished;
      status.currentSegment = segmentIndex;
      status.segmentCount = segmentCount;

      Logger::log("PenInverseFollower termine. Erreur max stylo=" +
                  String(status.maxLateralErrorCm, 2) + " cm");

      return;
    }

    Segment s = segments[segmentIndex];

    float dx = s.b.x - s.a.x;
    float dy = s.b.y - s.a.y;
    float length = sqrt(dx * dx + dy * dy);

    if (length < 0.001f) {
      segmentIndex++;
      resetPid(pidLine);
      lastNormalCorrection = 0.0f;
      return;
    }

    float ux = dx / length;
    float uy = dy / length;

    float nx = -uy;
    float ny = ux;

    float penX = 0.0f;
    float penY = 0.0f;
    computePenPosition(penX, penY);

    float rx = penX - s.a.x;
    float ry = penY - s.a.y;

    float progress = rx * ux + ry * uy;
    float lateralError = rx * nx + ry * ny;

    if (fabs(lateralError) > status.maxLateralErrorCm) {
      status.maxLateralErrorCm = fabs(lateralError);
    }

    float endTolerance = cfg.segmentToleranceCm;

    if (segmentIndex + 1 < segmentCount) {
      endTolerance = fmin(endTolerance, 0.02f);
    }

    if (progress >= length - endTolerance) {
      segmentIndex++;
      resetPid(pidLine);
      lastNormalCorrection = 0.0f;

      if (segmentIndex >= segmentCount) {
        running = false;
        finished = true;
        hardStopMotorsTracked();

        Logger::log("PenInverseFollower termine. Erreur max stylo=" +
                    String(status.maxLateralErrorCm, 2) + " cm");
      } else {
        Logger::log("Passage segment stylo " + String(segmentIndex + 1));
      }

      status.running = running;
      status.finished = finished;
      status.currentSegment = segmentIndex;
      status.segmentCount = segmentCount;
      status.penX = penX;
      status.penY = penY;
      status.lateralErrorCm = lateralError;
      status.progressCm = progress;
      status.segmentLengthCm = length;

      return;
    }

    float lookProgress = clampFloat(progress + cfg.lookaheadCm, 0.0f, length);

    float targetX = s.a.x + lookProgress * ux;
    float targetY = s.a.y + lookProgress * uy;

    float errorX = targetX - penX;
    float errorY = targetY - penY;

    bool stairLike = segmentCount <= 3 && !cfg.allowReverse;
    float correctionError = lateralError;

    if (stairLike && fabs(correctionError) < cfg.stairLineDeadbandCm) {
      correctionError = 0.0f;
      resetPid(pidLine);
    }

    float pidNormal = updatePid(pidLine, correctionError, dt, cfg.integralLimit);
    float normalCorrection = cfg.lineGain * correctionError + pidNormal;

    status.normalCorrectionLimited = false;
    if (cfg.maxNormalCorrectionCms > 0.0f && fabs(normalCorrection) > cfg.maxNormalCorrectionCms) {
      normalCorrection = (normalCorrection > 0.0f) ? cfg.maxNormalCorrectionCms : -cfg.maxNormalCorrectionCms;
      status.normalCorrectionLimited = true;
    }

    if (stairLike && cfg.stairNormalSlewCms > 0.0f) {
      normalCorrection = clampFloat(
        normalCorrection,
        lastNormalCorrection - cfg.stairNormalSlewCms,
        lastNormalCorrection + cfg.stairNormalSlewCms
      );
    }

    lastNormalCorrection = normalCorrection;

    float vPenX = cfg.penSpeedCms * ux
                + cfg.targetGain * errorX
                - normalCorrection * nx;

    float vPenY = cfg.penSpeedCms * uy
                + cfg.targetGain * errorY
                - normalCorrection * ny;

    float vPenNorm = sqrt(vPenX * vPenX + vPenY * vPenY);

    if (vPenNorm > cfg.penSpeedMaxCms && vPenNorm > 0.001f) {
      vPenX *= cfg.penSpeedMaxCms / vPenNorm;
      vPenY *= cfg.penSpeedMaxCms / vPenNorm;
    }

    float ex = cos(odometryState.thetaRad);
    float ey = sin(odometryState.thetaRad);

    float normalRobotX = -sin(odometryState.thetaRad);
    float normalRobotY = cos(odometryState.thetaRad);

    float vCenter = vPenX * ex + vPenY * ey;
    float omega = (vPenX * normalRobotX + vPenY * normalRobotY) / cfg.penOffsetCm;

    status.reverseLimited = false;
    status.omegaLimited = false;

    if (!cfg.allowReverse && vCenter < cfg.minForwardSpeedCms) {
      vCenter = cfg.minForwardSpeedCms;
      status.reverseLimited = true;
    }

    if (cfg.maxOmegaRadS > 0.0f && fabs(omega) > cfg.maxOmegaRadS) {
      omega = (omega > 0.0f) ? cfg.maxOmegaRadS : -cfg.maxOmegaRadS;
      status.omegaLimited = true;
    }

    float vLeftDesired = vCenter - omega * cfg.wheelBaseCm * 0.5f;
    float vRightDesired = vCenter + omega * cfg.wheelBaseCm * 0.5f;

    vLeftDesired = clampFloat(vLeftDesired, -cfg.wheelSpeedMaxCms, cfg.wheelSpeedMaxCms);
    vRightDesired = clampFloat(vRightDesired, -cfg.wheelSpeedMaxCms, cfg.wheelSpeedMaxCms);

    if (fabs(vLeftDesired) < cfg.minCommandSpeedCms) vLeftDesired = 0.0f;
    if (fabs(vRightDesired) < cfg.minCommandSpeedCms) vRightDesired = 0.0f;

    int pwmLeftRaw = leftSpeedToRawPwm(vLeftDesired);
    int pwmRightRaw = rightSpeedToRawPwm(vRightDesired);

    bool useDither = cfg.pwmDither && segmentCount > 3;

    int pwmLeftTarget = 0;
    int pwmRightTarget = 0;

    if (!useDither && !cfg.allowReverse) {
      PwmPair pair = forwardOnlyPairForStair(pwmLeftRaw, pwmRightRaw);
      pwmLeftTarget = pair.left;
      pwmRightTarget = pair.right;
    } else {
      pwmLeftTarget = useDither ? ditherPwmCommand(pwmLeftRaw, ditherAccumulatorLeft)
                                : applyMinPwm(pwmLeftRaw);
      pwmRightTarget = useDither ? ditherPwmCommand(pwmRightRaw, ditherAccumulatorRight)
                                 : applyMinPwm(pwmRightRaw);
    }

    if (!useDither) {
      ditherAccumulatorLeft = 0.0f;
      ditherAccumulatorRight = 0.0f;
    }

    int pwmLeft = pwmLeftTarget;
    int pwmRight = pwmRightTarget;

    // Si on est en micro-impulsions, on ne rampe pas vers minPwm :
    // il faut vraiment envoyer une impulsion suffisante pour vaincre les frottements.
    // Pour les commandes au-dessus de minPwm, on garde la rampe classique.
    if (!useDither || !isSmallDitherCommand(pwmLeftRaw)) {
      pwmLeft = applyContinuousSlew(lastPwmLeft, pwmLeftTarget);
    }

    if (!useDither || !isSmallDitherCommand(pwmRightRaw)) {
      pwmRight = applyContinuousSlew(lastPwmRight, pwmRightTarget);
    }

    pwmLeft = enforcePwmFloor(pwmLeft);
    pwmRight = enforcePwmFloor(pwmRight);

    lastPwmLeft = pwmLeft;
    lastPwmRight = pwmRight;

    setMotorPwmTracked(pwmLeft, pwmRight);

    float vLeftActual = pwmToLeftSpeed(pwmLeft);
    float vRightActual = pwmToRightSpeed(pwmRight);

    status.running = running;
    status.finished = finished;
    status.currentSegment = segmentIndex;
    status.segmentCount = segmentCount;
    status.phaseName = "FOLLOW_SEGMENT";
    status.cornerActive = false;

    status.penX = penX;
    status.penY = penY;

    status.targetX = targetX;
    status.targetY = targetY;

    status.lateralErrorCm = lateralError;
    status.progressCm = progress;
    status.segmentLengthCm = length;

    status.vPenX = vPenX;
    status.vPenY = vPenY;

    status.vCenterCms = (vLeftActual + vRightActual) * 0.5f;
    status.omegaRadS = (vRightActual - vLeftActual) / cfg.wheelBaseCm;

    status.vLeftCms = vLeftActual;
    status.vRightCms = vRightActual;

    status.pwmLeft = pwmLeft;
    status.pwmRight = pwmRight;
    status.targetPwmLeft = pwmLeftTarget;
    status.targetPwmRight = pwmRightTarget;
  }

  bool isRunning() {
    return running;
  }

  bool isFinished() {
    return finished;
  }
}

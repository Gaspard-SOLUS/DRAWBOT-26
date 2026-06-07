#include <Arduino.h>
#include <math.h>
#include <Preferences.h>

#include "pen_inverse_follower.h"
#include "app_state.h"
#include "odometry.h"
#include "encodeurs.h"
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

  static const int MAX_SEGMENTS_LOCAL = TrajectoryGenerator::MAX_SEGMENTS;
  PenInverseFollower::Segment segments[MAX_SEGMENTS_LOCAL];

  int segmentCount = 0;
  int segmentIndex = 0;

  bool running = false;
  bool finished = false;
  bool straightLineMode = false;
  bool straightBrakeActive = false;
  float straightTargetCm = 0.0f;
  float straightStartLeftCm = 0.0f;
  float straightStartRightCm = 0.0f;
  unsigned long straightBrakeEndMs = 0;
  int straightDirection = 1;

  PID pidLine;

  int lastPwmLeft = 0;
  int lastPwmRight = 0;
  unsigned long pwmTraceTick = 0;
  unsigned long lastPwmTraceMs = 0;
  float pulseAccumulatorLeft = 0.0f;
  float pulseAccumulatorRight = 0.0f;
  int straightLockSegmentIndex = -1;
  float straightLockStartLeftCm = 0.0f;
  float straightLockStartRightCm = 0.0f;

  void resetStraightLock();

  float clampFloat(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
  }

  float degToRad(float deg) {
    return deg * PI / 180.0f;
  }

  float radToDeg(float rad) {
    return rad * 180.0f / PI;
  }

  float normalizeRad(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
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

  int applySlew(int previous, int target) {
    int step = cfg.pwmSlewStep;
    if (step < 1) step = 1;

    if (target > previous + step) return previous + step;
    if (target < previous - step) return previous - step;
    return target;
  }

  int leftSpeedToRawPwm(float speedCms) {
    if (fabs(cfg.coefLeftCmsPerPwm) < 0.001f) return 0;
    return (int)round(speedCms / cfg.coefLeftCmsPerPwm);
  }

  int rightSpeedToRawPwm(float speedCms) {
    if (fabs(cfg.coefRightCmsPerPwm) < 0.001f) return 0;
    return (int)round(speedCms / cfg.coefRightCmsPerPwm);
  }

  int continuousPwmCommand(int rawPwm) {
    return applyMinPwm(rawPwm);
  }

  void scaledWheelPwmCommands(float vLeftDesired,
                              float vRightDesired,
                              int& pwmLeftTarget,
                              int& pwmRightTarget) {
    int rawLeft = leftSpeedToRawPwm(vLeftDesired);
    int rawRight = rightSpeedToRawPwm(vRightDesired);

    if (rawLeft == 0 && rawRight == 0) {
      pwmLeftTarget = 0;
      pwmRightTarget = 0;
      return;
    }

    float scale = 1.0f;

    if (rawLeft != 0 && abs(rawLeft) < cfg.minPwm) {
      scale = fmax(scale, ((float)cfg.minPwm) / ((float)abs(rawLeft)));
    }

    if (rawRight != 0 && abs(rawRight) < cfg.minPwm) {
      scale = fmax(scale, ((float)cfg.minPwm) / ((float)abs(rawRight)));
    }

    float scaledLeft = ((float)rawLeft) * scale;
    float scaledRight = ((float)rawRight) * scale;
    float maxAbs = fmax(fabs(scaledLeft), fabs(scaledRight));

    if (maxAbs > 255.0f) {
      float downScale = 255.0f / maxAbs;
      scaledLeft *= downScale;
      scaledRight *= downScale;
    }

    pwmLeftTarget = applyMinPwm((int)round(scaledLeft));
    pwmRightTarget = applyMinPwm((int)round(scaledRight));
  }

  int signFromPwmOrSpeed(int pwm, float speedCms, int fallbackSign) {
    if (pwm > 0) return 1;
    if (pwm < 0) return -1;
    if (speedCms > 0.01f) return 1;
    if (speedCms < -0.01f) return -1;
    return fallbackSign;
  }

  void boostInnerWheelPwmInCorner(float vLeftDesired,
                                  float vRightDesired,
                                  int& pwmLeftTarget,
                                  int& pwmRightTarget) {
    if (cfg.cornerInnerBoost <= 0.0f) {
      return;
    }

    int absLeft = abs(pwmLeftTarget);
    int absRight = abs(pwmRightTarget);

    if ((absLeft == 0 && absRight == 0) || absLeft == absRight) {
      return;
    }

    float boost = clampFloat(cfg.cornerInnerBoost, 0.0f, 1.0f);

    if (absLeft < absRight) {
      int boosted = absLeft + (int)round(((float)(absRight - absLeft)) * boost);
      if (boosted > 0) {
        boosted = constrain(boosted, cfg.minPwm, 255);
        int sign = signFromPwmOrSpeed(pwmLeftTarget, vLeftDesired, pwmRightTarget >= 0 ? 1 : -1);
        pwmLeftTarget = sign * boosted;
      }
    } else {
      int boosted = absRight + (int)round(((float)(absLeft - absRight)) * boost);
      if (boosted > 0) {
        boosted = constrain(boosted, cfg.minPwm, 255);
        int sign = signFromPwmOrSpeed(pwmRightTarget, vRightDesired, pwmLeftTarget >= 0 ? 1 : -1);
        pwmRightTarget = sign * boosted;
      }
    }
  }

  int applyMinimumAbsPwm(int pwm, int minAbsPwm) {
    if (pwm == 0) {
      return 0;
    }

    int sign = pwm > 0 ? 1 : -1;
    int absPwm = abs(pwm);

    if (absPwm < minAbsPwm) {
      absPwm = minAbsPwm;
    }

    if (absPwm > 255) {
      absPwm = 255;
    }

    return sign * absPwm;
  }

  void applyCornerTurnMinimum(int& pwmLeftTarget, int& pwmRightTarget) {
    int minTurnPwm = cfg.cornerTurnMinPwm;
    if (minTurnPwm < cfg.minPwm) {
      minTurnPwm = cfg.minPwm;
    }
    if (minTurnPwm > 255) {
      minTurnPwm = 255;
    }

    pwmLeftTarget = applyMinimumAbsPwm(pwmLeftTarget, minTurnPwm);
    pwmRightTarget = applyMinimumAbsPwm(pwmRightTarget, minTurnPwm);
  }

  void printPwmTrace(unsigned long now,
                     int pwmLeftTarget,
                     int pwmRightTarget,
                     int pwmLeft,
                     int pwmRight,
                     float vLeftDesired,
                     float vRightDesired,
                     float remainingToCorner) {
    if (segmentCount <= 1) {
      return;
    }

    if (lastPwmTraceMs != 0 && now - lastPwmTraceMs < 100) {
      return;
    }
    lastPwmTraceMs = now;

    String line = "PF tick=" + String(++pwmTraceTick) +
                  " t=" + String(now) +
                  " seg=" + String(segmentIndex + 1) + "/" + String(segmentCount) +
                  " ph=" + status.phaseName +
                  " tgt=" + String(pwmLeftTarget) + "/" + String(pwmRightTarget) +
                  " pwm=" + String(pwmLeft) + "/" + String(pwmRight) +
                  " v=" + String(vLeftDesired, 1) + "/" + String(vRightDesired, 1) +
                  " spd=" + String(odometryState.speedLeftCms, 1) + "/" + String(odometryState.speedRightCms, 1) +
                  " rem=" + String(remainingToCorner, 2);
    Logger::trace(line);
  }

  bool isPulseRange(int rawPwm) {
    return cfg.minPwm > 0 && rawPwm != 0 && abs(rawPwm) < cfg.minPwm;
  }

  int pulsePwmCommand(int rawPwm, float& accumulator) {
    if (rawPwm == 0) {
      accumulator = 0.0f;
      return 0;
    }

    int sign = (rawPwm > 0) ? 1 : -1;
    int absRaw = abs(rawPwm);

    if (!isPulseRange(rawPwm)) {
      accumulator = 0.0f;
      return applyMinPwm(rawPwm);
    }

    float duty = ((float)absRaw) / ((float)cfg.minPwm);
    duty = clampFloat(duty, 0.0f, 1.0f);
    accumulator += duty;

    if (accumulator >= 1.0f) {
      accumulator -= 1.0f;
      return sign * cfg.minPwm;
    }

    return 0;
  }

  int applyContinuousSlew(int previous, int target, bool directSignFlip = false) {
    target = applyMinPwm(target);

    if (target == 0) {
      return 0;
    }

    if (previous == 0) {
      return target;
    }

    if ((previous > 0 && target < 0) || (previous < 0 && target > 0)) {
      if (directSignFlip) {
        return target;
      }

      return 0;
    }

    return applyMinPwm(applySlew(previous, target));
  }

  float pwmToLeftSpeed(int pwm) {
    return ((float)pwm) * cfg.coefLeftCmsPerPwm;
  }

  float pwmToRightSpeed(int pwm) {
    return ((float)pwm) * cfg.coefRightCmsPerPwm;
  }

  void setMotorPwmTracked(int pwmLeft, int pwmRight) {
    pwmLeft = constrain(pwmLeft, -255, 255);
    pwmRight = constrain(pwmRight, -255, 255);

    motorState.pwmLeft = pwmLeft;
    motorState.pwmRight = pwmRight;
    motorState.mode = "PEN_INVERSE";

    setMotors(pwmLeft, pwmRight);
  }

  void hardStopMotorsTracked() {
    lastPwmLeft = 0;
    lastPwmRight = 0;
    pwmTraceTick = 0;
    lastPwmTraceMs = 0;
    pulseAccumulatorLeft = 0.0f;
    pulseAccumulatorRight = 0.0f;
    resetStraightLock();
    straightLineMode = false;
    straightBrakeActive = false;
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
    status.encoderBalanceErrorCm = 0.0f;
    status.straightStopDistanceCm = 0.0f;

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
    pwmTraceTick = 0;
    lastPwmTraceMs = 0;
    pulseAccumulatorLeft = 0.0f;
    pulseAccumulatorRight = 0.0f;
    resetStraightLock();
    straightLineMode = false;
    straightBrakeActive = false;

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

  void resetStraightLock() {
    straightLockSegmentIndex = -1;
    straightLockStartLeftCm = 0.0f;
    straightLockStartRightCm = 0.0f;
  }

  int baseStraightPwm() {
    float avgCoef = (fabs(cfg.coefLeftCmsPerPwm) + fabs(cfg.coefRightCmsPerPwm)) * 0.5f;
    if (avgCoef < 0.001f) {
      return cfg.minPwm;
    }

    int pwm = (int)round(cfg.penSpeedCms / avgCoef);
    if (pwm < cfg.minPwm) pwm = cfg.minPwm;
    if (pwm > 255) pwm = 255;
    return pwm;
  }

  float estimateStraightStopDistanceCm() {
    float distance = cfg.straightStopCompensationCm;
    float speed = fabs((odometryState.speedLeftCms + odometryState.speedRightCms) * 0.5f);

    if (cfg.straightStopDecelCms2 > 0.01f) {
      distance += (speed * speed) / (2.0f * cfg.straightStopDecelCms2);
    }

    return clampFloat(distance, 0.0f, straightTargetCm * 0.5f);
  }

  void finishStraightLine(unsigned long now, float progress, float balanceError, float stopDistance) {
    running = false;
    finished = true;
    straightLineMode = false;

    lastPwmLeft = 0;
    lastPwmRight = 0;

    status.running = running;
    status.finished = finished;
    status.currentSegment = 1;
    status.segmentCount = 1;
    status.phaseName = "STRAIGHT_DONE";
    status.progressCm = progress;
    status.segmentLengthCm = straightTargetCm;
    status.encoderBalanceErrorCm = balanceError;
    status.straightStopDistanceCm = stopDistance;
    status.vCenterCms = 0.0f;
    status.omegaRadS = 0.0f;
    status.vLeftCms = 0.0f;
    status.vRightCms = 0.0f;
    status.pwmLeft = 0;
    status.pwmRight = 0;
    status.targetPwmLeft = 0;
    status.targetPwmRight = 0;

    motorState.pwmLeft = 0;
    motorState.pwmRight = 0;

    if (cfg.straightBrakeMs > 0) {
      straightBrakeActive = true;
      straightBrakeEndMs = now + (unsigned long)cfg.straightBrakeMs;
      motorState.mode = "PEN_BRAKE";
      brakeMotors();
    } else {
      straightBrakeActive = false;
      motorState.mode = "PEN_INVERSE";
      stopMotors();
    }

    Logger::log("Ligne droite terminee. Progress=" + String(progress, 3) +
                " cm stopEst=" + String(stopDistance, 3) +
                " cm erreur encodeurs L-R=" + String(balanceError, 3) + " cm");
  }

  void updateStraightLine(unsigned long now, float dt) {
    (void)dt;

    float currentLeft = getLeftDistanceCm();
    float currentRight = getRightDistanceCm();

    float leftTravel = (currentLeft - straightStartLeftCm) * straightDirection;
    float rightTravel = (currentRight - straightStartRightCm) * straightDirection;
    float progress = (leftTravel + rightTravel) * 0.5f;
    float balanceError = leftTravel - rightTravel;
    float remaining = straightTargetCm - progress;
    float stopDistance = estimateStraightStopDistanceCm();
    float stopThreshold = fmax(cfg.segmentToleranceCm, stopDistance);

    if (remaining <= stopThreshold) {
      finishStraightLine(now, progress, balanceError, stopDistance);
      return;
    }

    int basePwm = baseStraightPwm();
    int correction = (int)round(fabs(balanceError) * cfg.straightEncoderKp);
    correction = constrain(correction, 0, 255 - basePwm);

    int pwmLeft = basePwm;
    int pwmRight = basePwm;

    if (balanceError > 0.0f) {
      pwmRight += correction;
    } else if (balanceError < 0.0f) {
      pwmLeft += correction;
    }

    pwmLeft *= straightDirection;
    pwmRight *= straightDirection;

    pwmLeft = applyContinuousSlew(lastPwmLeft, pwmLeft);
    pwmRight = applyContinuousSlew(lastPwmRight, pwmRight);

    lastPwmLeft = pwmLeft;
    lastPwmRight = pwmRight;

    setMotorPwmTracked(pwmLeft, pwmRight);

    status.running = running;
    status.finished = finished;
    status.currentSegment = 0;
    status.segmentCount = 1;
    status.phaseName = "STRAIGHT_ENCODERS";
    status.penX = odometryState.penXCm;
    status.penY = odometryState.penYCm;
    status.lateralErrorCm = 0.0f;
    status.progressCm = progress;
    status.segmentLengthCm = straightTargetCm;
    status.encoderBalanceErrorCm = balanceError;
    status.straightStopDistanceCm = stopDistance;
    status.vCenterCms = (odometryState.speedLeftCms + odometryState.speedRightCms) * 0.5f;
    status.omegaRadS = (odometryState.speedRightCms - odometryState.speedLeftCms) / cfg.wheelBaseCm;
    status.vLeftCms = odometryState.speedLeftCms;
    status.vRightCms = odometryState.speedRightCms;
    status.pwmLeft = pwmLeft;
    status.pwmRight = pwmRight;
    status.targetPwmLeft = pwmLeft;
    status.targetPwmRight = pwmRight;
  }

  void updateStraightLockSegment(int currentSegment,
                                 float progress,
                                 float length,
                                 float lateralError,
                                 float penX,
                                 float penY,
                                 float targetX,
                                 float targetY) {
    if (straightLockSegmentIndex != currentSegment) {
      straightLockSegmentIndex = currentSegment;
      straightLockStartLeftCm = getLeftDistanceCm();
      straightLockStartRightCm = getRightDistanceCm();
      pulseAccumulatorLeft = 0.0f;
      pulseAccumulatorRight = 0.0f;
    }

    float leftTravel = getLeftDistanceCm() - straightLockStartLeftCm;
    float rightTravel = getRightDistanceCm() - straightLockStartRightCm;
    float balanceError = leftTravel - rightTravel;

    int basePwm = baseStraightPwm();
    int correction = (int)round(fabs(balanceError) * cfg.straightEncoderKp);
    correction = constrain(correction, 0, 255 - basePwm);

    int pwmLeft = basePwm;
    int pwmRight = basePwm;

    if (balanceError > 0.0f) {
      pwmRight += correction;
    } else if (balanceError < 0.0f) {
      pwmLeft += correction;
    }

    pwmLeft = applyContinuousSlew(lastPwmLeft, pwmLeft);
    pwmRight = applyContinuousSlew(lastPwmRight, pwmRight);

    lastPwmLeft = pwmLeft;
    lastPwmRight = pwmRight;

    setMotorPwmTracked(pwmLeft, pwmRight);

    status.running = running;
    status.finished = finished;
    status.currentSegment = currentSegment;
    status.segmentCount = segmentCount;
    status.phaseName = "STRAIGHT_SEGMENT";
    status.cornerActive = false;
    status.remainingToCornerCm = 0.0f;
    status.cornerAngleErrorDeg = 0.0f;

    status.penX = penX;
    status.penY = penY;
    status.targetX = targetX;
    status.targetY = targetY;
    status.lateralErrorCm = lateralError;
    status.progressCm = progress;
    status.segmentLengthCm = length;
    status.encoderBalanceErrorCm = balanceError;

    status.vCenterCms = (odometryState.speedLeftCms + odometryState.speedRightCms) * 0.5f;
    status.omegaRadS = (odometryState.speedRightCms - odometryState.speedLeftCms) / cfg.wheelBaseCm;
    status.vLeftCms = odometryState.speedLeftCms;
    status.vRightCms = odometryState.speedRightCms;
    status.pwmLeft = pwmLeft;
    status.pwmRight = pwmRight;
    status.targetPwmLeft = pwmLeft;
    status.targetPwmRight = pwmRight;
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

    if (cfg.wheelBaseCm < 1.0f) cfg.wheelBaseCm = RobotParams::WHEEL_BASE_CM;
    if (cfg.penOffsetCm < 0.1f) cfg.penOffsetCm = RobotParams::PEN_OFFSET_CM;
    if (cfg.distanceScale <= 0.0f) cfg.distanceScale = 1.0f;
    if (cfg.penSpeedCms < 0.0f) cfg.penSpeedCms = 0.0f;
    if (cfg.penSpeedMaxCms < 0.1f) cfg.penSpeedMaxCms = 0.1f;
    if (cfg.wheelSpeedMaxCms < 0.1f) cfg.wheelSpeedMaxCms = 0.1f;
    if (cfg.maxOmegaRadS < 0.1f) cfg.maxOmegaRadS = 0.1f;
    if (cfg.maxNormalCorrectionCms < 0.0f) cfg.maxNormalCorrectionCms = 0.0f;
    if (cfg.minCommandSpeedCms < 0.0f) cfg.minCommandSpeedCms = 0.0f;
    if (cfg.cornerInnerBoost < 0.0f) cfg.cornerInnerBoost = 0.0f;
    if (cfg.cornerInnerBoost > 1.0f) cfg.cornerInnerBoost = 1.0f;
    if (cfg.cornerTurnMinPwm < cfg.minPwm) cfg.cornerTurnMinPwm = cfg.minPwm;
    if (cfg.cornerTurnMinPwm > 255) cfg.cornerTurnMinPwm = 255;
    if (cfg.cornerMaxDurationS < 0.1f) cfg.cornerMaxDurationS = 0.1f;
    if (cfg.minPwm < 0) cfg.minPwm = 0;
    if (cfg.minPwm > 255) cfg.minPwm = 255;
    if (cfg.pwmSlewStep < 1) cfg.pwmSlewStep = 1;
    if (cfg.pwmSlewStep > 255) cfg.pwmSlewStep = 255;
    if (cfg.straightEncoderKp < 0.0f) cfg.straightEncoderKp = 0.0f;
    if (cfg.straightStopCompensationCm < 0.0f) cfg.straightStopCompensationCm = 0.0f;
    if (cfg.straightStopDecelCms2 < 0.0f) cfg.straightStopDecelCms2 = 0.0f;
    if (cfg.straightBrakeMs < 0) cfg.straightBrakeMs = 0;
    if (cfg.straightBrakeMs > 500) cfg.straightBrakeMs = 500;
    if (cfg.segmentToleranceCm < 0.01f) cfg.segmentToleranceCm = 0.01f;

    Odometry::setGeometry(cfg.wheelBaseCm, cfg.penOffsetCm);
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
      cfg.cornerInnerBoost = prefs.getFloat("cornerBoost", cfg.cornerInnerBoost);
      cfg.cornerTurnMinPwm = prefs.getInt("cornerTurnPwm", cfg.cornerTurnMinPwm);
      cfg.cornerMaxDurationS = prefs.getFloat("cornerDur", cfg.cornerMaxDurationS);

      cfg.kp = prefs.getFloat("kp", cfg.kp);
      cfg.ki = prefs.getFloat("ki", cfg.ki);
      cfg.kd = prefs.getFloat("kd", cfg.kd);
      cfg.integralLimit = prefs.getFloat("iLimit", cfg.integralLimit);

      cfg.coefLeftCmsPerPwm = prefs.getFloat("coefL", cfg.coefLeftCmsPerPwm);
      cfg.coefRightCmsPerPwm = prefs.getFloat("coefR", cfg.coefRightCmsPerPwm);

      cfg.minPwm = prefs.getInt("minPwm", cfg.minPwm);
      cfg.pwmSlewStep = prefs.getInt("slew", cfg.pwmSlewStep);
      cfg.pwmDither = prefs.getBool("dither", cfg.pwmDither);
      cfg.straightEncoderKp = prefs.getFloat("strKp", cfg.straightEncoderKp);
      cfg.straightStopCompensationCm = prefs.getFloat("strStop", cfg.straightStopCompensationCm);
      cfg.straightStopDecelCms2 = prefs.getFloat("strDecel", cfg.straightStopDecelCms2);
      cfg.straightBrakeMs = prefs.getInt("strBrake", cfg.straightBrakeMs);

      cfg.allowReverse = prefs.getBool("reverse", cfg.allowReverse);
      cfg.minForwardSpeedCms = prefs.getFloat("minFwd", cfg.minForwardSpeedCms);

      cfg.segmentToleranceCm = prefs.getFloat("segTol", cfg.segmentToleranceCm);
    }

    prefs.end();

    Odometry::setGeometry(cfg.wheelBaseCm, cfg.penOffsetCm);
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
    prefs.putFloat("cornerBoost", cfg.cornerInnerBoost);
    prefs.putInt("cornerTurnPwm", cfg.cornerTurnMinPwm);
    prefs.putFloat("cornerDur", cfg.cornerMaxDurationS);

    prefs.putFloat("kp", cfg.kp);
    prefs.putFloat("ki", cfg.ki);
    prefs.putFloat("kd", cfg.kd);
    prefs.putFloat("iLimit", cfg.integralLimit);

    prefs.putFloat("coefL", cfg.coefLeftCmsPerPwm);
    prefs.putFloat("coefR", cfg.coefRightCmsPerPwm);

    prefs.putInt("minPwm", cfg.minPwm);
    prefs.putInt("slew", cfg.pwmSlewStep);
    prefs.putBool("dither", cfg.pwmDither);
    prefs.putFloat("strKp", cfg.straightEncoderKp);
    prefs.putFloat("strStop", cfg.straightStopCompensationCm);
    prefs.putFloat("strDecel", cfg.straightStopDecelCms2);
    prefs.putInt("strBrake", cfg.straightBrakeMs);

    prefs.putBool("reverse", cfg.allowReverse);
    prefs.putFloat("minFwd", cfg.minForwardSpeedCms);

    prefs.putFloat("segTol", cfg.segmentToleranceCm);

    prefs.end();

    Logger::log("Config PenInverseFollower enregistree en flash");
    return true;
  }

  void resetConfigToDefaults() {
    cfg = Config();
    Odometry::setGeometry(cfg.wheelBaseCm, cfg.penOffsetCm);
    applyPidConfig();
    Logger::log("Config PenInverseFollower remise aux valeurs par defaut");
  }

  void startLine(float distanceCm) {
    float d = distanceCm * cfg.distanceScale;

    copySegment(0, 0.0f, 0.0f, d, 0.0f);

    Odometry::resetPose(-cfg.penOffsetCm, 0.0f, 0.0f);
    startSegments(1);

    straightLineMode = true;
    straightDirection = (d >= 0.0f) ? 1 : -1;
    straightTargetCm = fabs(d);
    straightStartLeftCm = getLeftDistanceCm();
    straightStartRightCm = getRightDistanceCm();
    status.phaseName = "STRAIGHT_ENCODERS";
    status.straightStopDistanceCm = 0.0f;

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
    float d2 = d2Cm * cfg.distanceScale;
    float d3 = d3Cm * cfg.distanceScale;

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

    theta -= degToRad(angleRightDeg);

    Point p3 {
      p2.x + d3 * cos(theta),
      p2.y + d3 * sin(theta)
    };

    segments[0] = {p0, p1};
    segments[1] = {p1, p2};
    segments[2] = {p2, p3};

    Odometry::resetPose(-cfg.penOffsetCm, 0.0f, 0.0f);
    startSegments(3);

    Logger::log("Escalier stylo lance");
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
    if (straightBrakeActive) {
      if ((long)(now - straightBrakeEndMs) >= 0) {
        straightBrakeActive = false;
        stopMotors();
        motorState.mode = "IDLE";
      }
      return;
    }

    if (!running || finished) {
      return;
    }

    if (dt <= 0.0f) {
      dt = 0.02f;
    }

    if (straightLineMode) {
      updateStraightLine(now, dt);
      return;
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

    bool hasNextSegment = (segmentIndex + 1) < segmentCount;
    bool hasPreviousSegment = segmentIndex > 0;
    float remainingToCorner = length - progress;
    float cornerAngleErrorDeg = 0.0f;
    float speedBasedCornerApproachCm = fmin(4.0f, fabs(cfg.penSpeedCms) * 0.22f);
    float dtBasedCornerApproachCm = fmin(4.0f, fabs(cfg.penSpeedCms) * dt * 2.5f);
    float cornerTriggerCm = fmax(cfg.cornerApproachCm,
                                 fmax(speedBasedCornerApproachCm, dtBasedCornerApproachCm));
    float leavingCornerDistanceCm = fmax(cornerTriggerCm * 2.0f,
                                         fmin(cfg.lookaheadCm, 4.0f));
    bool approachingCorner = cfg.cornerMode && hasNextSegment &&
                             remainingToCorner <= cornerTriggerCm;
    bool leavingCorner = false;

    bool nearCornerCandidate = cfg.cornerMode &&
                               (approachingCorner ||
                                (hasPreviousSegment && progress <= leavingCornerDistanceCm));

    if (nearCornerCandidate) {
      float desiredHeading = atan2(uy, ux);

      if (approachingCorner) {
        Segment next = segments[segmentIndex + 1];
        float ndx = next.b.x - next.a.x;
        float ndy = next.b.y - next.a.y;
        if (sqrt(ndx * ndx + ndy * ndy) > 0.001f) {
          desiredHeading = atan2(ndy, ndx);
        }
      }

      cornerAngleErrorDeg = radToDeg(normalizeRad(desiredHeading - odometryState.thetaRad));

      if (hasPreviousSegment &&
          progress <= leavingCornerDistanceCm &&
          fabs(cornerAngleErrorDeg) > cfg.cornerExitAngleDeg) {
        leavingCorner = true;
      }
    }

    bool cornerActive = approachingCorner || leavingCorner;
    float endTolerance = cfg.segmentToleranceCm;
    if (cfg.cornerMode && hasNextSegment) {
      endTolerance = fmin(endTolerance, 0.05f);
    }

    if (progress >= length - endTolerance) {
      segmentIndex++;
      resetPid(pidLine);
      pulseAccumulatorLeft = 0.0f;
      pulseAccumulatorRight = 0.0f;
      resetStraightLock();

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

    float effectiveLookahead = cfg.lookaheadCm;

    if (cornerActive) {
      if (approachingCorner) {
        effectiveLookahead = fmin(effectiveLookahead, fmax(0.0f, remainingToCorner));
      }

      if (leavingCorner) {
        float cornerLookahead = fmax(0.25f, progress * 0.75f);
        effectiveLookahead = fmin(effectiveLookahead, cornerLookahead);
      }
    }

    float lookProgress = clampFloat(progress + effectiveLookahead, 0.0f, length);

    float targetX = s.a.x + lookProgress * ux;
    float targetY = s.a.y + lookProgress * uy;

    if (cornerActive) {
      resetStraightLock();
    }

    float errorX = targetX - penX;
    float errorY = targetY - penY;

    float pidNormal = updatePid(pidLine, lateralError, dt, cfg.integralLimit);
    float normalCorrection = cfg.lineGain * lateralError + pidNormal;

    status.normalCorrectionLimited = false;
    if (cfg.maxNormalCorrectionCms > 0.0f && fabs(normalCorrection) > cfg.maxNormalCorrectionCms) {
      normalCorrection = (normalCorrection > 0.0f) ? cfg.maxNormalCorrectionCms : -cfg.maxNormalCorrectionCms;
      status.normalCorrectionLimited = true;
    }

    float pathSpeed = cfg.penSpeedCms;
    if (cornerActive && cfg.cornerSpeedCms > 0.0f && cfg.cornerSpeedCms < pathSpeed) {
      pathSpeed = cfg.cornerSpeedCms;
    }

    float vPenX = pathSpeed * ux
                + cfg.targetGain * errorX
                - normalCorrection * nx;

    float vPenY = pathSpeed * uy
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

    float omegaLimit = cornerActive ? cfg.cornerOmegaRadS : cfg.maxOmegaRadS;
    if (omegaLimit > 0.0f && fabs(omega) > omegaLimit) {
      omega = (omega > 0.0f) ? omegaLimit : -omegaLimit;
      status.omegaLimited = true;
    }

    float vLeftDesired = vCenter - omega * cfg.wheelBaseCm * 0.5f;
    float vRightDesired = vCenter + omega * cfg.wheelBaseCm * 0.5f;

    vLeftDesired = clampFloat(vLeftDesired, -cfg.wheelSpeedMaxCms, cfg.wheelSpeedMaxCms);
    vRightDesired = clampFloat(vRightDesired, -cfg.wheelSpeedMaxCms, cfg.wheelSpeedMaxCms);

    if (fabs(vLeftDesired) < cfg.minCommandSpeedCms) vLeftDesired = 0.0f;
    if (fabs(vRightDesired) < cfg.minCommandSpeedCms) vRightDesired = 0.0f;

    int pwmLeftTarget = 0;
    int pwmRightTarget = 0;

    int rawLeftPwm = leftSpeedToRawPwm(vLeftDesired);
    int rawRightPwm = rightSpeedToRawPwm(vRightDesired);
    bool rawOppositeWheelTurn = (rawLeftPwm < 0 && rawRightPwm > 0) ||
                                (rawLeftPwm > 0 && rawRightPwm < 0);
    bool precisionCorner = segmentCount > 1 &&
                           (cornerActive || rawOppositeWheelTurn) &&
                           (isPulseRange(rawLeftPwm) || isPulseRange(rawRightPwm));

    if (precisionCorner) {
      pwmLeftTarget = pulsePwmCommand(rawLeftPwm, pulseAccumulatorLeft);
      pwmRightTarget = pulsePwmCommand(rawRightPwm, pulseAccumulatorRight);
    } else {
      pulseAccumulatorLeft = 0.0f;
      pulseAccumulatorRight = 0.0f;
      scaledWheelPwmCommands(vLeftDesired, vRightDesired, pwmLeftTarget, pwmRightTarget);
    }

    bool oppositeWheelTurn = (pwmLeftTarget < 0 && pwmRightTarget > 0) ||
                             (pwmLeftTarget > 0 && pwmRightTarget < 0);
    bool turnAssistActive = !precisionCorner &&
                            (cornerActive || (segmentCount > 1 && oppositeWheelTurn)) &&
                            cfg.cornerInnerBoost > 0.0f;

    if (turnAssistActive) {
      boostInnerWheelPwmInCorner(vLeftDesired, vRightDesired, pwmLeftTarget, pwmRightTarget);
    }

    if (!precisionCorner && segmentCount > 1 && oppositeWheelTurn && cfg.cornerTurnMinPwm > cfg.minPwm) {
      applyCornerTurnMinimum(pwmLeftTarget, pwmRightTarget);
    }

    bool leftSignFlip = segmentCount > 1 && lastPwmLeft != 0 && pwmLeftTarget != 0 &&
                        ((lastPwmLeft > 0 && pwmLeftTarget < 0) ||
                         (lastPwmLeft < 0 && pwmLeftTarget > 0));
    bool rightSignFlip = segmentCount > 1 && lastPwmRight != 0 && pwmRightTarget != 0 &&
                         ((lastPwmRight > 0 && pwmRightTarget < 0) ||
                          (lastPwmRight < 0 && pwmRightTarget > 0));

    int pwmLeft = applyContinuousSlew(lastPwmLeft, pwmLeftTarget, leftSignFlip);
    int pwmRight = applyContinuousSlew(lastPwmRight, pwmRightTarget, rightSignFlip);

    lastPwmLeft = pwmLeft;
    lastPwmRight = pwmRight;

    setMotorPwmTracked(pwmLeft, pwmRight);

    float vLeftActual = pwmToLeftSpeed(pwmLeft);
    float vRightActual = pwmToRightSpeed(pwmRight);

    status.running = running;
    status.finished = finished;
    status.currentSegment = segmentIndex;
    status.segmentCount = segmentCount;
    status.phaseName = precisionCorner ? "CORNER_PRECISION" : (cornerActive ? "CORNER" : "FOLLOW_SEGMENT");
    status.cornerActive = cornerActive;
    status.remainingToCornerCm = fmax(0.0f, remainingToCorner);
    status.cornerAngleErrorDeg = cornerAngleErrorDeg;

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

    printPwmTrace(now,
                  pwmLeftTarget,
                  pwmRightTarget,
                  pwmLeft,
                  pwmRight,
                  vLeftDesired,
                  vRightDesired,
                  remainingToCorner);
  }

  bool isRunning() {
    return running;
  }

  bool isFinished() {
    return finished;
  }
}

#include <Arduino.h>
#include <math.h>

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

  PenInverseFollower::Config cfg;
  PenInverseFollower::Status status;

  static const int MAX_SEGMENTS = 5;
  PenInverseFollower::Segment segments[MAX_SEGMENTS];

  int segmentCount = 0;
  int segmentIndex = 0;

  bool running = false;
  bool finished = false;

  PID pidLine;

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

  int leftSpeedToPwm(float speedCms) {
    if (fabs(cfg.coefLeftCmsPerPwm) < 0.001f) return 0;

    int pwm = (int)round(speedCms / cfg.coefLeftCmsPerPwm);
    return applyMinPwm(pwm);
  }

  int rightSpeedToPwm(float speedCms) {
    if (fabs(cfg.coefRightCmsPerPwm) < 0.001f) return 0;

    int pwm = (int)round(speedCms / cfg.coefRightCmsPerPwm);
    return applyMinPwm(pwm);
  }

  void setMotorPwmTracked(int pwmLeft, int pwmRight) {
    pwmLeft = constrain(pwmLeft, -255, 255);
    pwmRight = constrain(pwmRight, -255, 255);

    motorState.pwmLeft = pwmLeft;
    motorState.pwmRight = pwmRight;
    motorState.mode = "PEN_INVERSE";

    setMotors(pwmLeft, pwmRight);
  }

  void resetStatus() {
    status = PenInverseFollower::Status();

    status.running = running;
    status.finished = finished;
    status.currentSegment = segmentIndex;
    status.segmentCount = segmentCount;
  }

  void startSegments(int count) {
    segmentCount = count;
    segmentIndex = 0;

    running = true;
    finished = false;

    applyPidConfig();
    resetStatus();

    Logger::log("PenInverseFollower demarre avec " + String(segmentCount) + " segment(s)");
  }
}

namespace PenInverseFollower {
  void begin() {
    running = false;
    finished = false;

    segmentCount = 0;
    segmentIndex = 0;

    applyPidConfig();
    resetStatus();

    Logger::log("PenInverseFollower pret");
  }

  void setConfig(const Config& newCfg) {
    cfg = newCfg;
    applyPidConfig();

    Logger::log("Config PenInverseFollower mise a jour");
  }

  Config getConfig() {
    return cfg;
  }

  Status getStatus() {
    return status;
  }

  void startLine(float distanceCm) {
    segments[0] = {
      {0.0f, 0.0f},
      {distanceCm, 0.0f}
    };

    startSegments(1);

    Logger::log("Test ligne stylo : distance=" + String(distanceCm, 1) + " cm");
  }

  void startOneAngle(float d1Cm, float angleDeg, float d2Cm) {
    Point p0 {0.0f, 0.0f};

    float theta = 0.0f;

    Point p1 {
      p0.x + d1Cm * cos(theta),
      p0.y + d1Cm * sin(theta)
    };

    theta += degToRad(angleDeg);

    Point p2 {
      p1.x + d2Cm * cos(theta),
      p1.y + d2Cm * sin(theta)
    };

    segments[0] = {p0, p1};
    segments[1] = {p1, p2};

    startSegments(2);

    Logger::log("Test un angle stylo : d1=" + String(d1Cm, 1) +
                " angle=" + String(angleDeg, 1) +
                " d2=" + String(d2Cm, 1));
  }

  void startStair(float d1Cm, float angleLeftDeg, float d2Cm, float angleRightDeg, float d3Cm) {
    Point p0 {0.0f, 0.0f};

    float theta = 0.0f;

    Point p1 {
      p0.x + d1Cm * cos(theta),
      p0.y + d1Cm * sin(theta)
    };

    theta += degToRad(angleLeftDeg);

    Point p2 {
      p1.x + d2Cm * cos(theta),
      p1.y + d2Cm * sin(theta)
    };

    theta -= degToRad(angleRightDeg);

    Point p3 {
      p2.x + d3Cm * cos(theta),
      p2.y + d3Cm * sin(theta)
    };

    segments[0] = {p0, p1};
    segments[1] = {p1, p2};
    segments[2] = {p2, p3};

    startSegments(3);

    Logger::log("Escalier stylo lance : d1=" + String(d1Cm, 1) +
                " aL=" + String(angleLeftDeg, 1) +
                " d2=" + String(d2Cm, 1) +
                " aR=" + String(angleRightDeg, 1) +
                " d3=" + String(d3Cm, 1));
  }

  void stop() {
    running = false;
    finished = false;

    setMotorPwmTracked(0, 0);
    stopMotors();

    motorState.mode = "IDLE";

    resetStatus();

    Logger::log("PenInverseFollower stop");
  }

  void update(unsigned long now, float dt) {
    (void)now;

    if (!running || finished) {
      return;
    }

    if (segmentIndex >= segmentCount) {
      running = false;
      finished = true;

      setMotorPwmTracked(0, 0);
      stopMotors();

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

    if (progress >= length - cfg.segmentToleranceCm) {
      segmentIndex++;
      resetPid(pidLine);

      Logger::log("Passage segment stylo " + String(segmentIndex + 1));

      if (segmentIndex >= segmentCount) {
        running = false;
        finished = true;
        setMotorPwmTracked(0, 0);
        stopMotors();
        motorState.mode = "IDLE";
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

    float pidNormal = updatePid(pidLine, lateralError, dt, cfg.integralLimit);

    float vPenX = cfg.penSpeedCms * ux
                + cfg.targetGain * errorX
                - (cfg.lineGain * lateralError + pidNormal) * nx;

    float vPenY = cfg.penSpeedCms * uy
                + cfg.targetGain * errorY
                - (cfg.lineGain * lateralError + pidNormal) * ny;

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

    float vLeft = vCenter - omega * cfg.wheelBaseCm * 0.5f;
    float vRight = vCenter + omega * cfg.wheelBaseCm * 0.5f;

    vLeft = clampFloat(vLeft, -cfg.wheelSpeedMaxCms, cfg.wheelSpeedMaxCms);
    vRight = clampFloat(vRight, -cfg.wheelSpeedMaxCms, cfg.wheelSpeedMaxCms);

    int pwmLeft = leftSpeedToPwm(vLeft);
    int pwmRight = rightSpeedToPwm(vRight);

    setMotorPwmTracked(pwmLeft, pwmRight);

    status.running = running;
    status.finished = finished;
    status.currentSegment = segmentIndex;
    status.segmentCount = segmentCount;

    status.penX = penX;
    status.penY = penY;

    status.targetX = targetX;
    status.targetY = targetY;

    status.lateralErrorCm = lateralError;
    status.progressCm = progress;
    status.segmentLengthCm = length;

    status.vPenX = vPenX;
    status.vPenY = vPenY;

    status.vCenterCms = vCenter;
    status.omegaRadS = omega;

    status.vLeftCms = vLeft;
    status.vRightCms = vRight;

    status.pwmLeft = pwmLeft;
    status.pwmRight = pwmRight;
  }

  bool isRunning() {
    return running;
  }

  bool isFinished() {
    return finished;
  }
}

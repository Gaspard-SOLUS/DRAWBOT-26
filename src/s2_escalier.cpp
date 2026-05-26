#include <Arduino.h>
#include <math.h>
#include <Preferences.h>

#include "s2_escalier.h"
#include "moteurs.h"
#include "odometry.h"
#include "sensors.h"
#include "app_state.h"
#include "logger.h"

namespace S2Escalier {

  struct Vec2 { float x = 0.0f; float y = 0.0f; };

  enum class Phase {
    NONE,
    LINE_1,
    PIVOT_LEFT,
    LINE_2,
    PIVOT_RIGHT,
    LINE_3,
    DONE
  };

  static Preferences prefs;
  static Config cfg;
  static RuntimeStatus st;

  static Phase phase = Phase::NONE;
  static Phase nextPhaseAfterPause = Phase::NONE;
  static unsigned long pauseUntilMs = 0;
  static unsigned long lastLogMs = 0;

  static float theta0Rad = 0.0f;
  static float theta0Deg = 0.0f;
  static float startGyroDeg = 0.0f;
  static float startMagDeg = 0.0f;

  static Vec2 lineStart;
  static Vec2 lineTarget;
  static Vec2 lineDir;
  static float lineLengthCm = 0.0f;
  static float lineTargetThetaRad = 0.0f;
  static float lineTargetThetaDeg = 0.0f;

  static float targetPivotThetaRad = 0.0f;
  static float targetPivotThetaDeg = 0.0f;

  static float headingIntegral = 0.0f;
  static float lastHeadingErrRad = 0.0f;
  static bool headingPidPrimed = false;

  static int lastPwmLeft = 0;
  static int lastPwmRight = 0;

  static float constrainFloat(float v, float mn, float mx) {
    if (v < mn) return mn;
    if (v > mx) return mx;
    return v;
  }

  static float radToDegLocal(float r) { return r * 180.0f / PI; }
  static float degToRadLocal(float d) { return d * PI / 180.0f; }

  static float normalizeDeg(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  }

  static float normalizeRad(float a) {
    while (a > PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
  }

  static float currentThetaRad() {
    if (cfg.headingSource == 1 && sensorState.imuOk) {
      float deltaDeg = normalizeDeg(sensorState.yawGyroDeg - startGyroDeg);
      return theta0Rad + degToRadLocal(deltaDeg);
    }

    if (cfg.headingSource == 2 && sensorState.magOk && sensorState.magCalibrationDone) {
      float deltaDeg = normalizeDeg(sensorState.headingMagDeg - startMagDeg);
      return theta0Rad + degToRadLocal(deltaDeg);
    }

    return odometryState.thetaRad;
  }

  static Vec2 currentPenPosition(float theta) {
    Vec2 p;
    p.x = odometryState.xCm + cfg.penOffsetCm * cos(theta);
    p.y = odometryState.yCm + cfg.penOffsetCm * sin(theta);
    return p;
  }

  static int signInt(int v) {
    if (v > 0) return 1;
    if (v < 0) return -1;
    return 0;
  }

  static int rampOnePwm(int current, int target) {
    int step = constrain(cfg.pwmRampStep, 1, 255);

    if (signInt(current) != 0 && signInt(target) != 0 && signInt(current) != signInt(target)) {
      if (abs(current) <= step) return 0;
      return current - signInt(current) * step;
    }

    int delta = target - current;
    if (delta > step) delta = step;
    if (delta < -step) delta = -step;
    return current + delta;
  }

  static int wheelSpeedToPwm(float speedCms) {
    if (fabs(speedCms) < 0.08f) return 0;

    float a = fabs(speedCms) / cfg.maxWheelSpeedCms;
    a = constrainFloat(a, 0.0f, 1.0f);

    int pwm = cfg.minPwm + (int)((cfg.maxPwm - cfg.minPwm) * a);
    pwm = constrain(pwm, cfg.minPwm, cfg.maxPwm);
    return speedCms < 0.0f ? -pwm : pwm;
  }

  static void applyPwmTargets(int targetLeft, int targetRight, bool useRamp = true) {
    targetLeft = constrain(targetLeft, -255, 255);
    targetRight = constrain(targetRight, -255, 255);

    if (useRamp) {
      lastPwmLeft = rampOnePwm(lastPwmLeft, targetLeft);
      lastPwmRight = rampOnePwm(lastPwmRight, targetRight);
    } else {
      lastPwmLeft = targetLeft;
      lastPwmRight = targetRight;
    }

    st.pwmLeft = lastPwmLeft;
    st.pwmRight = lastPwmRight;
    motorState.pwmLeft = lastPwmLeft;
    motorState.pwmRight = lastPwmRight;
    motorState.mode = "S2_ESCALIER";
    setMotors(lastPwmLeft, lastPwmRight);
  }

  static void applyWheelSpeeds(float vLeftCms, float vRightCms) {
    int targetLeft = wheelSpeedToPwm(vLeftCms);
    int targetRight = wheelSpeedToPwm(vRightCms);
    applyPwmTargets(targetLeft, targetRight, true);
  }

  static void hardStop() {
    stopMotors();
    lastPwmLeft = 0;
    lastPwmRight = 0;
    st.pwmLeft = 0;
    st.pwmRight = 0;
    motorState.pwmLeft = 0;
    motorState.pwmRight = 0;
  }

  static void resetHeadingPid() {
    headingIntegral = 0.0f;
    lastHeadingErrRad = 0.0f;
    headingPidPrimed = false;
  }

  static void beginPause(unsigned long now, Phase nextPhase) {
    hardStop();
    resetHeadingPid();
    nextPhaseAfterPause = nextPhase;
    pauseUntilMs = now + cfg.cornerPauseMs;
    st.state = State::CORNER_PAUSE;
  }

  static void startLineFromCurrentPen(Phase linePhase, float headingRad, float lengthCm) {
    float theta = currentThetaRad();
    Vec2 pen = currentPenPosition(theta);

    lineStart = pen;
    lineLengthCm = lengthCm;
    lineTargetThetaRad = headingRad;
    lineTargetThetaDeg = normalizeDeg(radToDegLocal(headingRad));
    lineDir.x = cos(headingRad);
    lineDir.y = sin(headingRad);

    lineTarget.x = lineStart.x + lengthCm * lineDir.x;
    lineTarget.y = lineStart.y + lengthCm * lineDir.y;

    phase = linePhase;
    st.state = State::LINE_RUNNING;
    st.currentSegmentLengthCm = lengthCm;
    st.progressCm = 0.0f;
    st.remainingCm = lengthCm;
    st.targetThetaDeg = lineTargetThetaDeg;
    resetHeadingPid();

    if (linePhase == Phase::LINE_1) st.segmentIndex = 0;
    else if (linePhase == Phase::LINE_2) st.segmentIndex = 1;
    else if (linePhase == Phase::LINE_3) st.segmentIndex = 2;

    Logger::log("Escalier : demarrage ligne " + String(st.segmentIndex + 1) +
                " cible cap=" + String(lineTargetThetaDeg, 1) +
                " longueur=" + String(lengthCm, 1));
  }

  static void startPivot(Phase pivotPhase, float targetThetaRad) {
    phase = pivotPhase;
    st.state = State::PIVOT_RUNNING;
    targetPivotThetaRad = targetThetaRad;
    targetPivotThetaDeg = normalizeDeg(radToDegLocal(targetThetaRad));
    st.targetThetaDeg = targetPivotThetaDeg;
    st.progressCm = 0.0f;
    st.remainingCm = 0.0f;
    st.currentSegmentLengthCm = 0.0f;
    resetHeadingPid();

    Logger::log("Escalier : pivot vers cap=" + String(targetPivotThetaDeg, 1));
  }

  static void finishSequence() {
    hardStop();
    phase = Phase::DONE;
    st.state = State::FINISHED;
    motorState.mode = "S2_ESCALIER_FINISHED";
    Logger::log("Sequence escalier terminee");
  }

  static bool updateLine(unsigned long now, float dt) {
    if (dt <= 0.001f) dt = 0.02f;

    float theta = currentThetaRad();
    Vec2 pen = currentPenPosition(theta);

    Vec2 AP;
    AP.x = pen.x - lineStart.x;
    AP.y = pen.y - lineStart.y;

    float progress = AP.x * lineDir.x + AP.y * lineDir.y;
    float lateral = AP.x * (-lineDir.y) + AP.y * lineDir.x;
    float remaining = lineLengthCm - progress;

    st.penX = pen.x;
    st.penY = pen.y;
    st.progressCm = progress;
    st.remainingCm = remaining;
    st.currentSegmentLengthCm = lineLengthCm;
    st.lateralErrorCm = lateral;
    st.thetaDeg = normalizeDeg(radToDegLocal(theta));
    st.targetThetaDeg = lineTargetThetaDeg;

    if (progress >= lineLengthCm - cfg.endToleranceCm) {
      if (phase == Phase::LINE_1) {
        beginPause(now, Phase::PIVOT_LEFT);
      } else if (phase == Phase::LINE_2) {
        beginPause(now, Phase::PIVOT_RIGHT);
      } else {
        finishSequence();
      }
      return true;
    }

    float speedScale = constrainFloat(remaining / cfg.slowZoneCm, 0.35f, 1.0f);
    float vCenter = cfg.lineSpeedCms * speedScale;

    float headingErrRad = normalizeRad(lineTargetThetaRad - theta);
    float dErr = 0.0f;
    if (!headingPidPrimed) {
      lastHeadingErrRad = headingErrRad;
      headingPidPrimed = true;
    } else {
      dErr = (headingErrRad - lastHeadingErrRad) / dt;
      lastHeadingErrRad = headingErrRad;
    }

    headingIntegral += headingErrRad * dt;
    headingIntegral = constrainFloat(headingIntegral, -cfg.headingIntegralLimit, cfg.headingIntegralLimit);

    float omega = cfg.kpHeading * headingErrRad + cfg.kiHeading * headingIntegral + cfg.kdHeading * dErr;
    omega = constrainFloat(omega, -cfg.maxOmegaRadS, cfg.maxOmegaRadS);

    float vLeft = vCenter - omega * cfg.wheelBaseCm * 0.5f;
    float vRight = vCenter + omega * cfg.wheelBaseCm * 0.5f;

    st.headingErrorDeg = radToDegLocal(headingErrRad);
    st.activeSpeedCms = vCenter;
    st.activeOmegaRadS = omega;
    st.activeLatCorrectionCms = 0.0f;
    st.entryZoneActive = false;

    applyWheelSpeeds(vLeft, vRight);
    return false;
  }

  static bool updatePivot(unsigned long now) {
    float theta = currentThetaRad();
    Vec2 pen = currentPenPosition(theta);
    float errRad = normalizeRad(targetPivotThetaRad - theta);
    float errDeg = radToDegLocal(errRad);

    st.penX = pen.x;
    st.penY = pen.y;
    st.thetaDeg = normalizeDeg(radToDegLocal(theta));
    st.targetThetaDeg = targetPivotThetaDeg;
    st.headingErrorDeg = errDeg;
    st.lateralErrorCm = 0.0f;
    st.activeLatCorrectionCms = 0.0f;
    st.entryZoneActive = false;

    if (fabs(errDeg) <= cfg.pivotAngleToleranceDeg) {
      if (phase == Phase::PIVOT_LEFT) {
        beginPause(now, Phase::LINE_2);
      } else {
        beginPause(now, Phase::LINE_3);
      }
      return true;
    }

    float scale = constrainFloat(fabs(errDeg) / cfg.pivotSlowdownDeg, 0.45f, 1.0f);
    int slow = (int)(cfg.pivotSlowPwm * scale);
    slow = constrain(slow, cfg.pivotMinPwm, cfg.pivotMaxPwm);

    int fast = (int)(slow * cfg.pivotRatio);
    fast = constrain(fast, cfg.pivotMinPwm, cfg.pivotMaxPwm);

    int leftPwm;
    int rightPwm;

    if (errRad > 0.0f) {
      // Tourner vers la gauche : roue droite plus rapide.
      leftPwm = slow;
      rightPwm = fast;
    } else {
      // Tourner vers la droite : roue gauche plus rapide.
      leftPwm = fast;
      rightPwm = slow;
    }

    st.activeSpeedCms = 0.0f;
    st.activeOmegaRadS = errRad;
    applyPwmTargets(leftPwm, rightPwm, true);
    return false;
  }

  void begin() {
    loadConfig();
    reset();
    Logger::log("Module S2 Escalier : lignes avec maintien de cap + pivots autour du stylo");
  }

  void update(unsigned long now, float dt) {
    if (st.state == State::CORNER_PAUSE) {
      hardStop();
      if (now < pauseUntilMs) return;

      if (nextPhaseAfterPause == Phase::PIVOT_LEFT) {
        startPivot(Phase::PIVOT_LEFT, theta0Rad + PI * 0.5f);
      } else if (nextPhaseAfterPause == Phase::LINE_2) {
        startLineFromCurrentPen(Phase::LINE_2, theta0Rad + PI * 0.5f, cfg.dist2Cm);
      } else if (nextPhaseAfterPause == Phase::PIVOT_RIGHT) {
        startPivot(Phase::PIVOT_RIGHT, theta0Rad);
      } else if (nextPhaseAfterPause == Phase::LINE_3) {
        startLineFromCurrentPen(Phase::LINE_3, theta0Rad, cfg.dist3Cm);
      } else {
        finishSequence();
      }
      nextPhaseAfterPause = Phase::NONE;
      return;
    }

    if (st.state == State::LINE_RUNNING) {
      updateLine(now, dt);
    } else if (st.state == State::PIVOT_RUNNING) {
      updatePivot(now);
    } else {
      return;
    }

    if (now - lastLogMs >= 250) {
      lastLogMs = now;
      Logger::log("ESC phase=" + String((int)phase) +
                  " seg=" + String(st.segmentIndex + 1) +
                  " prog=" + String(st.progressCm, 1) +
                  " headErr=" + String(st.headingErrorDeg, 1) +
                  " pwmL=" + String(st.pwmLeft) +
                  " pwmR=" + String(st.pwmRight));
    }
  }

  void start() {
    hardStop();
    Odometry::reset();
    Sensors::resetGyroYaw();

    theta0Rad = odometryState.thetaRad;
    theta0Deg = normalizeDeg(radToDegLocal(theta0Rad));
    startGyroDeg = sensorState.yawGyroDeg;
    startMagDeg = sensorState.headingMagDeg;

    st = RuntimeStatus();
    phase = Phase::LINE_1;
    nextPhaseAfterPause = Phase::NONE;
    pauseUntilMs = 0;
    resetHeadingPid();

    startLineFromCurrentPen(Phase::LINE_1, theta0Rad, cfg.dist1Cm);
    motorState.mode = "S2_ESCALIER_RUNNING";
    Logger::log("Sequence escalier demarree : ligne/pivot/ligne/pivot/ligne");
  }

  void stop() {
    hardStop();
    if (st.state != State::FINISHED) {
      st.state = State::IDLE;
      phase = Phase::NONE;
      motorState.mode = "IDLE";
    }
  }

  void reset() {
    hardStop();
    resetHeadingPid();
    st = RuntimeStatus();
    st.state = State::IDLE;
    phase = Phase::NONE;
    nextPhaseAfterPause = Phase::NONE;
    pauseUntilMs = 0;
    Odometry::reset();
  }

  Config getConfig() { return cfg; }

  void setConfig(const Config& newCfg) {
    cfg = newCfg;

    cfg.dist1Cm = constrainFloat(cfg.dist1Cm, 1.0f, 200.0f);
    cfg.dist2Cm = constrainFloat(cfg.dist2Cm, 1.0f, 200.0f);
    cfg.dist3Cm = constrainFloat(cfg.dist3Cm, 1.0f, 300.0f);

    cfg.wheelBaseCm = constrainFloat(cfg.wheelBaseCm, 3.0f, 30.0f);
    cfg.penOffsetCm = constrainFloat(cfg.penOffsetCm, 1.0f, 30.0f);

    cfg.lineSpeedCms = constrainFloat(cfg.lineSpeedCms, 0.3f, 12.0f);
    cfg.slowZoneCm = constrainFloat(cfg.slowZoneCm, 0.5f, 20.0f);
    cfg.endToleranceCm = constrainFloat(cfg.endToleranceCm, 0.05f, 3.0f);
    cfg.cornerPauseMs = constrain((int)cfg.cornerPauseMs, 0, 2000);

    cfg.minPwm = constrain(cfg.minPwm, 0, 255);
    cfg.maxPwm = constrain(cfg.maxPwm, cfg.minPwm, 255);
    cfg.maxWheelSpeedCms = constrainFloat(cfg.maxWheelSpeedCms, 1.0f, 80.0f);
    cfg.pwmRampStep = constrain(cfg.pwmRampStep, 1, 255);

    cfg.kpHeading = constrainFloat(cfg.kpHeading, 0.0f, 10.0f);
    cfg.kiHeading = constrainFloat(cfg.kiHeading, 0.0f, 2.0f);
    cfg.kdHeading = constrainFloat(cfg.kdHeading, 0.0f, 3.0f);
    cfg.maxOmegaRadS = constrainFloat(cfg.maxOmegaRadS, 0.05f, 5.0f);
    cfg.headingIntegralLimit = constrainFloat(cfg.headingIntegralLimit, 0.0f, 5.0f);

    cfg.pivotSlowPwm = constrain(cfg.pivotSlowPwm, 0, 255);
    cfg.pivotRatio = constrainFloat(cfg.pivotRatio, 1.0f, 4.0f);
    cfg.pivotMinPwm = constrain(cfg.pivotMinPwm, 0, 255);
    cfg.pivotMaxPwm = constrain(cfg.pivotMaxPwm, cfg.pivotMinPwm, 255);
    cfg.pivotAngleToleranceDeg = constrainFloat(cfg.pivotAngleToleranceDeg, 0.5f, 15.0f);
    cfg.pivotSlowdownDeg = constrainFloat(cfg.pivotSlowdownDeg, 2.0f, 90.0f);

    cfg.headingSource = constrain(cfg.headingSource, 0, 2);
  }

  void loadConfig() {
    prefs.begin("s2esc", true);
    cfg.dist1Cm = prefs.getFloat("d1", cfg.dist1Cm);
    cfg.dist2Cm = prefs.getFloat("d2", cfg.dist2Cm);
    cfg.dist3Cm = prefs.getFloat("d3", cfg.dist3Cm);
    cfg.wheelBaseCm = prefs.getFloat("wb", cfg.wheelBaseCm);
    cfg.penOffsetCm = prefs.getFloat("po", cfg.penOffsetCm);

    cfg.lineSpeedCms = prefs.getFloat("ls", cfg.lineSpeedCms);
    cfg.slowZoneCm = prefs.getFloat("sz", cfg.slowZoneCm);
    cfg.endToleranceCm = prefs.getFloat("et", cfg.endToleranceCm);
    cfg.cornerPauseMs = prefs.getULong("cpause", cfg.cornerPauseMs);

    cfg.minPwm = prefs.getInt("minp", cfg.minPwm);
    cfg.maxPwm = prefs.getInt("maxp", cfg.maxPwm);
    cfg.maxWheelSpeedCms = prefs.getFloat("mws", cfg.maxWheelSpeedCms);
    cfg.pwmRampStep = prefs.getInt("ramp", cfg.pwmRampStep);

    cfg.kpHeading = prefs.getFloat("khp", cfg.kpHeading);
    cfg.kiHeading = prefs.getFloat("khi", cfg.kiHeading);
    cfg.kdHeading = prefs.getFloat("khd", cfg.kdHeading);
    cfg.maxOmegaRadS = prefs.getFloat("omax", cfg.maxOmegaRadS);
    cfg.headingIntegralLimit = prefs.getFloat("hil", cfg.headingIntegralLimit);

    cfg.pivotSlowPwm = prefs.getInt("pslow", cfg.pivotSlowPwm);
    cfg.pivotRatio = prefs.getFloat("pratio", cfg.pivotRatio);
    cfg.pivotMinPwm = prefs.getInt("pmin", cfg.pivotMinPwm);
    cfg.pivotMaxPwm = prefs.getInt("pmax", cfg.pivotMaxPwm);
    cfg.pivotAngleToleranceDeg = prefs.getFloat("ptol", cfg.pivotAngleToleranceDeg);
    cfg.pivotSlowdownDeg = prefs.getFloat("pslowd", cfg.pivotSlowdownDeg);

    cfg.headingSource = prefs.getInt("hs", cfg.headingSource);
    prefs.end();
    setConfig(cfg);
  }

  void saveConfig() {
    prefs.begin("s2esc", false);
    prefs.putFloat("d1", cfg.dist1Cm);
    prefs.putFloat("d2", cfg.dist2Cm);
    prefs.putFloat("d3", cfg.dist3Cm);
    prefs.putFloat("wb", cfg.wheelBaseCm);
    prefs.putFloat("po", cfg.penOffsetCm);

    prefs.putFloat("ls", cfg.lineSpeedCms);
    prefs.putFloat("sz", cfg.slowZoneCm);
    prefs.putFloat("et", cfg.endToleranceCm);
    prefs.putULong("cpause", cfg.cornerPauseMs);

    prefs.putInt("minp", cfg.minPwm);
    prefs.putInt("maxp", cfg.maxPwm);
    prefs.putFloat("mws", cfg.maxWheelSpeedCms);
    prefs.putInt("ramp", cfg.pwmRampStep);

    prefs.putFloat("khp", cfg.kpHeading);
    prefs.putFloat("khi", cfg.kiHeading);
    prefs.putFloat("khd", cfg.kdHeading);
    prefs.putFloat("omax", cfg.maxOmegaRadS);
    prefs.putFloat("hil", cfg.headingIntegralLimit);

    prefs.putInt("pslow", cfg.pivotSlowPwm);
    prefs.putFloat("pratio", cfg.pivotRatio);
    prefs.putInt("pmin", cfg.pivotMinPwm);
    prefs.putInt("pmax", cfg.pivotMaxPwm);
    prefs.putFloat("ptol", cfg.pivotAngleToleranceDeg);
    prefs.putFloat("pslowd", cfg.pivotSlowdownDeg);

    prefs.putInt("hs", cfg.headingSource);
    prefs.end();
  }

  RuntimeStatus getStatus() { return st; }

  String stateName() {
    switch (st.state) {
      case State::IDLE: return "IDLE";
      case State::LINE_RUNNING: return "LINE_RUNNING";
      case State::PIVOT_RUNNING: return "PIVOT_RUNNING";
      case State::CORNER_PAUSE: return "CORNER_PAUSE";
      case State::FINISHED: return "FINISHED";
      case State::ERROR: return "ERROR";
    }
    return "UNKNOWN";
  }

  String configJson() {
    String j = "{";
    j += "\"dist1Cm\":" + String(cfg.dist1Cm, 3) + ",";
    j += "\"dist2Cm\":" + String(cfg.dist2Cm, 3) + ",";
    j += "\"dist3Cm\":" + String(cfg.dist3Cm, 3) + ",";
    j += "\"wheelBaseCm\":" + String(cfg.wheelBaseCm, 3) + ",";
    j += "\"penOffsetCm\":" + String(cfg.penOffsetCm, 3) + ",";
    j += "\"lineSpeedCms\":" + String(cfg.lineSpeedCms, 3) + ",";
    j += "\"penSpeedCms\":" + String(cfg.lineSpeedCms, 3) + ",";
    j += "\"slowZoneCm\":" + String(cfg.slowZoneCm, 3) + ",";
    j += "\"endToleranceCm\":" + String(cfg.endToleranceCm, 3) + ",";
    j += "\"cornerPauseMs\":" + String(cfg.cornerPauseMs) + ",";
    j += "\"minPwm\":" + String(cfg.minPwm) + ",";
    j += "\"maxPwm\":" + String(cfg.maxPwm) + ",";
    j += "\"maxWheelSpeedCms\":" + String(cfg.maxWheelSpeedCms, 3) + ",";
    j += "\"pwmRampStep\":" + String(cfg.pwmRampStep) + ",";
    j += "\"kpHeading\":" + String(cfg.kpHeading, 4) + ",";
    j += "\"kiHeading\":" + String(cfg.kiHeading, 4) + ",";
    j += "\"kdHeading\":" + String(cfg.kdHeading, 4) + ",";
    j += "\"maxOmegaRadS\":" + String(cfg.maxOmegaRadS, 4) + ",";
    j += "\"headingIntegralLimit\":" + String(cfg.headingIntegralLimit, 4) + ",";
    j += "\"pivotSlowPwm\":" + String(cfg.pivotSlowPwm) + ",";
    j += "\"pivotRatio\":" + String(cfg.pivotRatio, 4) + ",";
    j += "\"pivotMinPwm\":" + String(cfg.pivotMinPwm) + ",";
    j += "\"pivotMaxPwm\":" + String(cfg.pivotMaxPwm) + ",";
    j += "\"pivotAngleToleranceDeg\":" + String(cfg.pivotAngleToleranceDeg, 3) + ",";
    j += "\"pivotSlowdownDeg\":" + String(cfg.pivotSlowdownDeg, 3) + ",";
    j += "\"headingSource\":" + String(cfg.headingSource);
    j += "}";
    return j;
  }

  String statusJson() {
    String j = "{";
    j += "\"state\":\"" + stateName() + "\",";
    j += "\"phaseIndex\":" + String((int)phase) + ",";
    j += "\"segmentIndex\":" + String(st.segmentIndex) + ",";
    j += "\"penX\":" + String(st.penX, 3) + ",";
    j += "\"penY\":" + String(st.penY, 3) + ",";
    j += "\"progressCm\":" + String(st.progressCm, 3) + ",";
    j += "\"remainingCm\":" + String(st.remainingCm, 3) + ",";
    j += "\"currentSegmentLengthCm\":" + String(st.currentSegmentLengthCm, 3) + ",";
    j += "\"lateralErrorCm\":" + String(st.lateralErrorCm, 3) + ",";
    j += "\"headingErrorDeg\":" + String(st.headingErrorDeg, 3) + ",";
    j += "\"thetaDeg\":" + String(st.thetaDeg, 3) + ",";
    j += "\"targetThetaDeg\":" + String(st.targetThetaDeg, 3) + ",";
    j += "\"activeSpeedCms\":" + String(st.activeSpeedCms, 3) + ",";
    j += "\"activeOmegaRadS\":" + String(st.activeOmegaRadS, 4) + ",";
    j += "\"activeLatCorrectionCms\":" + String(st.activeLatCorrectionCms, 3) + ",";
    j += "\"pwmLeft\":" + String(st.pwmLeft) + ",";
    j += "\"pwmRight\":" + String(st.pwmRight) + ",";
    j += "\"entryZoneActive\":false";
    j += "}";
    return j;
  }
}

#include <Arduino.h>
#include <math.h>
#include <Preferences.h>

#include "s2_escalier.h"
#include "moteurs.h"
#include "encodeurs.h"
#include "odometry.h"
#include "sensors.h"
#include "app_state.h"
#include "logger.h"

namespace S2Escalier {

  struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
  };

  static Preferences prefs;
  static Config cfg;
  static RuntimeStatus st;

  static Vec2 points[4];
  static unsigned long lastLogMs = 0;

  static float lastLatErr = 0.0f;
  static float latIntegral = 0.0f;
  static bool derivativeReady = false;

  static float startThetaOdoRad = 0.0f;
  static float startGyroDeg = 0.0f;
  static float startMagDeg = 0.0f;

  static float constrainFloat(float v, float mn, float mx) {
    if (v < mn) return mn;
    if (v > mx) return mx;
    return v;
  }

  static float degToRad(float deg) {
    return deg * PI / 180.0f;
  }

  static float radToDegLocal(float rad) {
    return rad * 180.0f / PI;
  }

  static float angleDiffDeg(float target, float current) {
    float e = target - current;
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
  }

  static float currentThetaRad() {
    if (cfg.headingSource == 1 && sensorState.imuOk) {
      float delta = angleDiffDeg(sensorState.yawGyroDeg, startGyroDeg);
      return startThetaOdoRad + degToRad(delta);
    }

    if (cfg.headingSource == 2 && sensorState.magOk && sensorState.magCalibrationDone) {
      float delta = angleDiffDeg(sensorState.headingMagDeg, startMagDeg);
      return startThetaOdoRad + degToRad(delta);
    }

    return odometryState.thetaRad;
  }

  static Vec2 currentPenPosition(float theta) {
    Vec2 p;
    p.x = odometryState.xCm + cfg.penOffsetCm * cos(theta);
    p.y = odometryState.yCm + cfg.penOffsetCm * sin(theta);
    return p;
  }

  static int wheelSpeedToPwm(float speedCms) {
    const float deadband = 0.08f;
    if (fabs(speedCms) < deadband) return 0;

    float a = fabs(speedCms) / cfg.maxWheelSpeedCms;
    a = constrainFloat(a, 0.0f, 1.0f);

    int pwm = cfg.minPwm + (int)((cfg.maxPwm - cfg.minPwm) * a);
    pwm = constrain(pwm, cfg.minPwm, cfg.maxPwm);

    return speedCms < 0.0f ? -pwm : pwm;
  }

  static void applyWheelSpeeds(float vLeftCms, float vRightCms) {
    int pwmL = wheelSpeedToPwm(vLeftCms);
    int pwmR = wheelSpeedToPwm(vRightCms);

    pwmL = constrain(pwmL, -cfg.maxPwm, cfg.maxPwm);
    pwmR = constrain(pwmR, -cfg.maxPwm, cfg.maxPwm);

    st.pwmLeft = pwmL;
    st.pwmRight = pwmR;

    motorState.pwmLeft = pwmL;
    motorState.pwmRight = pwmR;
    motorState.mode = "S2_ESCALIER_2PID";

    setMotors(pwmL, pwmR);
  }

  static void buildTargetPolyline() {
    float theta0 = currentThetaRad();
    Vec2 pen0 = currentPenPosition(theta0);

    Vec2 t;
    t.x = cos(theta0);
    t.y = sin(theta0);

    Vec2 n;
    n.x = -sin(theta0);
    n.y = cos(theta0);

    points[0] = pen0;

    points[1].x = points[0].x + cfg.dist1Cm * t.x;
    points[1].y = points[0].y + cfg.dist1Cm * t.y;

    points[2].x = points[1].x + cfg.dist2Cm * n.x;
    points[2].y = points[1].y + cfg.dist2Cm * n.y;

    points[3].x = points[2].x + cfg.dist3Cm * t.x;
    points[3].y = points[2].y + cfg.dist3Cm * t.y;
  }

  static void resetPidTerms() {
    lastLatErr = 0.0f;
    latIntegral = 0.0f;
    derivativeReady = false;
  }

  static const PidProfile& activeProfileFor(int segmentIndex, float progressCm, float segmentLengthCm) {
    // PID A : segment 1 + premiere moitie du segment 2.
    // PID B : deuxieme moitie du segment 2 + segment 3.
    if (segmentIndex == 0) {
      st.activePid = 1;
      return cfg.pidA;
    }

    if (segmentIndex == 1 && progressCm < segmentLengthCm * 0.5f) {
      st.activePid = 1;
      return cfg.pidA;
    }

    st.activePid = 2;
    return cfg.pidB;
  }

  static bool goToNextSegmentIfNeeded(float progressCm, float segmentLengthCm) {
    if (progressCm < segmentLengthCm - cfg.endToleranceCm) {
      return false;
    }

    st.segmentIndex++;
    resetPidTerms();

    if (st.segmentIndex >= 3) {
      stop();
      st.state = State::FINISHED;
      motorState.mode = "S2_ESCALIER_FINISHED";
      Logger::log("Sequence escalier terminee");
      return true;
    }

    Logger::log("Passage au segment " + String(st.segmentIndex + 1));
    return true;
  }

  static void updateTrajectoryTracking(float dt) {
    if (dt <= 0.001f) dt = 0.02f;

    Vec2 A = points[st.segmentIndex];
    Vec2 B = points[st.segmentIndex + 1];

    Vec2 AB;
    AB.x = B.x - A.x;
    AB.y = B.y - A.y;

    float len = sqrt(AB.x * AB.x + AB.y * AB.y);
    if (len < 0.001f) {
      st.state = State::ERROR;
      stopMotors();
      Logger::log("Erreur escalier : segment de longueur nulle");
      return;
    }

    Vec2 t;
    t.x = AB.x / len;
    t.y = AB.y / len;

    Vec2 n;
    n.x = -t.y;
    n.y = t.x;

    float theta = currentThetaRad();
    Vec2 P = currentPenPosition(theta);

    Vec2 AP;
    AP.x = P.x - A.x;
    AP.y = P.y - A.y;

    float progress = AP.x * t.x + AP.y * t.y;
    float lateralError = AP.x * n.x + AP.y * n.y;

    st.penX = P.x;
    st.penY = P.y;
    st.progressCm = progress;
    st.currentSegmentLengthCm = len;
    st.lateralErrorCm = lateralError;
    st.thetaDeg = radToDegLocal(theta);

    if (goToNextSegmentIfNeeded(progress, len)) {
      return;
    }

    const PidProfile& pid = activeProfileFor(st.segmentIndex, progress, len);

    st.activeKp = pid.kpLat;
    st.activeKi = pid.kiLat;
    st.activeKd = pid.kdLat;
    st.activeSpeedCms = pid.penSpeedCms;
    st.activeMaxCorrectionCms = pid.maxLatCorrectionCms;

    float remaining = len - progress;
    float speedScale = constrainFloat(remaining / cfg.slowZoneCm, 0.35f, 1.0f);
    float vAlong = pid.penSpeedCms * speedScale;

    latIntegral += lateralError * dt;
    latIntegral = constrainFloat(latIntegral, -cfg.integralLimit, cfg.integralLimit);

    float dErr = 0.0f;
    if (derivativeReady) {
      dErr = (lateralError - lastLatErr) / dt;
    } else {
      // Evite le coup de derive au debut d'un segment ou au changement PID A -> PID B.
      dErr = 0.0f;
      derivativeReady = true;
    }
    lastLatErr = lateralError;

    // Correction laterale PID sur le point stylo.
    // Signe negatif : si le stylo est a gauche de la ligne, on corrige vers la droite.
    float vLat = -(pid.kpLat * lateralError + pid.kiLat * latIntegral + pid.kdLat * dErr);
    vLat = constrainFloat(vLat, -pid.maxLatCorrectionCms, pid.maxLatCorrectionCms);

    // Vitesse desiree du STYLO en coordonnees globales.
    float vPenX = vAlong * t.x + vLat * n.x;
    float vPenY = vAlong * t.y + vLat * n.y;

    // Inversion cinematique pour un point situe a cfg.penOffsetCm devant l'axe des roues.
    float c = cos(theta);
    float sTheta = sin(theta);

    float vCenter = c * vPenX + sTheta * vPenY;
    float omega = (-sTheta * vPenX + c * vPenY) / cfg.penOffsetCm;

    float vLeft = vCenter - omega * cfg.wheelBaseCm * 0.5f;
    float vRight = vCenter + omega * cfg.wheelBaseCm * 0.5f;

    applyWheelSpeeds(vLeft, vRight);
  }

  void begin() {
    loadConfig();
    reset();
    Logger::log("Module S2 Escalier 2 PID pret");
  }

  void update(unsigned long now, float dt) {
    if (st.state != State::RUNNING) {
      return;
    }

    updateTrajectoryTracking(dt);

    if (now - lastLogMs >= 250) {
      lastLogMs = now;
      Logger::log("ESC seg=" + String(st.segmentIndex + 1) +
                  " pid=" + String(st.activePid) +
                  " s=" + String(st.progressCm, 1) +
                  " e=" + String(st.lateralErrorCm, 2) +
                  " kp=" + String(st.activeKp, 2) +
                  " kd=" + String(st.activeKd, 2) +
                  " pwmL=" + String(st.pwmLeft) +
                  " pwmR=" + String(st.pwmRight));
    }
  }

  void start() {
    stopMotors();
    Odometry::reset();
    Sensors::resetGyroYaw();

    startThetaOdoRad = odometryState.thetaRad;
    startGyroDeg = sensorState.yawGyroDeg;
    startMagDeg = sensorState.headingMagDeg;

    st = RuntimeStatus();
    st.segmentIndex = 0;
    st.state = State::RUNNING;

    resetPidTerms();
    buildTargetPolyline();

    motorState.mode = "S2_ESCALIER_RUNNING";
    Logger::log("Sequence escalier demarree : 2 PID lateraux");
  }

  void stop() {
    stopMotors();
    st.pwmLeft = 0;
    st.pwmRight = 0;

    motorState.pwmLeft = 0;
    motorState.pwmRight = 0;

    if (st.state != State::FINISHED) {
      st.state = State::IDLE;
      motorState.mode = "IDLE";
    }
  }

  void reset() {
    stopMotors();
    resetPidTerms();
    st = RuntimeStatus();
    st.state = State::IDLE;
    st.segmentIndex = 0;
    Odometry::reset();
    motorState.mode = "IDLE";
  }

  Config getConfig() {
    return cfg;
  }

  void setConfig(const Config& c) {
    cfg = c;

    cfg.dist1Cm = max(1.0f, cfg.dist1Cm);
    cfg.dist2Cm = max(1.0f, cfg.dist2Cm);
    cfg.dist3Cm = max(1.0f, cfg.dist3Cm);

    cfg.penOffsetCm = max(1.0f, cfg.penOffsetCm);
    cfg.wheelBaseCm = max(1.0f, cfg.wheelBaseCm);
    cfg.slowZoneCm = max(0.1f, cfg.slowZoneCm);
    cfg.endToleranceCm = constrainFloat(cfg.endToleranceCm, 0.05f, 2.0f);

    cfg.maxWheelSpeedCms = max(1.0f, cfg.maxWheelSpeedCms);
    cfg.minPwm = constrain(cfg.minPwm, 0, 255);
    cfg.maxPwm = constrain(cfg.maxPwm, cfg.minPwm, 255);

    cfg.pidA.penSpeedCms = constrainFloat(cfg.pidA.penSpeedCms, 0.4f, 10.0f);
    cfg.pidB.penSpeedCms = constrainFloat(cfg.pidB.penSpeedCms, 0.4f, 10.0f);

    cfg.pidA.maxLatCorrectionCms = constrainFloat(cfg.pidA.maxLatCorrectionCms, 0.1f, 10.0f);
    cfg.pidB.maxLatCorrectionCms = constrainFloat(cfg.pidB.maxLatCorrectionCms, 0.1f, 10.0f);

    cfg.integralLimit = constrainFloat(cfg.integralLimit, 0.0f, 50.0f);
    cfg.headingSource = constrain(cfg.headingSource, 0, 2);
  }

  void loadConfig() {
    prefs.begin("s2esc2pid", true);

    cfg.dist1Cm = prefs.getFloat("d1", cfg.dist1Cm);
    cfg.dist2Cm = prefs.getFloat("d2", cfg.dist2Cm);
    cfg.dist3Cm = prefs.getFloat("d3", cfg.dist3Cm);

    cfg.wheelBaseCm = prefs.getFloat("wb", cfg.wheelBaseCm);
    cfg.penOffsetCm = prefs.getFloat("po", cfg.penOffsetCm);
    cfg.slowZoneCm = prefs.getFloat("slow", cfg.slowZoneCm);
    cfg.endToleranceCm = prefs.getFloat("tol", cfg.endToleranceCm);

    cfg.minPwm = prefs.getInt("minp", cfg.minPwm);
    cfg.maxPwm = prefs.getInt("maxp", cfg.maxPwm);
    cfg.maxWheelSpeedCms = prefs.getFloat("vwmax", cfg.maxWheelSpeedCms);

    cfg.pidA.penSpeedCms = prefs.getFloat("a_v", cfg.pidA.penSpeedCms);
    cfg.pidA.kpLat = prefs.getFloat("a_kp", cfg.pidA.kpLat);
    cfg.pidA.kiLat = prefs.getFloat("a_ki", cfg.pidA.kiLat);
    cfg.pidA.kdLat = prefs.getFloat("a_kd", cfg.pidA.kdLat);
    cfg.pidA.maxLatCorrectionCms = prefs.getFloat("a_max", cfg.pidA.maxLatCorrectionCms);

    cfg.pidB.penSpeedCms = prefs.getFloat("b_v", cfg.pidB.penSpeedCms);
    cfg.pidB.kpLat = prefs.getFloat("b_kp", cfg.pidB.kpLat);
    cfg.pidB.kiLat = prefs.getFloat("b_ki", cfg.pidB.kiLat);
    cfg.pidB.kdLat = prefs.getFloat("b_kd", cfg.pidB.kdLat);
    cfg.pidB.maxLatCorrectionCms = prefs.getFloat("b_max", cfg.pidB.maxLatCorrectionCms);

    cfg.integralLimit = prefs.getFloat("ilim", cfg.integralLimit);
    cfg.headingSource = prefs.getInt("head", cfg.headingSource);

    prefs.end();
    setConfig(cfg);
  }

  void saveConfig() {
    prefs.begin("s2esc2pid", false);

    prefs.putFloat("d1", cfg.dist1Cm);
    prefs.putFloat("d2", cfg.dist2Cm);
    prefs.putFloat("d3", cfg.dist3Cm);

    prefs.putFloat("wb", cfg.wheelBaseCm);
    prefs.putFloat("po", cfg.penOffsetCm);
    prefs.putFloat("slow", cfg.slowZoneCm);
    prefs.putFloat("tol", cfg.endToleranceCm);

    prefs.putInt("minp", cfg.minPwm);
    prefs.putInt("maxp", cfg.maxPwm);
    prefs.putFloat("vwmax", cfg.maxWheelSpeedCms);

    prefs.putFloat("a_v", cfg.pidA.penSpeedCms);
    prefs.putFloat("a_kp", cfg.pidA.kpLat);
    prefs.putFloat("a_ki", cfg.pidA.kiLat);
    prefs.putFloat("a_kd", cfg.pidA.kdLat);
    prefs.putFloat("a_max", cfg.pidA.maxLatCorrectionCms);

    prefs.putFloat("b_v", cfg.pidB.penSpeedCms);
    prefs.putFloat("b_kp", cfg.pidB.kpLat);
    prefs.putFloat("b_ki", cfg.pidB.kiLat);
    prefs.putFloat("b_kd", cfg.pidB.kdLat);
    prefs.putFloat("b_max", cfg.pidB.maxLatCorrectionCms);

    prefs.putFloat("ilim", cfg.integralLimit);
    prefs.putInt("head", cfg.headingSource);

    prefs.end();
    Logger::log("Config S2 Escalier 2 PID sauvegardee");
  }

  RuntimeStatus getStatus() {
    return st;
  }

  String stateName() {
    switch (st.state) {
      case State::IDLE: return "IDLE";
      case State::RUNNING: return "RUNNING";
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
    j += "\"slowZoneCm\":" + String(cfg.slowZoneCm, 3) + ",";
    j += "\"endToleranceCm\":" + String(cfg.endToleranceCm, 3) + ",";
    j += "\"minPwm\":" + String(cfg.minPwm) + ",";
    j += "\"maxPwm\":" + String(cfg.maxPwm) + ",";
    j += "\"maxWheelSpeedCms\":" + String(cfg.maxWheelSpeedCms, 3) + ",";

    j += "\"speedA\":" + String(cfg.pidA.penSpeedCms, 3) + ",";
    j += "\"kpA\":" + String(cfg.pidA.kpLat, 5) + ",";
    j += "\"kiA\":" + String(cfg.pidA.kiLat, 5) + ",";
    j += "\"kdA\":" + String(cfg.pidA.kdLat, 5) + ",";
    j += "\"maxCorrA\":" + String(cfg.pidA.maxLatCorrectionCms, 3) + ",";

    j += "\"speedB\":" + String(cfg.pidB.penSpeedCms, 3) + ",";
    j += "\"kpB\":" + String(cfg.pidB.kpLat, 5) + ",";
    j += "\"kiB\":" + String(cfg.pidB.kiLat, 5) + ",";
    j += "\"kdB\":" + String(cfg.pidB.kdLat, 5) + ",";
    j += "\"maxCorrB\":" + String(cfg.pidB.maxLatCorrectionCms, 3) + ",";

    j += "\"integralLimit\":" + String(cfg.integralLimit, 3) + ",";
    j += "\"headingSource\":" + String(cfg.headingSource);
    j += "}";
    return j;
  }

  String statusJson() {
    String j = "{";
    j += "\"state\":\"" + stateName() + "\",";
    j += "\"segmentIndex\":" + String(st.segmentIndex) + ",";
    j += "\"activePid\":" + String(st.activePid) + ",";
    j += "\"penX\":" + String(st.penX, 3) + ",";
    j += "\"penY\":" + String(st.penY, 3) + ",";
    j += "\"progressCm\":" + String(st.progressCm, 3) + ",";
    j += "\"segmentLengthCm\":" + String(st.currentSegmentLengthCm, 3) + ",";
    j += "\"lateralErrorCm\":" + String(st.lateralErrorCm, 3) + ",";
    j += "\"thetaDeg\":" + String(st.thetaDeg, 3) + ",";
    j += "\"activeKp\":" + String(st.activeKp, 5) + ",";
    j += "\"activeKi\":" + String(st.activeKi, 5) + ",";
    j += "\"activeKd\":" + String(st.activeKd, 5) + ",";
    j += "\"activeSpeedCms\":" + String(st.activeSpeedCms, 3) + ",";
    j += "\"activeMaxCorrectionCms\":" + String(st.activeMaxCorrectionCms, 3) + ",";
    j += "\"pwmLeft\":" + String(st.pwmLeft) + ",";
    j += "\"pwmRight\":" + String(st.pwmRight);
    j += "}";
    return j;
  }
}

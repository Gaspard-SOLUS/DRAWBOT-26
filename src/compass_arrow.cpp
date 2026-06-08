#include <Arduino.h>
#include <math.h>

#include "compass_arrow.h"
#include "app_state.h"
#include "logger.h"
#include "moteurs.h"
#include "odometry.h"
#include "pen_inverse_follower.h"
#include "robot.h"
#include "sensors.h"
#include "soutenance2.h"
#include "trajectory_generator.h"

namespace {
  enum class Phase {
    Idle,
    Calibrating,
    Aligning,
    AlignSettling,
    Drawing,
    Done,
    Error
  };

  Phase phase = Phase::Idle;
  CompassArrow::Config cfg;
  CompassArrow::Status status;

  unsigned long settleStartMs = 0;
  unsigned long alignStartMs = 0;
  float alignInitialHeadingDeg = 0.0f;
  float alignTargetRad = 0.0f;
  float alignTargetDeg = 0.0f;
  float alignProgressDeg = 0.0f;
  float alignRemainingDeg = 0.0f;
  bool drawAfterAlignment = false;
  bool followerConfigSaved = false;
  PenInverseFollower::Config previousFollowerConfig;

  const float ALIGN_ODOM_TOLERANCE_DEG = 3.0f;
  const float ALIGN_MIN_TARGET_DEG = 1.0f;

  bool startDrawing();

  const char* phaseName(Phase value) {
    switch (value) {
      case Phase::Idle: return "IDLE";
      case Phase::Calibrating: return "CALIBRATION";
      case Phase::Aligning: return "ALIGNEMENT_NORD";
      case Phase::AlignSettling: return "STABILISATION_NORD";
      case Phase::Drawing: return "DESSIN_FLECHE";
      case Phase::Done: return "TERMINE";
      case Phase::Error: return "ERREUR";
    }

    return "INCONNU";
  }

  int safePwm(int pwm) {
    int absPwm = abs(pwm);
    if (absPwm == 0) return 0;
    if (absPwm < 180) absPwm = 180;
    if (absPwm > 255) absPwm = 255;
    return absPwm;
  }

  void setRoseMotors(int left, int right, const String& mode) {
    motorState.pwmLeft = constrain(left, -255, 255);
    motorState.pwmRight = constrain(right, -255, 255);
    motorState.mode = mode;

    status.pwmLeft = motorState.pwmLeft;
    status.pwmRight = motorState.pwmRight;

    setMotors(motorState.pwmLeft, motorState.pwmRight);
  }

  void stopRoseMotors() {
    if (motorState.mode == "ROSE_CALIB" || motorState.mode == "ROSE_ALIGN") {
      motorState.pwmLeft = 0;
      motorState.pwmRight = 0;
      motorState.mode = "IDLE";
      stopMotors();
    }

    status.pwmLeft = 0;
    status.pwmRight = 0;
  }

  void spinSameDirection(int pwm, const String& mode) {
    int commandPwm = safePwm(pwm);

    if (cfg.clockwise) {
      setRoseMotors(commandPwm, -commandPwm, mode);
    } else {
      setRoseMotors(-commandPwm, commandPwm, mode);
    }
  }

  void spinSigned(int pwm, float signedTurnRad, const String& mode) {
    int commandPwm = safePwm(pwm);

    if (signedTurnRad >= 0.0f) {
      setRoseMotors(-commandPwm, commandPwm, mode);
    } else {
      setRoseMotors(commandPwm, -commandPwm, mode);
    }
  }

  void updateStatusBase() {
    status.running = phase == Phase::Calibrating ||
                     phase == Phase::Aligning ||
                     phase == Phase::AlignSettling ||
                     phase == Phase::Drawing;
    status.calibrating = phase == Phase::Calibrating;
    status.drawing = phase == Phase::Drawing;
    status.finished = phase == Phase::Done;
    status.phaseName = phaseName(phase);
    status.headingDeg = Sensors::normalizeAngleDeg(sensorState.headingMagDeg);
    status.northErrorDeg = CompassArrow::northErrorDeg(status.headingDeg);
    status.inNorthWindow = CompassArrow::isInNorthWindow(status.headingDeg);
    status.alignInitialHeadingDeg = alignInitialHeadingDeg;
    status.alignTargetDeg = alignTargetDeg;
    if (phase == Phase::Aligning || phase == Phase::AlignSettling) {
      float progressDeg = fabsf(odometryState.thetaRad) * 180.0f / PI;
      float targetDeg = fabsf(alignTargetDeg);
      alignProgressDeg = fminf(progressDeg, targetDeg);
      alignRemainingDeg = fmaxf(0.0f, targetDeg - alignProgressDeg);
    }

    status.alignProgressDeg = alignProgressDeg;
    status.alignRemainingDeg = alignRemainingDeg;
  }

  bool addMove(TrajectoryGenerator::Trajectory& trajectory, TrajectoryGenerator::Point& current, TrajectoryGenerator::Point next) {
    if (!TrajectoryGenerator::addSegment(trajectory, current, next)) {
      return false;
    }

    current = next;
    return true;
  }

  bool buildArrowTrajectory(TrajectoryGenerator::Trajectory& trajectory, const CompassArrow::Config& arrowCfg, float distanceScale) {
    TrajectoryGenerator::clear(trajectory);

    float shaft = arrowCfg.shaftCm * distanceScale;
    float side = arrowCfg.headSideCm * distanceScale;
    float fillStep = arrowCfg.fillStepCm * distanceScale;

    if (shaft <= 0.1f || side <= 0.1f || fillStep <= 0.05f) {
      return false;
    }

    float halfBase = side * 0.5f;
    float headHeight = side * sqrt(3.0f) * 0.5f;

    float baseX = shaft;
    float tipX = shaft + headHeight;

    TrajectoryGenerator::Point current {0.0f, 0.0f};
    TrajectoryGenerator::Point baseCenter {baseX, 0.0f};
    TrajectoryGenerator::Point leftBase {baseX, halfBase};
    TrajectoryGenerator::Point tip {tipX, 0.0f};
    TrajectoryGenerator::Point rightBase {baseX, -halfBase};

    // Trait principal, demi-base gauche, puis fermeture du triangle.
    if (!addMove(trajectory, current, baseCenter)) return false;
    if (!addMove(trajectory, current, leftBase)) return false;
    if (!addMove(trajectory, current, tip)) return false;
    if (!addMove(trajectory, current, rightBase)) return false;
    if (!addMove(trajectory, current, baseCenter)) return false;

    bool leftToRight = true;
    float x = baseX + fillStep;

    while (x <= tipX + 0.001f) {
      float t = (x - baseX) / headHeight;
      t = constrain(t, 0.0f, 1.0f);

      float halfWidth = halfBase * (1.0f - t);
      TrajectoryGenerator::Point left {x, halfWidth};
      TrajectoryGenerator::Point right {x, -halfWidth};

      if (leftToRight) {
        if (!addMove(trajectory, current, left)) return false;
        if (!addMove(trajectory, current, right)) return false;
      } else {
        if (!addMove(trajectory, current, right)) return false;
        if (!addMove(trajectory, current, left)) return false;
      }

      leftToRight = !leftToRight;
      x += fillStep;
    }

    if (!addMove(trajectory, current, tip)) return false;
    return trajectory.count > 0;
  }

  void restoreFollowerConfigIfNeeded() {
    if (!followerConfigSaved) {
      return;
    }

    PenInverseFollower::setConfig(previousFollowerConfig);
    followerConfigSaved = false;
  }

  bool magnetometerCanStart(const char* actionName) {
    if (!sensorState.magOk) {
      status.message = "LIS3MDL non detecte";
      phase = Phase::Error;
      Logger::log(String("Erreur fleche Nord ") + actionName + " : magnetometre non detecte");
      return false;
    }

    unsigned long now = millis();
    bool recentRead = sensorState.magReadCount > 0 &&
                      sensorState.magLastReadMs > 0 &&
                      now - sensorState.magLastReadMs <= 1000;

    if (!recentRead) {
      status.message = "Cap magnetique invalide";
      phase = Phase::Error;
      Logger::log(String("Erreur fleche Nord ") + actionName +
                  " : lecture magnetometre absente ou trop ancienne");
      return false;
    }

    if (sensorState.magCalibrationRunning) {
      status.message = "Calibration magnetometre en cours";
      Logger::log(String("Erreur fleche Nord ") + actionName + " : attendre la fin de calibration");
      return false;
    }

    return true;
  }

  float signedTurnToNorthDeg(float headingDeg) {
    float h = Sensors::normalizeAngleDeg(headingDeg);
    if (CompassArrow::isInNorthWindow(h)) {
      return 0.0f;
    }

    if (h <= 180.0f) {
      return h;
    }

    return h - 360.0f;
  }

  bool startComputedNorthAlignment(bool drawAfter, const char* logName) {
    if (!magnetometerCanStart(logName)) {
      return false;
    }

    PenInverseFollower::stop();
    Soutenance2::stop();
    stopRoseMotors();
    restoreFollowerConfigIfNeeded();

    drawAfterAlignment = drawAfter;
    alignInitialHeadingDeg = Sensors::normalizeAngleDeg(sensorState.headingMagDeg);
    alignTargetDeg = signedTurnToNorthDeg(alignInitialHeadingDeg);
    alignTargetRad = alignTargetDeg * PI / 180.0f;
    alignProgressDeg = 0.0f;
    alignRemainingDeg = fabs(alignTargetDeg);
    alignStartMs = millis();

    status.finished = false;
    status.trajectorySegments = 0;

    if (fabs(alignTargetDeg) <= ALIGN_MIN_TARGET_DEG) {
      stopRoseMotors();
      phase = Phase::Done;
      status.message = "Cap initial deja dans la fenetre Nord";
      updateStatusBase();
      Logger::log(String("Alignement Nord non lance : cap initial=") +
                  String(alignInitialHeadingDeg, 1) +
                  " deg, cible=0 deg");

      if (drawAfterAlignment) {
        return startDrawing();
      }

      return true;
    }

    Odometry::resetPose(0.0f, 0.0f, 0.0f);

    phase = Phase::Aligning;
    status.message = drawAfter ? "Rotation calculee Nord puis dessin"
                               : "Rotation calculee vers le Nord";
    updateStatusBase();

    Logger::log(String("Alignement Nord calcule : capInitial=") +
                String(alignInitialHeadingDeg, 1) +
                " deg rotation=" + String(alignTargetDeg, 1) +
                " deg distanceRoue=" +
                String(fabs(alignTargetRad) * RobotParams::WHEEL_BASE_CM * 0.5f, 2) +
                " cm sens=" + String(alignTargetDeg >= 0.0f ? "gauche" : "droite") +
                " pwm=" + String(cfg.alignPwm));

    return true;
  }

  bool startDrawing() {
    stopRoseMotors();

    previousFollowerConfig = PenInverseFollower::getConfig();
    followerConfigSaved = true;

    PenInverseFollower::Config drawCfg = previousFollowerConfig;
    drawCfg.minPwm = max(drawCfg.minPwm, 180);
    drawCfg.pwmDither = false;
    drawCfg.allowReverse = true;

    if (cfg.drawSpeedCms > 0.1f) {
      drawCfg.penSpeedCms = cfg.drawSpeedCms;
      drawCfg.penSpeedMaxCms = max(drawCfg.penSpeedMaxCms, cfg.drawSpeedCms + 2.0f);
    }

    PenInverseFollower::setConfig(drawCfg);

    TrajectoryGenerator::Trajectory trajectory;
    if (!buildArrowTrajectory(trajectory, cfg, drawCfg.distanceScale)) {
      restoreFollowerConfigIfNeeded();
      status.message = "Erreur generation trajectoire fleche";
      phase = Phase::Error;
      Logger::log(status.message);
      return false;
    }

    Odometry::resetPose(-drawCfg.penOffsetCm, 0.0f, 0.0f);

    if (!PenInverseFollower::startTrajectory(trajectory)) {
      restoreFollowerConfigIfNeeded();
      status.message = "Erreur lancement follower fleche";
      phase = Phase::Error;
      Logger::log(status.message);
      return false;
    }

    status.trajectorySegments = trajectory.count;
    status.message = "Fleche Nord en cours";
    phase = Phase::Drawing;

    Logger::log("Fleche Nord lancee : trait=" + String(cfg.shaftCm, 1) +
                " cm triangle=" + String(cfg.headSideCm, 1) +
                " cm remplissage=" + String(cfg.fillStepCm, 2) +
                " cm segments=" + String(trajectory.count) +
                " vitesse=" + String(drawCfg.penSpeedCms, 1) +
                " cm/s");

    return true;
  }

  void applyConfig(const CompassArrow::Config& newCfg) {
    cfg = newCfg;

    cfg.shaftCm = constrain(cfg.shaftCm, 2.0f, 30.0f);
    cfg.headSideCm = constrain(cfg.headSideCm, 1.0f, 8.0f);
    cfg.fillStepCm = constrain(cfg.fillStepCm, 0.15f, 2.0f);
    cfg.drawSpeedCms = constrain(cfg.drawSpeedCms, 2.0f, 16.0f);

    cfg.alignPwm = safePwm(cfg.alignPwm);
    cfg.calibrationPwm = safePwm(cfg.calibrationPwm);

    cfg.settleMs = constrain(cfg.settleMs, 50UL, 1500UL);
    cfg.alignTimeoutMs = constrain(cfg.alignTimeoutMs, 3000UL, 60000UL);
  }
}

namespace CompassArrow {
  void begin() {
    phase = Phase::Idle;
    cfg = Config();
    alignInitialHeadingDeg = 0.0f;
    alignTargetRad = 0.0f;
    alignTargetDeg = 0.0f;
    alignProgressDeg = 0.0f;
    alignRemainingDeg = 0.0f;
    status = Status();
    updateStatusBase();
    Logger::log("Module fleche Nord pret");
  }

  Config defaultConfig() {
    return Config();
  }

  float northErrorDeg(float headingDeg) {
    float h = Sensors::normalizeAngleDeg(headingDeg);
    return min(h, 360.0f - h);
  }

  bool isInNorthWindow(float headingDeg) {
    float h = Sensors::normalizeAngleDeg(headingDeg);
    return h >= 358.0f || h <= 3.0f;
  }

  Status getStatus() {
    updateStatusBase();
    return status;
  }

  bool startCalibration(const Config& config) {
    applyConfig(config);

    if (!sensorState.magOk) {
      status.message = "LIS3MDL non detecte";
      phase = Phase::Error;
      Logger::log("Erreur fleche Nord : magnetometre non detecte");
      return false;
    }

    PenInverseFollower::stop();
    Soutenance2::stop();
    stopRoseMotors();
    restoreFollowerConfigIfNeeded();
    drawAfterAlignment = false;

    Sensors::startMagCalibration();
    if (!sensorState.magCalibrationRunning) {
      status.message = "Calibration magnetometre impossible";
      phase = Phase::Error;
      return false;
    }

    phase = Phase::Calibrating;
    status.message = "Calibration magnetometre en cours";
    status.trajectorySegments = 0;
    spinSameDirection(cfg.calibrationPwm, "ROSE_CALIB");

    Logger::log("Calibration fleche Nord lancee : rotation automatique pwm=" +
                String(cfg.calibrationPwm) +
                " sens=" + String(cfg.clockwise ? "horaire" : "antihoraire"));

    return true;
  }

  bool startAlignNorth(const Config& config) {
    applyConfig(config);

    return startComputedNorthAlignment(false, "alignement");
  }

  bool startDrawOnly(const Config& config) {
    applyConfig(config);

    if (sensorState.magCalibrationRunning) {
      status.message = "Calibration magnetometre en cours";
      Logger::log("Erreur fleche Nord dessin : attendre la fin de calibration");
      return false;
    }

    PenInverseFollower::stop();
    Soutenance2::stop();
    stopRoseMotors();
    restoreFollowerConfigIfNeeded();
    drawAfterAlignment = false;

    if (!isInNorthWindow(sensorState.headingMagDeg)) {
      Logger::log("Attention fleche Nord : dessin lance alors que cap=" +
                  String(sensorState.headingMagDeg, 1) +
                  " deg, hors fenetre [358;3]");
    }

    return startDrawing();
  }

  bool startArrow(const Config& config) {
    applyConfig(config);

    return startComputedNorthAlignment(true, "sequence complete");
  }

  void stop() {
    PenInverseFollower::stop();
    stopRoseMotors();
    restoreFollowerConfigIfNeeded();
    drawAfterAlignment = false;
    alignInitialHeadingDeg = 0.0f;
    alignTargetRad = 0.0f;
    alignTargetDeg = 0.0f;
    alignProgressDeg = 0.0f;
    alignRemainingDeg = 0.0f;

    phase = Phase::Idle;
    status = Status();
    updateStatusBase();

    Logger::log("Fleche Nord stop");
  }

  void update(unsigned long now, float dt) {
    (void)dt;

    updateStatusBase();

    if (phase == Phase::Calibrating) {
      if (!sensorState.magCalibrationRunning) {
        stopRoseMotors();
        phase = Phase::Done;
        status.message = "Calibration magnetometre terminee";
        Logger::log("Calibration fleche Nord terminee");
        updateStatusBase();
        return;
      }

      spinSameDirection(cfg.calibrationPwm, "ROSE_CALIB");
      updateStatusBase();
      return;
    }

    if (phase == Phase::Aligning) {
      float progressRad = fabs(odometryState.thetaRad);
      float targetRad = fabs(alignTargetRad);
      float remainingRad = max(0.0f, targetRad - progressRad);
      float remainingDeg = remainingRad * 180.0f / PI;

      if (remainingDeg <= ALIGN_ODOM_TOLERANCE_DEG || progressRad >= targetRad) {
        stopRoseMotors();
        settleStartMs = now;
        phase = Phase::AlignSettling;
        status.message = "Rotation Nord calculee terminee";
        Logger::log("Rotation Nord terminee par odometrie : cible=" +
                    String(alignTargetDeg, 1) +
                    " deg progression=" + String(progressRad * 180.0f / PI, 1) +
                    " deg capInitial=" + String(alignInitialHeadingDeg, 1) +
                    " deg");
        updateStatusBase();
        return;
      }

      if (now - alignStartMs > cfg.alignTimeoutMs) {
        stopRoseMotors();
        phase = Phase::Error;
        status.message = "Arret securite : rotation Nord non terminee";
        Logger::log("Erreur alignement Nord : timeout cible=" +
                    String(alignTargetDeg, 1) +
                    " deg progression=" + String(progressRad * 180.0f / PI, 1) +
                    " deg");
        updateStatusBase();
        return;
      }

      spinSigned(cfg.alignPwm, alignTargetRad, "ROSE_ALIGN");

      updateStatusBase();
      return;
    }

    if (phase == Phase::AlignSettling) {
      stopRoseMotors();

      if (now - settleStartMs < cfg.settleMs) {
        updateStatusBase();
        return;
      }

      if (drawAfterAlignment) {
        startDrawing();
      } else {
        phase = Phase::Done;
        status.message = "Robot oriente vers le Nord par odometrie";
        Logger::log("Rotation Nord terminee, robot au Nord calcule");
      }

      updateStatusBase();
      return;
    }

    if (phase == Phase::Drawing) {
      if (PenInverseFollower::isFinished()) {
        restoreFollowerConfigIfNeeded();
        phase = Phase::Done;
        status.message = "Fleche Nord terminee";
        Logger::log("Fleche Nord terminee");
      } else if (!PenInverseFollower::isRunning()) {
        restoreFollowerConfigIfNeeded();
        phase = Phase::Error;
        status.message = "Follower fleche arrete";
        Logger::log("Erreur fleche Nord : follower arrete");
      }

      updateStatusBase();
    }
  }
}

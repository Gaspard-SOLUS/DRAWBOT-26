#include <Arduino.h>
#include <math.h>

#include "compass_arrow.h"
#include "app_state.h"
#include "logger.h"
#include "moteurs.h"
#include "odometry.h"
#include "pen_inverse_follower.h"
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
  bool followerConfigSaved = false;
  PenInverseFollower::Config previousFollowerConfig;

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
    TrajectoryGenerator::Point bottomBase {baseX, -halfBase};
    TrajectoryGenerator::Point tip {tipX, 0.0f};
    TrajectoryGenerator::Point topBase {baseX, halfBase};

    // Trait principal puis contour du triangle.
    if (!addMove(trajectory, current, baseCenter)) return false;
    if (!addMove(trajectory, current, bottomBase)) return false;
    if (!addMove(trajectory, current, tip)) return false;
    if (!addMove(trajectory, current, topBase)) return false;
    if (!addMove(trajectory, current, bottomBase)) return false;

    bool goingUp = true;
    float x = baseX;

    while (x <= tipX + 0.001f) {
      float t = (x - baseX) / headHeight;
      t = constrain(t, 0.0f, 1.0f);

      float halfWidth = halfBase * (1.0f - t);
      TrajectoryGenerator::Point bottom {x, -halfWidth};
      TrajectoryGenerator::Point top {x, halfWidth};

      if (goingUp) {
        if (!addMove(trajectory, current, bottom)) return false;
        if (!addMove(trajectory, current, top)) return false;
      } else {
        if (!addMove(trajectory, current, top)) return false;
        if (!addMove(trajectory, current, bottom)) return false;
      }

      goingUp = !goingUp;
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

  bool startDrawing() {
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

    cfg.alignToleranceDeg = constrain(cfg.alignToleranceDeg, 0.5f, 5.0f);
    cfg.slowZoneDeg = constrain(cfg.slowZoneDeg, 2.0f, 45.0f);
    cfg.settleMs = constrain(cfg.settleMs, 50UL, 1500UL);
    cfg.pulsePeriodMs = constrain(cfg.pulsePeriodMs, 80UL, 1000UL);
    cfg.pulseOnMs = constrain(cfg.pulseOnMs, 20UL, cfg.pulsePeriodMs);
  }
}

namespace CompassArrow {
  void begin() {
    phase = Phase::Idle;
    cfg = Config();
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
    restoreFollowerConfigIfNeeded();

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

  bool startArrow(const Config& config) {
    applyConfig(config);

    if (!sensorState.magOk) {
      status.message = "LIS3MDL non detecte";
      phase = Phase::Error;
      Logger::log("Erreur fleche Nord : magnetometre non detecte");
      return false;
    }

    if (sensorState.magCalibrationRunning) {
      status.message = "Calibration magnetometre en cours";
      Logger::log("Erreur fleche Nord : attendre la fin de calibration");
      return false;
    }

    PenInverseFollower::stop();
    Soutenance2::stop();
    restoreFollowerConfigIfNeeded();

    phase = Phase::Aligning;
    status.finished = false;
    status.trajectorySegments = 0;
    status.message = "Alignement vers le Nord";

    Logger::log("Fleche Nord : alignement lance pwm=" + String(cfg.alignPwm) +
                " tolerance=" + String(cfg.alignToleranceDeg, 1) +
                " deg sens=" + String(cfg.clockwise ? "horaire" : "antihoraire"));

    return true;
  }

  void stop() {
    PenInverseFollower::stop();
    stopRoseMotors();
    restoreFollowerConfigIfNeeded();

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
      if (status.northErrorDeg <= cfg.alignToleranceDeg) {
        stopRoseMotors();
        settleStartMs = now;
        phase = Phase::AlignSettling;
        status.message = "Nord detecte, stabilisation";
        updateStatusBase();
        return;
      }

      if (status.northErrorDeg <= cfg.slowZoneDeg) {
        unsigned long t = now % cfg.pulsePeriodMs;
        if (t < cfg.pulseOnMs) {
          spinSameDirection(180, "ROSE_ALIGN");
        } else {
          stopRoseMotors();
        }
      } else {
        spinSameDirection(cfg.alignPwm, "ROSE_ALIGN");
      }

      updateStatusBase();
      return;
    }

    if (phase == Phase::AlignSettling) {
      stopRoseMotors();

      if (now - settleStartMs < cfg.settleMs) {
        updateStatusBase();
        return;
      }

      if (status.inNorthWindow) {
        startDrawing();
      } else {
        phase = Phase::Aligning;
        status.message = "Correction finale Nord";
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

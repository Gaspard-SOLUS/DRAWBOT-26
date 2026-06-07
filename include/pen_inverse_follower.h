#pragma once
#include <Arduino.h>
#include "trajectory_generator.h"
#include "robot.h"

namespace PenInverseFollower {
  struct Point {
    float x;
    float y;
  };

  struct Segment {
    Point a;
    Point b;
  };

  struct Config {
    float wheelBaseCm = RobotParams::WHEEL_BASE_CM;
    float penOffsetCm = RobotParams::PEN_OFFSET_CM;

    // Correction globale des distances demandées.
    // Exemple : demandé 20 cm, réel 16 cm => distanceScale = 20 / 16 = 1.25.
    float distanceScale = 0.93f;

    // Suivi de trajectoire du stylo.
    float penSpeedCms = 13.0f;
    float lineGain = 0.45f;
    float targetGain = 0.8f;
    float lookaheadCm = 0.35f;

    // Saturations physiques.
    float penSpeedMaxCms = 18.0f;
    float wheelSpeedMaxCms = 18.0f;

    // Compatibilité avec la page escalier : limitation de rotation et de correction.
    float maxOmegaRadS = 4.0f;
    float maxNormalCorrectionCms = 2.5f;

    // Le centre du robot ne doit pas reculer pendant l'escalier.
    bool allowReverse = false;
    float minForwardSpeedCms = 0.0f;

    // PID sur l'erreur latérale du stylo.
    float kp = 0.02f;
    float ki = 0.00f;
    float kd = 0.015f;
    float integralLimit = 5.0f;

    // Conversion vitesse roue -> PWM.
    float coefLeftCmsPerPwm = 0.0709f;
    float coefRightCmsPerPwm = 0.0686f;

    int minPwm = 180;

    // En dessous de cette vitesse roue théorique, on considère que la roue ne doit pas bouger.
    float minCommandSpeedCms = 0.0f;

    // Rampe PWM : variation maximale de PWM à chaque update.
    int pwmSlewStep = 20;

    // Ancienne option conservée pour compatibilité API.
    // Les commandes moteur sont maintenant continues : 0 ou au moins minPwm.
    bool pwmDither = false;

    float straightEncoderKp = 22.0f;
    float straightStopCompensationCm = 0.0f;
    float straightStopDecelCms2 = 0.0f;
    int straightBrakeMs = 80;

    float segmentToleranceCm = 0.08f;

    // Champs conservés pour compatibilité avec l'API/page escalier.
    // Le mode cercle réel utilise surtout startTrajectory(...).
    bool cornerMode = true;
    float cornerApproachCm = 1.2f;
    float cornerSpeedCms = 8.0f;
    float cornerOmegaRadS = 4.0f;
    float cornerExitAngleDeg = 5.0f;
    // 0 = suivi inverse pur du stylo. A n'augmenter qu'en dernier recours.
    float cornerInnerBoost = 0.0f;
    int cornerTurnMinPwm = 180;
    float cornerMaxDurationS = 1.80f;
  };

  struct Status {
    bool running = false;
    bool finished = false;

    int currentSegment = 0;
    int segmentCount = 0;

    String phaseName = "IDLE";
    bool cornerActive = false;
    float remainingToCornerCm = 0.0f;
    float cornerAngleErrorDeg = 0.0f;

    float penX = 0.0f;
    float penY = 0.0f;

    float targetX = 0.0f;
    float targetY = 0.0f;

    float lateralErrorCm = 0.0f;
    float maxLateralErrorCm = 0.0f;

    float progressCm = 0.0f;
    float segmentLengthCm = 0.0f;

    float vPenX = 0.0f;
    float vPenY = 0.0f;

    float vCenterCms = 0.0f;
    float omegaRadS = 0.0f;

    float vLeftCms = 0.0f;
    float vRightCms = 0.0f;

    int pwmLeft = 0;
    int pwmRight = 0;

    int targetPwmLeft = 0;
    int targetPwmRight = 0;

    float encoderBalanceErrorCm = 0.0f;
    float straightStopDistanceCm = 0.0f;

    bool reverseLimited = false;
    bool omegaLimited = false;
    bool normalCorrectionLimited = false;
  };

  void begin();

  void setConfig(const Config& cfg);
  Config getConfig();
  Status getStatus();

  bool loadConfig();
  bool saveConfig();
  void resetConfigToDefaults();

  void startLine(float distanceCm);
  void startOneAngle(float d1Cm, float angleDeg, float d2Cm);
  void startStair(float d1Cm, float angleLeftDeg, float d2Cm, float angleRightDeg, float d3Cm);

  // Nouveau : permet de suivre un cercle, une rosace ou n'importe quelle liste de segments.
  bool startTrajectory(const TrajectoryGenerator::Trajectory& trajectory);

  void stop();
  void update(unsigned long now, float dt);

  bool isRunning();
  bool isFinished();
}

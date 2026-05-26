#pragma once
#include <Arduino.h>

namespace S2Escalier {

  enum class State {
    IDLE,
    LINE_RUNNING,
    PIVOT_RUNNING,
    CORNER_PAUSE,
    FINISHED,
    ERROR
  };

  struct Config {
    // Trajectoire demandee par le cahier des charges
    float dist1Cm = 20.0f;
    float dist2Cm = 10.0f;
    float dist3Cm = 40.0f;

    // Geometrie reelle du robot
    float wheelBaseCm = 8.3f;
    float penOffsetCm = 13.0f;

    // Lignes droites : avance avec maintien de cap, pas PID lateral agressif
    float lineSpeedCms = 2.4f;
    float slowZoneCm = 3.0f;
    float endToleranceCm = 0.35f;
    unsigned long cornerPauseMs = 150;

    // Conversion vitesse roue -> PWM
    int minPwm = 145;
    int maxPwm = 205;
    float maxWheelSpeedCms = 18.0f;
    int pwmRampStep = 20;

    // Correcteur de cap pendant les lignes droites
    // erreur en rad -> omega en rad/s
    float kpHeading = 1.4f;
    float kiHeading = 0.0f;
    float kdHeading = 0.08f;
    float maxOmegaRadS = 0.55f;
    float headingIntegralLimit = 0.4f;

    // Pivot entre deux lignes : roue lente / roue rapide, meme sens.
    // Cela reduit l'arc du stylo par rapport a une rotation sur place.
    int pivotSlowPwm = 125;
    float pivotRatio = 1.94f;
    int pivotMinPwm = 105;
    int pivotMaxPwm = 245;
    float pivotAngleToleranceDeg = 3.0f;
    float pivotSlowdownDeg = 18.0f;

    // Source orientation : 0 = odometrie encodeurs, 1 = gyro, 2 = magnetometre
    int headingSource = 0;
  };

  struct RuntimeStatus {
    State state = State::IDLE;
    int phaseIndex = 0;      // 0 line1, 1 pivotG, 2 line2, 3 pivotD, 4 line3
    int segmentIndex = 0;    // 0,1,2 pour les lignes

    float penX = 0.0f;
    float penY = 0.0f;
    float progressCm = 0.0f;
    float remainingCm = 0.0f;
    float currentSegmentLengthCm = 0.0f;

    float headingErrorDeg = 0.0f;
    float lateralErrorCm = 0.0f;          // garde le nom pour l'interface, ici debug ligne
    float thetaDeg = 0.0f;
    float targetThetaDeg = 0.0f;

    float activeSpeedCms = 0.0f;
    float activeOmegaRadS = 0.0f;
    float activeLatCorrectionCms = 0.0f;  // compatibilite interface/debug

    int pwmLeft = 0;
    int pwmRight = 0;
    bool entryZoneActive = false;
  };

  void begin();
  void update(unsigned long now, float dt);

  void start();
  void stop();
  void reset();

  Config getConfig();
  void setConfig(const Config& newCfg);
  void loadConfig();
  void saveConfig();

  RuntimeStatus getStatus();
  String stateName();
  String configJson();
  String statusJson();
}

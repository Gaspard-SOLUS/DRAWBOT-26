#pragma once
#include <Arduino.h>

namespace S2Escalier {

  enum class State {
    IDLE,
    RUNNING,
    FINISHED,
    ERROR
  };

  struct PidProfile {
    float penSpeedCms = 3.0f;
    float kpLat = 0.65f;
    float kiLat = 0.0f;
    float kdLat = 0.22f;
    float maxLatCorrectionCms = 5.0f;
  };

  struct Config {
    // Geometrie demandee par le cahier des charges
    float dist1Cm = 20.0f;
    float dist2Cm = 10.0f;
    float dist3Cm = 40.0f;

    // Geometrie reelle du robot
    float wheelBaseCm = 8.3f;
    float penOffsetCm = 13.0f;

    // Fin de segment
    float slowZoneCm = 3.0f;
    float endToleranceCm = 0.35f;

    // Conversion vitesse roue -> PWM
    int minPwm = 145;
    int maxPwm = 210;
    float maxWheelSpeedCms = 18.0f;

    // PID A : segment 1 + premiere moitie du segment 2
    PidProfile pidA;

    // PID B : deuxieme moitie du segment 2 + segment 3
    PidProfile pidB;

    // Limite integrale commune
    float integralLimit = 8.0f;

    // 0 = odometrie encodeurs, 1 = gyro, 2 = magnetometre
    int headingSource = 0;
  };

  struct RuntimeStatus {
    State state = State::IDLE;
    int segmentIndex = 0;
    int activePid = 0; // 1 = PID A, 2 = PID B
    float penX = 0.0f;
    float penY = 0.0f;
    float progressCm = 0.0f;
    float currentSegmentLengthCm = 0.0f;
    float lateralErrorCm = 0.0f;
    float thetaDeg = 0.0f;
    float activeKp = 0.0f;
    float activeKi = 0.0f;
    float activeKd = 0.0f;
    float activeSpeedCms = 0.0f;
    float activeMaxCorrectionCms = 0.0f;
    int pwmLeft = 0;
    int pwmRight = 0;
  };

  void begin();
  void update(unsigned long now, float dt);

  void start();
  void stop();
  void reset();

  Config getConfig();
  void setConfig(const Config& cfg);
  void loadConfig();
  void saveConfig();

  RuntimeStatus getStatus();
  String stateName();
  String configJson();
  String statusJson();
}

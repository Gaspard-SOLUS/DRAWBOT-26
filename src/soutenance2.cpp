#include <math.h>

#include "soutenance2.h"
#include "app_state.h"
#include "logger.h"
#include "moteurs.h"
#include "odometry.h"
#include "pen_inverse_follower.h"

namespace {
  bool spinCircleRunning = false;
  bool spinClockwise = true;
  int spinPwmLeft = 0;
  int spinPwmRight = 0;
  float spinTargetRad = 2.0f * PI;
  float spinStopAdvanceRad = 0.0f;
  float spinStartThetaRad = 0.0f;
  float spinRadiusCm = 8.0f;

  int clampCirclePwm(int pwm) {
    int absPwm = abs(pwm);
    if (absPwm == 0) return 0;
    if (absPwm < 180) absPwm = 180;
    if (absPwm > 255) absPwm = 255;
    return absPwm;
  }

  void setTrackedMotors(int left, int right) {
    motorState.pwmLeft = left;
    motorState.pwmRight = right;
    motorState.mode = "CIRCLE_SPIN";
    setMotors(left, right);
  }
}

namespace Soutenance2 {
  void begin() {
    spinCircleRunning = false;
    Logger::log("Module soutenance 2 pret");
  }

  void update(unsigned long now, float dt) {
    (void)now;
    (void)dt;

    if (!spinCircleRunning) {
      return;
    }

    float travelled = fabs(odometryState.thetaRad - spinStartThetaRad);
    float stopAt = spinTargetRad - spinStopAdvanceRad;
    if (stopAt < 0.1f) stopAt = spinTargetRad;

    if (travelled >= stopAt) {
      stop();
      Logger::log("Cercle rotation termine : rayon demande=" +
                  String(spinRadiusCm, 1) +
                  " cm angle=" +
                  String(Odometry::radToDeg(travelled), 1) +
                  " deg");
      return;
    }

    setTrackedMotors(spinPwmLeft, spinPwmRight);
  }

  bool startSpinCircle(float radiusCm, bool clockwise, int pwm, float stopAdvanceDeg) {
    int commandPwm = clampCirclePwm(pwm);

    if (commandPwm == 0) {
      Logger::log("Erreur cercle rotation : PWM nul");
      return false;
    }

    PenInverseFollower::stop();
    Odometry::resetPose(0.0f, 0.0f, 0.0f);

    spinCircleRunning = true;
    spinClockwise = clockwise;
    spinRadiusCm = radiusCm;
    spinStartThetaRad = odometryState.thetaRad;
    spinTargetRad = 2.0f * PI;
    spinStopAdvanceRad = fabs(stopAdvanceDeg) * PI / 180.0f;

    if (spinClockwise) {
      spinPwmLeft = commandPwm;
      spinPwmRight = -commandPwm;
    } else {
      spinPwmLeft = -commandPwm;
      spinPwmRight = commandPwm;
    }

    setTrackedMotors(spinPwmLeft, spinPwmRight);

    Logger::log("Cercle rotation lance : rayon demande=" +
                String(spinRadiusCm, 1) +
                " cm pwm=" + String(commandPwm) +
                " sens=" + String(spinClockwise ? "horaire" : "antihoraire") +
                " anticipation=" + String(stopAdvanceDeg, 1) + " deg");

    return true;
  }

  void stop() {
    spinCircleRunning = false;
    spinPwmLeft = 0;
    spinPwmRight = 0;

    if (motorState.mode == "CIRCLE_SPIN") {
      motorState.pwmLeft = 0;
      motorState.pwmRight = 0;
      motorState.mode = "IDLE";
      stopMotors();
    }
  }
}

#include <math.h>

#include "soutenance2.h"
#include "app_state.h"
#include "logger.h"
#include "moteurs.h"
#include "odometry.h"
#include "encodeurs.h"
#include "pen_inverse_follower.h"

namespace {
  enum class CircleMode {
    Idle,
    Spin,
    WheelDistance
  };

  CircleMode circleMode = CircleMode::Idle;
  bool spinClockwise = true;
  int spinPwmLeft = 0;
  int spinPwmRight = 0;
  float spinTargetRad = 2.0f * PI;
  float spinStopAdvanceRad = 0.0f;
  float spinStartThetaRad = 0.0f;
  float spinRadiusCm = 8.0f;

  bool wheelCircleClockwise = true;
  int wheelCirclePwm = 220;
  float wheelCircleRadiusCm = 5.0f;
  float wheelCircleRequestedRadiusCm = 5.0f;
  float wheelCircleRadiusScale = 2.0f;
  float wheelCirclePenOffsetCm = 13.0f;
  float wheelCircleBaseCm = 8.3f;
  float wheelCircleTargetLeftCm = 0.0f;
  float wheelCircleTargetRightCm = 0.0f;
  float wheelCircleAbsLeftCm = 0.0f;
  float wheelCircleAbsRightCm = 0.0f;
  float wheelCircleLastLeftCm = 0.0f;
  float wheelCircleLastRightCm = 0.0f;
  float wheelCircleShapeLeadRad = 35.0f * PI / 180.0f;
  int wheelCircleLastPwmLeft = 0;
  int wheelCircleLastPwmRight = 0;

  int clampCirclePwm(int pwm) {
    int absPwm = abs(pwm);
    if (absPwm == 0) return 0;
    if (absPwm < 180) absPwm = 180;
    if (absPwm > 255) absPwm = 255;
    return absPwm;
  }

  int slewCirclePwm(int previous, int target) {
    const int step = 18;

    if (target == 0) {
      return 0;
    }

    if (previous == 0) {
      return target;
    }

    if ((previous > 0 && target < 0) || (previous < 0 && target > 0)) {
      return target;
    }

    if (target > previous + step) return previous + step;
    if (target < previous - step) return previous - step;
    return target;
  }

  void setTrackedMotors(int left, int right) {
    motorState.pwmLeft = left;
    motorState.pwmRight = right;
    motorState.mode = (circleMode == CircleMode::WheelDistance) ? "CIRCLE_WHEEL" : "CIRCLE_SPIN";
    setMotors(left, right);
  }

  float initialCircleHeading(bool clockwise) {
    return clockwise ? PI : 0.0f;
  }

  void circleWheelStateAt(
    float radiusCm,
    float penOffsetCm,
    float wheelBaseCm,
    bool clockwise,
    float progressRad,
    float& theta,
    float& alpha,
    float& leftShape,
    float& rightShape
  ) {
    float dir = clockwise ? -1.0f : 1.0f;
    theta = initialCircleHeading(clockwise);
    alpha = -0.5f * PI;

    int steps = (int)ceil(progressRad / 0.025f);
    if (steps < 1) steps = 1;
    if (steps > 360) steps = 360;

    float ds = progressRad / (float)steps;

    for (int i = 0; i < steps; i++) {
      float dAlpha = dir * ds;
      float midAlpha = alpha + 0.5f * dAlpha;
      float dThetaDAlpha = (radiusCm / penOffsetCm) * cos(midAlpha - theta);
      theta += dThetaDAlpha * dAlpha;
      alpha += dAlpha;
    }

    float dThetaDAlpha = (radiusCm / penOffsetCm) * cos(alpha - theta);
    float centerPerAlpha = radiusCm * sin(theta - alpha);
    float rotPerAlpha = dThetaDAlpha * wheelBaseCm * 0.5f;

    leftShape = (centerPerAlpha - rotPerAlpha) * dir;
    rightShape = (centerPerAlpha + rotPerAlpha) * dir;
  }

  void circleWheelShapeAt(float radiusCm, float penOffsetCm, float wheelBaseCm, bool clockwise, float progressRad, float& leftShape, float& rightShape) {
    float theta = 0.0f;
    float alpha = 0.0f;
    circleWheelStateAt(radiusCm, penOffsetCm, wheelBaseCm, clockwise, progressRad, theta, alpha, leftShape, rightShape);
  }

  void computeWheelCircleTargets(float radiusCm, float penOffsetCm, float wheelBaseCm, bool clockwise, float& leftTargetCm, float& rightTargetCm) {
    leftTargetCm = 0.0f;
    rightTargetCm = 0.0f;

    const int steps = 720;
    float previousLeft = 0.0f;
    float previousRight = 0.0f;

    for (int i = 0; i < steps; i++) {
      float s = 2.0f * PI * ((float)i / (float)steps);
      float leftShape = 0.0f;
      float rightShape = 0.0f;
      circleWheelShapeAt(radiusCm, penOffsetCm, wheelBaseCm, clockwise, s, leftShape, rightShape);

      float ds = 2.0f * PI / (float)steps;
      leftTargetCm += fabs(leftShape * ds);
      rightTargetCm += fabs(rightShape * ds);

      previousLeft = leftShape;
      previousRight = rightShape;
    }

    (void)previousLeft;
    (void)previousRight;
  }

  int shapeToPwm(float shape, float maxShape, int maxPwm) {
    if (maxShape < 0.0001f || fabs(shape) < 0.0001f) {
      return 0;
    }

    int pwm = (int)round(((float)maxPwm) * fabs(shape) / maxShape);
    if (pwm > 0 && pwm < 180) pwm = 180;
    if (pwm > 255) pwm = 255;

    return (shape >= 0.0f) ? pwm : -pwm;
  }

  void commandWheelDistanceCircle() {
    float leftNow = getLeftDistanceCm();
    float rightNow = getRightDistanceCm();

    wheelCircleAbsLeftCm += fabs(leftNow - wheelCircleLastLeftCm);
    wheelCircleAbsRightCm += fabs(rightNow - wheelCircleLastRightCm);
    wheelCircleLastLeftCm = leftNow;
    wheelCircleLastRightCm = rightNow;

    if (wheelCircleTargetLeftCm <= 0.001f || wheelCircleTargetRightCm <= 0.001f) {
      return;
    }

    float progressLeft = wheelCircleAbsLeftCm / wheelCircleTargetLeftCm;
    float progressRight = wheelCircleAbsRightCm / wheelCircleTargetRightCm;

    if ((progressLeft >= 1.0f && progressRight >= 1.0f) ||
        max(progressLeft, progressRight) >= 1.20f) {
      float leftDone = wheelCircleAbsLeftCm;
      float rightDone = wheelCircleAbsRightCm;
      Soutenance2::stop();
      Logger::log("Cercle 5cm distance roues termine : L=" +
                  String(leftDone, 2) + "/" + String(wheelCircleTargetLeftCm, 2) +
                  " cm R=" + String(rightDone, 2) + "/" +
                  String(wheelCircleTargetRightCm, 2) + " cm");
      return;
    }

    float progress = (progressLeft + progressRight) * 0.5f;
    progress = constrain(progress, 0.0f, 0.999f);

    float leftShape = 0.0f;
    float rightShape = 0.0f;
    circleWheelShapeAt(
      wheelCircleRadiusCm,
      wheelCirclePenOffsetCm,
      wheelCircleBaseCm,
      wheelCircleClockwise,
      progress * 2.0f * PI + wheelCircleShapeLeadRad,
      leftShape,
      rightShape
    );

    float maxShape = max(fabs(leftShape), fabs(rightShape));
    int leftPwmTarget = shapeToPwm(leftShape, maxShape, wheelCirclePwm);
    int rightPwmTarget = shapeToPwm(rightShape, maxShape, wheelCirclePwm);

    float balance = progressLeft - progressRight;
    if (balance > 0.04f && rightPwmTarget != 0) {
      rightPwmTarget = (rightPwmTarget > 0) ? min(255, rightPwmTarget + 25)
                                           : max(-255, rightPwmTarget - 25);
    } else if (balance < -0.04f && leftPwmTarget != 0) {
      leftPwmTarget = (leftPwmTarget > 0) ? min(255, leftPwmTarget + 25)
                                         : max(-255, leftPwmTarget - 25);
    }

    int leftPwm = slewCirclePwm(wheelCircleLastPwmLeft, leftPwmTarget);
    int rightPwm = slewCirclePwm(wheelCircleLastPwmRight, rightPwmTarget);

    wheelCircleLastPwmLeft = leftPwm;
    wheelCircleLastPwmRight = rightPwm;

    setTrackedMotors(leftPwm, rightPwm);
  }
}

namespace Soutenance2 {
  void begin() {
    circleMode = CircleMode::Idle;
    Logger::log("Module soutenance 2 pret");
  }

  void update(unsigned long now, float dt) {
    (void)now;
    (void)dt;

    if (circleMode == CircleMode::WheelDistance) {
      commandWheelDistanceCircle();
      return;
    }

    if (circleMode != CircleMode::Spin) {
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

    circleMode = CircleMode::Spin;
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

  bool startWheelDistanceCircle(float radiusCm, bool clockwise, int pwm, float penOffsetCm, float wheelBaseCm, float radiusScale, float shapeLeadDeg) {
    int commandPwm = clampCirclePwm(pwm);

    if (commandPwm == 0 || radiusCm <= 0.0f || penOffsetCm <= 0.1f || wheelBaseCm <= 0.1f) {
      Logger::log("Erreur cercle distance roues : parametres invalides");
      return false;
    }

    PenInverseFollower::stop();

    if (radiusScale <= 0.0f) radiusScale = 1.0f;

    wheelCircleRequestedRadiusCm = radiusCm;
    wheelCircleRadiusScale = radiusScale;
    wheelCircleRadiusCm = radiusCm * wheelCircleRadiusScale;
    wheelCircleShapeLeadRad = constrain(shapeLeadDeg, 0.0f, 90.0f) * PI / 180.0f;
    wheelCircleClockwise = clockwise;
    wheelCirclePwm = commandPwm;
    wheelCirclePenOffsetCm = penOffsetCm;
    wheelCircleBaseCm = wheelBaseCm;

    computeWheelCircleTargets(
      wheelCircleRadiusCm,
      wheelCirclePenOffsetCm,
      wheelCircleBaseCm,
      wheelCircleClockwise,
      wheelCircleTargetLeftCm,
      wheelCircleTargetRightCm
    );

    float theta0 = 0.0f;
    float alpha0 = 0.0f;
    float leftShape0 = 0.0f;
    float rightShape0 = 0.0f;
    circleWheelStateAt(
      wheelCircleRadiusCm,
      wheelCirclePenOffsetCm,
      wheelCircleBaseCm,
      wheelCircleClockwise,
      wheelCircleShapeLeadRad,
      theta0,
      alpha0,
      leftShape0,
      rightShape0
    );

    float penX0 = wheelCircleRadiusCm * cos(alpha0);
    float penY0 = wheelCircleRadiusCm + wheelCircleRadiusCm * sin(alpha0);
    float baseX = penX0 - wheelCirclePenOffsetCm * cos(theta0);
    float baseY = penY0 - wheelCirclePenOffsetCm * sin(theta0);
    Odometry::resetPose(baseX, baseY, theta0);

    wheelCircleAbsLeftCm = 0.0f;
    wheelCircleAbsRightCm = 0.0f;
    wheelCircleLastLeftCm = getLeftDistanceCm();
    wheelCircleLastRightCm = getRightDistanceCm();
    wheelCircleLastPwmLeft = 0;
    wheelCircleLastPwmRight = 0;
    circleMode = CircleMode::WheelDistance;

    Logger::log("Cercle distance roues lance : rayon demande=" +
                String(wheelCircleRequestedRadiusCm, 1) +
                " cm rayon effectif=" + String(wheelCircleRadiusCm, 1) +
                " cm echelle=" + String(wheelCircleRadiusScale, 2) +
                " phase=" + String(shapeLeadDeg, 1) +
                " deg" +
                " theta0=" + String(Odometry::normalizeAngleDeg(Odometry::radToDeg(theta0)), 1) +
                " deg" +
                " cm offsetStylo=" + String(wheelCirclePenOffsetCm, 1) +
                " cm cibleL=" + String(wheelCircleTargetLeftCm, 2) +
                " cm cibleR=" + String(wheelCircleTargetRightCm, 2) +
                " cm pwmMax=" + String(wheelCirclePwm));

    commandWheelDistanceCircle();
    return true;
  }

  void stop() {
    circleMode = CircleMode::Idle;
    spinPwmLeft = 0;
    spinPwmRight = 0;
    wheelCircleLastPwmLeft = 0;
    wheelCircleLastPwmRight = 0;

    if (motorState.mode == "CIRCLE_SPIN" || motorState.mode == "CIRCLE_WHEEL") {
      motorState.pwmLeft = 0;
      motorState.pwmRight = 0;
      motorState.mode = "IDLE";
      stopMotors();
    }
  }
}

#include "motion_controller.h"
#include "moteurs.h"
#include "encodeurs.h"
#include "app_state.h"
#include "logger.h"
#include <math.h>

namespace MotionController {

  // Paramètres par défaut
  float wheelBaseCm     = 8.3f;
  float turnWheelBaseCm = 8.8f;
  float penOffsetCm     = 13.0f;

  int pwmMin = 115;
  int pwmMax = 230;

  float kSync = 8.0f;
  float slowdownFraction = 0.25f;

  // État interne
  static State state = IDLE;

  static float refDistL = 0.0f;
  static float refDistR = 0.0f;

  static float targetDistL = 0.0f;
  static float targetDistR = 0.0f;

  static int signL = 1;
  static int signR = 1;

  static int cruisePwm = 180;

  // ============================================================
  // Helpers
  // ============================================================
  static int clampPwmMagnitude(int p) {
    if (p < pwmMin) p = pwmMin;
    if (p > pwmMax) p = pwmMax;
    return p;
  }

  static int applySign(int magnitude, int sign) {
    return sign >= 0 ? magnitude : -magnitude;
  }

  static float absDeltaL() {
    return fabs(getLeftDistanceCm() - refDistL);
  }

  static float absDeltaR() {
    return fabs(getRightDistanceCm() - refDistR);
  }

  static void captureRef() {
    refDistL = getLeftDistanceCm();
    refDistR = getRightDistanceCm();
  }

  static void writeMotors(int pwmL, int pwmR) {
    motorState.pwmLeft  = pwmL;
    motorState.pwmRight = pwmR;
    setMotors(pwmL, pwmR);
  }

  static void stopMotorsAndIdle(const char* reason) {
    stopMotors();
    motorState.pwmLeft = 0;
    motorState.pwmRight = 0;
    motorState.mode = "IDLE";
    state = IDLE;
    Logger::log(String("[MC] Stop: ") + reason);
  }

  // ============================================================
  // Init
  // ============================================================
  void init() {
    state = IDLE;
  }

  // ============================================================
  // Avance droite
  // ============================================================
  void startMoveStraight(float distanceCm, int pwm) {
    if (fabs(distanceCm) < 0.1f) return;
    stop();

    captureRef();
    targetDistL = fabs(distanceCm);
    targetDistR = fabs(distanceCm);
    signL = (distanceCm > 0) ? 1 : -1;
    signR = (distanceCm > 0) ? 1 : -1;
    cruisePwm = clampPwmMagnitude(pwm);

    motorState.mode = "MC_STRAIGHT";
    state = STRAIGHT;
    Logger::log("[MC] Start straight " + String(distanceCm) + " cm");
  }

  // ============================================================
  // Rotation sur place
  // angle positif = anti-horaire (gauche)
  // ============================================================
  void startRotateInPlace(float angleDeg, int pwm) {
    if (fabs(angleDeg) < 0.5f) return;
    stop();

    float angleRad = fabs(angleDeg) * PI / 180.0f;
    float dist = angleRad * (turnWheelBaseCm * 0.5f);

    captureRef();
    targetDistL = dist;
    targetDistR = dist;

    if (angleDeg > 0) { signL = -1; signR = +1; }  // gauche : roue G recule, roue D avance
    else              { signL = +1; signR = -1; }

    cruisePwm = clampPwmMagnitude(pwm);
    motorState.mode = "MC_ROT";
    state = ROT_IN_PLACE;
    Logger::log("[MC] Start rotateInPlace " + String(angleDeg) + " deg, d/wheel="
                + String(dist));
  }

  // ============================================================
  // Rotation autour du stylo
  //
  // Géométrie : stylo à (penOffsetCm, 0) en repère robot.
  // Centre de rotation = stylo. Les deux roues sont à distance
  // R = sqrt(penOffset² + (wb/2)²) du stylo (rayons égaux par symétrie).
  // Les deux roues parcourent la même distance, en sens opposé.
  // ============================================================
  void startRotateAroundPen(float angleDeg, int pwm) {
    if (fabs(angleDeg) < 0.5f) return;
    stop();

    float angleRad = fabs(angleDeg) * PI / 180.0f;
    float halfWB = turnWheelBaseCm * 0.5f;
    float radius = sqrtf(penOffsetCm * penOffsetCm + halfWB * halfWB);
    float dist = angleRad * radius;

    captureRef();
    targetDistL = dist;
    targetDistR = dist;

    // Anti-horaire (gauche) vue de dessus, centre devant :
    //   roue gauche RECULE, roue droite AVANCE
    if (angleDeg > 0) { signL = -1; signR = +1; }
    else              { signL = +1; signR = -1; }

    cruisePwm = clampPwmMagnitude(pwm);
    motorState.mode = "MC_PEN";
    state = ROT_AROUND_PEN;
    Logger::log("[MC] Start rotateAroundPen " + String(angleDeg)
                + " deg, R=" + String(radius) + " d/wheel=" + String(dist));
  }

  // ============================================================
  // Arc au stylo
  //
  // Le stylo décrit un cercle de rayon radiusCm.
  // Centre du cercle = à gauche (ou droite) du stylo à distance radiusCm.
  //
  // Roues à distances du centre :
  //   rInner = sqrt(penOffset² + (radius - halfWB)²)
  //   rOuter = sqrt(penOffset² + (radius + halfWB)²)
  //
  // Les deux roues avancent en sens normal.
  // ============================================================
  void startArcAtPen(float radiusCm, float angleDeg, bool leftTurn, int pwm) {
    if (fabs(angleDeg) < 0.5f || radiusCm < 1.0f) return;
    stop();

    float angleRad = fabs(angleDeg) * PI / 180.0f;
    float halfWB = turnWheelBaseCm * 0.5f;

    float rInner = sqrtf(penOffsetCm * penOffsetCm + (radiusCm - halfWB) * (radiusCm - halfWB));
    float rOuter = sqrtf(penOffsetCm * penOffsetCm + (radiusCm + halfWB) * (radiusCm + halfWB));

    float distInner = angleRad * rInner;
    float distOuter = angleRad * rOuter;

    captureRef();

    if (leftTurn) {
      targetDistL = distInner;
      targetDistR = distOuter;
    } else {
      targetDistL = distOuter;
      targetDistR = distInner;
    }

    signL = +1;
    signR = +1;

    cruisePwm = clampPwmMagnitude(pwm);
    motorState.mode = "MC_ARC";
    state = ARC;
    Logger::log("[MC] Start arc r=" + String(radiusCm) + " ang=" + String(angleDeg)
                + (leftTurn ? " LEFT" : " RIGHT")
                + " dL=" + String(targetDistL) + " dR=" + String(targetDistR));
  }

  // ============================================================
  // UPDATE — appelé en boucle
  // ============================================================
  void update() {
    if (state == IDLE) return;

    float dL = absDeltaL();
    float dR = absDeltaR();

    bool doneL = (dL >= targetDistL);
    bool doneR = (dR >= targetDistR);

    if (doneL && doneR) {
      stopMotorsAndIdle("target reached");
      return;
    }

    int pwmLMag = cruisePwm;
    int pwmRMag = cruisePwm;

    if (state == STRAIGHT || state == ROT_IN_PLACE || state == ROT_AROUND_PEN) {
      // Asservissement gauche/droite (cibles égales)
      float err = dL - dR;
      int correction = (int)(kSync * err);

      pwmLMag = cruisePwm - correction;
      pwmRMag = cruisePwm + correction;

      // Ralentissement final
      float remaining = ((targetDistL - dL) + (targetDistR - dR)) * 0.5f;
      float slowZone  = targetDistL * slowdownFraction;
      if (remaining < slowZone && slowZone > 0.01f) {
        float ratio = remaining / slowZone;
        if (ratio < 0.35f) ratio = 0.35f;
        pwmLMag = (int)(pwmLMag * ratio);
        pwmRMag = (int)(pwmRMag * ratio);
      }
    }
    else if (state == ARC) {
      // Cibles différentes : on régule sur la PROGRESSION (pas la distance)
      float progL = (targetDistL > 0.01f) ? dL / targetDistL : 1.0f;
      float progR = (targetDistR > 0.01f) ? dR / targetDistR : 1.0f;

      // Ratio des cibles : la roue extérieure tourne plus vite
      // On donne cruisePwm à l'extérieure, et un pwm réduit à l'intérieure
      if (targetDistL >= targetDistR) {
        pwmLMag = cruisePwm;
        pwmRMag = (int)(cruisePwm * (targetDistR / targetDistL));
      } else {
        pwmRMag = cruisePwm;
        pwmLMag = (int)(cruisePwm * (targetDistL / targetDistR));
      }

      // Correction si une roue prend de l'avance sur sa progression cible
      float errProg = progL - progR;
      int corr = (int)(kSync * 40.0f * errProg);
      pwmLMag -= corr;
      pwmRMag += corr;

      // Ralentissement final basé sur la progression moyenne
      float progMean = (progL + progR) * 0.5f;
      if (progMean > (1.0f - slowdownFraction)) {
        float remaining = 1.0f - progMean;
        float ratioSlow = remaining / slowdownFraction;
        if (ratioSlow < 0.35f) ratioSlow = 0.35f;
        pwmLMag = (int)(pwmLMag * ratioSlow);
        pwmRMag = (int)(pwmRMag * ratioSlow);
      }
    }

    // Si une roue a fini, on la stoppe
    if (doneL) pwmLMag = 0;
    if (doneR) pwmRMag = 0;

    // Clamp (en magnitude)
    if (pwmLMag != 0) pwmLMag = clampPwmMagnitude(abs(pwmLMag));
    if (pwmRMag != 0) pwmRMag = clampPwmMagnitude(abs(pwmRMag));

    writeMotors(applySign(pwmLMag, signL), applySign(pwmRMag, signR));
  }

  // ============================================================
  // Contrôle
  // ============================================================
  void stop() {
    if (state != IDLE) {
      stopMotors();
      motorState.pwmLeft = 0;
      motorState.pwmRight = 0;
      motorState.mode = "IDLE";
      state = IDLE;
    }
  }

  bool isBusy() { return state != IDLE; }
  State getState() { return state; }

  const char* getStateName() {
    switch (state) {
      case IDLE:           return "IDLE";
      case STRAIGHT:       return "STRAIGHT";
      case ROT_IN_PLACE:   return "ROT_IN_PLACE";
      case ROT_AROUND_PEN: return "ROT_AROUND_PEN";
      case ARC:            return "ARC";
    }
    return "?";
  }

  float getProgress() {
    if (state == IDLE) return 0.0f;
    if (targetDistL < 0.01f && targetDistR < 0.01f) return 1.0f;
    float pL = (targetDistL > 0.01f) ? absDeltaL() / targetDistL : 1.0f;
    float pR = (targetDistR > 0.01f) ? absDeltaR() / targetDistR : 1.0f;
    float p = (pL + pR) * 0.5f;
    if (p > 1.0f) p = 1.0f;
    if (p < 0.0f) p = 0.0f;
    return p;
  }

  float getCurrentDistanceCm() {
    return (absDeltaL() + absDeltaR()) * 0.5f;
  }

  float getTargetDistanceCm() {
    return (targetDistL + targetDistR) * 0.5f;
  }
}

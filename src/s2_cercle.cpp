#include <Arduino.h>
#include <math.h>
#include <Preferences.h>

#include "s2_cercle.h"
#include "moteurs.h"
#include "encodeurs.h"
#include "odometry.h"
#include "app_state.h"
#include "logger.h"

namespace S2Cercle {

static Config cfg;
static RuntimeStatus st;
static Preferences prefs;

static float startLeftDistCm = 0.0f;
static float startRightDistCm = 0.0f;

static float iLeft = 0.0f;
static float iRight = 0.0f;
static float prevErrLeft = 0.0f;
static float prevErrRight = 0.0f;
static bool firstPid = true;

// Nouveau controle V8 : PID uniquement sur la roue exterieure,
// roue interieure verrouillee par le ratio geometrique + correction douce de progression.
// Cela evite le probleme de la V7 : avec minPwm=170, une petite vitesse interieure
// etait forcee a 170 en continu, donc les deux roues avancaient presque pareil => ligne droite.

static int cmdLeft = 0;
static int cmdRight = 0;

static float filteredLeftSpeed = 0.0f;
static float filteredRightSpeed = 0.0f;
static bool firstSpeedFilter = true;

static bool geometryOk = false;

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int signInt(int v) {
  if (v > 0) return 1;
  if (v < 0) return -1;
  return 0;
}

static int rampMotor(int current, int target) {
  int step = constrain(cfg.pwmRampStep, 1, 255);

  // Si on doit inverser le sens, on repasse d'abord par zero.
  if (signInt(current) != 0 && signInt(target) != 0 && signInt(current) != signInt(target)) {
    if (abs(current) <= step) return 0;
    return current - signInt(current) * step;
  }

  int delta = target - current;
  if (delta > step) delta = step;
  if (delta < -step) delta = -step;
  return current + delta;
}

static void applyPwmSmooth(int targetLeft, int targetRight) {
  targetLeft = constrain(targetLeft, -cfg.maxPwm, cfg.maxPwm);
  targetRight = constrain(targetRight, -cfg.maxPwm, cfg.maxPwm);

  cmdLeft = rampMotor(cmdLeft, targetLeft);
  cmdRight = rampMotor(cmdRight, targetRight);

  st.pwmLeft = cmdLeft;
  st.pwmRight = cmdRight;

  motorState.pwmLeft = cmdLeft;
  motorState.pwmRight = cmdRight;
  motorState.mode = "S2_CERCLE";

  setMotors(cmdLeft, cmdRight);
}

static void stopInternal() {
  cmdLeft = 0;
  cmdRight = 0;
  st.pwmLeft = 0;
  st.pwmRight = 0;
  motorState.pwmLeft = 0;
  motorState.pwmRight = 0;
  motorState.mode = "IDLE";
  stopMotors();
}

static void resetPid() {
  iLeft = 0.0f;
  iRight = 0.0f;
  prevErrLeft = 0.0f;
  prevErrRight = 0.0f;
  firstPid = true;
  firstSpeedFilter = true;
  filteredLeftSpeed = 0.0f;
  filteredRightSpeed = 0.0f;
}

static void computeGeometry() {
  geometryOk = false;
  st.errorMessage = "";

  st.requestedRadiusCm = cfg.radiusCm;
  st.effectiveRadiusCm = cfg.radiusCm * cfg.radiusScale + cfg.radiusOffsetCm;

  // Avec un stylo a 13 cm devant l'axe des roues, un cercle continu n'est possible
  // avec cette methode que si le rayon effectif du stylo est > penOffset.
  // Attention : radiusCm est le rayon demandé par l'utilisateur, mais la géométrie
  // peut utiliser un rayon effectif calibre pour compenser les erreurs réelles.
  if (st.effectiveRadiusCm <= cfg.penOffsetCm + 0.2f) {
    st.errorMessage = "Rayon effectif trop petit pour le mode continu. Augmenter radiusScale/radiusOffset ou utiliser R >= 14 cm.";
    return;
  }

  float robotRadiusSq = st.effectiveRadiusCm * st.effectiveRadiusCm - cfg.penOffsetCm * cfg.penOffsetCm;
  if (robotRadiusSq <= 0.0f) {
    st.errorMessage = "Geometrie impossible.";
    return;
  }

  st.robotRadiusCm = sqrt(robotRadiusSq);
  st.innerWheelRadiusCm = st.robotRadiusCm - cfg.wheelBaseCm * 0.5f;
  st.outerWheelRadiusCm = st.robotRadiusCm + cfg.wheelBaseCm * 0.5f;

  if (st.innerWheelRadiusCm <= 0.3f) {
    st.errorMessage = "Rayon proche de la limite : roue interieure trop lente. Augmenter le rayon.";
    return;
  }

  const float fullTurn = 2.0f * PI * cfg.closureFactor;

  const float targetInnerDistance = fullTurn * st.innerWheelRadiusCm;
  const float targetOuterDistance = fullTurn * st.outerWheelRadiusCm;

  if (cfg.direction == 1) {
    // Cercle a gauche : roue gauche interieure, roue droite exterieure.
    st.targetLeftDistanceCm = targetInnerDistance;
    st.targetRightDistanceCm = targetOuterDistance;
  } else {
    // Cercle a droite : roue gauche exterieure, roue droite interieure.
    st.targetLeftDistanceCm = targetOuterDistance;
    st.targetRightDistanceCm = targetInnerDistance;
  }

  st.targetOuterDistanceCm = targetOuterDistance;
  st.targetWheelDistanceDiffCm = 2.0f * PI * cfg.wheelBaseCm * cfg.closureFactor;
  geometryOk = true;
}


static float outerSpeedPidToAveragePwm(float targetSpeed,
                                       float measuredSpeed,
                                       float& integral,
                                       float& prevErr,
                                       float dt) {
  if (targetSpeed < 0.01f) return 0.0f;

  float err = targetSpeed - measuredSpeed;

  if (fabs(err) < cfg.speedDeadbandCms) {
    err = 0.0f;
  }

  if (err != 0.0f) {
    integral += err * dt;
    integral = clampf(integral, -cfg.integralLimit, cfg.integralLimit);
  }

  float dErr = 0.0f;
  if (!firstPid && dt > 0.001f) {
    dErr = (err - prevErr) / dt;
  }
  prevErr = err;

  float pid = cfg.kpSpeed * err + cfg.kiSpeed * integral + cfg.kdSpeed * dErr;
  pid = clampf(pid, -cfg.maxPidCorrectionPwm, cfg.maxPidCorrectionPwm);

  float avgPwm = cfg.kFF * targetSpeed + pid;

  // La roue exterieure est la roue maitre : elle doit tourner en continu.
  // Si la commande est non nulle mais sous le seuil moteur, on met le seuil.
  if (avgPwm > 1.0f && avgPwm < cfg.minPwm) {
    avgPwm = cfg.minPwm;
  }

  return clampf(avgPwm, 0.0f, (float)cfg.maxPwm);
}

static int averagePwmToPulseCommand(float avgPwm, unsigned long now, float& dutyOut) {
  avgPwm = clampf(avgPwm, 0.0f, (float)cfg.maxPwm);

  if (avgPwm < 1.0f) {
    dutyOut = 0.0f;
    return 0;
  }

  // Si la commande moyenne depasse le seuil reel du moteur, on peut l'envoyer en continu.
  if (avgPwm >= cfg.minPwm) {
    dutyOut = 1.0f;
    return (int)avgPwm;
  }

  // Sinon, on produit une vitesse moyenne faible par impulsions.
  // C'est indispensable pour R=14-18 cm : la roue interieure doit souvent
  // aller beaucoup moins vite que ce que permet un PWM continu de 170.
  unsigned long period = max((unsigned long)60, cfg.pulsePeriodMs);
  float duty = clampf(avgPwm / max(1.0f, (float)cfg.minPwm), 0.03f, 0.95f);
  unsigned long onTime = (unsigned long)(period * duty);
  unsigned long phase = now % period;

  dutyOut = duty;
  return (phase < onTime) ? cfg.minPwm : 0;
}

static void computeTargets() {
  computeGeometry();

  if (!geometryOk) {
    st.targetLeftSpeedCms = 0.0f;
    st.targetRightSpeedCms = 0.0f;
    return;
  }

  float outerSpeed = clampf(cfg.outerWheelSpeedCms, 1.0f, 20.0f);
  st.speedRatio = st.innerWheelRadiusCm / st.outerWheelRadiusCm;
  float innerSpeed = outerSpeed * st.speedRatio;

  if (cfg.direction == 1) {
    // Cercle a gauche : roue gauche interieure, roue droite exterieure.
    st.targetLeftSpeedCms = innerSpeed;
    st.targetRightSpeedCms = outerSpeed;
  } else {
    // Cercle a droite : roue gauche exterieure, roue droite interieure.
    st.targetLeftSpeedCms = outerSpeed;
    st.targetRightSpeedCms = innerSpeed;
  }
}

void begin() {
  loadConfig();
  computeTargets();
  stopInternal();
  st.state = State::IDLE;
  Logger::log("Module cercle 14-20 pret");
}

void update(unsigned long now, float dt) {
  (void)now;

  if (st.state != State::RUNNING) return;

  computeTargets();

  if (!geometryOk) {
    stopInternal();
    st.state = State::ERROR;
    Logger::log("Cercle erreur : " + st.errorMessage);
    return;
  }

  st.rawLeftSpeedCms = odometryState.speedLeftCms;
  st.rawRightSpeedCms = odometryState.speedRightCms;

  float alpha = clampf(cfg.speedFilterAlpha, 0.05f, 1.0f);
  if (firstSpeedFilter) {
    filteredLeftSpeed = st.rawLeftSpeedCms;
    filteredRightSpeed = st.rawRightSpeedCms;
    firstSpeedFilter = false;
  } else {
    filteredLeftSpeed += alpha * (st.rawLeftSpeedCms - filteredLeftSpeed);
    filteredRightSpeed += alpha * (st.rawRightSpeedCms - filteredRightSpeed);
  }

  st.measuredLeftSpeedCms = filteredLeftSpeed;
  st.measuredRightSpeedCms = filteredRightSpeed;

  st.leftDistanceCm = fabs(getLeftDistanceCm() - startLeftDistCm);
  st.rightDistanceCm = fabs(getRightDistanceCm() - startRightDistCm);
  st.outerDistanceCm = (cfg.direction == 1) ? st.rightDistanceCm : st.leftDistanceCm;

  st.leftProgress = st.leftDistanceCm / max(0.1f, st.targetLeftDistanceCm);
  st.rightProgress = st.rightDistanceCm / max(0.1f, st.targetRightDistanceCm);

  // Progression distance : utile pour debug, mais PAS utilisée comme critère principal d'arrêt.
  // Si la roue intérieure est très lente, cette moyenne peut retarder l'arrêt et faire faire
  // plusieurs tours. On garde donc cette valeur seulement pour l'interface.
  float distanceProgress01 = 0.5f * (st.leftProgress + st.rightProgress);
  st.progressPercent = 100.0f * distanceProgress01;

  // Critère physique correct pour un cercle complet d'un robot différentiel :
  // le robot a fait 360° quand |distance_roue_ext - distance_roue_int| = 2*pi*wheelBase.
  // Ce critère évite de refaire un tour parce que la roue intérieure n'a pas exactement
  // atteint sa distance cible.
  st.wheelDistanceDiffCm = fabs(st.rightDistanceCm - st.leftDistanceCm);
  st.targetWheelDistanceDiffCm = 2.0f * PI * cfg.wheelBaseCm * cfg.closureFactor;
  st.angularProgress = st.wheelDistanceDiffCm / max(0.1f, st.targetWheelDistanceDiffCm);
  st.angularProgressPercent = 100.0f * st.angularProgress;

  if (st.angularProgress >= cfg.stopTurnFactor) {
    stopInternal();
    st.state = State::FINISHED;
    Logger::log("Cercle termine : angularProgress=" + String(st.angularProgress, 3) +
                " distanceProgress=" + String(distanceProgress01, 3));
    return;
  }

  // Par défaut, PAS de ralentissement en fin de cercle.
  // Le ralentissement faisait tomber la roue intérieure sous sa vitesse fiable, donc elle
  // s'arrêtait par impulsions et le cercle se cassait vers 1/2 ou 3/4 pour R=16/18.
  float slowScale = 1.0f;
  if (cfg.endSlowdownEnabled) {
    float p = clampf(st.angularProgress, 0.0f, 1.0f);
    if (p > cfg.endSlowdownStart) {
      float span = max(0.02f, 1.0f - cfg.endSlowdownStart);
      float t = (1.0f - p) / span;
      slowScale = clampf(t, cfg.endSlowdownMinScale, 1.0f);
    }
  }

  float targetLeft = st.targetLeftSpeedCms * slowScale;
  float targetRight = st.targetRightSpeedCms * slowScale;

  // V8 : controle maitre-esclave.
  // 1) La roue exterieure est controlee par PID vitesse.
  // 2) La roue interieure est calculee a partir du ratio geometrique.
  // 3) Une petite correction de progression evite que la roue interieure prenne trop d'avance ou de retard.
  const bool leftIsOuter = (cfg.direction == 0);
  float targetOuterSpeed = leftIsOuter ? targetLeft : targetRight;
  float measuredOuterSpeed = leftIsOuter ? st.measuredLeftSpeedCms : st.measuredRightSpeedCms;

  float outerAvgPwm = outerSpeedPidToAveragePwm(targetOuterSpeed, measuredOuterSpeed, iRight, prevErrRight, dt);

  // Erreur de ratio : les progressions normalisees des deux roues doivent rester proches.
  float innerProgress = (cfg.direction == 1) ? st.leftProgress : st.rightProgress;
  float outerProgress = (cfg.direction == 1) ? st.rightProgress : st.leftProgress;
  float ratioErr = outerProgress - innerProgress;
  float ratioTrim = clampf(cfg.ratioTrimKp * ratioErr, -cfg.maxRatioTrimPwm, cfg.maxRatioTrimPwm);

  float innerAvgPwm = outerAvgPwm * st.speedRatio + ratioTrim;
  innerAvgPwm = clampf(innerAvgPwm, 0.0f, (float)cfg.maxPwm);

  float dutyInner = 0.0f;
  int pwmOuter = averagePwmToPulseCommand(outerAvgPwm, now, st.innerPulseDuty /* dummy overwritten below */);
  int pwmInner = averagePwmToPulseCommand(innerAvgPwm, now, dutyInner);

  st.outerBasePwm = outerAvgPwm;
  st.innerAveragePwm = innerAvgPwm;
  st.innerPulseDuty = dutyInner;

  int pwmL = 0;
  int pwmR = 0;
  if (cfg.direction == 1) {
    // gauche = interieure, droite = exterieure
    pwmL = pwmInner;
    pwmR = pwmOuter;
  } else {
    // gauche = exterieure, droite = interieure
    pwmL = pwmOuter;
    pwmR = pwmInner;
  }

  firstPid = false;

  applyPwmSmooth(pwmL, pwmR);
}

void start() {
  stopInternal();
  computeTargets();

  if (!geometryOk) {
    st.state = State::ERROR;
    Logger::log("Impossible de lancer cercle : " + st.errorMessage);
    return;
  }

  resetPid();
  startLeftDistCm = getLeftDistanceCm();
  startRightDistCm = getRightDistanceCm();
  st.outerDistanceCm = 0.0f;
  st.progressPercent = 0.0f;
  st.angularProgress = 0.0f;
  st.angularProgressPercent = 0.0f;
  st.wheelDistanceDiffCm = 0.0f;
  st.targetWheelDistanceDiffCm = 2.0f * PI * cfg.wheelBaseCm * cfg.closureFactor;
  st.state = State::RUNNING;
  motorState.mode = "S2_CERCLE";

  Logger::log("Cercle lance : R=" + String(cfg.radiusCm, 1) +
              " Reff=" + String(st.effectiveRadiusCm, 2) +
              " Rrobot=" + String(st.robotRadiusCm, 2) +
              " targetL=" + String(st.targetLeftDistanceCm, 2) +
              " targetR=" + String(st.targetRightDistanceCm, 2) +
              " factor=" + String(cfg.closureFactor, 2) +
              " outerSpeed=" + String(cfg.outerWheelSpeedCms, 1));
}

void stop() {
  stopInternal();
  st.state = State::IDLE;
  Logger::log("Cercle stop");
}

void reset() {
  stop();
  resetPid();
  st = RuntimeStatus();
}

Config getConfig() {
  return cfg;
}

void setConfig(const Config& newCfg) {
  cfg = newCfg;

  cfg.radiusCm = clampf(cfg.radiusCm, 14.0f, 20.0f);
  cfg.radiusScale = clampf(cfg.radiusScale, 0.50f, 1.80f);
  cfg.radiusOffsetCm = clampf(cfg.radiusOffsetCm, -8.0f, 8.0f);
  cfg.wheelBaseCm = clampf(cfg.wheelBaseCm, 5.0f, 15.0f);
  cfg.penOffsetCm = clampf(cfg.penOffsetCm, 5.0f, 20.0f);
  cfg.direction = cfg.direction ? 1 : 0;
  cfg.outerWheelSpeedCms = clampf(cfg.outerWheelSpeedCms, 1.0f, 20.0f);
  cfg.closureFactor = clampf(cfg.closureFactor, 0.70f, 1.30f);
  cfg.stopTurnFactor = clampf(cfg.stopTurnFactor, 0.90f, 1.05f);
  cfg.endSlowdownEnabled = cfg.endSlowdownEnabled ? 1 : 0;
  cfg.endSlowdownStart = clampf(cfg.endSlowdownStart, 0.50f, 0.98f);
  cfg.endSlowdownMinScale = clampf(cfg.endSlowdownMinScale, 0.40f, 1.0f);
  cfg.minPwm = constrain(cfg.minPwm, 0, 255);
  cfg.maxPwm = constrain(cfg.maxPwm, cfg.minPwm, 255);
  cfg.pwmRampStep = constrain(cfg.pwmRampStep, 1, 255);
  cfg.integralLimit = clampf(cfg.integralLimit, 0.0f, 200.0f);
  cfg.kpSpeed = clampf(cfg.kpSpeed, 0.0f, 80.0f);
  cfg.kiSpeed = clampf(cfg.kiSpeed, 0.0f, 30.0f);
  cfg.kdSpeed = clampf(cfg.kdSpeed, 0.0f, 10.0f);
  cfg.kFF = clampf(cfg.kFF, 0.0f, 60.0f);
  cfg.minReliableSpeedCms = clampf(cfg.minReliableSpeedCms, 0.3f, 8.0f);
  cfg.pulsePeriodMs = (unsigned long)constrain((int)cfg.pulsePeriodMs, 60, 1000);
  cfg.speedFilterAlpha = clampf(cfg.speedFilterAlpha, 0.05f, 1.0f);
  cfg.maxPidCorrectionPwm = clampf(cfg.maxPidCorrectionPwm, 0.0f, 100.0f);
  cfg.speedDeadbandCms = clampf(cfg.speedDeadbandCms, 0.0f, 2.0f);
  cfg.ratioTrimKp = clampf(cfg.ratioTrimKp, 0.0f, 120.0f);
  cfg.maxRatioTrimPwm = clampf(cfg.maxRatioTrimPwm, 0.0f, 120.0f);

  computeTargets();
}

void loadConfig() {
  prefs.begin("s2circle", true);
  cfg.radiusCm = prefs.getFloat("radius", cfg.radiusCm);
  cfg.radiusScale = prefs.getFloat("rscale", cfg.radiusScale);
  cfg.radiusOffsetCm = prefs.getFloat("roffset", cfg.radiusOffsetCm);
  cfg.wheelBaseCm = prefs.getFloat("base", cfg.wheelBaseCm);
  cfg.penOffsetCm = prefs.getFloat("offset", cfg.penOffsetCm);
  cfg.direction = prefs.getInt("dir", cfg.direction);
  cfg.outerWheelSpeedCms = prefs.getFloat("outerSp", cfg.outerWheelSpeedCms);
  cfg.closureFactor = prefs.getFloat("close", cfg.closureFactor);
  cfg.stopTurnFactor = prefs.getFloat("stopTurn", cfg.stopTurnFactor);
  cfg.endSlowdownEnabled = prefs.getInt("endSlow", cfg.endSlowdownEnabled);
  cfg.endSlowdownStart = prefs.getFloat("slowStart", cfg.endSlowdownStart);
  cfg.endSlowdownMinScale = prefs.getFloat("slowMin", cfg.endSlowdownMinScale);
  cfg.minPwm = prefs.getInt("minPwm", cfg.minPwm);
  cfg.maxPwm = prefs.getInt("maxPwm", cfg.maxPwm);
  cfg.pwmRampStep = prefs.getInt("ramp", cfg.pwmRampStep);
  cfg.kpSpeed = prefs.getFloat("kp", cfg.kpSpeed);
  cfg.kiSpeed = prefs.getFloat("ki", cfg.kiSpeed);
  cfg.kdSpeed = prefs.getFloat("kd", cfg.kdSpeed);
  cfg.integralLimit = prefs.getFloat("ilim", cfg.integralLimit);
  cfg.kFF = prefs.getFloat("kff", cfg.kFF);
  cfg.minReliableSpeedCms = prefs.getFloat("minRel", cfg.minReliableSpeedCms);
  cfg.pulsePeriodMs = (unsigned long)prefs.getInt("pulse", (int)cfg.pulsePeriodMs);
  cfg.speedFilterAlpha = prefs.getFloat("sfAlpha", cfg.speedFilterAlpha);
  cfg.maxPidCorrectionPwm = prefs.getFloat("pidMax", cfg.maxPidCorrectionPwm);
  cfg.speedDeadbandCms = prefs.getFloat("spdDb", cfg.speedDeadbandCms);
  cfg.ratioTrimKp = prefs.getFloat("ratioKp", cfg.ratioTrimKp);
  cfg.maxRatioTrimPwm = prefs.getFloat("ratioMax", cfg.maxRatioTrimPwm);
  prefs.end();
  setConfig(cfg);
}

void saveConfig() {
  prefs.begin("s2circle", false);
  prefs.putFloat("radius", cfg.radiusCm);
  prefs.putFloat("rscale", cfg.radiusScale);
  prefs.putFloat("roffset", cfg.radiusOffsetCm);
  prefs.putFloat("base", cfg.wheelBaseCm);
  prefs.putFloat("offset", cfg.penOffsetCm);
  prefs.putInt("dir", cfg.direction);
  prefs.putFloat("outerSp", cfg.outerWheelSpeedCms);
  prefs.putFloat("close", cfg.closureFactor);
  prefs.putFloat("stopTurn", cfg.stopTurnFactor);
  prefs.putInt("endSlow", cfg.endSlowdownEnabled);
  prefs.putFloat("slowStart", cfg.endSlowdownStart);
  prefs.putFloat("slowMin", cfg.endSlowdownMinScale);
  prefs.putInt("minPwm", cfg.minPwm);
  prefs.putInt("maxPwm", cfg.maxPwm);
  prefs.putInt("ramp", cfg.pwmRampStep);
  prefs.putFloat("kp", cfg.kpSpeed);
  prefs.putFloat("ki", cfg.kiSpeed);
  prefs.putFloat("kd", cfg.kdSpeed);
  prefs.putFloat("ilim", cfg.integralLimit);
  prefs.putFloat("kff", cfg.kFF);
  prefs.putFloat("minRel", cfg.minReliableSpeedCms);
  prefs.putInt("pulse", (int)cfg.pulsePeriodMs);
  prefs.putFloat("sfAlpha", cfg.speedFilterAlpha);
  prefs.putFloat("pidMax", cfg.maxPidCorrectionPwm);
  prefs.putFloat("spdDb", cfg.speedDeadbandCms);
  prefs.putFloat("ratioKp", cfg.ratioTrimKp);
  prefs.putFloat("ratioMax", cfg.maxRatioTrimPwm);
  prefs.end();
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
  computeTargets();
  String j = "{";
  j += "\"radiusCm\":" + String(cfg.radiusCm, 3) + ",";
  j += "\"radiusScale\":" + String(cfg.radiusScale, 4) + ",";
  j += "\"radiusOffsetCm\":" + String(cfg.radiusOffsetCm, 3) + ",";
  j += "\"wheelBaseCm\":" + String(cfg.wheelBaseCm, 3) + ",";
  j += "\"penOffsetCm\":" + String(cfg.penOffsetCm, 3) + ",";
  j += "\"direction\":" + String(cfg.direction) + ",";
  j += "\"outerWheelSpeedCms\":" + String(cfg.outerWheelSpeedCms, 3) + ",";
  j += "\"closureFactor\":" + String(cfg.closureFactor, 4) + ",";
  j += "\"stopTurnFactor\":" + String(cfg.stopTurnFactor, 4) + ",";
  j += "\"endSlowdownEnabled\":" + String(cfg.endSlowdownEnabled) + ",";
  j += "\"endSlowdownStart\":" + String(cfg.endSlowdownStart, 4) + ",";
  j += "\"endSlowdownMinScale\":" + String(cfg.endSlowdownMinScale, 4) + ",";
  j += "\"minPwm\":" + String(cfg.minPwm) + ",";
  j += "\"maxPwm\":" + String(cfg.maxPwm) + ",";
  j += "\"pwmRampStep\":" + String(cfg.pwmRampStep) + ",";
  j += "\"kpSpeed\":" + String(cfg.kpSpeed, 4) + ",";
  j += "\"kiSpeed\":" + String(cfg.kiSpeed, 4) + ",";
  j += "\"kdSpeed\":" + String(cfg.kdSpeed, 4) + ",";
  j += "\"integralLimit\":" + String(cfg.integralLimit, 4) + ",";
  j += "\"kFF\":" + String(cfg.kFF, 4) + ",";
  j += "\"minReliableSpeedCms\":" + String(cfg.minReliableSpeedCms, 3) + ",";
  j += "\"pulsePeriodMs\":" + String((int)cfg.pulsePeriodMs) + ",";
  j += "\"speedFilterAlpha\":" + String(cfg.speedFilterAlpha, 4) + ",";
  j += "\"maxPidCorrectionPwm\":" + String(cfg.maxPidCorrectionPwm, 3) + ",";
  j += "\"speedDeadbandCms\":" + String(cfg.speedDeadbandCms, 3) + ",";
  j += "\"ratioTrimKp\":" + String(cfg.ratioTrimKp, 3) + ",";
  j += "\"maxRatioTrimPwm\":" + String(cfg.maxRatioTrimPwm, 3);
  j += "}";
  return j;
}

String statusJson() {
  computeTargets();
  String j = "{";
  j += "\"state\":\"" + stateName() + "\",";
  j += "\"requestedRadiusCm\":" + String(st.requestedRadiusCm, 3) + ",";
  j += "\"effectiveRadiusCm\":" + String(st.effectiveRadiusCm, 3) + ",";
  j += "\"robotRadiusCm\":" + String(st.robotRadiusCm, 3) + ",";
  j += "\"innerWheelRadiusCm\":" + String(st.innerWheelRadiusCm, 3) + ",";
  j += "\"outerWheelRadiusCm\":" + String(st.outerWheelRadiusCm, 3) + ",";
  j += "\"speedRatio\":" + String(st.speedRatio, 4) + ",";
  j += "\"targetLeftSpeedCms\":" + String(st.targetLeftSpeedCms, 3) + ",";
  j += "\"targetRightSpeedCms\":" + String(st.targetRightSpeedCms, 3) + ",";
  j += "\"measuredLeftSpeedCms\":" + String(st.measuredLeftSpeedCms, 3) + ",";
  j += "\"measuredRightSpeedCms\":" + String(st.measuredRightSpeedCms, 3) + ",";
  j += "\"rawLeftSpeedCms\":" + String(st.rawLeftSpeedCms, 3) + ",";
  j += "\"rawRightSpeedCms\":" + String(st.rawRightSpeedCms, 3) + ",";
  j += "\"targetLeftDistanceCm\":" + String(st.targetLeftDistanceCm, 3) + ",";
  j += "\"targetRightDistanceCm\":" + String(st.targetRightDistanceCm, 3) + ",";
  j += "\"targetOuterDistanceCm\":" + String(st.targetOuterDistanceCm, 3) + ",";
  j += "\"leftDistanceCm\":" + String(st.leftDistanceCm, 3) + ",";
  j += "\"rightDistanceCm\":" + String(st.rightDistanceCm, 3) + ",";
  j += "\"outerDistanceCm\":" + String(st.outerDistanceCm, 3) + ",";
  j += "\"leftProgress\":" + String(st.leftProgress, 4) + ",";
  j += "\"rightProgress\":" + String(st.rightProgress, 4) + ",";
  j += "\"progressPercent\":" + String(st.progressPercent, 2) + ",";
  j += "\"angularProgress\":" + String(st.angularProgress, 4) + ",";
  j += "\"angularProgressPercent\":" + String(st.angularProgressPercent, 2) + ",";
  j += "\"wheelDistanceDiffCm\":" + String(st.wheelDistanceDiffCm, 3) + ",";
  j += "\"targetWheelDistanceDiffCm\":" + String(st.targetWheelDistanceDiffCm, 3) + ",";
  j += "\"pwmLeft\":" + String(st.pwmLeft) + ",";
  j += "\"pwmRight\":" + String(st.pwmRight) + ",";
  j += "\"outerBasePwm\":" + String(st.outerBasePwm, 2) + ",";
  j += "\"innerAveragePwm\":" + String(st.innerAveragePwm, 2) + ",";
  j += "\"innerPulseDuty\":" + String(st.innerPulseDuty, 3) + ",";
  j += "\"errorMessage\":\"" + st.errorMessage + "\"";
  j += "}";
  return j;
}

}

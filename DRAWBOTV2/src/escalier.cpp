#include <Arduino.h>
#include "include/escalier.h"
#include "include/moteurs.h"
#include "include/encodeurs.h"
#include "include/robot.h"

// ==================================================================
// MACHINE À ÉTATS
// ==================================================================
enum EscalierStep {
    STEP_IDLE = 0,

    // Segment 1 : ligne droite 20 cm
    STEP_SEG1_RUN,
    STEP_SEG1_BRAKE,

    // Virage gauche 90°
    STEP_TURN_LEFT_RUN,
    STEP_TURN_LEFT_BRAKE,

    // Segment 2 : ligne droite 10 cm
    STEP_SEG2_RUN,
    STEP_SEG2_BRAKE,

    // Virage droit 90°
    STEP_TURN_RIGHT_RUN,
    STEP_TURN_RIGHT_BRAKE,

    // Segment 3 : ligne droite 40 cm
    STEP_SEG3_RUN,
    STEP_SEG3_BRAKE,

    STEP_FINISHED
};

// Noms lisibles pour debug / IHM
static const char* STEP_NAMES[] = {
    "IDLE",
    "SEG1_RUN", "SEG1_BRAKE",
    "TURN_LEFT_RUN", "TURN_LEFT_BRAKE",
    "SEG2_RUN", "SEG2_BRAKE",
    "TURN_RIGHT_RUN", "TURN_RIGHT_BRAKE",
    "SEG3_RUN", "SEG3_BRAKE",
    "FINISHED"
};

// ==================================================================
// ÉTAT INTERNE
// ==================================================================
static EscalierStep  currentStep = STEP_IDLE;
static EscalierParams params;

// Pour les lignes droites
static float straightErrorTicks = 0.0f;
static int   leftCmd  = 0;
static int   rightCmd = 0;

// Pour le freinage
static unsigned long brakeStart = 0;

// Pour la mesure d'angle par encodeurs pendant le virage
// On mesure l'arc parcouru par la roue extérieure.
// arc_ext = R_ext × θ   avec R_ext = entraxe/2 + entraxe/2 (roue intérieure quasi statique)
// On cible la distance parcourue par la roue rapide correspondant à 90°.
static float turnTargetTicksOuter = 0.0f;

// ==================================================================
// UTILITAIRES
// ==================================================================
static int clampCmd(int v) {
    if (v >  255) return  255;
    if (v < -255) return -255;
    return v;
}

// Calcule le nombre de ticks que la roue EXTÉRIEURE doit parcourir
// pour effectuer un angle donné (degrés).
// La roue intérieure est supposée avancer à ~innerPwm/outerPwm × distance ext.
// En première approximation on se base sur l'arc extérieur.
//   arc_ext = (entraxe) × angle_rad    (quand la roue intérieure est quasi immobile)
static float ticksForTurn(float angleDeg) {
    float angleRad = angleDeg * RobotParams::PI_F / 180.0f;
    float arcCm    = RobotParams::WHEEL_BASE_CM * angleRad;
    return arcCm / RobotParams::CM_PAR_TICK;
}

// ==================================================================
// API
// ==================================================================
void escalierSetParams(const EscalierParams& p) { params = p; }
const EscalierParams& escalierGetParams()        { return params; }

void escalierReset() {
    currentStep       = STEP_IDLE;
    straightErrorTicks = 0.0f;
    leftCmd = rightCmd = 0;
    brakeStart = 0;
    turnTargetTicksOuter = 0.0f;
}

bool escalierIsRunning()  { return currentStep != STEP_IDLE && currentStep != STEP_FINISHED; }
bool escalierIsFinished() { return currentStep == STEP_FINISHED; }

const char* escalierGetStepName() {
    int idx = static_cast<int>(currentStep);
    return STEP_NAMES[idx];
}

void escalierStart() {
    escalierReset();
    resetEncoders();
    currentStep = STEP_SEG1_RUN;
    Serial.println("[ESCALIER] Démarrage séquence");
}

// ==================================================================
// UPDATE – à appeler dans loop()
// ==================================================================
void escalierUpdate(unsigned long nowMs) {

    switch (currentStep) {

    // ------------------------------------------------------------------
    // IDLE / FINISHED – ne rien faire
    // ------------------------------------------------------------------
    case STEP_IDLE:
    case STEP_FINISHED:
        stopMotors();
        break;

    // ==================================================================
    // SEGMENT 1 – ligne droite
    // ==================================================================
    case STEP_SEG1_RUN:
    case STEP_SEG2_RUN:
    case STEP_SEG3_RUN: {
        float targetCm = (currentStep == STEP_SEG1_RUN) ? params.seg1Cm
                       : (currentStep == STEP_SEG2_RUN) ? params.seg2Cm
                       :                                   params.seg3Cm;

        float distCm    = getAverageDistanceCm();
        float remaining = targetCm - distCm;

        if (remaining <= 0.0f) {
            // Cible atteinte → freinage
            brakeMotors();
            brakeStart = nowMs;
            currentStep = (currentStep == STEP_SEG1_RUN) ? STEP_SEG1_BRAKE
                        : (currentStep == STEP_SEG2_RUN) ? STEP_SEG2_BRAKE
                        :                                   STEP_SEG3_BRAKE;
            Serial.printf("[ESCALIER] %s → BRAKE (dist=%.1f cm)\n",
                          STEP_NAMES[static_cast<int>(currentStep)-1], distCm);
            break;
        }

        // Choix PWM selon zone
        int basePwm = (remaining <= params.slowZoneCm) ? params.slowPwm : params.cruisePwm;

        // Correction de cap : différence de ticks gauche/droite
        long lTicks = getLeftEncoderTicks();
        long rTicks = getRightEncoderTicks();
        straightErrorTicks = static_cast<float>(lTicks - rTicks);

        float correction = params.kpStraight * straightErrorTicks;

        leftCmd  = clampCmd(basePwm - static_cast<int>(correction));
        rightCmd = clampCmd(basePwm + static_cast<int>(correction));

        // Garantir le PWM minimum mécanique
        if (leftCmd  > 0 && leftCmd  < RobotParams::PWM_MIN) leftCmd  = RobotParams::PWM_MIN;
        if (rightCmd > 0 && rightCmd < RobotParams::PWM_MIN) rightCmd = RobotParams::PWM_MIN;

        setMotors(leftCmd, rightCmd);
        break;
    }

    // ==================================================================
    // FREINAGE après chaque segment droit
    // ==================================================================
    case STEP_SEG1_BRAKE:
    case STEP_SEG2_BRAKE:
    case STEP_SEG3_BRAKE: {
        brakeMotors();
        if (nowMs - brakeStart >= params.brakeMs) {
            stopMotors();
            delay(100); // petite pause pour stabilisation
            resetEncoders();

            if (currentStep == STEP_SEG1_BRAKE) {
                // Prépare virage gauche
                turnTargetTicksOuter = ticksForTurn(90.0f);
                currentStep = STEP_TURN_LEFT_RUN;
                Serial.printf("[ESCALIER] → TURN_LEFT (cible=%.0f ticks ext)\n",
                              turnTargetTicksOuter);
            } else if (currentStep == STEP_SEG2_BRAKE) {
                // Prépare virage droit
                turnTargetTicksOuter = ticksForTurn(90.0f);
                currentStep = STEP_TURN_RIGHT_RUN;
                Serial.printf("[ESCALIER] → TURN_RIGHT (cible=%.0f ticks ext)\n",
                              turnTargetTicksOuter);
            } else {
                // Fin de séquence
                currentStep = STEP_FINISHED;
                Serial.println("[ESCALIER] Séquence TERMINÉE");
            }
        }
        break;
    }

    // ==================================================================
    // VIRAGE GAUCHE
    // Roue droite = extérieure (rapide), roue gauche = intérieure (lente)
    // On mesure les ticks de la roue droite.
    // ==================================================================
    case STEP_TURN_LEFT_RUN: {
        long outerTicks = getRightEncoderTicks(); // droite = extérieure

        if (outerTicks >= static_cast<long>(turnTargetTicksOuter)) {
            brakeMotors();
            brakeStart  = nowMs;
            currentStep = STEP_TURN_LEFT_BRAKE;
            Serial.printf("[ESCALIER] TURN_LEFT terminé (ticks droite=%ld)\n", outerTicks);
            break;
        }

        // Différentiel de vitesse : droite rapide, gauche lente
        setMotors(static_cast<int>(params.turnLeftInnerPwm),
                  static_cast<int>(params.turnLeftOuterPwm));
        break;
    }

    case STEP_TURN_LEFT_BRAKE: {
        brakeMotors();
        if (nowMs - brakeStart >= params.brakeMs) {
            stopMotors();
            delay(100);
            resetEncoders();
            currentStep = STEP_SEG2_RUN;
            Serial.println("[ESCALIER] → SEG2");
        }
        break;
    }

    // ==================================================================
    // VIRAGE DROIT
    // Roue gauche = extérieure (rapide), roue droite = intérieure (lente)
    // ==================================================================
    case STEP_TURN_RIGHT_RUN: {
        long outerTicks = getLeftEncoderTicks(); // gauche = extérieure

        if (outerTicks >= static_cast<long>(turnTargetTicksOuter)) {
            brakeMotors();
            brakeStart  = nowMs;
            currentStep = STEP_TURN_RIGHT_BRAKE;
            Serial.printf("[ESCALIER] TURN_RIGHT terminé (ticks gauche=%ld)\n", outerTicks);
            break;
        }

        // Différentiel de vitesse : gauche rapide, droite lente
        setMotors(static_cast<int>(params.turnRightOuterPwm),
                  static_cast<int>(params.turnRightInnerPwm));
        break;
    }

    case STEP_TURN_RIGHT_BRAKE: {
        brakeMotors();
        if (nowMs - brakeStart >= params.brakeMs) {
            stopMotors();
            delay(100);
            resetEncoders();
            currentStep = STEP_SEG3_RUN;
            Serial.println("[ESCALIER] → SEG3");
        }
        break;
    }

    } // switch
}
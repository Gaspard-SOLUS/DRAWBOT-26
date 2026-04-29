#pragma once
#include <Arduino.h>

// =====================================================================
// PARAMÈTRES DE LA SÉQUENCE ESCALIER
// Tous les paramètres réglables depuis l'interface web
// =====================================================================
struct EscalierParams {
    // Distances des trois segments (cm)
    float seg1Cm        = 20.0f;
    float seg2Cm        = 10.0f;
    float seg3Cm        = 40.0f;

    // PWM de croisière sur les lignes droites
    int   cruisePwm     = 200;
    // PWM lent en fin de ligne droite (zone de décélération)
    int   slowPwm       = 150;
    // Distance de décélération avant l'arrêt (cm)
    float slowZoneCm    = 3.0f;

    // Gain proportionnel pour corriger le cap en ligne droite
    // (erreur en ticks → correction PWM)
    float kpStraight    = 0.8f;

    // ---- Virage gauche (90°) ----
    // PWM roue extérieure (droite) pendant le virage
    float turnLeftOuterPwm  = 190.0f;
    // PWM roue intérieure (gauche) pendant le virage
    float turnLeftInnerPwm  = 80.0f;

    // ---- Virage droit (90°) ----
    float turnRightOuterPwm = 190.0f;
    float turnRightInnerPwm = 80.0f;

    // Durée du freinage actif après chaque phase (ms)
    unsigned long brakeMs = 120;
};

// =====================================================================
// API DE LA SÉQUENCE
// =====================================================================
void escalierSetParams(const EscalierParams& p);
const EscalierParams& escalierGetParams();

// Lance la séquence escalier complète
void escalierStart();

// A appeler dans la boucle principale
void escalierUpdate(unsigned long nowMs);

// Interrogation d'état
bool escalierIsRunning();
bool escalierIsFinished();
void escalierReset();

// Description textuelle de l'étape courante (pour debug / IHM)
const char* escalierGetStepName();
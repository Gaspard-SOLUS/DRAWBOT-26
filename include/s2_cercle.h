#pragma once
#include <Arduino.h>

namespace S2Cercle {

  enum class State {
    IDLE,
    RUNNING,
    FINISHED,
    ERROR
  };

  struct Config {
    // Rayon demande par l'utilisateur : c'est le rayon du cercle trace par le stylo,
    // pas le rayon du centre des roues.
    float radiusCm = 16.0f;

    // Calibration du rayon : permet de corriger l'ecart entre rayon demande et rayon obtenu.
    // effectiveRadiusCm = radiusCm * radiusScale + radiusOffsetCm
    // Exemple : si R demande = 16 cm mais R obtenu = 18 cm, essayer radiusScale = 16/18 = 0.889.
    float radiusScale = 1.0f;
    float radiusOffsetCm = 0.0f;

    float wheelBaseCm = 8.3f;
    float penOffsetCm = 13.0f;

    // 1 = cercle vers la gauche, 0 = cercle vers la droite.
    int direction = 1;

    // Vitesse cible de la roue exterieure.
    float outerWheelSpeedCms = 7.0f;

    // Facteur applique aux distances cibles des roues.
    // Normalement = 1.0 pour un tour complet.
    // Si le robot ferme trop le cercle ou fait plus d'un tour, diminuer.
    // Si le robot s'arrete trop tot, augmenter legerement.
    float closureFactor = 1.0f;

    // Stop du cercle base sur le changement d'angle mesure par les encodeurs.
    // 1.0 = un tour exact. Baisser legerement (0.98-0.995) si le stylo repasse
    // trop sur le debut du cercle.
    float stopTurnFactor = 1.0f;

    // Desactive par defaut : le ralentissement en fin de cercle faisait entrer
    // la roue interieure en regime impulsionnel et cassait le cercle pour R=16/18.
    int endSlowdownEnabled = 0;
    float endSlowdownStart = 0.90f;
    float endSlowdownMinScale = 0.75f;

    // Conversion et securites PWM.
    int minPwm = 170;
    int maxPwm = 250;
    int pwmRampStep = 8;

    // PID vitesse roue gauche / droite.
    float kpSpeed = 10.0f;
    float kiSpeed = 1.2f;
    float kdSpeed = 0.0f;
    float integralLimit = 20.0f;

    // Feed-forward : estimation directe PWM ~= kFF * vitesse.
    // IMPORTANT : on n'ajoute plus minPwm en continu pour les petites vitesses,
    // sinon les petits rayons donnent presque le meme cercle que les grands.
    float kFF = 22.0f;

    // Sous cette vitesse, un moteur DC ne tourne pas proprement en continu.
    // On utilise donc des impulsions PWM pour obtenir une faible vitesse moyenne.
    float minReliableSpeedCms = 4.5f;
    unsigned long pulsePeriodMs = 120;

    // Filtrage et limitation du PID vitesse. Très important pour éviter les oscillations,
    // surtout sur les grands rayons où les deux roues ont des vitesses proches.
    float speedFilterAlpha = 0.30f;      // 0.15 = très filtré, 0.35 = plus réactif
    float maxPidCorrectionPwm = 35.0f;  // limite la correction PID ajoutée au feed-forward
    float speedDeadbandCms = 0.25f;

    // Correction douce du ratio de distance entre les deux roues.
    // Elle sert à garder le rayon sans laisser un PID individuel pousser une roue trop fort.
    float ratioTrimKp = 35.0f;
    float maxRatioTrimPwm = 35.0f;     // petite zone morte pour ne pas corriger le bruit
  };

  struct RuntimeStatus {
    State state = State::IDLE;

    float requestedRadiusCm = 0.0f;
    float effectiveRadiusCm = 0.0f;
    float robotRadiusCm = 0.0f;
    float innerWheelRadiusCm = 0.0f;
    float outerWheelRadiusCm = 0.0f;

    float targetLeftSpeedCms = 0.0f;
    float targetRightSpeedCms = 0.0f;
    float speedRatio = 0.0f;
    float measuredLeftSpeedCms = 0.0f;
    float measuredRightSpeedCms = 0.0f;
    float rawLeftSpeedCms = 0.0f;
    float rawRightSpeedCms = 0.0f;

    float targetLeftDistanceCm = 0.0f;
    float targetRightDistanceCm = 0.0f;
    float targetOuterDistanceCm = 0.0f;

    float leftDistanceCm = 0.0f;
    float rightDistanceCm = 0.0f;
    float outerDistanceCm = 0.0f;

    float leftProgress = 0.0f;
    float rightProgress = 0.0f;
    float progressPercent = 0.0f;

    // Progression angulaire calculee avec la difference des distances roues.
    // C'est le vrai critere pour savoir si le robot a fait 360 degres.
    float angularProgress = 0.0f;
    float angularProgressPercent = 0.0f;
    float wheelDistanceDiffCm = 0.0f;
    float targetWheelDistanceDiffCm = 0.0f;

    int pwmLeft = 0;
    int pwmRight = 0;

    float outerBasePwm = 0.0f;
    float innerAveragePwm = 0.0f;
    float innerPulseDuty = 0.0f;

    String errorMessage = "";
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

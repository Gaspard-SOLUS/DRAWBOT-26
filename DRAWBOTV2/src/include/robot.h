#pragma once

namespace RobotParams {

    // ===================== ROUES =====================
    // Diamètre des roues en cm
    static constexpr float WHEEL_DIAMETER_CM = 9.0f;

    // Entraxe (distance entre les deux roues) en cm
    // A mesurer précisément sur votre robot !
    static constexpr float WHEEL_BASE_CM = 12.5f;

    // ===================== ENCODEURS =====================
    // Nombre de ticks par tour de roue (encodeur en quadrature)
    // Motoréducteur N20 100RPM : 6 ticks/tr moteur × rapport réducteur
    // A calibrer expérimentalement !
    static constexpr float TICKS_PER_REVOLUTION = 360.0f;

    // ===================== CONVERSIONS =====================
    static constexpr float PI_F = 3.14159265f;

    // Périmètre d'une roue en cm
    static constexpr float WHEEL_PERIMETER_CM = PI_F * WHEEL_DIAMETER_CM;

    // Centimètres par tick encodeur
    static constexpr float CM_PAR_TICK = WHEEL_PERIMETER_CM / TICKS_PER_REVOLUTION;

    // ===================== VITESSES =====================
    // Vitesse de croisière en cm/s pour les lignes droites
    static constexpr float CRUISE_SPEED_CM_S = 15.0f;

    // ===================== PWM =====================
    // PWM minimum pour que les moteurs bougent (deadzone mécanique)
    static constexpr int PWM_MIN = 130;
    static constexpr int PWM_MAX = 255;

} // namespace RobotParams
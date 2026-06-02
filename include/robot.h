#pragma once

namespace RobotParams {
  constexpr float ROBOT_PI = 3.14159265358979323846f;

  // Diamètre de la roue en cm
  constexpr float ROUE_DIAMETER_CM = 9.0f;

  // Nombre de ticks par tour
  constexpr float TICKS_PAR_TOUR = 2100.0f;

  constexpr float WHEEL_BASE_CM = 8.3f;
  constexpr float PEN_OFFSET_CM = 13.0f;

  constexpr float ROUE_CIRCONFERENCE_CM = ROBOT_PI * ROUE_DIAMETER_CM;
  constexpr float CM_PAR_TICK = ROUE_CIRCONFERENCE_CM / TICKS_PAR_TOUR;
}

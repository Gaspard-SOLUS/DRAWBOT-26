#pragma once

namespace RobotParams {
  // Diamètre de la roue en cm
  constexpr float ROUE_DIAMETER_CM = 9.0f;

  // Nombre de ticks par tour
  constexpr float TICKS_PAR_TOUR = 2100.0f;

  //constexpr float PI = 3.1415926535897932384626433832795f;
  constexpr float ROUE_CIRCONFERENCE_CM = PI * ROUE_DIAMETER_CM;
  constexpr float CM_PAR_TICK = ROUE_CIRCONFERENCE_CM / TICKS_PAR_TOUR;
}
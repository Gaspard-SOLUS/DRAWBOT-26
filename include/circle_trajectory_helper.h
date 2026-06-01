#pragma once
#include <Arduino.h>
#include "trajectory_generator.h"

namespace CircleTrajectoryHelper {
  enum class StartMode {
    Bottom,
    Right
  };

  struct CircleRequest {
    float radiusCm = 5.0f;
    int segments = 96;
    bool clockwise = true;
    StartMode startMode = StartMode::Bottom;
  };

  bool buildSmallCircle(TrajectoryGenerator::Trajectory& trajectory, const CircleRequest& request);
  float initialHeadingRad(const TrajectoryGenerator::Trajectory& trajectory);
}

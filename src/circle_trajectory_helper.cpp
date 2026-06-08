#include <Arduino.h>
#include <math.h>

#include "circle_trajectory_helper.h"

namespace CircleTrajectoryHelper {
  bool buildSmallCircle(TrajectoryGenerator::Trajectory& trajectory, const CircleRequest& request) {
    if (request.radiusCm <= 0.0f) {
      return false;
    }

    int segments = request.segments;
    if (segments < 24) segments = 24;
    if (segments > TrajectoryGenerator::MAX_SEGMENTS) {
      segments = TrajectoryGenerator::MAX_SEGMENTS;
    }

    float radius = request.radiusCm * request.distanceScale;
    float centerX = 0.0f;
    float centerY = radius;
    float startAngleDeg = -90.0f;

    if (request.startMode == StartMode::Right) {
      centerX = -radius;
      centerY = 0.0f;
      startAngleDeg = 0.0f;
    }

    return TrajectoryGenerator::generateCircle(
      trajectory,
      radius,
      segments,
      centerX,
      centerY,
      startAngleDeg,
      request.clockwise,
      1.0f
    );
  }

  float initialHeadingRad(const TrajectoryGenerator::Trajectory& trajectory) {
    if (trajectory.count <= 0) {
      return 0.0f;
    }

    const TrajectoryGenerator::Point& a = trajectory.segments[0].a;
    const TrajectoryGenerator::Point& b = trajectory.segments[0].b;

    return atan2(b.y - a.y, b.x - a.x);
  }
}

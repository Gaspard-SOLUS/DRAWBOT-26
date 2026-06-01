#include <Arduino.h>
#include <math.h>

#include "trajectory_generator.h"

namespace TrajectoryGenerator {
  float degToRad(float deg) {
    return deg * PI / 180.0f;
  }

  float radToDeg(float rad) {
    return rad * 180.0f / PI;
  }

  float distance(Point a, Point b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy);
  }

  float headingDeg(Point a, Point b) {
    return radToDeg(atan2(b.y - a.y, b.x - a.x));
  }

  void clear(Trajectory& trajectory) {
    trajectory.count = 0;
  }

  bool addSegment(Trajectory& trajectory, Point a, Point b) {
    if (trajectory.count >= MAX_SEGMENTS) {
      return false;
    }

    if (distance(a, b) < 0.001f) {
      return true;
    }

    trajectory.segments[trajectory.count] = Segment(a, b);
    trajectory.count++;
    return true;
  }

  bool generateLine(Trajectory& trajectory, float distanceCm, float distanceScale) {
    clear(trajectory);

    float d = distanceCm * distanceScale;
    Point p0 {0.0f, 0.0f};
    Point p1 {d, 0.0f};

    return addSegment(trajectory, p0, p1);
  }

  bool generateOneAngle(Trajectory& trajectory, float d1Cm, float angleDeg, float d2Cm, float distanceScale) {
    clear(trajectory);

    float d1 = d1Cm * distanceScale;
    float d2 = d2Cm * distanceScale;

    Point p0 {0.0f, 0.0f};
    Point p1 {d1, 0.0f};

    float theta = degToRad(angleDeg);
    Point p2 {
      p1.x + d2 * cos(theta),
      p1.y + d2 * sin(theta)
    };

    return addSegment(trajectory, p0, p1) &&
           addSegment(trajectory, p1, p2);
  }

  bool generateStair(
    Trajectory& trajectory,
    float d1Cm,
    float angleLeftDeg,
    float d2Cm,
    float angleRightDeg,
    float d3Cm,
    float distanceScale
  ) {
    clear(trajectory);

    float d1 = d1Cm * distanceScale;
    float d2 = d2Cm * distanceScale;
    float d3 = d3Cm * distanceScale;

    Point p0 {0.0f, 0.0f};

    float theta = 0.0f;
    Point p1 {
      p0.x + d1 * cos(theta),
      p0.y + d1 * sin(theta)
    };

    theta += degToRad(angleLeftDeg);
    Point p2 {
      p1.x + d2 * cos(theta),
      p1.y + d2 * sin(theta)
    };

    theta -= degToRad(angleRightDeg);
    Point p3 {
      p2.x + d3 * cos(theta),
      p2.y + d3 * sin(theta)
    };

    return addSegment(trajectory, p0, p1) &&
           addSegment(trajectory, p1, p2) &&
           addSegment(trajectory, p2, p3);
  }

  bool generateCircle(
    Trajectory& trajectory,
    float radiusCm,
    int segmentCount,
    float centerXcm,
    float centerYcm,
    float startAngleDeg,
    bool clockwise,
    float distanceScale
  ) {
    clear(trajectory);

    if (radiusCm <= 0.0f) {
      return false;
    }

    int n = constrain(segmentCount, 8, MAX_SEGMENTS);
    float r = radiusCm * distanceScale;
    float sign = clockwise ? -1.0f : 1.0f;
    float start = degToRad(startAngleDeg);

    Point previous {
      centerXcm + r * cos(start),
      centerYcm + r * sin(start)
    };

    for (int i = 1; i <= n; i++) {
      float a = start + sign * 2.0f * PI * ((float)i / (float)n);
      Point current {
        centerXcm + r * cos(a),
        centerYcm + r * sin(a)
      };

      if (!addSegment(trajectory, previous, current)) {
        return false;
      }

      previous = current;
    }

    return true;
  }

  bool generateCompassRose(Trajectory& trajectory, float branchLengthCm, float northHeadingDeg, float distanceScale) {
    clear(trajectory);

    float length = branchLengthCm * distanceScale;
    if (length <= 0.0f) {
      return false;
    }

    Point center {0.0f, 0.0f};

    // Rose simple en 4 branches. northHeadingDeg permet d'orienter la branche Nord
    // selon la calibration magnétomètre. Les segments sont volontairement indépendants :
    // pour un vrai dessin sans lever le stylo, il faudra ensuite définir un chemin continu.
    for (int i = 0; i < 4; i++) {
      float angle = degToRad(northHeadingDeg + i * 90.0f);
      Point end {
        center.x + length * cos(angle),
        center.y + length * sin(angle)
      };

      if (!addSegment(trajectory, center, end)) {
        return false;
      }
    }

    return true;
  }
}

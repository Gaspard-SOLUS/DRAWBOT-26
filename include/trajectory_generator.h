#pragma once
#include <Arduino.h>

namespace TrajectoryGenerator {
  struct Point {
    float x;
    float y;

    Point() : x(0.0f), y(0.0f) {}
    Point(float xValue, float yValue) : x(xValue), y(yValue) {}
  };

  struct Segment {
    Point a;
    Point b;

    Segment() : a(), b() {}
    Segment(Point aValue, Point bValue) : a(aValue), b(bValue) {}
  };

  static const int MAX_SEGMENTS = 160;

  struct Trajectory {
    Segment segments[MAX_SEGMENTS];
    int count;

    Trajectory() : count(0) {}
  };

  void clear(Trajectory& trajectory);

  bool addSegment(Trajectory& trajectory, Point a, Point b);

  bool generateLine(Trajectory& trajectory, float distanceCm, float distanceScale = 1.0f);

  bool generateOneAngle(
    Trajectory& trajectory,
    float d1Cm,
    float angleDeg,
    float d2Cm,
    float distanceScale = 1.0f
  );

  bool generateStair(
    Trajectory& trajectory,
    float d1Cm,
    float angleLeftDeg,
    float d2Cm,
    float angleRightDeg,
    float d3Cm,
    float distanceScale = 1.0f
  );

  bool generateCircle(
    Trajectory& trajectory,
    float radiusCm,
    int segmentCount,
    float centerXcm = 0.0f,
    float centerYcm = 0.0f,
    float startAngleDeg = 0.0f,
    bool clockwise = false,
    float distanceScale = 1.0f
  );

  bool generateCompassRose(
    Trajectory& trajectory,
    float branchLengthCm,
    float northHeadingDeg,
    float distanceScale = 1.0f
  );

  float degToRad(float deg);
  float radToDeg(float rad);
  float distance(Point a, Point b);
  float headingDeg(Point a, Point b);
}

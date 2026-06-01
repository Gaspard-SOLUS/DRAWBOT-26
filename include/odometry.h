#pragma once
#include <Arduino.h>

namespace Odometry {
  void begin();
  void update(unsigned long now);
  void reset();

  // Réinitialise l'odométrie avec une position et une orientation imposées.
  // Utile pour la séquence cercle : le stylo démarre sur le cercle et le robot est placé tangentiellement.
  void resetPose(float xCm, float yCm, float thetaRad);

  float radToDeg(float rad);
  float normalizeAngleDeg(float angle);
}

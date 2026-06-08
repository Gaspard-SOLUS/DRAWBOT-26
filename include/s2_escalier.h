#pragma once

namespace S2Escalier {
  void setStepLength(float cm);   // longueur de chaque marche
  void setNbSteps(int n);         // nombre de marches
  float getStepLength();
  int getNbSteps();

  void run();   // exécute l'escalier complet
  void stop();
}

#include <Arduino.h>
#include "soutenance2.h"
#include "moteurs.h"
#include "logger.h"

namespace Soutenance2 {

void begin() {
  Logger::log("Soutenance2 initialisee");
}

void update(unsigned long now, float dt) {
  (void)now;
  (void)dt;
  // Fichier minimal : les sequences peuvent etre reliees ici ensuite.
}

}

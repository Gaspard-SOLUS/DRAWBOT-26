#include "soutenance2.h"
#include "logger.h"
#include "s2_escalier.h"

namespace Soutenance2 {
  void begin() {
    S2Escalier::begin();
    Logger::log("Module soutenance 2 pret");
  }

  void update(unsigned long now, float dt) {
    S2Escalier::update(now, dt);
  }
}

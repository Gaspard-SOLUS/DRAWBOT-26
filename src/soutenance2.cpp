#include "soutenance2.h"
#include "logger.h"

namespace Soutenance2 {
  void begin() {
    Logger::log("Module soutenance 2 pret");
  }

  void update(unsigned long now, float dt) {
    // Ici tu mettras plus tard les machines d'etats :
    // - sequence escalier
    // - cercle
    // - rose des vents
    // - PID
    // - commandes parametrees
  }
}
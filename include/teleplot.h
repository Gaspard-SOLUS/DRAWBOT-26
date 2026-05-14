#pragma once
#include <Arduino.h>

namespace Teleplot {
  void begin();
  void update(unsigned long now);
  void send(const char* name, float value);
}
#pragma once
#include <Arduino.h>

namespace Logger {
  void begin();
  void log(const String& message);
  void trace(const String& message);
  String getBuffer();
}

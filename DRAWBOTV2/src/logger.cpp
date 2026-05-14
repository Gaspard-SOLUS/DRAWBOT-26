#include "logger.h"

namespace Logger {
  static String buffer;

  void begin() {
    buffer = "";
  }

  void log(const String& message) {
    String line = "[" + String(millis() / 1000.0f, 2) + "s] " + message + "\n";
    Serial.print(line);

    buffer += line;

    if (buffer.length() > 6000) {
      buffer.remove(0, buffer.length() - 6000);
    }
  }

  String getBuffer() {
    return buffer;
  }
}
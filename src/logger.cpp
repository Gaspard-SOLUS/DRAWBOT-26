#include "logger.h"

namespace Logger {
  static String buffer;
  static const int BUFFER_LIMIT = 20000;

  void appendLine(const String& line) {
    Serial.print(line);

    buffer += line;

    if (buffer.length() > BUFFER_LIMIT) {
      buffer.remove(0, buffer.length() - BUFFER_LIMIT);
    }
  }

  void begin() {
    buffer = "";
  }

  void log(const String& message) {
    String line = "[" + String(millis() / 1000.0f, 2) + "s] " + message + "\n";
    appendLine(line);
  }

  void trace(const String& message) {
    appendLine(message + "\n");
  }

  String getBuffer() {
    return buffer;
  }
}

#include <Arduino.h>
#include <math.h>

#include "motor_calibration.h"
#include "moteurs.h"
#include "encodeurs.h"
#include "app_state.h"
#include "logger.h"

namespace {
  MotorCalibration::Result result;
  unsigned long startTimeMs = 0;

  int safePwm(int pwm) {
    return constrain(pwm, -255, 255);
  }

  float absNonZeroPwm(int pwm) {
    int a = abs(pwm);
    if (a < 1) return 1.0f;
    return (float)a;
  }

  void applyMotors(int left, int right) {
    motorState.pwmLeft = safePwm(left);
    motorState.pwmRight = safePwm(right);
    motorState.mode = "MOTOR_CALIBRATION";

    setMotors(motorState.pwmLeft, motorState.pwmRight);
  }

  void finalizeMeasurement(unsigned long now) {
    result.running = false;
    result.finished = true;

    result.elapsedMs = now - startTimeMs;

    result.endDistLeftCm = getLeftDistanceCm();
    result.endDistRightCm = getRightDistanceCm();

    result.deltaLeftCm = result.endDistLeftCm - result.startDistLeftCm;
    result.deltaRightCm = result.endDistRightCm - result.startDistRightCm;

    float elapsedS = result.elapsedMs / 1000.0f;
    if (elapsedS <= 0.001f) elapsedS = 0.001f;

    result.speedLeftCms = result.deltaLeftCm / elapsedS;
    result.speedRightCms = result.deltaRightCm / elapsedS;

    result.coefLeftCmsPerPwm = fabs(result.speedLeftCms) / absNonZeroPwm(result.pwmLeft);
    result.coefRightCmsPerPwm = fabs(result.speedRightCms) / absNonZeroPwm(result.pwmRight);

    stopMotors();

    motorState.pwmLeft = 0;
    motorState.pwmRight = 0;
    motorState.mode = "IDLE";

    Logger::log("Calibration moteurs terminee");
    Logger::log("L: delta=" + String(result.deltaLeftCm, 2) +
                " cm speed=" + String(result.speedLeftCms, 2) +
                " cm/s coef=" + String(result.coefLeftCmsPerPwm, 6));
    Logger::log("R: delta=" + String(result.deltaRightCm, 2) +
                " cm speed=" + String(result.speedRightCms, 2) +
                " cm/s coef=" + String(result.coefRightCmsPerPwm, 6));
  }
}

namespace MotorCalibration {
  void begin() {
    result = Result();
    startTimeMs = 0;
  }

  void start(const Config& config) {
    stopMotors();

    result = Result();

    result.running = true;
    result.finished = false;

    result.pwmLeft = safePwm(config.pwmLeft);
    result.pwmRight = safePwm(config.pwmRight);
    result.durationMs = config.durationMs;

    if (result.durationMs < 300) result.durationMs = 300;
    if (result.durationMs > 10000) result.durationMs = 10000;

    result.startDistLeftCm = getLeftDistanceCm();
    result.startDistRightCm = getRightDistanceCm();

    result.endDistLeftCm = result.startDistLeftCm;
    result.endDistRightCm = result.startDistRightCm;

    startTimeMs = millis();

    applyMotors(result.pwmLeft, result.pwmRight);

    Logger::log("Calibration moteurs lancee : pwmL=" + String(result.pwmLeft) +
                " pwmR=" + String(result.pwmRight) +
                " duree=" + String(result.durationMs) + " ms");
  }

  void stop() {
    if (result.running) {
      finalizeMeasurement(millis());
    } else {
      stopMotors();
      motorState.pwmLeft = 0;
      motorState.pwmRight = 0;
      motorState.mode = "IDLE";
    }
  }

  void update(unsigned long now) {
    if (!result.running) return;

    result.elapsedMs = now - startTimeMs;
    result.endDistLeftCm = getLeftDistanceCm();
    result.endDistRightCm = getRightDistanceCm();

    result.deltaLeftCm = result.endDistLeftCm - result.startDistLeftCm;
    result.deltaRightCm = result.endDistRightCm - result.startDistRightCm;

    if (result.elapsedMs >= result.durationMs) {
      finalizeMeasurement(now);
    }
  }

  bool isRunning() {
    return result.running;
  }

  bool isFinished() {
    return result.finished;
  }

  Result getResult() {
    return result;
  }

  String resultJson() {
    Result r = getResult();

    String json = "{";

    json += "\"running\":" + String(r.running ? "true" : "false") + ",";
    json += "\"finished\":" + String(r.finished ? "true" : "false") + ",";

    json += "\"pwmLeft\":" + String(r.pwmLeft) + ",";
    json += "\"pwmRight\":" + String(r.pwmRight) + ",";

    json += "\"durationMs\":" + String(r.durationMs) + ",";
    json += "\"elapsedMs\":" + String(r.elapsedMs) + ",";

    json += "\"startDistLeftCm\":" + String(r.startDistLeftCm, 4) + ",";
    json += "\"startDistRightCm\":" + String(r.startDistRightCm, 4) + ",";

    json += "\"endDistLeftCm\":" + String(r.endDistLeftCm, 4) + ",";
    json += "\"endDistRightCm\":" + String(r.endDistRightCm, 4) + ",";

    json += "\"deltaLeftCm\":" + String(r.deltaLeftCm, 4) + ",";
    json += "\"deltaRightCm\":" + String(r.deltaRightCm, 4) + ",";

    json += "\"speedLeftCms\":" + String(r.speedLeftCms, 4) + ",";
    json += "\"speedRightCms\":" + String(r.speedRightCms, 4) + ",";

    json += "\"coefLeftCmsPerPwm\":" + String(r.coefLeftCmsPerPwm, 6) + ",";
    json += "\"coefRightCmsPerPwm\":" + String(r.coefRightCmsPerPwm, 6);

    json += "}";

    return json;
  }
}

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <WiFiUdp.h>
#include <math.h>

#include "include/pins.h"
#include "include/moteurs.h"
#include "include/encodeurs.h"
#include "include/page_web.h"

// ==================================================
// WIFI POINT D'ACCES ESP32
// ==================================================
const char* DRAWBOT_AP_SSID = "DRAWBOT_#S4S25-G-2314";
const char* DRAWBOT_AP_PASS = "!12345678!";

WebServer server(80);

// ==================================================
// TELEPLOT UDP VERS PC
// ==================================================
const char* pcIP = "192.168.4.2";
const int teleplotPort = 47269;
WiFiUDP udp;

// ==================================================
// CONSTANTES ROBOT
// ==================================================
const float WHEEL_BASE_CM = 8.3f;
const float PEN_OFFSET_CM = 13.0f;
const unsigned long CONTROL_PERIOD_MS = 20;
const unsigned long STATUS_PERIOD_MS = 100;

// ==================================================
// LSM6DS3 - REGISTRES I2C
// ==================================================
const uint8_t LSM6DS3_ADDR = 0x6B;

const uint8_t REG_WHO_AM_I = 0x0F;
const uint8_t REG_CTRL1_XL = 0x10;
const uint8_t REG_CTRL2_G  = 0x11;
const uint8_t REG_CTRL3_C  = 0x12;

const uint8_t REG_OUTX_L_G  = 0x22;
const uint8_t REG_OUTX_L_XL = 0x28;

// ==================================================
// MAGNETOMETRE
// ==================================================
Adafruit_LIS3MDL lis3mdl;
bool magOk = false;

// Calibration magnétomètre
bool magCalibrationRunning = false;
bool magCalibrationDone = false;

unsigned long magCalibStart = 0;
const unsigned long MAG_CALIB_DURATION_MS = 20000;

float magMinX =  1e9;
float magMaxX = -1e9;
float magMinY =  1e9;
float magMaxY = -1e9;

float magOffsetX = 0.0f;
float magOffsetY = 0.0f;
float magScaleX = 1.0f;
float magScaleY = 1.0f;

// ==================================================
// DONNEES CAPTEURS
// ==================================================
bool imuOk = false;

float accX = 0.0f;
float accY = 0.0f;
float accZ = 0.0f;

float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;

float magX = 0.0f;
float magY = 0.0f;
float magZ = 0.0f;
float headingMag = 0.0f;

// ==================================================
// DONNEES ENCODEURS / ODOMETRIE
// ==================================================
float speedL = 0.0f;
float speedR = 0.0f;

float lastDistL = 0.0f;
float lastDistR = 0.0f;

float odoX = 0.0f;
float odoY = 0.0f;
float odoThetaRad = 0.0f;

float yawGyroDeg = 0.0f;

// ==================================================
// COMMANDE MOTEURS
// ==================================================
int currentPwmL = 0;
int currentPwmR = 0;

enum RobotMode {
  MODE_IDLE,
  MODE_MANUAL
};

RobotMode robotMode = MODE_IDLE;

// ==================================================
// TIMERS
// ==================================================
unsigned long lastControlUpdate = 0;
unsigned long lastStatusUpdate = 0;
unsigned long lastImuTime = 0;

// ==================================================
// CONSOLE INTERNE
// ==================================================
String consoleBuffer = "";

void pushLog(const String& msg) {
  String line = "[" + String(millis() / 1000.0f, 2) + "s] " + msg + "\n";
  Serial.print(line);

  consoleBuffer += line;

  if (consoleBuffer.length() > 6000) {
    consoleBuffer.remove(0, consoleBuffer.length() - 6000);
  }
}

// ==================================================
// OUTILS
// ==================================================
float radToDeg(float rad) {
  return rad * 180.0f / PI;
}

float normalizeAngleDeg(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

String modeName() {
  switch (robotMode) {
    case MODE_IDLE: return "IDLE";
    case MODE_MANUAL: return "MANUAL";
  }
  return "UNKNOWN";
}

String stateName() {
  if (currentPwmL == 0 && currentPwmR == 0) {
    return "STOP";
  }

  if (currentPwmL > 0 && currentPwmR > 0) {
    return "AVANCE";
  }

  if (currentPwmL < 0 && currentPwmR < 0) {
    return "RECULE";
  }

  if (currentPwmL < 0 && currentPwmR > 0) {
    return "ROTATION_GAUCHE";
  }

  if (currentPwmL > 0 && currentPwmR < 0) {
    return "ROTATION_DROITE";
  }

  return "MANUEL";
}

void setRobotMotors(int leftPwm, int rightPwm) {
  currentPwmL = constrain(leftPwm, -255, 255);
  currentPwmR = constrain(rightPwm, -255, 255);

  setMotors(currentPwmL, currentPwmR);
}

void stopRobot() {
  currentPwmL = 0;
  currentPwmR = 0;
  stopMotors();
  robotMode = MODE_IDLE;
}

// ==================================================
// I2C LSM6DS3
// ==================================================
void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(address, (uint8_t)1);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}

int16_t readInt16(uint8_t address, uint8_t regLow) {
  Wire.beginTransmission(address);
  Wire.write(regLow);
  Wire.endTransmission(false);

  Wire.requestFrom(address, (uint8_t)2);

  uint8_t low = 0;
  uint8_t high = 0;

  if (Wire.available()) low = Wire.read();
  if (Wire.available()) high = Wire.read();

  return (int16_t)((high << 8) | low);
}

bool initIMU() {
  uint8_t whoami = readRegister(LSM6DS3_ADDR, REG_WHO_AM_I);

  if (whoami != 0x69 && whoami != 0x6A) {
    pushLog("Erreur : LSM6DS3 non detecte. WHO_AM_I = 0x" + String(whoami, HEX));
    return false;
  }

  // CTRL3_C : BDU = 1, IF_INC = 1
  writeRegister(LSM6DS3_ADDR, REG_CTRL3_C, 0b01000100);

  // Accelerometre : 104 Hz, ±2g
  writeRegister(LSM6DS3_ADDR, REG_CTRL1_XL, 0b01000000);

  // Gyroscope : 104 Hz, 245 dps
  writeRegister(LSM6DS3_ADDR, REG_CTRL2_G, 0b01000000);

  pushLog("LSM6DS3 detecte et initialise");
  return true;
}

void readIMU() {
  if (!imuOk) return;

  int16_t gxRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_G);
  int16_t gyRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_G + 2);
  int16_t gzRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_G + 4);

  int16_t axRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_XL);
  int16_t ayRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_XL + 2);
  int16_t azRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_XL + 4);

  // ±2g : 0.061 mg/LSB = 0.000061 g/LSB
  accX = axRaw * 0.000061f;
  accY = ayRaw * 0.000061f;
  accZ = azRaw * 0.000061f;

  // 245 dps : 8.75 mdps/LSB = 0.00875 °/s
  gyroX = gxRaw * 0.00875f;
  gyroY = gyRaw * 0.00875f;
  gyroZ = gzRaw * 0.00875f;
}

void updateGyroYaw(unsigned long now) {
  if (!imuOk) return;

  if (lastImuTime == 0) {
    lastImuTime = now;
    return;
  }

  float dt = (now - lastImuTime) / 1000.0f;
  lastImuTime = now;

  yawGyroDeg += gyroZ * dt;
  yawGyroDeg = normalizeAngleDeg(yawGyroDeg);
}

// ==================================================
// MAGNETOMETRE
// ==================================================
void initMagnetometer() {
  magOk = lis3mdl.begin_I2C(0x1E);

  if (!magOk) {
    pushLog("Erreur : LIS3MDL non detecte");
    return;
  }

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  pushLog("LIS3MDL detecte et initialise");
}

void startMagCalibration() {
  magCalibrationRunning = true;
  magCalibrationDone = false;
  magCalibStart = millis();

  magMinX =  1e9;
  magMaxX = -1e9;
  magMinY =  1e9;
  magMaxY = -1e9;

  pushLog("Calibration magnetometre demarree pour 20 secondes");
  pushLog("Tourner le robot a plat sur lui-meme pendant la calibration");
}

void readMagnetometer() {
  if (!magOk) return;

  sensors_event_t event;
  lis3mdl.getEvent(&event);

  float rawX = event.magnetic.x;
  float rawY = event.magnetic.y;
  float rawZ = event.magnetic.z;

  if (magCalibrationRunning) {
    if (rawX < magMinX) magMinX = rawX;
    if (rawX > magMaxX) magMaxX = rawX;
    if (rawY < magMinY) magMinY = rawY;
    if (rawY > magMaxY) magMaxY = rawY;

    if (millis() - magCalibStart >= MAG_CALIB_DURATION_MS) {
      magOffsetX = (magMaxX + magMinX) * 0.5f;
      magOffsetY = (magMaxY + magMinY) * 0.5f;

      float radiusX = (magMaxX - magMinX) * 0.5f;
      float radiusY = (magMaxY - magMinY) * 0.5f;
      float avgRadius = (radiusX + radiusY) * 0.5f;

      if (radiusX > 0.001f) magScaleX = avgRadius / radiusX;
      if (radiusY > 0.001f) magScaleY = avgRadius / radiusY;

      magCalibrationRunning = false;
      magCalibrationDone = true;

      pushLog("Calibration magnetometre terminee");
      pushLog("offsetX=" + String(magOffsetX, 2) + " offsetY=" + String(magOffsetY, 2));
      pushLog("scaleX=" + String(magScaleX, 4) + " scaleY=" + String(magScaleY, 4));
    }
  }

  magX = (rawX - magOffsetX) * magScaleX;
  magY = (rawY - magOffsetY) * magScaleY;
  magZ = rawZ;

  headingMag = atan2(-magX, -magY) * 180.0f / PI;
  headingMag = normalizeAngleDeg(headingMag);
}

// ==================================================
// ENCODEURS / VITESSE / ODOMETRIE
// ==================================================
void updateEncodersAndOdometry(unsigned long now) {
  static unsigned long lastOdoTime = 0;

  updateEncoderMeasurements(now);

  if (lastOdoTime == 0) {
    lastOdoTime = now;
    lastDistL = getLeftDistanceCm();
    lastDistR = getRightDistanceCm();
    return;
  }

  float dt = (now - lastOdoTime) / 1000.0f;

  if (dt <= 0.0f) return;

  float currentDistL = getLeftDistanceCm();
  float currentDistR = getRightDistanceCm();

  float dL = currentDistL - lastDistL;
  float dR = currentDistR - lastDistR;

  speedL = dL / dt;
  speedR = dR / dt;

  float dCenter = (dL + dR) * 0.5f;
  float dTheta = (dR - dL) / WHEEL_BASE_CM;

  odoThetaRad += dTheta;

  float thetaMid = odoThetaRad - dTheta * 0.5f;

  odoX += dCenter * cos(thetaMid);
  odoY += dCenter * sin(thetaMid);

  lastDistL = currentDistL;
  lastDistR = currentDistR;
  lastOdoTime = now;
}

void resetOdometry() {
  resetEncoders();

  lastDistL = 0.0f;
  lastDistR = 0.0f;

  speedL = 0.0f;
  speedR = 0.0f;

  odoX = 0.0f;
  odoY = 0.0f;
  odoThetaRad = 0.0f;

  pushLog("Encodeurs et odometrie remis a zero");
}

void resetIMUOrientation() {
  yawGyroDeg = 0.0f;
  lastImuTime = 0;

  pushLog("Orientation IMU remise a zero");
}

// ==================================================
// API WEB
// ==================================================
void handleRoot() {
  server.send_P(200, "text/html", WEB_PAGE);
}

void handleStatus() {
  String magCalibState = "NON";
  if (magCalibrationRunning) magCalibState = "EN COURS";
  if (magCalibrationDone) magCalibState = "OK";

  String json = "{";

  json += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  json += "\"uptime\":" + String(millis() / 1000.0f, 2) + ",";
  json += "\"state\":\"" + stateName() + "\",";
  json += "\"mode\":\"" + modeName() + "\",";

  json += "\"pwmL\":" + String(currentPwmL) + ",";
  json += "\"pwmR\":" + String(currentPwmR) + ",";

  json += "\"accX\":" + String(accX, 4) + ",";
  json += "\"accY\":" + String(accY, 4) + ",";
  json += "\"accZ\":" + String(accZ, 4) + ",";

  json += "\"gyroX\":" + String(gyroX, 3) + ",";
  json += "\"gyroY\":" + String(gyroY, 3) + ",";
  json += "\"gyroZ\":" + String(gyroZ, 3) + ",";

  json += "\"magX\":" + String(magX, 3) + ",";
  json += "\"magY\":" + String(magY, 3) + ",";
  json += "\"magZ\":" + String(magZ, 3) + ",";
  json += "\"headingMag\":" + String(headingMag, 3) + ",";
  json += "\"magCalib\":\"" + magCalibState + "\",";

  json += "\"ticksL\":" + String(getLeftEncoderTicks()) + ",";
  json += "\"ticksR\":" + String(getRightEncoderTicks()) + ",";

  json += "\"distL\":" + String(getLeftDistanceCm(), 3) + ",";
  json += "\"distR\":" + String(getRightDistanceCm(), 3) + ",";

  json += "\"speedL\":" + String(speedL, 3) + ",";
  json += "\"speedR\":" + String(speedR, 3) + ",";

  json += "\"odoX\":" + String(odoX, 3) + ",";
  json += "\"odoY\":" + String(odoY, 3) + ",";
  json += "\"odoTheta\":" + String(normalizeAngleDeg(radToDeg(odoThetaRad)), 3) + ",";

  json += "\"yawGyro\":" + String(yawGyroDeg, 3);

  json += "}";

  server.send(200, "application/json", json);
}

void handleLogs() {
  server.send(200, "text/plain", consoleBuffer);
}

void handleManual() {
  int l = server.hasArg("l") ? server.arg("l").toInt() : 0;
  int r = server.hasArg("r") ? server.arg("r").toInt() : 0;

  robotMode = MODE_MANUAL;
  setRobotMotors(l, r);

  pushLog("Commande manuelle : L=" + String(currentPwmL) + " R=" + String(currentPwmR));
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  stopRobot();
  pushLog("STOP moteurs");
  server.send(200, "text/plain", "STOP");
}

void handleResetEncoders() {
  stopRobot();
  resetOdometry();
  server.send(200, "text/plain", "RESET_ENCODERS");
}

void handleResetIMU() {
  resetIMUOrientation();
  server.send(200, "text/plain", "RESET_IMU");
}

void handleCalibrateMag() {
  startMagCalibration();
  server.send(200, "text/plain", "CALIB_MAG");
}

// ==================================================
// SEND TELEPLOT
// ==================================================
void sendTeleplot(const char* name, float value) {
  udp.beginPacket(pcIP, teleplotPort);
  udp.printf("%s:%f|g\n", name, value);
  udp.endPacket();
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/logs", handleLogs);
  server.on("/api/manual", handleManual);
  server.on("/api/stop", handleStop);
  server.on("/api/reset-encoders", handleResetEncoders);
  server.on("/api/reset-imu", handleResetIMU);
  server.on("/api/calibrate-mag", handleCalibrateMag);

  server.begin();
  pushLog("Serveur web lance sur le port 80");
}

void sendTeleplotData() {
  sendTeleplot("pwmL", currentPwmL);
  sendTeleplot("pwmR", currentPwmR);

  sendTeleplot("ticksL", getLeftEncoderTicks());
  sendTeleplot("ticksR", getRightEncoderTicks());

  sendTeleplot("distL_cm", getLeftDistanceCm());
  sendTeleplot("distR_cm", getRightDistanceCm());

  sendTeleplot("speedL_cm_s", speedL);
  sendTeleplot("speedR_cm_s", speedR);

  sendTeleplot("accX_g", accX);
  sendTeleplot("accY_g", accY);
  sendTeleplot("accZ_g", accZ);

  sendTeleplot("gyroZ_dps", gyroZ);
  sendTeleplot("yawGyro_deg", yawGyroDeg);

  sendTeleplot("magX_uT", magX);
  sendTeleplot("magY_uT", magY);
  sendTeleplot("headingMag_deg", headingMag);

  sendTeleplot("odoX_cm", odoX);
  sendTeleplot("odoY_cm", odoY);
  sendTeleplot("odoTheta_deg", normalizeAngleDeg(radToDeg(odoThetaRad)));
}

// ==================================================
// SETUP
// ==================================================
void setup() {
  Serial.begin(115200);
  delay(800);

  pushLog("=== DRAWBOT - SOUTENANCE 1 ===");

  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  initMotors();
  initEncoders();
  resetEncoders();
  stopRobot();

  Wire.begin(21, 22);

  imuOk = initIMU();
  initMagnetometer();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(DRAWBOT_AP_SSID, DRAWBOT_AP_PASS);

  pushLog("WiFi AP demarre");
  pushLog("SSID : " + String(DRAWBOT_AP_SSID));
  pushLog("Mot de passe : " + String(DRAWBOT_AP_PASS));
  pushLog("Adresse web : http://" + WiFi.softAPIP().toString());

  setupWebServer();

  udp.begin(teleplotPort);
  pushLog("UDP Teleplot initialise vers " + String(pcIP) + ":" + String(teleplotPort));

  lastControlUpdate = millis();
  lastStatusUpdate = millis();
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  unsigned long now = millis();

  server.handleClient();

  static unsigned long lastTeleplot = 0;

  if (now - lastTeleplot >= 100) {
    lastTeleplot = now;
    sendTeleplotData();
  }

  if (now - lastControlUpdate >= CONTROL_PERIOD_MS) {
    lastControlUpdate = now;

    readIMU();
    updateGyroYaw(now);
    readMagnetometer();
    updateEncodersAndOdometry(now);
  }

  // LED
  static unsigned long lastLedToggle = 0;
  static bool ledState = false;

  if (now - lastLedToggle >= 500) {
    lastLedToggle = now;
    ledState = !ledState;
    digitalWrite(LEDU1, ledState);
  }
}
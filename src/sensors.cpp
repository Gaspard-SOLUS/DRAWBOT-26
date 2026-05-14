#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#include "sensors.h"
#include "app_state.h"
#include "logger.h"

// ==================================================
// LSM6DS3 - REGISTRES I2C
// ==================================================
static const uint8_t LSM6DS3_ADDR = 0x6B;

static const uint8_t REG_WHO_AM_I = 0x0F;
static const uint8_t REG_CTRL1_XL = 0x10;
static const uint8_t REG_CTRL2_G  = 0x11;
static const uint8_t REG_CTRL3_C  = 0x12;

static const uint8_t REG_OUTX_L_G  = 0x22;
static const uint8_t REG_OUTX_L_XL = 0x28;

// ==================================================
// MAGNETOMETRE LIS3MDL
// ==================================================
static Adafruit_LIS3MDL lis3mdl;

// Calibration magnétomètre
static unsigned long magCalibStart = 0;
static const unsigned long MAG_CALIB_DURATION_MS = 20000;

static float magMinX =  1e9;
static float magMaxX = -1e9;
static float magMinY =  1e9;
static float magMaxY = -1e9;

static float magOffsetX = 0.0f;
static float magOffsetY = 0.0f;

static float magScaleX = 1.0f;
static float magScaleY = 1.0f;

// Gyroscope intégré
static unsigned long lastImuTime = 0;

// ==================================================
// OUTILS I2C BAS NIVEAU POUR LE LSM6DS3
// ==================================================
static void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static uint8_t readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(address, (uint8_t)1);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}

static int16_t readInt16(uint8_t address, uint8_t regLow) {
  Wire.beginTransmission(address);
  Wire.write(regLow);
  Wire.endTransmission(false);

  Wire.requestFrom(address, (uint8_t)2);

  uint8_t low = 0;
  uint8_t high = 0;

  if (Wire.available()) {
    low = Wire.read();
  }

  if (Wire.available()) {
    high = Wire.read();
  }

  return (int16_t)((high << 8) | low);
}

// ==================================================
// FONCTIONS INTERNES IMU
// ==================================================
static bool initIMU() {
  uint8_t whoami = readRegister(LSM6DS3_ADDR, REG_WHO_AM_I);

  if (whoami != 0x69 && whoami != 0x6A) {
    Logger::log("Erreur : LSM6DS3 non detecte. WHO_AM_I = 0x" + String(whoami, HEX));
    return false;
  }

  // CTRL3_C :
  // BDU = 1 : bloque la mise à jour des registres pendant la lecture
  // IF_INC = 1 : incrément automatique de l'adresse des registres
  writeRegister(LSM6DS3_ADDR, REG_CTRL3_C, 0b01000100);

  // Accelerometre :
  // ODR = 104 Hz
  // FS = ±2g
  writeRegister(LSM6DS3_ADDR, REG_CTRL1_XL, 0b01000000);

  // Gyroscope :
  // ODR = 104 Hz
  // FS = 245 dps
  writeRegister(LSM6DS3_ADDR, REG_CTRL2_G, 0b01000000);

  Logger::log("LSM6DS3 detecte et initialise");
  return true;
}

static void readIMU() {
  if (!sensorState.imuOk) {
    return;
  }

  int16_t gxRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_G);
  int16_t gyRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_G + 2);
  int16_t gzRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_G + 4);

  int16_t axRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_XL);
  int16_t ayRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_XL + 2);
  int16_t azRaw = readInt16(LSM6DS3_ADDR, REG_OUTX_L_XL + 4);

  // Accelerometre ±2g :
  // Sensibilité = 0.061 mg/LSB = 0.000061 g/LSB
  sensorState.accX = axRaw * 0.000061f;
  sensorState.accY = ayRaw * 0.000061f;
  sensorState.accZ = azRaw * 0.000061f;

  // Gyroscope 245 dps :
  // Sensibilité = 8.75 mdps/LSB = 0.00875 °/s/LSB
  sensorState.gyroX = gxRaw * 0.00875f;
  sensorState.gyroY = gyRaw * 0.00875f;
  sensorState.gyroZ = gzRaw * 0.00875f;
}

static void updateGyroYaw(unsigned long now) {
  if (!sensorState.imuOk) {
    return;
  }

  if (lastImuTime == 0) {
    lastImuTime = now;
    return;
  }

  float dt = (now - lastImuTime) / 1000.0f;
  lastImuTime = now;

  sensorState.yawGyroDeg += sensorState.gyroZ * dt;
  sensorState.yawGyroDeg = Sensors::normalizeAngleDeg(sensorState.yawGyroDeg);
}

// ==================================================
// FONCTIONS INTERNES MAGNETOMETRE
// ==================================================
static void initMagnetometer() {
  sensorState.magOk = lis3mdl.begin_I2C(0x1E);

  if (!sensorState.magOk) {
    Logger::log("Erreur : LIS3MDL non detecte");
    return;
  }

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  Logger::log("LIS3MDL detecte et initialise");
}

static void readMagnetometer() {
  if (!sensorState.magOk) {
    return;
  }

  sensors_event_t event;
  lis3mdl.getEvent(&event);

  float rawX = event.magnetic.x;
  float rawY = event.magnetic.y;
  float rawZ = event.magnetic.z;

  if (sensorState.magCalibrationRunning) {
    if (rawX < magMinX) {
      magMinX = rawX;
    }

    if (rawX > magMaxX) {
      magMaxX = rawX;
    }

    if (rawY < magMinY) {
      magMinY = rawY;
    }

    if (rawY > magMaxY) {
      magMaxY = rawY;
    }

    if (millis() - magCalibStart >= MAG_CALIB_DURATION_MS) {
      magOffsetX = (magMaxX + magMinX) * 0.5f;
      magOffsetY = (magMaxY + magMinY) * 0.5f;

      float radiusX = (magMaxX - magMinX) * 0.5f;
      float radiusY = (magMaxY - magMinY) * 0.5f;
      float avgRadius = (radiusX + radiusY) * 0.5f;

      if (radiusX > 0.001f) {
        magScaleX = avgRadius / radiusX;
      }

      if (radiusY > 0.001f) {
        magScaleY = avgRadius / radiusY;
      }

      sensorState.magCalibrationRunning = false;
      sensorState.magCalibrationDone = true;

      Logger::log("Calibration magnetometre terminee");
      Logger::log("offsetX=" + String(magOffsetX, 2) + " offsetY=" + String(magOffsetY, 2));
      Logger::log("scaleX=" + String(magScaleX, 4) + " scaleY=" + String(magScaleY, 4));
    }
  }

  sensorState.magX = (rawX - magOffsetX) * magScaleX;
  sensorState.magY = (rawY - magOffsetY) * magScaleY;
  sensorState.magZ = rawZ;

  sensorState.headingMagDeg = atan2(-sensorState.magX, -sensorState.magY) * 180.0f / PI;
  sensorState.headingMagDeg = Sensors::normalizeAngleDeg(sensorState.headingMagDeg);
}

// ==================================================
// API PUBLIQUE DU MODULE SENSORS
// ==================================================
namespace Sensors {

  void begin() {
    sensorState.imuOk = initIMU();
    initMagnetometer();

    sensorState.yawGyroDeg = 0.0f;
    lastImuTime = 0;

    sensorState.magCalibrationRunning = false;
    sensorState.magCalibrationDone = false;

    Logger::log("Module capteurs initialise");
  }

  void update(unsigned long now, float dt) {
    (void)dt;

    readIMU();
    updateGyroYaw(now);
    readMagnetometer();
  }

  void resetGyroYaw() {
    sensorState.yawGyroDeg = 0.0f;
    lastImuTime = 0;

    Logger::log("Orientation IMU remise a zero");
  }

  void startMagCalibration() {
    if (!sensorState.magOk) {
      Logger::log("Impossible de calibrer : LIS3MDL non detecte");
      return;
    }

    sensorState.magCalibrationRunning = true;
    sensorState.magCalibrationDone = false;

    magCalibStart = millis();

    magMinX =  1e9;
    magMaxX = -1e9;
    magMinY =  1e9;
    magMaxY = -1e9;

    Logger::log("Calibration magnetometre demarree pour 20 secondes");
    Logger::log("Tourner le robot a plat sur lui-meme pendant la calibration");
  }

  float normalizeAngleDeg(float angle) {
    while (angle < 0.0f) {
      angle += 360.0f;
    }

    while (angle >= 360.0f) {
      angle -= 360.0f;
    }

    return angle;
  }
}
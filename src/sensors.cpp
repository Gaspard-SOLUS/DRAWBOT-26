#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
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

// ==================================================
// MEMOIRE FLASH POUR CALIBRATION
// ==================================================
static Preferences magPrefs;
static const char* MAG_PREF_NAMESPACE = "mag_calib";

// ==================================================
// CALIBRATION MAGNETOMETRE
// ==================================================
static unsigned long magCalibStart = 0;
static const unsigned long MAG_CALIB_DURATION_MS = 20000;

static float magMinX =  1e9;
static float magMaxX = -1e9;
static float magMinY =  1e9;
static float magMaxY = -1e9;

static bool magHasReferenceHeading = false;
static float magReferenceHeadingDeg = 0.0f;

static const float MAG_HEADING_CHANGE_EPS_DEG = 0.6f;
static const float MAG_CALIB_MIN_RANGE_UT = 5.0f;

// ==================================================
// GYROSCOPE INTEGRE
// ==================================================
static unsigned long lastImuTime = 0;

// ==================================================
// OUTILS I2C BAS NIVEAU POUR LSM6DS3
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

static float angleDistanceDeg(float a, float b) {
  float diff = fabs(Sensors::normalizeAngleDeg(a) - Sensors::normalizeAngleDeg(b));
  if (diff > 180.0f) {
    diff = 360.0f - diff;
  }

  return diff;
}

// ==================================================
// SAUVEGARDE / CHARGEMENT CALIBRATION MAGNETOMETRE
// ==================================================
static void loadMagCalibration() {
  magPrefs.begin(MAG_PREF_NAMESPACE, true);

  bool valid = magPrefs.getBool("valid", false);

  if (valid) {
    sensorState.magOffsetX = magPrefs.getFloat("offsetX", 0.0f);
    sensorState.magOffsetY = magPrefs.getFloat("offsetY", 0.0f);
    sensorState.magScaleX = magPrefs.getFloat("scaleX", 1.0f);
    sensorState.magScaleY = magPrefs.getFloat("scaleY", 1.0f);

    sensorState.magCalibrationLoaded = true;
    sensorState.magCalibrationDone = true;

    Logger::log("Calibration magnetometre chargee depuis la memoire flash");
    Logger::log("offsetX=" + String(sensorState.magOffsetX, 2) +
                " offsetY=" + String(sensorState.magOffsetY, 2));
    Logger::log("scaleX=" + String(sensorState.magScaleX, 4) +
                " scaleY=" + String(sensorState.magScaleY, 4));
  } else {
    sensorState.magOffsetX = 0.0f;
    sensorState.magOffsetY = 0.0f;
    sensorState.magScaleX = 1.0f;
    sensorState.magScaleY = 1.0f;

    sensorState.magCalibrationLoaded = false;
    sensorState.magCalibrationDone = false;

    Logger::log("Aucune calibration magnetometre sauvegardee");
  }

  magPrefs.end();
}

static void saveMagCalibration() {
  magPrefs.begin(MAG_PREF_NAMESPACE, false);

  magPrefs.putBool("valid", true);
  magPrefs.putFloat("offsetX", sensorState.magOffsetX);
  magPrefs.putFloat("offsetY", sensorState.magOffsetY);
  magPrefs.putFloat("scaleX", sensorState.magScaleX);
  magPrefs.putFloat("scaleY", sensorState.magScaleY);

  magPrefs.end();

  sensorState.magCalibrationLoaded = true;
  sensorState.magCalibrationDone = true;

  Logger::log("Calibration magnetometre enregistree en memoire flash");
}

static void eraseMagCalibration() {
  magPrefs.begin(MAG_PREF_NAMESPACE, false);
  magPrefs.clear();
  magPrefs.end();

  sensorState.magOffsetX = 0.0f;
  sensorState.magOffsetY = 0.0f;
  sensorState.magScaleX = 1.0f;
  sensorState.magScaleY = 1.0f;

  sensorState.magCalibrationLoaded = false;
  sensorState.magCalibrationDone = false;
  sensorState.magCalibrationRunning = false;

  Logger::log("Calibration magnetometre effacee");
}

// ==================================================
// IMU LSM6DS3
// ==================================================
static bool initIMU() {
  uint8_t whoami = readRegister(LSM6DS3_ADDR, REG_WHO_AM_I);

  if (whoami != 0x69 && whoami != 0x6A) {
    Logger::log("Erreur : LSM6DS3 non detecte. WHO_AM_I = 0x" + String(whoami, HEX));
    return false;
  }

  // BDU = 1, IF_INC = 1
  writeRegister(LSM6DS3_ADDR, REG_CTRL3_C, 0b01000100);

  // Accelerometre : 104 Hz, ±2g
  writeRegister(LSM6DS3_ADDR, REG_CTRL1_XL, 0b01000000);

  // Gyroscope : 104 Hz, 245 dps
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

  // ±2g : 0.061 mg/LSB = 0.000061 g/LSB
  sensorState.accX = axRaw * 0.000061f;
  sensorState.accY = ayRaw * 0.000061f;
  sensorState.accZ = azRaw * 0.000061f;

  // 245 dps : 8.75 mdps/LSB = 0.00875 °/s/LSB
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
// MAGNETOMETRE LIS3MDL
// ==================================================
static void initMagnetometer() {
  static const uint8_t addresses[] = {0x1C, 0x1E};

  sensorState.magOk = false;
  sensorState.magAddress = 0;

  for (uint8_t address : addresses) {
    if (lis3mdl.begin_I2C(address, &Wire)) {
      sensorState.magOk = true;
      sensorState.magAddress = address;
      break;
    }
  }

  if (!sensorState.magOk) {
    Logger::log("Erreur : LIS3MDL non detecte");
    return;
  }

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  Logger::log("LIS3MDL detecte et initialise a l'adresse 0x" +
              String(sensorState.magAddress, HEX));

  loadMagCalibration();
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
  unsigned long now = millis();

  sensorState.magRawX = rawX;
  sensorState.magRawY = rawY;
  sensorState.magRawZ = rawZ;
  sensorState.magReadCount++;
  sensorState.magLastReadMs = now;

  if (sensorState.magCalibrationRunning) {
    if (rawX < magMinX) magMinX = rawX;
    if (rawX > magMaxX) magMaxX = rawX;
    if (rawY < magMinY) magMinY = rawY;
    if (rawY > magMaxY) magMaxY = rawY;

    sensorState.magCalibrationRangeX = magMaxX - magMinX;
    sensorState.magCalibrationRangeY = magMaxY - magMinY;

    if (now - magCalibStart >= MAG_CALIB_DURATION_MS) {
      if (sensorState.magCalibrationRangeX < MAG_CALIB_MIN_RANGE_UT ||
          sensorState.magCalibrationRangeY < MAG_CALIB_MIN_RANGE_UT) {
        sensorState.magCalibrationRunning = false;
        sensorState.magCalibrationDone = false;

        Logger::log("Calibration magnetometre invalide : variation trop faible");
        Logger::log("rangeX=" + String(sensorState.magCalibrationRangeX, 2) +
                    " uT rangeY=" + String(sensorState.magCalibrationRangeY, 2) +
                    " uT");
        return;
      }

      sensorState.magOffsetX = (magMaxX + magMinX) * 0.5f;
      sensorState.magOffsetY = (magMaxY + magMinY) * 0.5f;

      float radiusX = (magMaxX - magMinX) * 0.5f;
      float radiusY = (magMaxY - magMinY) * 0.5f;
      float avgRadius = (radiusX + radiusY) * 0.5f;

      if (radiusX > 0.001f) {
        sensorState.magScaleX = avgRadius / radiusX;
      }

      if (radiusY > 0.001f) {
        sensorState.magScaleY = avgRadius / radiusY;
      }

      sensorState.magCalibrationRunning = false;
      sensorState.magCalibrationDone = true;

      Logger::log("Calibration magnetometre terminee");
      Logger::log("offsetX=" + String(sensorState.magOffsetX, 2) +
                  " offsetY=" + String(sensorState.magOffsetY, 2));
      Logger::log("scaleX=" + String(sensorState.magScaleX, 4) +
                  " scaleY=" + String(sensorState.magScaleY, 4));

      saveMagCalibration();
    }
  }

  sensorState.magX = (rawX - sensorState.magOffsetX) * sensorState.magScaleX;
  sensorState.magY = (rawY - sensorState.magOffsetY) * sensorState.magScaleY;
  sensorState.magZ = rawZ;

  sensorState.headingMagDeg = atan2(-sensorState.magX, -sensorState.magY) * 180.0f / PI;
  sensorState.headingMagDeg = Sensors::normalizeAngleDeg(sensorState.headingMagDeg);

  if (!magHasReferenceHeading) {
    magHasReferenceHeading = true;
    magReferenceHeadingDeg = sensorState.headingMagDeg;
    sensorState.magLastHeadingChangeMs = now;
    sensorState.magHeadingDeltaDeg = 0.0f;
  } else {
    sensorState.magHeadingDeltaDeg = angleDistanceDeg(sensorState.headingMagDeg, magReferenceHeadingDeg);
    if (sensorState.magHeadingDeltaDeg >= MAG_HEADING_CHANGE_EPS_DEG) {
      magReferenceHeadingDeg = sensorState.headingMagDeg;
      sensorState.magLastHeadingChangeMs = now;
    }
  }
}

// ==================================================
// API PUBLIQUE
// ==================================================
namespace Sensors {

  void begin() {
    sensorState.imuOk = initIMU();
    initMagnetometer();

    sensorState.yawGyroDeg = 0.0f;
    lastImuTime = 0;

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
    sensorState.magCalibrationLoaded = false;

    magCalibStart = millis();

    magMinX =  1e9;
    magMaxX = -1e9;
    magMinY =  1e9;
    magMaxY = -1e9;

    sensorState.magCalibrationRangeX = 0.0f;
    sensorState.magCalibrationRangeY = 0.0f;

    Logger::log("Calibration magnetometre demarree pour 20 secondes");
    Logger::log("Tourner le robot a plat sur lui-meme pendant la calibration");
  }

  void clearMagCalibration() {
    eraseMagCalibration();
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

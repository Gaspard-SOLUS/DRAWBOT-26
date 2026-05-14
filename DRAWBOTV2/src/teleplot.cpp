#include <Arduino.h>
#include <WiFiUdp.h>

#include "teleplot.h"
#include "app_state.h"
#include "encodeurs.h"
#include "odometry.h"
#include "logger.h"

static const char* TELEPLOT_PC_IP = "192.168.4.2";
static const int TELEPLOT_PORT = 47269;

static WiFiUDP udp;
static unsigned long lastTeleplot = 0;
static const unsigned long TELEPLOT_PERIOD_MS = 100;

namespace Teleplot {
  void begin() {
    udp.begin(TELEPLOT_PORT);
    Logger::log("Teleplot actif vers " + String(TELEPLOT_PC_IP) + ":" + String(TELEPLOT_PORT));
  }

  void send(const char* name, float value) {
    udp.beginPacket(TELEPLOT_PC_IP, TELEPLOT_PORT);
    udp.printf("%s:%f|g\n", name, value);
    udp.endPacket();
  }

  void update(unsigned long now) {
    if (now - lastTeleplot < TELEPLOT_PERIOD_MS) return;
    lastTeleplot = now;

    send("cmd_pwm_left", motorState.pwmLeft);
    send("cmd_pwm_right", motorState.pwmRight);

    send("enc_ticks_left", getLeftEncoderTicks());
    send("enc_ticks_right", getRightEncoderTicks());

    send("wheel_dist_left_cm", getLeftDistanceCm());
    send("wheel_dist_right_cm", getRightDistanceCm());

    send("wheel_speed_left_cms", odometryState.speedLeftCms);
    send("wheel_speed_right_cms", odometryState.speedRightCms);

    send("imu_acc_x_g", sensorState.accX);
    send("imu_acc_y_g", sensorState.accY);
    send("imu_acc_z_g", sensorState.accZ);

    send("imu_gyro_x_dps", sensorState.gyroX);
    send("imu_gyro_y_dps", sensorState.gyroY);
    send("imu_gyro_z_dps", sensorState.gyroZ);

    send("imu_yaw_gyro_deg", sensorState.yawGyroDeg);

    send("mag_x_uT", sensorState.magX);
    send("mag_y_uT", sensorState.magY);
    send("mag_z_uT", sensorState.magZ);
    send("mag_heading_deg", sensorState.headingMagDeg);

    send("odo_x_cm", odometryState.xCm);
    send("odo_y_cm", odometryState.yCm);
    send("odo_theta_deg", Odometry::normalizeAngleDeg(Odometry::radToDeg(odometryState.thetaRad)));
  }
}
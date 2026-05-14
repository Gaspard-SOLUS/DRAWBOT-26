#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "wifi_param.h"
#include "encodeurs.h"
#include "PID_vitesse.h"
#include "moteurs.h"

// ================= WIFI =================
const char* ssid = "SFR-8e1e";
const char* password = "Y2MHGM1AEM1J";

// IP fixe ESP32
IPAddress local_IP(192, 168, 0, 27);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(1, 1, 1, 1);

// ================= TELEPLOT UDP =================
const char* hostIP = "192.168.0.32";
const int teleplotPort = 47269;

static WiFiUDP udp;

// ================= GUI TCP =================
static WiFiServer server(GUI_PORT);
static WiFiClient client;

static void handlePidCommand(const String& cmd) {
  float kpValue = getKp();
  float kiValue = getKi();
  float kdValue = getKd();

  int start = 0;
  while (start < cmd.length()) {
    int sep = cmd.indexOf(';', start);
    if (sep == -1) sep = cmd.length();

    String token = cmd.substring(start, sep);
    token.trim();

    int eq = token.indexOf('=');
    if (eq > 0) {
      String key = token.substring(0, eq);
      String value = token.substring(eq + 1);

      if (key == "kp") kpValue = value.toFloat();
      else if (key == "ki") kiValue = value.toFloat();
      else if (key == "kd") kdValue = value.toFloat();
    }

    start = sep + 1;
  }

  setPidGains(kpValue, kiValue, kdValue);

  Serial.print("Nouveaux PID -> kp=");
  Serial.print(kpValue, 3);
  Serial.print(" ki=");
  Serial.print(kiValue, 3);
  Serial.print(" kd=");
  Serial.println(kdValue, 3);
}

void wifiInit() {
  Serial.println("Connexion WiFi...");

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Echec configuration IP statique");
  }

  WiFi.begin(ssid, password);

  unsigned long startAttempt = millis();
  const unsigned long timeoutMs = 15000;

  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < timeoutMs) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.println("Echec connexion WiFi");
    return;
  }

  Serial.println();
  Serial.println("WiFi connecte !");
  Serial.print("IP ESP32 : ");
  Serial.println(WiFi.localIP());

  Serial.print("Envoi UDP vers ");
  Serial.print(hostIP);
  Serial.print(":");
  Serial.println(teleplotPort);

  server.begin();
  Serial.print("Serveur TCP lance sur port ");
  Serial.println(GUI_PORT);
}

void wifiSendTelemetry() {
  udp.beginPacket(hostIP, teleplotPort);

  udp.printf("targetL:%f|g\n", getLeftTargetSpeedCmPerSec());
  udp.printf("targetR:%f|g\n", getRightTargetSpeedCmPerSec());

  udp.printf("speedL:%f|g\n", getLeftSpeedCmParSec());
  udp.printf("speedR:%f|g\n", getRightSpeedCmParSec());

  udp.printf("errL:%f|g\n", getLeftPidError());
  udp.printf("errR:%f|g\n", getRightPidError());

  udp.printf("outL:%f|g\n", getLeftPidOutput());
  udp.printf("outR:%f|g\n", getRightPidOutput());

  udp.printf("distAvg:%f|g\n", getAverageDistanceCm());
  udp.printf("ticksL:%ld|g\n", getLeftEncoderTicks());
  udp.printf("ticksR:%ld|g\n", getRightEncoderTicks());

  udp.endPacket();
}

void wifiHandleClient() {
  if (client && !client.connected()) {
    Serial.println("Client GUI deconnecte");
    client.stop();
  }

  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client = newClient;
      Serial.print("Client GUI connecte depuis ");
      Serial.println(client.remoteIP());
    }
  }

  if (client && client.connected() && client.available()) {
    String cmd = client.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) {
      return;
    }

    Serial.println("CMD: " + cmd);

    if (cmd == "FORWARD") {
      setSpeedTargetsCmPerSec(18.0f, 18.0f);
      client.println("ACK;FORWARD");
    }
    else if (cmd == "BACKWARD") {
      setSpeedTargetsCmPerSec(-18.0f, -18.0f);
      client.println("ACK;BACKWARD");
    }
    else if (cmd == "LEFT") {
      setSpeedTargetsCmPerSec(-15.0f, 15.0f);
      client.println("ACK;LEFT");
    }
    else if (cmd == "RIGHT") {
      setSpeedTargetsCmPerSec(15.0f, -15.0f);
      client.println("ACK;RIGHT");
    }
    else if (cmd == "STOP") {
      stopMotors();
      client.println("ACK;STOP");
    }
    else if (cmd.startsWith("SET_PID;")) {
      handlePidCommand(cmd);
      client.println("ACK;SET_PID");
    }
    else if (cmd == "PING") {
      client.println("ACK;PING");
    }
    else {
      client.println("ACK;UNKNOWN");
    }
  }
}

void wifiSendToGUI() {
  if (client && client.connected()) {
    String msg = "TEL;";
    msg += "targetL=" + String(getLeftTargetSpeedCmPerSec()) + ";";
    msg += "targetR=" + String(getRightTargetSpeedCmPerSec()) + ";";

    msg += "speedL=" + String(getLeftSpeedCmParSec()) + ";";
    msg += "speedR=" + String(getRightSpeedCmParSec()) + ";";

    msg += "errL=" + String(getLeftPidError()) + ";";
    msg += "errR=" + String(getRightPidError()) + ";";

    msg += "outL=" + String(getLeftPidOutput()) + ";";
    msg += "outR=" + String(getRightPidOutput()) + ";";

    msg += "kp=" + String(getKp()) + ";";
    msg += "ki=" + String(getKi()) + ";";
    msg += "kd=" + String(getKd());

    client.println(msg);
  }
}
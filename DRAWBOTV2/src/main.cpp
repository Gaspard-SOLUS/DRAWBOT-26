#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

#include "include/pins.h"
#include "include/moteurs.h"
#include "include/encodeurs.h"
#include "include/avance.h"
#include "include/pid_vitesse.h"

// ================= WIFI =================
const char* web_ssid = "SFR-8e1e";
const char* web_password = "Y2MHGM1AEM1J";

WebServer server(80);

// ================= PARAMÈTRES RÉGLABLES =================

// Segments droits
float DIST_1_CM = 20.0;
float DIST_2_CM = 10.0;
float DIST_3_CM = 40.0;

// Avance avec avance.h
int CRUISE_PWM = 210;
int SLOW_PWM = 150;
float SLOW_ZONE_CM = 2.0;
float KP_STRAIGHT = 0.5;

// PID vitesse
float KP_PID = 6.0;
float KI_PID = 1.6;
float KD_PID = 0.0;

// Virage gauche 90°
float TURN_LEFT_CM = 5.0;
int LEFT_TURN_PWM_L_1 = 130;
int LEFT_TURN_PWM_R_1 = 240;
int LEFT_TURN_PWM_L_2 = 150;
int LEFT_TURN_PWM_R_2 = 190;
float LEFT_TURN_PHASE_1_CM = 1.0;

// Virage droite 90°
float TURN_RIGHT_CM = 5.0;
int RIGHT_TURN_PWM_L_1 = 240;
int RIGHT_TURN_PWM_R_1 = 130;
int RIGHT_TURN_PWM_L_2 = 190;
int RIGHT_TURN_PWM_R_2 = 150;
float RIGHT_TURN_PHASE_1_CM = 1.0;

// ================= ÉTATS =================

enum State {
  IDLE,
  START,
  AVANCE_1,
  TURN_LEFT,
  AVANCE_2,
  TURN_RIGHT,
  AVANCE_3,
  FINISHED
};

State state = IDLE;

unsigned long lastEncoderUpdate = 0;
unsigned long lastPrint = 0;

bool sequenceRunning = false;

// ================= OUTILS =================

float distAbs() {
  return (fabs(getLeftDistanceCm()) + fabs(getRightDistanceCm())) / 2.0;
}

void updateEncoderTask(unsigned long now) {
  if (now - lastEncoderUpdate >= 20) {
    lastEncoderUpdate = now;
    updateEncoderMeasurements(now);
  }
}

void printTelemetry(unsigned long now) {
  if (now - lastPrint >= 200) {
    lastPrint = now;

    Serial.print("state=");
    Serial.print(state);
    Serial.print(" | L=");
    Serial.print(getLeftDistanceCm(), 2);
    Serial.print(" | R=");
    Serial.print(getRightDistanceCm(), 2);
    Serial.print(" | distAbs=");
    Serial.println(distAbs(), 2);

    Serial.print(">state:");
    Serial.println(state);
    Serial.print(">distL:");
    Serial.println(getLeftDistanceCm());
    Serial.print(">distR:");
    Serial.println(getRightDistanceCm());
    Serial.print(">distAbs:");
    Serial.println(distAbs());
  }
}

void stopSequence() {
  stopMotors();
  sequenceRunning = false;
  state = IDLE;
  resetEncoders();
  Serial.println("Sequence stoppee");
}

// ================= PAGE WEB =================

String htmlPage() {
  String html = "";

  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<title>Drawbot Control</title>";
  html += "<style>";
  html += "body{font-family:Arial;margin:30px;background:#f5f5f5;}";
  html += "h1{color:#222;} .box{background:white;padding:18px;margin-bottom:18px;border-radius:10px;}";
  html += "input{width:80px;margin:5px;} button{padding:10px 18px;margin:5px;font-size:15px;}";
  html += "</style></head><body>";

  html += "<h1>Drawbot - Sequence escalier</h1>";

  html += "<div class='box'>";
  html += "<h2>Commandes</h2>";
  html += "<a href='/start'><button>Lancer escalier</button></a>";
  html += "<a href='/stop'><button>Stop</button></a>";
  html += "</div>";

  html += "<form action='/update' method='GET'>";

  html += "<div class='box'>";
  html += "<h2>Distances</h2>";
  html += "Ligne 1: <input name='d1' value='" + String(DIST_1_CM) + "'> cm<br>";
  html += "Ligne 2: <input name='d2' value='" + String(DIST_2_CM) + "'> cm<br>";
  html += "Ligne 3: <input name='d3' value='" + String(DIST_3_CM) + "'> cm<br>";
  html += "</div>";

  html += "<div class='box'>";
  html += "<h2>Avance droite</h2>";
  html += "CRUISE_PWM: <input name='cruise' value='" + String(CRUISE_PWM) + "'><br>";
  html += "SLOW_PWM: <input name='slow' value='" + String(SLOW_PWM) + "'><br>";
  html += "SLOW_ZONE_CM: <input name='slowzone' value='" + String(SLOW_ZONE_CM) + "'><br>";
  html += "KP_STRAIGHT: <input name='kpstraight' value='" + String(KP_STRAIGHT) + "'><br>";
  html += "</div>";

  html += "<div class='box'>";
  html += "<h2>PID vitesse</h2>";
  html += "Kp: <input name='kppid' value='" + String(KP_PID) + "'><br>";
  html += "Ki: <input name='kipid' value='" + String(KI_PID) + "'><br>";
  html += "Kd: <input name='kdpid' value='" + String(KD_PID) + "'><br>";
  html += "</div>";

  html += "<div class='box'>";
  html += "<h2>Virage gauche 90 deg</h2>";
  html += "TURN_LEFT_CM: <input name='tlcm' value='" + String(TURN_LEFT_CM) + "'><br>";
  html += "Phase 1 gauche: <input name='tl_l1' value='" + String(LEFT_TURN_PWM_L_1) + "'>";
  html += " droite: <input name='tl_r1' value='" + String(LEFT_TURN_PWM_R_1) + "'><br>";
  html += "Phase 2 gauche: <input name='tl_l2' value='" + String(LEFT_TURN_PWM_L_2) + "'>";
  html += " droite: <input name='tl_r2' value='" + String(LEFT_TURN_PWM_R_2) + "'><br>";
  html += "Fin phase 1: <input name='tlp1' value='" + String(LEFT_TURN_PHASE_1_CM) + "'> cm<br>";
  html += "</div>";

  html += "<div class='box'>";
  html += "<h2>Virage droite 90 deg</h2>";
  html += "TURN_RIGHT_CM: <input name='trcm' value='" + String(TURN_RIGHT_CM) + "'><br>";
  html += "Phase 1 gauche: <input name='tr_l1' value='" + String(RIGHT_TURN_PWM_L_1) + "'>";
  html += " droite: <input name='tr_r1' value='" + String(RIGHT_TURN_PWM_R_1) + "'><br>";
  html += "Phase 2 gauche: <input name='tr_l2' value='" + String(RIGHT_TURN_PWM_L_2) + "'>";
  html += " droite: <input name='tr_r2' value='" + String(RIGHT_TURN_PWM_R_2) + "'><br>";
  html += "Fin phase 1: <input name='trp1' value='" + String(RIGHT_TURN_PHASE_1_CM) + "'> cm<br>";
  html += "</div>";

  html += "<button type='submit'>Enregistrer les valeurs</button>";
  html += "</form>";

  html += "</body></html>";

  return html;
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleStart() {
  resetEncoders();
  sequenceRunning = true;
  state = START;
  server.send(200, "text/html", "<h1>Sequence lancee</h1><a href='/'>Retour</a>");
}

void handleStop() {
  stopSequence();
  server.send(200, "text/html", "<h1>Robot stoppe</h1><a href='/'>Retour</a>");
}

void readFloatParam(const char* name, float& value) {
  if (server.hasArg(name)) value = server.arg(name).toFloat();
}

void readIntParam(const char* name, int& value) {
  if (server.hasArg(name)) value = server.arg(name).toInt();
}

void handleUpdate() {
  readFloatParam("d1", DIST_1_CM);
  readFloatParam("d2", DIST_2_CM);
  readFloatParam("d3", DIST_3_CM);

  readIntParam("cruise", CRUISE_PWM);
  readIntParam("slow", SLOW_PWM);
  readFloatParam("slowzone", SLOW_ZONE_CM);
  readFloatParam("kpstraight", KP_STRAIGHT);

  readFloatParam("kppid", KP_PID);
  readFloatParam("kipid", KI_PID);
  readFloatParam("kdpid", KD_PID);
  setPidGains(KP_PID, KI_PID, KD_PID);

  readFloatParam("tlcm", TURN_LEFT_CM);
  readIntParam("tl_l1", LEFT_TURN_PWM_L_1);
  readIntParam("tl_r1", LEFT_TURN_PWM_R_1);
  readIntParam("tl_l2", LEFT_TURN_PWM_L_2);
  readIntParam("tl_r2", LEFT_TURN_PWM_R_2);
  readFloatParam("tlp1", LEFT_TURN_PHASE_1_CM);

  readFloatParam("trcm", TURN_RIGHT_CM);
  readIntParam("tr_l1", RIGHT_TURN_PWM_L_1);
  readIntParam("tr_r1", RIGHT_TURN_PWM_R_1);
  readIntParam("tr_l2", RIGHT_TURN_PWM_L_2);
  readIntParam("tr_r2", RIGHT_TURN_PWM_R_2);
  readFloatParam("trp1", RIGHT_TURN_PHASE_1_CM);

  server.send(200, "text/html", "<h1>Valeurs enregistrees</h1><a href='/'>Retour</a>");
}

// ================= SÉQUENCE ESCALIER =================

void updateSequence() {
  switch (state) {

    case IDLE:
      stopMotors();
      break;

    case START:
      Serial.println("PHASE 1 : AVANCE 20 CM");
      startAvanceForwardDistance(DIST_1_CM, CRUISE_PWM, SLOW_PWM, SLOW_ZONE_CM, KP_STRAIGHT);
      state = AVANCE_1;
      break;

    case AVANCE_1:
      updateAvance(millis());

      if (isAvanceTermine()) {
        stopMotors();
        delay(200);
        resetEncoders();

        Serial.println("PHASE 2 : VIRAGE GAUCHE 90");
        state = TURN_LEFT;
      }
      break;

    case TURN_LEFT: {
      float d = distAbs();

      if (d < LEFT_TURN_PHASE_1_CM) {
        setMotors(LEFT_TURN_PWM_L_1, LEFT_TURN_PWM_R_1);
      } else {
        setMotors(LEFT_TURN_PWM_L_2, LEFT_TURN_PWM_R_2);
      }

      if (d >= TURN_LEFT_CM) {
        stopMotors();
        delay(200);
        resetEncoders();

        Serial.println("PHASE 3 : AVANCE 10 CM");
        startAvanceForwardDistance(DIST_2_CM, CRUISE_PWM, SLOW_PWM, SLOW_ZONE_CM, KP_STRAIGHT);
        state = AVANCE_2;
      }
      break;
    }

    case AVANCE_2:
      updateAvance(millis());

      if (isAvanceTermine()) {
        stopMotors();
        delay(200);
        resetEncoders();

        Serial.println("PHASE 4 : VIRAGE DROITE 90");
        state = TURN_RIGHT;
      }
      break;

    case TURN_RIGHT: {
      float d = distAbs();

      if (d < RIGHT_TURN_PHASE_1_CM) {
        setMotors(RIGHT_TURN_PWM_L_1, RIGHT_TURN_PWM_R_1);
      } else {
        setMotors(RIGHT_TURN_PWM_L_2, RIGHT_TURN_PWM_R_2);
      }

      if (d >= TURN_RIGHT_CM) {
        stopMotors();
        delay(200);
        resetEncoders();

        Serial.println("PHASE 5 : AVANCE 40 CM");
        startAvanceForwardDistance(DIST_3_CM, CRUISE_PWM, SLOW_PWM, SLOW_ZONE_CM, KP_STRAIGHT);
        state = AVANCE_3;
      }
      break;
    }

    case AVANCE_3:
      updateAvance(millis());

      if (isAvanceTermine()) {
        stopMotors();
        Serial.println("SEQUENCE ESCALIER TERMINEE");
        state = FINISHED;
      }
      break;

    case FINISHED:
      stopMotors();
      sequenceRunning = false;
      break;
  }
}

// ================= SETUP / LOOP =================

void setup() {
  Serial.begin(115200);

  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  initMotors();
  initEncoders();
  initPidVitesse();
  resetEncoders();

  setPidGains(KP_PID, KI_PID, KD_PID);

  WiFi.begin(web_ssid, web_password);
  Serial.print("Connexion WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("IP ESP32 : ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/update", handleUpdate);

  server.begin();

  lastEncoderUpdate = millis();
  lastPrint = millis();

  Serial.println("Serveur web demarre");
}

void loop() {
  unsigned long now = millis();

  server.handleClient();
  updateEncoderTask(now);
  printTelemetry(now);

  if (sequenceRunning || state != IDLE) {
    updateSequence();
  }
}
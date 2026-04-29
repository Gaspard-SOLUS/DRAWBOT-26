#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "include/pins.h"
#include "include/moteurs.h"
#include "include/encodeurs.h"
#include "include/avance.h"

// ==================================================
// ================= WIFI AP ========================
// ==================================================

const char* apSsid = "Drawbot";
const char* apPassword = "12345678";

WebServer server(80);

// ==================================================
// ================= ODOMETRIE ======================
// ==================================================

float wheelBaseCm = 8.3;
float penOffsetCm = 13.5;

float xRobot = 0.0;
float yRobot = 0.0;
float thetaRobot = 0.0;

float xPen = 0.0;
float yPen = 0.0;

float lastLeftCm = 0.0;
float lastRightCm = 0.0;

// ==================================================
// =============== PARAMETRES REGLABLES =============
// ==================================================

float dist1Cm = 12.0;
float dist2Cm = 13.0;
float dist3Cm = 13.0;

int cruisePwm = 210;
int slowPwm = 150;
float slowZoneCm = 0.0;
float kpStraight = 1.5;

// Virage gauche
float leftTurnStartDistanceCm = 11.0;
float leftTurnStopDistanceCm = 3.3;

int leftTurnStartLeftPwm = 140;
int leftTurnStartRightPwm = 225;

int leftTurnLeftPwm = 160;
int leftTurnRightPwm = 220;

// Virage droite
float rightTurnStartDistanceCm = 11.0;
float rightTurnStopDistanceCm = 3.3;

int rightTurnStartLeftPwm = 225;
int rightTurnStartRightPwm = 140;

int rightTurnLeftPwm = 220;
int rightTurnRightPwm = 160;

// ==================================================
// ================= VARIABLES ======================
// ==================================================

unsigned long lastEncoderUpdate = 0;
unsigned long lastPrint = 0;

enum State {
  START,
  AVANCE_1,
  TURN_LEFT,
  AVANCE_2,
  TURN_RIGHT,
  AVANCE_3,
  FINISHED
};

State state = FINISHED;

// ==================================================
// ================= ODOMETRIE ======================
// ==================================================

void resetOdometry() {
  xRobot = 0.0;
  yRobot = 0.0;
  thetaRobot = 0.0;

  xPen = penOffsetCm;
  yPen = 0.0;

  lastLeftCm = getLeftDistanceCm();
  lastRightCm = getRightDistanceCm();
}

void updateOdometry() {
  float leftCm = getLeftDistanceCm();
  float rightCm = getRightDistanceCm();

  float dL = leftCm - lastLeftCm;
  float dR = rightCm - lastRightCm;

  lastLeftCm = leftCm;
  lastRightCm = rightCm;

  float dCenter = (dL + dR) / 2.0;
  float dTheta = (dR - dL) / wheelBaseCm;

  thetaRobot += dTheta;

  xRobot += dCenter * cos(thetaRobot);
  yRobot += dCenter * sin(thetaRobot);

  xPen = xRobot + penOffsetCm * cos(thetaRobot);
  yPen = yRobot + penOffsetCm * sin(thetaRobot);
}

float distAbs() {
  return (fabs(getLeftDistanceCm()) + fabs(getRightDistanceCm())) / 2.0;
}

// ==================================================
// ================= TELEMETRIE =====================
// ==================================================

void updateEncoderTask(unsigned long now) {
  if (now - lastEncoderUpdate >= 20) {
    lastEncoderUpdate = now;
    updateEncoderMeasurements(now);
    updateOdometry();
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
    Serial.print(distAbs(), 2);
    Serial.print(" | theta=");
    Serial.print(thetaRobot * 180.0 / PI, 2);
    Serial.print(" | xPen=");
    Serial.print(xPen, 2);
    Serial.print(" | yPen=");
    Serial.println(yPen, 2);

    Serial.print(">state:");
    Serial.println(state);

    Serial.print(">distL:");
    Serial.println(getLeftDistanceCm());

    Serial.print(">distR:");
    Serial.println(getRightDistanceCm());

    Serial.print(">distAbs:");
    Serial.println(distAbs());

    Serial.print(">thetaDeg:");
    Serial.println(thetaRobot * 180.0 / PI);

    Serial.print(">xPen:");
    Serial.println(xPen);

    Serial.print(">yPen:");
    Serial.println(yPen);
  }
}

// ==================================================
// ================= WEB PAGE =======================
// ==================================================

String htmlInput(String label, String name, String value) {
  String s = "";
  s += "<label>" + label + "</label><br>";
  s += "<input name='" + name + "' value='" + value + "'><br><br>";
  return s;
}

void handleRoot() {
  String html = "";

  html += "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body{font-family:Arial;background:#111;color:white;margin:0;padding:20px 20px 20px 240px;}";
  html += ".sidebar{position:fixed;left:0;top:0;width:220px;height:100vh;background:#181818;padding:15px;box-sizing:border-box;}";
  html += ".content{max-width:900px;}";
  html += "input{width:100%;padding:10px;font-size:18px;margin-top:5px;box-sizing:border-box;}";
  html += "button,a{display:block;text-align:center;padding:14px;margin:10px 0;background:#2b7cff;color:white;text-decoration:none;border-radius:8px;border:0;font-size:18px;}";
  html += ".stop{background:#d22;}";
  html += ".card{background:#222;padding:15px;border-radius:10px;margin-bottom:15px;}";
  html += "@media(max-width:700px){body{padding:170px 15px 15px 15px}.sidebar{width:100%;height:auto;}.sidebar a{display:inline-block;width:30%;margin:5px;}}";
  html += "</style>";
  html += "</head><body>";

  html += "<div class='sidebar'>";
  html += "<h2>Commandes</h2>";
  html += "<a href='/go'>GO</a>";
  html += "<a class='stop' href='/stop'>STOP</a>";
  html += "<a href='/reset'>RESET</a>";
  html += "</div>";

  html += "<div class='content'>";
  html += "<h1>Drawbot Reglages</h1>";

  html += "<div class='card'>";
  html += "<p>Etat : " + String(state) + "</p>";
  html += "<p>distAbs : " + String(distAbs(), 2) + " cm</p>";
  html += "<p>theta : " + String(thetaRobot * 180.0 / PI, 2) + " deg</p>";
  html += "<p>xPen : " + String(xPen, 2) + " cm</p>";
  html += "<p>yPen : " + String(yPen, 2) + " cm</p>";
  html += "</div>";

  html += "<form action='/set'>";

  html += "<div class='card'>";
  html += "<h2>Lignes droites</h2>";
  html += htmlInput("Distance 1 cm", "dist1", String(dist1Cm));
  html += htmlInput("Distance 2 cm", "dist2", String(dist2Cm));
  html += htmlInput("Distance 3 cm", "dist3", String(dist3Cm));
  html += htmlInput("Cruise PWM", "cruise", String(cruisePwm));
  html += htmlInput("Slow PWM", "slow", String(slowPwm));
  html += htmlInput("Slow zone cm", "slowzone", String(slowZoneCm));
  html += htmlInput("KP straight", "kp", String(kpStraight));
  html += "</div>";

  html += "<div class='card'>";
  html += "<h2>Virage gauche</h2>";
  html += htmlInput("Distance debut doux gauche cm", "leftStartDist", String(leftTurnStartDistanceCm));
  html += htmlInput("Distance arret virage gauche cm", "leftStopDist", String(leftTurnStopDistanceCm));
  html += htmlInput("PWM gauche debut gauche", "leftStartL", String(leftTurnStartLeftPwm));
  html += htmlInput("PWM droite debut gauche", "leftStartR", String(leftTurnStartRightPwm));
  html += htmlInput("PWM gauche virage gauche", "leftTurnL", String(leftTurnLeftPwm));
  html += htmlInput("PWM droite virage gauche", "leftTurnR", String(leftTurnRightPwm));
  html += "</div>";

  html += "<div class='card'>";
  html += "<h2>Virage droite</h2>";
  html += htmlInput("Distance debut doux droite cm", "rightStartDist", String(rightTurnStartDistanceCm));
  html += htmlInput("Distance arret virage droite cm", "rightStopDist", String(rightTurnStopDistanceCm));
  html += htmlInput("PWM gauche debut droite", "rightStartL", String(rightTurnStartLeftPwm));
  html += htmlInput("PWM droite debut droite", "rightStartR", String(rightTurnStartRightPwm));
  html += htmlInput("PWM gauche virage droite", "rightTurnL", String(rightTurnLeftPwm));
  html += htmlInput("PWM droite virage droite", "rightTurnR", String(rightTurnRightPwm));
  html += "</div>";

  html += "<div class='card'>";
  html += "<h2>Robot</h2>";
  html += htmlInput("Wheel base cm", "wheelbase", String(wheelBaseCm));
  html += htmlInput("Pen offset cm", "penoffset", String(penOffsetCm));
  html += "</div>";

  html += "<button type='submit'>Enregistrer les reglages</button>";
  html += "</form>";

  html += "</div>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleSet() {
  if (server.hasArg("dist1")) dist1Cm = server.arg("dist1").toFloat();
  if (server.hasArg("dist2")) dist2Cm = server.arg("dist2").toFloat();
  if (server.hasArg("dist3")) dist3Cm = server.arg("dist3").toFloat();

  if (server.hasArg("leftStartDist")) leftTurnStartDistanceCm = server.arg("leftStartDist").toFloat();
  if (server.hasArg("leftStopDist")) leftTurnStopDistanceCm = server.arg("leftStopDist").toFloat();
  if (server.hasArg("leftStartL")) leftTurnStartLeftPwm = server.arg("leftStartL").toInt();
  if (server.hasArg("leftStartR")) leftTurnStartRightPwm = server.arg("leftStartR").toInt();
  if (server.hasArg("leftTurnL")) leftTurnLeftPwm = server.arg("leftTurnL").toInt();
  if (server.hasArg("leftTurnR")) leftTurnRightPwm = server.arg("leftTurnR").toInt();

  if (server.hasArg("rightStartDist")) rightTurnStartDistanceCm = server.arg("rightStartDist").toFloat();
  if (server.hasArg("rightStopDist")) rightTurnStopDistanceCm = server.arg("rightStopDist").toFloat();
  if (server.hasArg("rightStartL")) rightTurnStartLeftPwm = server.arg("rightStartL").toInt();
  if (server.hasArg("rightStartR")) rightTurnStartRightPwm = server.arg("rightStartR").toInt();
  if (server.hasArg("rightTurnL")) rightTurnLeftPwm = server.arg("rightTurnL").toInt();
  if (server.hasArg("rightTurnR")) rightTurnRightPwm = server.arg("rightTurnR").toInt();

  if (server.hasArg("cruise")) cruisePwm = server.arg("cruise").toInt();
  if (server.hasArg("slow")) slowPwm = server.arg("slow").toInt();
  if (server.hasArg("slowzone")) slowZoneCm = server.arg("slowzone").toFloat();
  if (server.hasArg("kp")) kpStraight = server.arg("kp").toFloat();

  if (server.hasArg("wheelbase")) wheelBaseCm = server.arg("wheelbase").toFloat();
  if (server.hasArg("penoffset")) penOffsetCm = server.arg("penoffset").toFloat();

  Serial.println("Reglages mis a jour");

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleGo() {
  stopMotors();

  resetEncoders();
  resetOdometry();

  state = START;

  Serial.println("GO depuis page web");

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStop() {
  stopMotors();
  state = FINISHED;

  Serial.println("STOP depuis page web");

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleReset() {
  resetEncoders();
  resetOdometry();

  Serial.println("Reset odometrie depuis page web");

  server.sendHeader("Location", "/");
  server.send(303);
}

// ==================================================
// ================= SETUP ==========================
// ==================================================

void setup() {
  Serial.begin(115200);

  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  initMotors();
  initEncoders();

  resetEncoders();
  resetOdometry();

  delay(1000);

  Serial.println("=== DRAWBOT WIFI CONTROL ===");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid, apPassword);

  Serial.print("WiFi cree : ");
  Serial.println(apSsid);

  Serial.print("IP : ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/go", handleGo);
  server.on("/stop", handleStop);
  server.on("/reset", handleReset);

  server.begin();

  Serial.println("Serveur web lance");

  lastEncoderUpdate = millis();
  lastPrint = millis();
}

// ==================================================
// ================= LOOP ===========================
// ==================================================

void loop() {
  unsigned long now = millis();

  server.handleClient();

  updateEncoderTask(now);
  printTelemetry(now);

  switch (state) {
    case START:
      Serial.println("PHASE 1 : AVANCE 1");

      resetEncoders();
      resetOdometry();

      startAvanceForwardDistance(
        dist1Cm,
        cruisePwm,
        slowPwm,
        slowZoneCm,
        kpStraight
      );

      state = AVANCE_1;
      break;

    case AVANCE_1:
      updateAvance(now);

      if (isAvanceTermine()) {
        resetEncoders();
        resetOdometry();

        Serial.println("PHASE 2 : ANGLE GAUCHE");
        state = TURN_LEFT;
      }
      break;

    case TURN_LEFT: {
      float d = distAbs();

      if (d < leftTurnStartDistanceCm) {
        setMotors(leftTurnStartLeftPwm, leftTurnStartRightPwm);
      } else {
        setMotors(leftTurnLeftPwm, leftTurnRightPwm);
      }

      if (d >= leftTurnStopDistanceCm) {
        resetEncoders();
        resetOdometry();

        startAvanceForwardDistance(dist2Cm, cruisePwm, slowPwm, slowZoneCm, kpStraight);
        state = AVANCE_2;
      }
      break;
    }

    case AVANCE_2:
      updateAvance(now);

      if (isAvanceTermine()) {
        resetEncoders();
        resetOdometry();

        Serial.println("PHASE 4 : ANGLE DROIT");
        state = TURN_RIGHT;
      }
      break;

    case TURN_RIGHT: {
      float d = distAbs();

      if (d < rightTurnStartDistanceCm) {
        setMotors(rightTurnStartLeftPwm, rightTurnStartRightPwm);
      } else {
        setMotors(rightTurnLeftPwm, rightTurnRightPwm);
      }

      if (d >= rightTurnStopDistanceCm) {
        resetEncoders();
        resetOdometry();

        startAvanceForwardDistance(dist3Cm, cruisePwm, slowPwm, slowZoneCm, kpStraight);
        state = AVANCE_3;
      }
      break;
    }

    case AVANCE_3:
      updateAvance(now);

      if (isAvanceTermine()) {
        stopMotors();
        Serial.println("SEQUENCE TERMINEE");
        state = FINISHED;
      }
      break;

    case FINISHED:
      stopMotors();
      break;
  }
}
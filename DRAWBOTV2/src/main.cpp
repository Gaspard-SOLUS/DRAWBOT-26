#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#include "include/pins.h"
#include "include/moteurs.h"
#include "include/encodeurs.h"
#include "include/avance.h"

// ==================================================
// WIFI ESP32 EN POINT D'ACCES
// ==================================================
const char* AP_SSID = "DRAWBOT_ESP32";
const char* AP_PASS = "12345678";

WebServer server(80);
Preferences prefs;

// ==================================================
// PARAMETRES MODIFIABLES DEPUIS LA PAGE WEB
// ==================================================
struct Params {
  float dist1 = 12.0;
  float turnDist = 5.0;
  float dist2 = 13.0;

  int cruisePwm = 210;
  int slowPwm = 150;
  float slowZone = 2.0;
  float kpStraight = 0.5;

  int turnStartL = 140;
  int turnStartR = 250;
  int turnEndL = 150;
  int turnEndR = 190;

  int avance2StartL = 205;
  int avance2StartR = 175;
  int avance2EndL = 190;
  int avance2EndR = 180;
};

Params p;

// ==================================================
// ETATS ROBOT
// ==================================================
enum State {
  IDLE,
  AVANCE_1,
  TURN_CURVE,
  AVANCE_2,
  FINISHED,
  MANUAL
};

State state = IDLE;

unsigned long lastEncoderUpdate = 0;
unsigned long lastPrint = 0;

// ==================================================
// OUTILS
// ==================================================
float distAbs() {
  return (fabs(getLeftDistanceCm()) + fabs(getRightDistanceCm())) / 2.0;
}

void updateEncoderTask(unsigned long now) {
  if (now - lastEncoderUpdate >= 20) {
    lastEncoderUpdate = now;
    updateEncoderMeasurements(now);
  }
}

void stopRobot() {
  stopMotors();
  state = IDLE;
}

void resetRobot() {
  stopMotors();
  resetEncoders();
  state = IDLE;
}

void startSequence() {
  stopMotors();
  resetEncoders();

  Serial.println("PHASE 1 : AVANCE 1");
  startAvanceForwardDistance(
    p.dist1,
    p.cruisePwm,
    p.slowPwm,
    p.slowZone,
    p.kpStraight
  );

  state = AVANCE_1;
}

String stateName() {
  switch (state) {
    case IDLE: return "IDLE";
    case AVANCE_1: return "AVANCE_1";
    case TURN_CURVE: return "TURN_CURVE";
    case AVANCE_2: return "AVANCE_2";
    case FINISHED: return "FINISHED";
    case MANUAL: return "MANUAL";
  }
  return "UNKNOWN";
}

// ==================================================
// SAUVEGARDE FLASH
// ==================================================
void loadParams() {
  prefs.begin("drawbot", true);

  p.dist1 = prefs.getFloat("dist1", p.dist1);
  p.turnDist = prefs.getFloat("turnDist", p.turnDist);
  p.dist2 = prefs.getFloat("dist2", p.dist2);

  p.cruisePwm = prefs.getInt("cruisePwm", p.cruisePwm);
  p.slowPwm = prefs.getInt("slowPwm", p.slowPwm);
  p.slowZone = prefs.getFloat("slowZone", p.slowZone);
  p.kpStraight = prefs.getFloat("kpStraight", p.kpStraight);

  p.turnStartL = prefs.getInt("turnStartL", p.turnStartL);
  p.turnStartR = prefs.getInt("turnStartR", p.turnStartR);
  p.turnEndL = prefs.getInt("turnEndL", p.turnEndL);
  p.turnEndR = prefs.getInt("turnEndR", p.turnEndR);

  p.avance2StartL = prefs.getInt("avance2StartL", p.avance2StartL);
  p.avance2StartR = prefs.getInt("avance2StartR", p.avance2StartR);
  p.avance2EndL = prefs.getInt("avance2EndL", p.avance2EndL);
  p.avance2EndR = prefs.getInt("avance2EndR", p.avance2EndR);

  prefs.end();
}

void saveParams() {
  prefs.begin("drawbot", false);

  prefs.putFloat("dist1", p.dist1);
  prefs.putFloat("turnDist", p.turnDist);
  prefs.putFloat("dist2", p.dist2);

  prefs.putInt("cruisePwm", p.cruisePwm);
  prefs.putInt("slowPwm", p.slowPwm);
  prefs.putFloat("slowZone", p.slowZone);
  prefs.putFloat("kpStraight", p.kpStraight);

  prefs.putInt("turnStartL", p.turnStartL);
  prefs.putInt("turnStartR", p.turnStartR);
  prefs.putInt("turnEndL", p.turnEndL);
  prefs.putInt("turnEndR", p.turnEndR);

  prefs.putInt("avance2StartL", p.avance2StartL);
  prefs.putInt("avance2StartR", p.avance2StartR);
  prefs.putInt("avance2EndL", p.avance2EndL);
  prefs.putInt("avance2EndR", p.avance2EndR);

  prefs.end();
}

// ==================================================
// PAGE HTML
// ==================================================
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Drawbot Control Panel</title>
<style>
:root {
  --bg: #0f172a;
  --card: #111827;
  --panel: #020617;
  --accent: #38bdf8;
  --green: #22c55e;
  --red: #ef4444;
  --orange: #f97316;
  --text: #e5e7eb;
  --muted: #94a3b8;
  --border: #1f2937;
}

* {
  box-sizing: border-box;
}

body {
  margin: 0;
  font-family: Arial, sans-serif;
  background: var(--bg);
  color: var(--text);
}

.app {
  display: grid;
  grid-template-columns: 1fr 330px;
  min-height: 100vh;
}

main {
  padding: 24px;
}

.sidebar {
  background: var(--panel);
  border-left: 1px solid var(--border);
  padding: 20px;
  position: sticky;
  top: 0;
  height: 100vh;
  overflow-y: auto;
}

h1 {
  margin: 0 0 4px;
  font-size: 28px;
}

.subtitle {
  color: var(--muted);
  margin-bottom: 24px;
}

.grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
  gap: 18px;
}

.card {
  background: var(--card);
  border: 1px solid var(--border);
  border-radius: 16px;
  padding: 18px;
  box-shadow: 0 10px 30px rgba(0,0,0,0.25);
}

.card h2 {
  margin-top: 0;
  font-size: 18px;
}

label {
  display: block;
  margin-top: 12px;
  color: var(--muted);
  font-size: 14px;
}

input {
  width: 100%;
  margin-top: 5px;
  padding: 10px;
  border-radius: 10px;
  border: 1px solid var(--border);
  background: #020617;
  color: var(--text);
  font-size: 15px;
}

button {
  border: none;
  padding: 12px 14px;
  border-radius: 12px;
  color: white;
  cursor: pointer;
  font-weight: bold;
  margin: 5px 0;
  width: 100%;
}

.btn-blue { background: var(--accent); color: #00111d; }
.btn-green { background: var(--green); }
.btn-red { background: var(--red); }
.btn-orange { background: var(--orange); }
.btn-dark { background: #334155; }

.status {
  padding: 14px;
  border-radius: 14px;
  background: #0f172a;
  border: 1px solid var(--border);
  margin-bottom: 16px;
}

.status div {
  margin-bottom: 7px;
}

.commands {
  background: #020617;
  border: 1px solid var(--border);
  border-radius: 14px;
  padding: 12px;
  font-family: Consolas, monospace;
  font-size: 13px;
  color: #a7f3d0;
  height: 240px;
  overflow-y: auto;
}

.manual-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 8px;
  margin-top: 10px;
}

.manual-grid button {
  margin: 0;
}

@media(max-width: 900px) {
  .app {
    grid-template-columns: 1fr;
  }

  .sidebar {
    position: relative;
    height: auto;
    border-left: none;
    border-top: 1px solid var(--border);
  }
}
</style>
</head>
<body>
<div class="app">
<main>
  <h1>Drawbot Control Panel</h1>
  <div class="subtitle">Réglage des distances, PWM et commandes du robot en temps réel</div>

  <div class="grid">
    <section class="card">
      <h2>1. Distances</h2>
      <label>Distance ligne droite 1 — cm</label>
      <input id="dist1" type="number" step="0.1">

      <label>Distance du virage — cm</label>
      <input id="turnDist" type="number" step="0.1">

      <label>Distance ligne droite 2 — cm</label>
      <input id="dist2" type="number" step="0.1">
    </section>

    <section class="card">
      <h2>2. Avance 1 avec avance.h</h2>
      <label>PWM vitesse normale</label>
      <input id="cruisePwm" type="number">

      <label>PWM ralentissement</label>
      <input id="slowPwm" type="number">

      <label>Zone de ralentissement — cm</label>
      <input id="slowZone" type="number" step="0.1">

      <label>Correction ligne droite KP</label>
      <input id="kpStraight" type="number" step="0.01">
    </section>

    <section class="card">
      <h2>3. Virage / coude</h2>
      <label>Début virage PWM gauche</label>
      <input id="turnStartL" type="number">

      <label>Début virage PWM droite</label>
      <input id="turnStartR" type="number">

      <label>Fin virage PWM gauche</label>
      <input id="turnEndL" type="number">

      <label>Fin virage PWM droite</label>
      <input id="turnEndR" type="number">
    </section>

    <section class="card">
      <h2>4. Avance 2 corrigée</h2>
      <label>Début avance 2 PWM gauche</label>
      <input id="avance2StartL" type="number">

      <label>Début avance 2 PWM droite</label>
      <input id="avance2StartR" type="number">

      <label>Fin avance 2 PWM gauche</label>
      <input id="avance2EndL" type="number">

      <label>Fin avance 2 PWM droite</label>
      <input id="avance2EndR" type="number">
    </section>
  </div>
</main>

<aside class="sidebar">
  <h2>Commandes robot</h2>

  <div class="status">
    <div><strong>État :</strong> <span id="state">---</span></div>
    <div><strong>Distance G :</strong> <span id="distL">0</span> cm</div>
    <div><strong>Distance D :</strong> <span id="distR">0</span> cm</div>
    <div><strong>Distance moyenne :</strong> <span id="distAbs">0</span> cm</div>
  </div>

  <button class="btn-blue" onclick="sendConfig()">Appliquer les valeurs</button>
  <button class="btn-green" onclick="saveConfig()">Enregistrer en mémoire</button>
  <button class="btn-orange" onclick="cmd('/start')">START séquence</button>
  <button class="btn-red" onclick="cmd('/stop')">STOP moteurs</button>
  <button class="btn-dark" onclick="cmd('/reset')">RESET encodeurs</button>

  <h2>Commande manuelle</h2>
  <div class="manual-grid">
    <div></div>
    <button class="btn-dark" onclick="manual(180,180)">↑</button>
    <div></div>

    <button class="btn-dark" onclick="manual(-160,160)">←</button>
    <button class="btn-red" onclick="manual(0,0)">■</button>
    <button class="btn-dark" onclick="manual(160,-160)">→</button>

    <div></div>
    <button class="btn-dark" onclick="manual(-160,-160)">↓</button>
    <div></div>
  </div>

  <h2>Commandes envoyées</h2>
  <div id="log" class="commands"></div>
</aside>
</div>

<script>
const fields = [
  "dist1", "turnDist", "dist2",
  "cruisePwm", "slowPwm", "slowZone", "kpStraight",
  "turnStartL", "turnStartR", "turnEndL", "turnEndR",
  "avance2StartL", "avance2StartR", "avance2EndL", "avance2EndR"
];

function log(txt) {
  const box = document.getElementById("log");
  box.innerHTML = "> " + txt + "<br>" + box.innerHTML;
}

async function loadConfig() {
  const res = await fetch("/api/config");
  const data = await res.json();

  fields.forEach(id => {
    document.getElementById(id).value = data[id];
  });

  log("Configuration chargée");
}

async function sendConfig() {
  const params = new URLSearchParams();

  fields.forEach(id => {
    params.append(id, document.getElementById(id).value);
  });

  await fetch("/api/set?" + params.toString());
  log("Valeurs appliquées au robot");
}

async function saveConfig() {
  await sendConfig();
  await fetch("/api/save");
  log("Configuration enregistrée en mémoire flash");
}

async function cmd(route) {
  await fetch("/api" + route);
  log("Commande envoyée : " + route);
}

async function manual(left, right) {
  await fetch(`/api/manual?l=${left}&r=${right}`);
  log(`Manuel : gauche=${left}, droite=${right}`);
}

async function refreshStatus() {
  const res = await fetch("/api/status");
  const data = await res.json();

  document.getElementById("state").textContent = data.state;
  document.getElementById("distL").textContent = data.distL.toFixed(2);
  document.getElementById("distR").textContent = data.distR.toFixed(2);
  document.getElementById("distAbs").textContent = data.distAbs.toFixed(2);
}

loadConfig();
setInterval(refreshStatus, 300);
</script>
</body>
</html>
)rawliteral";

// ==================================================
// API WEB
// ==================================================
void handleRoot() {
  server.send_P(200, "text/html", PAGE_HTML);
}

void handleConfig() {
  String json = "{";
  json += "\"dist1\":" + String(p.dist1) + ",";
  json += "\"turnDist\":" + String(p.turnDist) + ",";
  json += "\"dist2\":" + String(p.dist2) + ",";

  json += "\"cruisePwm\":" + String(p.cruisePwm) + ",";
  json += "\"slowPwm\":" + String(p.slowPwm) + ",";
  json += "\"slowZone\":" + String(p.slowZone) + ",";
  json += "\"kpStraight\":" + String(p.kpStraight) + ",";

  json += "\"turnStartL\":" + String(p.turnStartL) + ",";
  json += "\"turnStartR\":" + String(p.turnStartR) + ",";
  json += "\"turnEndL\":" + String(p.turnEndL) + ",";
  json += "\"turnEndR\":" + String(p.turnEndR) + ",";

  json += "\"avance2StartL\":" + String(p.avance2StartL) + ",";
  json += "\"avance2StartR\":" + String(p.avance2StartR) + ",";
  json += "\"avance2EndL\":" + String(p.avance2EndL) + ",";
  json += "\"avance2EndR\":" + String(p.avance2EndR);
  json += "}";

  server.send(200, "application/json", json);
}

void handleSet() {
  if (server.hasArg("dist1")) p.dist1 = server.arg("dist1").toFloat();
  if (server.hasArg("turnDist")) p.turnDist = server.arg("turnDist").toFloat();
  if (server.hasArg("dist2")) p.dist2 = server.arg("dist2").toFloat();

  if (server.hasArg("cruisePwm")) p.cruisePwm = server.arg("cruisePwm").toInt();
  if (server.hasArg("slowPwm")) p.slowPwm = server.arg("slowPwm").toInt();
  if (server.hasArg("slowZone")) p.slowZone = server.arg("slowZone").toFloat();
  if (server.hasArg("kpStraight")) p.kpStraight = server.arg("kpStraight").toFloat();

  if (server.hasArg("turnStartL")) p.turnStartL = server.arg("turnStartL").toInt();
  if (server.hasArg("turnStartR")) p.turnStartR = server.arg("turnStartR").toInt();
  if (server.hasArg("turnEndL")) p.turnEndL = server.arg("turnEndL").toInt();
  if (server.hasArg("turnEndR")) p.turnEndR = server.arg("turnEndR").toInt();

  if (server.hasArg("avance2StartL")) p.avance2StartL = server.arg("avance2StartL").toInt();
  if (server.hasArg("avance2StartR")) p.avance2StartR = server.arg("avance2StartR").toInt();
  if (server.hasArg("avance2EndL")) p.avance2EndL = server.arg("avance2EndL").toInt();
  if (server.hasArg("avance2EndR")) p.avance2EndR = server.arg("avance2EndR").toInt();

  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  String json = "{";
  json += "\"state\":\"" + stateName() + "\",";
  json += "\"distL\":" + String(getLeftDistanceCm(), 3) + ",";
  json += "\"distR\":" + String(getRightDistanceCm(), 3) + ",";
  json += "\"distAbs\":" + String(distAbs(), 3);
  json += "}";

  server.send(200, "application/json", json);
}

void handleStart() {
  startSequence();
  server.send(200, "text/plain", "START");
}

void handleStop() {
  stopRobot();
  server.send(200, "text/plain", "STOP");
}

void handleReset() {
  resetRobot();
  server.send(200, "text/plain", "RESET");
}

void handleSave() {
  saveParams();
  server.send(200, "text/plain", "SAVED");
}

void handleManual() {
  int l = server.hasArg("l") ? server.arg("l").toInt() : 0;
  int r = server.hasArg("r") ? server.arg("r").toInt() : 0;

  state = MANUAL;
  setMotors(l, r);

  server.send(200, "text/plain", "MANUAL");
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/api/config", handleConfig);
  server.on("/api/set", handleSet);
  server.on("/api/status", handleStatus);
  server.on("/api/start", handleStart);
  server.on("/api/stop", handleStop);
  server.on("/api/reset", handleReset);
  server.on("/api/save", handleSave);
  server.on("/api/manual", handleManual);

  server.begin();
}

// ==================================================
// TELEMETRIE SERIE
// ==================================================
void printTelemetry(unsigned long now) {
  if (now - lastPrint >= 200) {
    lastPrint = now;

    Serial.print("state=");
    Serial.print(stateName());
    Serial.print(" | L=");
    Serial.print(getLeftDistanceCm(), 2);
    Serial.print(" | R=");
    Serial.print(getRightDistanceCm(), 2);
    Serial.print(" | distAbs=");
    Serial.println(distAbs(), 2);
  }
}

// ==================================================
// SETUP
// ==================================================
void setup() {
  Serial.begin(115200);

  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  initMotors();
  initEncoders();
  resetEncoders();

  loadParams();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println("=== DRAWBOT WEB CONTROL ===");
  Serial.print("WiFi : ");
  Serial.println(AP_SSID);
  Serial.print("Mot de passe : ");
  Serial.println(AP_PASS);
  Serial.print("Adresse page web : http://");
  Serial.println(WiFi.softAPIP());

  setupWebServer();

  lastEncoderUpdate = millis();
  lastPrint = millis();
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  unsigned long now = millis();

  server.handleClient();
  updateEncoderTask(now);
  printTelemetry(now);

  switch (state) {
    case IDLE:
      stopMotors();
      break;

    case AVANCE_1:
      updateAvance(now);

      if (isAvanceTermine()) {
        stopMotors();
        delay(10);

        resetEncoders();
        Serial.println("PHASE 2 : COUDE");
        state = TURN_CURVE;
      }
      break;

    case TURN_CURVE: {
      float d = distAbs();

      if (d < 1.0) {
        setMotors(p.turnStartL, p.turnStartR);
      } else {
        setMotors(p.turnEndL, p.turnEndR);
      }

      if (d >= p.turnDist) {
        stopMotors();
        delay(10);

        resetEncoders();
        Serial.println("PHASE 3 : AVANCE 2");
        state = AVANCE_2;
      }
      break;
    }

    case AVANCE_2: {
      float d = distAbs();

      if (d < 1.0) {
        setMotors(p.avance2StartL, p.avance2StartR);
      } else {
        setMotors(p.avance2EndL, p.avance2EndR);
      }

      if (d >= p.dist2) {
        stopMotors();
        Serial.println("SEQUENCE TERMINEE");
        state = FINISHED;
      }
      break;
    }

    case FINISHED:
      stopMotors();
      break;

    case MANUAL:
      // Les moteurs restent avec la dernière commande manuelle.
      break;
  }
}
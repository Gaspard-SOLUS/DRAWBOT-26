#pragma once
#include <Arduino.h>

const char WEB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Drawbot - Soutenance 1</title>

<style>
:root {
  --bg: #0f172a;
  --card: #111827;
  --panel: #020617;
  --border: #1e293b;
  --text: #e5e7eb;
  --muted: #94a3b8;
  --accent: #38bdf8;
  --green: #22c55e;
  --red: #ef4444;
  --orange: #f97316;
  --purple: #a855f7;
}

* {
  box-sizing: border-box;
}

body {
  margin: 0;
  background: var(--bg);
  color: var(--text);
  font-family: Arial, Helvetica, sans-serif;
}

.app {
  display: grid;
  grid-template-columns: 1fr 360px;
  min-height: 100vh;
}

main {
  padding: 26px;
}

.sidebar {
  background: var(--panel);
  border-left: 1px solid var(--border);
  padding: 22px;
  height: 100vh;
  position: sticky;
  top: 0;
  overflow-y: auto;
}

h1 {
  margin: 0;
  font-size: 30px;
}

.subtitle {
  margin-top: 6px;
  margin-bottom: 24px;
  color: var(--muted);
}

.section-title {
  margin-top: 28px;
  margin-bottom: 12px;
  font-size: 22px;
}

.grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
  gap: 18px;
}

.card {
  background: var(--card);
  border: 1px solid var(--border);
  border-radius: 18px;
  padding: 18px;
  box-shadow: 0 14px 32px rgba(0,0,0,0.25);
}

.card h2 {
  margin: 0 0 12px 0;
  font-size: 18px;
}

.card p {
  color: var(--muted);
  margin-top: 0;
  line-height: 1.45;
}

.value {
  display: flex;
  justify-content: space-between;
  gap: 12px;
  padding: 8px 0;
  border-bottom: 1px solid rgba(148,163,184,0.15);
}

.value:last-child {
  border-bottom: none;
}

.value span:first-child {
  color: var(--muted);
}

.value span:last-child {
  font-family: Consolas, monospace;
  font-weight: bold;
}

.badge {
  display: inline-block;
  border-radius: 999px;
  padding: 5px 10px;
  font-weight: bold;
  font-size: 12px;
}

.ok {
  color: #052e16;
  background: var(--green);
}

.warn {
  color: #431407;
  background: var(--orange);
}

.off {
  color: white;
  background: #475569;
}

button {
  width: 100%;
  border: none;
  border-radius: 12px;
  padding: 12px 14px;
  margin: 5px 0;
  font-weight: bold;
  color: white;
  cursor: pointer;
}

.btn-blue { background: var(--accent); color: #082f49; }
.btn-green { background: var(--green); }
.btn-red { background: var(--red); }
.btn-orange { background: var(--orange); }
.btn-purple { background: var(--purple); }
.btn-dark { background: #334155; }

.manual-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 8px;
  margin: 12px 0;
}

.manual-grid button {
  margin: 0;
  min-height: 48px;
  font-size: 18px;
}

label {
  display: block;
  color: var(--muted);
  margin-top: 12px;
  margin-bottom: 6px;
}

input {
  width: 100%;
  border-radius: 12px;
  border: 1px solid var(--border);
  background: #020617;
  color: var(--text);
  padding: 11px;
  font-size: 15px;
}

.console {
  background: #020617;
  border: 1px solid var(--border);
  border-radius: 14px;
  padding: 12px;
  height: 260px;
  overflow-y: auto;
  font-family: Consolas, monospace;
  font-size: 13px;
  color: #a7f3d0;
  white-space: pre-wrap;
}

.command-log {
  background: #020617;
  border: 1px solid var(--border);
  border-radius: 14px;
  padding: 12px;
  height: 160px;
  overflow-y: auto;
  font-family: Consolas, monospace;
  font-size: 13px;
  color: #bae6fd;
}

.slider-row {
  display: grid;
  grid-template-columns: 1fr 70px;
  gap: 10px;
  align-items: center;
}

input[type="range"] {
  padding: 0;
}

@media(max-width: 1000px) {
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
  <h1>Drawbot -  Tests unitaires</h1>
  <div class="subtitle">
    Soutenance 1 — pilotage, capteurs, encodeurs, position, orientation et console de debug.
  </div>

  <h2 class="section-title">État général du système</h2>
  <div class="grid">
    <section class="card">
      <h2>Interface graphique</h2>
      <p>Vérification que le navigateur communique correctement avec l’ESP32.</p>
      <div class="value"><span>Connexion web</span><span id="webStatus" class="badge off">---</span></div>
      <div class="value"><span>IP ESP32</span><span id="ip">---</span></div>
      <div class="value"><span>État robot</span><span id="state">---</span></div>
      <div class="value"><span>Temps depuis démarrage</span><span id="uptime">---</span></div>
    </section>

    <section class="card">
      <h2>Commandes moteurs actuelles</h2>
      <p>Valeurs PWM réellement envoyées aux moteurs gauche et droit.</p>
      <div class="value"><span>PWM gauche</span><span id="pwmL">0</span></div>
      <div class="value"><span>PWM droite</span><span id="pwmR">0</span></div>
      <div class="value"><span>Mode</span><span id="mode">---</span></div>
    </section>
  </div>

  <h2 class="section-title">Tests unitaires capteurs</h2>
  <div class="grid">
    <section class="card">
      <h2>IMU - données brutes</h2>
      <p>Accéléromètre et gyroscope issus du LSM6DS3.</p>
      <div class="value"><span>Acc X</span><span id="accX">---</span></div>
      <div class="value"><span>Acc Y</span><span id="accY">---</span></div>
      <div class="value"><span>Acc Z</span><span id="accZ">---</span></div>
      <div class="value"><span>Gyro X</span><span id="gyroX">---</span></div>
      <div class="value"><span>Gyro Y</span><span id="gyroY">---</span></div>
      <div class="value"><span>Gyro Z</span><span id="gyroZ">---</span></div>
    </section>

    <section class="card">
      <h2>Magnétomètre - données brutes</h2>
      <p>Données brutes et angle de cap calculé avec correction offset/scale.</p>
      <div class="value"><span>Mag X corrigé</span><span id="magX">---</span></div>
      <div class="value"><span>Mag Y corrigé</span><span id="magY">---</span></div>
      <div class="value"><span>Mag Z</span><span id="magZ">---</span></div>
      <div class="value"><span>Cap magnétique</span><span id="headingMag">---</span></div>
      <div class="value"><span>Calibration</span><span id="magCalib">---</span></div>
    </section>

    <section class="card">
      <h2>Encodeurs</h2>
      <p>Ticks, distances et vitesses de rotation équivalentes des roues.</p>
      <div class="value"><span>Ticks gauche</span><span id="ticksL">---</span></div>
      <div class="value"><span>Ticks droite</span><span id="ticksR">---</span></div>
      <div class="value"><span>Distance gauche</span><span id="distL">---</span></div>
      <div class="value"><span>Distance droite</span><span id="distR">---</span></div>
      <div class="value"><span>Vitesse gauche</span><span id="speedL">---</span></div>
      <div class="value"><span>Vitesse droite</span><span id="speedR">---</span></div>
    </section>
  </div>

  <h2 class="section-title">Position et orientation</h2>
  <div class="grid">
    <section class="card">
      <h2>Position / orientation par encodeurs</h2>
      <p>Odométrie différentielle calculée à partir des distances gauche/droite.</p>
      <div class="value"><span>X stylo estimé</span><span id="odoX">---</span></div>
      <div class="value"><span>Y stylo estimé</span><span id="odoY">---</span></div>
      <div class="value"><span>Orientation encodeurs</span><span id="odoTheta">---</span></div>
    </section>

    <section class="card">
      <h2>Orientation par IMU</h2>
      <p>Yaw intégré avec le gyroscope Z. Utile pour visualiser la dérive.</p>
      <div class="value"><span>Yaw gyro intégré</span><span id="yawGyro">---</span></div>
      <div class="value"><span>Cap magnétique</span><span id="headingMag2">---</span></div>
    </section>
  </div>

  <h2 class="section-title">Console ESP32</h2>
  <section class="card">
    <p>démarrage, commandes reçues, erreurs capteurs, calibration, reset.</p>
    <div id="console" class="console">Chargement de la console...</div>
  </section>
</main>

<aside class="sidebar">
  <h2>Commandes robot</h2>

  <label>PWM manuel</label>
  <div class="slider-row">
    <input id="manualPwm" type="range" min="80" max="255" value="170">
    <input id="manualPwmNumber" type="number" min="80" max="255" value="170">
  </div>

  <div class="manual-grid">
    <div></div>
    <button class="btn-dark" onclick="drive('forward')">↑</button>
    <div></div>

    <button class="btn-dark" onclick="drive('left')">←</button>
    <button class="btn-red" onclick="drive('stop')">■</button>
    <button class="btn-dark" onclick="drive('right')">→</button>

    <div></div>
    <button class="btn-dark" onclick="drive('backward')">↓</button>
    <div></div>
  </div>

  <button class="btn-red" onclick="cmd('/api/stop')">STOP moteurs</button>
  <button class="btn-blue" onclick="cmd('/api/reset-encoders')">RESET encodeurs</button>
  <button class="btn-purple" onclick="cmd('/api/reset-imu')">RESET orientation IMU</button>
  <button class="btn-green" onclick="cmd('/api/calibrate-mag')">Calibration magnétomètre 20 s</button>

  <h2>Tests rapides</h2>
  <button class="btn-orange" onclick="testForward()">Test avancer 1 s</button>
  <button class="btn-orange" onclick="testTurn()">Test rotation 1 s</button>

  <h2>Commandes envoyées</h2>
  <div id="commandLog" class="command-log"></div>
</aside>

</div>

<script>
const pwmSlider = document.getElementById("manualPwm");
const pwmNumber = document.getElementById("manualPwmNumber");

pwmSlider.addEventListener("input", () => {
  pwmNumber.value = pwmSlider.value;
});

pwmNumber.addEventListener("input", () => {
  pwmSlider.value = pwmNumber.value;
});

function addCommandLog(txt) {
  const box = document.getElementById("commandLog");
  const time = new Date().toLocaleTimeString();
  box.innerHTML = `[${time}] ${txt}<br>` + box.innerHTML;
}

async function cmd(route) {
  try {
    await fetch(route);
    addCommandLog(route);
  } catch(e) {
    addCommandLog("ERREUR : " + route);
  }
}

function getPwm() {
  return parseInt(document.getElementById("manualPwm").value);
}

async function drive(direction) {
  const pwm = getPwm();
  let l = 0;
  let r = 0;

  if (direction === "forward") {
    l = pwm;
    r = pwm;
  } else if (direction === "backward") {
    l = -pwm;
    r = -pwm;
  } else if (direction === "left") {
    l = -pwm;
    r = pwm;
  } else if (direction === "right") {
    l = pwm;
    r = -pwm;
  } else {
    l = 0;
    r = 0;
  }

  await fetch(`/api/manual?l=${l}&r=${r}`);
  addCommandLog(`manuel l=${l} r=${r}`);
}

async function testForward() {
  const pwm = getPwm();
  await fetch(`/api/manual?l=${pwm}&r=${pwm}`);
  addCommandLog(`test avancer 1s pwm=${pwm}`);
  setTimeout(() => drive("stop"), 1000);
}

async function testTurn() {
  const pwm = getPwm();
  await fetch(`/api/manual?l=${-pwm}&r=${pwm}`);
  addCommandLog(`test rotation 1s pwm=${pwm}`);
  setTimeout(() => drive("stop"), 1000);
}

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function fmt(value, digits = 2) {
  if (value === null || value === undefined || isNaN(value)) return "---";
  return Number(value).toFixed(digits);
}

async function refreshStatus() {
  try {
    const res = await fetch("/api/status");
    const d = await res.json();

    document.getElementById("webStatus").textContent = "OK";
    document.getElementById("webStatus").className = "badge ok";

    setText("ip", d.ip);
    setText("state", d.state);
    setText("mode", d.mode);
    setText("uptime", fmt(d.uptime, 1) + " s");

    setText("pwmL", d.pwmL);
    setText("pwmR", d.pwmR);

    setText("accX", fmt(d.accX, 3) + " g");
    setText("accY", fmt(d.accY, 3) + " g");
    setText("accZ", fmt(d.accZ, 3) + " g");

    setText("gyroX", fmt(d.gyroX, 2) + " °/s");
    setText("gyroY", fmt(d.gyroY, 2) + " °/s");
    setText("gyroZ", fmt(d.gyroZ, 2) + " °/s");

    setText("magX", fmt(d.magX, 2) + " µT");
    setText("magY", fmt(d.magY, 2) + " µT");
    setText("magZ", fmt(d.magZ, 2) + " µT");
    setText("headingMag", fmt(d.headingMag, 1) + "°");
    setText("headingMag2", fmt(d.headingMag, 1) + "°");
    setText("magCalib", d.magCalib);

    setText("ticksL", d.ticksL);
    setText("ticksR", d.ticksR);
    setText("distL", fmt(d.distL, 2) + " cm");
    setText("distR", fmt(d.distR, 2) + " cm");
    setText("speedL", fmt(d.speedL, 2) + " cm/s");
    setText("speedR", fmt(d.speedR, 2) + " cm/s");

    setText("odoX", fmt(d.odoX, 2) + " cm");
    setText("odoY", fmt(d.odoY, 2) + " cm");
    setText("odoTheta", fmt(d.odoTheta, 1) + "°");
    setText("yawGyro", fmt(d.yawGyro, 1) + "°");

  } catch(e) {
    document.getElementById("webStatus").textContent = "OFF";
    document.getElementById("webStatus").className = "badge off";
  }
}

async function refreshConsole() {
  try {
    const res = await fetch("/api/logs");
    const txt = await res.text();
    document.getElementById("console").textContent = txt;
  } catch(e) {
    document.getElementById("console").textContent = "Erreur de lecture console";
  }
}

setInterval(refreshStatus, 200);
setInterval(refreshConsole, 1000);

refreshStatus();
refreshConsole();
</script>

</body>
</html>
)rawliteral";
#include "web_pages.h"

namespace WebPages {
  static String commonHead(const String& title) {
    String html;

    html += "<!DOCTYPE html><html lang='fr'><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>" + title + "</title>";

    html += R"rawliteral(
<style>
:root {
  --bg:#0f172a;
  --card:#111827;
  --panel:#020617;
  --border:#1e293b;
  --text:#e5e7eb;
  --muted:#94a3b8;
  --accent:#38bdf8;
  --green:#22c55e;
  --red:#ef4444;
  --orange:#f97316;
  --purple:#a855f7;
}
* { box-sizing:border-box; }
body {
  margin:0;
  background:var(--bg);
  color:var(--text);
  font-family:Arial, Helvetica, sans-serif;
}
nav {
  display:flex;
  gap:12px;
  padding:16px 24px;
  background:var(--panel);
  border-bottom:1px solid var(--border);
  position:sticky;
  top:0;
  z-index:10;
}
nav a {
  color:var(--text);
  text-decoration:none;
  font-weight:bold;
  background:#1e293b;
  padding:10px 14px;
  border-radius:12px;
}
nav a:hover { background:var(--accent); color:#082f49; }
main { padding:26px; }
h1 { margin-top:0; }
.subtitle { color:var(--muted); margin-bottom:24px; }
.grid {
  display:grid;
  grid-template-columns:repeat(auto-fit, minmax(260px, 1fr));
  gap:18px;
}
.card {
  background:var(--card);
  border:1px solid var(--border);
  border-radius:18px;
  padding:18px;
  box-shadow:0 14px 32px rgba(0,0,0,0.25);
}
.card h2 { margin-top:0; }
.value {
  display:flex;
  justify-content:space-between;
  border-bottom:1px solid rgba(148,163,184,0.15);
  padding:8px 0;
}
.value span:first-child { color:var(--muted); }
.value span:last-child {
  font-family:Consolas, monospace;
  font-weight:bold;
}
button {
  border:none;
  border-radius:12px;
  padding:12px 14px;
  color:white;
  font-weight:bold;
  cursor:pointer;
}
.btn-blue { background:var(--accent); color:#082f49; }
.btn-green { background:var(--green); }
.btn-red { background:var(--red); }
.btn-orange { background:var(--orange); }
.btn-dark { background:#334155; }
.manual-grid {
  display:grid;
  grid-template-columns:repeat(3, 80px);
  gap:8px;
  margin-top:12px;
}
.manual-grid button {
  height:56px;
  font-size:20px;
}
.console {
  background:#020617;
  border:1px solid var(--border);
  border-radius:14px;
  padding:12px;
  height:260px;
  overflow-y:auto;
  font-family:Consolas, monospace;
  font-size:13px;
  color:#a7f3d0;
  white-space:pre-wrap;
}
input {
  padding:10px;
  border-radius:10px;
  border:1px solid var(--border);
  background:#020617;
  color:var(--text);
}
</style>
)rawliteral";

    html += "</head><body>";

    html += R"rawliteral(
<nav>
  <a href="/">Accueil</a>
  <a href="/soutenance1">Soutenance 1</a>
  <a href="/soutenance2">Soutenance 2</a>
</nav>
)rawliteral";

    return html;
  }

  static String commonScript() {
    return R"rawliteral(
<script>
function getPwm() {
  const input = document.getElementById("manualPwm");
  return input ? parseInt(input.value) : 170;
}

async function drive(direction) {
  const pwm = getPwm();
  let l = 0;
  let r = 0;

  if (direction === "forward") { l = pwm; r = pwm; }
  else if (direction === "backward") { l = -pwm; r = -pwm; }
  else if (direction === "left") { l = -pwm; r = pwm; }
  else if (direction === "right") { l = pwm; r = -pwm; }

  await fetch(`/api/manual?l=${l}&r=${r}`);
}

async function api(route) {
  await fetch(route);
}

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function fmt(v, d = 2) {
  if (v === null || v === undefined || isNaN(v)) return "---";
  return Number(v).toFixed(d);
}

async function refreshStatus() {
  try {
    const res = await fetch("/api/status");
    const d = await res.json();

    setText("state", d.state);
    setText("mode", d.mode);
    setText("ip", d.ip);
    setText("uptime", fmt(d.uptime, 1) + " s");

    setText("pwmL", d.pwmL);
    setText("pwmR", d.pwmR);

    setText("ticksL", d.ticksL);
    setText("ticksR", d.ticksR);
    setText("distL", fmt(d.distL, 2) + " cm");
    setText("distR", fmt(d.distR, 2) + " cm");
    setText("speedL", fmt(d.speedL, 2) + " cm/s");
    setText("speedR", fmt(d.speedR, 2) + " cm/s");

    setText("accX", fmt(d.accX, 3) + " g");
    setText("accY", fmt(d.accY, 3) + " g");
    setText("accZ", fmt(d.accZ, 3) + " g");

    setText("gyroX", fmt(d.gyroX, 2) + " °/s");
    setText("gyroY", fmt(d.gyroY, 2) + " °/s");
    setText("gyroZ", fmt(d.gyroZ, 2) + " °/s");
    setText("yawGyro", fmt(d.yawGyro, 1) + "°");

    setText("magX", fmt(d.magX, 2) + " µT");
    setText("magY", fmt(d.magY, 2) + " µT");
    setText("magZ", fmt(d.magZ, 2) + " µT");
    setText("headingMag", fmt(d.headingMag, 1) + "°");
    setText("magCalib", d.magCalib);

    setText("odoX", fmt(d.odoX, 2) + " cm");
    setText("odoY", fmt(d.odoY, 2) + " cm");
    setText("odoTheta", fmt(d.odoTheta, 1) + "°");

  } catch(e) {}
}

async function refreshLogs() {
  const box = document.getElementById("console");
  if (!box) return;

  try {
    const res = await fetch("/api/logs");
    box.textContent = await res.text();
  } catch(e) {
    box.textContent = "Erreur lecture console";
  }
}

setInterval(refreshStatus, 200);
setInterval(refreshLogs, 1000);
refreshStatus();
refreshLogs();
</script>
)rawliteral";
  }

  static String manualControls() {
    return R"rawliteral(
<section class="card">
  <h2>Commandes de base</h2>
  <p class="subtitle">Pilotage manuel du robot depuis la page web.</p>

  <label>PWM manuel : </label>
  <input id="manualPwm" type="number" value="170" min="80" max="255">

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

  <br>
  <button class="btn-red" onclick="api('/api/stop')">STOP moteurs</button>
  <button class="btn-blue" onclick="api('/api/reset-encoders')">RESET encodeurs</button>
  <button class="btn-blue" onclick="api('/api/reset-imu')">RESET IMU</button>
</section>
)rawliteral";
  }

  String home() {
    String html = commonHead("Drawbot - Accueil");

    html += R"rawliteral(
<main>
  <h1>Drawbot - Interface principale</h1>
  <p class="subtitle">
    Interface de contrôle du robot Drawbot : pilotage manuel, mesures capteurs,
    odométrie, Teleplot et préparation soutenance 2.
  </p>

  <div class="grid">
)rawliteral";

    html += manualControls();

    html += R"rawliteral(
    <section class="card">
      <h2>État rapide</h2>
      <div class="value"><span>IP ESP32</span><span id="ip">---</span></div>
      <div class="value"><span>État robot</span><span id="state">---</span></div>
      <div class="value"><span>Mode</span><span id="mode">---</span></div>
      <div class="value"><span>PWM gauche</span><span id="pwmL">---</span></div>
      <div class="value"><span>PWM droite</span><span id="pwmR">---</span></div>
      <div class="value"><span>Uptime</span><span id="uptime">---</span></div>
    </section>

    <section class="card">
      <h2>Accès rapides</h2>
      <p>Utilise ces pages pour présenter chaque étape du projet.</p>
      <p><a href="/soutenance1">→ Page soutenance 1 : tests unitaires</a></p>
      <p><a href="/soutenance2">→ Page soutenance 2 : séquences finales</a></p>
    </section>
  </div>
</main>
)rawliteral";

    html += commonScript();
    html += "</body></html>";
    return html;
  }

  String soutenance1() {
    String html = commonHead("Drawbot - Soutenance 1");

    html += R"rawliteral(
<main>
  <h1>Soutenance 1 - Tests unitaires</h1>
  <p class="subtitle">
    Cette page sert à prouver le fonctionnement des commandes, des encodeurs,
    de l'IMU, du magnétomètre, de l'odométrie et de Teleplot.
  </p>

  <div class="grid">
)rawliteral";

    html += manualControls();

    html += R"rawliteral(
    <section class="card">
      <h2>Encodeurs</h2>
      <div class="value"><span>Ticks gauche</span><span id="ticksL">---</span></div>
      <div class="value"><span>Ticks droite</span><span id="ticksR">---</span></div>
      <div class="value"><span>Distance gauche</span><span id="distL">---</span></div>
      <div class="value"><span>Distance droite</span><span id="distR">---</span></div>
      <div class="value"><span>Vitesse gauche</span><span id="speedL">---</span></div>
      <div class="value"><span>Vitesse droite</span><span id="speedR">---</span></div>
    </section>

    <section class="card">
      <h2>IMU</h2>
      <div class="value"><span>Acc X</span><span id="accX">---</span></div>
      <div class="value"><span>Acc Y</span><span id="accY">---</span></div>
      <div class="value"><span>Acc Z</span><span id="accZ">---</span></div>
      <div class="value"><span>Gyro X</span><span id="gyroX">---</span></div>
      <div class="value"><span>Gyro Y</span><span id="gyroY">---</span></div>
      <div class="value"><span>Gyro Z</span><span id="gyroZ">---</span></div>
      <div class="value"><span>Yaw intégré</span><span id="yawGyro">---</span></div>
    </section>

    <section class="card">
      <h2>Magnétomètre</h2>
      <button class="btn-green" onclick="api('/api/calibrate-mag')">Calibration 20 secondes</button>
      <div class="value"><span>Mag X</span><span id="magX">---</span></div>
      <div class="value"><span>Mag Y</span><span id="magY">---</span></div>
      <div class="value"><span>Mag Z</span><span id="magZ">---</span></div>
      <div class="value"><span>Cap magnétique</span><span id="headingMag">---</span></div>
      <div class="value"><span>Calibration</span><span id="magCalib">---</span></div>
    </section>

    <section class="card">
      <h2>Odométrie</h2>
      <div class="value"><span>X</span><span id="odoX">---</span></div>
      <div class="value"><span>Y</span><span id="odoY">---</span></div>
      <div class="value"><span>Theta</span><span id="odoTheta">---</span></div>
    </section>

    <section class="card">
      <h2>Console ESP32</h2>
      <div id="console" class="console">Chargement...</div>
    </section>
  </div>
</main>
)rawliteral";

    html += commonScript();
    html += "</body></html>";
    return html;
  }

  String soutenance2() {
    String html = commonHead("Drawbot - Soutenance 2");

    html += R"rawliteral(
<main>
  <h1>Soutenance 2 - Séquences finales</h1>
  <p class="subtitle">
    Page prévue pour intégrer les séquences finales : escalier, cercle,
    rose des vents, paramètres, lancement et validation.
  </p>

  <div class="grid">
)rawliteral";

    html += manualControls();

    html += R"rawliteral(
    <section class="card">
      <h2>Zone de développement soutenance 2</h2>
      <p>
        Ici tu pourras ajouter les boutons de lancement des séquences :
        escalier, cercle paramétrable, rose des vents, etc.
      </p>
      <button class="btn-orange" onclick="api('/api/s2/test')">Test soutenance 2</button>
    </section>

    <section class="card">
      <h2>État robot</h2>
      <div class="value"><span>État</span><span id="state">---</span></div>
      <div class="value"><span>Mode</span><span id="mode">---</span></div>
      <div class="value"><span>PWM gauche</span><span id="pwmL">---</span></div>
      <div class="value"><span>PWM droite</span><span id="pwmR">---</span></div>
      <div class="value"><span>X</span><span id="odoX">---</span></div>
      <div class="value"><span>Y</span><span id="odoY">---</span></div>
      <div class="value"><span>Theta</span><span id="odoTheta">---</span></div>
    </section>

    <section class="card">
      <h2>Console ESP32</h2>
      <div id="console" class="console">Chargement...</div>
    </section>
  </div>
</main>
)rawliteral";

    html += commonScript();
    html += "</body></html>";
    return html;
  }
}
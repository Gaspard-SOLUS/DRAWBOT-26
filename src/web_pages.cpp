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
.btn-purple { background:var(--purple); color:white; }
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
  <button class="btn-blue" onclick="api('/api/reset-encoders')">RESET encodeurs</button><br><br>
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
    odométrie, Teleplot et soutenance 2.
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
      <p>Acceder à chaque étape du projet.</p>
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
      Cette page regroupe les trois séquences finales du cahier des charges.
    </p>

    <div class="grid">
      <section class="card">
        <h2>Séquence 1 : l'escalier</h2>
        <p>
          Objectif : avancer, tourner à gauche, avancer, tourner à droite,
          puis avancer à nouveau afin de dessiner un escalier.
        </p>
        <p>
          Contraintes : distances précises et angles proches de 90°.
        </p>
        <a href="/soutenance2/escalier">
          <button class="btn-orange">Ouvrir la page escalier</button>
        </a>
      </section>

      <section class="card">
        <h2>Séquence 2 : le cercle</h2>
        <p>
          Objectif : dessiner un cercle dont le rayon est paramétrable
          depuis l'interface web.
        </p>
        <p>
          Rayon demandé entre 14 cm et 20 cm pour le mode cercle continu.
        </p>
        <a href="/soutenance2/cercle">
          <button class="btn-green">Ouvrir la page cercle</button>
        </a>
      </section>

      <section class="card">
        <h2>Séquence 3 : rose des vents</h2>
        <p>
          Objectif : dessiner une flèche ou une rose des vents orientée
          vers le Nord terrestre grâce au magnétomètre.
        </p>
        <p>
          Cette séquence utilise la calibration magnétomètre.
        </p>
        <a href="/soutenance2/rose-des-vents">
          <button class="btn-purple">Ouvrir la page rose des vents</button>
        </a>
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

  String soutenance2Escalier() {
    String html = commonHead("Drawbot - Séquence escalier");

    html += R"rawliteral(
  <main>
    <h1>Soutenance 2 - Séquence 1 : l'escalier</h1>
    <p class="subtitle">
      Suivi de trajectoire du point stylo : une seule ligne géométrique par segment, avec transition contrôlée aux coins.
    </p>

    <p>
      <a href="/soutenance2">← Retour soutenance 2</a>
    </p>

    <div class="grid">
      <section class="card">
        <h2>1. Géométrie de l'escalier</h2>
        <label>Segment 1 - cm</label>
        <input id="dist1Cm" type="number" value="20" step="0.1"><br>
        <label>Segment 2 - cm</label>
        <input id="dist2Cm" type="number" value="10" step="0.1"><br>
        <label>Segment 3 - cm</label>
        <input id="dist3Cm" type="number" value="40" step="0.1"><br>
        <label>Écartement roues - cm</label>
        <input id="wheelBaseCm" type="number" value="8.3" step="0.1"><br>
        <label>Offset stylo - cm</label>
        <input id="penOffsetCm" type="number" value="13" step="0.1">
      </section>

      <section class="card">
        <h2>2. Vitesse et transitions</h2>
        <label>Vitesse stylo - cm/s</label>
        <input id="penSpeedCms" type="number" value="2.0" step="0.1"><br>
        <label>Zone ralentissement - cm</label>
        <input id="slowZoneCm" type="number" value="3.0" step="0.1"><br>
        <label>Tolérance fin segment - cm</label>
        <input id="endToleranceCm" type="number" value="0.35" step="0.05"><br>
        <label>Pause au coin - ms</label>
        <input id="cornerPauseMs" type="number" value="80" step="10"><br>
        <label>Zone entrée après coin - cm</label>
        <input id="entryZoneCm" type="number" value="1.5" step="0.1"><br>
        <label>Facteur vitesse entrée</label>
        <input id="entrySpeedScale" type="number" value="0.65" step="0.05"><br>
        <label>Facteur correction entrée</label>
        <input id="entryCorrectionScale" type="number" value="0.55" step="0.05">
      </section>

      <section class="card">
        <h2>3. Moteurs / PWM</h2>
        <label>PWM minimum</label>
        <input id="minPwm" type="number" value="145" step="1"><br>
        <label>PWM maximum</label>
        <input id="maxPwm" type="number" value="205" step="1"><br>
        <label>Vitesse roue max estimée - cm/s</label>
        <input id="maxWheelSpeedCms" type="number" value="18" step="0.5"><br>
        <label>Rampe PWM par cycle</label>
        <input id="pwmRampStep" type="number" value="45" step="1">
      </section>

      <section class="card">
        <h2>4. PID latéral du stylo</h2>
        <label>Kp latéral</label>
        <input id="kpLat" type="number" value="0.45" step="0.01"><br>
        <label>Ki latéral</label>
        <input id="kiLat" type="number" value="0" step="0.01"><br>
        <label>Kd latéral</label>
        <input id="kdLat" type="number" value="0.08" step="0.01"><br>
        <label>Correction latérale max - cm/s</label>
        <input id="maxLatCorrectionCms" type="number" value="1.6" step="0.1"><br>
        <label>Limite intégrale</label>
        <input id="integralLimit" type="number" value="10" step="0.5"><br>
        <label>Source orientation : 0 odométrie, 1 gyro, 2 magnéto</label>
        <input id="headingSource" type="number" value="0" min="0" max="2" step="1">
      </section>

      <section class="card">
        <h2>Commandes escalier</h2>
        <button class="btn-blue" onclick="loadEscalierConfig()">Charger réglages</button>
        <button class="btn-blue" onclick="applyEscalierConfig()">Appliquer réglages</button>
        <button class="btn-green" onclick="saveEscalierConfig()">Enregistrer en mémoire</button>
        <button class="btn-green" onclick="startStair()">Lancer escalier</button>
        <button class="btn-red" onclick="api('/api/s2/escalier/stop')">STOP escalier</button>
        <button class="btn-red" onclick="api('/api/stop')">STOP moteurs</button>
        <button class="btn-blue" onclick="api('/api/reset-encoders')">RESET odométrie</button>
      </section>

      <section class="card">
        <h2>État escalier</h2>
        <div class="value"><span>État escalier</span><span id="escState">---</span></div>
        <div class="value"><span>Segment</span><span id="escSegment">---</span></div>
        <div class="value"><span>Progression</span><span id="escProgress">---</span></div>
        <div class="value"><span>Restant</span><span id="escRemaining">---</span></div>
        <div class="value"><span>Erreur latérale</span><span id="escError">---</span></div>
        <div class="value"><span>Zone entrée</span><span id="escEntry">---</span></div>
        <div class="value"><span>PWM gauche</span><span id="escPwmL">---</span></div>
        <div class="value"><span>PWM droite</span><span id="escPwmR">---</span></div>
      </section>

      <section class="card">
        <h2>État robot</h2>
        <div class="value"><span>État</span><span id="state">---</span></div>
        <div class="value"><span>Mode</span><span id="mode">---</span></div>
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

  <script>
  const escFields = [
    "dist1Cm", "dist2Cm", "dist3Cm",
    "wheelBaseCm", "penOffsetCm",
    "penSpeedCms", "slowZoneCm", "endToleranceCm",
    "entryZoneCm", "entrySpeedScale", "entryCorrectionScale", "cornerPauseMs",
    "minPwm", "maxPwm", "maxWheelSpeedCms", "pwmRampStep",
    "kpLat", "kiLat", "kdLat", "maxLatCorrectionCms", "integralLimit", "headingSource"
  ];

  function escParams() {
    const params = new URLSearchParams();
    escFields.forEach(id => params.append(id, document.getElementById(id).value));
    return params;
  }

  async function loadEscalierConfig() {
    const res = await fetch('/api/s2/escalier/config');
    const data = await res.json();
    escFields.forEach(id => {
      if (data[id] !== undefined) document.getElementById(id).value = data[id];
    });
  }

  async function applyEscalierConfig() {
    await fetch('/api/s2/escalier/set?' + escParams().toString());
  }

  async function saveEscalierConfig() {
    await applyEscalierConfig();
    await fetch('/api/s2/escalier/save');
  }

  async function startStair() {
    await fetch('/api/s2/escalier/start?' + escParams().toString());
  }

  async function refreshEscalierStatus() {
    try {
      const res = await fetch('/api/s2/escalier/status');
      const d = await res.json();
      document.getElementById('escState').textContent = d.state;
      document.getElementById('escSegment').textContent = (d.segmentIndex + 1);
      document.getElementById('escProgress').textContent = d.progressCm.toFixed(2) + ' cm';
      document.getElementById('escRemaining').textContent = d.remainingCm.toFixed(2) + ' cm';
      document.getElementById('escError').textContent = d.lateralErrorCm.toFixed(2) + ' cm';
      document.getElementById('escEntry').textContent = d.entryZoneActive ? 'OUI' : 'NON';
      document.getElementById('escPwmL').textContent = d.pwmLeft;
      document.getElementById('escPwmR').textContent = d.pwmRight;
    } catch(e) {}
  }

  loadEscalierConfig();
  setInterval(refreshEscalierStatus, 250);
  </script>
  )rawliteral";

    html += commonScript();
    html += "</body></html>";
    return html;
  }

  String soutenance2Cercle() {
    String html = commonHead("Drawbot - Séquence cercle");

    html += R"rawliteral(
  <main>
    <h1>Soutenance 2 - Séquence 2 : cercle</h1>
    <p class="subtitle">
      Mode continu pour rayons 14 à 20 cm : calcul géométrique + PID vitesse sur les deux roues.
    </p>

    <p><a href="/soutenance2">← Retour soutenance 2</a></p>

    <div class="grid">
      <section class="card">
        <h2>1. Géométrie</h2>
        <label>Rayon demandé du cercle stylo en cm</label>
        <input id="radiusCm" type="number" min="14" max="20" step="0.1" value="16">

        <label>Échelle du rayon</label>
        <input id="radiusScale" type="number" min="0.5" max="1.8" step="0.001" value="1.000">

        <label>Offset de correction du rayon en cm</label>
        <input id="radiusOffsetCm" type="number" min="-8" max="8" step="0.1" value="0.0">

        <label>Écartement des roues en cm</label>
        <input id="wheelBaseCm" type="number" min="5" max="15" step="0.1" value="8.3">

        <label>Offset stylo : axe roues → stylo en cm</label>
        <input id="penOffsetCm" type="number" min="5" max="20" step="0.1" value="13">

        <label>Sens du cercle</label>
        <select id="direction">
          <option value="1">Gauche</option>
          <option value="0">Droite</option>
        </select>
      </section>

      <section class="card">
        <h2>2. Vitesse et fermeture</h2>
        <label>Vitesse roue extérieure en cm/s</label>
        <input id="outerWheelSpeedCms" type="number" min="1" max="20" step="0.1" value="7">

        <label>Facteur distance roues / fermeture</label>
        <input id="closureFactor" type="number" min="0.7" max="1.3" step="0.01" value="1.00">
        <p class="hint">1.00 = distance theorique des roues. A ajuster legerement seulement apres calibration.</p>

        <label>Facteur d'arrêt angulaire</label>
        <input id="stopTurnFactor" type="number" min="0.90" max="1.05" step="0.001" value="1.000">
        <p class="hint">1.000 = arrêt à 360°. Mettre 0.990 si le robot repasse un peu sur le début.</p>

        <label>Ralentissement final activé ?</label>
        <select id="endSlowdownEnabled">
          <option value="0">Non</option>
          <option value="1">Oui</option>
        </select>

        <label>Début ralentissement final</label>
        <input id="endSlowdownStart" type="number" min="0.50" max="0.98" step="0.01" value="0.90">

        <label>Vitesse mini ralentissement final</label>
        <input id="endSlowdownMinScale" type="number" min="0.40" max="1.00" step="0.01" value="0.75">

        <label>PWM minimum</label>
        <input id="minPwm" type="number" min="0" max="255" step="1" value="170">

        <label>PWM maximum</label>
        <input id="maxPwm" type="number" min="0" max="255" step="1" value="250">

        <label>Rampe PWM par cycle</label>
        <input id="pwmRampStep" type="number" min="1" max="255" step="1" value="8">
      </section>

      <section class="card">
        <h2>3. PID vitesse roues</h2>
        <label>Kp vitesse</label>
        <input id="kpSpeed" type="number" step="0.1" value="10">

        <label>Ki vitesse</label>
        <input id="kiSpeed" type="number" step="0.1" value="1.2">

        <label>Kd vitesse</label>
        <input id="kdSpeed" type="number" step="0.01" value="0.00">

        <label>Limite intégrale</label>
        <input id="integralLimit" type="number" step="1" value="20">

        <label>Feed-forward kFF</label>
        <input id="kFF" type="number" step="0.1" value="22">

        <label>Vitesse fiable minimum moteur en cm/s</label>
        <input id="minReliableSpeedCms" type="number" min="0.3" max="8" step="0.1" value="4.5">

        <label>Période impulsions basse vitesse en ms</label>
        <input id="pulsePeriodMs" type="number" min="60" max="1000" step="10" value="120">

        <label>Filtre vitesse encodeurs alpha</label>
        <input id="speedFilterAlpha" type="number" min="0.05" max="1" step="0.01" value="0.30">
        <p class="hint">Baisser vers 0.15 si les grands rayons oscillent encore.</p>

        <label>Correction PID max en PWM</label>
        <input id="maxPidCorrectionPwm" type="number" min="0" max="100" step="1" value="35">

        <label>Zone morte erreur vitesse en cm/s</label>
        <input id="speedDeadbandCms" type="number" min="0" max="2" step="0.05" value="0.25">

        <label>Kp correction ratio distances</label>
        <input id="ratioTrimKp" type="number" min="0" max="120" step="1" value="35">

        <label>Correction ratio max en PWM</label>
        <input id="maxRatioTrimPwm" type="number" min="0" max="120" step="1" value="35">
      </section>

      <section class="card">
        <h2>4. Commandes</h2>
        <button class="btn-blue" onclick="sendCircleConfig()">Appliquer les valeurs</button>
        <button class="btn-green" onclick="saveCircleConfig()">Enregistrer en mémoire</button>
        <button class="btn-purple" onclick="startCircle()">Lancer cercle</button>
        <button class="btn-red" onclick="api('/api/s2/cercle/stop')">STOP cercle</button>
        <button class="btn-orange" onclick="api('/api/reset-encoders')">RESET odométrie</button>
      </section>

      <section class="card">
        <h2>5. Géométrie calculée</h2>
        <div class="value"><span>Rayon demandé stylo</span><span id="requestedRadiusCm">---</span></div>
        <div class="value"><span>Rayon effectif utilisé</span><span id="effectiveRadiusCm">---</span></div>
        <div class="value"><span>Rayon axe robot</span><span id="robotRadiusCm">---</span></div>
        <div class="value"><span>Rayon roue intérieure</span><span id="innerWheelRadiusCm">---</span></div>
        <div class="value"><span>Rayon roue extérieure</span><span id="outerWheelRadiusCm">---</span></div>
        <div class="value"><span>Ratio vitesse int/ext</span><span id="speedRatio">---</span></div>
        <div class="value"><span>Distance cible roue gauche</span><span id="targetLeftDistanceCm">---</span></div>
        <div class="value"><span>Distance cible roue droite</span><span id="targetRightDistanceCm">---</span></div>
        <div class="value"><span>Distance cible roue extérieure</span><span id="targetOuterDistanceCm">---</span></div>
      </section>

      <section class="card">
        <h2>6. État cercle</h2>
        <div class="value"><span>État</span><span id="circleState">---</span></div>
        <div class="value"><span>Progression distance</span><span id="progressPercent">---</span></div>
        <div class="value"><span>Progression angulaire</span><span id="angularProgressPercent">---</span></div>
        <div class="value"><span>Différence roues</span><span id="wheelDistanceDiffCm">---</span></div>
        <div class="value"><span>Différence cible</span><span id="targetWheelDistanceDiffCm">---</span></div>
        <div class="value"><span>Distance gauche</span><span id="leftDistanceCm">---</span></div>
        <div class="value"><span>Distance droite</span><span id="rightDistanceCm">---</span></div>
        <div class="value"><span>Distance extérieure</span><span id="outerDistanceCm">---</span></div>
        <div class="value"><span>Vitesse G cible / mesurée</span><span id="speedL">---</span></div>
        <div class="value"><span>Vitesse D cible / mesurée</span><span id="speedR">---</span></div>
        <div class="value"><span>PWM gauche</span><span id="circlePwmL">---</span></div>
        <div class="value"><span>PWM droite</span><span id="circlePwmR">---</span></div>
        <div class="value"><span>PWM moyen extérieur</span><span id="outerBasePwm">---</span></div>
        <div class="value"><span>PWM moyen intérieur</span><span id="innerAveragePwm">---</span></div>
        <div class="value"><span>Duty impulsions intérieur</span><span id="innerPulseDuty">---</span></div>
        <div class="value"><span>Erreur</span><span id="circleError">---</span></div>
      </section>

      <section class="card">
        <h2>Console ESP32</h2>
        <div id="console" class="console">Chargement...</div>
      </section>
    </div>
  </main>

  <script>
  const circleFields = [
    "radiusCm", "radiusScale", "radiusOffsetCm", "wheelBaseCm", "penOffsetCm", "direction",
    "outerWheelSpeedCms", "closureFactor", "stopTurnFactor", "endSlowdownEnabled", "endSlowdownStart", "endSlowdownMinScale", "minPwm", "maxPwm", "pwmRampStep",
    "kpSpeed", "kiSpeed", "kdSpeed", "integralLimit", "kFF",
    "minReliableSpeedCms", "pulsePeriodMs", "speedFilterAlpha", "maxPidCorrectionPwm", "speedDeadbandCms", "ratioTrimKp", "maxRatioTrimPwm"
  ];

  function setCircleText(id, value) {
    const el = document.getElementById(id);
    if (el) el.textContent = value;
  }

  async function loadCircleConfig() {
    try {
      const res = await fetch('/api/s2/cercle/config');
      const d = await res.json();
      circleFields.forEach(id => {
        const el = document.getElementById(id);
        if (el && d[id] !== undefined) el.value = d[id];
      });
    } catch(e) {}
  }

  async function sendCircleConfig() {
    const params = new URLSearchParams();
    circleFields.forEach(id => {
      const el = document.getElementById(id);
      if (el) params.append(id, el.value);
    });
    await fetch('/api/s2/cercle/set?' + params.toString());
  }

  async function saveCircleConfig() {
    await sendCircleConfig();
    await fetch('/api/s2/cercle/save');
  }

  async function startCircle() {
    await sendCircleConfig();
    await fetch('/api/s2/cercle/start');
  }

  async function refreshCircleStatus() {
    try {
      const res = await fetch('/api/s2/cercle/status');
      const d = await res.json();
      setCircleText('circleState', d.state);
      setCircleText('requestedRadiusCm', d.requestedRadiusCm.toFixed(2) + ' cm');
      setCircleText('effectiveRadiusCm', d.effectiveRadiusCm.toFixed(2) + ' cm');
      setCircleText('robotRadiusCm', d.robotRadiusCm.toFixed(2) + ' cm');
      setCircleText('innerWheelRadiusCm', d.innerWheelRadiusCm.toFixed(2) + ' cm');
      setCircleText('outerWheelRadiusCm', d.outerWheelRadiusCm.toFixed(2) + ' cm');
      setCircleText('speedRatio', d.speedRatio.toFixed(3));
      setCircleText('targetLeftDistanceCm', d.targetLeftDistanceCm.toFixed(2) + ' cm');
      setCircleText('targetRightDistanceCm', d.targetRightDistanceCm.toFixed(2) + ' cm');
      setCircleText('targetOuterDistanceCm', d.targetOuterDistanceCm.toFixed(2) + ' cm');
      setCircleText('progressPercent', d.progressPercent.toFixed(1) + ' %');
      setCircleText('angularProgressPercent', d.angularProgressPercent.toFixed(1) + ' %');
      setCircleText('wheelDistanceDiffCm', d.wheelDistanceDiffCm.toFixed(2) + ' cm');
      setCircleText('targetWheelDistanceDiffCm', d.targetWheelDistanceDiffCm.toFixed(2) + ' cm');
      setCircleText('leftDistanceCm', d.leftDistanceCm.toFixed(2) + ' cm');
      setCircleText('rightDistanceCm', d.rightDistanceCm.toFixed(2) + ' cm');
      setCircleText('outerDistanceCm', d.outerDistanceCm.toFixed(2) + ' cm');
      setCircleText('speedL', d.targetLeftSpeedCms.toFixed(2) + ' / ' + d.measuredLeftSpeedCms.toFixed(2));
      setCircleText('speedR', d.targetRightSpeedCms.toFixed(2) + ' / ' + d.measuredRightSpeedCms.toFixed(2));
      setCircleText('circlePwmL', d.pwmLeft);
      setCircleText('circlePwmR', d.pwmRight);
      setCircleText('outerBasePwm', d.outerBasePwm.toFixed(1));
      setCircleText('innerAveragePwm', d.innerAveragePwm.toFixed(1));
      setCircleText('innerPulseDuty', (100*d.innerPulseDuty).toFixed(0) + ' %');
      setCircleText('circleError', d.errorMessage || '-');
    } catch(e) {}
  }

  loadCircleConfig();
  setInterval(refreshCircleStatus, 250);
  </script>
  )rawliteral";

    html += commonScript();
    html += "</body></html>";
    return html;
  }

  String soutenance2RoseDesVents() {
    String html = commonHead("Drawbot - Rose des vents");

    html += R"rawliteral(
  <main>
    <h1>Soutenance 2 - Séquence 3 : rose des vents</h1>
    <p class="subtitle">
      Orientation du robot vers le Nord grâce au magnétomètre.
    </p>

    <p>
      <a href="/soutenance2">← Retour soutenance 2</a>
    </p>

    <div class="grid">
      <section class="card">
        <h2>Calibration magnétomètre</h2>
        <p>
          Les valeurs de calibration sont automatiquement chargées au démarrage.
        </p>

        <div class="value"><span>Calibration chargée</span><span id="magCalibrationLoaded">---</span></div>
        <div class="value"><span>Offset X</span><span id="magOffsetX">---</span></div>
        <div class="value"><span>Offset Y</span><span id="magOffsetY">---</span></div>
        <div class="value"><span>Scale X</span><span id="magScaleX">---</span></div>
        <div class="value"><span>Scale Y</span><span id="magScaleY">---</span></div>

        <br>
        <button class="btn-green" onclick="api('/api/calibrate-mag')">
          Recalibrer et enregistrer
        </button>

        <button class="btn-red" onclick="api('/api/clear-mag-calibration')">
          Effacer calibration sauvegardée
        </button>
      </section>

      <section class="card">
        <h2>Rose des vents</h2>

        <label>Longueur flèche en cm</label>
        <input id="compassLength" type="number" value="10" min="3" max="30" step="0.1"><br>

        <label>PWM rotation</label>
        <input id="compassPwm" type="number" value="150" min="80" max="255" step="1">

        <br><br>
        <button class="btn-purple" onclick="startCompass()">
          Lancer orientation Nord
        </button>

        <button class="btn-red" onclick="api('/api/stop')">STOP</button>
      </section>

      <section class="card">
        <h2>Orientation</h2>
        <div class="value"><span>Cap magnétique</span><span id="headingMag">---</span></div>
        <div class="value"><span>Yaw gyro</span><span id="yawGyro">---</span></div>
        <div class="value"><span>Theta odométrie</span><span id="odoTheta">---</span></div>
      </section>

      <section class="card">
        <h2>Console ESP32</h2>
        <div id="console" class="console">Chargement...</div>
      </section>
    </div>
  </main>

  <script>
  async function startCompass() {
    const params = new URLSearchParams();
    params.append("length", document.getElementById("compassLength").value);
    params.append("pwm", document.getElementById("compassPwm").value);

    await fetch("/api/s2/rose/start?" + params.toString());
  }
  </script>
  )rawliteral";

    html += commonScript();

    html += R"rawliteral(
  <script>
  async function refreshCompassCalibration() {
    try {
      const res = await fetch("/api/status");
      const d = await res.json();

      setText("magCalibrationLoaded", d.magCalibrationLoaded ? "OUI" : "NON");
      setText("magOffsetX", fmt(d.magOffsetX, 2));
      setText("magOffsetY", fmt(d.magOffsetY, 2));
      setText("magScaleX", fmt(d.magScaleX, 4));
      setText("magScaleY", fmt(d.magScaleY, 4));
    } catch(e) {}
  }

  setInterval(refreshCompassCalibration, 500);
  refreshCompassCalibration();
  </script>
  )rawliteral";

    html += "</body></html>";
    return html;
  }
}
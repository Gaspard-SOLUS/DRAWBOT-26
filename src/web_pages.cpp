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
          Rayon demandé entre 2 cm et 20 cm.
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
      Réglage et lancement de la séquence escalier.
    </p>

    <p>
      <a href="/soutenance2">← Retour soutenance 2</a>
    </p>

    <div class="grid">
      <section class="card">
        <h2>Paramètres escalier</h2>

        <label>Distance 1 en cm</label>
        <input id="stairDist1" type="number" value="20" step="0.1"><br>

        <label>Angle gauche en degrés</label>
        <input id="stairAngleLeft" type="number" value="90" step="1"><br>

        <label>Distance 2 en cm</label>
        <input id="stairDist2" type="number" value="10" step="0.1"><br>

        <label>Angle droite en degrés</label>
        <input id="stairAngleRight" type="number" value="90" step="1"><br>

        <label>Distance 3 en cm</label>
        <input id="stairDist3" type="number" value="40" step="0.1">

        <br><br>
        <button class="btn-green" onclick="startStair()">Lancer escalier</button>
        <button class="btn-red" onclick="api('/api/stop')">STOP</button>
        <button class="btn-blue" onclick="api('/api/reset-encoders')">RESET odométrie</button>
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

  <script>
  async function startStair() {
    const params = new URLSearchParams();
    params.append("d1", document.getElementById("stairDist1").value);
    params.append("aL", document.getElementById("stairAngleLeft").value);
    params.append("d2", document.getElementById("stairDist2").value);
    params.append("aR", document.getElementById("stairAngleRight").value);
    params.append("d3", document.getElementById("stairDist3").value);

    await fetch("/api/s2/escalier/start?" + params.toString());
  }
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
      Réglage du rayon et lancement du tracé du cercle.
    </p>

    <p>
      <a href="/soutenance2">← Retour soutenance 2</a>
    </p>

    <div class="grid">
      <section class="card">
        <h2>Paramètres cercle</h2>

        <label>Rayon demandé en cm</label>
        <input id="circleRadius" type="number" value="10" min="2" max="20" step="0.1"><br>

        <label>Nombre de segments si approximation polygonale</label>
        <input id="circleSegments" type="number" value="36" min="12" max="96" step="1"><br>

        <label>PWM de base</label>
        <input id="circlePwm" type="number" value="170" min="80" max="255" step="1">

        <br><br>
        <button class="btn-green" onclick="startCircle()">Lancer cercle</button>
        <button class="btn-red" onclick="api('/api/stop')">STOP</button><br>
        <button class="btn-blue" onclick="api('/api/reset-encoders')">RESET odométrie</button>
      </section>

      <section class="card">
        <h2>Informations géométriques</h2>
        <p>
          Le stylo est décalé par rapport à l'axe des roues. La stratégie dépend
          donc du rayon demandé.
        </p>
        <div class="value"><span>Rayon minimum continu</span><span>≈ 13 cm</span></div>
        <div class="value"><span>Écartement roues</span><span>8.3 cm</span></div>
        <div class="value"><span>Offset stylo</span><span>13 cm</span></div>
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
  async function startCircle() {
    const params = new URLSearchParams();
    params.append("r", document.getElementById("circleRadius").value);
    params.append("n", document.getElementById("circleSegments").value);
    params.append("pwm", document.getElementById("circlePwm").value);

    await fetch("/api/s2/cercle/start?" + params.toString());
  }
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
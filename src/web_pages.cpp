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

    <a href="/simulation">
      <button class="btn-blue">Ouvrir la simulation odométrique</button>
    </a>

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
      Réglage et lancement de la séquence escalier avec suivi de trajectoire du stylo.
    </p>

    <p>
      <a href="/soutenance2">← Retour soutenance 2</a>
    </p>

    <div class="grid">
      <section class="card">
        <h2>1. Tests progressifs</h2>
        <p>
          Commence par tester une ligne droite, puis un seul angle, puis l'escalier complet.
        </p>

        <label>Distance ligne test — cm</label>
        <input id="lineDistance" type="number" value="20" step="0.1">

        <button class="btn-blue" onclick="startLineTest()">Tester ligne droite</button>

        <br><br>

        <label>Angle test distance 1 — cm</label>
        <input id="angleD1" type="number" value="20" step="0.1">

        <label>Angle test — degrés</label>
        <input id="angleValue" type="number" value="90" step="1">

        <label>Angle test distance 2 — cm</label>
        <input id="angleD2" type="number" value="10" step="0.1">

        <button class="btn-orange" onclick="startAngleTest()">Tester un angle</button>
      </section>

      <section class="card">
        <h2>2. Paramètres escalier complet</h2>

        <label>Distance 1 — cm</label>
        <input id="stairDist1" type="number" value="20" step="0.1">

        <label>Angle gauche — degrés</label>
        <input id="stairAngleLeft" type="number" value="90" step="1">

        <label>Distance 2 — cm</label>
        <input id="stairDist2" type="number" value="10" step="0.1">

        <label>Compensation trait 2 - cm</label>
        <input id="stairMiddleExtra" type="number" value="4.0" step="0.1">

        <label>Correction angle 2 - deg</label>
        <input id="stairAngleTrim" type="number" value="35.0" step="1">

        <label>Angle droite — degrés</label>
        <input id="stairAngleRight" type="number" value="90" step="1">

        <label>Distance 3 — cm</label>
        <input id="stairDist3" type="number" value="40" step="0.1">

        <br><br>

        <button class="btn-green" onclick="startStair()">Lancer escalier complet</button>
        <button class="btn-red" onclick="api('/api/s2/escalier/stop')">STOP follower</button>
        <button class="btn-red" onclick="api('/api/stop')">STOP moteurs</button>
        <button class="btn-blue" onclick="api('/api/reset-encoders')">RESET odométrie</button>
      </section>

      <section class="card">
        <h2>3. Géométrie robot</h2>

        <label>Écartement des roues — cm</label>
        <input id="wheelBase" type="number" value="8.3" step="0.1">

        <label>Distance axe roues → stylo — cm</label>
        <input id="penOffset" type="number" value="13.0" step="0.1">

        <label>Echelle distance</label>
        <input id="distanceScale" type="number" value="0.930" step="0.01">

        <label>Tolérance fin de segment — cm</label>
        <input id="segTol" type="number" value="0.08" step="0.01">

        <p>
          Le stylo est devant l'axe des roues. La commande calcule donc les vitesses
          des roues à partir de la vitesse souhaitée du stylo.
        </p>
      </section>

      <section class="card">
        <h2>4. Commande du stylo</h2>

        <label>Vitesse stylo — cm/s</label>
        <input id="penSpeed" type="number" value="13.0" step="0.1">

        <label>Gain retour vers ligne</label>
        <input id="lineGain" type="number" value="0.45" step="0.05">

        <label>Gain attraction point cible</label>
        <input id="targetGain" type="number" value="0.8" step="0.1">

        <label>Lookahead — cm</label>
        <input id="lookahead" type="number" value="0.35" step="0.05">

        <label>Vitesse stylo max — cm/s</label>
        <input id="penSpeedMax" type="number" value="18.0" step="0.1">

        <label>Vitesse roue max — cm/s</label>
        <input id="wheelSpeedMax" type="number" value="18.0" step="0.1">
      </section>

      <section class="card">
        <h2>5. PID correction latérale</h2>

        <label>Kp</label>
        <input id="kp" type="number" value="0.20" step="0.01">

        <label>Ki</label>
        <input id="ki" type="number" value="0.00" step="0.01">

        <label>Kd</label>
        <input id="kd" type="number" value="0.05" step="0.01">

        <label>Deadband escalier - cm</label>
        <input id="stairLineDeadband" type="number" value="0.25" step="0.01">

        <label>Lissage correction escalier</label>
        <input id="stairNormalSlew" type="number" value="0.25" step="0.01">

        <label>Limite intégrale</label>
        <input id="iLimit" type="number" value="10" step="1">

        <p>
          Le PID corrige l'écart latéral du stylo par rapport à la ligne idéale.
        </p>
      </section>

      <section class="card">
        <h2>6. Conversion vitesse → PWM</h2>

        <label>Coefficient moteur gauche — cm/s/PWM</label>
        <input id="coefL" type="number" value="0.0709" step="0.0001">

        <label>Coefficient moteur droit — cm/s/PWM</label>
        <input id="coefR" type="number" value="0.0686" step="0.0001">

        <label>PWM minimum</label>
        <input id="minPwm" type="number" value="180" min="0" max="255" step="1">

        <label>Rampe PWM / cycle</label>
        <input id="pwmSlewStep" type="number" value="20" min="1" max="255" step="1">

        <p>
          Le PWM minimum sert à compenser les frottements : si une roue doit bouger,
          on évite de lui envoyer un PWM trop faible pour démarrer.
        </p>

        <button class="btn-blue" onclick="sendFollowerConfig()">Appliquer paramètres</button>
        <button class="btn-dark" onclick="loadFollowerConfig()">Recharger paramètres ESP32</button>
      </section>

      <section class="card">
        <h2>7. État robot</h2>

        <div class="value"><span>État</span><span id="state">---</span></div>
        <div class="value"><span>Mode</span><span id="mode">---</span></div>
        <div class="value"><span>PWM gauche</span><span id="pwmL">---</span></div>
        <div class="value"><span>PWM droite</span><span id="pwmR">---</span></div>

        <div class="value"><span>X base</span><span id="odoX">---</span></div>
        <div class="value"><span>Y base</span><span id="odoY">---</span></div>
        <div class="value"><span>X stylo</span><span id="penX">---</span></div>
        <div class="value"><span>Y stylo</span><span id="penY">---</span></div>
        <div class="value"><span>Theta</span><span id="odoTheta">---</span></div>
      </section>

      <section class="card">
        <h2>8. Suivi du stylo</h2>

        <div class="value"><span>Follower actif</span><span id="followerRunning">---</span></div>
        <div class="value"><span>Segment</span><span id="followerSegment">---</span></div>
        <div class="value"><span>Cible X</span><span id="followerTargetX">---</span></div>
        <div class="value"><span>Cible Y</span><span id="followerTargetY">---</span></div>
        <div class="value"><span>Erreur latérale</span><span id="followerLateralError">---</span></div>
        <div class="value"><span>Erreur max</span><span id="followerMaxLateralError">---</span></div>
        <div class="value"><span>Progression</span><span id="followerProgress">---</span></div>
        <div class="value"><span>Longueur segment</span><span id="followerSegmentLength">---</span></div>
        <div class="value"><span>V roue gauche</span><span id="followerVLeft">---</span></div>
        <div class="value"><span>V roue droite</span><span id="followerVRight">---</span></div>
        <div class="value"><span>Omega</span><span id="followerOmega">---</span></div>
      </section>

      <section class="card">
        <h2>Console ESP32</h2>
        <div id="console" class="console">Chargement...</div>
      </section>
    </div>
  </main>

  <script>
  function getValue(id) {
    return document.getElementById(id).value;
  }

  function setText(id, value) {
    const el = document.getElementById(id);
    if (el) el.textContent = value;
  }

  function fmt(v, d = 2) {
    if (v === null || v === undefined || isNaN(v)) return "---";
    return Number(v).toFixed(d);
  }

  function appendFollowerConfig(params) {
    params.append("wheelBase", getValue("wheelBase"));
    params.append("penOffset", getValue("penOffset"));
    params.append("distanceScale", getValue("distanceScale"));
    params.append("stairMiddleExtra", getValue("stairMiddleExtra"));
    params.append("stairAngleTrim", getValue("stairAngleTrim"));
    params.append("stairLineDeadband", getValue("stairLineDeadband"));
    params.append("stairNormalSlew", getValue("stairNormalSlew"));

    params.append("penSpeed", getValue("penSpeed"));
    params.append("lineGain", getValue("lineGain"));
    params.append("targetGain", getValue("targetGain"));
    params.append("lookahead", getValue("lookahead"));

    params.append("penSpeedMax", getValue("penSpeedMax"));
    params.append("wheelSpeedMax", getValue("wheelSpeedMax"));

    params.append("kp", getValue("kp"));
    params.append("ki", getValue("ki"));
    params.append("kd", getValue("kd"));
    params.append("iLimit", getValue("iLimit"));

    params.append("coefL", getValue("coefL"));
    params.append("coefR", getValue("coefR"));
    params.append("minPwm", getValue("minPwm"));
    params.append("pwmSlewStep", getValue("pwmSlewStep"));

    params.append("segTol", getValue("segTol"));
  }

  async function sendFollowerConfig() {
    const params = new URLSearchParams();
    appendFollowerConfig(params);

    await fetch("/api/s2/escalier/config/set?" + params.toString());
    await refreshLogs();
  }

  async function loadFollowerConfig() {
    const res = await fetch("/api/s2/escalier/config");
    const d = await res.json();

    document.getElementById("wheelBase").value = d.wheelBase;
    document.getElementById("penOffset").value = d.penOffset;
    document.getElementById("distanceScale").value = d.distanceScale;
    document.getElementById("stairMiddleExtra").value = d.stairMiddleExtra;
    document.getElementById("stairAngleTrim").value = d.stairAngleTrim;
    document.getElementById("stairLineDeadband").value = d.stairLineDeadband;
    document.getElementById("stairNormalSlew").value = d.stairNormalSlew;

    document.getElementById("penSpeed").value = d.penSpeed;
    document.getElementById("lineGain").value = d.lineGain;
    document.getElementById("targetGain").value = d.targetGain;
    document.getElementById("lookahead").value = d.lookahead;

    document.getElementById("penSpeedMax").value = d.penSpeedMax;
    document.getElementById("wheelSpeedMax").value = d.wheelSpeedMax;

    document.getElementById("kp").value = d.kp;
    document.getElementById("ki").value = d.ki;
    document.getElementById("kd").value = d.kd;
    document.getElementById("iLimit").value = d.iLimit;

    document.getElementById("coefL").value = d.coefL;
    document.getElementById("coefR").value = d.coefR;
    document.getElementById("minPwm").value = d.minPwm;
    document.getElementById("pwmSlewStep").value = d.pwmSlewStep;

    document.getElementById("segTol").value = d.segTol;
  }

  async function startLineTest() {
    const params = new URLSearchParams();

    params.append("d", getValue("lineDistance"));
    appendFollowerConfig(params);

    await fetch("/api/s2/escalier/start-line?" + params.toString());
    await refreshLogs();
  }

  async function startAngleTest() {
    const params = new URLSearchParams();

    params.append("d1", getValue("angleD1"));
    params.append("a", getValue("angleValue"));
    params.append("d2", getValue("angleD2"));
    appendFollowerConfig(params);

    await fetch("/api/s2/escalier/start-angle?" + params.toString());
    await refreshLogs();
  }

  async function startStair() {
    const params = new URLSearchParams();

    params.append("d1", getValue("stairDist1"));
    params.append("aL", getValue("stairAngleLeft"));
    params.append("d2", getValue("stairDist2"));
    params.append("aR", getValue("stairAngleRight"));
    params.append("d3", getValue("stairDist3"));
    appendFollowerConfig(params);

    await fetch("/api/s2/escalier/start?" + params.toString());
    await refreshLogs();
  }

  async function api(route) {
    await fetch(route);
    await refreshLogs();
  }

  async function refreshStatus() {
    try {
      const res = await fetch("/api/status");
      const d = await res.json();

      setText("state", d.state);
      setText("mode", d.mode);
      setText("pwmL", d.pwmL);
      setText("pwmR", d.pwmR);

      setText("odoX", fmt(d.odoX, 2) + " cm");
      setText("odoY", fmt(d.odoY, 2) + " cm");
      setText("penX", fmt(d.penX, 2) + " cm");
      setText("penY", fmt(d.penY, 2) + " cm");
      setText("odoTheta", fmt(d.odoTheta, 1) + "°");

      setText("followerRunning", d.followerRunning ? "OUI" : "NON");
      setText("followerSegment", d.followerSegment + " / " + d.followerSegmentCount);
      setText("followerTargetX", fmt(d.followerTargetX, 2) + " cm");
      setText("followerTargetY", fmt(d.followerTargetY, 2) + " cm");
      setText("followerLateralError", fmt(d.followerLateralError, 2) + " cm");
      setText("followerMaxLateralError", fmt(d.followerMaxLateralError, 2) + " cm");
      setText("followerProgress", fmt(d.followerProgress, 2) + " cm");
      setText("followerSegmentLength", fmt(d.followerSegmentLength, 2) + " cm");
      setText("followerVLeft", fmt(d.followerVLeft, 2) + " cm/s");
      setText("followerVRight", fmt(d.followerVRight, 2) + " cm/s");
      setText("followerOmega", fmt(d.followerOmega, 2) + " rad/s");
    } catch(e) {}
  }

  async function refreshLogs() {
    const box = document.getElementById("console");

    try {
      const res = await fetch("/api/logs");
      box.textContent = await res.text();
    } catch(e) {
      box.textContent = "Erreur lecture console";
    }
  }

  loadFollowerConfig();
  refreshStatus();
  refreshLogs();

  setInterval(refreshStatus, 250);
  setInterval(refreshLogs, 1000);
  </script>
  )rawliteral";

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
    <h1>Soutenance 2 - Sequence 3 : fleche Nord</h1>

    <p>
      <a href="/soutenance2">&larr; Retour soutenance 2</a>
    </p>

    <div class="grid">
      <section class="card">
        <h2>Calibration magnetometre</h2>

        <label>PWM calibration</label>
        <input id="calibrationPwm" type="number" value="190" min="180" max="255" step="1">

        <label>Sens de rotation</label>
        <select id="clockwise">
          <option value="1">Horaire</option>
          <option value="0">Antihoraire</option>
        </select>

        <br><br>
        <button class="btn-green" onclick="startCalibration()">Calibration automatique 20 s</button>
        <button class="btn-red" onclick="api('/api/clear-mag-calibration')">Effacer calibration</button>

        <div class="value"><span>Magnetometre</span><span id="magOk">---</span></div>
        <div class="value"><span>Adresse LIS3MDL</span><span id="magAddress">---</span></div>
        <div class="value"><span>Lectures</span><span id="magReadCount">---</span></div>
        <div class="value"><span>Derniere lecture</span><span id="magReadAge">---</span></div>
        <div class="value"><span>Dernier changement cap</span><span id="magHeadingChangeAge">---</span></div>
        <div class="value"><span>Calibration</span><span id="magCalib">---</span></div>
        <div class="value"><span>Calibration chargee</span><span id="magCalibrationLoaded">---</span></div>
        <div class="value"><span>Variation calib X/Y</span><span id="magCalibrationRange">---</span></div>
        <div class="value"><span>Raw X/Y/Z</span><span id="magRaw">---</span></div>
        <div class="value"><span>Offset X</span><span id="magOffsetX">---</span></div>
        <div class="value"><span>Offset Y</span><span id="magOffsetY">---</span></div>
        <div class="value"><span>Scale X</span><span id="magScaleX">---</span></div>
        <div class="value"><span>Scale Y</span><span id="magScaleY">---</span></div>
      </section>

      <section class="card">
        <h2>Angle actuel</h2>

        <div style="font-size:48px;font-weight:bold;font-family:Consolas,monospace">
          <span id="headingMag">---</span>
        </div>
        <div id="northBadge" style="display:inline-block;margin:10px 0;padding:8px 12px;border-radius:999px;background:#334155;font-weight:bold">
          ---
        </div>

        <div class="value"><span>Direction</span><span id="headingCardinal">---</span></div>
        <div class="value"><span>Erreur au Nord</span><span id="northError">---</span></div>
        <div class="value"><span>Fenetre Nord</span><span>358 deg a 3 deg</span></div>
        <div class="value"><span>Cap initial utilise</span><span id="alignInitialHeading">---</span></div>
        <div class="value"><span>Rotation calculee</span><span id="alignTargetDeg">---</span></div>
        <div class="value"><span>Progression rotation</span><span id="alignProgressDeg">---</span></div>
        <div class="value"><span>Reste a tourner</span><span id="alignRemainingDeg">---</span></div>
        <div class="value"><span>Phase</span><span id="rosePhase">---</span></div>
        <div class="value"><span>Message</span><span id="roseMessage">---</span></div>
        <div class="value"><span>PWM G/D</span><span id="rosePwm">---</span></div>
      </section>

      <section class="card">
        <h2>Fleche Nord</h2>

        <label>Trait principal - cm</label>
        <input id="shaft" type="number" value="10.0" min="2" max="30" step="0.1">

        <label>Cote triangle - cm</label>
        <input id="head" type="number" value="3.0" min="1" max="8" step="0.1">

        <label>Pas remplissage triangle - cm</label>
        <input id="fill" type="number" value="0.35" min="0.15" max="2" step="0.05">

        <label>Vitesse dessin - cm/s</label>
        <input id="drawSpeed" type="number" value="8.0" min="2" max="16" step="0.1">

        <label>PWM alignement Nord</label>
        <input id="alignPwm" type="number" value="180" min="180" max="255" step="1">

        <div class="value"><span>Arret Nord</span><span>358 deg a 3 deg</span></div>

        <br><br>
        <button class="btn-blue" onclick="startAlignNorth()">1. Tourner vers le Nord</button>
        <button class="btn-purple" onclick="startDrawOnly()">2. Dessiner la fleche</button>
        <button class="btn-purple" onclick="startArrow()">3. Nord puis fleche</button>
        <button class="btn-red" onclick="api('/api/s2/rose/stop')">STOP rose</button>
        <button class="btn-red" onclick="api('/api/stop')">STOP moteurs</button>
      </section>

      <section class="card">
        <h2>Etat robot</h2>
        <div class="value"><span>Etat</span><span id="state">---</span></div>
        <div class="value"><span>Mode</span><span id="mode">---</span></div>
        <div class="value"><span>PWM gauche</span><span id="pwmL">---</span></div>
        <div class="value"><span>PWM droite</span><span id="pwmR">---</span></div>
        <div class="value"><span>X stylo</span><span id="penX">---</span></div>
        <div class="value"><span>Y stylo</span><span id="penY">---</span></div>
        <div class="value"><span>Segments fleche</span><span id="roseSegments">---</span></div>
      </section>

      <section class="card">
        <h2>Console ESP32</h2>
        <div id="console" class="console">Chargement...</div>
      </section>
    </div>
  </main>

  <script>
  function el(id) { return document.getElementById(id); }
  function v(id) { return el(id).value; }
  function setText(id, value) { const e = el(id); if (e) e.textContent = value; }
  function fmt(value, digits = 1) {
    if (value === null || value === undefined || isNaN(value)) return "---";
    return Number(value).toFixed(digits);
  }

  function directionName(deg) {
    if (deg === null || deg === undefined || isNaN(deg)) return "---";
    const h = ((Number(deg) % 360) + 360) % 360;
    if (h >= 358 || h <= 3) return "Nord";
    if (h < 87) return "Nord-Est";
    if (h <= 93) return "Est";
    if (h < 177) return "Sud-Est";
    if (h <= 183) return "Sud";
    if (h < 267) return "Sud-Ouest";
    if (h <= 273) return "Ouest";
    return "Nord-Ouest";
  }

  function roseParams() {
    const params = new URLSearchParams();
    params.append("shaft", v("shaft"));
    params.append("head", v("head"));
    params.append("fill", v("fill"));
    params.append("drawSpeed", v("drawSpeed"));
    params.append("alignPwm", v("alignPwm"));
    params.append("calibrationPwm", v("calibrationPwm"));
    params.append("clockwise", v("clockwise"));
    return params;
  }

  async function api(route) {
    await fetch(route);
    await refreshAll();
  }

  async function startCalibration() {
    await fetch("/api/s2/rose/calibrate/start?" + roseParams().toString());
    await refreshAll();
  }

  async function startAlignNorth() {
    await fetch("/api/s2/rose/align/start?" + roseParams().toString());
    await refreshAll();
  }

  async function startDrawOnly() {
    await fetch("/api/s2/rose/draw/start?" + roseParams().toString());
    await refreshAll();
  }

  async function startArrow() {
    await fetch("/api/s2/rose/start?" + roseParams().toString());
    await refreshAll();
  }

  async function refreshLogs() {
    try {
      const res = await fetch("/api/logs");
      el("console").textContent = await res.text();
    } catch(e) {
      el("console").textContent = "Erreur lecture console";
    }
  }

  async function refreshAll() {
    try {
      const stamp = Date.now();
      const statusRes = await fetch("/api/status?t=" + stamp);
      const status = await statusRes.json();

      const roseRes = await fetch("/api/s2/rose/status?t=" + stamp);
      const rose = await roseRes.json();

      setText("state", status.state);
      setText("mode", status.mode);
      setText("pwmL", status.pwmL);
      setText("pwmR", status.pwmR);
      setText("penX", fmt(status.penX, 2) + " cm");
      setText("penY", fmt(status.penY, 2) + " cm");

      setText("magOk", status.magOk ? "OK" : "NON");
      setText("magAddress", status.magAddress || "---");
      setText("magReadCount", status.magReadCount);
      setText("magReadAge", status.magReadAgeMs >= 0 ? status.magReadAgeMs + " ms" : "---");
      setText("magHeadingChangeAge", status.magHeadingChangeAgeMs >= 0 ? status.magHeadingChangeAgeMs + " ms" : "---");
      setText("magCalib", status.magCalib);
      setText("magCalibrationLoaded", status.magCalibrationLoaded ? "OUI" : "NON");
      setText("magCalibrationRange", fmt(status.magCalibrationRangeX, 2) + " / " + fmt(status.magCalibrationRangeY, 2) + " uT");
      setText("magRaw", fmt(status.magRawX, 2) + " / " + fmt(status.magRawY, 2) + " / " + fmt(status.magRawZ, 2) + " uT");
      setText("magOffsetX", fmt(status.magOffsetX, 2));
      setText("magOffsetY", fmt(status.magOffsetY, 2));
      setText("magScaleX", fmt(status.magScaleX, 4));
      setText("magScaleY", fmt(status.magScaleY, 4));

      setText("headingMag", fmt(status.headingMag, 1) + " deg");
      setText("headingCardinal", directionName(status.headingMag));
      setText("northError", fmt(rose.northError, 2) + " deg");
      setText("alignInitialHeading", fmt(rose.alignInitialHeading, 1) + " deg");
      setText("alignTargetDeg", fmt(rose.alignTargetDeg, 1) + " deg");
      setText("alignProgressDeg", fmt(rose.alignProgressDeg, 1) + " deg");
      setText("alignRemainingDeg", fmt(rose.alignRemainingDeg, 1) + " deg");
      setText("rosePhase", rose.phase);
      setText("roseMessage", rose.message || "---");
      setText("rosePwm", rose.pwmLeft + " / " + rose.pwmRight);
      setText("roseSegments", rose.trajectorySegments);

      const badge = el("northBadge");
      if (rose.inNorthWindow) {
        badge.textContent = "NORD OK";
        badge.style.background = "#22c55e";
        badge.style.color = "#052e16";
      } else {
        badge.textContent = "PAS AU NORD";
        badge.style.background = "#334155";
        badge.style.color = "#e5e7eb";
      }
    } catch(e) {}
  }

  refreshAll();
  refreshLogs();
  setInterval(refreshAll, 250);
  setInterval(refreshLogs, 1000);
  </script>
  )rawliteral";

    html += "</body></html>";
    return html;
  }

  String simulation() {
    String html = commonHead("Drawbot - Simulation odométrique");

    html += R"rawliteral(
  <main>
    <h1>Simulation odométrique du Drawbot</h1>
    <p class="subtitle">
      Visualisation en temps réel du mouvement estimé du robot, de l'axe des roues
      et de la position du stylo à partir des encodeurs.
    </p>

    <p>
      <a href="/soutenance2">← Retour soutenance 2</a>
    </p>

    <div class="grid">
      <section class="card">
        <h2>Vue 2D du robot</h2>
        <canvas id="simCanvas" width="900" height="600"
          style="width:100%; max-width:900px; background:#020617; border:1px solid #1e293b; border-radius:14px;">
        </canvas>

        <br><br>

        <button class="btn-blue" onclick="resetView()">Recentrer la vue</button>
        <button class="btn-orange" onclick="clearTrace()">Effacer la trace virtuelle</button>
        <button class="btn-red" onclick="api('/api/stop')">STOP moteurs</button>
        <button class="btn-green" onclick="api('/api/reset-encoders'); clearTrace();">RESET odométrie + trace</button>
      </section>

      <section class="card">
        <h2>Données odométrie</h2>
        <div class="value"><span>Base X</span><span id="odoX">---</span></div>
        <div class="value"><span>Base Y</span><span id="odoY">---</span></div>
        <div class="value"><span>Stylo X</span><span id="penX">---</span></div>
        <div class="value"><span>Stylo Y</span><span id="penY">---</span></div>
        <div class="value"><span>Orientation</span><span id="odoTheta">---</span></div>
        <div class="value"><span>Vitesse gauche</span><span id="speedL">---</span></div>
        <div class="value"><span>Vitesse droite</span><span id="speedR">---</span></div>
      </section>

      <section class="card">
        <h2>Paramètres géométriques</h2>
        <div class="value"><span>Écartement roues</span><span>8.3 cm</span></div>
        <div class="value"><span>Offset stylo</span><span>13.0 cm</span></div>
        <div class="value"><span>Modèle robot</span><span>rectangle 2D</span></div>
        <p>
          Le rectangle représente le robot vu de dessus. Le point rouge représente
          le stylo. La trace rouge correspond au chemin du stylo.
        </p>
      </section>

      <section class="card">
        <h2>Commandes manuelles</h2>

        <label>PWM manuel</label>
        <input id="manualPwm" type="number" value="160" min="80" max="255">

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
      </section>

      <section class="card">
        <h2>Console ESP32</h2>
        <div id="console" class="console">Chargement...</div>
      </section>
    </div>
  </main>

  <script>
  const canvas = document.getElementById("simCanvas");
  const ctx = canvas.getContext("2d");

  const WHEEL_BASE_CM = 8.3;
  const PEN_OFFSET_CM = 13.0;

  // Représentation visuelle du robot.
  // Longueur du robot dessinée : axe des roues -> avant un peu plus loin que le stylo.
  const ROBOT_LENGTH_CM = 18.0;
  const ROBOT_WIDTH_CM = 10.0;

  let scale = 8;              // pixels par cm
  let centerX = canvas.width / 2;
  let centerY = canvas.height / 2;

  let trace = [];
  let lastPen = null;

  function worldToCanvas(xCm, yCm) {
    return {
      x: centerX + xCm * scale,
      y: centerY - yCm * scale
    };
  }

  function drawGrid() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(148,163,184,0.15)";

    const stepCm = 5;
    const stepPx = stepCm * scale;

    for (let x = centerX % stepPx; x < canvas.width; x += stepPx) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, canvas.height);
      ctx.stroke();
    }

    for (let y = centerY % stepPx; y < canvas.height; y += stepPx) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(canvas.width, y);
      ctx.stroke();
    }

    // axes
    ctx.strokeStyle = "rgba(56,189,248,0.5)";
    ctx.beginPath();
    ctx.moveTo(0, centerY);
    ctx.lineTo(canvas.width, centerY);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(centerX, 0);
    ctx.lineTo(centerX, canvas.height);
    ctx.stroke();
  }

  function drawTrace() {
    if (trace.length < 2) return;

    ctx.strokeStyle = "#ef4444";
    ctx.lineWidth = 2;
    ctx.beginPath();

    const first = worldToCanvas(trace[0].x, trace[0].y);
    ctx.moveTo(first.x, first.y);

    for (let i = 1; i < trace.length; i++) {
      const p = worldToCanvas(trace[i].x, trace[i].y);
      ctx.lineTo(p.x, p.y);
    }

    ctx.stroke();
  }

  function drawRobot(baseX, baseY, thetaDeg, penX, penY) {
    const theta = thetaDeg * Math.PI / 180.0;

    const base = worldToCanvas(baseX, baseY);
    const pen = worldToCanvas(penX, penY);

    ctx.save();
    ctx.translate(base.x, base.y);
    ctx.rotate(-theta);

    // Robot rectangle vu de dessus
    // L'axe des roues est à x=0.
    const front = ROBOT_LENGTH_CM * scale;
    const rear = -4 * scale;
    const width = ROBOT_WIDTH_CM * scale;

    ctx.fillStyle = "rgba(56,189,248,0.20)";
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.rect(rear, -width / 2, front - rear, width);
    ctx.fill();
    ctx.stroke();

    // axe des roues
    ctx.strokeStyle = "#e5e7eb";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(0, -WHEEL_BASE_CM * scale / 2);
    ctx.lineTo(0, WHEEL_BASE_CM * scale / 2);
    ctx.stroke();

    // ligne centre -> stylo
    ctx.strokeStyle = "#facc15";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(PEN_OFFSET_CM * scale, 0);
    ctx.stroke();

    ctx.restore();

    // point base
    ctx.fillStyle = "#38bdf8";
    ctx.beginPath();
    ctx.arc(base.x, base.y, 5, 0, 2 * Math.PI);
    ctx.fill();

    // point stylo
    ctx.fillStyle = "#ef4444";
    ctx.beginPath();
    ctx.arc(pen.x, pen.y, 6, 0, 2 * Math.PI);
    ctx.fill();
  }

  function redraw(d) {
    drawGrid();
    drawTrace();
    drawRobot(d.odoX, d.odoY, d.odoTheta, d.penX, d.penY);
  }

  function clearTrace() {
    trace = [];
    lastPen = null;
  }

  function resetView() {
    centerX = canvas.width / 2;
    centerY = canvas.height / 2;
  }

  function addPenPoint(x, y) {
    if (!lastPen) {
      trace.push({x, y});
      lastPen = {x, y};
      return;
    }

    const dx = x - lastPen.x;
    const dy = y - lastPen.y;
    const dist = Math.sqrt(dx * dx + dy * dy);

    // évite d'ajouter trop de points identiques
    if (dist > 0.05) {
      trace.push({x, y});
      lastPen = {x, y};
    }

    // limite mémoire navigateur
    if (trace.length > 3000) {
      trace.shift();
    }
  }

  function setText(id, value) {
    const el = document.getElementById(id);
    if (el) el.textContent = value;
  }

  function fmt(v, d = 2) {
    if (v === null || v === undefined || isNaN(v)) return "---";
    return Number(v).toFixed(d);
  }

  async function refreshSimulation() {
    try {
      const res = await fetch("/api/status");
      const d = await res.json();

      addPenPoint(d.penX, d.penY);
      redraw(d);

      setText("odoX", fmt(d.odoX, 2) + " cm");
      setText("odoY", fmt(d.odoY, 2) + " cm");
      setText("penX", fmt(d.penX, 2) + " cm");
      setText("penY", fmt(d.penY, 2) + " cm");
      setText("odoTheta", fmt(d.odoTheta, 1) + "°");
      setText("speedL", fmt(d.speedL, 2) + " cm/s");
      setText("speedR", fmt(d.speedR, 2) + " cm/s");
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

  setInterval(refreshSimulation, 100);
  setInterval(refreshLogs, 1000);

  drawGrid();
  refreshSimulation();
  refreshLogs();
  </script>
  )rawliteral";

    html += commonScript();
    html += "</body></html>";
    return html;
  }
}

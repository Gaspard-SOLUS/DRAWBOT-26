#include <Arduino.h>
#include <WebServer.h>

#include "web_api_motor_calibration.h"
#include "motor_calibration.h"
#include "logger.h"

namespace {
  WebServer* webServer = nullptr;

  int readIntArg(const char* name, int defaultValue) {
    if (webServer != nullptr && webServer->hasArg(name)) {
      return webServer->arg(name).toInt();
    }
    return defaultValue;
  }

  unsigned long readULongArg(const char* name, unsigned long defaultValue) {
    if (webServer != nullptr && webServer->hasArg(name)) {
      return (unsigned long)webServer->arg(name).toInt();
    }
    return defaultValue;
  }

  void handleStart() {
    MotorCalibration::Config cfg;

    int pwm = readIntArg("pwm", 180);
    cfg.pwmLeft = readIntArg("pwmL", pwm);
    cfg.pwmRight = readIntArg("pwmR", pwm);
    cfg.durationMs = readULongArg("duration", 2000);

    MotorCalibration::start(cfg);

    webServer->send(200, "application/json", MotorCalibration::resultJson());
  }

  void handleStop() {
    MotorCalibration::stop();
    webServer->send(200, "application/json", MotorCalibration::resultJson());
  }

  void handleStatus() {
    webServer->send(200, "application/json", MotorCalibration::resultJson());
  }

  void handlePage() {
    const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Calibration moteurs Drawbot</title>
<style>
:root{--bg:#0f172a;--card:#111827;--panel:#020617;--border:#1e293b;--text:#e5e7eb;--muted:#94a3b8;--green:#22c55e;--red:#ef4444;--blue:#38bdf8}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font-family:Arial,Helvetica,sans-serif}
main{padding:24px;max-width:1100px;margin:auto}
h1{margin:0 0 8px}
p{color:var(--muted);line-height:1.45}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:16px}
.card{background:var(--card);border:1px solid var(--border);border-radius:16px;padding:18px}
label{display:block;color:var(--muted);font-size:14px;margin-top:12px}
input{width:100%;border-radius:10px;border:1px solid var(--border);background:var(--panel);color:var(--text);padding:10px;font-size:16px;margin-top:5px}
button{border:0;border-radius:12px;padding:12px 14px;color:white;font-weight:bold;cursor:pointer;margin-top:12px;width:100%}
.green{background:var(--green)}.red{background:var(--red)}.blue{background:var(--blue);color:#082f49}
.value{display:flex;justify-content:space-between;border-bottom:1px solid rgba(148,163,184,.16);padding:9px 0;gap:12px}
.value span:first-child{color:var(--muted)}
.value span:last-child{font-family:Consolas,monospace;font-weight:bold;text-align:right}
.console{background:var(--panel);border:1px solid var(--border);border-radius:12px;padding:12px;font-family:Consolas,monospace;color:#a7f3d0;white-space:pre-wrap;min-height:180px}
a{color:var(--blue)}
</style>
</head>
<body>
<main>
<h1>Calibration coefficients moteurs</h1>
<p>
Cette page mesure le coefficient <strong>cm/s/PWM</strong> de chaque roue avec les encodeurs.
Les deux moteurs peuvent avoir des coefficients différents.
</p>
<p><a href="/">← Retour accueil</a></p>

<div class="grid">
<section class="card">
<h2>Réglage du test</h2>

<label>PWM commun</label>
<input id="pwm" type="number" value="180" min="-255" max="255">

<label>PWM gauche</label>
<input id="pwmL" type="number" value="180" min="-255" max="255">

<label>PWM droite</label>
<input id="pwmR" type="number" value="180" min="-255" max="255">

<label>Durée du test — ms</label>
<input id="duration" type="number" value="2000" min="300" max="10000">

<button class="green" onclick="startTest()">Lancer calibration</button>
<button class="red" onclick="stopTest()">STOP</button>

<p>
Méthode : pose le robot au sol, lance le test, il roule pendant la durée choisie puis s'arrête.
Le code calcule la distance parcourue par chaque roue et en déduit le coefficient.
</p>
</section>

<section class="card">
<h2>Résultats</h2>
<div class="value"><span>État</span><span id="running">---</span></div>
<div class="value"><span>Temps</span><span id="elapsed">---</span></div>
<div class="value"><span>Δ gauche</span><span id="deltaL">---</span></div>
<div class="value"><span>Δ droite</span><span id="deltaR">---</span></div>
<div class="value"><span>Vitesse gauche</span><span id="speedL">---</span></div>
<div class="value"><span>Vitesse droite</span><span id="speedR">---</span></div>
<div class="value"><span>Coef gauche</span><span id="coefL">---</span></div>
<div class="value"><span>Coef droite</span><span id="coefR">---</span></div>

<button class="blue" onclick="copyCoeffs()">Copier coefficients</button>
</section>

<section class="card">
<h2>Console</h2>
<div id="console" class="console">---</div>
</section>
</div>
</main>

<script>
let lastData = null;

function v(id){return document.getElementById(id).value;}
function set(id, value){document.getElementById(id).textContent = value;}
function fmt(x,d=4){return Number.isFinite(Number(x)) ? Number(x).toFixed(d) : "---";}

async function startTest(){
  const params = new URLSearchParams();
  params.append("pwm", v("pwm"));
  params.append("pwmL", v("pwmL"));
  params.append("pwmR", v("pwmR"));
  params.append("duration", v("duration"));

  const res = await fetch("/api/calibration/motors/start?" + params.toString());
  lastData = await res.json();
  render(lastData);
  refreshLogs();
}

async function stopTest(){
  const res = await fetch("/api/calibration/motors/stop");
  lastData = await res.json();
  render(lastData);
  refreshLogs();
}

async function refreshStatus(){
  try{
    const res = await fetch("/api/calibration/motors/status");
    lastData = await res.json();
    render(lastData);
  }catch(e){}
}

async function refreshLogs(){
  try{
    const res = await fetch("/api/logs");
    document.getElementById("console").textContent = await res.text();
  }catch(e){}
}

function render(d){
  set("running", d.running ? "EN COURS" : (d.finished ? "TERMINE" : "ARRET"));
  set("elapsed", d.elapsedMs + " / " + d.durationMs + " ms");
  set("deltaL", fmt(d.deltaLeftCm, 2) + " cm");
  set("deltaR", fmt(d.deltaRightCm, 2) + " cm");
  set("speedL", fmt(d.speedLeftCms, 3) + " cm/s");
  set("speedR", fmt(d.speedRightCms, 3) + " cm/s");
  set("coefL", fmt(d.coefLeftCmsPerPwm, 6));
  set("coefR", fmt(d.coefRightCmsPerPwm, 6));
}

async function copyCoeffs(){
  if(!lastData)return;
  const text =
`coefL = ${fmt(lastData.coefLeftCmsPerPwm, 6)}
coefR = ${fmt(lastData.coefRightCmsPerPwm, 6)}`;
  await navigator.clipboard.writeText(text);
}

refreshStatus();
refreshLogs();
setInterval(refreshStatus, 250);
setInterval(refreshLogs, 1000);
</script>
</body>
</html>
)rawliteral";

    webServer->send_P(200, "text/html", html);
  }
}

namespace WebApiMotorCalibration {
  void registerRoutes(WebServer& server) {
    webServer = &server;

    server.on("/calibration/motors", handlePage);
    server.on("/api/calibration/motors/start", handleStart);
    server.on("/api/calibration/motors/stop", handleStop);
    server.on("/api/calibration/motors/status", handleStatus);
  }
}

#include <Arduino.h>
#include "page_s2_cercle.h"

namespace PageS2Cercle {
String html() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Drawbot - Petit cercle</title>
<style>
:root{--bg:#0f172a;--panel:#020617;--card:#111827;--border:#1e293b;--text:#e5e7eb;--muted:#94a3b8;--accent:#38bdf8;--green:#22c55e;--red:#ef4444;--orange:#f97316;--purple:#a855f7}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:Arial,Helvetica,sans-serif}main{padding:22px}h1{margin:0 0 6px;font-size:26px}.subtitle{color:var(--muted);margin-bottom:18px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(285px,1fr));gap:16px}.card{background:var(--card);border:1px solid var(--border);border-radius:16px;padding:16px;box-shadow:0 10px 28px rgba(0,0,0,.25)}h2{font-size:18px;margin:0 0 12px}label{display:block;color:var(--muted);font-size:13px;margin-top:10px;margin-bottom:5px}input,select{width:100%;border-radius:10px;border:1px solid var(--border);background:#020617;color:var(--text);padding:9px;font-size:14px}button{border:0;border-radius:12px;padding:11px 12px;color:white;font-weight:bold;cursor:pointer;width:100%;margin-top:8px}.btn-blue{background:var(--accent);color:#082f49}.btn-green{background:var(--green)}.btn-red{background:var(--red)}.btn-orange{background:var(--orange)}.btn-purple{background:var(--purple)}.btn-dark{background:#334155}.value{display:flex;justify-content:space-between;gap:12px;border-bottom:1px solid rgba(148,163,184,.15);padding:7px 0;font-size:13px}.value span:first-child{color:var(--muted)}.value span:last-child{font-family:Consolas,monospace;font-weight:bold;text-align:right}.console{background:#020617;border:1px solid var(--border);border-radius:14px;padding:10px;height:180px;overflow:auto;color:#a7f3d0;font-family:Consolas,monospace;font-size:12px;white-space:pre-wrap}a{color:var(--accent)}p{color:var(--muted);font-size:13px;line-height:1.42}
</style>
</head>
<body>
<main>
<h1>Soutenance 2 - Petit cercle</h1>
<p class="subtitle">Commande d'un cercle de petit rayon avec le stylo. Le robot peut avancer et reculer pour suivre la trajectoire.</p>
<p><a href="/soutenance2">← Retour soutenance 2</a></p>

<div class="grid">
<section class="card">
  <h2>1. Cercle</h2>
  <label>Rayon cercle — cm</label>
  <input id="radius" type="number" value="5.0" step="0.1">

  <label>Nombre de segments</label>
  <input id="segments" type="number" value="96" step="1">

  <label>Sens</label>
  <select id="clockwise">
    <option value="1">Horaire</option>
    <option value="0">Antihoraire</option>
  </select>

  <label>Position de départ</label>
  <select id="startMode">
    <option value="bottom">Stylo en bas du cercle</option>
    <option value="right">Stylo à droite du cercle</option>
  </select>

  <label>Orientation initiale</label>
  <select id="initialHeadingMode">
    <option value="tangent">Tangente automatique</option>
    <option value="zero">0°</option>
  </select>

  <p>
    Pour un petit cercle, il faut démarrer tangent au cercle. Sinon le robot commence par faire une grande correction.
  </p>

  <button class="btn-green" onclick="startCircle()">Lancer cercle</button>
  <button class="btn-red" onclick="api('/api/s2/cercle/stop')">STOP cercle</button>
  <button class="btn-red" onclick="api('/api/stop')">STOP moteurs</button>
  <button class="btn-blue" onclick="api('/api/reset-encoders')">RESET odométrie</button>
</section>

<section class="card">
  <h2>2. Réglages suivi</h2>
  <label>Échelle distance</label>
  <input id="distanceScale" type="number" value="1.0" step="0.01">

  <label>Vitesse stylo — cm/s</label>
  <input id="penSpeed" type="number" value="0.5" step="0.1">

  <label>Lookahead — cm</label>
  <input id="lookahead" type="number" value="0.35" step="0.05">

  <label>Gain attraction cible</label>
  <input id="targetGain" type="number" value="0.8" step="0.1">

  <label>Gain retour ligne</label>
  <input id="lineGain" type="number" value="0.45" step="0.05">

  <label>Vitesse stylo max — cm/s</label>
  <input id="penSpeedMax" type="number" value="1.5" step="0.1">

  <label>Vitesse roue max — cm/s</label>
  <input id="wheelSpeedMax" type="number" value="3.0" step="0.1">
</section>

<section class="card">
  <h2>3. PID</h2>
  <label>Kp</label>
  <input id="kp" type="number" value="8" step="0.1">

  <label>Ki</label>
  <input id="ki" type="number" value="0.00" step="0.01">

  <label>Kd</label>
  <input id="kd" type="number" value="4" step="0.1">

  <label>Limite intégrale</label>
  <input id="iLimit" type="number" value="5" step="1">
</section>

<section class="card">
  <h2>4. Moteurs</h2>
  <label>Autoriser recul</label>
  <select id="allowReverse">
    <option value="1">Oui</option>
    <option value="0">Non</option>
  </select>

  <label>Écartement roues — cm</label>
  <input id="wheelBase" type="number" value="8.3" step="0.1">

  <label>Offset stylo — cm</label>
  <input id="penOffset" type="number" value="13.0" step="0.1">

  <label>Coef gauche cm/s/PWM</label>
  <input id="coefL" type="number" value="0.0709" step="0.001">

  <label>Coef droite cm/s/PWM</label>
  <input id="coefR" type="number" value="0.0686" step="0.001">

  <label>PWM minimum</label>
  <input id="minPwm" type="number" value="180" step="1">

  <label>Rampe PWM</label>
  <input id="pwmSlewStep" type="number" value="6" step="1">

  <label>Micro-impulsions PWM</label>
  <select id="pwmDither">
    <option value="1">Oui - conseillé pour petit cercle</option>
    <option value="0">Non</option>
  </select>

  <p>
    Si le PWM minimum est élevé, les micro-impulsions évitent que chaque petite correction devienne une grosse marche avant/arrière.
  </p>

  <label>Tolérance segment — cm</label>
  <input id="segTol" type="number" value="0.08" step="0.01">
</section>

<section class="card">
  <h2>5. État robot</h2>
  <div class="value"><span>État</span><span id="state">---</span></div>
  <div class="value"><span>Mode</span><span id="mode">---</span></div>
  <div class="value"><span>PWM G/D</span><span id="pwm">---</span></div>
  <div class="value"><span>Stylo X/Y</span><span id="pen">---</span></div>
  <div class="value"><span>Base X/Y</span><span id="odo">---</span></div>
  <div class="value"><span>Theta</span><span id="theta">---</span></div>
  <div class="value"><span>Erreur latérale</span><span id="err">---</span></div>
  <div class="value"><span>Segment</span><span id="segment">---</span></div>
  <div class="value"><span>V roues</span><span id="wheels">---</span></div>
</section>

<section class="card">
  <h2>Console ESP32</h2>
  <div id="console" class="console">Chargement...</div>
</section>
</div>
</main>

<script>
function v(id){return document.getElementById(id).value;}
function setText(id,value){const el=document.getElementById(id);if(el)el.textContent=value;}
function fmt(x,d=2){return Number.isFinite(Number(x))?Number(x).toFixed(d):"---";}

function appendCircleParams(params){
  params.append("radius", v("radius"));
  params.append("segments", v("segments"));
  params.append("clockwise", v("clockwise"));
  params.append("startMode", v("startMode"));
  params.append("initialHeadingMode", v("initialHeadingMode"));

  params.append("distanceScale", v("distanceScale"));
  params.append("penSpeed", v("penSpeed"));
  params.append("lookahead", v("lookahead"));
  params.append("targetGain", v("targetGain"));
  params.append("lineGain", v("lineGain"));
  params.append("penSpeedMax", v("penSpeedMax"));
  params.append("wheelSpeedMax", v("wheelSpeedMax"));

  params.append("kp", v("kp"));
  params.append("ki", v("ki"));
  params.append("kd", v("kd"));
  params.append("iLimit", v("iLimit"));

  params.append("allowReverse", v("allowReverse"));
  params.append("wheelBase", v("wheelBase"));
  params.append("penOffset", v("penOffset"));
  params.append("coefL", v("coefL"));
  params.append("coefR", v("coefR"));
  params.append("minPwm", v("minPwm"));
  params.append("pwmSlewStep", v("pwmSlewStep"));
  params.append("pwmDither", v("pwmDither"));
  params.append("segTol", v("segTol"));
}

async function startCircle(){
  const params = new URLSearchParams();
  appendCircleParams(params);
  await fetch("/api/s2/cercle/start-small?" + params.toString());
  await refreshLogs();
}

async function api(route){
  await fetch(route);
  await refreshLogs();
}

async function refreshStatus(){
  try{
    const res = await fetch("/api/status");
    const d = await res.json();

    setText("state", d.state);
    setText("mode", d.mode);
    setText("pwm", d.pwmL + " / " + d.pwmR);
    setText("pen", fmt(d.penX,2) + " / " + fmt(d.penY,2) + " cm");
    setText("odo", fmt(d.odoX,2) + " / " + fmt(d.odoY,2) + " cm");
    setText("theta", fmt(d.odoTheta,1) + "°");
    setText("err", fmt(d.followerLateralError,2) + " cm");
    setText("segment", d.followerSegment + " / " + d.followerSegmentCount);
    setText("wheels", fmt(d.followerVLeft,2) + " / " + fmt(d.followerVRight,2) + " cm/s");
  }catch(e){}
}

async function refreshLogs(){
  try{
    const res = await fetch("/api/logs");
    document.getElementById("console").textContent = await res.text();
  }catch(e){
    document.getElementById("console").textContent = "Erreur lecture console";
  }
}

refreshStatus();
refreshLogs();
setInterval(refreshStatus,250);
setInterval(refreshLogs,1000);
</script>
</body>
</html>
)rawliteral";

  return html;
}
}

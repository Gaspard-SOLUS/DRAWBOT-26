/**
 * ============================================================
 *  DRAWBOT – main.cpp
 *  Plateforme Gyrobot / NodeMCU ESP32
 *  ECE – Systèmes Bouclés 2026
 * ============================================================
 *
 *  Architecture :
 *    - Point d'accès WiFi (AP) propre sur 192.168.4.1
 *    - Serveur HTTP port 80  → page de réglages + commandes
 *    - Séquence 1 : escalier  (module escalier.cpp)
 *    - Séquence 2 : cercle    (à implémenter)
 *    - Séquence 3 : rose des vents (à implémenter)
 *    - Odométrie différentielle mise à jour toutes les 20 ms
 *    - Télémétrie série (compatible Teleplot) toutes les 200 ms
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "include/pins.h"
#include "include/moteurs.h"
#include "include/encodeurs.h"
#include "include/avance.h"
#include "include/escalier.h"
#include "include/pid_vitesse.h"

// ============================================================
// WIFI – Point d'accès
// ============================================================
static const char* AP_SSID     = "Drawbot";
static const char* AP_PASSWORD = "12345678";

WebServer server(80);

// ============================================================
// ODOMÉTRIE
// ============================================================
static float wheelBaseCm  = 12.5f;
static float penOffsetCm  = 13.5f;                      // stylo en avant du centre

static float xRobot    = 0.0f;
static float yRobot    = 0.0f;
static float thetaRad  = 0.0f;   // cap en radians (0 = avant)
static float xPen      = 0.0f;
static float yPen      = 0.0f;

static float lastLeftCm  = 0.0f;
static float lastRightCm = 0.0f;

void resetOdometry() {
    xRobot   = 0.0f;
    yRobot   = 0.0f;
    thetaRad = 0.0f;
    xPen     = penOffsetCm;
    yPen     = 0.0f;
    lastLeftCm  = getLeftDistanceCm();
    lastRightCm = getRightDistanceCm();
}

void updateOdometry() {
    float lCm = getLeftDistanceCm();
    float rCm = getRightDistanceCm();
    float dL  = lCm - lastLeftCm;
    float dR  = rCm - lastRightCm;
    lastLeftCm  = lCm;
    lastRightCm = rCm;

    float dCenter = (dL + dR) * 0.5f;
    float dTheta  = (dR - dL) / wheelBaseCm;
    thetaRad += dTheta;

    xRobot += dCenter * cosf(thetaRad);
    yRobot += dCenter * sinf(thetaRad);
    xPen    = xRobot + penOffsetCm * cosf(thetaRad);
    yPen    = yRobot + penOffsetCm * sinf(thetaRad);
}

// Distance absolue moyenne (utile pour debug)
static float distAbs() {
    return (fabsf(getLeftDistanceCm()) + fabsf(getRightDistanceCm())) * 0.5f;
}

// ============================================================
// MACHINE À ÉTATS PRINCIPALE
// ============================================================
enum MainState : uint8_t {
    STATE_IDLE = 0,
    STATE_SEQ1_ESCALIER,
    STATE_SEQ2_CERCLE,
    STATE_SEQ3_ROSE,
    STATE_FINISHED
};

static MainState mainState = STATE_IDLE;

static const char* stateNames[] = {
    "IDLE", "SEQ1_ESCALIER", "SEQ2_CERCLE", "SEQ3_ROSE", "FINISHED"
};

// ============================================================
// TIMING
// ============================================================
static unsigned long lastEncoderUpdate = 0;
static unsigned long lastTelemetry     = 0;

// ============================================================
// PID – paramètres exposés à la page web
// ============================================================
static float pidKp = 6.0f;
static float pidKi = 1.6f;
static float pidKd = 0.0f;

// ============================================================
// HELPERS HTML
// ============================================================
static String htmlInput(const String& label, const String& name, const String& value) {
    return "<label>" + label + "</label><br>"
           "<input name='" + name + "' value='" + value + "'><br><br>";
}

static String htmlSection(const String& title, const String& content) {
    return "<div class='card'><h2>" + title + "</h2>" + content + "</div>";
}

// ============================================================
// PAGE PRINCIPALE
// ============================================================
void handleRoot() {
    const EscalierParams& p = escalierGetParams();

    String html;
    html.reserve(8192);

    html += "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<meta charset='utf-8'>";
    html += "<title>Drawbot</title>";
    html += "<style>";
    html += "body{font-family:Arial,sans-serif;background:#111;color:#eee;margin:0;padding:20px 20px 20px 230px;}";
    html += ".sidebar{position:fixed;left:0;top:0;width:210px;height:100vh;background:#181818;padding:15px;box-sizing:border-box;overflow-y:auto;}";
    html += ".sidebar h2{color:#0af;margin-top:0;}";
    html += ".content{max-width:860px;}";
    html += "input{width:100%;padding:8px;font-size:16px;margin-top:4px;box-sizing:border-box;background:#333;color:#eee;border:1px solid #555;border-radius:4px;}";
    html += "a.btn,button{display:block;text-align:center;padding:12px;margin:8px 0;background:#2b7cff;color:#fff;text-decoration:none;border-radius:6px;border:0;font-size:16px;cursor:pointer;width:100%;}";
    html += "a.btn.stop{background:#c22;}";
    html += "a.btn.seq1{background:#0a7;}";
    html += "a.btn.seq2{background:#a70;}";
    html += "a.btn.seq3{background:#70a;}";
    html += ".card{background:#222;padding:15px;border-radius:8px;margin-bottom:15px;}";
    html += ".telemetry{font-family:monospace;font-size:14px;line-height:1.7;}";
    html += "h1{color:#0af;}h2{color:#8cf;margin-top:0;}";
    html += "label{font-size:14px;color:#aaa;}";
    html += "@media(max-width:700px){body{padding:10px}.sidebar{position:static;width:100%;height:auto;display:flex;flex-wrap:wrap;gap:6px;padding:10px;}.sidebar a.btn{width:calc(50% - 3px);margin:0;}}";
    html += "</style></head><body>";

    // --- Sidebar ---
    html += "<div class='sidebar'>";
    html += "<h2>Drawbot</h2>";
    html += "<a class='btn' href='/go_seq1'>▶ Séquence 1<br><small>Escalier</small></a>";
    html += "<a class='btn seq2' href='/go_seq2'>▶ Séquence 2<br><small>Cercle</small></a>";
    html += "<a class='btn seq3' href='/go_seq3'>▶ Séquence 3<br><small>Rose des vents</small></a>";
    html += "<a class='btn stop' href='/stop'>■ STOP</a>";
    html += "<a class='btn' href='/reset'>↺ Reset</a>";
    html += "</div>";

    // --- Contenu ---
    html += "<div class='content'>";
    html += "<h1>Drawbot – Réglages</h1>";

    // Télémétrie live
    html += "<div class='card telemetry'>";
    html += "<b>État :</b> " + String(stateNames[mainState]) + "<br>";
    if (mainState == STATE_SEQ1_ESCALIER) {
        html += "<b>Étape :</b> " + String(escalierGetStepName()) + "<br>";
    }
    html += "<b>Dist moy :</b> " + String(distAbs(), 2) + " cm | ";
    html += "<b>Theta :</b> " + String(thetaRad * 180.0f / PI, 1) + "°<br>";
    html += "<b>xPen :</b> " + String(xPen, 2) + " cm | ";
    html += "<b>yPen :</b> " + String(yPen, 2) + " cm<br>";
    html += "<b>Ticks G :</b> " + String(getLeftEncoderTicks()) + " | ";
    html += "<b>Ticks D :</b> " + String(getRightEncoderTicks()) + "<br>";
    html += "<b>Vit G :</b> " + String(getLeftSpeedCmParSec(), 1) + " cm/s | ";
    html += "<b>Vit D :</b> " + String(getRightSpeedCmParSec(), 1) + " cm/s";
    html += "</div>";

    html += "<form action='/set' method='get'>";

    // --- Section Séquence 1 ---
    String s1 = "";
    s1 += htmlInput("Segment 1 (cm)", "seg1", String(p.seg1Cm));
    s1 += htmlInput("Segment 2 (cm)", "seg2", String(p.seg2Cm));
    s1 += htmlInput("Segment 3 (cm)", "seg3", String(p.seg3Cm));
    s1 += htmlInput("PWM croisière", "cruise", String(p.cruisePwm));
    s1 += htmlInput("PWM lent", "slow", String(p.slowPwm));
    s1 += htmlInput("Zone lente (cm)", "slowzone", String(p.slowZoneCm));
    s1 += htmlInput("Kp cap droit", "kp", String(p.kpStraight));
    s1 += htmlInput("Durée freinage (ms)", "brakeMs", String((int)p.brakeMs));
    html += htmlSection("Séquence 1 – Escalier – Lignes droites", s1);

    String s1t = "";
    s1t += htmlInput("Virage gauche – PWM roue ext. (droite)", "tlOuterPwm", String(p.turnLeftOuterPwm));
    s1t += htmlInput("Virage gauche – PWM roue int. (gauche)", "tlInnerPwm", String(p.turnLeftInnerPwm));
    s1t += htmlInput("Virage droite – PWM roue ext. (gauche)", "trOuterPwm", String(p.turnRightOuterPwm));
    s1t += htmlInput("Virage droite – PWM roue int. (droite)", "trInnerPwm", String(p.turnRightInnerPwm));
    html += htmlSection("Séquence 1 – Escalier – Virages 90°", s1t);

    // --- Section PID vitesse ---
    String spid = "";
    spid += htmlInput("Kp", "pidKp", String(pidKp));
    spid += htmlInput("Ki", "pidKi", String(pidKi));
    spid += htmlInput("Kd", "pidKd", String(pidKd));
    html += htmlSection("PID Vitesse (boucle fermée vitesse roues)", spid);

    // --- Section robot ---
    String srobot = "";
    srobot += htmlInput("Entraxe (cm)", "wheelbase", String(wheelBaseCm));
    srobot += htmlInput("Décalage stylo (cm)", "penoffset", String(penOffsetCm));
    html += htmlSection("Paramètres robot", srobot);

    html += "<button type='submit'>💾 Enregistrer les réglages</button>";
    html += "</form>";
    html += "</div>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}

// ============================================================
// HANDLER SET – applique les paramètres reçus
// ============================================================
void handleSet() {
    EscalierParams p = escalierGetParams(); // copie mutable

    if (server.hasArg("seg1"))      p.seg1Cm           = server.arg("seg1").toFloat();
    if (server.hasArg("seg2"))      p.seg2Cm           = server.arg("seg2").toFloat();
    if (server.hasArg("seg3"))      p.seg3Cm           = server.arg("seg3").toFloat();
    if (server.hasArg("cruise"))    p.cruisePwm        = server.arg("cruise").toInt();
    if (server.hasArg("slow"))      p.slowPwm          = server.arg("slow").toInt();
    if (server.hasArg("slowzone"))  p.slowZoneCm       = server.arg("slowzone").toFloat();
    if (server.hasArg("kp"))        p.kpStraight       = server.arg("kp").toFloat();
    if (server.hasArg("brakeMs"))   p.brakeMs          = (unsigned long)server.arg("brakeMs").toInt();
    if (server.hasArg("tlOuterPwm")) p.turnLeftOuterPwm  = server.arg("tlOuterPwm").toFloat();
    if (server.hasArg("tlInnerPwm")) p.turnLeftInnerPwm  = server.arg("tlInnerPwm").toFloat();
    if (server.hasArg("trOuterPwm")) p.turnRightOuterPwm = server.arg("trOuterPwm").toFloat();
    if (server.hasArg("trInnerPwm")) p.turnRightInnerPwm = server.arg("trInnerPwm").toFloat();
    if (server.hasArg("wheelbase")) wheelBaseCm = server.arg("wheelbase").toFloat();
    if (server.hasArg("penoffset")) penOffsetCm = server.arg("penoffset").toFloat();

    escalierSetParams(p);

    // PID
    if (server.hasArg("pidKp")) pidKp = server.arg("pidKp").toFloat();
    if (server.hasArg("pidKi")) pidKi = server.arg("pidKi").toFloat();
    if (server.hasArg("pidKd")) pidKd = server.arg("pidKd").toFloat();
    setPidGains(pidKp, pidKi, pidKd);

    Serial.println("[WEB] Réglages mis à jour");
    server.sendHeader("Location", "/");
    server.send(303);
}

// ============================================================
// HANDLERS COMMANDES
// ============================================================
void handleGoSeq1() {
    stopMotors();
    resetEncoders();
    resetOdometry();
    escalierStart();
    mainState = STATE_SEQ1_ESCALIER;
    Serial.println("[WEB] GO Séquence 1 – Escalier");
    server.sendHeader("Location", "/");
    server.send(303);
}

void handleGoSeq2() {
    stopMotors();
    resetEncoders();
    resetOdometry();
    mainState = STATE_SEQ2_CERCLE;
    Serial.println("[WEB] GO Séquence 2 – Cercle (TODO)");
    server.sendHeader("Location", "/");
    server.send(303);
}

void handleGoSeq3() {
    stopMotors();
    resetEncoders();
    resetOdometry();
    mainState = STATE_SEQ3_ROSE;
    Serial.println("[WEB] GO Séquence 3 – Rose des vents (TODO)");
    server.sendHeader("Location", "/");
    server.send(303);
}

void handleStop() {
    stopMotors();
    escalierReset();
    mainState = STATE_IDLE;
    Serial.println("[WEB] STOP");
    server.sendHeader("Location", "/");
    server.send(303);
}

void handleReset() {
    stopMotors();
    resetEncoders();
    resetOdometry();
    escalierReset();
    mainState = STATE_IDLE;
    Serial.println("[WEB] Reset odométrie");
    server.sendHeader("Location", "/");
    server.send(303);
}

// ============================================================
// TÉLÉMÉTRIE SÉRIE – compatible Teleplot
// Préfixe '>' → variable tracée dans Teleplot
// ============================================================
void printTelemetry(unsigned long now) {
    if (now - lastTelemetry < 200) return;
    lastTelemetry = now;

    // Format lisible
    Serial.printf("state=%s | distL=%.2f | distR=%.2f | theta=%.1f° | xPen=%.2f | yPen=%.2f\n",
        stateNames[mainState],
        getLeftDistanceCm(), getRightDistanceCm(),
        thetaRad * 180.0f / PI, xPen, yPen);

    if (mainState == STATE_SEQ1_ESCALIER) {
        Serial.printf("  escalier=%s\n", escalierGetStepName());
    }

    // Variables Teleplot
    Serial.printf(">distL:%.3f\n",   getLeftDistanceCm());
    Serial.printf(">distR:%.3f\n",   getRightDistanceCm());
    Serial.printf(">speedL:%.3f\n",  getLeftSpeedCmParSec());
    Serial.printf(">speedR:%.3f\n",  getRightSpeedCmParSec());
    Serial.printf(">thetaDeg:%.2f\n", thetaRad * 180.0f / PI);
    Serial.printf(">xPen:%.3f\n",    xPen);
    Serial.printf(">yPen:%.3f\n",    yPen);
    Serial.printf(">ticksL:%ld\n",   getLeftEncoderTicks());
    Serial.printf(">ticksR:%ld\n",   getRightEncoderTicks());

    // PID debug si actif
    Serial.printf(">pidErrL:%.3f\n", getLeftPidError());
    Serial.printf(">pidErrR:%.3f\n", getRightPidError());
    Serial.printf(">pidOutL:%.3f\n", getLeftPidOutput());
    Serial.printf(">pidOutR:%.3f\n", getRightPidOutput());
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== DRAWBOT – Démarrage ===");

    // LEDs utilisateur
    pinMode(LEDU1, OUTPUT);
    pinMode(LEDU2, OUTPUT);
    digitalWrite(LEDU1, HIGH);
    digitalWrite(LEDU2, LOW);

    // Modules matériel
    initMotors();
    initEncoders();
    initPidVitesse();
    setPidGains(pidKp, pidKi, pidKd);

    resetEncoders();
    resetOdometry();

    // WiFi en mode Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    IPAddress apIP = WiFi.softAPIP();
    Serial.printf("[WiFi] AP créé : %s – IP : %s\n", AP_SSID, apIP.toString().c_str());

    // Routes HTTP
    server.on("/",         handleRoot);
    server.on("/set",      handleSet);
    server.on("/go_seq1",  handleGoSeq1);
    server.on("/go_seq2",  handleGoSeq2);
    server.on("/go_seq3",  handleGoSeq3);
    server.on("/stop",     handleStop);
    server.on("/reset",    handleReset);
    server.begin();
    Serial.println("[Web] Serveur HTTP démarré sur port 80");

    lastEncoderUpdate = millis();
    lastTelemetry     = millis();

    digitalWrite(LEDU1, LOW);
    digitalWrite(LEDU2, HIGH);
    Serial.println("[DRAWBOT] Prêt. Connectez-vous au WiFi \"Drawbot\" → http://192.168.4.1");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    unsigned long now = millis();

    // --- Serveur web ---
    server.handleClient();

    // --- Mise à jour encodeurs + odométrie (toutes les 20 ms) ---
    if (now - lastEncoderUpdate >= 20) {
        lastEncoderUpdate = now;
        updateEncoderMeasurements(now);
        updateOdometry();
    }

    // --- Télémétrie série ---
    printTelemetry(now);

    // --- Machine à états principale ---
    switch (mainState) {

    // ----------------------------------------------------------
    case STATE_IDLE:
        // Rien à faire, attente commande web
        break;

    // ----------------------------------------------------------
    case STATE_SEQ1_ESCALIER:
        escalierUpdate(now);
        if (escalierIsFinished()) {
            stopMotors();
            mainState = STATE_FINISHED;
            Serial.println("[MAIN] Séquence 1 terminée !");
        }
        break;

    // ----------------------------------------------------------
    case STATE_SEQ2_CERCLE:
        // TODO : implémenter cercle.cpp / cercleUpdate(now)
        // Exemple d'appel futur :
        //   cercleUpdate(now);
        //   if (cercleIsFinished()) { mainState = STATE_FINISHED; }
        //
        // Pour l'instant on s'arrête proprement
        stopMotors();
        mainState = STATE_FINISHED;
        Serial.println("[MAIN] Séquence 2 – non encore implémentée");
        break;

    // ----------------------------------------------------------
    case STATE_SEQ3_ROSE:
        // TODO : implémenter rose.cpp / roseUpdate(now)
        stopMotors();
        mainState = STATE_FINISHED;
        Serial.println("[MAIN] Séquence 3 – non encore implémentée");
        break;

    // ----------------------------------------------------------
    case STATE_FINISHED:
        stopMotors();
        // Reste dans cet état jusqu'à commande web
        break;
    }
}
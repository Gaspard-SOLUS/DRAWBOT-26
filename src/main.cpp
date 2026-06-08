#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_LIS3MDL.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <vector>
#include <string.h>

// =====================================================================
//  CONFIGURATION MATÉRIELLE
// =====================================================================

#define PIN_SDA 21
#define PIN_SCL 22


// User led
#define LEDU1 25
#define LEDU2 26

// Enable moteurs droit et gauche
#define EN_D 4
#define EN_G 23

// Commande PWM moteur droit
#define IN_1_D 16
#define IN_2_D 17

// Commande PWM moteur gauche
#define IN_1_G 19
#define IN_2_G 18

// Encodeur gauche
#define ENC_G_CH_A 27
#define ENC_G_CH_B 14

// Encodeur droit
#define ENC_D_CH_A 32
#define ENC_D_CH_B 33


#define ADDR_LSM6DS3 0x6B
#define ADDR_LIS3MDL 0x1E

// =====================================================================
//  CONFIGURATION WIFI
// =====================================================================
const char* ssid     = "Drawbot";
const char* password = "12345678";
WebServer server(80);

// =====================================================================
//  PARAMÈTRES PHYSIQUES DU ROBOT
// =====================================================================
const float WHEEL_RADIUS = 4.5f;
const int   ENCODER_CPR  = 4300;
const float WHEEL_BASE   = 8.0f;
const float PEN_OFFSET   = 13.0f;

// =====================================================================
//  PARAMÈTRES PID
// =====================================================================
const float KP_LIN = 4.0f;
const float KI_LIN = 0.08f;
const float KD_LIN = 1.5f;

const float KP_ANG = 1.2f;
const float KI_ANG = 0.02f;
const float KD_ANG = 0.6f;

// =====================================================================
//  PARAMÈTRES DE CONTRÔLE
// =====================================================================
const float POS_TOLERANCE   = 0.8f;
const float ANGLE_TOLERANCE = 3.0f;
const float MAX_SPEED       = 45.0f;
const float MIN_SPEED       = 18.0f;
const float TE_MS           = 20.0f;
const float TE_S            = TE_MS / 1000.0f;
const float ANTI_WINDUP_LIN = 30.0f;
const float ANTI_WINDUP_ANG = 15.0f;
const float DERIV_FILTER_N  = 10.0f;

// =====================================================================
//  PWM
// =====================================================================
const int pwmFreq       = 5000;
const int pwmResolution = 8;
const int pwmChannelD   = 0;
const int pwmChannelG   = 1;

// =====================================================================
//  OBJETS GLOBAUX
// =====================================================================
Adafruit_LSM6DS3 lsm6ds3;
Adafruit_LIS3MDL lis3mdl;
ESP32Encoder encoderD;
ESP32Encoder encoderG;

// Indique si les capteurs ont bien ete detectes au demarrage.
// Important : on ne bloque pas le programme si un capteur ne repond pas,
// sinon le WiFi et l'interface deviennent impossibles a utiliser.
bool imuOk = false;
bool magOk = false;

// =====================================================================
//  ÉTAT DU ROBOT
// =====================================================================
float robot_x     = -PEN_OFFSET;
float robot_y     = 0.0f;
float robot_angle = 0.0f;
float pen_x       = 0.0f;
float pen_y       = 0.0f;
long  prev_count_left  = 0;
long  prev_count_right = 0;

// =====================================================================
//  VARIABLES DEBUG / WEB
// =====================================================================
float current_target_angle = 0.0f;
float ang_error = 0.0f;
float lin_error = 0.0f;

float desired_circle_radius  = 5.0f;
float desired_north_angle    = 0.0f;   // angle reçu depuis Python pour la séquence 3
bool  should_draw_circle     = false;
bool  should_draw_stairs     = false;
bool  should_draw_arrow      = false;
bool  should_calibrate_mag   = false;
bool  should_turn_north      = false;
bool  manual_mode            = false;

// =====================================================================
//  PARAMÈTRES POUR LE SUIVI DE CERCLE PAR WAYPOINTS
// =====================================================================
std::vector<std::pair<float, float>> circle_waypoints;
bool should_follow_waypoints = false;
int current_waypoint_idx = 0;
float waypoint_tolerance = 0.5f;

// =====================================================================
//  CALIBRATION MAGNÉTOMÈTRE
// =====================================================================
float mag_x_min    = -91.51f,  mag_x_max = -43.82f;
float mag_y_min    = 20.93f,  mag_y_max = -69.92f;
bool  mag_calibrated = true;

// =====================================================================
//  OFFSET MÉCANIQUE CAPTEUR
// =====================================================================
const float MAG_MOUNTING_OFFSET = 152.0f;

// =====================================================================
//  ODOMÉTRIE
// =====================================================================
void updateOdometry() {
    long count_left  = encoderG.getCount();
    long count_right = encoderD.getCount();

    float dl = (float)(count_left  - prev_count_left)  * (2.0f * PI * WHEEL_RADIUS) / ENCODER_CPR;
    float dr = (float)(count_right - prev_count_right) * (2.0f * PI * WHEEL_RADIUS) / ENCODER_CPR;

    //dl = -dl;

    prev_count_left  = count_left;
    prev_count_right = count_right;

    float d_center = (dl + dr) / 2.0f;
    float d_theta  = (dr - dl) / WHEEL_BASE;

    float half_theta = d_theta / 2.0f;
    robot_angle += half_theta * (180.0f / PI);

    float angle_rad = robot_angle * (PI / 180.0f);
    robot_x += d_center * cos(angle_rad);
    robot_y += d_center * sin(angle_rad);

    robot_angle += half_theta * (180.0f / PI);

    robot_angle = fmod(robot_angle + 180.0f, 360.0f);
    if (robot_angle < 0.0f) robot_angle += 360.0f;
    robot_angle -= 180.0f;

    angle_rad = robot_angle * (PI / 180.0f);
    pen_x = robot_x + PEN_OFFSET * cos(angle_rad);
    pen_y = robot_y + PEN_OFFSET * sin(angle_rad);
}

// =====================================================================
//  NORMALISATION D'ANGLE
// =====================================================================
float normalizeAngle(float a) {
    a = fmod(a + 180.0f, 360.0f);
    if (a < 0) a += 360.0f;
    return a - 180.0f;
}

// =====================================================================
//  INITIALISATION ENCODEURS
// =====================================================================
static void initEncoders() {
    encoderD.attachFullQuad(ENC_D_CH_A, ENC_D_CH_B);
    encoderG.attachFullQuad(ENC_G_CH_B, ENC_G_CH_A);

    gpio_pullup_dis((gpio_num_t)ENC_D_CH_A);
    gpio_pullup_dis((gpio_num_t)ENC_D_CH_B);
    gpio_pullup_dis((gpio_num_t)ENC_G_CH_A);
    gpio_pullup_dis((gpio_num_t)ENC_G_CH_B);

    pinMode(ENC_D_CH_A, INPUT_PULLUP);
    pinMode(ENC_D_CH_B, INPUT_PULLUP);
    pinMode(ENC_G_CH_A, INPUT_PULLUP);
    pinMode(ENC_G_CH_B, INPUT_PULLUP);

    encoderD.clearCount();
    encoderG.clearCount();
    prev_count_left  = 0;
    prev_count_right = 0;
}

// =====================================================================
//  INITIALISATION MOTEURS
// =====================================================================
static void initMotors() {
    ledcSetup(pwmChannelD, pwmFreq, pwmResolution);
    ledcSetup(pwmChannelG, pwmFreq, pwmResolution);
    ledcAttachPin(EN_D, pwmChannelD);
    ledcAttachPin(EN_G, pwmChannelG);

    pinMode(IN_1_D, OUTPUT); digitalWrite(IN_1_D, LOW);
    pinMode(IN_2_D, OUTPUT); digitalWrite(IN_2_D, LOW);
    pinMode(IN_1_G, OUTPUT); digitalWrite(IN_1_G, LOW);
    pinMode(IN_2_G, OUTPUT); digitalWrite(IN_2_G, LOW);
}

// =====================================================================
//  COMMANDE MOTEURS
// =====================================================================
void setMotorSpeed(int speedD, int speedG) {
    auto mapSpeed = [](int input) -> int {
        if (input == 0) return 0;
        if (input > 0) return map(input, 0, 100, 100, 255);
        else           return -map(abs(input), 0, 100, 100, 255);
    };
    int pwmD = mapSpeed(speedD);
    int pwmG = mapSpeed(speedG);

    digitalWrite(IN_1_D, pwmD > 0 ? LOW  : HIGH);
    digitalWrite(IN_2_D, pwmD > 0 ? HIGH : LOW);
    ledcWrite(pwmChannelD, abs(pwmD));

    digitalWrite(IN_1_G, pwmG > 0 ? HIGH : LOW);
    digitalWrite(IN_2_G, pwmG > 0 ? LOW  : HIGH);
    ledcWrite(pwmChannelG, abs(pwmG));
}

static void brakeMotors() {
    digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, LOW);
    digitalWrite(IN_1_G, LOW); digitalWrite(IN_2_G, LOW);
    ledcWrite(pwmChannelD, 0);
    ledcWrite(pwmChannelG, 0);
}

// =====================================================================
//  PROFIL DE VITESSE
// =====================================================================
float speedProfile(float distance_remaining, float distance_total) {
    float dist_done = distance_total - distance_remaining;
    float accel = constrain(dist_done / 4.0f, 0.0f, 1.0f);
    float decel = constrain(distance_remaining / 3.0f, 0.0f, 1.0f);
    return max(min(accel, decel), 0.3f);
}

// =====================================================================
//  ALLER À UN POINT — PID
// =====================================================================
const float TURN_SPEED_MAX = 30.0f;
const float ANG_CMD_MAX    = 35.0f;

void goToPoint(float x_target, float y_target,
               float speed_limit = MAX_SPEED,
               bool chain_next = false,
               bool allow_overshoot_exit = true,
               float heading_threshold = 20.0f)
{
    float integral_lin = 0.0f;
    float integral_ang = 0.0f;
    float prev_err_lin = 0.0f;
    float prev_err_ang = 0.0f;
    float filtered_deriv_ang = 0.0f;

    updateOdometry();
    float dx = x_target - pen_x;
    float dy = y_target - pen_y;
    float target_angle   = atan2(dy, dx) * (180.0f / PI);
    float init_ang_err   = normalizeAngle(target_angle - robot_angle);
    bool  go_backward    = (fabs(init_ang_err) > 120.0f);
    float distance_total = hypot(dx, dy);

    if (distance_total < POS_TOLERANCE) {
        if (!chain_next) brakeMotors();
        return;
    }

    Serial.printf("[GoTo] (%.1f,%.1f) dist=%.1f\n", x_target, y_target, distance_total);

    unsigned long last_tick          = millis();
    unsigned long start_time         = millis();
    unsigned long GOTO_TIMEOUT_MS    = (unsigned long)(distance_total * 600.0f + 4000.0f);
    float         best_dist          = distance_total;
    unsigned long last_progress_time = millis();
    const unsigned long STALL_TIMEOUT_MS = 2500;
    const float         STALL_THRESHOLD  = 0.4f;

    while (true) {
        unsigned long now = millis();
        if (now - start_time > GOTO_TIMEOUT_MS)           { brakeMotors(); break; }
        if (now - last_progress_time > STALL_TIMEOUT_MS)  { brakeMotors(); break; }
        if (now - last_tick < (unsigned long)TE_MS)       { server.handleClient(); continue; }
        last_tick = now;

        updateOdometry();

        float ex       = x_target - pen_x;
        float ey       = y_target - pen_y;
        float dist_err = hypot(ex, ey);

        if (dist_err < best_dist - STALL_THRESHOLD) {
            best_dist = dist_err;
            last_progress_time = millis();
        }
        if (allow_overshoot_exit && dist_err > best_dist + 2.0f) { brakeMotors(); break; }

        float desired_angle  = atan2(ey, ex) * (180.0f / PI);
        float heading_target = go_backward ? normalizeAngle(desired_angle + 180.0f) : desired_angle;
        float heading_err    = normalizeAngle(heading_target - robot_angle);

        lin_error            = dist_err;
        ang_error            = heading_err;
        current_target_angle = heading_target;

        if (dist_err < POS_TOLERANCE) {
            brakeMotors();
            delay(chain_next ? 60 : 80);
            break;
        }

        float cmd_right, cmd_left;

        if (fabs(heading_err) > heading_threshold) {
            integral_lin = 0.0f;
            prev_err_lin = dist_err;
            float P   = KP_ANG * heading_err;
            integral_ang += KI_ANG * heading_err * TE_S;
            integral_ang  = constrain(integral_ang, -ANTI_WINDUP_ANG, ANTI_WINDUP_ANG);
            float raw_d   = (heading_err - prev_err_ang) / TE_S;
            float tau_d   = (KD_ANG / KP_ANG) / DERIV_FILTER_N;
            float alpha   = tau_d / (tau_d + TE_S);
            filtered_deriv_ang = alpha * filtered_deriv_ang + (1.0f - alpha) * KD_ANG * raw_d;
            prev_err_ang  = heading_err;
            float tc      = constrain(P + integral_ang + filtered_deriv_ang, -TURN_SPEED_MAX, TURN_SPEED_MAX);
            if (fabs(tc) < MIN_SPEED && fabs(heading_err) > ANGLE_TOLERANCE)
                tc = (tc > 0) ? MIN_SPEED : -MIN_SPEED;
            cmd_right = -tc; cmd_left = tc;

        } else {
            float P_lin  = KP_LIN * dist_err;
            integral_lin += KI_LIN * dist_err * TE_S;
            integral_lin  = constrain(integral_lin, -ANTI_WINDUP_LIN, ANTI_WINDUP_LIN);
            float D_lin   = KD_LIN * (dist_err - prev_err_lin) / TE_S;
            prev_err_lin  = dist_err;
            float lin_cmd = constrain(
                (P_lin + integral_lin + D_lin) * speedProfile(dist_err, distance_total),
                MIN_SPEED, speed_limit);
            if (go_backward) lin_cmd = -lin_cmd;

            float P_ang  = KP_ANG * heading_err;
            integral_ang += KI_ANG * heading_err * TE_S;
            integral_ang  = constrain(integral_ang, -ANTI_WINDUP_ANG, ANTI_WINDUP_ANG);
            float tau_d   = (KD_ANG / KP_ANG) / DERIV_FILTER_N;
            float alpha   = tau_d / (tau_d + TE_S);
            float raw_d   = (heading_err - prev_err_ang) / TE_S;
            filtered_deriv_ang = alpha * filtered_deriv_ang + (1.0f - alpha) * KD_ANG * raw_d;
            prev_err_ang  = heading_err;
            float ang_cmd = constrain(P_ang + integral_ang + filtered_deriv_ang, -ANG_CMD_MAX, ANG_CMD_MAX);

            cmd_right = lin_cmd + ang_cmd;
            cmd_left  = lin_cmd - ang_cmd;
        }

        cmd_right = constrain(cmd_right, -100.0f, 100.0f);
        cmd_left  = constrain(cmd_left,  -100.0f, 100.0f);
        setMotorSpeed((int)cmd_right, (int)cmd_left);
    }
}

// =====================================================================
//  TOURNER SUR PLACE VERS UN ANGLE ABSOLU
// =====================================================================
void turnToAngle(float target_deg) {
    float integral  = 0.0f;
    float prev_err  = 0.0f;
    unsigned long last_tick  = millis();
    unsigned long start_time = millis();
    const unsigned long TURN_TIMEOUT_MS = 5000;

    Serial.printf("[Turn] Vers %.1f°\n", target_deg);

    while (true) {
        unsigned long now = millis();
        if (now - start_time > TURN_TIMEOUT_MS)      { brakeMotors(); break; }
        if (now - last_tick < (unsigned long)TE_MS)  { server.handleClient(); continue; }
        last_tick = now;

        updateOdometry();
        float err = normalizeAngle(target_deg - robot_angle);
        if (fabs(err) < ANGLE_TOLERANCE) { brakeMotors(); delay(50); break; }

        float P   = KP_ANG * err;
        integral += KI_ANG * err * TE_S;
        integral  = constrain(integral, -ANTI_WINDUP_ANG, ANTI_WINDUP_ANG);
        float D   = KD_ANG * (err - prev_err) / TE_S;
        prev_err  = err;

        float cmd = constrain(P + integral + D, -TURN_SPEED_MAX, TURN_SPEED_MAX);
        if (fabs(cmd) < MIN_SPEED && fabs(err) > 1.0f)
            cmd = (cmd > 0) ? MIN_SPEED : -MIN_SPEED;

        setMotorSpeed((int)(-cmd), (int)(cmd));
    }
}

// =====================================================================
//  FONCTION ANGLE AMÉLIORÉE POUR COLORIER
// =====================================================================
void angle(float angle_deg, int speed = 30) {
    const float CPR_D = 4212.0f;
    const float CPR_G = 4312.0f;

    speed = constrain(speed, 15, 60);

    float angle_rad = abs(angle_deg) * PI / 180.0f;
    float distance_roue = angle_rad * (WHEEL_BASE / 2.0f);

    float tour_roue = distance_roue / (2.0f * PI * WHEEL_RADIUS);

    long targetD = (long)(tour_roue * CPR_D);
    long targetG = (long)(tour_roue * CPR_G);

    encoderD.clearCount();
    encoderG.clearCount();

    Serial.printf("[angle] angle=%.1f° | targetD=%ld | targetG=%ld\n",
                  angle_deg, targetD, targetG);

    int speedD, speedG;

    if (angle_deg > 0) {
        speedD = -speed;
        speedG =  speed;
    } else {
        speedD =  speed;
        speedG = -speed;
    }

    unsigned long start_time = millis();
    const unsigned long TIMEOUT_MS = 8000;

    while (true) {
        server.handleClient();

        long countD = abs(encoderD.getCount());
        long countG = abs(encoderG.getCount());

        bool doneD = countD >= targetD;
        bool doneG = countG >= targetG;

        if (doneD && doneG) {
            break;
        }

        int cmdD = doneD ? 0 : speedD;
        int cmdG = doneG ? 0 : speedG;

        setMotorSpeed(cmdD, cmdG);

        if (millis() - start_time > TIMEOUT_MS) {
            Serial.println("[angle] Timeout !");
            break;
        }

        delay(10);
    }

    brakeMotors();
    delay(100);

    Serial.printf("[angle] Fin | encD=%ld | encG=%ld\n",
                  encoderD.getCount(), encoderG.getCount());
}

// =====================================================================
//  FONCTION POUR GÉNÉRER UN CERCLE EN WAYPOINTS
// =====================================================================
void generateCircleWaypoints(float cx, float cy, float radius, int num_points = 48) {
    circle_waypoints.clear();
    for (int i = 0; i <= num_points; i++) {
        float angle_step = 2.0f * PI * i / num_points;
        float x = cx + radius * cos(angle_step);
        float y = cy + radius * sin(angle_step);
        circle_waypoints.push_back({x, y});
    }
    Serial.printf("[Waypoints] Généré %d points pour cercle R=%.1f\n", 
                  (int)circle_waypoints.size(), radius);
}

// =====================================================================
//  FONCTION POUR SUIVRE UNE LISTE DE WAYPOINTS AVEC PID
// =====================================================================
void followWaypoints(const std::vector<std::pair<float, float>>& waypoints,
                     float speed_limit = MAX_SPEED,
                     float tolerance = 0.8f) {
    
    for (size_t i = 0; i < waypoints.size(); i++) {
        float tx = waypoints[i].first;
        float ty = waypoints[i].second;
        
        Serial.printf("[Waypoint %d/%d] (%.1f, %.1f)\n", 
                      i+1, (int)waypoints.size(), tx, ty);
        
        goToPoint(tx, ty, speed_limit, false, true, 20.0f);
        
        updateOdometry();
        float dx = tx - pen_x;
        float dy = ty - pen_y;
        if (hypot(dx, dy) > tolerance) {
            goToPoint(tx, ty, speed_limit * 0.7f, false, true, 20.0f);
        }
        
        server.handleClient();
    }
}

// =====================================================================
//  SÉQUENCE CERCLE AVEC WAYPOINTS (PID)
// =====================================================================
void drawCircleSmooth(float radius, float speed_cms = 12.0f) {
    Serial.printf("\n=== CERCLE LISSE R=%.1f cm ===\n", radius);
    
    // 1. Aller au point de départ (stylo à (radius, 0))
    float start_x = radius;
    float start_y = 0;
    goToPoint(start_x, start_y, MAX_SPEED, false, true, 20.0f);
    delay(400);
    
    // 2. S'orienter tangentiellement (90° pour cercle anti-horaire)
    turnToAngle(90.0f);
    delay(300);
    
    updateOdometry();
    Serial.printf("[Cercle] Départ stylo: (%.2f, %.2f), angle=%.1f°\n", 
                  pen_x, pen_y, robot_angle);
    
    // 3. Calcul des paramètres
    float omega_target = speed_cms / radius;           // rad/s (vitesse angulaire cible)
    float angle_total = 2.0f * PI;                      // 360° en radians
    float duration_s = angle_total / omega_target;      // durée totale du cercle (secondes)
    float duration_ms = duration_s * 1000.0f;
    
    // Vitesses roues pour virage différentiel pur
    float R_axle = radius - PEN_OFFSET;
    if (R_axle < 0.5f) R_axle = 0.5f;  // sécurité
    
    float v_right = omega_target * (R_axle + WHEEL_BASE / 2.0f);
    float v_left  = omega_target * (R_axle - WHEEL_BASE / 2.0f);
    
    // Normaliser pour que la roue la plus rapide = MAX_SPEED %
    float v_max = max(fabs(v_right), fabs(v_left));
    float speed_limit = 35.0f;  // vitesse max %
    float scale = speed_limit / v_max;
    
    int pwm_right = (int)(v_right * scale);
    int pwm_left  = (int)(v_left * scale);
    
    pwm_right = constrain(pwm_right, -100, 100);
    pwm_left  = constrain(pwm_left, -100, 100);
    
    Serial.printf("[Cercle] vR=%d%%, vL=%d%%, omega=%.3f rad/s, duree=%.1fs\n",
                  pwm_right, pwm_left, omega_target, duration_s);
    
    // 4. Variables pour PID de correction (pour garder le rayon constant)
    unsigned long last_time = micros();
    float integral_radius = 0.0f;
    float prev_radius_error = 0.0f;
    float target_radius = radius;
    
    // 5. Boucle principale - maintient le cercle par PID
    unsigned long start_ms = millis();
    float angle_traveled = 0.0f;
    
    while (millis() - start_ms < duration_ms) {
        unsigned long now_us = micros();
        float dt = (now_us - last_time) / 1000000.0f;
        if (dt < 0.005f) {  // min 5ms
            delay(1);
            continue;
        }
        last_time = now_us;
        
        updateOdometry();
        
        // Calculer le rayon actuel à partir de la position du stylo
        float current_radius = hypot(pen_x, pen_y);
        float radius_error = target_radius - current_radius;
        
        // PID pour corriger le rayon
        const float KR_P = 2.5f;   // Proportionnel
        const float KR_I = 0.05f;  // Intégral
        const float KR_D = 0.8f;   // Dérivé
        
        integral_radius += KR_I * radius_error * dt;
        integral_radius = constrain(integral_radius, -15.0f, 15.0f);
        
        float derivative = (radius_error - prev_radius_error) / dt;
        prev_radius_error = radius_error;
        
        float radius_correction = KR_P * radius_error + integral_radius + KR_D * derivative;
        radius_correction = constrain(radius_correction, -12.0f, 12.0f);
        
        // Appliquer la correction en modulant la différence de vitesse
        int pwmR = pwm_right + (int)radius_correction;
        int pwmL = pwm_left  - (int)radius_correction;
        
        pwmR = constrain(pwmR, -100, 100);
        pwmL = constrain(pwmL, -100, 100);
        
        setMotorSpeed(pwmR, pwmL);
        
        // Calculer l'angle parcouru (pour debug)
        static float prev_angle = robot_angle;
        float delta_angle = fabs(normalizeAngle(robot_angle - prev_angle));
        if (delta_angle < 180) {
            angle_traveled += fabs(delta_angle) * (PI / 180.0f);
        }
        prev_angle = robot_angle;
        
        // Debug périodique
        static unsigned long last_debug = 0;
        if (millis() - last_debug > 1000) {
            Serial.printf("[Cercle] r=%.2f (err=%.2f), angle=%.1f°/%.1f°, corr=%.1f\n",
                          current_radius, radius_error, angle_traveled * 180/PI, 
                          angle_total * 180/PI, radius_correction);
            last_debug = millis();
        }
        
        server.handleClient();
        delay(TE_MS);
    }
    
    // 6. Arrêt et vérification
    brakeMotors();
    delay(500);
    
    updateOdometry();
    float final_radius = hypot(pen_x, pen_y);
    float closure_error = hypot(pen_x - start_x, pen_y - start_y);
    Serial.printf("✓ CERCLE LISSE | rayon final=%.2f cm, erreur fermeture=%.2f cm\n", 
                  final_radius, closure_error);
}

// =====================================================================
//  CERCLE PAR ASSERVISSEMENT DE LA DIFFÉRENCE D'ENCODEURS
//  Méthode plus robuste basée sur l'odométrie
// =====================================================================
void drawCircleEncoderAsserv(float radius) {
    Serial.printf("\n=== CERCLE ASSERV. ENCODEURS R=%.1f cm ===\n", radius);
    
    // 1. Position de départ
    float start_x = radius;
    float start_y = 0;
    goToPoint(start_x, start_y, MAX_SPEED, false, true, 20.0f);
    turnToAngle(90.0f);
    delay(300);
    
    // 2. Objectif : parcourir exactement 2*PI*radius cm avec le stylo
    float target_perimeter = 2.0f * PI * radius;  // cm à parcourir
    float distance_per_tour_roue = 2.0f * PI * WHEEL_RADIUS;
    float tours_roue_necessaires = target_perimeter / distance_per_tour_roue;
    long target_encoder_total = (long)(tours_roue_necessaires * ENCODER_CPR);
    
    // Pour un cercle, la roue extérieure parcourt plus de distance
    float R_ext = radius + WHEEL_BASE/2;
    float R_int = radius - WHEEL_BASE/2;
    float ratio = R_ext / R_int;
    
    // Encodeurs cibles
    long target_enc_ext = (long)(target_encoder_total * ratio / (1 + ratio));
    long target_enc_int = target_encoder_total - target_enc_ext;
    
    encoderD.clearCount();
    encoderG.clearCount();
    
    Serial.printf("[Cercle] target_perimeter=%.1f cm, enc_ext=%ld, enc_int=%ld\n",
                  target_perimeter, target_enc_ext, target_enc_int);
    
    // 3. Vitesse de base
    int base_speed = 35;
    int speed_ext = base_speed;
    int speed_int = (int)(base_speed / ratio);
    if (speed_int < 12) speed_int = 12;
    
    // Identifier quelle roue est extérieure (cercle anti-horaire = gauche extérieure)
    int speed_out = speed_ext;   // roue gauche (extérieure)
    int speed_in = -speed_int;   // roue droite (intérieure, tourne en sens inverse)
    
    unsigned long start_time = millis();
    const unsigned long TIMEOUT_MS = 30000;
    
    // 4. PID pour maintenir le ratio
    float integral = 0.0f;
    float prev_error = 0.0f;
    const float KP_RATIO = 0.5f;
    const float KI_RATIO = 0.01f;
    const float KD_RATIO = 0.1f;
    
    while (true) {
        server.handleClient();
        
        long enc_left = abs(encoderG.getCount());
        long enc_right = abs(encoderD.getCount());
        
        // Vérifier si on a fini
        bool done_left = enc_left >= target_enc_ext;
        bool done_right = enc_right >= target_enc_int;
        
        if (done_left && done_right) {
            break;
        }
        
        if (millis() - start_time > TIMEOUT_MS) {
            Serial.println("[Cercle] Timeout!");
            break;
        }
        
        // Calculer l'erreur de ratio (deviation par rapport au cercle parfait)
        float expected_enc_left = enc_right * ratio;
        float ratio_error = expected_enc_left - enc_left;
        
        // PID sur l'erreur de ratio
        float dt = 0.05f;
        integral += KI_RATIO * ratio_error * dt;
        integral = constrain(integral, -20.0f, 20.0f);
        float derivative = (ratio_error - prev_error) / dt;
        prev_error = ratio_error;
        
        float correction = KP_RATIO * ratio_error + integral + KD_RATIO * derivative;
        correction = constrain(correction, -15.0f, 15.0f);
        
        // Ajuster les vitesses
        int cmd_out = (int)(speed_out + correction);
        int cmd_in = (int)(speed_in - correction);
        
        cmd_out = constrain(cmd_out, 10, 60);
        cmd_in = constrain(cmd_in, -60, -10);
        
        setMotorSpeed(cmd_in, cmd_out);  // droit, gauche
        
        delay(TE_MS);
    }
    
    brakeMotors();
    delay(300);
    
    updateOdometry();
    float final_radius = hypot(pen_x, pen_y);
    Serial.printf("✓ CERCLE ASSERV | rayon final=%.2f cm\n", final_radius);
}
// =====================================================================
//  FONCTION POUR COLORIER LE TRIANGLE DE LA FLÈCHE
// =====================================================================
void fillTriangle(float tip_x, float tip_y, float angle_rad,
                  float branch_len, float branch_angle_rad) {
    
    // Calcul des trois points du triangle
    float P1x = tip_x;
    float P1y = tip_y;
    
    float P2x = tip_x + branch_len * cos(angle_rad + PI - branch_angle_rad);
    float P2y = tip_y + branch_len * sin(angle_rad + PI - branch_angle_rad);
    
    float P3x = tip_x + branch_len * cos(angle_rad + PI + branch_angle_rad);
    float P3y = tip_y + branch_len * sin(angle_rad + PI + branch_angle_rad);
    
    Serial.printf("[FillTriangle] Points: P1=(%.1f,%.1f) P2=(%.1f,%.1f) P3=(%.1f,%.1f)\n",
                  P1x, P1y, P2x, P2y, P3x, P3y);
    
    // Calcul du centre du triangle
    float center_x = (P1x + P2x + P3x) / 3.0f;
    float center_y = (P1y + P2y + P3y) / 3.0f;
    
    float min_x = min(P1x, min(P2x, P3x));
    float max_x = max(P1x, max(P2x, P3x));
    float min_y = min(P1y, min(P2y, P3y));
    float max_y = max(P1y, max(P2y, P3y));
    
    // Remplissage par lignes horizontales
    float step_y = 0.4f;
    float line_step_x = 0.3f;
    
    for (float y = min_y + step_y; y <= max_y - step_y; y += step_y) {
        // Trouver les intersections avec les bords du triangle à cette hauteur
        std::vector<float> intersections;
        
        // Vérifier chaque arête du triangle
        float x1, y1, x2, y2;
        
        // Arête P1-P2
        x1 = P1x; y1 = P1y; x2 = P2x; y2 = P2y;
        if ((y1 <= y && y2 >= y) || (y2 <= y && y1 >= y)) {
            if (y1 != y2) {
                float t = (y - y1) / (y2 - y1);
                float ix = x1 + t * (x2 - x1);
                intersections.push_back(ix);
            }
        }
        
        // Arête P2-P3
        x1 = P2x; y1 = P2y; x2 = P3x; y2 = P3y;
        if ((y1 <= y && y2 >= y) || (y2 <= y && y1 >= y)) {
            if (y1 != y2) {
                float t = (y - y1) / (y2 - y1);
                float ix = x1 + t * (x2 - x1);
                intersections.push_back(ix);
            }
        }
        
        // Arête P3-P1
        x1 = P3x; y1 = P3y; x2 = P1x; y2 = P1y;
        if ((y1 <= y && y2 >= y) || (y2 <= y && y1 >= y)) {
            if (y1 != y2) {
                float t = (y - y1) / (y2 - y1);
                float ix = x1 + t * (x2 - x1);
                intersections.push_back(ix);
            }
        }
        
        // Trier les intersections
        if (intersections.size() >= 2) {
            std::sort(intersections.begin(), intersections.end());
            float left = intersections[0] + line_step_x;
            float right = intersections[intersections.size()-1] - line_step_x;
            
            if (left < right) {
                // Aller au point de départ de la ligne
                goToPoint(left, y, 12.0f, false, true, 15.0f);
                delay(50);
                
                // Tracer la ligne horizontale
                goToPoint(right, y, 12.0f, false, true, 15.0f);
                delay(30);
            }
        }
        
        server.handleClient();
    }
    
    delay(200);
}

// =====================================================================
//  SÉQUENCE N°1 : ESCALIER
// =====================================================================
void drawStaircase() {
    setMotorSpeed(96, 92);
    delay(780);
    brakeMotors(); delay(400);
    angle(80.0);
    brakeMotors(); delay(400);
    setMotorSpeed(-82, -82);
    delay(665);
    brakeMotors(); delay(400);
    setMotorSpeed(46, 45);
    delay(500);
    brakeMotors(); delay(400);
    angle(83);
    brakeMotors(); delay(400);
    setMotorSpeed(-96, -95);
    delay(1360);
    brakeMotors(); delay(400);
}
// =====================================================================
//  LIRE LE NORD MAGNÉTIQUE
// =====================================================================
float readMagneticNorth() {
    float mx_sum = 0, my_sum = 0;
    for (int i = 0; i < 50; i++) {
        sensors_event_t accel, gyro, temp, mag;
        lsm6ds3.getEvent(&accel, &gyro, &temp);
        lis3mdl.getEvent(&mag);
        mx_sum += mag.magnetic.x;
        my_sum += mag.magnetic.y;
        delay(10);
    }
       float mx = mx_sum / 50.0f;
    float my = my_sum / 50.0f;

    // Correction hard-iron
    if (mag_calibrated) {
        mx -= (mag_x_min + mag_x_max) / 2.0f;
        my -= (mag_y_min + mag_y_max) / 2.0f;
    }

    // Angle brut du capteur
    float cap_capteur = atan2(my, mx) * (180.0f / PI);

    // Offset de montage mécanique
    float cap_relatif = normalizeAngle(cap_capteur - MAG_MOUNTING_OFFSET);

    // Angle absolu dans le repère odométrie
    float cap_absolu = normalizeAngle(robot_angle + cap_relatif);

    Serial.printf("[Mag] mx=%.2f my=%.2f | brut=%.1f° | relatif=%.1f° | absolu=%.1f° | robot_angle=%.1f°\n",
                  mx, my, cap_capteur, cap_relatif, cap_absolu, robot_angle);

    return cap_absolu;
}

// =====================================================================
//  SÉQUENCE N°3 : FLÈCHE NORD AVEC TRIANGLE REMPLI
// =====================================================================
void drawNorthArrow(float north_angle_deg) {
   float angles = north_angle_deg-10;
   angle(-angles);
    brakeMotors();
    delay(600);
    setMotorSpeed(46, 45);
    delay(500);
    brakeMotors(); delay(400);
    angle(1);
    brakeMotors(); delay(400);
    angle(-1);
    brakeMotors(); delay(400);
    setMotorSpeed(-100, -100);
    delay(5);
    brakeMotors(); delay(400);
    angle(1);
    brakeMotors(); delay(400);
    angle(-1);
    brakeMotors(); delay(400);
    setMotorSpeed(-100, -100);
    delay(5);
    brakeMotors(); delay(400);
    angle(2);
    brakeMotors(); delay(400);
    angle(-2);
    brakeMotors(); delay(400);
    setMotorSpeed(-100, -100);
    delay(5);
    brakeMotors(); delay(400);
    setMotorSpeed(-100, -100);
    delay(5);
    brakeMotors(); delay(400);
    angle(2);
    brakeMotors(); delay(400);
    angle(-2);
    brakeMotors(); delay(400);
    setMotorSpeed(-100, -100);
    delay(5);
    brakeMotors(); delay(400);
    angle(3);
    brakeMotors(); delay(400);
    setMotorSpeed(-100, -100);
    delay(5);
    brakeMotors(); delay(400);
    angle(-3);
    brakeMotors(); delay(400);
    setMotorSpeed(-100, -100);
    delay(5);
    brakeMotors(); delay(400);

}

// =====================================================================
//  CALIBRATION MAGNÉTOMÈTRE
// =====================================================================
void calibrateMagnetometer() {
    Serial.println("[Mag] Calibration : robot tourne sur lui-même 6 secondes...");
    Serial.println("[Mag] Eloignez le robot de tout metal et cable !");

    mag_x_min =  1000.0f; mag_x_max = -1000.0f;
    mag_y_min =  1000.0f; mag_y_max = -1000.0f;

    unsigned long start = millis();

    setMotorSpeed(-40, 40);

    while (millis() - start < 6000) {
        sensors_event_t accel, gyro, temp, mag;
        lsm6ds3.getEvent(&accel, &gyro, &temp);
        lis3mdl.getEvent(&mag);

        if (mag.magnetic.x < mag_x_min) mag_x_min = mag.magnetic.x;
        if (mag.magnetic.x > mag_x_max) mag_x_max = mag.magnetic.x;
        if (mag.magnetic.y < mag_y_min) mag_y_min = mag.magnetic.y;
        if (mag.magnetic.y > mag_y_max) mag_y_max = mag.magnetic.y;

        server.handleClient();
        delay(20);
    }

    brakeMotors();
    mag_calibrated = true;

    Serial.println("[Mag] Calibration OK ! Copiez ces valeurs dans le code :");
    Serial.printf("  mag_x_min = %.2ff;  mag_x_max = %.2ff;\n", mag_x_min, mag_x_max);
    Serial.printf("  mag_y_min = %.2ff;  mag_y_max = %.2ff;\n", mag_y_min, mag_y_max);
    Serial.println("  mag_calibrated = true;");
}

// =====================================================================
//  RESET POSITION
// =====================================================================
void resetPositionOnly() {
    encoderD.clearCount();
    encoderG.clearCount();
    prev_count_left  = 0;
    prev_count_right = 0;
    robot_x = -PEN_OFFSET * cos(robot_angle * PI / 180.0f);
    robot_y = -PEN_OFFSET * sin(robot_angle * PI / 180.0f);
    pen_x   = 0.0f;
    pen_y   = 0.0f;
}

void resetOdometry() {
    encoderD.clearCount();
    encoderG.clearCount();
    prev_count_left  = 0;
    prev_count_right = 0;
    robot_x     = -PEN_OFFSET;
    robot_y     = 0.0f;
    robot_angle = 0.0f;
    pen_x       = 0.0f;
    pen_y       = 0.0f;
}

// =====================================================================
//  SERVEUR WEB
// =====================================================================
void addCORSHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
}

void handleRoot() {
    String html = R"=====(
<!DOCTYPE html><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Drawbot v10 - PID Waypoints</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
    .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
    h1 { color: #2c3e50; text-align: center; }
    .section { background: #ecf0f1; padding: 15px; border-radius: 5px; margin: 15px 0; }
    .section h2 { color: #34495e; margin-top: 0; }
    .data-panel { background: #3498db; color: white; padding: 12px; border-radius: 5px; margin: 10px 0; font-size: 14px; }
    .label { font-weight: bold; } .value { float: right; }
    input { padding: 8px; margin: 5px 0; width: 100%; box-sizing: border-box; }
    button { background: #27ae60; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; width: 100%; margin: 5px 0; }
    button:hover { background: #229954; }
    .btn-orange { background: #e67e22; } .btn-orange:hover { background: #ca6f1e; }
    .btn-blue   { background: #2980b9; } .btn-blue:hover   { background: #1a6a9a; }
    .btn-purple { background: #8e44ad; } .btn-purple:hover { background: #7d3c98; }
    .info { background: #fff3cd; border: 1px solid #ffc107; padding: 10px; border-radius: 5px; margin: 10px 0; }
  </style>
</head>
<body><div class="container">
  <h1>Drawbot v10 - PID Waypoints</h1>
  <div class="section">
    <h2>Position du Stylo</h2>
    <div class="data-panel"><span class="label">X:</span><span class="value">%X_VALUE% cm</span></div>
    <div class="data-panel"><span class="label">Y:</span><span class="value">%Y_VALUE% cm</span></div>
    <div class="data-panel"><span class="label">Angle:</span><span class="value">%ANGLE_VALUE%</span></div>
    <div class="data-panel"><span class="label">Err Lin:</span><span class="value">%LIN_ERR% cm</span></div>
    <div class="data-panel"><span class="label">Err Ang:</span><span class="value">%ANG_ERR%</span></div>
  </div>
  <div class="section">
    <h2>Cercle (Sequence 2) - AVEC PID WAYPOINTS</h2>
    <div class="info">Le cercle est genere en 48 points et suivi par PID pour une precision maximale</div>
    <label>Rayon (cm):</label>
    <input type="number" id="radius" min="2" max="20" step="0.5" value="5" />
    <button class="btn-blue" onclick="fetch('/circle?radius='+document.getElementById('radius').value).then(()=>alert('Cercle PID lance'))">Dessiner Cercle (PID Waypoints)</button>
  </div>
  <div class="section">
    <h2>Escalier (Sequence 1)</h2>
    <button onclick="fetch('/stairs').then(()=>alert('Escalier lance'))">Dessiner Escalier</button>
  </div>
  <div class="section">
    <h2>Fleche Nord (Sequence 3) - AVEC TRIANGLE REMPLI</h2>
    <div class="info">La fleche est dessinee puis le triangle est rempli automatiquement</div>
    <button class="btn-orange" onclick="fetch('/calibrate_mag').then(()=>alert('Calibration lancee (6 sec)'))">Calibrer Magnetometre</button>
    <label>Angle reçu depuis Python / angle cible (°):</label>
    <input type="number" id="north_angle" min="-180" max="180" step="1" value="0" />
    <button class="btn-purple" onclick="fetch('/turn_north?north_angle='+document.getElementById('north_angle').value).then(()=>alert('Rotation vers angle lancee'))">Tourner vers l'angle cible (test)</button>
    <button onclick="fetch('/arrow?north_angle='+document.getElementById('north_angle').value).then(()=>alert('Fleche avec angle cible lancee'))">Dessiner Fleche + Remplissage</button>
  </div>
  <div class="section">
    <h2>Reset</h2>
    <button onclick="fetch('/reset').then(()=>alert('Reset OK'))">Reset Position</button>
  </div>
</div>
<script>setInterval(()=>location.reload(), 1500);</script>
</body></html>
)=====";
    html.replace("%X_VALUE%",     String(pen_x, 2));
    html.replace("%Y_VALUE%",     String(pen_y, 2));
    html.replace("%ANGLE_VALUE%", String(robot_angle, 1));
    html.replace("%LIN_ERR%",     String(lin_error, 2));
    html.replace("%ANG_ERR%",     String(ang_error, 1));
    server.send(200, "text/html", html);
}

void handleData() {
    addCORSHeaders();
    server.send(200, "application/json",
        "{\"x\":"            + String(pen_x, 2) +
        ",\"y\":"            + String(pen_y, 2) +
        ",\"angle\":"        + String(robot_angle, 1) +
        ",\"target_angle\":" + String(current_target_angle, 1) +
        ",\"ang_error\":"    + String(ang_error, 1) +
        ",\"lin_error\":"    + String(lin_error, 2) + "}");
}

void handleCircleRequest() {
    if (!server.hasArg("radius")) { server.send(400, "text/plain", "radius manquant"); return; }
    float r = server.arg("radius").toFloat();
    if (r < 2.0f || r > 20.0f) { server.send(400, "text/plain", "Rayon invalide"); return; }
    desired_circle_radius = r;
    should_draw_circle    = true;
    addCORSHeaders();
    server.send(200, "application/json", "{\"ok\":true,\"radius\":" + String(r, 1) + ",\"method\":\"PID_waypoints\"}");
}

void handleStairsRequest() {
    should_draw_stairs = true;
    addCORSHeaders();
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleArrowRequest() {
    if (server.hasArg("north_angle")) {
        desired_north_angle = normalizeAngle(server.arg("north_angle").toFloat());
    } else if (server.hasArg("angle")) {
        // Alias pratique si on appelle /arrow?angle=...
        desired_north_angle = normalizeAngle(server.arg("angle").toFloat());
    }

    should_draw_arrow = true;
    addCORSHeaders();
    server.send(200, "application/json",
        "{\"ok\":true,\"north_angle\":" + String(desired_north_angle, 1) + "}");
}

void handleCalibrateMagRequest() {
    should_calibrate_mag = true;
    addCORSHeaders();
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleTurnNorthRequest() {
    if (server.hasArg("north_angle")) {
        desired_north_angle = normalizeAngle(server.arg("north_angle").toFloat());
    } else if (server.hasArg("angle")) {
        desired_north_angle = normalizeAngle(server.arg("angle").toFloat());
    }

    should_turn_north = true;
    addCORSHeaders();
    server.send(200, "application/json",
        "{\"ok\":true,\"north_angle\":" + String(desired_north_angle, 1) + "}");
}

void handleMove() {
    if (!server.hasArg("dir")) { server.send(400, "application/json", "{\"error\":\"dir manquant\"}"); return; }
    String dir   = server.arg("dir");
    int    speed = server.hasArg("speed") ? server.arg("speed").toInt() : 60;
    speed = constrain(speed, 0, 100);
    manual_mode = true;

    if      (dir == "fwd")   setMotorSpeed( speed,  speed);
    else if (dir == "back")  setMotorSpeed(-speed, -speed);
    else if (dir == "left")  setMotorSpeed( speed, -speed);
    else if (dir == "right") setMotorSpeed(-speed,  speed);
    else { server.send(400, "application/json", "{\"error\":\"dir invalide\"}"); return; }

    addCORSHeaders();
    server.send(200, "application/json", "{\"ok\":true,\"dir\":\"" + dir + "\",\"speed\":" + speed + "}");
}

void handleStop() {
    manual_mode = false; brakeMotors();
    addCORSHeaders();
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleIMU() {
    sensors_event_t accel, gyro, temp, mag;
    memset(&accel, 0, sizeof(accel));
    memset(&gyro,  0, sizeof(gyro));
    memset(&temp,  0, sizeof(temp));
    memset(&mag,   0, sizeof(mag));

    if (imuOk) {
        lsm6ds3.getEvent(&accel, &gyro, &temp);
    }
    if (magOk) {
        lis3mdl.getEvent(&mag);
    }

    addCORSHeaders();
    server.send(200, "application/json",
        "{\"imu_ok\":" + String(imuOk ? "true" : "false") +
        ",\"mag_ok\":" + String(magOk ? "true" : "false") +
        ",\"ax\":" + String(accel.acceleration.x, 3) +
        ",\"ay\":" + String(accel.acceleration.y, 3) +
        ",\"az\":" + String(accel.acceleration.z, 3) +
        ",\"gx\":" + String(gyro.gyro.x * 180.0f / PI, 2) +
        ",\"gy\":" + String(gyro.gyro.y * 180.0f / PI, 2) +
        ",\"gz\":" + String(gyro.gyro.z * 180.0f / PI, 2) +
        ",\"mx\":" + String(mag.magnetic.x, 2) +
        ",\"my\":" + String(mag.magnetic.y, 2) +
        ",\"mz\":" + String(mag.magnetic.z, 2) + "}");
}

void handleEncoders() {
    addCORSHeaders();
    server.send(200, "application/json",
        "{\"enc_left\":"  + String(encoderG.getCount()) +
        ",\"enc_right\":" + String(encoderD.getCount()) + "}");
}

void handleReset() {
    resetOdometry();
    addCORSHeaders();
    server.send(200, "application/json", "{\"ok\":true}");
}

// =====================================================================
//  WiFi
// =====================================================================
void initWiFi() {
    Serial.println();
    Serial.println("===== DEMARRAGE WIFI DRAWBOT =====");

    // On force le mode point d'acces pour que le robot cree son propre WiFi.
    WiFi.persistent(false);
    WiFi.disconnect(true, true);
    delay(300);
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);

    IPAddress localIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(localIP, gateway, subnet);

    // Canal 6, reseau visible, 4 clients max.
    bool ok = WiFi.softAP(ssid, password, 6, 0, 4);
    delay(500);

    if (ok) {
        Serial.println("WiFi AP demarre correctement");
        Serial.print("Nom du WiFi : ");
        Serial.println(ssid);
        Serial.print("Mot de passe : ");
        Serial.println(password);
        Serial.print("Interface : http://");
        Serial.println(WiFi.softAPIP());
        Serial.print("MAC AP : ");
        Serial.println(WiFi.softAPmacAddress());
    } else {
        Serial.println("ERREUR : impossible de demarrer le WiFi AP");
    }

    Serial.println("==================================");
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LEDU1, OUTPUT);
    pinMode(LEDU2, OUTPUT);
    digitalWrite(LEDU1, HIGH);
    digitalWrite(LEDU2, LOW);

    // 1) On demarre le WiFi en premier.
    // Comme ca, meme si un capteur pose probleme, le reseau Drawbot apparait quand meme.
    initWiFi();

    // 2) On demarre tout de suite le serveur web.
    server.on("/",             handleRoot);
    server.on("/data",         handleData);
    server.on("/circle",       handleCircleRequest);
    server.on("/stairs",       handleStairsRequest);
    server.on("/arrow",        handleArrowRequest);
    server.on("/calibrate_mag",handleCalibrateMagRequest);
    server.on("/turn_north",   handleTurnNorthRequest);
    server.on("/move",         handleMove);
    server.on("/stop",         handleStop);
    server.on("/imu",          handleIMU);
    server.on("/encoders",     handleEncoders);
    server.on("/reset",        handleReset);
    server.begin();
    Serial.println("Serveur web demarre");

    // 3) Initialisation I2C et capteurs. On ne bloque plus si un capteur ne repond pas.
    Wire.begin(PIN_SDA, PIN_SCL, 400000);

    imuOk = lsm6ds3.begin_I2C(ADDR_LSM6DS3);
    if (imuOk) {
        lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
        lsm6ds3.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
        Serial.println("IMU LSM6DS3 detectee");
    } else {
        Serial.println("ATTENTION : IMU LSM6DS3 non detectee, le WiFi reste actif");
    }

    magOk = lis3mdl.begin_I2C(ADDR_LIS3MDL);
    if (magOk) {
        lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
        Serial.println("Magnetometre LIS3MDL detecte");
    } else {
        Serial.println("ATTENTION : magnetometre LIS3MDL non detecte, le WiFi reste actif");
    }

    // 4) Initialisation actionneurs et encodeurs.
    initMotors();
    initEncoders();
    resetOdometry();

    digitalWrite(LEDU1, LOW);
    digitalWrite(LEDU2, HIGH);

    Serial.println("Drawbot pret !");
    Serial.print("Connectez-vous au WiFi : ");
    Serial.println(ssid);
    Serial.print("Puis ouvrez : http://");
    Serial.println(WiFi.softAPIP());
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
    server.handleClient();

    if (should_draw_stairs) {
        should_draw_stairs = false;
        resetOdometry();
        drawStaircase();
    }

    if (should_draw_circle) {
        should_draw_circle = false;
        resetOdometry();
         //drawCircleSmooth(desired_circle_radius);           // Méthode 1
    drawCircleEncoderAsserv(desired_circle_radius); // Méthode 2
        Serial.printf("✓ Cercle PID R=%.1f cm termine\n", desired_circle_radius);
    }

    if (should_calibrate_mag) {
        should_calibrate_mag = false;
        calibrateMagnetometer();
    }

    if (should_turn_north) {
        should_turn_north = false;
        brakeMotors();
        delay(600);
        updateOdometry();
        Serial.printf("[Test Angle] Cible reçue = %.1f°\n", desired_north_angle);
        turnToAngle(desired_north_angle);
        Serial.printf("[Test Angle] OK — robot_angle=%.1f°\n", robot_angle);
    }

    if (should_draw_arrow) {
        should_draw_arrow = false;
        drawNorthArrow(desired_north_angle);
    }

    updateOdometry();
    delay(10);
}
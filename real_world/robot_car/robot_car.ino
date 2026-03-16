/*
 * MIT 1.104 Lab 7 – Robot Car PID Controller (ESP32)
 *
 * Firmware uploaded ONCE by the TA.  The car boots into IDLE mode
 * (motors off, WiFi SoftAP active).  A laptop Python script sends
 * PID gains, a START command, and a continuous distance stream.
 * The car runs the PID loop until it receives STOP (or the stream
 * times out).
 *
 * UDP message protocol  (laptop → car):
 *   PID:Kp,Ki,Kd,setpoint   – configure gains  (e.g. PID:5.0,0.0,2.0,300.0)
 *   START                    – begin PID control
 *   DIST:xxx.x               – webcam distance in mm
 *   STOP                     – stop motors, return to IDLE
 */

#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================
// HARDWARE PIN CONFIGURATION  (TA sets once)
// ==========================================
// L298N Motor A (left)
#define ENA 25
#define IN1 26
#define IN2 27
// L298N Motor B (right)
#define ENB 14
#define IN3 12
#define IN4 13
// HC-SR04 Ultrasonic Sensor
#define TRIG 5
#define ECHO 18

// ==========================================
// WIFI CONFIGURATION  (TA sets once per car)
// ==========================================
const char* WIFI_SSID = "RobotCar_XX";  // Change XX to car number
const char* WIFI_PASS = "1104lab7";
const int   UDP_PORT  = 4210;

// ==========================================
// MOTOR / CONTROL LIMITS  (TA-tunable)
// ==========================================
int   CONTROL_HZ      = 20;
int   PWM_MIN         = 40;
int   PWM_MAX         = 200;
float INTEGRAL_LIMIT  = 5000.0;

// ==========================================
// SENSOR FUSION PARAMETERS
// ==========================================
float DIST_MIN_MM         = 20.0;
float DIST_MAX_MM         = 2000.0;
float OUTLIER_THRESHOLD   = 100.0;
unsigned long WEBCAM_TIMEOUT_MS = 500;

// Timeout: if no UDP packet for this long while RUNNING, auto-stop
unsigned long STREAM_TIMEOUT_MS = 3000;

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum CarState { IDLE, RUNNING };
CarState state = IDLE;

// ---------------------------------------------------------------------------
// PID parameters (received from laptop)
// ---------------------------------------------------------------------------
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
float SETPOINT_MM = 300.0;
bool  pid_configured = false;

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
WiFiUDP udp;

volatile float  webcam_dist_mm = -1.0;
unsigned long   last_webcam_ms = 0;
unsigned long   last_any_packet_ms = 0;

float last_fused_mm = -1.0;

float e_prev     = 0.0;
float e_integral = 0.0;

unsigned long loop_interval_ms;
unsigned long last_control_ms = 0;

// ESP32 LEDC PWM channels
#define PWM_CHAN_A 0
#define PWM_CHAN_B 1
#define PWM_FREQ   1000
#define PWM_RES    8

// =====================================================================
// HC-SR04
// =====================================================================
float readUltrasonic() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long duration = pulseIn(ECHO, HIGH, 25000);
    if (duration == 0) return -1.0;

    float dist = duration * 0.343 / 2.0;
    if (dist < DIST_MIN_MM || dist > DIST_MAX_MM) return -1.0;
    return dist;
}

// =====================================================================
// UDP message handler
// =====================================================================
void receiveUDP() {
    int packetSize = udp.parsePacket();
    if (packetSize == 0) return;

    char buf[128];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len <= 0) return;
    buf[len] = '\0';

    last_any_packet_ms = millis();

    // --- PID:Kp,Ki,Kd,setpoint ---
    if (strncmp(buf, "PID:", 4) == 0) {
        float p, i, d, sp;
        if (sscanf(buf + 4, "%f,%f,%f,%f", &p, &i, &d, &sp) == 4) {
            Kp = p;  Ki = i;  Kd = d;  SETPOINT_MM = sp;
            pid_configured = true;

            // Reset PID state for a fresh run
            e_prev     = 0.0;
            e_integral = 0.0;
            last_fused_mm = -1.0;
            webcam_dist_mm = -1.0;

            Serial.print("CONFIG  Kp="); Serial.print(Kp, 3);
            Serial.print(" Ki=");        Serial.print(Ki, 3);
            Serial.print(" Kd=");        Serial.print(Kd, 3);
            Serial.print(" SP=");        Serial.println(SETPOINT_MM, 1);
        }
        return;
    }

    // --- START ---
    if (strncmp(buf, "START", 5) == 0) {
        if (!pid_configured) {
            Serial.println("WARNING: START received but PID not configured. Ignoring.");
            return;
        }
        if (state == IDLE) {
            state = RUNNING;
            last_control_ms = millis();
            Serial.println("STATE   IDLE -> RUNNING");
            Serial.println("WEBCAM\tSONAR\tFUSED\tERR\tU\tPWM");
        }
        return;
    }

    // --- STOP ---
    if (strncmp(buf, "STOP", 4) == 0) {
        if (state == RUNNING) {
            state = IDLE;
            stopMotors();
            Serial.println("STATE   RUNNING -> IDLE  (STOP received)");
        }
        return;
    }

    // --- DIST:xxx.x ---
    if (strncmp(buf, "DIST:", 5) == 0) {
        float val = atof(buf + 5);
        if (val >= DIST_MIN_MM && val <= DIST_MAX_MM) {
            webcam_dist_mm = val;
            last_webcam_ms = millis();
        }
        return;
    }
}

// =====================================================================
// Sensor fusion
// =====================================================================
float fusedDistance(float sonar_mm) {
    unsigned long now = millis();

    bool webcam_ok = (webcam_dist_mm > 0)
                  && ((now - last_webcam_ms) < WEBCAM_TIMEOUT_MS)
                  && (webcam_dist_mm >= DIST_MIN_MM)
                  && (webcam_dist_mm <= DIST_MAX_MM);

    bool sonar_ok  = (sonar_mm > 0)
                  && (sonar_mm >= DIST_MIN_MM)
                  && (sonar_mm <= DIST_MAX_MM);

    float result;

    if (webcam_ok && sonar_ok) {
        if (fabs(webcam_dist_mm - sonar_mm) < OUTLIER_THRESHOLD) {
            result = (webcam_dist_mm + sonar_mm) / 2.0;
        } else {
            if (last_fused_mm > 0) {
                float dw = fabs(webcam_dist_mm - last_fused_mm);
                float ds = fabs(sonar_mm - last_fused_mm);
                result = (dw < ds) ? webcam_dist_mm : sonar_mm;
            } else {
                result = (webcam_dist_mm + sonar_mm) / 2.0;
            }
        }
    } else if (webcam_ok) {
        result = webcam_dist_mm;
    } else if (sonar_ok) {
        result = sonar_mm;
    } else {
        result = (last_fused_mm > 0) ? last_fused_mm : SETPOINT_MM;
    }

    last_fused_mm = result;
    return result;
}

// =====================================================================
// Motor control
// =====================================================================
void setMotors(float u) {
    bool forward = (u < 0);
    int pwm = (int)constrain(fabs(u), 0, PWM_MAX);
    if (pwm < PWM_MIN) pwm = 0;

    if (forward) {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    }

    ledcWrite(PWM_CHAN_A, pwm);
    ledcWrite(PWM_CHAN_B, pwm);
}

void stopMotors() {
    ledcWrite(PWM_CHAN_A, 0);
    ledcWrite(PWM_CHAN_B, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// =====================================================================
// Setup
// =====================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== MIT 1.104 Robot Car (laptop-driven) ===");

    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);

    ledcSetup(PWM_CHAN_A, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CHAN_B, PWM_FREQ, PWM_RES);
    ledcAttachPin(ENA, PWM_CHAN_A);
    ledcAttachPin(ENB, PWM_CHAN_B);
    stopMotors();

    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.print("SoftAP: "); Serial.println(WIFI_SSID);
    Serial.print("IP:     "); Serial.println(WiFi.softAPIP());

    udp.begin(UDP_PORT);
    Serial.print("UDP port "); Serial.println(UDP_PORT);

    loop_interval_ms = 1000 / CONTROL_HZ;

    Serial.println("STATE   IDLE  (waiting for PID config + START)\n");
}

// =====================================================================
// Main loop
// =====================================================================
void loop() {
    receiveUDP();

    // ---- IDLE: just keep listening, motors stay off ----
    if (state == IDLE) return;

    // ---- RUNNING: auto-stop if no packets for STREAM_TIMEOUT_MS ----
    if (millis() - last_any_packet_ms > STREAM_TIMEOUT_MS) {
        state = IDLE;
        stopMotors();
        Serial.println("STATE   RUNNING -> IDLE  (stream timeout)");
        return;
    }

    // ---- RUNNING: control loop at fixed rate ----
    unsigned long now = millis();
    if (now - last_control_ms < loop_interval_ms) return;
    float dt = (now - last_control_ms) / 1000.0;
    last_control_ms = now;

    float sonar_mm = readUltrasonic();
    float fused    = fusedDistance(sonar_mm);

    float e  = SETPOINT_MM - fused;
    float de = (dt > 0) ? (e - e_prev) / dt : 0.0;

    e_integral += e * dt;
    e_integral  = constrain(e_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float u = Kp * e + Ki * e_integral + Kd * de;
    e_prev  = e;

    setMotors(u);

    int pwm_out = (int)constrain(fabs(u), 0, PWM_MAX);
    if (pwm_out < PWM_MIN) pwm_out = 0;

    Serial.print(webcam_dist_mm, 1); Serial.print("\t");
    Serial.print(sonar_mm, 1);       Serial.print("\t");
    Serial.print(fused, 1);          Serial.print("\t");
    Serial.print(e, 1);              Serial.print("\t");
    Serial.print(u, 1);              Serial.print("\t");
    Serial.println(pwm_out);
}

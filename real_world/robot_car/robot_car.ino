/*
 * MIT 1.104 Lab 7 – Robot Car PID Controller (ESP32)
 *
 * Firmware uploaded ONCE by the TA.  The car boots into IDLE mode
 * (motors off, Bluetooth active).  A laptop Python script connects
 * via Bluetooth SPP, sends PID gains, a START command, and a
 * continuous distance stream.  The car runs the PID loop until it
 * receives STOP (or the stream times out).
 *
 * Message protocol  (laptop → car, newline-terminated):
 *   PID:Kp,Ki,Kd,setpoint   – configure gains  (e.g. PID:5.0,0.0,2.0,300.0)
 *   START                    – begin PID control
 *   DIST:xxx.x               – webcam distance in mm
 *   STOP                     – stop motors, return to IDLE
 */

#include "BluetoothSerial.h"

// ==========================================
// HARDWARE PIN CONFIGURATION  (TA sets once)
// ==========================================
// L298N Motor A (left)
#define ENA 27
#define IN1 26
#define IN2 25
// L298N Motor B (right)
#define ENB 14
#define IN3 17
#define IN4 16
// HC-SR04 Ultrasonic Sensor
#define TRIG 5
#define ECHO 18
// Status LED – sonar signal good (on-board or external; change pin if needed)
#define LED_SONAR_PIN 2
// Status LED – webcam/camera signal fresh
#define LED_CAM_PIN   4

// ==========================================
// BLUETOOTH CONFIGURATION  (TA sets once per car)
// ==========================================
const char* BT_NAME = "RobotCar_XX";  // Change XX to car number

// ==========================================
// MOTOR / CONTROL LIMITS  (TA-tunable)
// ==========================================
int   CONTROL_HZ      = 20;
int   PWM_MIN         = 40;
int   PWM_MAX         = 200;
float INTEGRAL_LIMIT  = 5000.0;
int   DEBUG_PRINT_HZ  = 5;

// ==========================================
// SENSOR FUSION PARAMETERS
// ==========================================
float DIST_MIN_MM         = 20.0;
float DIST_MAX_MM         = 2000.0;
float OUTLIER_THRESHOLD   = 100.0;
unsigned long WEBCAM_TIMEOUT_MS = 150;

// Timeout: if no message for this long while RUNNING, auto-stop
unsigned long STREAM_TIMEOUT_MS = 3000;

// Fixed run duration: car automatically returns to IDLE after this many ms.
// Set to 0 to disable (run until STOP or stream timeout).
unsigned long RUN_DURATION_MS = 10000;

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
float SETPOINT_MM = 100.0;
bool  pid_configured = false;

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
BluetoothSerial SerialBT;

volatile float  webcam_dist_mm = -1.0;
unsigned long   last_webcam_ms = 0;
unsigned long   last_any_packet_ms = 0;

float last_fused_mm = -1.0;

float e_prev     = 0.0;
float e_integral = 0.0;

unsigned long loop_interval_ms;
unsigned long last_control_ms = 0;
unsigned long run_start_ms    = 0;
unsigned long last_debug_ms   = 0;

// ESP32 LEDC PWM (Core 3.x API: ledcAttach + ledcWrite by pin)
#define PWM_FREQ   1000
#define PWM_RES    8

// =====================================================================
// HC-SR04 (bounded pulseIn)
//
// With max range < 50 cm for this lab setup, pulseIn with a short timeout
// keeps the read bounded and avoids interrupt/noise complexity.
// =====================================================================
#define SONAR_TIMEOUT_US 4500

float readUltrasonicResult() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    unsigned long dur = pulseIn(ECHO, HIGH, SONAR_TIMEOUT_US);
    if (dur == 0) return -1.0;

    float dist = dur * 0.343f / 2.0f;
    if (dist < DIST_MIN_MM || dist > DIST_MAX_MM) return -1.0;
    return dist;
}

// =====================================================================
// Message parser (shared by receiveBT)
// =====================================================================
void processMessage(const char* buf) {
    last_any_packet_ms = millis();

    // --- PID:Kp,Ki,Kd,setpoint ---
    if (strncmp(buf, "PID:", 4) == 0) {
        float p, i, d, sp;
        if (sscanf(buf + 4, "%f,%f,%f,%f", &p, &i, &d, &sp) == 4) {
            Kp = p;  Ki = i;  Kd = d;  SETPOINT_MM = sp;
            pid_configured = true;

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
            run_start_ms    = millis();
            Serial.println("STATE   IDLE -> RUNNING");
            if (RUN_DURATION_MS > 0)
                Serial.print("  run duration: "), Serial.print(RUN_DURATION_MS), Serial.println(" ms");
            Serial.println("WEBCAM\tSONAR\tFUSED\tERR\tU\tPWM");
        }
        return;
    }

    // --- STOP ---
    if (strncmp(buf, "STOP", 4) == 0) {
        if (state == RUNNING) {
            state = IDLE;
            stopMotors();
            digitalWrite(LED_SONAR_PIN, LOW); digitalWrite(LED_CAM_PIN, LOW);
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
// Bluetooth serial receiver (line-buffered)
// =====================================================================
static char btBuf[128];
static int  btBufIdx = 0;
const int   BT_BYTES_PER_LOOP = 64;

void receiveBT() {
    int processed = 0;
    while (SerialBT.available() && processed < BT_BYTES_PER_LOOP) {
        char c = SerialBT.read();
        processed++;
        if (c == '\n' || c == '\r') {
            if (btBufIdx > 0) {
                btBuf[btBufIdx] = '\0';
                processMessage(btBuf);
                btBufIdx = 0;
            }
        } else if (btBufIdx < (int)sizeof(btBuf) - 1) {
            btBuf[btBufIdx++] = c;
        }
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
// Status LEDs
//   LED_SONAR_PIN  ON  – sonar returned a valid reading this tick
//                  OFF – no echo, out of range, or echo ISR not yet fired
//   LED_CAM_PIN    ON  – a fresh DIST packet arrived within WEBCAM_TIMEOUT_MS
//                  OFF – webcam data stale or laptop not streaming
// =====================================================================
void updateLEDs(float sonar_mm) {
    digitalWrite(LED_SONAR_PIN, (sonar_mm > 0) ? HIGH : LOW);
    bool cam_ok = (webcam_dist_mm > 0)
               && ((millis() - last_webcam_ms) < WEBCAM_TIMEOUT_MS);
    digitalWrite(LED_CAM_PIN, cam_ok ? HIGH : LOW);
}

// =====================================================================
// Motor control
// =====================================================================
void setMotors(float u) {
    bool forward = (u < 0);
    int pwm = (int)constrain(fabs(u), 0, PWM_MAX);
    if (pwm < PWM_MIN) pwm = 0;

    if (forward) {
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    }

    if (forward) {
        ledcWrite(ENA, pwm);
        ledcWrite(ENB, pwm);
    } else {
        ledcWrite(ENA, pwm);
        ledcWrite(ENB, 0.85*pwm);
    }
}

void stopMotors() {
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// =====================================================================
// Setup
// =====================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== MIT 1.104 Robot Car (Bluetooth) ===");

    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
    pinMode(LED_SONAR_PIN, OUTPUT); digitalWrite(LED_SONAR_PIN, LOW);
    pinMode(LED_CAM_PIN,   OUTPUT); digitalWrite(LED_CAM_PIN,   LOW);

    ledcAttach(ENA, PWM_FREQ, PWM_RES);
    ledcAttach(ENB, PWM_FREQ, PWM_RES);
    stopMotors();

    SerialBT.begin(BT_NAME);
    Serial.print("Bluetooth: "); Serial.println(BT_NAME);

    loop_interval_ms = 1000 / CONTROL_HZ;

    Serial.println("STATE   IDLE  (waiting for BT connection + PID config + START)\n");
}

// =====================================================================
// Main loop
// =====================================================================
void loop() {
    receiveBT();

    // ---- IDLE: just keep listening, motors stay off ----
    if (state == IDLE) return;

    // ---- RUNNING: auto-stop if no messages for STREAM_TIMEOUT_MS ----
    if (millis() - last_any_packet_ms > STREAM_TIMEOUT_MS) {
        state = IDLE;
        stopMotors();
        digitalWrite(LED_SONAR_PIN, LOW); digitalWrite(LED_CAM_PIN, LOW);
        Serial.println("STATE   RUNNING -> IDLE  (stream timeout)");
        return;
    }

    // ---- RUNNING: auto-stop after fixed run duration ----
    if (RUN_DURATION_MS > 0 && millis() - run_start_ms >= RUN_DURATION_MS) {
        state = IDLE;
        stopMotors();
        digitalWrite(LED_SONAR_PIN, LOW); digitalWrite(LED_CAM_PIN, LOW);
        Serial.println("STATE   RUNNING -> IDLE  (run duration elapsed)");
        return;
    }

    // ---- RUNNING: control loop at fixed rate ----
    unsigned long now = millis();
    if (now - last_control_ms < loop_interval_ms) return;
    float dt = (now - last_control_ms) / 1000.0;
    last_control_ms = now;

    float sonar_mm = readUltrasonicResult();
    updateLEDs(sonar_mm);
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

    unsigned long debug_interval_ms = 1000UL / (unsigned long)max(DEBUG_PRINT_HZ, 1);
    if (millis() - last_debug_ms >= debug_interval_ms) {
        last_debug_ms = millis();
        Serial.print(webcam_dist_mm, 1); Serial.print("\t");
        Serial.print(sonar_mm, 1);       Serial.print("\t");
        Serial.print(fused, 1);          Serial.print("\t");
        Serial.print(e, 1);              Serial.print("\t");
        Serial.print(u, 1);              Serial.print("\t");
        Serial.println(pwm_out);
    }
}

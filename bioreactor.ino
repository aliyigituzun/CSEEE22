/************************************************************
   Integrated Bioreactor Controller
   - Motor speed (RPM) with PID
   - Heating via thermistor
   - pH control via acid/base pumps
   - WiFi (eduroam placeholder) + MQTT over TLS (HiveMQ Cloud)
************************************************************/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "esp_wifi.h"
#include <PID_v1.h>
#include <math.h>

// ===========================================================
// ------------------------ CONFIG ---------------------------
// ===========================================================

// --- eduroam enterprise WiFi config ---
#define USE_EDUROAM 1
#define WIFI_USER "zcabayu@ucl.ac.uk"
#define WIFI_SSID "eduroam"
#define WIFI_PASS "xxx"   // TODO: real eduroam password

const char* ssid     = WIFI_SSID;
const char* eap_id   = WIFI_USER;   // identity
const char* eap_user = WIFI_USER;   // username
const char* eap_pass = WIFI_PASS;   // password
const char* pass     = WIFI_PASS;

// --- MQTT config (HiveMQ Cloud over TLS) ---
static const char* mqtt_host = "b6dbb6381f0c4bbf8360ef22013ff085.s1.eu.hivemq.cloud";
static const uint16_t mqtt_port = 8883;
static const char* mqtt_user = "CSEEE22";
static const char* mqtt_pass = "Team22thebest";
static const char* client_id = "reactor-heater-esp32-001";

// Topic names (state)
const char* TOPIC_STATE_PH    = "reactor/state/ph";
const char* TOPIC_STATE_HEAT  = "reactor/state/temp";
const char* TOPIC_STATE_RPM   = "reactor/state/rpm";  // added

// Topic names (setpoints)
const char* TOPIC_SETPOINT_PH   = "reactor/setpoint/ph";
const char* TOPIC_SETPOINT_TEMP = "reactor/setpoint/temp";
const char* TOPIC_SETPOINT_RPM  = "reactor/setpoint/rpm";

// ===========================================================
// ----------------- WPA2-Enterprise includes ----------------
// ===========================================================
#if __has_include("esp_eap_client.h")
  #include "esp_eap_client.h"
  #define USE_NEW_EAP_API 1
#elif __has_include("esp_wpa2.h")
  #include "esp_wpa2.h"
  #define USE_NEW_EAP_API 0
#else
  #error "No enterprise WiFi API found. Install/upgrade ESP32 core."
#endif

// ===========================================================
// --------------------- PIN ASSIGNMENT ----------------------
// ===========================================================

// ---- Motor ----
const int PIN_MOTOR_ENCODER = 4;      // was ENCODER_PIN
const int PIN_MOTOR_PWM     = 10;     // was MOTOR_PWM_PIN

// ---- Heating ----
const int PIN_THERMISTOR    = A0;     // CHANGED from A0 to avoid conflict with pH sensor
const int PIN_HEATER        = 2;      // CHANGED from D2

// ---- pH ----
const int PIN_PH_SENSOR     = A1;     // keep pH on A0
const int PIN_ACID_PUMP     = 5;      // CHANGED from D3
const int PIN_BASE_PUMP     = 6;      // CHANGED from D4

// ===========================================================
// -------------------- MOTOR + PID --------------------------
// ===========================================================

const int PULSES_PER_REVOLUTION = 70;

// PID Variables and Constants
double rpmSetpoint, rpmInput, rpmOutput;
double Kp = 2.5, Ki = 0.5, Kd = 0.1;
PID motorPID(&rpmInput, &rpmOutput, &rpmSetpoint, Kp, Ki, Kd, DIRECT);

// Control timing
const unsigned long controlInterval = 100; // ms
unsigned long lastMotorControlTime = 0;

// Encoder state
volatile long encoderTicks = 0;

void IRAM_ATTR encoderISR() {
  encoderTicks++;
}

// ===========================================================
// ---------------------- HEATING ----------------------------
// ===========================================================

// Thermistor constants
float R  = 15000.0;
float R0 = 10000.0;
float beta = 4220.0;
float baseKelvin = 298.15; // 25°C in Kelvin
float VCC = 5.0;           // PSU 5V

// Target temp (setpoint from MQTT)
float targetTempC = 35.0;  // default target

// Measured temp
float currentTempC = 0.0;
bool heaterOn = false;

const unsigned long HEAT_SAMPLE_INTERVAL = 2000; // ms
unsigned long lastHeatSampleTime = 0;

// ===========================================================
// ------------------------ pH -------------------------------
// ===========================================================

// pH sensor calibration
const float PH_SLOPE  = 3.947;   // from your line equation
float phOffset        = 2.547;   // adjust after calibration

// pH setpoint and band
float phSetpoint   = 7.0;   // default centre value
float phTolerance  = 0.2;   // ±band around setpoint

float currentPH = 7.0;

// pH control timing/state (non-blocking)
enum PhState {
  PH_IDLE,
  PH_ADDING_ACID,
  PH_ADDING_BASE,
  PH_MIXING
};

PhState phState = PH_IDLE;
unsigned long phStateStartMillis = 0;

const unsigned long PH_PUMP_ON_TIME   = 5000;   // 5 s
const unsigned long PH_MIX_TIME       = 10000;  // 10 s
const unsigned long PH_SAMPLE_INTERVAL = 1000;  // 1 s
unsigned long lastPhSampleTime = 0;

// ===========================================================
// ------------------- WiFi / MQTT ---------------------------
// ===========================================================

WiFiClientSecure tlsClient;
PubSubClient mqtt(tlsClient);

// State publish timing
const unsigned long PUBLISH_INTERVAL = 1000; // ms
unsigned long lastPublishTime = 0;

// ===========================================================
// ----------------- HELPER FUNCTIONS ------------------------
// ===========================================================

// ---- WiFi (you can drop your working eduroam code into this) ----
void wifi_connect() {
  Serial.println();
  Serial.println("Connecting to eduroam...");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

#if USE_NEW_EAP_API
  // New enterprise API
  esp_eap_client_set_identity((const uint8_t*)WIFI_USER, strlen(WIFI_USER));
  esp_eap_client_set_username((const uint8_t*)WIFI_USER, strlen(WIFI_USER));
  esp_eap_client_set_password((const uint8_t*)WIFI_PASS, strlen(WIFI_PASS));
  esp_eap_client_enable();
  WiFi.begin(WIFI_SSID);
#else
  // Old WPA2 enterprise API
  esp_wifi_sta_wpa2_ent_set_identity((const uint8_t*)WIFI_USER, strlen(WIFI_USER));
  esp_wifi_sta_wpa2_ent_set_username((const uint8_t*)WIFI_USER, strlen(WIFI_USER));
  esp_wifi_sta_wpa2_ent_set_password((const uint8_t*)WIFI_PASS, strlen(WIFI_PASS));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(WIFI_SSID);
#endif

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());
}


// ---- MQTT callback: setpoints from broker ----
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();
  float value = msg.toFloat();

  Serial.print("MQTT message on [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);

  if (strcmp(topic, TOPIC_SETPOINT_RPM) == 0) {
    if (value >= 0) {
      rpmSetpoint = value;
      Serial.print("New RPM setpoint: ");
      Serial.println(rpmSetpoint);
    }
  } else if (strcmp(topic, TOPIC_SETPOINT_TEMP) == 0) {
    targetTempC = value;
    Serial.print("New Temperature setpoint: ");
    Serial.println(targetTempC);
  } else if (strcmp(topic, TOPIC_SETPOINT_PH) == 0) {
    phSetpoint = value;
    Serial.print("New pH setpoint: ");
    Serial.println(phSetpoint);
  }
}

// ---- MQTT reconnect ----
void reconnectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");

    // TLS: not verifying cert (for lab use)
    tlsClient.setInsecure();

    if (mqtt.connect(client_id, mqtt_user, mqtt_pass)) {
      Serial.println("connected");

      // Subscribe to setpoint topics
      mqtt.subscribe(TOPIC_SETPOINT_RPM);
      mqtt.subscribe(TOPIC_SETPOINT_TEMP);
      mqtt.subscribe(TOPIC_SETPOINT_PH);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// ---- Motor: apply PID output ----
void applyMotorOutput(int pwm) {
  if (rpmSetpoint == 0 && pwm < 10) {
    pwm = 0;
  }
  pwm = constrain(pwm, 0, 255);
  analogWrite(PIN_MOTOR_PWM, pwm);
}

// ---- Motor: calculate RPM from encoder ----
void updateMotor(unsigned long now) {
  if (now - lastMotorControlTime < controlInterval) return;
  lastMotorControlTime = now;

  noInterrupts();
  long ticks = encoderTicks;
  encoderTicks = 0;
  interrupts();

  double timeInSeconds = (double)controlInterval / 1000.0;
  rpmInput = (double)ticks / (double)PULSES_PER_REVOLUTION; // revs
  rpmInput = rpmInput / timeInSeconds;                      // RPS
  rpmInput = rpmInput * 60.0;                               // RPM

  motorPID.Compute();
  applyMotorOutput((int)rpmOutput);

  Serial.print("[MOTOR] Target RPM: ");
  Serial.print(rpmSetpoint);
  Serial.print(" | Current RPM: ");
  Serial.print(rpmInput, 1);
  Serial.print(" | PWM: ");
  Serial.println((int)rpmOutput);
}

// ---- Heating: read thermistor and control heater ----
void updateHeating(unsigned long now) {
  if (now - lastHeatSampleTime < HEAT_SAMPLE_INTERVAL) return;
  lastHeatSampleTime = now;

  int raw = analogRead(PIN_THERMISTOR); // assume 12-bit ADC (0-4095)
  float thermoVoltage = (VCC * raw) / 4095.0;

  if (thermoVoltage <= 0.0 || thermoVoltage >= VCC) {
    return;
  }

  float thermistorResistance =
      (R * thermoVoltage) / (VCC - thermoVoltage);

  float x = log(thermistorResistance / R0);
  currentTempC = 1.0 / ((1.0 / baseKelvin) + (x / beta)) - 273.15;

  if (currentTempC < targetTempC - 0.5) {
    heaterOn = true;
  } else if (currentTempC > targetTempC + 0.5) {
    heaterOn = false;
  }
  digitalWrite(PIN_HEATER, heaterOn ? HIGH : LOW);

  Serial.print("[HEAT] Target: ");
  Serial.print(targetTempC);
  Serial.print(" °C | Temp: ");
  Serial.print(currentTempC);
  Serial.print(" °C | Heater: ");
  Serial.println(heaterOn ? "ON" : "OFF");
}

// ---- pH: read sensor ----
void samplePH(unsigned long now) {
  if (now - lastPhSampleTime < PH_SAMPLE_INTERVAL) return;
  lastPhSampleTime = now;

  int sensorValue = analogRead(PIN_PH_SENSOR);
  float phVoltage = sensorValue * (3.3 / 4095.0);
  currentPH = PH_SLOPE * phVoltage + phOffset;

  Serial.print("[pH] pH: ");
  Serial.print(currentPH);
  Serial.print(" (V=");
  Serial.print(phVoltage, 3);
  Serial.println(")");
}

// ---- pH: non-blocking control state machine ----
void updatePH(unsigned long now) {
  float phLow  = phSetpoint - phTolerance;
  float phHigh = phSetpoint + phTolerance;

  switch (phState) {
    case PH_IDLE:
      digitalWrite(PIN_ACID_PUMP, LOW);
      digitalWrite(PIN_BASE_PUMP, LOW);
      samplePH(now);

      if (currentPH > phHigh) {
        Serial.println("[pH] HIGH → Adding ACID");
        digitalWrite(PIN_ACID_PUMP, HIGH);
        digitalWrite(PIN_BASE_PUMP, LOW);
        phState = PH_ADDING_ACID;
        phStateStartMillis = now;
      } else if (currentPH < phLow) {
        Serial.println("[pH] LOW → Adding BASE");
        digitalWrite(PIN_BASE_PUMP, HIGH);
        digitalWrite(PIN_ACID_PUMP, LOW);
        phState = PH_ADDING_BASE;
        phStateStartMillis = now;
      }
      break;

    case PH_ADDING_ACID:
      if (now - phStateStartMillis >= PH_PUMP_ON_TIME) {
        digitalWrite(PIN_ACID_PUMP, LOW);
        phState = PH_MIXING;
        phStateStartMillis = now;
        Serial.println("[pH] Stop ACID, mixing...");
      }
      break;

    case PH_ADDING_BASE:
      if (now - phStateStartMillis >= PH_PUMP_ON_TIME) {
        digitalWrite(PIN_BASE_PUMP, LOW);
        phState = PH_MIXING;
        phStateStartMillis = now;
        Serial.println("[pH] Stop BASE, mixing...");
      }
      break;

    case PH_MIXING:
      if (now - phStateStartMillis >= PH_MIX_TIME) {
        phState = PH_IDLE;
        Serial.println("[pH] Mixing done, back to IDLE.");
      }
      break;
  }
}

// ---- MQTT: publish state periodically ----
void publishState(unsigned long now) {
  if (now - lastPublishTime < PUBLISH_INTERVAL) return;
  lastPublishTime = now;

  char buf[32];

  // RPM
  dtostrf(rpmInput, 0, 1, buf);
  mqtt.publish(TOPIC_STATE_RPM, buf);

  // pH
  dtostrf(currentPH, 0, 2, buf);
  mqtt.publish(TOPIC_STATE_PH, buf);

  // Temperature
  dtostrf(currentTempC, 0, 2, buf);
  mqtt.publish(TOPIC_STATE_HEAT, buf);
}

// ===========================================================
// ------------------------ SETUP ----------------------------
// ===========================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Integrated Bioreactor Controller starting...");

  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCODER), encoderISR, RISING);

  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);

  pinMode(PIN_PH_SENSOR, INPUT);
  pinMode(PIN_ACID_PUMP, OUTPUT);
  pinMode(PIN_BASE_PUMP, OUTPUT);
  digitalWrite(PIN_ACID_PUMP, LOW);
  digitalWrite(PIN_BASE_PUMP, LOW);

  analogReadResolution(12);

  rpmSetpoint = 0.0;
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(0, 255);
  motorPID.SetSampleTime(controlInterval);

  wifi_connect();

  tlsClient.setInsecure();                  // no CA cert (lab use)
  mqtt.setServer(mqtt_host, mqtt_port);
  mqtt.setCallback(mqttCallback);
}

// ===========================================================
// ------------------------- LOOP ----------------------------
// ===========================================================

void loop() {
  if (!mqtt.connected()) {
    reconnectMQTT();
  }
  mqtt.loop();

  unsigned long now = millis();

  updateMotor(now);
  updateHeating(now);
  updatePH(now);
  publishState(now);
}

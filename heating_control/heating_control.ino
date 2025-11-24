// =============================
// WiFi + MQTT + Heater Control
// Arduino Nano ESP32
// =============================

#define USE_EDUROAM 1
#define WIFI_USER "zcabayu@ucl.ac.uk"
#define WIFI_SSID "eduroam"
#define WIFI_PASS "xxx"  // TODO: put your real eduroam password here

const char* pass = WIFI_PASS;

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "esp_wifi.h"
#include <math.h>

#if __has_include("esp_eap_client.h")
  #include "esp_eap_client.h"
  #define USE_NEW_EAP_API 1
#elif __has_include("esp_wpa2.h")
  // older cores expose WPA2 Enterprise via esp_wpa2.h
  #include "esp_wpa2.h"
  #define USE_NEW_EAP_API 0
#else
  #error "No enterprise WiFi API found. Please install/upgrade Arduino-ESP32 core."
#endif

// ---------- WiFi / EAP config ----------

static const char* ssid      = WIFI_SSID;
static const char* eap_id    = WIFI_USER; // outer identity
static const char* eap_user  = WIFI_USER; // inner identity/username
static const char* eap_pass  = WIFI_PASS; // password

// ---------- MQTT config ----------

static const char* mqtt_host = "b6dbb6381f0c4bbf8360ef22013ff085.s1.eu.hivemq.cloud";
static const uint16_t mqtt_port = 8883;             // TLS
static const char* mqtt_user = "CSEEE22";
static const char* mqtt_pass = "Team22thebest";
static const char* client_id = "reactor-esp32-001";  // must be unique per device

WiFiClientSecure tlsClient;
PubSubClient mqtt(tlsClient);

//
// reactor/state
// reactor/state/ph
// reactor/state/rpm
// reactor/state/temp
// reactor/setpoint/ph
// reactor/setpoint/rpm
// reactor/setpoint/temp
// reactor/alarm
// reactor/cmd
// reactor/cmd/heater
// reactor/cmd/pump
// reactor/cmd/motor
//---------------------------------------

static const char* T_STATE         = "reactor/state";
static const char* T_STATE_TEMP    = "reactor/state/temp";

static const char* T_SETPOINT_TEMP = "reactor/setpoint/temp";
// (ph and rpm present but not used by heater code yet)

static const char* T_CMD_ROOT      = "reactor/cmd";
static const char* T_CMD_HEATER    = "reactor/cmd/heater";
// (pump/motor stubs can be added later)

unsigned long lastTelemetryMs = 0;
const unsigned long telemetryPeriodMs = 2000;  // 2s

// =============================
// Heater / Thermistor config
// =============================

const byte thermistorPin = A0;
const byte heaterpin     = 2;

// Nano ESP32 is 3.3 V; assume divider on 3.3 V
const float Vcc   = 3.3;             // supply to the thermistor divider
const float R     = 15000.0f;        // 15 kΩ fixed resistor
const float beta  = 3700.0f;         // thermistor beta
const float Ro    = 10000.0f;        // resistance at To
const float To    = 25.0f;           // reference temperature in °C

// ESP32 ADC: 12-bit (0..4095), 0..3.3V
const float Kadc  = 3.3f / 4095.0f;  // convert raw ADC into volts

// Control setpoints
float targetTemp  = 35.0f;  // °C, will be updated from MQTT setpoint
float deltaT      = 0.5f;   // hysteresis band

// Runtime variables
float Vadc = 0.0f;
float temp = 0.0f;
float Rth  = 0.0f;
long currentTime = 0, previousTime = 0, T1 = 0, T2 = 0;
// T1 controls 100ms updates (control)
// T2 controls 1-second serial prints

int heater        = 0;  // PWM value (0..255)
int previousHeater = 0;

// Optional: mode for heater command override
enum HeaterMode { HEATER_AUTO, HEATER_FORCE_ON, HEATER_FORCE_OFF };
HeaterMode heaterMode = HEATER_AUTO;

// =============================
// WiFi helpers
// =============================

void wifi_connect(float timeout_sec = 15.0f) {
  Serial.println();
  Serial.println("[WiFi] Connecting to eduroam...");

  WiFi.disconnect(true /*wifioff*/);
  WiFi.mode(WIFI_STA);

#if USE_NEW_EAP_API
  esp_eap_client_set_identity((const uint8_t*)eap_id,   strlen(eap_id));
  esp_eap_client_set_username((const uint8_t*)eap_user, strlen(eap_user));
  esp_eap_client_set_password((const uint8_t*)eap_pass, strlen(eap_pass));
  esp_wifi_sta_enterprise_enable();
  WiFi.begin(ssid);
#else
  esp_wifi_sta_wpa2_ent_set_identity((const uint8_t*)eap_id,   strlen(eap_id));
  esp_wifi_sta_wpa2_ent_set_username((const uint8_t*)eap_user, strlen(eap_user));
  esp_wifi_sta_wpa2_ent_set_password((const uint8_t*)eap_pass, strlen(eap_pass));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(ssid);
#endif

  unsigned long deadline = millis() + (unsigned long)(timeout_sec * 1000.0f);
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected, IP = ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Failed to connect within timeout.");
  }
}

void ensureWifi() {
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost connection, retrying...");
    wifi_connect(20);
    delay(250);
  }
}

// =============================
// MQTT helpers
// =============================

void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  // Copy payload into a null-terminated buffer for easy parsing
  char buf[128];
  unsigned int n = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
  memcpy(buf, payload, n);
  buf[n] = '\0';

  Serial.print("[MQTT] ");
  Serial.print(topic);
  Serial.print(" -> ");
  Serial.println(buf);

  // --- Setpoint change detection: reactor/setpoint/temp ---
  if (strcmp(topic, T_SETPOINT_TEMP) == 0) {
    float newTarget = atof(buf);  // payload is ASCII float, e.g. "37.5"

    // Only update if changed (within small epsilon)
    if (fabsf(newTarget - targetTemp) > 0.01f) {
      Serial.print("[SETPOINT] temp changed ");
      Serial.print(targetTemp);
      Serial.print(" -> ");
      Serial.println(newTarget);
      targetTemp = newTarget;
    }
    return;
  }

  // --- Command handling: reactor/cmd/heater ---
  if (strcmp(topic, T_CMD_HEATER) == 0) {
    // Expected payload: "auto", "on", "off"
    if (strcasecmp(buf, "auto") == 0) {
      heaterMode = HEATER_AUTO;
      Serial.println("[CMD] Heater mode: AUTO");
    } else if (strcasecmp(buf, "on") == 0) {
      heaterMode = HEATER_FORCE_ON;
      Serial.println("[CMD] Heater mode: FORCE ON");
    } else if (strcasecmp(buf, "off") == 0) {
      heaterMode = HEATER_FORCE_OFF;
      Serial.println("[CMD] Heater mode: FORCE OFF");
    } else {
      Serial.println("[CMD] Unknown heater command (expected auto/on/off)");
    }
    return;
  }

  // You can later add handling for:
  //  - reactor/setpoint/ph
  //  - reactor/setpoint/rpm
  //  - reactor/cmd/pump
  //  - reactor/cmd/motor
}

bool connectMqtt() {
  mqtt.setServer(mqtt_host, mqtt_port);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(30);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(2048);

  tlsClient.setCACert(ISRG_ROOT_X1);

  // Last Will: reactor/state with "status":"offline"
  const bool cleanSession = false;
  const char* lastWillPayload = "{\"status\":\"offline\"}";
  if (!mqtt.connect(client_id, mqtt_user, mqtt_pass,
                    T_STATE, 1, true, lastWillPayload, cleanSession)) {
    Serial.print("[MQTT] Connect failed, state=");
    Serial.println(mqtt.state());
    return false;
  }

  // On connect, announce online presence on reactor/state
  mqtt.publish(T_STATE, "{\"status\":\"online\"}", true);  // retained

  // Subscriptions for setpoints and commands
  mqtt.subscribe("reactor/setpoint/#");
  mqtt.subscribe("reactor/cmd/#");

  Serial.println("[MQTT] Connected & subscribed.");
  return true;
}

void maybeReconnectMqtt() {
  static unsigned long nextAttempt = 0;
  if (mqtt.connected()) return;

  unsigned long now = millis();
  if (now < nextAttempt) return;

  Serial.println("[MQTT] Reconnecting...");
  if (connectMqtt()) {
    nextAttempt = now + 5000;
  } else {
    nextAttempt = now + 5000;
  }
}

void publishTelemetry() {
  if (!mqtt.connected()) return;

  // 1) Simple temp value on reactor/state/temp
  char tempBuf[32];
  snprintf(tempBuf, sizeof(tempBuf), "%.2f", temp);
  mqtt.publish(T_STATE_TEMP, tempBuf, false);

  // 2) JSON summary on reactor/state
  char json[256];
  unsigned long nowSec = millis() / 1000;

  int n = snprintf(json, sizeof(json),
                   "{\"ts\":%lu,"
                   "\"status\":\"online\","
                   "\"temp_C\":%.2f,"
                   "\"target_C\":%.2f,"
                   "\"delta_C\":%.2f,"
                   "\"pwm\":%d,"
                   "\"heater_on\":%s,"
                   "\"mode\":\"%s\"}",
                   (unsigned long)nowSec,
                   temp,
                   targetTemp,
                   deltaT,
                   heater,
                   (heater > 0 ? "true" : "false"),
                   (heaterMode == HEATER_AUTO     ? "auto" :
                    heaterMode == HEATER_FORCE_ON ? "force_on" :
                                                    "force_off"));

  if (n > 0 && n < (int)sizeof(json)) {
    mqtt.publish(T_STATE, (const uint8_t*)json, (unsigned)strlen(json), false);
    Serial.print("[MQTT] Telemetry: ");
    Serial.println(json);
  } else {
    Serial.println("[MQTT] Telemetry JSON truncated!");
  }
}

// =============================
// Heater control
// =============================

void updateHeaterControl() {
  currentTime = micros();

  // Control loop every 100 ms
  if (currentTime - T1 > 100000) { // 100000 us = 100 ms
    previousTime = currentTime;
    T1 += 100000;

    int adcRaw = analogRead(thermistorPin);
    Vadc = Kadc * adcRaw;
    Rth  = R * Vadc / (Vcc - Vadc); // voltage divider math

    // Beta equation, returns temp in °C
    float ToK = To + 273.15f;
    float tempK = (ToK * beta) / (beta + ToK * logf(Rth / Ro));
    temp = tempK - 273.15f;

    int desiredHeater = heater;

    if (heaterMode == HEATER_AUTO) {
      // Bang-bang with hysteresis
      if (temp < targetTemp - deltaT) {
        desiredHeater = 200;       // ~78% duty
      } else if (temp > targetTemp + deltaT) {
        desiredHeater = 0;
      }
    } else if (heaterMode == HEATER_FORCE_ON) {
      desiredHeater = 200;
    } else if (heaterMode == HEATER_FORCE_OFF) {
      desiredHeater = 0;
    }

    if (desiredHeater != heater) {
      heater = desiredHeater;

      analogWrite(heaterpin, heater);
      digitalWrite(13, heater > 0);

      Serial.print("[HEATER] PWM=");
      Serial.print(heater);
      Serial.print(" temp=");
      Serial.print(temp);
      Serial.print(" target=");
      Serial.print(targetTemp);
      Serial.print(" mode=");
      Serial.println(
        heaterMode == HEATER_AUTO     ? "auto" :
        heaterMode == HEATER_FORCE_ON ? "force_on" : "force_off"
      );
    }
  }

  // Debug prints every 1 second
  if (currentTime - T2 > 1000000) { // 1,000,000 us = 1 s
    T2 += 1000000;
    Serial.print("[THERM] Vadc=");
    Serial.print(Vadc, 3);
    Serial.print(" V, Rth=");
    Serial.print(Rth, 1);
    Serial.print(" ohm, temp=");
    Serial.print(temp, 2);
    Serial.print(" C, target=");
    Serial.println(targetTemp, 2);
  }
}

// =============================
// Arduino setup/loop
// =============================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] Heater + MQTT + WiFi starting...");

  pinMode(thermistorPin, INPUT);
  pinMode(heaterpin, OUTPUT);
  pinMode(13, OUTPUT);
  analogWrite(heaterpin, 0);
  digitalWrite(13, LOW);

  T1 = micros();
  T2 = T1;

  wifi_connect(20);
  ensureWifi();
  connectMqtt();
}

void loop() {
  ensureWifi();

  if (!mqtt.connected()) {
    maybeReconnectMqtt();
  }
  mqtt.loop();  // handle keepalive + incoming messages

  updateHeaterControl();

  unsigned long now = millis();
  if (mqtt.connected() && now - lastTelemetryMs >= telemetryPeriodMs) {
    lastTelemetryMs = now;
    publishTelemetry();
  }
}

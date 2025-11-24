/************************************************************
   pH Meter + eduroam WiFi (WPA2-Enterprise) + MQTT + Control
*************************************************************/

// ===========================================================
// ------------------------ CONFIG ---------------------------
// ===========================================================

// --- eduroam enterprise WiFi config ---
#define USE_EDUROAM 1
#define WIFI_USER "zcabayu@ucl.ac.uk"
#define WIFI_SSID "eduroam"
#define WIFI_PASS "xxx"   // TODO: put your real eduroam password here

const char* ssid     = WIFI_SSID;
const char* eap_id   = WIFI_USER;   // identity
const char* eap_user = WIFI_USER;   // username
const char* eap_pass = WIFI_PASS;   // password
const char* pass     = WIFI_PASS;

// --- MQTT config ---
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "esp_wifi.h"
#include <math.h>

const char* mqtt_server = "192.168.1.10";  // <-- set your MQTT broker address

// --- Topic names ---
const char* TOPIC_STATE_PH    = "reactor/state/ph";
const char* TOPIC_SETPOINT_PH = "reactor/setpoint/ph";

// ===========================================================
// ----------- Includes for WPA2 Enterprise handling ---------
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
// ----------------- MQTT Client Setup -----------------------
// ===========================================================
WiFiClientSecure espClient;
PubSubClient client(espClient);


// ===========================================================
// ------------------ SENSOR / CONTROL ------------------------
// ===========================================================

#define SensorPin A0
#define LED_PIN LED_BUILTIN
#define PUMP_PIN 5

#define Offset 0.00
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth 40

int pHArray[ArrayLenth];
int pHArrayIndex = 0;

float pHValue    = 7.0;
float voltage    = 0.0;
float phSetpoint = 7.0;
float phTolerance = 0.1;


// ===========================================================
// ----------------------- WIFI CODE -------------------------
// ===========================================================

void wifi_connect(float timeout_sec = 15.0f) {
  Serial.println();
  Serial.println("[WiFi] Connecting to eduroam...");

  WiFi.disconnect(true);
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

  unsigned long deadline = millis() + (unsigned long)(timeout_sec * 1000.0);
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


// ===========================================================
// ----------------------- MQTT CODE -------------------------
// ===========================================================

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[64];
  if (length >= sizeof(msg)) length = sizeof(msg) - 1;

  for (unsigned int i = 0; i < length; i++)
    msg[i] = (char)payload[i];
  msg[length] = '\0';

  Serial.print("[MQTT] Received on ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(msg);

  if (strcmp(topic, TOPIC_SETPOINT_PH) == 0) {
    float newSP = atof(msg);
    if (newSP > 0 && newSP < 14) {
      phSetpoint = newSP;
      Serial.print("New pH setpoint = ");
      Serial.println(phSetpoint);
    }
  }
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("[MQTT] Connecting...");
    if (client.connect("reactor-ph-controller")) {
      Serial.println("connected!");

      client.subscribe(TOPIC_SETPOINT_PH);
      Serial.println("[MQTT] Subscribed to pH setpoint");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" — retrying in 5 seconds");
      delay(5000);
    }
  }
}


// ===========================================================
// ---------------------- CONTROL LOOP ------------------------
// ===========================================================

void updatePumpControl() {
  bool pumpOn = digitalRead(PUMP_PIN);

  // ASSUMPTION: Pump doses ACID → lowers pH.
  if (pHValue > phSetpoint + phTolerance) {
    pumpOn = true;
  } else if (pHValue < phSetpoint - phTolerance) {
    pumpOn = false;
  }

  digitalWrite(PUMP_PIN, pumpOn ? HIGH : LOW);
}


// ===========================================================
// -------------------- AVERAGE FUNCTION ----------------------
// ===========================================================
double avergearray(int* arr, int number) {
  if (number <= 0)
    return 0;

  long amount = 0;

  if (number < 5) {
    for (int i = 0; i < number; i++)
      amount += arr[i];
    return (double)amount / number;
  }

  int minVal, maxVal;
  if (arr[0] < arr[1]) {
    minVal = arr[0];
    maxVal = arr[1];
  } else {
    minVal = arr[1];
    maxVal = arr[0];
  }

  for (int i = 2; i < number; i++) {
    if (arr[i] < minVal) {
      amount += minVal;
      minVal = arr[i];
    }
    else if (arr[i] > maxVal) {
      amount += maxVal;
      maxVal = arr[i];
    }
    else {
      amount += arr[i];
    }
  }

  return (double)amount / (number - 2);
}


// ===========================================================
// ------------------------- SETUP ---------------------------
// ===========================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  analogReadResolution(12);

  wifi_connect();
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
}


// ===========================================================
// -------------------------- LOOP ---------------------------
// ===========================================================
void loop() {
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();

  ensureWifi();
  if (!client.connected())
    reconnectMQTT();
  client.loop();

  // --------- Sampling every 20ms ---------
  if (millis() - samplingTime > samplingInterval) {
    pHArray[pHArrayIndex++] = analogRead(SensorPin);
    if (pHArrayIndex == ArrayLenth) pHArrayIndex = 0;

    double avgAdc = avergearray(pHArray, ArrayLenth);
    voltage = avgAdc * 3.3 / 4095.0;

    pHValue = 3.5 * voltage + Offset;

    updatePumpControl();

    samplingTime = millis();
  }

  // --------- Print + MQTT update ---------
  if (millis() - printTime > printInterval) {
    Serial.print("Voltage = "); Serial.print(voltage, 3);
    Serial.print(" V, pH = "); Serial.println(pHValue, 2);

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    char payload[16];
    dtostrf(pHValue, 4, 2, payload);
    client.publish(TOPIC_STATE_PH, payload, true);

    printTime = millis();
  }
}

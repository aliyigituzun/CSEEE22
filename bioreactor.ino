#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "esp_wifi.h"
#include <PID_v1.h>
#include <math.h>

#define USE_EDUROAM 1
#define WIFI_USER "xxx@ucl.ac.uk"
#define WIFI_SSID "eduroam"
#define WIFI_PASS "xxx"

static const char* mqtt_host = ""; // MQTT URL
static const uint16_t mqtt_port = 8883;
static const char* mqtt_user = ""; // MQTT Username
static const char* mqtt_pass = "";
static const char* client_id = "reactor-arduino-001";

const char* TOPIC_STATE_PH    = "reactor/state/ph";
const char* TOPIC_STATE_HEAT  = "reactor/state/temp";
const char* TOPIC_STATE_RPM   = "reactor/state/rpm";
const char* TOPIC_SETPOINT_PH   = "reactor/setpoint/ph";
const char* TOPIC_SETPOINT_TEMP = "reactor/setpoint/temp";
const char* TOPIC_SETPOINT_RPM  = "reactor/setpoint/rpm";

#if __has_include("esp_eap_client.h")
  #include "esp_eap_client.h"
  #define USE_NEW_EAP_API 1
#elif __has_include("esp_wpa2.h")
  #include "esp_wpa2.h"
  #define USE_NEW_EAP_API 0
#else
  #error "No enterprise WiFi API found. Install/upgrade ESP32 core."
#endif

const int PIN_MOTOR_ENCODER = 4;
const int PIN_MOTOR_PWM     = 10;

const int PIN_THERMISTOR    = A0;
const int PIN_HEATER        = 2;

const int PIN_PH_SENSOR     = A1;
const int PIN_ACID_PUMP     = 5;
const int PIN_BASE_PUMP     = 6;

const int PULSES_PER_REVOLUTION = 70;
double rpmSetpoint = 0.0, rpmInput = 0.0, rpmOutput = 0.0;
double Kp = 2.5, Ki = 0.5, Kd = 0.1;
PID motorPID(&rpmInput, &rpmOutput, &rpmSetpoint, Kp, Ki, Kd, DIRECT);

const int MOTOR_PWM_CHANNEL = 0;
const int MOTOR_PWM_FREQ    = 20000;
const int MOTOR_PWM_RES_BITS = 8;

const unsigned long MOTOR_CONTROL_INTERVAL = 100;
unsigned long lastMotorControlTime = 0;
volatile long encoderTicks = 0;

void IRAM_ATTR encoderISR() {
  encoderTicks++;
}

void applyMotorOutput(int pwm) {
  if (rpmSetpoint == 0 && pwm < 10) pwm = 0;
  pwm = constrain(pwm, 0, 255);
  ledcWrite(MOTOR_PWM_CHANNEL, pwm);
}

void updateMotor(unsigned long now) {
  if (now - lastMotorControlTime < MOTOR_CONTROL_INTERVAL) return;
  lastMotorControlTime = now;

  noInterrupts();
  long ticks = encoderTicks;
  encoderTicks = 0;
  interrupts();

  double timeInSeconds = (double)MOTOR_CONTROL_INTERVAL / 1000.0;
  rpmInput = (double)ticks / PULSES_PER_REVOLUTION;
  rpmInput = rpmInput / timeInSeconds * 60.0; // RPM

  motorPID.Compute();
  applyMotorOutput((int)rpmOutput);

  Serial.print("[MOTOR] Target: ");
  Serial.print(rpmSetpoint, 0);
  Serial.print(" RPM | Current: ");
  Serial.print(rpmInput, 1);
  Serial.print(" RPM | PWM: ");
  Serial.println((int)rpmOutput);
}

float R  = 15000.0;
float R0 = 10000.0;
float beta = 4220.0;
float baseKelvin = 298.15;
float VCC = 5.0;
float targetTempC = 35.0;
float currentTempC = 0.0;
bool heaterOn = false;
const unsigned long HEAT_SAMPLE_INTERVAL = 2000;
unsigned long lastHeatSampleTime = 0;
const float MAX_TEMP_SAFETY = 60.0;

void updateHeating(unsigned long now) {
  if (now - lastHeatSampleTime < HEAT_SAMPLE_INTERVAL) return;
  lastHeatSampleTime = now;

  int raw = analogRead(PIN_THERMISTOR);
  float thermoVoltage = (VCC * raw) / 4095.0;

  if (thermoVoltage <= 0.0 || thermoVoltage >= VCC) {
    Serial.println("[HEAT] ERROR: Voltage out of range!");
    heaterOn = false;
    digitalWrite(PIN_HEATER, LOW);
    return;
  }

  float thermistorResistance = (R * thermoVoltage) / (VCC - thermoVoltage);
  float x = log(thermistorResistance / R0);
  currentTempC = 1.0 / ((1.0 / baseKelvin) + (x / beta)) - 273.15;

  if (currentTempC > MAX_TEMP_SAFETY) {
    Serial.println("[HEAT] SAFETY: Max temp exceeded!");
    heaterOn = false;
  } else if (currentTempC < targetTempC - 0.5) {
    heaterOn = true;
  } else if (currentTempC > targetTempC + 0.5) {
    heaterOn = false;
  }

  digitalWrite(PIN_HEATER, heaterOn ? HIGH : LOW);

  Serial.print("[HEAT] Target: ");
  Serial.print(targetTempC, 1);
  Serial.print("°C | Current: ");
  Serial.print(currentTempC, 2);
  Serial.print("°C | Heater: ");
  Serial.println(heaterOn ? "ON" : "OFF");
}

const float PH_SLOPE  = 3.947;
float phOffset        = 2.547;

float phSetpoint   = 7.0;
float phTolerance  = 0.2;
float currentPH    = 7.0;

float mqttPhSetpoint = 7.0;

#define PH_NUM_READINGS 5
float phBuffer[PH_NUM_READINGS];
int phBufferIndex = 0;
bool phBufferFilled = false;

enum PhState {
  PH_IDLE,
  PH_ADDING_ACID,
  PH_ADDING_BASE,
  PH_MIXING
};

PhState phState = PH_IDLE;
unsigned long phStateStartMillis = 0;
const unsigned long PH_PUMP_ON_TIME   = 5000;
const unsigned long PH_MIX_TIME       = 10000;
const unsigned long PH_SAMPLE_INTERVAL = 1000;
unsigned long lastPhSampleTime = 0;

float calculateMedian(float arr[], int size) {
  if (size == 0) return phSetpoint;
  float temp[PH_NUM_READINGS];
  for (int i = 0; i < size; i++) {
    temp[i] = arr[i];
  }
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (temp[j] < temp[i]) {
        float swap = temp[i];
        temp[i] = temp[j];
        temp[j] = swap;
      }
    }
  }
  
  if (size % 2 == 1) {
    return temp[size / 2];
  } else {
    return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
  }
}

void samplePH(unsigned long now) {
  if (now - lastPhSampleTime < PH_SAMPLE_INTERVAL) return;
  lastPhSampleTime = now;

  int sensorValue = analogRead(PIN_PH_SENSOR);
  float phVoltage = sensorValue * (3.3 / 4095.0);

  float rawPH = PH_SLOPE * phVoltage + phOffset;  
  phBuffer[phBufferIndex] = rawPH;
  phBufferIndex = (phBufferIndex + 1) % PH_NUM_READINGS;
  if (!phBufferFilled && phBufferIndex == 0) {
    phBufferFilled = true;
  }
  int samplesToUse = phBufferFilled ? PH_NUM_READINGS : phBufferIndex;
  currentPH = calculateMedian(phBuffer, samplesToUse);

  Serial.print("[pH] Raw: ");
  Serial.print(rawPH, 2);
  Serial.print(" | Median: ");
  Serial.print(currentPH, 2);
  Serial.print(" (V=");
  Serial.print(phVoltage, 3);
  Serial.println(")");
}

void updatePH(unsigned long now) {
  samplePH(now);
  
  float phLow = phSetpoint - phTolerance;
  float phHigh = phSetpoint + phTolerance;

  switch (phState) {
    case PH_IDLE:
      digitalWrite(PIN_ACID_PUMP, LOW);
      digitalWrite(PIN_BASE_PUMP, LOW);

      if (currentPH > phHigh) {
        Serial.println("[pH] pH HIGH → Adding ACID");
        digitalWrite(PIN_ACID_PUMP, HIGH);
        phState = PH_ADDING_ACID;
        phStateStartMillis = now;
      } else if (currentPH < phLow) {
        Serial.println("[pH] pH LOW → Adding BASE");
        digitalWrite(PIN_BASE_PUMP, HIGH);
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
        Serial.println("[pH] Mixing complete → IDLE");
      }
      break;
  }
}

WiFiClientSecure tlsClient;
PubSubClient mqtt(tlsClient);
const unsigned long PUBLISH_INTERVAL = 1000;
unsigned long lastPublishTime = 0;

void wifi_connect() {
  Serial.println("\nConnecting to eduroam...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

#if USE_NEW_EAP_API
  esp_eap_client_set_identity((const uint8_t*)WIFI_USER, strlen(WIFI_USER));
  esp_eap_client_set_username((const uint8_t*)WIFI_USER, strlen(WIFI_USER));
  esp_eap_client_set_password((const uint8_t*)WIFI_PASS, strlen(WIFI_PASS));
  esp_eap_client_enable();
  WiFi.begin(WIFI_SSID);
#else
  esp_wifi_sta_wpa2_ent_set_identity((const uint8_t*)WIFI_USER, strlen(WIFI_USER));
  esp_wifi_sta_wpa2_ent_set_username((const uint8_t*)WIFI_USER, strlen(WIFI_USER));
  esp_wifi_sta_wpa2_ent_set_password((const uint8_t*)WIFI_PASS, strlen(WIFI_PASS));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(WIFI_SSID);
#endif

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 30000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi FAILED! Running offline.");
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();
  float value = msg.toFloat();

  Serial.print("MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);

  if (strcmp(topic, TOPIC_SETPOINT_RPM) == 0) {
    if (value >= 0 && value <= 2000) {
      rpmSetpoint = value;
      Serial.print("New RPM: ");
      Serial.println(rpmSetpoint);
    }
  } else if (strcmp(topic, TOPIC_SETPOINT_TEMP) == 0) {
    if (value >= 0 && value <= 80) {
      targetTempC = value;
      Serial.print("New Temp: ");
      Serial.println(targetTempC);
    }
  } else if (strcmp(topic, TOPIC_SETPOINT_PH) == 0) {
    if (value >= 0 && value <= 14) {
      phSetpoint = value;
      Serial.print("New pH: ");
      Serial.println(phSetpoint);
    }
  }
}

void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;

  unsigned long mqttStart = millis();
  while (!mqtt.connected() && millis() - mqttStart < 10000) {
    Serial.print("MQTT connecting...");
    tlsClient.setInsecure();
    
    if (mqtt.connect(client_id, mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      mqtt.subscribe(TOPIC_SETPOINT_RPM);
      mqtt.subscribe(TOPIC_SETPOINT_TEMP);
      mqtt.subscribe(TOPIC_SETPOINT_PH);
      return;
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retrying...");
      delay(2000);
    }
  }
  
  if (!mqtt.connected()) {
    Serial.println("MQTT FAILED! Running offline.");
  }
}

void publishState(unsigned long now) {
  if (now - lastPublishTime < PUBLISH_INTERVAL) return;
  lastPublishTime = now;
  if (!mqtt.connected()) return;

  char buf[32];
  dtostrf(rpmInput, 0, 1, buf);
  mqtt.publish(TOPIC_STATE_RPM, buf);

  dtostrf(currentPH, 0, 2, buf);
  mqtt.publish(TOPIC_STATE_PH, buf);

  dtostrf(currentTempC, 0, 2, buf);
  mqtt.publish(TOPIC_STATE_HEAT, buf);

  Serial.println("[MQTT] State published");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Bioreactor Controller Starting ===");

  pinMode(PIN_MOTOR_PWM, OUTPUT);
  analogReadResolution(12);
  ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_PWM, MOTOR_PWM_CHANNEL);
  ledcWrite(MOTOR_PWM_CHANNEL, 0)
  pinMode(PIN_MOTOR_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCODER), encoderISR, RISING);

  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);

  pinMode(PIN_PH_SENSOR, INPUT);
  pinMode(PIN_ACID_PUMP, OUTPUT);
  pinMode(PIN_BASE_PUMP, OUTPUT);
  digitalWrite(PIN_ACID_PUMP, LOW);
  digitalWrite(PIN_BASE_PUMP, LOW);

  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(0, 255);
  motorPID.SetSampleTime(MOTOR_CONTROL_INTERVAL);

  float initPH = phSetpoint;
  for (int i = 0; i < PH_NUM_READINGS; i++) {
    phBuffer[i] = initPH;
  }
  Serial.print("pH median buffer initialized: ");
  Serial.println(initPH, 2);

  wifi_connect();

  tlsClient.setInsecure();
  mqtt.setServer(mqtt_host, mqtt_port);
  mqtt.setCallback(mqttCallback);

  delay(2000);
  Serial.println("=== System Ready ===");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) {
      reconnectMQTT();
    }
    mqtt.loop();
  }

  unsigned long now = millis();

  updateMotor(now);
  updateHeating(now);
  updatePH(now);
  publishState(now);
}
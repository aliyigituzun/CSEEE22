// ============================================================
// pH CONTROL SYSTEM WITH ROLLING MEDIAN FILTER
// ============================================================

// ------------------------------
// SENSOR CALIBRATION CONSTANTS
// ------------------------------
#define PH_PIN A1
float Slope  = 3.947;   // adjust after calibration
float Offset = 2.547;   // adjust after calibration

// ------------------------------
// PUMP PIN CONFIGURATION
// ------------------------------
#define ACID_PUMP_PIN 5
#define BASE_PUMP_PIN 6

// ------------------------------
// pH CONTROL LIMITS
// ------------------------------
float PH_LOW  = 6.8; //minimum threshold
float PH_HIGH = 7.2; //maximum threshold

// ------------------------------
// TIMING CONSTANTS
// ------------------------------
// These constants make pump/mix times easy to tune.
// Adjust them here instead of editing multiple delay() calls.
const int PUMP_TIME   = 5000;   // ms → how long to run pump (5 seconds)
const int MIX_DELAY   = 10000;  // ms → wait for solution to mix (10 seconds)
const int LOOP_DELAY  = 1000;   // ms → delay between checks when pH is OK

// ------------------------------
// MEDIAN FILTER BUFFER
// ------------------------------
//number of measurements it will base the median on
#define NUM_READINGS 5
float pHBuffer[NUM_READINGS];
int bufferCount = NUM_READINGS;
int bufferIndex = 0;

// ------------------------------
// HELPER FUNCTION: MEDIAN
// ------------------------------
float median(float arr[], int size) {
  float temp[size];
  for (int i = 0; i < size; i++) temp[i] = arr[i];
  for (int i = 0; i < size-1; i++) {
    for (int j = i+1; j < size; j++) {
      if (temp[j] < temp[i]) {
        float swap = temp[i];
        temp[i] = temp[j];
        temp[j] = swap;
      }
    }
  }
  return (size % 2 == 1) ? temp[size/2] : (temp[size/2 - 1] + temp[size/2]) / 2.0;
}

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("pH Control System Setting up");

  pinMode(PH_PIN, INPUT);
  pinMode(ACID_PUMP_PIN, OUTPUT);
  pinMode(BASE_PUMP_PIN, OUTPUT);

  digitalWrite(ACID_PUMP_PIN, LOW);
  digitalWrite(BASE_PUMP_PIN, LOW);

  // Initialize buffer with mean of thresholds
  float initPH = (PH_LOW + PH_HIGH) / 2.0;
  for (int i = 0; i < NUM_READINGS; i++) {
    pHBuffer[i] = initPH;
  }
  Serial.print ("Initializing buffer with safe pH value: ");
  Serial.println(initPH, 2);

  delay(5000);  // sensor stabilization
  Serial.println("pH Control System Started");
}

void loop() {

  // ------------------------------
  // 1. MEASURE pH MEDIAN
  // ------------------------------

  // read sensor
  int sensorValue = analogRead(PH_PIN);
  float voltage = sensorValue * (3.3 / 4095.0);
  float pH = Slope * voltage + Offset;

  // update buffer
  pHBuffer[bufferIndex] = pH;
  bufferIndex = (bufferIndex + 1) % NUM_READINGS;

  // compute median
  float pH_median = median(pHBuffer, NUM_READINGS);

  Serial.print("Raw pH: ");
  Serial.print(pH, 2);
  Serial.print(" | Median pH: ");
  Serial.println(pH_median, 2);

  // ------------------------------
  // 2. CONTROL LOGIC
  // ------------------------------

  if (pH_median > PH_HIGH) {
    Serial.println("pH HIGH → Adding ACID");
    digitalWrite(ACID_PUMP_PIN, HIGH);
    delay(PUMP_TIME);   // use constant
    digitalWrite(ACID_PUMP_PIN, LOW);
    delay(MIX_DELAY);   // use constant
  }
  else if (pH_median < PH_LOW) {
    Serial.println("pH LOW → Adding BASE");
    digitalWrite(BASE_PUMP_PIN, HIGH);
    delay(PUMP_TIME);   // use constant
    digitalWrite(BASE_PUMP_PIN, LOW);
    delay(MIX_DELAY);   // use constant
  }
  else {
    Serial.println("pH OK");
    digitalWrite(ACID_PUMP_PIN, LOW);
    digitalWrite(BASE_PUMP_PIN, LOW);
    delay(LOOP_DELAY);  // use constant
  }
}
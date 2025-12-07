const byte sensorPin = A1;                  // Analog input pin
const float VoltageFactor = 5.0 / 1023.0;   // Convert ADC counts to volts - 4096 max because its a 12 bit system
const float targetPH = 7.0;
const float pHTolerance = 0.2;

// const int ACID_PUMP = 5;
// const int BASE_PUMP = 6;

// void adjustPh(float pH) {
//     if (pH - pHTolerance > targetPH) {
//     Serial.println("pH HIGH → Adding ACID");
//     digitalWrite(ACID_PUMP_PIN, HIGH);
//     digitalWrite(BASE_PUMP_PIN, LOW);
//   }

//   // If pH too LOW → ADD BASE
//   else if (pH + pHTolerance < targePH) {
//     Serial.println("pH LOW → Adding BASE");
//     digitalWrite(BASE_PUMP_PIN, HIGH);
//     digitalWrite(ACID_PUMP_PIN, LOW);
//   }
// }

void setup() {
  Serial.begin(9600);       // Open serial monitor at 9600 baud
  pinMode(sensorPin, INPUT); // a1 pin (the sensor) is set to input to recieve data

  // pinMode(ACID_PUMP_PIN, OUTPUT);
  // pinMode(BASE_PUMP_PIN, OUTPUT);

  // digitalWrite(ACID_PUMP_PIN, LOW);
  // digitalWrite(BASE_PUMP_PIN, LOW);
}

void loop() {
  // Read raw ADC (analouge to digital converter - turns signal into value) value (0–4095)
  float rawValue = analogRead(sensorPin);

  // Convert to voltage`
  float voltage = rawValue * VoltageFactor;

  // Convert to pH
  float pH = (voltage * 3.362) + 3.488;
  float betterpH = (0.009722 * voltage * voltage) + (voltage * 0.4326) - 1.449;
  
  // Activate pumps to maintain the pH
  //adjustPh(pH);

  // Print results
  Serial.print("Raw ADC = ");
  Serial.print(rawValue);
  Serial.print(" | Voltage = ");
  Serial.print(voltage, 3); // 3 decimal places
  Serial.println(" V");
  // Serial.print(" | pH = ");
  // Serial.print(pH, 3);
  // Serial.println();
  // Serial.print(" | betterpH = ");
  // Serial.print(betterpH, 3);
  // Serial.println();

  delay(1000); // Update once per second
}
#define THERMISTOR_PIN A0
#define HEATER_PIN 3

float R = 15000;
float R0 = 10000;
float beta = 3700;
// float beta = 4220;
float targetTemp = 35; // degrees Celcius
float baseKelvin = 298.15; // Thermistor's base temperature, in kelvin (25°C)
float currentTemp;
float thermistorResistance;
float voltage;
float VCC = 3.3; // ESP32 uses 3.3V
// bool heater;
int inputVoltage;

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12); // make sure ADC is 12-bit
  pinMode(THERMISTOR_PIN, INPUT);
  pinMode(HEATER_PIN, OUTPUT);
}

void loop()
{
  Vadc = analogRead(A0); // 12-bit int values between 0-4095
  voltage = (VCC * Vadc) / 4095.0; // by default, arduino nano esp32 uses 3.3V
  
  // beta is a constant value given by our model of thermistor
  thermistorResistance = R * voltage / (VCC - voltage);

  // currentTemp = (baseKelvin) * beta / (beta + (baseKelvin) * log(termistorResistance / R0)) - 273.15;

  // another B-parameter equation for calculating temperature
  float x = log(thermistorResistance / R0);
  currentTemp = 1.0 / ((1.0 / baseKelvin) + (x / beta)) - 273.15;

  Serial.println("");

  Serial.print("Target: ");
  Serial.print(targetTemp);
  Serial.println("°C");

  Serial.print("Temperature: ");
  Serial.print(currentTemp);
  Serial.println("°C");

  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

  Serial.print("ADC: ");
  Serial.print(Vadc);

  Serial.print("Resistance: ");
  Serial.print(thermistorResistance);
  Serial.println(" Ω");

  if (currentTemp < targetTemp - 0.5) // stop 0.5°C before the target
  {
    digitalWrite(HEATER_PIN, HIGH);
  }
  else
  {
    digitalWrite(HEATER_PIN, LOW);
  }

  delay(2000); // two second delay between each read
}

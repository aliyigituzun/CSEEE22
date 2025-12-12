#define THERMISTOR_PIN A0
#define HEATER_PIN D2

float R = 15000;
float R0 = 10000;
float beta = 4220;
float targetTemp = 20;
float baseKelvin = 298.15;
float currentTemp;
float thermistorResistance;
float voltage;
float VCC = 5;
int counter = 0;
float offset = -2.0;
int Vadc;

void setup()
{
  pinMode(HEATER_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop()
{
  Vadc = analogRead(A0);
  voltage = (VCC * Vadc) / 4095;
  thermistorResistance = (R * voltage) / (VCC - voltage);

  float x = log(thermistorResistance / R0);
  currentTemp = 1.0 / ((1.0 / baseKelvin) + (x / beta)) - 273.15;
  counter++;
  Serial.println("");
  Serial.println(counter);

  Serial.print("Target: ");
  Serial.print(targetTemp+offset);
  Serial.println("°C");

  Serial.print("Temperature: ");
  Serial.print(currentTemp);
  Serial.println("°C");

  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

  Serial.print("ADC: ");
  Serial.println(Vadc);

  Serial.print("Resistance: ");
  Serial.print(thermistorResistance);
  Serial.println(" Ω");

  if (currentTemp < targetTemp - 0.5)
  {
    Serial.println("Setting HEATER_PIN to HIGH");
    digitalWrite(HEATER_PIN, HIGH);
  }
  else
  {
    Serial.println("Setting HEATER_PIN to Low");
    digitalWrite(HEATER_PIN, LOW);
  }

  delay(2000);
}
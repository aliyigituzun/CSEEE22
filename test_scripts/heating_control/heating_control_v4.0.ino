/*
 * This is the code for controlling the heating subsystem
 * There are multiple constants at the top. R and R0 are standard resistances   
   for 25°C of the resistor
 * "thermistorResistance" is the actual calculated resistance
 * The ESP32 internally uses 3.3V, but from the PSU it receives 5V, which is  
   why VCC = 5
 * Some values are commented out; they were for testing
*/

// ==== How it works ====
/*
 * Read analog output from pin A0 and transform it into a voltage
 * Calculate current thermal resistance of the thermistor
 * Use an equation that uses constants given by our specific thermistor 
   (ND06P00103K)
*/

#define THERMISTOR_PIN A0
#define HEATER_PIN D2

float R = 15000;
float R0 = 10000;
//float beta = 3700;
float beta = 4220;
float targetTemp = 20; // degrees Celcius
float baseKelvin = 298.15; // Thermistor's base temperature, in kelvin (25°C)
float currentTemp;
float thermistorResistance;
float voltage;
float VCC = 5;
// float VCC = 3.3;
int counter = 0;
float offset -2.0;

// bool heater;
int Vadc;

void setup()
{
  //analogReadResolution(12); // make sure ADC is 12-bit
  pinMode(HEATER_PIN, OUTPUT);
  Serial.begin(115200);
  // make sure the board rate is 115200
}

void loop()
{
  Vadc = analogRead(A0); // 12-bit int values between 0-4095
  voltage = (VCC * Vadc) / 4095;
  
  // beta is a constant value given by our model of thermistor
  thermistorResistance = (R * voltage) / (VCC - voltage);

  // unused
  // currentTemp = (baseKelvin) * beta / (beta + (baseKelvin) * log(termistorResistance / R0)) - 273.15;

  // another B-parameter equation for calculating temperature
  float x = log(thermistorResistance / R0);
  currentTemp = 1.0 / ((1.0 / baseKelvin) + (x / beta)) - 273.15;
  counter++;
  
  // debug
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

  if (currentTemp < targetTemp - 0.5) // stop 0.5°C before the target
  {
    Serial.println("Setting HEATER_PIN to HIGH");
    digitalWrite(HEATER_PIN, HIGH);
    //analogWrite(HEATER_PIN, 200); // unused
  }
  else
  {
    Serial.println("Setting HEATER_PIN to Low");
    digitalWrite(HEATER_PIN, LOW);
    //analogWrite(HEATER_PIN, 0); // unused
  }

  delay(2000); // two second delay between each read
}
const byte thermistorpin = A0;
const byte heaterpin = 2;
const float targetTemp = 35; // celsius
const float deltaT = 0.5;
const float Vcc = 5; // voltage
const float R = 15000; // 15 kΩ
const float beta = 3700;
const float Ro = 10000;
const float To = 25;
const float Kadc = 3.3 / 4095; // convert raw ADC into volts

float Vadc, temp, Rth;
long currentTime, previousTime, T1, T2;
// T1 controls 100ms updates
// T2 controls 1-second serial prints

int heater = 0; // using PWM
int previousHeater = 0;

void setup()
{
  pinMode(thermistorPin, INPUT);
  pinMode(heaterpin, OUTPUT);
  pinMode(13, OUTPUT);

  Serial.begin(2000000);

  T1 = micros();
  T2 = T1;
}

void loop()
{
  currentTime = micros();

  if (currentTime - T1 > 100000) // per 100ms
  {
    previousTime = currentTime;
    T1 += 100000;

    Vadc = Kadc * analogRead(thermistorPin);
    Rth = R * Vadc / (Vcc - Vadc);
    temp = (To + 273) * beta / (beta + (To + 273) * log(Rth / Ro)) - 273;

    if (T < targetTemp - deltaT)
      heater = 200;
    if (T > targetTemp + deltaT)
      heater = 0;

    // makes sure only updates when the heater changes state
    if (heater != previousHeater)
    {
      analogWrite(heaterpin, heater); 
      digitalWrite(13, heater > 0);
      previousHeater = heater;
    }
  }

  if (currentTime - T2 > 1000000) // per second
  {
    T2 += 1000000;
    Serial.println(Vadc);
    Serial.println(Rth);
    Serial.println(temp);
    Serial.println("")
  }
}

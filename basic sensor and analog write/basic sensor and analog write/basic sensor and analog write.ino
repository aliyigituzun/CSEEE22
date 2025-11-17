const int transistorPin = 9

  void setup()
  {
    Serial.begin(9600);
    pinMode(transistorPin, OUTPUT);
    

  }

  void loop()
  {
    // potentiometer
    int sensorValue = analogRead(A0);

    // analog is 10-bit, so 0-1023
    // voltage is 8-bit, so 0-255
    int outputValue = map(sensorValue, 0, 1023, 0, 255);

    analogWrite(transistorPin, outputValue);
  }

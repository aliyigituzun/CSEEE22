#include <PID_v1.h>

const int ENCODER_PIN = 2;
const int MOTOR_PWM_PIN = 10;
const int PULSES_PER_REVOLUTION = 70;

double Setpoint, Input, Output;
double Kp = 2.5, Ki = 0.5, Kd = 0.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const long controlInterval = 100;
unsigned long lastTime;

volatile long encoderTicks = 0;
long ticksToMeasure = 0;

void encoderISR() {
  encoderTicks++;
}

void applyMotorOutput(int pwm) {
  if (Setpoint == 0 && pwm < 10) { 
    pwm = 0;
  }
  pwm = constrain(pwm, 0, 255);
  analogWrite(MOTOR_PWM_PIN, pwm);
}

void calculateRPM() {
  noInterrupts();
  ticksToMeasure = encoderTicks;
  encoderTicks = 0;
  interrupts();
  
  const double timeInSeconds = (double)controlInterval / 1000.0;
  
  Input = (double)ticksToMeasure / PULSES_PER_REVOLUTION;
  Input = Input / timeInSeconds;
  Input = Input * 60.0;
}

void processSerialInput() {
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();
    if (inputString.length() > 0) {
      double newSetpoint = inputString.toDouble();
      if (newSetpoint >= 0) {
        Setpoint = newSetpoint;
        Serial.print("\n>>> New Target RPM: ");
        Serial.print(Setpoint);
        Serial.println(" <<<");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  Setpoint = 0.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(controlInterval);
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  
  if (now - lastTime >= controlInterval) {
    calculateRPM();
    myPID.Compute(); 
    applyMotorOutput((int)Output);
    lastTime = now;
    
    Serial.print("Target RPM: ");
    Serial.print(Setpoint);
    Serial.print(" | Current RPM: ");
    Serial.print(Input, 1);
    Serial.print(" | PWM Output: ");
    Serial.println((int)Output);
  }
  
  processSerialInput();
}
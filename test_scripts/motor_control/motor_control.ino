#include <PID_v1.h>

// Pins and Encoder
const int ENCODER_PIN = 2;   // Interrupt Pin for Encoder
const int MOTOR_PWM_PIN = 10; // PWM Pin for Motor Speed
const int PULSES_PER_REVOLUTION = 70;

// PID Variables and Constants (Tune these!)
double Setpoint, Input, Output;
double Kp = 2.5, Ki = 0.5, Kd = 0.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Control Timing
const long controlInterval = 100; // ms
unsigned long lastTime;

// Motor State
volatile long encoderTicks = 0;
long ticksToMeasure = 0;

void encoderISR() {
  encoderTicks++;
}

void applyMotorOutput(int pwm) {
  // If setpoint is zero, ensure PWM output is zero.
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
  
  Input = (double)ticksToMeasure / PULSES_PER_REVOLUTION; // Revolutions
  Input = Input / timeInSeconds;                        // RPS
  Input = Input * 60.0;                                 // RPM
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
  // Removed pinMode and digitalWrite for MOTOR_DIR_PIN
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
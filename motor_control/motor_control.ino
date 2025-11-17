// Motor (plant) and sensor parameters
const float Npulses = 70; // Number of pulses per revolution
const float freqtoRPM = 60/Npulses;
const int RPMmax = 1500; // Max RPM indicated by sensor (to mitigate any noise)
const int Tmin = 6e7/RPMmax/Npulses; // Tmin = (60,000,000 microseconds per minute) / (max RPM * pulses per rev)

// Declare constants and variables
const byte encoderpin = 2, motorpin = 10;
long currtime, prevtime, pulseT, prevpulseT, T1;
float measspeed, meanmeasspeed, freq, deltaT;
int Vmotor, trig;

void setup()
{
  pinMode(encoderpin, INPUT_PULLUP); // one of the interrupt pins (D2/D3 on an Uno/Nano/Nucleo32)
  pinMode(motorpin, OUTPUT); // motor PWM control signal, to MOSFET
  pinMode(13, OUTPUT); // The on-board LED, blinks when shaft is rotated
  attachInterrupt(digitalPinToInterrupt(encoderpin), freqcount, RISING);
  Serial.begin(250000); // use serial monitor to monitor speed, motor voltage etc

  // Pins D9 and D10 - 7.8 kHz 10bit PWM - for Uno or Nano
  TCCR1A = 0b00000011; // 10bit
  TCCR1B = 0b00000001; // 7.8 kHz
}

void loop()
{
  currtime = micros();
  deltaT = (currtime-prevtime)*1e-6; // measure deltaT
  if (currtime-T1>0) {
  prevtime = currtime;
  T1 = T1+10000; // 10 ms update period

  measspeed = freq*freqtoRPM; // measured speed in RPM (N pulses per revolution)

  Vmotor = constrain(Vmotor,0,1023); // Limit motor voltage to 5 V - assuming 5 V motor supply
  if (Vmotor==255) {Vmotor=256;} // Work-around for bug in Nano/Uno with 10-bit configuration
  analogWrite(motorpin,Vmotor);
  if (currtime-pulseT>1e5) {measspeed=0; freq=0;} // Indicate zero measured speed if no pulses detected for 0.1 s
  meanmeasspeed = 0.1*measspeed+0.9*meanmeasspeed;

  Serial.print(0); Serial.print(","); Serial.print(1000); Serial.print(",");
  Serial.println(meanmeasspeed); // for Arduino IDE or CoolTerm etc Serial Plotter
}

/////////////////////////// Interrupt routine to measure frequency ///////////////////////////
void freqcount()
{
  pulseT = micros();
  if(abs(pulseT-prevpulseT)>Tmin){ // Ignore indicated speeds > RPMmax
  freq = 0.75*freq+2.5e5/float(pulseT-prevpulseT); // Calculate speed sensor frequency (Hz)
  prevpulseT = pulseT;
  trig++; if (2*trig>=Npulses) {trig=0;
  digitalWrite(13, !digitalRead(13));}} // blink on-board LED in time with motor, one blink per revolution
}
#include <Wire.h>

// DAC COMMANDS
#define ADDR 0b0011101
#define CODE_LOAD 0x01
#define USER_CONFIG 0b00001000

// MOTOR PINS
int DIR = 2;

// motor speed detect
volatile long pwm_value = 0;
volatile long prev_time = 0;
volatile double rpm = 0;
volatile double timeSec = 0;
volatile double microSec = 0;

double normalDist[7] = {0.00443185, 0.05399097, 0.24197072, 0.39894228, 0.24197072, 0.05399097, 0.00443185};
double rpms[7];
double times[4];

bool flop = 0;

void setup() {
  Serial.begin(115200);

  // Setup DAC I2C
  Wire.begin();

  // Setup motor pins
  pinMode(DIR, OUTPUT);
  attachInterrupt(1, falling, FALLING);

  prev_time = micros();
}

// Set voltage on DAC
void setVoltage(double VOLTAGE){
  uint16_t voltageBinary = 0;
  voltageBinary = (uint16_t) ((VOLTAGE / 5.0) * 16383.0);
  voltageBinary = voltageBinary << 2;
  sendCommand(CODE_LOAD, voltageBinary);
}

// Send command to DAC
void sendCommand(uint8_t COMMAND, uint16_t DATA){
  uint8_t data[3];
  data[0] = COMMAND;
  data[1] = (DATA >> 8) & 0xff;
  data[2] = (DATA << 0) & 0xff;
  Wire.beginTransmission(ADDR);
  Wire.write(data, 3);
  Wire.endTransmission();
}

// Set motor speed
void setMotorSpeed(double rpm, boolean forward){
  if(!forward){
    digitalWrite(DIR, LOW);
  }
  else{
    digitalWrite(DIR, HIGH);
  }
  double voltageSet = (rpm / 15000.0) * 5.0;
  setVoltage(voltageSet);
}

void falling() {
  microSec = micros();
  pwm_value = microSec - prev_time;
  for(int a=6;a>0;a--){
    rpms[a] = rpms[a - 1];
  }
  for(int a=4;a>0;a--){
    times[a] = times[a - 1];
  }
  times[0] = ((microSec + pwm_value / 2.0) / 1000000.0);
  timeSec = times[3];
  rpms[0] = 10.0 / (((double) pwm_value) / 1000000.0);
  rpm = rpms[0] * normalDist[0] + rpms[1] * normalDist[1] + rpms[2] * normalDist[2] + rpms[3] * normalDist[3] + rpms[4] * normalDist[4] + rpms[5] * normalDist[5] + rpms[6] * normalDist[6];
  prev_time = microSec;
}

double index = 0;
void loop() {
  delay(10);
  index += 1;
  if(index > 100){
    index = 0;
    flop = !flop;
  }
  setMotorSpeed(3000 + 5000 * flop, 0);
  Serial.print(timeSec, 4);
  Serial.print(", ");
  Serial.print(rpm);
  Serial.println();
}

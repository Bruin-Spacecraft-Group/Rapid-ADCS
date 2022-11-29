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
int delayCycles = 6;
volatile int cycleIndex = 0;

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
  cycleIndex++;
  if(cycleIndex >= delayCycles){
    cycleIndex = 0;
    microSec = micros();
    pwm_value = microSec - prev_time;
    timeSec = ((microSec + pwm_value / 2.0) / 1000000.0);
    prev_time = microSec;
    rpm = 60.0 / (((double) pwm_value) / 1000000.0);
  }
}

double index = 0;
void loop() {
  delay(10);
  index += 10;
  if(index > 10000){
    index = 2000;
  }
  setMotorSpeed(index, 0);
  Serial.print(timeSec, 4);
  Serial.print(", ");
  Serial.print(rpm);
  Serial.println();
}

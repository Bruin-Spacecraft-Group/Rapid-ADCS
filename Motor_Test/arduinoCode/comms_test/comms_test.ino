/*
 * README
 * For option 4 to work, make sure you set your serial monitor to "no line ending."
 */


#include <Wire.h>
#include "GenericFifo.hpp"

using namespace fifolib::generic;

// DAC COMMANDS
#define ADDR 0b0011101
#define CODE_LOAD 0x01
#define USER_CONFIG 0b00001000

// MOTOR PINS
int DIR = 2;

// motor speed detect
volatile int pwm_value = 0;
volatile int prev_time = 0;
volatile double rpm = 0;
volatile double prev_rpm = 0;
volatile int count = 0;
volatile double totalRPM = 0;
int maxCount = 10;
volatile double timeSec = 0;
volatile double prevTime = 0;
bool highSpeed = 0;
volatile long acc_t1 = 0;
volatile double accel = 0;
volatile double cur_time = 0;
double tmp = 0;

int test_config = 0;

GenericFifoWriter<sizeof(long)>* writer;
GenericFifoReader<sizeof(long)>* reader;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Hello world!");

  setMotorSpeed(1000, 0);

  writer = init_writer<sizeof(long)>(10);
  reader = open_reader<sizeof(long)>(*writer, 1);

  // Setup DAC I2C

  // Setup motor pins
  pinMode(DIR, OUTPUT);
  attachInterrupt(1, falling, FALLING);

  prevTime = micros();
  acc_t1 = prev_time;
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

long counter = 0;

void falling() {
  cur_time = micros();
  pwm_value = cur_time-prev_time;
  rpm = ((1.0 / (((double) pwm_value) / 1000000.0)) / 6.0) * 60.0;
  totalRPM += rpm;
  timeSec += (((double) pwm_value) / 1000000.0);
  count++;
  if(count >= maxCount){
    rpm = totalRPM / (double) maxCount;
    count = 0;
    totalRPM = 0;
    accel = (rpm - prev_rpm) / ((cur_time - acc_t1) / 1000000.0);
    acc_t1 = cur_time;
    prev_rpm = rpm;
//    Serial.print("rpm = ");
//    Serial.println(rpm);
  }
  prev_time = cur_time;
  writer->try_write(++counter);
  Serial.println(String("Wrote counter: ") + String(counter));

}

double configured_rpm = 0; // consider this min rpm
bool change_rpm = true;
bool cycle_started = false;
long t1 = 0;

void stopAndWaitForStop() {
  setMotorSpeed(1000, 0);
  while (rpm >= 1100) {;
//    Serial.print("J'attend. rpm = ");
//    Serial.println(rpm);
    delay(10);
  }
  setMotorSpeed(0, 0);
  delay(500);
}

long buffer = 0;

void loop() {
  delay(100);
  while (reader->try_read(buffer)) {
    const String msg = String(buffer);
    Serial.println(msg);
  }
}

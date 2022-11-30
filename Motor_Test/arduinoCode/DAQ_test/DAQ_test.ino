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
long t1 = 0;

int test_config = 0;

struct DataSample {
  const double timeSec;
  const double rpm;
};

GenericFifoWriter<sizeof(DataSample)>* writer;
GenericFifoReader<sizeof(DataSample)>* reader;

void setup() {
  Serial.begin(115200);

  writer = init_writer<sizeof(DataSample)>(15);
  reader = open_reader<sizeof(DataSample)>(*writer, 1);

  // Setup DAC I2C
  Wire.begin();

  // Setup motor pins
  pinMode(DIR, OUTPUT);

  setMotorSpeed(0, 0);

  Serial.println("Please configure DAQ test.");
  Serial.println("  1. Speed vs torque");
  Serial.println("  2. Speed control precision");
  Serial.println("  3. Max speed");
  Serial.println("  4. Current vs continuous speed");
  Serial.println("  5. Frequency response");

  while (test_config < 1 || test_config > 5) {
    Serial.print("Please input a number between 1 and 5. ");
    while (Serial.available() == 0) {}
    test_config = Serial.parseInt();
    Serial.println(test_config);
  }

  attachInterrupt(1, falling, FALLING);

  prevTime = micros();
  t1 = prevTime;
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

// interrupts to sample motor speed
void falling() {
  cur_time = micros();
  pwm_value = cur_time-prev_time;
  rpm = ((1.0 / (((double) pwm_value) / 1000000.0)) / 6.0) * 60.0;
  totalRPM += rpm;
  timeSec += (((double) pwm_value) / 1000000.0);

  count++;
  if (count >= maxCount){
    rpm = totalRPM / (double) maxCount;
    count = 0;
    totalRPM = 0;
    prev_rpm = rpm;

    const DataSample sample = {timeSec, rpm}; // {timeSec, rpm}
    writer->try_write(sample);
    std::size_t failedWrites = writer->getFailedWrites();
    if (failedWrites % 10 == 0 && failedWrites > 0) {
      Serial.println(String("Lost: ") + String(failedWrites) + String(" messages"));
    }
    count = 0;
  }
  prev_time = cur_time;
}

double configured_rpm = 0; // consider this min rpm
bool change_rpm = true;
bool cycle_started = false;

void stopAndWaitForStop() {
  setMotorSpeed(1000, 0);
  while (rpm >= 1100) {;
    delay(10);
  }
  setMotorSpeed(0, 0);
  delay(500);
}

DataSample buffer{};
String suffix("");
double freq = 0.5;
double delta_t = 0;

void loop() {

  if (test_config == 1) { // Test maximal speed/torque curve
    if (!cycle_started) {
      setMotorSpeed(15000, 0);
      configured_rpm = 15000;
      cycle_started = true;
    } else if (buffer.rpm >= 14700) {
      stopAndWaitForStop();
      cycle_started = false;
      return;
    }
  } else if (test_config == 2) { // Speed control precision
    if (!cycle_started) {
      configured_rpm += 500; // Go in increments of 500 rpm
      if (configured_rpm > 15000) {
        configured_rpm = 0;
        stopAndWaitForStop();
        return;
      }
      setMotorSpeed(configured_rpm, 0);
      cycle_started = true;
      t1 = micros();
    } else if ((micros() - t1) / 1000000.0 >= 5.0) { // 5 seconds to wait for motor velocity to converge
      cycle_started = false;
      return;
    } // else, do nothing. Let test run
  } else if (test_config == 3) { // Vroom vroom bitch
    setMotorSpeed(15000, 0); // You can end this test manually
    configured_rpm = 15000;
  } else if (test_config == 4) { // You'll need to manually record current draw from power supply
    configured_rpm += 500; // Go in increments of 500 rpm
    if (configured_rpm > 15000) {
      configured_rpm = 0;
      stopAndWaitForStop();
      return;
    }
    setMotorSpeed(configured_rpm, 0);
    delay(3000); // Wait for things to settle
    Serial.flush();
    Serial.print("Current draw (in Amps): ");
    while (Serial.available() == 0) {}
    float current = Serial.parseFloat();
    Serial.println(current, 3);

    suffix = String(", ") + String(current, 3);

    reader->clear_buffer();
    delay(1500);
    // return;
  } else if (test_config == 5) {
    double t2 = micros();
    delta_t += (t2 - t1)/1000000.00;
    t1 = t2;

    configured_rpm = 15000*sin(2*PI*delta_t*freq);
    setMotorSpeed(configured_rpm, 0);
    suffix = String(", ") + String(freq);

    if (delta_t >= 5) {
      freq += 0.5;
      delta_t = 0;
    }
  }

  // delay(10);
//  Serial.print("Messages to be printed: ");
//  Serial.println(reader->num_messages_to_read());
  while (reader->try_read(buffer)) {
    const String msg = String(buffer.timeSec, 4) + String(", ") + String(buffer.rpm, 4) + String(", ") + String(configured_rpm) + suffix;
    Serial.println(msg);
    
//    Serial.print(buffer.timeSec);
//    Serial.print(", ");
//    Serial.print(configured_rpm);
//    Serial.print(", ");
//    Serial.println();
  }
}

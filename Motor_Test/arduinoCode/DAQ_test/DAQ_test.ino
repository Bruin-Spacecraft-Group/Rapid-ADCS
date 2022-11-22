#include <Wire.h>

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
long acc_t1 = 0;
volatile double accel = 0;
double cur_time = 0;
double tmp = 0;

int test_config = 0;

void setup() {
  Serial.begin(115200);

  // Setup DAC I2C
  Wire.begin();

  // Setup motor pins
  pinMode(DIR, OUTPUT);
  attachInterrupt(1, rising, RISING);

  Serial.println("Please configure DAQ test.");
  Serial.println("  1. Speed vs torque");
  Serial.println("  2. Speed control precision");
  Serial.println("  3. Max speed");
  Serial.println("  4. Current vs continuous speed");

  while (test_config < 1 || test_config > 4) {
    Serial.println("Please input a number between 1 and 4.");
    test_config = Serial.parseInt();
  }

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

// interrupts for determining motor speed
void rising() {
  attachInterrupt(1, falling, FALLING);
}
void falling() {
  attachInterrupt(1, rising, RISING);
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
  }
  prev_time = cur_time;


}

double configured_rpm = 0;
bool change_rpm = true;
bool cycle_started = false;
long t1 = 0;

void waitForStop() {
  while (rpm >= 100) {
    delay(10);
  }
}

void loop() {

  if (test_config == 1) { // Test maximal speed/torque curve
    if (!cycle_started) {
      setMotorSpeed(15000, 0);
      configured_rpm = 15000;
      cycle_started = true;
    } else if (rpm >= 14700) {
      setMotorSpeed(0, 0);
      configured_rpm = 0;
      waitForStop()
      cycle_started = false;
      return;
    }
  } else if (test_config == 2) { // Speed control precision
    if (!cycle_started) {
      configured_rpm += 500; // Go in increments of 500 rpm
      if (configured_rpm > 15000) {
        configured_rpm = 0;
        setMotorSpeed(0, 0);
        waitForStop()
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
  } else if (test_config == 4) { // You'll need to manually record current draw from power supply
    configured_rpm += 500; // Go in increments of 500 rpm
    if (configured_rpm > 15000) {
      configured_rpm = 0;
      setMotorSpeed(0, 0);
      waitForStop()
      return;
    }
    setMotorSpeed(configured_rpm, 0);
    delay(3000); // Wait for things to settle
    Serial.println("Current draw (in Amps):");
    float current = Serial.parseFloat();
    Serial.print(configured_rpm);
    Serial.print(", ");
    Serial.print(rpm);
    Serial.print(", ");
    Serial.print(current);
    return;
  }

  delay(10);

  Serial.print(timeSec);
  Serial.print(", ");
  Serial.print(configured_rpm);
  Serial.print(", ");
  Serial.print(rpm);
  Serial.print(", ");
  Serial.print(accel);
  Serial.println();
}

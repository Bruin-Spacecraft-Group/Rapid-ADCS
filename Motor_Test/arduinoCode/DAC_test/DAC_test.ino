#include <Wire.h>

// DAC COMMANDS
#define ADDR 0b0011101
#define CODE_LOAD 0x01
#define USER_CONFIG 0b00001000

double reqVoltage = 2.0;

void setup() {
  Serial.begin(9600);

  // Setup DAC I2C
  Wire.begin();
}

// Set voltage on DAC
void setVoltage(double VOLTAGE){
  uint16_t voltageBinary = 0;
  voltageBinary = (uint16_t) ((VOLTAGE / 5.0) * 16383.0);
  Serial.println(voltageBinary);
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

double voltage = 0;
double index = 0;
void loop() {
  delay(5);
  index += 0.1;
  voltage = 1.25 * sin(index) + 1.25 * cos(1.1 * index) + 2.5;
  setVoltage(voltage);
}

#include "util.hpp"
#include <pigpio.h>
#include <unistd.h>
#include <iomanip>

using namespace std;

void parseTestConfig(int& test_config) {
  if (test_config < 1 || test_config > 6) {
    cout << "Please configure a valid test." << endl;
    cout << "  1. Speed vs torque" << endl;
    cout << "  2. Speed control precision" << endl;
    cout << "  3. Max speed" << endl;
    cout << "  4. Current vs continuous speed" << endl;
    cout << "  5. Frequency response" << endl;
    cout << "  6. Custom" << endl;
    cin >> test_config;
    parseTestConfig(test_config);
  }
}

void delay(int milli) {
  usleep(milli * 1000);
}

void setVoltage(double VOLTAGE, int i2c_handle) {
  uint16_t voltageBinary = 0;
  voltageBinary = (uint16_t)((VOLTAGE / 5.0) * 16383.0);
  voltageBinary = voltageBinary << 2;
  sendCommand(CODE_LOAD, voltageBinary, i2c_handle);
}

// Send command to DAC
void sendCommand(uint8_t COMMAND, uint16_t DATA, int i2c_handle) {
  // cout << "DATA: " << (int)DATA << endl;
  uint8_t data[3];
  data[0] = COMMAND;
  data[1] = (DATA >> 8) & 0xff;
  data[2] = (DATA << 0) & 0xff;
  if (write(i2c_handle, data, 3) != 3) {
    cerr << "Failed to write to the i2c bus." << endl;
    cerr << "Error: " << errno << endl;
  }
}

void setMotorSpeed(double rpm, int i2c_handle, bool forward) {
  if (!forward) {
    gpioWrite(DIR, PI_LOW);
  } else {
    gpioWrite(DIR, PI_HIGH);
  }
  double voltageSet = (rpm / 15000.0) * 5.0;
  setVoltage(voltageSet, i2c_handle);
}

void setMotorSpeed(double rpm, int i2c_handle) {
  if (rpm >= 0) {
    setMotorSpeed(rpm, i2c_handle, true);
  } else {
    setMotorSpeed(-rpm, i2c_handle, false);
  }
}

std::string to_precision(double number, size_t decimal_places) {
  std::stringstream ss;
  ss << std::fixed << setprecision(decimal_places) << number;
  return ss.str();
}

// void stopAndWaitForStop() {
//   setMotorSpeed(1000);
//   while (rpm >= 1100) {;
//     delay(10);
//   }
//   setMotorSpeed(0);
//   delay(500);
// }
// bool loop(std::ostream& output);
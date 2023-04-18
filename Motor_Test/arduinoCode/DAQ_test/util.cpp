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

void setMotorSpeed(double rpm, bool forward) {
  if (!forward) {
    gpioWrite(DIR_PIN, PI_LOW);
  } else {
    gpioWrite(DIR_PIN, PI_HIGH);
  }
  // double voltageSet = (rpm / 15000.0) * 5.0;
  // setVoltage(voltageSet, i2c_handle);
  const int dutycycle = rpm*(15000.0/PWM_RANGE);
  gpioPWM(PWM_PIN, dutycycle);
}

void setMotorSpeed(double rpm) {
  if (rpm >= 0) {
    setMotorSpeed(rpm, true);
  } else {
    setMotorSpeed(-rpm, false);
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
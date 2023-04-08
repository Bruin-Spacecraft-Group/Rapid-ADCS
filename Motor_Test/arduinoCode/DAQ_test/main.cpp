#include "util.hpp"
#include <iostream>
#include <pigpio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <chrono>
#include <cmath>

using namespace std;

int main() {
  // gpioTerminate();
  // gpioInitialise();
  // gpioSetMode(2, PI_INPUT);
  // gpioSetMode(3, PI_INPUT);

  int i2c_handle = 0;
  std::string filename = "/dev/i2c-1";
  if ((i2c_handle = open(filename.c_str(), O_RDWR)) < 0) {
    /* ERROR HANDLING: you can check errno to see what went wrong */
    cout << "Failed to open the i2c bus: " << errno << endl;
    exit(1);
  }
  if (ioctl(i2c_handle, I2C_SLAVE_FORCE, ADDR) < 0) {
    cout << "Failed to acquire bus access and/or talk to slave: " << errno << endl;
    /* ERROR HANDLING; you can check errno to see what went wrong */
    exit(1);
  }

  // setMotorSpeed(0, i2c_handle);
  // setMotorSpeed(15000, i2c_handle);

  cout << "Running" << endl;

  auto start = std::chrono::steady_clock::now();
  // bool flag = 1;
  int configured_rpm = 15000;
  setMotorSpeed(configured_rpm, i2c_handle);
  while (true) {
    auto cur = std::chrono::steady_clock::now();
    std::chrono::duration<double> t = cur - start;

    // if (t.count() >= 5) {
    //   start = cur;
    //   // flag = !flag;
    //   configured_rpm -= 500;
    //   if (configured_rpm >= 0) {
    //     setMotorSpeed(configured_rpm, i2c_handle);  // You can end this test manually
    //   }
    // }
    // setMotorSpeed(7500 + 7500 * std::sin(2 * M_PI * 100 * t.count() - M_PI_2),i2c_handle);
  }
  close(i2c_handle);
  // gpioTerminate();
}
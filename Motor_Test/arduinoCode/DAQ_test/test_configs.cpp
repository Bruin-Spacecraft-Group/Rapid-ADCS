#include "test_configs.hpp"
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <pigpio.h>
#include <sys/ioctl.h>
#include <cmath>
#include <iomanip>
#include <string>
#include "util.hpp"

using namespace std;
using namespace fifolib::generic;

falling Motor_test::callback{};

Motor_test::Motor_test() : rpm(0), configured_rpm(0), start(), cur() {
  gpioInitialise();

  // Setup motor pins
  gpioSetMode(DIR_PIN, PI_OUTPUT);
  gpioSetMode(PWM_PIN, PI_ALT0);
  gpioSetPWMrange(PWM_PIN, 15000);

  setMotorSpeed(0);
}

void Motor_test::setup(std::ostream& output) {
  callback.setup(output, this);

  gpioSetISRFunc(
      INTERRUPT_PIN, FALLING_EDGE, 0,
      [](int gpio, int level, uint32_t tick) { callback(gpio, level, tick); });

  auto tmp = chrono::steady_clock::now();
  start = tmp;
  cur = tmp;
}

void Motor_test::setRPM(double rpm) {
  this->rpm = rpm;
}

void Motor_test::stopAndWaitForStop() const {
  // setMotorSpeed(1000);
  // while (rpm >= 1100) {
  //   delay(10);
  // }
  // setMotorSpeed(0);
  // delay(500);
  setMotorSpeed(0);
}

chrono::time_point<chrono::steady_clock> Motor_test::getStartTime() const {
  return start;
}

chrono::time_point<chrono::steady_clock> Motor_test::getCurTime() const {
  return cur;
}

std::string Motor_test::test_data() const {
  return "";
}

Motor_test::~Motor_test() {
  setMotorSpeed(0);
  gpioSetISRFunc(INTERRUPT_PIN, FALLING_EDGE, 0, NULL);
  gpioTerminate();
}

falling::falling()
    : output(nullptr), motor_test(nullptr), prev(), cycleIndex(0) {}

void falling::setup(std::ostream& output, Motor_test* motor_test) {
  this->output = &output;
  this->motor_test = motor_test;
  prev = motor_test->getStartTime();
}

void falling::operator()(int gpio, int level, uint32_t tick) {
  cycleIndex = cycleIndex + 1;
  if (cycleIndex >= TICKS_FOR_AVERAGE) {
    cycleIndex = 0;
    auto cur = motor_test->getCurTime();
    chrono::microseconds microSec =
        chrono::duration_cast<chrono::microseconds>(cur - prev);
    chrono::duration<double> timeSec = cur - motor_test->getStartTime();

    prev = cur;
    double rpm = TICKS_FOR_AVERAGE * 10.0 / (microSec.count()/1'000'000.0);
    motor_test->setRPM(rpm);

    const string msg = to_precision(timeSec.count(), 4) + ", " +
                       to_precision(rpm, 4) + ", " + motor_test->test_data();
    *output << msg << endl;
  }
}

falling::~falling() {}

// 1. Speed vs torque
// 2. Speed control precision
// 3. Max speed
// 4. Current vs continuous speed
// 5. Frequency response
// 6. Custom

bool Speed_v_torque::loop() {
  cur = chrono::steady_clock::now();

  configured_rpm = 0;
  if (!cycle_started) {
    setMotorSpeed(15000);
    configured_rpm = 15000;
    cycle_started = true;
  } else if (rpm >= 14700) {
    ++cycles;
    stopAndWaitForStop();
    cycle_started = false;
    return cycles < 5;
  }
  return true;
}

std::string Speed_v_torque::test_data() const {
  return to_precision(configured_rpm, 4);
}

bool Speed_control_precision::loop() {
  cur = chrono::steady_clock::now();

  if (!cycle_started) {
    configured_rpm += 500;  // Go in increments of 500 rpm
    if (configured_rpm > 15000) {
      configured_rpm = 0;
      stopAndWaitForStop();
      return false;
    }
    setMotorSpeed(configured_rpm);
    cycle_started = true;
    t1 = chrono::steady_clock::now();
  } else {  // 5 seconds to wait for motor velocity to converge
    // auto t2 = chrono::steady_clock::now();
    std::chrono::duration<double> t = getCurTime() - t1;
    if (t.count() >= 5) {
      cycle_started = false;
    }
  }  // else, do nothing. Let test run
  return true;
}

std::string Speed_control_precision::test_data() const {
  return to_precision(configured_rpm, 4);
}

bool Max_speed::loop() {
  cur = chrono::steady_clock::now();
  std::chrono::duration<double> t = getCurTime() - getStartTime();

  setMotorSpeed(15000);  // You can end this test manually
  configured_rpm = 15000;
  if (t.count() >= 10.0) {
    return false;
  }
  return true;
}

std::string Max_speed::test_data() const {
  return to_precision(configured_rpm, 4);
}

bool Current_draw::loop() {
  cur = chrono::steady_clock::now();

  is_valid = false;

  configured_rpm += 500;  // Go in increments of 500 rpm
  if (configured_rpm > 15000) {
    configured_rpm = 0;
    stopAndWaitForStop();
    return false;
  }
  setMotorSpeed(configured_rpm);
  delay(1500);  // Wait for things to settle
  cout << "Current draw (in Amps): ";
  cin >> current_draw;

  is_valid = true;

  delay(1500);

  return true;
}

std::string Current_draw::test_data() const {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3) << current_draw;
  std::string current_fixed_decimal = ss.str();
  return to_precision(configured_rpm, 4) + ", " + current_fixed_decimal + ", " +
         std::to_string(is_valid);
}

bool Frequency_response::loop() {
  cur = chrono::steady_clock::now();
  if (freq == 0) {
    freq = FREQUENCY_START;
    t1 = getCurTime();
  }
  if (freq > MAX_FREQUENCY) {
    return false;
  }

  std::chrono::duration<double> t_ = getCurTime() - t1;
  double t = t_.count();

  double time_for_periods = 5 / freq;  // # of periods/frequency (hz)
  if (t >= time_for_periods) {
    t = 0;
    freq += FREQENCY_STEP;
    t1 = getCurTime();
    if (freq >= 2) {
      return false;
    }
  }

  // TODO: Change code so you don't get discontinuous sine
  configured_rpm = 7500 + 7500 * std::sin(2 * M_PI * freq * t - M_PI_2);
  setMotorSpeed(configured_rpm);

  return true;
}

std::string Frequency_response::test_data() const {
  return to_precision(configured_rpm, 4) + ", " + 
         to_precision(freq, 4);
}

bool Custom_test::loop() {
  delay(10);
  count += 1;
  if (count > 100) {
    return false;
    //      flop *= -1;
  }
  double configured_rpm = -5000 * flop;
  setMotorSpeed(configured_rpm);
  return true;
}
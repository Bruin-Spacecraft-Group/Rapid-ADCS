#include "test_configs.hpp"
#include "util.hpp"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <pigpio.h>
#include <sys/ioctl.h>
#include <cmath>
#include <iomanip>
#include <string>
#include <errno.h>

using namespace std;
using namespace fifolib::generic;

falling Motor_test::callback{};

Motor_test::Motor_test(bool suppress_lost_messages, size_t buffer_size)
    : reader(nullptr),
      buffer(),
      rpm(0),
      start(),
      cur(),
      suppress_lost_messages(suppress_lost_messages),
      buffer_size(buffer_size),
      i2c_handle(0) {
  gpioInitialise();

  // Setup motor pins
  gpioSetMode(DIR, PI_OUTPUT);

  // i2c_handle = i2cOpen(I2C_BUS, ADDR, 0);
  std::string filename = "/dev/i2c-1";
  if ((i2c_handle = open(filename.c_str(), O_RDWR)) < 0) {
    /* ERROR HANDLING: you can check errno to see what went wrong */
    cout << "Failed to open the i2c bus: " << errno << endl;
    exit(1);
  }
  if (ioctl(i2c_handle, I2C_SLAVE, ADDR) < 0) {
    cout << "Failed to acquire bus access and/or talk to slave: " << errno << endl;
    /* ERROR HANDLING; you can check errno to see what went wrong */
    exit(1);
  }

  setMotorSpeed(0);
}

void Motor_test::setup() {
  reader =
      open_reader<sizeof(DataSample)>(*callback.setup(this, buffer_size), 1);

  gpioSetISRFunc(
      GPIO_DAC, FALLING_EDGE, 0,
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

bool Motor_test::suppressLostMessages() const {
  return suppress_lost_messages;
}

void Motor_test::setMotorSpeed(double rpm) const {
  ::setMotorSpeed(rpm, i2c_handle);
}

Motor_test::~Motor_test() {
  setMotorSpeed(0);
  delete reader;
  close(i2c_handle);
  gpioSetISRFunc(GPIO_DAC, FALLING_EDGE, 0, NULL);
  gpioTerminate();
}

falling::falling()
    : writer(nullptr), motor_test(nullptr), prev(), cycleIndex(0) {}

const GenericFifoWriter<sizeof(DataSample)>* falling::setup(
    Motor_test* motor_test,
    size_t buffer_size) {
  writer = init_writer<sizeof(DataSample)>(buffer_size);
  this->motor_test = motor_test;
  prev = motor_test->getStartTime();
  return writer;
}

void falling::operator()(int gpio, int level, uint32_t tick) {
  cycleIndex = cycleIndex + 1;
  if (cycleIndex >= TICKS_FOR_AVERAGE) {
    cycleIndex = 0;
    auto cur = motor_test->getCurTime();
    std::chrono::duration<double> microSec = cur - prev;
    std::chrono::duration<double> timeSec = cur - motor_test->getStartTime();

    prev = cur;
    double rpm = TICKS_FOR_AVERAGE * 10.0 / microSec.count();
    motor_test->setRPM(rpm);

    const DataSample sample = {timeSec.count(), rpm};  // {timeSec, rpm}

    writer->try_write(sample);
    std::size_t failedWrites = writer->getFailedWrites();
    if (!motor_test->suppressLostMessages() && failedWrites % 10 == 0 &&
        failedWrites > 0) {
      cerr << "Lost: " << failedWrites << " messages" << endl;
    }
  }
}

falling::~falling() {
  delete writer;
}

// 1. Speed vs torque
// 2. Speed control precision
// 3. Max speed
// 4. Current vs continuous speed
// 5. Frequency response
// 6. Custom

bool Speed_v_torque::loop(std::ostream& output) {
  cur = chrono::steady_clock::now();

  double configured_rpm = 0;
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

  while (reader->try_read(buffer)) {
    const string msg = to_precision(buffer.timeSec, 4) + ", " +
                       to_precision(buffer.rpm, 4) + ", " +
                       to_precision(configured_rpm, 4);
    output << msg << endl;
  }
  return true;
}

bool Speed_control_precision::loop(std::ostream& output) {
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

  while (reader->try_read(buffer)) {
    const string msg = to_precision(buffer.timeSec, 4) + ", " +
                       to_precision(buffer.rpm, 4) + ", " +
                       to_precision(configured_rpm, 4);
    output << msg << endl;
  }
  return true;
}

bool Max_speed::loop(std::ostream& output) {
  cur = chrono::steady_clock::now();
  std::chrono::duration<double> t = getCurTime() - getStartTime();

  setMotorSpeed(15000);  // You can end this test manually
  double configured_rpm = 15000;
  if (t.count() >= 10.0) {
    return false;
  }

  while (reader->try_read(buffer)) {
    const string msg = to_precision(buffer.timeSec, 4) + ", " +
                       to_precision(buffer.rpm, 4) + ", " +
                       to_precision(configured_rpm, 4);
    output << msg << endl;
  }
  return true;
}

bool Current_draw::loop(std::ostream& output) {
  cur = chrono::steady_clock::now();

  suppress_lost_messages = true;
  configured_rpm += 500;  // Go in increments of 500 rpm
  if (configured_rpm > 15000) {
    configured_rpm = 0;
    stopAndWaitForStop();
    return false;
  }
  setMotorSpeed(configured_rpm);
  delay(3000);  // Wait for things to settle
  cout << "Current draw (in Amps): ";
  float current;
  cin >> current;

  std::stringstream ss;
  ss << std::fixed << std::setprecision(3) << current;
  std::string current_fixed_decimal = ss.str();

  reader->clear_buffer();
  delay(1500);

  while (reader->try_read(buffer)) {
    const string msg =
        to_precision(buffer.timeSec, 4) + ", " + to_precision(buffer.rpm, 4) +
        ", " + to_precision(configured_rpm, 4) + ", " + current_fixed_decimal;
    output << msg << endl;
  }
  return true;
}

bool Frequency_response::loop(std::ostream& output) {
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
  double configured_rpm = 7500 + 7500 * std::sin(2 * M_PI * freq * t - M_PI_2);
  setMotorSpeed(configured_rpm);

  while (reader->try_read(buffer)) {
    const string msg =
        to_precision(buffer.timeSec, 4) + ", " + to_precision(buffer.rpm, 4) +
        ", " + to_precision(configured_rpm, 4) + ", " + to_precision(freq, 4);
    output << msg << endl;
  }
  return true;
}

bool Custom_test::loop(std::ostream& output) {
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
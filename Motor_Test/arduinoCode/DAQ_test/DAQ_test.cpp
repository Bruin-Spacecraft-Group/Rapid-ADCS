/*
 * README
 * For option 4 to work, make sure you set your serial monitor to "no line ending."
 */


#include <pigpio.h>
#include <cxxopts.hpp>
#include <iostream>
#include <string>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>
#include <fstream>
#include <filesystem>
#include <cmath>
#include "GenericFifo.hpp"

using std::cout;
using std::endl;
using std::cin;
using std::cerr;
using namespace fifolib::generic;
using namespace std::string_literals;

// DAC COMMANDS
#define ADDR 0b0011101 // TODO: TO BE CHECKED
#define CODE_LOAD 0x01
#define USER_CONFIG 0b00001000
#define I2C_BUS 0

// MOTOR PINS
const int DIR = 12;

// DAC PINS
const int GPIO_DAC = 1;

volatile long pwm_value = 0;
volatile long prev_time = 0;
volatile double rpm = 0;
volatile double timeSec = 0;
volatile double microSec = 0;
int delayCycles = 6;
volatile int cycleIndex = 0;
double initialTime = 0;

// motor speed detect
volatile double prev_rpm = 0;
volatile int count = 0;
volatile double totalRPM = 0;
int maxCount = 10;
bool highSpeed = 0;
volatile long acc_t1 = 0;
volatile double accel = 0;
volatile double cur_time = 0;
double tmp = 0;
long t1 = 0;

int test_config = 0;
bool suppress_lost_messages = false;
std::string data_dir;

int i2c_handle;
clockid_t clock_id;

struct DataSample {
  const double timeSec;
  const double rpm;
};

GenericFifoWriter<sizeof(DataSample)>* writer;
GenericFifoReader<sizeof(DataSample)>* reader;

std::ostream* output_p;

int main(int argc, char** argv) {
	cxxopts::Options options("Motor Characterization Test", "Measures the performance characteristics of a DC brushless motor.");
	options.add_options()
			("t,test", "Variant of the test. (1-6)", cxxopts::value<int>()->default_value("0"))
			("s,suppress", "Suppress lost measurements")
			("d,data-dir", "Directory for gathered data. If not specified, uses present working directory", cxxopts::value<std::string>()->default_value(std::filesystem::current_path().string() + "/data"))
			("b,buffer-size", "Size of the buffer for measurements", cxxopts::value<std::size_t>()->default_value("8"))
			("no-save", "If used, gathered data is not stored and does not override any previously gathered data. Instead, data is simply printed to cout.")
			("h,help", "Print usage")
			;

	auto result = options.parse(argc, argv);

  if (result.count("help")) {
    cout << options.help() << endl;
    exit(0);
  }

  test_config = result["test"].as<int>();
	suppress_lost_messages = result["suppress"].as<bool>();
	data_dir = result["data-dir"].as<std::filesystem::path>();
	const std::size_t buffer_size = result["buffer-size"].as<std::size_t>();
	const bool save = !result["no-save"].as<bool>();

  writer = init_writer<sizeof(DataSample)>(buffer_size);
  reader = open_reader<sizeof(DataSample)>(*writer, 1);

  std::ofstream of;
  if (save) {
    of.open(data_dir);
    output_p = &of;
  } else {
    output_p = &cout;
  }

  parseTestConfig(test_config);

  clock_getcpuclockid(getpid(), &clock_id);

  gpioInitialise();

  // Setup motor pins
  gpioSetMode(DIR, PI_OUTPUT);

  // i2c_handle = i2cOpen(I2C_BUS, ADDR, 0);
  std::string filename = "/dev/i2c-1";
  if ((i2c_handle = open(filename.c_str(), O_RDWR)) < 0) {
    /* ERROR HANDLING: you can check errno to see what went wrong */
    cout << "Failed to open the i2c bus" << endl;
    exit(1);
  }
  if (ioctl(i2c_handle, I2C_SLAVE, ADDR) < 0) {
    cout << "Failed to acquire bus access and/or talk to slave." << endl;
    /* ERROR HANDLING; you can check errno to see what went wrong */
    exit(1);
  }

  setMotorSpeed(0);

  gpioSetISRFunc(GPIO_DAC, FALLING_EDGE, 0, falling);

  prev_time = micros();
  t1 = micros();
  acc_t1 = t1;
  initialTime = -1;

  while (loop(*output_p)) {}

  close(i2c_handle);
  if (of.is_open()) of.close();
}

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

long micros() {
  timespec t;
  clock_gettime(clock_id, &t);
  return t.tv_nsec*1000;
}

void delay(int milli) {
  usleep(milli*1000);
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
  // Wire.beginTransmission(ADDR);
  // Wire.write(data, 3);
  // Wire.endTransmission();

  // i2cWriteBlockData(i2c_handle, COMMAND, reinterpret_cast<char*>(data + 1), 3);
  if (write(i2c_handle, data, 3) != 3) {
    /* ERROR HANDLING: i2c transaction failed */
    cerr << "Failed to write to the i2c bus." << endl;
    cerr << "Error: " << errno << endl;
  }
}

// Set motor speed
void setMotorSpeed(double rpm, bool forward){
  if(!forward){
    // digitalWrite(DIR, LOW);
    gpioWrite(DIR, PI_LOW);
  }
  else{
    // digitalWrite(DIR, HIGH);
    gpioWrite(DIR, PI_HIGH);
  }
  double voltageSet = (rpm / 15000.0) * 5.0;
  setVoltage(voltageSet);
}

void setMotorSpeed(double rpm) {
  if (rpm >= 0) { setMotorSpeed(rpm, true); }
  else { setMotorSpeed(-rpm, false); }
}

void falling(int gpio, int level, uint32_t tick) {
  cycleIndex++;
  if(cycleIndex >= delayCycles){
    cycleIndex = 0;
    microSec = micros();
    pwm_value = microSec - prev_time;
    timeSec = (((microSec + prev_time / 2.0)) / 1000000.0);
    prev_time = microSec;
    rpm = delayCycles * 10.0 / (((double) pwm_value) / 1000000.0);

    if (initialTime == -1) { initialTime = timeSec; }
    const DataSample sample = {timeSec - initialTime, rpm}; // {timeSec, rpm}
    writer->try_write(sample);
    std::size_t failedWrites = writer->getFailedWrites();
    if (failedWrites % 10 == 0 && failedWrites > 0 && !suppress_lost_messages) {
      cerr << "Lost: "<< failedWrites << " messages" << endl;
    }
  }
}

double configured_rpm = 0; // consider this min rpm
bool change_rpm = true;
bool cycle_started = false;

void stopAndWaitForStop() {
  setMotorSpeed(1000);
  while (rpm >= 1100) {;
    delay(10);
  }
  setMotorSpeed(0);
  delay(500);
}

DataSample buffer{};
std::string suffix("");
double freq = 0.2;
double delta_t = 0;
double index = 0;
int flop = 1;

std::string to_precision(double number, size_t decimal_places) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(decimal_places) << number;
  return ss.str();
}

bool loop(std::ostream& output) {

  if (test_config == 1) { // Test maximal speed/torque curve
    if (!cycle_started) {
      setMotorSpeed(15000);
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
      setMotorSpeed(configured_rpm);
      cycle_started = true;
      t1 = micros();
    } else if ((micros() - t1) / 1000000.0 >= 5.0) { // 5 seconds to wait for motor velocity to converge
      cycle_started = false;
      return;
    } // else, do nothing. Let test run
  } else if (test_config == 3) { // Vroom vroom bitch
    setMotorSpeed(15000); // You can end this test manually
    configured_rpm = 15000;
  } else if (test_config == 4) { // You'll need to manually record current draw from power supply
    suppress_lost_messages = true;
    configured_rpm += 500; // Go in increments of 500 rpm
    if (configured_rpm > 15000) {
      configured_rpm = 0;
      stopAndWaitForStop();
      return;
    }
    setMotorSpeed(configured_rpm);
    delay(3000); // Wait for things to settle
    cout << "Current draw (in Amps): ";
    float current;
    cin >> current;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << current;
    std::string current_fixed_decimal = ss.str();

    suffix = ", "s + current_fixed_decimal;

    reader->clear_buffer();
    delay(1500);
    // return;
  } else if (test_config == 5) {
    double t2 = micros();
    delta_t += (t2 - t1)/1000000.00;
    t1 = t2;

    configured_rpm = 7500 + 7500*std::sin(2*M_PI*delta_t*freq);
    setMotorSpeed(configured_rpm);
    suffix = ", "s + std::to_string(freq);

    if (delta_t >= 5) {
      freq += 0.1;
      delta_t = 0;
    }
  } else if (test_config == 6) {
    delay(10);
    index += 1;
    if(index > 100){
      index = 0;
//      flop *= -1;
    }
    configured_rpm = -5000 * flop;
    setMotorSpeed(configured_rpm);
  }

  // delay(10);
//  Serial.print("Messages to be printed: ");
//  Serial.println(reader->num_messages_to_read());
  while (reader->try_read(buffer)) {
    const std::string msg = to_precision(buffer.timeSec, 4) + ", " + to_precision(buffer.rpm, 4) + ", " + std::to_string(configured_rpm) + suffix;
    output << msg << std::endl;
    
//    Serial.print(buffer.timeSec);
//    Serial.print(", ");
//    Serial.print(configured_rpm);
//    Serial.print(", ");
//    Serial.println();
  }
  return true;
}

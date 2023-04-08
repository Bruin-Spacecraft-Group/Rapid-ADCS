#pragma once

#include <stdint.h>
#include <chrono>
#include <cstdlib>
#include <iostream>

// DAC COMMANDS
constexpr const uint8_t ADDR = 0b0011101;  // TODO: TO BE CHECKED
constexpr const uint8_t CODE_LOAD = 0x01;
constexpr const uint8_t USER_CONFIG = 0b00001000;
constexpr const std::size_t I2C_BUS = 0;

// MOTOR PINS
constexpr const int DIR = 4;

// DAC PINS
constexpr const int GPIO_DAC = 17;

constexpr const int TICKS_FOR_AVERAGE = 1;
constexpr const double MAX_FREQUENCY = 2;
constexpr const double FREQUENCY_START = 0.2;
constexpr const double FREQENCY_STEP = 0.2;

struct DataSample {
  const double timeSec = 0;
  const double rpm = 0;
};

void parseTestConfig(int& test_config);
long micros();
void delay(int milli);
void setVoltage(double VOLTAGE, int i2c_handle);
void sendCommand(uint8_t COMMAND, uint16_t DATA, int i2c_handle);
void setMotorSpeed(double rpm, int i2c_handle, bool forward);
void setMotorSpeed(double rpm, int i2c_handle);
std::string to_precision(double number, size_t decimal_places);
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

// MOTOR PIN
constexpr const int DIR_PIN = 4;

// INTERRUPT PIN
constexpr const int INTERRUPT_PIN = 17;

// PWM PIN
constexpr const int PWM_PIN = 12;

constexpr const int PWM_RANGE = 15000;

constexpr const int TICKS_FOR_AVERAGE = 6;
constexpr const double MAX_FREQUENCY = 2;
constexpr const double FREQUENCY_START = 0.2;
constexpr const double FREQENCY_STEP = 0.2;

void parseTestConfig(int& test_config);
long micros();
void delay(int milli);
void setMotorSpeed(double rpm, bool forward);
void setMotorSpeed(double rpm);
std::string to_precision(double number, size_t decimal_places);
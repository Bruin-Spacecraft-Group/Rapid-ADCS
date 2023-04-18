#pragma once

#include <atomic>
#include <chrono>
#include <ostream>
#include <string>
#include <utility>
#include "GenericFifo.hpp"
#include "util.hpp"

struct falling;

class Motor_test {
 private:
  template <std::size_t max_message_size>
  using GenericFifoReader =
      fifolib::generic::GenericFifoReader<max_message_size>;
  using steady_clock = std::chrono::steady_clock;
  template <typename T>
  using time_point = std::chrono::time_point<T>;

 protected:
  // This needs to be a static, as there is no other way to use it as a callback
  // function, on account of pigpio being a function library. gpioSetISRFunc
  // requires a pure C function, and an instance method can not be used there
  static falling callback;

  std::atomic<float> rpm;
  std::atomic<float> configured_rpm;
  std::atomic<time_point<steady_clock>> start; // Undefined behavior?
  std::atomic<time_point<steady_clock>> cur;

  Motor_test();

 public:
  void setup(std::ostream& output);
  void setRPM(double rpm);
  virtual bool loop() = 0;
  void stopAndWaitForStop() const;
  time_point<steady_clock> getStartTime() const;
  time_point<steady_clock> getCurTime() const;
  virtual std::string test_data() const;
  virtual ~Motor_test();
};

struct falling {
  std::ostream* output;
  Motor_test* motor_test;
  std::chrono::time_point<std::chrono::steady_clock> prev;
  size_t cycleIndex;

  falling();
  void setup(std::ostream& output, Motor_test* motor_test);
  void operator()(int gpio, int level, uint32_t tick);
  ~falling();
};

// 1. Speed vs torque
// 2. Speed control precision
// 3. Max speed
// 4. Current vs continuous speed
// 5. Frequency response
// 6. Custom

class Speed_v_torque : public Motor_test {
 private:
  bool cycle_started;
  int cycles;

 public:
  template <typename... Args>
  Speed_v_torque(Args&&... args)
      : Motor_test(std::forward<decltype(args)>(args)...),
        cycle_started(false),
        cycles(0) {}
  virtual bool loop();
  virtual std::string test_data() const;
};

class Speed_control_precision : public Motor_test {
 private:
  bool cycle_started;
  std::chrono::time_point<std::chrono::steady_clock> t1;

 public:
  template <typename... Args>
  Speed_control_precision(Args&&... args)
      : Motor_test(std::forward<decltype(args)>(args)...),
        cycle_started(false),
        t1() {}
  virtual bool loop();
  virtual std::string test_data() const;
};

class Max_speed : public Motor_test {
 public:
  template <typename... Args>
  Max_speed(Args&&... args)
      : Motor_test(std::forward<decltype(args)>(args)...) {}
  virtual bool loop();
  virtual std::string test_data() const;
};

class Current_draw : public Motor_test {
 private:
  float current_draw;
  bool is_valid;

 public:
  template <typename... Args>
  Current_draw(Args&&... args)
      : Motor_test(std::forward<decltype(args)>(args)...), current_draw(0), is_valid(false) {}
  virtual bool loop();
  virtual std::string test_data() const;
};

class Frequency_response : public Motor_test {
 private:
  std::chrono::time_point<std::chrono::steady_clock> t1;
  float freq;

 public:
  template <typename... Args>
  Frequency_response(Args&&... args)
      : Motor_test(std::forward<decltype(args)>(args)...), t1(), freq(0) {}
  virtual bool loop();
  virtual std::string test_data() const;
};

class Custom_test : public Motor_test {
 private:
  int count;
  bool flop;

 public:
  template <typename... Args>
  Custom_test(Args&&... args)
      : Motor_test(std::forward<decltype(args)>(args)...), count(0), flop(1) {}
  virtual bool loop();
};
#pragma once

#include <atomic>
#include <chrono>
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

  GenericFifoReader<sizeof(DataSample)>* reader;
  DataSample buffer;
  std::atomic<double> rpm;
  std::atomic<time_point<steady_clock>> start;
  std::atomic<time_point<steady_clock>> cur;
  std::atomic<bool> suppress_lost_messages;
  std::size_t buffer_size;
  std::size_t i2c_handle;

  Motor_test(bool suppress_lost_messages, size_t buffer_size);

 public:
  void setup();
  void setRPM(double rpm);
  virtual bool loop(std::ostream& output) = 0;
  void stopAndWaitForStop() const;
  time_point<steady_clock> getStartTime() const;
  time_point<steady_clock> getCurTime() const;
  bool suppressLostMessages() const;
  void setMotorSpeed(double rpm) const;
  virtual ~Motor_test();
};

struct falling {
  fifolib::generic::GenericFifoWriter<sizeof(DataSample)>* writer;
  Motor_test* motor_test;
  std::chrono::time_point<std::chrono::steady_clock> prev;
  size_t cycleIndex;

  falling();
  const fifolib::generic::GenericFifoWriter<sizeof(DataSample)>* setup(
      Motor_test* motor_test,
      size_t buffer_size);
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
  virtual bool loop(std::ostream& output);
};

class Speed_control_precision : public Motor_test {
 private:
  bool cycle_started;
  int configured_rpm;
  std::chrono::time_point<std::chrono::steady_clock> t1;

 public:
  template <typename... Args>
  Speed_control_precision(Args&&... args)
      : Motor_test(std::forward<decltype(args)>(args)...),
        cycle_started(false),
        configured_rpm(0),
        t1() {}
  virtual bool loop(std::ostream& output);
};

class Max_speed : public Motor_test {
 public:
  template <typename... Args>
  Max_speed(Args&&... args)
      : Motor_test(std::forward<decltype(args)>(args)...) {}
  virtual bool loop(std::ostream& output);
};

class Current_draw : public Motor_test {
 private:
  int configured_rpm;

 public:
  template <typename... Args>
  Current_draw(Args&&... args)
      : Motor_test(std::forward<decltype(args)>(args)...), configured_rpm(0) {}
  virtual bool loop(std::ostream& output);
};

class Frequency_response : public Motor_test {
 private:
  std::chrono::time_point<std::chrono::steady_clock> t1;
  double freq;

 public:
  template <typename... Args>
  Frequency_response(Args&&... args)
      : Motor_test(std::forward<decltype(args)>(args)...), t1(), freq(0) {}
  virtual bool loop(std::ostream& output);
};

class Custom_test : public Motor_test {
 private:
  int count;
  bool flop;

 public:
  template <typename... Args>
  Custom_test(Args&&... args)
      : Motor_test(std::forward<decltype(args)>(args)...), count(0), flop(1) {}
  virtual bool loop(std::ostream& output);
};
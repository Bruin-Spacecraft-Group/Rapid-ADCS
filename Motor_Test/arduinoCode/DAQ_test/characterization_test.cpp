#include <filesystem>
#include <fstream>
#include <iostream>
#include <pigpio.h>
#include "GenericFifo.hpp"
#include "cxxopts.hpp"
#include "test_configs.hpp"
#include "util.hpp"

using namespace std;

int main(int argc, char** argv) {
  gpioInitialise();
  setMotorSpeed(0);

  cxxopts::Options options(
      "Motor Characterization Test",
      "Measures the performance characteristics of a DC brushless motor.");
  options.add_options()("t,test", "Variant of the test. (1-6)",
                        cxxopts::value<int>()->default_value("0"))(
      "d,data-dir",
      "Directory for gathered data. If not specified, uses present working "
      "directory",
      cxxopts::value<std::string>()->default_value(
          std::filesystem::current_path().string() + "/data"))(
      "save",
      "If used, gathered data is not stored and does not override any "
      "previously gathered data. Instead, data is simply printed to cout.")(
      "h,help", "Print usage");

  auto result = options.parse(argc, argv);

  if (result.count("help")) {
    cout << options.help() << endl;
    exit(0);
  }

  int test_config = result["test"].as<int>();
  std::filesystem::path data_dir = result["data-dir"].as<std::string>();
  const bool save = result["save"].as<bool>();

  std::ostream* output_p;
  std::ofstream of;
  if (save) {
    of.open(data_dir);
    output_p = &of;
  } else {
    output_p = &cout;
  }

  parseTestConfig(test_config);

  Motor_test* mt;
  switch (test_config) {
    case 1:
      mt = new Speed_v_torque();
      break;
    case 2:
      mt = new Speed_control_precision();
      break;
    case 3:
      mt = new Max_speed();
      break;
    case 4:
      mt = new Current_draw();
      break;
    case 5:
      mt = new Frequency_response();
      break;
    default:
      mt = new Custom_test();
      break;
  }

  // *output_p << "banana" << std::endl;
  mt->setup(*output_p);
  while (mt->loop()) {
  }

  if (of.is_open())
    of.close();

  delete mt;
  gpioTerminate();
}
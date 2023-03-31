#include <filesystem>
#include <fstream>
#include "GenericFifo.hpp"
#include "cxxopts.hpp"
#include "test_configs.hpp"
#include "util.hpp"

using namespace std;

int main(int argc, char** argv) {
  cxxopts::Options options(
      "Motor Characterization Test",
      "Measures the performance characteristics of a DC brushless motor.");
  options.add_options()("t,test", "Variant of the test. (1-6)",
                        cxxopts::value<int>()->default_value("0"))(
      "s,suppress", "Suppress lost measurements")(
      "d,data-dir",
      "Directory for gathered data. If not specified, uses present working "
      "directory",
      cxxopts::value<std::string>()->default_value(
          std::filesystem::current_path().string() + "/data"))(
      "b,buffer-size", "Size of the buffer for measurements",
      cxxopts::value<std::size_t>()->default_value("8"))(
      "no-save",
      "If used, gathered data is not stored and does not override any "
      "previously gathered data. Instead, data is simply printed to cout.")(
      "h,help", "Print usage");

  auto result = options.parse(argc, argv);

  if (result.count("help")) {
    cout << options.help() << endl;
    exit(0);
  }

  int test_config = result["test"].as<int>();
  const bool suppress_lost_messages = result["suppress"].as<bool>();
  std::filesystem::path data_dir =
      result["data-dir"].as<std::filesystem::path>();
  const std::size_t buffer_size = result["buffer-size"].as<std::size_t>();
  const bool save = !result["no-save"].as<bool>();

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
      mt = new Speed_v_torque(suppress_lost_messages, buffer_size);
      break;
    case 2:
      mt = new Speed_control_precision(suppress_lost_messages, buffer_size);
      break;
    case 3:
      mt = new Max_speed(suppress_lost_messages, buffer_size);
      break;
    case 4:
      mt = new Current_draw(suppress_lost_messages, buffer_size);
      break;
    case 5:
      mt = new Frequency_response(suppress_lost_messages, buffer_size);
      break;
    default:
      mt = new Custom_test(suppress_lost_messages, buffer_size);
      break;
  }

  while (mt->loop(*output_p)) {}

  if (of.is_open())
    of.close();

  delete mt;
}
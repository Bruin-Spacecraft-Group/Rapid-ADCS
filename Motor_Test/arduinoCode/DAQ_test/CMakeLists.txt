cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(motor_daq VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(characterization_test util.cpp test_configs.cpp characterization_test.cpp)

message(NOTICE "Cmake build type: ${CMAKE_BUILD_TYPE}")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	message(NOTICE "Debug mode: enabling -O0")
  target_compile_options(characterization_test PUBLIC -fstandalone-debug -O0 -march=native -DNATIVE)
  target_compile_options(characterization_test PUBLIC -fstandalone-debug -O0 -march=native -DNATIVE)
  target_compile_options(characterization_test PUBLIC -fstandalone-debug -O0 -march=native -DNATIVE)

endif()

if (CMAKE_BUILD_TYPE STREQUAL "Release")
	message(NOTICE "Release mode: enabling -O3 -march=native -DNDEBUG")
	target_compile_options(characterization_test PUBLIC -O3 -march=native -DNATIVE)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")

target_link_libraries(characterization_test pigpio)
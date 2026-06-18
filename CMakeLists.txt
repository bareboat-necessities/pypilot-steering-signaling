cmake_minimum_required(VERSION 3.16)
project(pypilot_steering_signaling LANGUAGES CXX)

option(PYPILOT_STEERING_SIGNALING_BUILD_TESTS "Build tests" ON)
option(PYPILOT_STEERING_SIGNALING_BUILD_EXAMPLES "Build examples" ON)

add_library(pypilot_steering_signaling INTERFACE)
target_include_directories(pypilot_steering_signaling INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(pypilot_steering_signaling INTERFACE cxx_std_11)

if(PYPILOT_STEERING_SIGNALING_BUILD_EXAMPLES)
  add_executable(steering_signaling_smoke examples/linux/steering_signaling_smoke.cpp)
  target_link_libraries(steering_signaling_smoke PRIVATE pypilot_steering_signaling)
endif()

if(PYPILOT_STEERING_SIGNALING_BUILD_TESTS)
  enable_testing()
  add_executable(test_rudder_calibration tests/test_rudder_calibration.cpp)
  target_link_libraries(test_rudder_calibration PRIVATE pypilot_steering_signaling)
  add_test(NAME test_rudder_calibration COMMAND test_rudder_calibration)
endif()

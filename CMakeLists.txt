cmake_minimum_required(VERSION 3.16)
project(pypilot_steering_signaling LANGUAGES CXX)

option(PYPILOT_STEERING_SIGNALING_BUILD_TESTS "Build tests" ON)
option(PYPILOT_STEERING_SIGNALING_BUILD_EXAMPLES "Build examples" ON)
set(PYPILOT_SERVO_PROTOCOL_DIR "" CACHE PATH "Path to pypilot-servo-protocol checkout")
set(PYPILOT_SYSLIB_DIR "" CACHE PATH "Path to pypilot-syslib checkout")

if(NOT PYPILOT_SERVO_PROTOCOL_DIR)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/deps/pypilot-servo-protocol/src/pypilot_servo_protocol.hpp")
    set(PYPILOT_SERVO_PROTOCOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/pypilot-servo-protocol")
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-servo-protocol/src/pypilot_servo_protocol.hpp")
    set(PYPILOT_SERVO_PROTOCOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-servo-protocol")
  endif()
endif()

if(NOT PYPILOT_SYSLIB_DIR)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/deps/pypilot-syslib/src/pypilot_syslib.hpp")
    set(PYPILOT_SYSLIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/pypilot-syslib")
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-syslib/src/pypilot_syslib.hpp")
    set(PYPILOT_SYSLIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-syslib")
  endif()
endif()

if(NOT EXISTS "${PYPILOT_SERVO_PROTOCOL_DIR}/src/pypilot_servo_protocol.hpp")
  message(FATAL_ERROR "pypilot-servo-protocol checkout not found; set PYPILOT_SERVO_PROTOCOL_DIR")
endif()
if(NOT EXISTS "${PYPILOT_SYSLIB_DIR}/src/pypilot_syslib.hpp")
  message(FATAL_ERROR "pypilot-syslib checkout not found; set PYPILOT_SYSLIB_DIR")
endif()

add_library(pypilot_steering_signaling INTERFACE)
target_include_directories(pypilot_steering_signaling INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${PYPILOT_SERVO_PROTOCOL_DIR}/src
  ${PYPILOT_SYSLIB_DIR}/src)
target_compile_features(pypilot_steering_signaling INTERFACE cxx_std_11)

if(PYPILOT_STEERING_SIGNALING_BUILD_EXAMPLES)
  add_executable(steering_signaling_smoke examples/linux/steering_signaling_smoke.cpp)
  target_link_libraries(steering_signaling_smoke PRIVATE pypilot_steering_signaling)
endif()

if(PYPILOT_STEERING_SIGNALING_BUILD_TESTS)
  enable_testing()
  foreach(test_name test_rudder_calibration test_rudder_runtime test_servo_safety test_servo_protocol_bridge test_servo_runtime test_servo_config_commands)
    add_executable(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE pypilot_steering_signaling)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

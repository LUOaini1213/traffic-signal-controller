cmake_minimum_required(VERSION 3.22)
project(traffic_signal_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(TSC_BUILD_TESTS "Build the GoogleTest suite" ON)
option(TSC_BUILD_SUMO "Build the SUMO harness (needs libsumocpp from the sumo package)" OFF)
set(TSC_SANITIZE "" CACHE STRING "Sanitizer build: empty, 'address' (ASan+UBSan) or 'thread' (TSan)")

# ---------------------------------------------------------------- dependencies
find_package(Threads REQUIRED)
find_package(nlohmann_json 3.10 QUIET)
if(NOT nlohmann_json_FOUND)
  # Fallback for machines without the distro package (apt: nlohmann-json3-dev).
  include(FetchContent)
  FetchContent_Declare(json GIT_REPOSITORY https://github.com/nlohmann/json.git GIT_TAG v3.11.3 GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(json)
endif()

# ---------------------------------------------------------------- warnings and sanitizers
# Applied only to targets from this repository, never to third-party code.
add_library(tsc_warnings INTERFACE)
target_compile_options(tsc_warnings INTERFACE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wnon-virtual-dtor
  -Wold-style-cast -Woverloaded-virtual -Wnull-dereference -Werror)

add_library(tsc_sanitizers INTERFACE)
if(TSC_SANITIZE STREQUAL "address")
  target_compile_options(tsc_sanitizers INTERFACE
    -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
  target_link_options(tsc_sanitizers INTERFACE -fsanitize=address,undefined)
elseif(TSC_SANITIZE STREQUAL "thread")
  target_compile_options(tsc_sanitizers INTERFACE -fsanitize=thread -fno-omit-frame-pointer)
  target_link_options(tsc_sanitizers INTERFACE -fsanitize=thread)
elseif(NOT TSC_SANITIZE STREQUAL "")
  message(FATAL_ERROR "TSC_SANITIZE must be empty, 'address' or 'thread'")
endif()

# ---------------------------------------------------------------- core library
add_library(tsc_core
  src/types.cpp
  src/config.cpp
  src/detector_monitor.cpp
  src/sequencer.cpp
  src/actuated.cpp
  src/fixed_time.cpp
  src/safety_guard.cpp
  src/auditor.cpp
  src/webster.cpp
  src/pipeline.cpp
  src/runtime.cpp)
target_include_directories(tsc_core PUBLIC include)
target_link_libraries(tsc_core
  PUBLIC nlohmann_json::nlohmann_json Threads::Threads tsc_sanitizers
  PRIVATE tsc_warnings)

add_executable(tsc apps/tsc_cli.cpp)
target_link_libraries(tsc PRIVATE tsc_core tsc_warnings)

# ---------------------------------------------------------------- SUMO harness
if(TSC_BUILD_SUMO)
  find_path(SUMO_INCLUDE_DIR libsumo/libsumo.h PATHS /usr/include ENV SUMO_HOME PATH_SUFFIXES include)
  find_library(SUMO_LIBRARY sumocpp)
  if(NOT SUMO_INCLUDE_DIR OR NOT SUMO_LIBRARY)
    message(FATAL_ERROR "TSC_BUILD_SUMO=ON but libsumo headers or libsumocpp were not found")
  endif()
  add_executable(tsc_sumo apps/tsc_sumo.cpp)
  # SUMO's headers are third-party: mark them SYSTEM so -Werror does not apply to them.
  target_include_directories(tsc_sumo SYSTEM PRIVATE ${SUMO_INCLUDE_DIR})
  target_link_libraries(tsc_sumo PRIVATE tsc_core tsc_warnings ${SUMO_LIBRARY})
endif()

# ---------------------------------------------------------------- tests
if(TSC_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(tsc_tests
    tests/test_config.cpp
    tests/test_actuated.cpp
    tests/test_fixed_time.cpp
    tests/test_safety_guard.cpp
    tests/test_auditor.cpp
    tests/test_faults.cpp
    tests/test_webster.cpp
    tests/test_property.cpp
    tests/test_concurrency.cpp)
  target_link_libraries(tsc_tests PRIVATE tsc_core tsc_warnings GTest::gtest GTest::gtest_main)
  target_compile_definitions(tsc_tests PRIVATE TSC_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  include(GoogleTest)
  gtest_discover_tests(tsc_tests DISCOVERY_TIMEOUT 60)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME ReplayCli
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli.py
      --binary $<TARGET_FILE:tsc> --config ${CMAKE_CURRENT_SOURCE_DIR}/configs/intersection.json)
  set_tests_properties(ReplayCli PROPERTIES TIMEOUT 120)
endif()

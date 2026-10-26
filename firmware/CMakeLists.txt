cmake_minimum_required(VERSION 3.16)
project(actuator_firmware CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(BUILD_TESTS "Build the unit tests" ON)
option(USE_GTEST_LITE "Use the bundled minimal test runner instead of GoogleTest" OFF)
option(ENABLE_SANITIZERS "Build with AddressSanitizer and UBSan" OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
endif()
if(ENABLE_SANITIZERS)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

# Portable firmware core: no OS, no heap after start-up, no exceptions.
# The same files build into the ESP32 sketch.
add_library(actuator_core STATIC
  core/protocol.cpp
  core/params.cpp
  core/plant.cpp
  core/controller.cpp
  core/node.cpp
  core/slcan.cpp)
target_include_directories(actuator_core PUBLIC core)

add_executable(actuator_sim host/sim_main.cpp)
target_link_libraries(actuator_sim PRIVATE actuator_core)

if(BUILD_TESTS)
  enable_testing()
  set(TEST_SOURCES tests/test_protocol.cpp tests/test_params.cpp tests/test_drive.cpp)
  if(USE_GTEST_LITE)
    add_executable(unit_tests ${TEST_SOURCES} tests/main_lite.cpp)
    target_compile_definitions(unit_tests PRIVATE USE_GTEST_LITE)
    target_link_libraries(unit_tests PRIVATE actuator_core)
    add_test(NAME unit_tests COMMAND unit_tests)
  else()
    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
      include(FetchContent)
      FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
      set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
      set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
      FetchContent_MakeAvailable(googletest)
    endif()
    add_executable(unit_tests ${TEST_SOURCES})
    target_link_libraries(unit_tests PRIVATE actuator_core GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(unit_tests)
  endif()
endif()

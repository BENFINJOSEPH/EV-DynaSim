# EV-DynaSim C++17 simulation engine.
#
# Targets
#   evdynasim_core   static library: physics, powertrain, battery, simulator
#   evdynasim_c      shared library (.dll / .so) with the C interface, used by Python (ctypes)
#   evdynasim_cli    command line executable
#   evdynasim_tests  GoogleTest unit tests (EVDS_BUILD_TESTS=ON)
#
# Build (Windows, Developer PowerShell for VS 2022, run from the repository root):
#   cmake -S cpp -B build
#   cmake --build build --config Release
#   ctest --test-dir build -C Release --output-on-failure
cmake_minimum_required(VERSION 3.20)
project(evdynasim_cpp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# All executables and shared libraries go to build/bin (build/bin/Release with MSVC),
# so the Python wrapper finds the library in a fixed place.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(EVDS_BUILD_TESTS "Build the GoogleTest unit tests" ON)
option(EVDS_USE_SYSTEM_GTEST "Use an installed GoogleTest instead of downloading it" OFF)

function(evds_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  endif()
endfunction()

# ---- core library ------------------------------------------------------------
add_library(evdynasim_core STATIC
  src/config.cpp
  src/interp.cpp
  src/params.cpp
  src/physics.cpp
  src/powertrain.cpp
  src/battery.cpp
  src/drive_cycle.cpp
  src/simulator.cpp
)
target_include_directories(evdynasim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(evdynasim_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
evds_warnings(evdynasim_core)

# ---- C interface (shared library) ----------------------------------------------
add_library(evdynasim_c SHARED src/c_api.cpp)
target_link_libraries(evdynasim_c PRIVATE evdynasim_core)
target_compile_definitions(evdynasim_c PRIVATE EVDS_BUILDING_DLL)
set_target_properties(evdynasim_c PROPERTIES CXX_VISIBILITY_PRESET hidden PREFIX "")
evds_warnings(evdynasim_c)

# ---- command line tool -------------------------------------------------------
add_executable(evdynasim_cli apps/main.cpp)
target_link_libraries(evdynasim_cli PRIVATE evdynasim_core)
evds_warnings(evdynasim_cli)

# ---- tests -------------------------------------------------------------------
if(EVDS_BUILD_TESTS)
  enable_testing()
  if(EVDS_USE_SYSTEM_GTEST)
    find_package(GTest REQUIRED)
  else()
    include(FetchContent)
    FetchContent_Declare(googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)  # match the MSVC runtime of our targets
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  add_executable(evdynasim_tests
    tests/test_config.cpp
    tests/test_physics.cpp
    tests/test_battery.cpp
    tests/test_simulator.cpp
  )
  target_link_libraries(evdynasim_tests PRIVATE evdynasim_core GTest::gtest_main)
  # Tests read configs/ from the repository root.
  target_compile_definitions(evdynasim_tests PRIVATE EVDS_REPO_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
  evds_warnings(evdynasim_tests)
  add_test(NAME evdynasim_tests COMMAND evdynasim_tests)
endif()

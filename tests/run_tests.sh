#!/bin/sh
# Build the library against the simulator (tests/sim/vex.h) and run the tests.
# Needs a C++17 compiler (g++ or clang++). Usage: tests/run_tests.sh
set -e
cd "$(dirname "$0")/.."
CXX="${CXX:-g++}"
mkdir -p build
$CXX -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter -pthread \
    -I include -I tests/sim \
    src/*.cpp tests/simulation_test.cpp \
    -o build/simulation_test
./build/simulation_test

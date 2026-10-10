#!/bin/sh
# Build the library against the simulator (tests/sim/vex.h) and run the tests, twice:
#   1. with the settings from include/simpleV5LibConfig.h (no tracking wheels)
#   2. with tracking wheels turned on, using an edited copy of the config
# Needs a C++17 compiler (g++ or clang++; on Windows, a MinGW g++ such as w64devkit, run from Git Bash:
# CXX=/d/Tools/w64devkit/bin/g++ tests/run_tests.sh). Usage: tests/run_tests.sh
set -e
cd "$(dirname "$0")/.."
CXX="${CXX:-g++}"
mkdir -p build

build_and_run() { # $1 = folder with the library headers, $2 = program name
    $CXX -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter -pthread \
        -I "$1" -I tests/sim \
        src/*.cpp tests/simulation_test.cpp \
        -o "build/$2"
    "./build/$2"
}

echo "=== Tests without tracking wheels"
build_and_run include simulation_test

echo "=== Tests with tracking wheels (forward wheel 3 in right of center, sideways wheel 4 in behind)"
mkdir -p build/tracking_include
cp include/*.h build/tracking_include/
CONFIG=build/tracking_include/simpleV5LibConfig.h
sed -i \
    -e 's/TRACKING_FORWARD_PORT = -1;/TRACKING_FORWARD_PORT = PORT8;/' \
    -e 's/TRACKING_FORWARD_OFFSET = 0;/TRACKING_FORWARD_OFFSET = 3.0;/' \
    -e 's/TRACKING_SIDEWAYS_PORT = -1;/TRACKING_SIDEWAYS_PORT = PORT9;/' \
    -e 's/TRACKING_SIDEWAYS_OFFSET = 0;/TRACKING_SIDEWAYS_OFFSET = -4.0;/' \
    "$CONFIG"
for setting in "PORT = PORT8" "FORWARD_OFFSET = 3.0" "PORT = PORT9" "SIDEWAYS_OFFSET = -4.0"; do
    grep -q "$setting" "$CONFIG" || { echo "Could not turn on tracking wheels ($setting) in $CONFIG"; exit 1; }
done
build_and_run build/tracking_include simulation_test_tracking

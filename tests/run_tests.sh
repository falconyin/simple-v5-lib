#!/bin/sh
# Build the library against the simulator (tests/sim/vex.h) and run the tests, twice:
#   1. with the settings from include/simpleV5LibConfig.h (no tracking wheels, no distance sensors)
#   2. with tracking wheels and distance sensors turned on, using an edited copy of the config
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

echo "=== Tests with tracking wheels (forward wheel 3 in right of center, sideways wheel 4 in behind) and distance sensors"
mkdir -p build/tracking_include
cp include/*.h build/tracking_include/
CONFIG=build/tracking_include/simpleV5LibConfig.h
sed -i \
    -e 's/TRACKING_FORWARD_PORT = -1;/TRACKING_FORWARD_PORT = PORT8;/' \
    -e 's/TRACKING_FORWARD_OFFSET = 0;/TRACKING_FORWARD_OFFSET = 3.0;/' \
    -e 's/TRACKING_SIDEWAYS_PORT = -1;/TRACKING_SIDEWAYS_PORT = PORT9;/' \
    -e 's/TRACKING_SIDEWAYS_OFFSET = 0;/TRACKING_SIDEWAYS_OFFSET = -4.0;/' \
    -e 's/DISTANCE_FRONT_PORT = -1;/DISTANCE_FRONT_PORT = PORT10;/' \
    -e 's/DISTANCE_FRONT_AHEAD = 0;/DISTANCE_FRONT_AHEAD = 6.0;/' \
    -e 's/DISTANCE_FRONT_RIGHT = 0;/DISTANCE_FRONT_RIGHT = 2.0;/' \
    -e 's/DISTANCE_BACK_PORT = -1;/DISTANCE_BACK_PORT = PORT11;/' \
    -e 's/DISTANCE_BACK_AHEAD = 0;/DISTANCE_BACK_AHEAD = -7.0;/' \
    -e 's/DISTANCE_BACK_RIGHT = 0;/DISTANCE_BACK_RIGHT = -1.5;/' \
    -e 's/DISTANCE_LEFT_PORT = -1;/DISTANCE_LEFT_PORT = PORT12;/' \
    -e 's/DISTANCE_LEFT_AHEAD = 0;/DISTANCE_LEFT_AHEAD = 3.0;/' \
    -e 's/DISTANCE_LEFT_RIGHT = 0;/DISTANCE_LEFT_RIGHT = -6.5;/' \
    -e 's/DISTANCE_RIGHT_PORT = -1;/DISTANCE_RIGHT_PORT = PORT13;/' \
    -e 's/DISTANCE_RIGHT_AHEAD = 0;/DISTANCE_RIGHT_AHEAD = -2.0;/' \
    -e 's/DISTANCE_RIGHT_RIGHT = 0;/DISTANCE_RIGHT_RIGHT = 6.5;/' \
    "$CONFIG"
for setting in "PORT = PORT8" "FORWARD_OFFSET = 3.0" "PORT = PORT9" "SIDEWAYS_OFFSET = -4.0" \
        "FRONT_PORT = PORT10" "FRONT_AHEAD = 6.0" "FRONT_RIGHT = 2.0" \
        "BACK_PORT = PORT11" "BACK_AHEAD = -7.0" "BACK_RIGHT = -1.5" \
        "LEFT_PORT = PORT12" "LEFT_AHEAD = 3.0" "LEFT_RIGHT = -6.5" \
        "RIGHT_PORT = PORT13" "RIGHT_AHEAD = -2.0" "RIGHT_RIGHT = 6.5"; do
    grep -q "$setting" "$CONFIG" || { echo "Could not turn on tracking wheels and distance sensors ($setting) in $CONFIG"; exit 1; }
done
build_and_run build/tracking_include simulation_test_tracking

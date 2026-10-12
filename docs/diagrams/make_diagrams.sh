#!/bin/sh
# Draw the diagrams in docs/images (see diagrams.cpp) and print the numbers the guides in docs
# quote. The path diagrams come from the simulator, so this needs the same C++17 compiler as
# tests/run_tests.sh. Usage: docs/diagrams/make_diagrams.sh
set -e
cd "$(dirname "$0")/../.."
CXX="${CXX:-g++}"
mkdir -p build docs/images
$CXX -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter -pthread \
    -I include -I tests/sim \
    src/*.cpp docs/diagrams/diagrams.cpp \
    -o build/diagrams
./build/diagrams docs/images

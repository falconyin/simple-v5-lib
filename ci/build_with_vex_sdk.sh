#!/bin/bash
# Compile the library and every example for the V5 brain against the official VEXcode V5 SDK,
# to catch mistakes that the simulator in tests/ can't (wrong VEX API names, C++11 problems, ...).
# It only compiles, it does not link a program.
#
# Needs: curl, unzip, python3 and clang (any recent version, it targets the V5's ARM CPU).
# Usage: ci/build_with_vex_sdk.sh             (uses the newest SDK)
#        VEX_SDK_VERSION=V5_20240802_15_00_00 ci/build_with_vex_sdk.sh
set -euo pipefail
cd "$(dirname "$0")/.."

CDN="https://content.vexrobotics.com/vexos/public/V5/vscode/sdk/cpp"
SDK_DIR="build/vex-sdk"
CXX="${CXX:-clang++}"

# ---------- Download the SDK ----------
mkdir -p "$SDK_DIR"
curl -fsSL "$CDN/manifest.json" -o "$SDK_DIR/manifest.json"
VERSION="${VEX_SDK_VERSION:-$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["latest"])' "$SDK_DIR/manifest.json")}"
echo "VEX SDK version: $VERSION"
if [ ! -d "$SDK_DIR/$VERSION" ]; then
    curl -fsSL "$CDN/$VERSION.zip" -o "$SDK_DIR/sdk.zip"
    unzip -q -o "$SDK_DIR/sdk.zip" -d "$SDK_DIR"
    rm "$SDK_DIR/sdk.zip"
fi
V5="$SDK_DIR/$VERSION/vexv5"
echo "SDK layout:"
find "$V5" -maxdepth 4 -type d | sed 's/^/  /' | head -40

# ---------- Find the header folders ----------
# VEX API headers (v5_vcs.h)
VEX_INC="$(dirname "$(find "$V5" -name v5_vcs.h | head -1)")"
# C library headers (newlib: stdio.h next to sys/)
C_INC="$(dirname "$(dirname "$(find "$V5" -path '*/include/sys/types.h' | grep -v '/c++/' | head -1)")")"
# C++ library headers (the folder with <vector>), and its ARM-specific bits folder
CXX_INC="$(dirname "$(find "$V5" -path '*/c++/*' -name vector -type f | head -1)")"
CXX_ARM_INC="$(dirname "$(dirname "$(find "$CXX_INC" -path '*thumb*' -name c++config.h | head -1)")")"
echo "VEX headers: $VEX_INC"
echo "C headers:   $C_INC"
echo "C++ headers: $CXX_INC"
echo "C++ ARM:     $CXX_ARM_INC"

# ---------- Compile ----------
# Same flags as a VEXcode V5 project (vex/mkenv.mk)
FLAGS=(
    -target thumbv7-none-eabi -march=armv7-a -mfpu=neon -mfloat-abi=softfp
    -fshort-enums -Wno-unknown-attributes
    -U__INT32_TYPE__ -U__UINT32_TYPE__ -D__INT32_TYPE__=long "-D__UINT32_TYPE__=unsigned long"
    -Os -Wall -Werror=return-type -fno-rtti -fno-threadsafe-statics -fno-exceptions
    -std=gnu++11 -ffunction-sections -fdata-sections -DVexV5
    -nostdinc++ -isystem "$CXX_INC" -isystem "$CXX_ARM_INC" -isystem "$C_INC" -isystem "$VEX_INC"
    -I include -I ci
)

mkdir -p build/obj
status=0
for file in src/*.cpp examples/*/main.cpp; do
    out="build/obj/$(echo "$file" | tr '/' '_').o"
    if "$CXX" "${FLAGS[@]}" -c "$file" -o "$out"; then
        echo "OK      $file"
    else
        echo "FAILED  $file"
        status=1
    fi
done
exit $status

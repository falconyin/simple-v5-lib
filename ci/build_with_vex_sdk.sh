#!/bin/bash
# Build the library and every example for the V5 brain against the official VEXcode V5 SDK,
# to catch mistakes that the simulator in tests/ can't (wrong VEX API names, C++11 problems,
# functions that don't exist on the brain, ...). Each example is linked into a full program
# (build/<example>.bin), the same file VEXcode downloads to the brain.
#
# Needs: curl, unzip, python3, clang (any recent version, it targets the V5's ARM CPU)
# and the ARM linker (arm-none-eabi-ld / arm-none-eabi-objcopy, Ubuntu: binutils-arm-none-eabi).
# Usage: ci/build_with_vex_sdk.sh             (uses the newest SDK)
#        VEX_SDK_VERSION=V5_20240802_15_00_00 ci/build_with_vex_sdk.sh
#
# With the VEX VS Code extension installed, its SDK and tools can be used instead, without
# downloading anything (then curl, unzip and python3 aren't needed): see ci/build_local_windows.sh,
# or set VEX_SDK_HOME to the folder holding the V5_... SDK folders, plus CXX, LD and OBJCOPY.
set -euo pipefail
cd "$(dirname "$0")/.."

CDN="https://content.vexrobotics.com/vexos/public/V5/vscode/sdk/cpp"
CXX="${CXX:-clang++}"

if [ -n "${VEX_SDK_HOME:-}" ]; then
    # ---------- Use an SDK that is already installed ----------
    SDK_DIR="$VEX_SDK_HOME"
    VERSION="${VEX_SDK_VERSION:-$(ls "$SDK_DIR" | grep '^V5_' | sort | tail -1)}"
    if [ -z "$VERSION" ] || [ ! -d "$SDK_DIR/$VERSION" ]; then
        echo "No V5 SDK '$VERSION' in $SDK_DIR" >&2
        exit 1
    fi
    echo "VEX SDK version: $VERSION (installed)"
else
    # ---------- Download the SDK ----------
    SDK_DIR="build/vex-sdk"
    mkdir -p "$SDK_DIR"
    curl -fsSL "$CDN/manifest.json" -o "$SDK_DIR/manifest.json"
    VERSION="${VEX_SDK_VERSION:-$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["latest"])' "$SDK_DIR/manifest.json")}"
    echo "VEX SDK version: $VERSION"
    if [ ! -d "$SDK_DIR/$VERSION" ]; then
        curl -fsSL "$CDN/$VERSION.zip" -o "$SDK_DIR/sdk.zip"
        unzip -q -o "$SDK_DIR/sdk.zip" -d "$SDK_DIR"
        rm "$SDK_DIR/sdk.zip"
    fi
fi
V5="$SDK_DIR/$VERSION/vexv5"
echo "SDK layout:"
find "$V5" -maxdepth 4 -type d | sed 's/^/  /' | head -40
echo "SDK files next to the headers and libraries:"
ls -la "$V5" "$V5/gcc/libs" 2>/dev/null | sed 's/^/  /'

# ---------- Find the header folders ----------
# VEX API headers (v5_vcs.h)
VEX_INC="$(dirname "$(find "$V5" -name v5_vcs.h | head -1)")"
# C library headers (newlib: stdio.h next to sys/)
C_INC="$(dirname "$(dirname "$(find "$V5" -path '*/include/sys/types.h' | grep -v '/c++/' | head -1)")")"
# C++ library headers (the folder with <vector>), and its ARM-specific bits folder. Both names exist
# in several folders (debug/vector, .../thumb/fpu/bits) and find's order differs between systems,
# so pick the ones VEXcode uses: the shortest path, and armv7-ar/thumb (vex/mkenv.mk).
CXX_INC="$(dirname "$(find "$V5" -path '*/c++/*' -name vector -type f | awk '{ print length, $0 }' | sort -n | head -1 | cut -d' ' -f2-)")"
CXX_ARM_INC="$(dirname "$(dirname "$(find "$CXX_INC" -path '*/armv7-ar/thumb/bits/c++config.h' | head -1)")")"
echo "VEX headers: $VEX_INC"
echo "C headers:   $C_INC"
echo "C++ headers: $CXX_INC"
echo "C++ ARM:     $CXX_ARM_INC"
# Compiler headers (stddef.h, stdarg.h, ...) that the SDK ships for its own clang. Searched last
# (-idirafter), so a clang that has its own, like the one in CI, never uses them; the VEX extension's
# clang has none and needs them.
CLANG_INC="$(ls -d "$V5"/clang/*/include 2>/dev/null | sort -V | tail -1)"
echo "clang hdrs:  ${CLANG_INC:-(none)}"

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
if [ -n "$CLANG_INC" ]; then
    FLAGS+=(-idirafter "$CLANG_INC")
fi

mkdir -p build/obj
status=0
for file in src/*.cpp examples/*/main.cpp; do
    out="build/obj/$(echo "$file" | tr '/' '_').o"
    if "$CXX" "${FLAGS[@]}" -c "$file" -o "$out"; then
        echo "compiled  $file"
    else
        echo "FAILED    $file"
        status=1
    fi
done
if [ $status -ne 0 ]; then
    exit $status
fi

# ---------- Link ----------
# Same as a VEXcode V5 project (vex/mkrules.mk): the VEX linker script, the VEX runtime
# library (libv5rt.a) and the C/C++ libraries that come with the SDK.
LD="${LD:-arm-none-eabi-ld}"
OBJCOPY="${OBJCOPY:-arm-none-eabi-objcopy}"
LSCRIPT="$(find "$V5" -maxdepth 1 -name '*.ld' | head -1)"
LINK_FLAGS=(-nostdlib -T "$LSCRIPT" --gc-sections -L "$V5" -L "$V5/gcc/libs")
# stdlib_*.lib: symbols of the standard library that lives on the brain itself
for lib in "$V5"/stdlib_*.lib; do
    [ -e "$lib" ] && LINK_FLAGS+=(-R "$lib")
done
LIBS=(--start-group -lv5rt -lstdc++ -lc -lm -lgcc --end-group)
echo "Linker script: $LSCRIPT"

LIBRARY_OBJECTS=(build/obj/src_*.o)
for example in examples/*/; do
    name="$(basename "$example")"
    elf="build/$name.elf"
    if "$LD" "${LINK_FLAGS[@]}" -Map="build/$name.map" -o "$elf" \
            "build/obj/examples_${name}_main.cpp.o" "${LIBRARY_OBJECTS[@]}" "${LIBS[@]}" \
       && "$OBJCOPY" -O binary "$elf" "build/$name.bin"; then
        echo "linked    $name ($(stat -c %s "build/$name.bin") bytes)"
    else
        echo "FAILED    linking $name"
        status=1
    fi
done
exit $status
